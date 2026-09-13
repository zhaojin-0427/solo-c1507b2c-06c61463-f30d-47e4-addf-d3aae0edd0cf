# -*- coding: utf-8 -*-
"""回程间隙工况：换向空程计算、闭环相容区间与间隙组合搜索。

工况（spec）以一次轮系换向为独立对象：
{
  "inputId":    输入轴 id,
  "observeId":  观察轴 id,
  "reversalDeg": 换向角（度，输入轴反向摆角幅度）,
  "meshes": { meshId: {
      "jn":        法向齿侧间隙 mm（实测值）,
      "jnLocked":  锁定实测间隙（搜索组合时该级不参变）,
      "grade":     间隙等级 id（未锁定时按等级给名义侧隙）,
      "dA":        中心距调整量 mm（搜索方案写回；草稿默认 0）,
      "xSum":      变位系数和（搜索方案写回；草稿默认 0）,
      "centerTol": 中心距公差 ±mm,
      "ecc":       偏心量 mm（径向跳动引起的侧隙变动）,
      "flank":     初始贴合齿面 "drive"（工作面）| "coast"（非工作面）
  } }
}

折算模型：
  圆周侧隙 jt = jn / cosα；从动轴自由摆角 φ = jt / r从动。
  观察轴空程贡献 = φ·|n观察/n从动|；输入轴换面角贡献 = φ/|n从动|。
  中心距公差与偏心量把 jn 展宽 ±2·(tol+e)·sinα（min 钳位到 0）。
  初始贴合工作面时换向需越过整个侧隙（系数 1）；已贴合非工作面时
  该级不换面（系数 0）。
分支/闭环：输入→观察轴的每条简单路径给出观察轴滞后区间
  [Φmin, Φmax]，各路径区间求交即相容区间；无交集时按路径对称差
  定位冲突啮合。
"""
from __future__ import annotations

import math
import time
from collections import defaultdict
from fractions import Fraction

import kinematics

PATH_CAP = 8           # 输入→观察轴简单路径条数上限
PATH_DEPTH = 12        # 单条路径最多经过的啮合数
SEARCH_NODE_CAP = 60_000

# 间隙等级：名义法向侧隙 = factor × 模数
GRADES = [
    {"id": "g0", "name": "消隙（零背隙）", "factor": 0.0},
    {"id": "g1", "name": "精密级 0.03m", "factor": 0.03},
    {"id": "g2", "name": "标准级 0.06m", "factor": 0.06},
    {"id": "g3", "name": "宽松级 0.10m", "factor": 0.10},
    {"id": "g4", "name": "大间隙级 0.16m", "factor": 0.16},
]
GRADE_MAP = {g["id"]: g for g in GRADES}


# ----------------------------- 基础工具 -----------------------------

def _num(v, d=0.0):
    return float(v) if isinstance(v, (int, float)) else float(d)


def _speed(speeds: dict, sid) -> Fraction | None:
    v = speeds.get(sid)
    if not v:
        return None
    try:
        return Fraction(v["s"])
    except (ValueError, KeyError, TypeError):
        return None


def _gname(g: dict) -> str:
    return g.get("name") or ("内齿圈 z%s" % g.get("z") if g.get("internal") else "齿轮 z%s" % g.get("z"))


def _r4(v):
    return None if v is None else round(v, 4)


def mesh_table(state: dict) -> dict:
    """meshId -> 两端齿轮/轴与齿形参数（跳过残缺啮合）。"""
    gears = {g["id"]: g for g in state.get("gears", [])}
    out = {}
    for e in state.get("meshes", []):
        ga, gb = gears.get(e.get("gearA")), gears.get(e.get("gearB"))
        if not ga or not gb:
            continue
        if not all(isinstance(g.get("z"), int) and g.get("z") > 0 for g in (ga, gb)):
            continue
        if not isinstance(ga.get("module"), (int, float)) or ga["module"] <= 0:
            continue
        out[e["id"]] = {
            "id": e["id"], "gearA": ga["id"], "gearB": gb["id"],
            "shaftA": ga.get("shaftId"), "shaftB": gb.get("shaftId"),
            "nameA": _gname(ga), "nameB": _gname(gb),
            "zA": ga["z"], "zB": gb["z"], "m": float(ga["module"]),
            "alpha": float(ga.get("pressureAngle") or 20.0),
            "internal": bool(ga.get("internal") or gb.get("internal")),
        }
    return out


def effective_jn(info: dict, ms: dict) -> float:
    """工况下一处啮合的名义法向侧隙：锁定实测值，否则由等级+中心距调整+变位和合成。"""
    if ms.get("jnLocked"):
        return max(0.0, _num(ms.get("jn"), 0.0))
    grade = GRADE_MAP.get(ms.get("grade"), GRADE_MAP["g2"])
    sin_a = math.sin(math.radians(info["alpha"]))
    jn = grade["factor"] * info["m"] \
        + 2.0 * (_num(ms.get("dA"), 0.0) + _num(ms.get("xSum"), 0.0) * info["m"]) * sin_a
    return max(0.0, jn)


# ----------------------------- 路径与折算系数 -----------------------------

def _prepare(state: dict, spec: dict):
    """运动学只依赖项目快照：转速、输入→观察轴路径、每级折算系数。"""
    an = kinematics.analyze(state)
    speeds = an.get("speeds", {})
    tbl = mesh_table(state)
    shafts = {s["id"] for s in state.get("shafts", [])}
    issues = []

    def add(sev, code, msg):
        issues.append({"severity": sev, "code": code, "message": msg})

    input_id = spec.get("inputId")
    observe_id = spec.get("observeId")
    if input_id not in shafts:
        add("error", "NO_INPUT", "回程间隙工况尚未指定有效的输入轴")
    if observe_id not in shafts:
        add("error", "NO_OBSERVE", "回程间隙工况尚未指定有效的观察轴")

    adj = defaultdict(list)
    for mid, info in tbl.items():
        adj[info["shaftA"]].append((info["shaftB"], mid))
        adj[info["shaftB"]].append((info["shaftA"], mid))

    # DFS 枚举输入→观察轴的简单路径（闭环会产生多条）
    raw_paths = []
    if input_id in shafts and observe_id in shafts and input_id != observe_id:
        stack = [(input_id, (input_id,), ())]
        while stack and len(raw_paths) < PATH_CAP:
            cur, sp, mp = stack.pop()
            if cur == observe_id and mp:
                raw_paths.append(mp)
                continue
            if len(mp) >= PATH_DEPTH:
                continue
            for nb, mid in adj.get(cur, ()):
                if nb in sp:
                    continue
                stack.append((nb, sp + (nb,), mp + (mid,)))

    n_obs = _speed(speeds, observe_id)
    if observe_id in shafts and n_obs is None:
        add("error", "OBSERVE_IDLE", "观察轴未连入动力链，无法折算空程")
    if n_obs == 0:
        add("error", "OBSERVE_FIXED", "观察轴转速为 0（固定构件），无法作为观察轴")

    paths = []
    mesh_union = set()
    for mp in raw_paths:
        stages = []
        prev = input_id
        ok = True
        for mid in mp:
            info = tbl[mid]
            if info["shaftA"] == prev:
                drv_shaft, z_d = info["shaftB"], info["zB"]
                name_d, name_r = info["nameA"], info["nameB"]
            else:
                drv_shaft, z_d = info["shaftA"], info["zA"]
                name_d, name_r = info["nameB"], info["nameA"]
            n_drv = _speed(speeds, drv_shaft)
            if n_drv is None:
                add("warning", "PATH_IDLE",
                    "路径经轴未连入动力链，该路径不参与：%s" % "、".join(
                        tbl[m]["nameA"] + "→" + tbl[m]["nameB"] for m in mp))
                ok = False
                break
            if n_drv == 0:
                add("warning", "PATH_FIXED",
                    "路径经过转速为 0 的构件（%s 一侧），该路径不参与" % name_r)
                ok = False
                break
            alpha = math.radians(info["alpha"])
            cos_a = math.cos(alpha)
            r = info["m"] * z_d / 2.0
            # 从动轴自由摆角（度/mm jn）= (1/cosα)/r·180/π
            k = 180.0 / (math.pi * cos_a * r)
            stages.append({
                "meshId": mid, "m": info["m"], "alpha": info["alpha"],
                "sinA": math.sin(alpha),
                "kObs": k * abs(float(n_obs / n_drv)) if n_obs else 0.0,
                "kIn": k / abs(float(n_drv)),
                "nDriver": float(n_drv),
                "driver": name_d, "driven": name_r,
                "internal": info["internal"],
            })
            mesh_union.add(mid)
            prev = drv_shaft
        if ok:
            paths.append({"meshes": list(mp), "stages": stages})

    if input_id in shafts and observe_id in shafts and input_id != observe_id \
            and not paths and not any(i["severity"] == "error" for i in issues):
        add("error", "NO_PATH", "输入轴到观察轴之间没有可用的啮合路径")

    mesh_info = {mid: tbl[mid] for mid in mesh_union}
    prep = {
        "paths": paths, "meshUnion": mesh_union, "meshInfo": mesh_info,
        "nObserve": None if not n_obs else float(n_obs),
    }
    return prep, issues


def _evaluate(prep: dict, spec: dict) -> dict:
    """按工况参数求各路径观察轴滞后区间、相容区间与各级换面角。"""
    ms_all = spec.get("meshes", {}) or {}
    path_res = []
    for p in prep["paths"]:
        lo = hi = 0.0
        cum_in = 0.0
        stages = []
        for st in p["stages"]:
            ms = ms_all.get(st["meshId"], {}) or {}
            jn = effective_jn(st, ms)
            band = 2.0 * (_num(ms.get("centerTol"), 0.05)
                          + _num(ms.get("ecc"), 0.0)) * st["sinA"]
            jn_min, jn_max = max(0.0, jn - band), jn + band
            factor = 0.0 if ms.get("flank") == "coast" else 1.0
            phi_obs = st["kObs"] * jn * factor
            phi_in = st["kIn"] * jn * factor
            start_deg = cum_in
            cum_in += phi_in
            lo += st["kObs"] * jn_min * factor
            hi += st["kObs"] * jn_max * factor
            stages.append({
                "meshId": st["meshId"], "driver": st["driver"], "driven": st["driven"],
                "internal": st["internal"],
                "nDriver": _r4(st["nDriver"]),
                "flank": ms.get("flank") if ms.get("flank") in ("drive", "coast") else "drive",
                "factor": factor,
                "jn": round(jn, 5), "jnMin": round(jn_min, 5), "jnMax": round(jn_max, 5),
                "phiObs": _r4(phi_obs),
                "phiObsMin": _r4(st["kObs"] * jn_min * factor),
                "phiObsMax": _r4(st["kObs"] * jn_max * factor),
                "phiIn": _r4(phi_in),
                "startDeg": _r4(start_deg), "switchDeg": _r4(cum_in),
            })
        path_res.append({"meshes": p["meshes"], "stages": stages,
                         "lostMin": lo, "lostMax": hi})

    # 相容区间：各路径 [Φmin, Φmax] 求交
    lo_all = max((p["lostMin"] for p in path_res), default=0.0)
    hi_all = min((p["lostMax"] for p in path_res), default=0.0)
    conflicts = []
    conflict_meshes = set()
    for i in range(len(path_res)):
        for j in range(i + 1, len(path_res)):
            a, b = path_res[i], path_res[j]
            if max(a["lostMin"], b["lostMin"]) > min(a["lostMax"], b["lostMax"]) + 1e-9:
                diff = sorted(set(a["meshes"]) ^ set(b["meshes"]))
                conflict_meshes.update(diff)
                conflicts.append({"pathA": i, "pathB": j, "meshes": diff})
    compatible = not conflicts and lo_all <= hi_all + 1e-9

    # 主路径：啮合数最少（其次区间最窄），侧栏与动画按它展开
    primary = 0
    if path_res:
        primary = min(range(len(path_res)),
                      key=lambda k: (len(path_res[k]["meshes"]), path_res[k]["lostMax"]))
    stages = path_res[primary]["stages"] if path_res else []
    total_switch = stages[-1]["switchDeg"] if stages else 0.0
    n_obs = prep.get("nObserve")

    return {
        "compatible": compatible,
        "lostMin": _r4(lo_all), "lostMax": _r4(hi_all),
        "lostMinIn": _r4(lo_all / abs(n_obs)) if n_obs else None,
        "lostMaxIn": _r4(hi_all / abs(n_obs)) if n_obs else None,
        "gap": _r4(lo_all - hi_all) if not compatible else 0.0,
        "conflicts": conflicts,
        "conflictMeshes": sorted(conflict_meshes),
        "paths": [{"index": k, "meshes": p["meshes"], "meshCount": len(p["meshes"]),
                   "lostMin": _r4(p["lostMin"]), "lostMax": _r4(p["lostMax"])}
                  for k, p in enumerate(path_res)],
        "primaryPath": primary,
        "stages": stages,
        "totalSwitchDeg": _r4(total_switch),
        "nObserve": _r4(n_obs),
    }


def analyze_case(state: dict, spec: dict) -> dict:
    prep, issues = _prepare(state, spec)
    res = _evaluate(prep, spec)
    ms_all = spec.get("meshes", {}) or {}
    off_path = sorted(mid for mid in ms_all if mid not in prep["meshUnion"])
    for mid in off_path:
        issues.append({"severity": "info", "code": "OFF_PATH",
                       "message": "啮合 %s 不在输入→观察路径上，未参与空程计算" % mid})
    res.update({
        "ok": not any(i["severity"] == "error" for i in issues),
        "issues": issues,
        "inputId": spec.get("inputId"),
        "observeId": spec.get("observeId"),
        "reversalDeg": _num(spec.get("reversalDeg"), 30.0),
        "grades": GRADES,
        "meshNames": {mid: "%s → %s" % (i["nameA"], i["nameB"])
                      for mid, i in prep["meshInfo"].items()},
    })
    return res


# ----------------------------- 间隙组合搜索 -----------------------------

def search(state: dict, spec: dict, params: dict) -> dict:
    """在未锁定实测间隙的啮合上枚举 间隙等级 × 中心距调整 × 变位和，
    按 最坏空程 → 冲突数 → 改动量 排序。"""
    da_max = abs(_num(params.get("daMax"), 0.3))
    da_steps = max(1, int(_num(params.get("daSteps"), 3)))
    x_min = _num(params.get("xMin"), -0.3)
    x_max = _num(params.get("xMax"), 0.5)
    x_step = abs(_num(params.get("xStep"), 0.1)) or 0.1
    grade_ids = [g for g in (params.get("grades") or [gr["id"] for gr in GRADES])
                 if g in GRADE_MAP] or ["g2"]
    limit = max(1, int(_num(params.get("limit"), 60)))
    deadline = time.time() + float(_num(params.get("timeBudget"), 8.0))

    prep, issues = _prepare(state, spec)
    if not prep["paths"]:
        return {"results": [], "totalMatched": 0, "truncated": False, "nodes": 0,
                    "note": "；".join(i["message"] for i in issues) or "没有可搜索的路径。"}

    ms_all = spec.get("meshes", {}) or {}
    mesh_ids = sorted(prep["meshUnion"])
    # 主路径折算系数（剪枝用）：kObs·factor 与 kObs·band·factor
    primary = min(range(len(prep["paths"])),
                  key=lambda k: len(prep["paths"][k]["meshes"]))
    coef = {}
    for st in prep["paths"][primary]["stages"]:
        ms = ms_all.get(st["meshId"], {}) or {}
        factor = 0.0 if ms.get("flank") == "coast" else 1.0
        band = 2.0 * (_num(ms.get("centerTol"), 0.05)
                      + _num(ms.get("ecc"), 0.0)) * st["sinA"]
        coef[st["meshId"]] = (st["kObs"] * factor, st["kObs"] * band * factor)

    das = [round(-da_max + da_max * i / da_steps, 4) for i in range(2 * da_steps + 1)]
    nxs = max(0, int(round((x_max - x_min) / x_step)))
    xss = [round(x_min + k * x_step, 4) for k in range(nxs + 1)]

    per_mesh = []
    for mid in mesh_ids:
        info = prep["meshInfo"][mid]
        ms = ms_all.get(mid, {}) or {}
        m, sin_a = info["m"], math.sin(math.radians(info["alpha"]))
        if ms.get("jnLocked"):
            per_mesh.append([{"grade": ms.get("grade", "g2"),
                              "dA": _num(ms.get("dA")), "xSum": _num(ms.get("xSum")),
                              "locked": True, "change": 0.0}])
            continue
        cur_f = GRADE_MAP.get(ms.get("grade"), GRADE_MAP["g2"])["factor"]
        da0, xs0 = _num(ms.get("dA")), _num(ms.get("xSum"))
        best_by_jn = {}
        for gid in grade_ids:
            f = GRADE_MAP[gid]["factor"]
            for da in das:
                for xs in xss:
                    jn = f * m + 2.0 * (da + xs * m) * sin_a
                    if jn < -1e-9:
                        continue
                    change = abs(da - da0) + abs(xs - xs0) * m + abs(f - cur_f) * m
                    key = round(max(0.0, jn), 4)
                    if key not in best_by_jn or change < best_by_jn[key]["change"]:
                        best_by_jn[key] = {"grade": gid, "dA": da, "xSum": xs,
                                           "change": round(change, 4)}
        opts = sorted(best_by_jn.values(),
                      key=lambda o: GRADE_MAP[o["grade"]]["factor"] * m
                      + 2.0 * (o["dA"] + o["xSum"] * m) * sin_a)
        per_mesh.append(opts)

    nodes = [0]
    truncated = [False]
    # 保留前 limit 个最优：仅当已集满 limit 且部分解不优于当前第 limit 名时剪枝
    cut_worst = [float("inf")]
    matched = [0]
    results = []
    seen = set()

    def stop():
        if nodes[0] > SEARCH_NODE_CAP or time.time() > deadline:
            truncated[0] = True
            return True
        return False

    def dfs(idx, chosen, partial_worst, partial_change):
        if stop():
            return
        if partial_worst >= cut_worst[0]:
            return
        if idx == len(mesh_ids):
            nodes[0] += 1
            trial = {mid: dict(ms_all.get(mid, {}) or {}) for mid in
                     set(list(ms_all) + mesh_ids)}
            sig = []
            for mid, opt in zip(mesh_ids, chosen):
                trial[mid]["grade"] = opt["grade"]
                trial[mid]["dA"] = opt["dA"]
                trial[mid]["xSum"] = opt["xSum"]
                sig.append((mid, opt["grade"], opt["dA"], opt["xSum"]))
            sig = tuple(sig)
            if sig in seen:
                return
            seen.add(sig)
            matched[0] += 1
            res = _evaluate(prep, {"meshes": trial})
            if res["compatible"]:
                worst = res["lostMax"] or 0.0
            else:
                worst = 1e9 + (res["gap"] or 0.0)
            results.append({
                "meshes": {mid: {"grade": o["grade"], "dA": o["dA"], "xSum": o["xSum"]}
                           for mid, o in zip(mesh_ids, chosen)},
                "worst": _r4(worst if worst < 1e9 else None),
                "lostMin": res["lostMin"], "lostMax": res["lostMax"],
                "compatible": res["compatible"],
                "conflicts": len(res["conflicts"]), "gap": res["gap"],
                "change": _r4(partial_change),
                "_w": worst,
            })
            results.sort(key=lambda r: (r["_w"], r["conflicts"], r["change"]))
            del results[limit:]
            if len(results) >= limit:
                cut_worst[0] = results[-1]["_w"]
            return
        mid = mesh_ids[idx]
        c, b = coef.get(mid, (0.0, 0.0))
        info = prep["meshInfo"][mid]
        sin_a = math.sin(math.radians(info["alpha"]))
        for opt in per_mesh[idx]:
            jn = max(0.0, GRADE_MAP[opt["grade"]]["factor"] * info["m"]
                     + 2.0 * (opt["dA"] + opt["xSum"] * info["m"]) * sin_a) \
                if not opt.get("locked") else effective_jn(info, {
                    "jnLocked": True, "jn": ms_all.get(mid, {}).get("jn")})
            dfs(idx + 1, chosen + [opt],
                partial_worst + c * jn + b,
                partial_change + opt["change"])
            if truncated[0]:
                return

    dfs(0, [], 0.0, 0.0)
    for r in results:
        r.pop("_w", None)
    return {
        "results": results,
        "totalMatched": matched[0],
        "truncated": truncated[0],
        "nodes": nodes[0],
        "meshNames": {mid: "%s → %s" % (i["nameA"], i["nameB"])
                      for mid, i in prep["meshInfo"].items()},
        "grades": GRADES,
        "note": None,
    }
