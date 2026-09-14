# -*- coding: utf-8 -*-
"""热平衡工况：以**一段连续运行或启停循环**为独立版本。

数据从已保存的载荷工况版本冻结（``freeze``）：各级啮合功率损失、各轴/轴承
转速与径向载荷、持续时间被固化为热源；草稿/版本不再随源轮系或源载荷变化而
自动重算，只标记过期。随后填写环境温度、齿轮/轴承/油/箱体热容量、节点间热阻、
润滑油黏温曲线、散热面积与风扇启停阈值，``analyze_case`` 按时间步显式欧拉迭代
各节点温度，并把油温经黏温曲线反馈到啮合效率与搅油/轴承黏性损失。

规格（spec）结构（与冻结数据 frozen 分开保存：snapshot 列存 frozen）：
{
  "ambient":  默认环境温度 °C（各段可用 ambient 覆盖）,
  "initTemp": 初始温度 °C,
  "dt":       报告时间步 s（内部按稳定性自动细分）,
  "oil": {"grade": 油品牌号, "points": [[°C, mm²/s], ...],
          "rho": 密度 kg/m³, "volumeL": 油量 L, "cp": 比热容 J/(kg·K),
          "minVisc": 允许最低黏度 mm²/s, "maxVisc": 允许最高黏度,
          "sens": 效率黏温反馈灵敏度},
  "housing": {"capacity": kJ/K, "area": 散热面积 m²,
              "rNat": 自然对流热阻 K/kW（按 1 m² 标定，面积倍乘电导）,
              "rFan": 风扇开启热阻 K/kW},
  "fan": {"mode": "auto"/"on"/"off", "onTemp": 启动温度, "offTemp": 停机温度,
          "powerKw": 风扇功率 kW, "minCycleMin": 最短启停周期 min（频繁启停判定）},
  "limits": {"mesh": 齿轮节点温度限, "bearing": 轴承温度限, "oil": 油温限},
  "segments": [{"id", "name", "kind": "run"/"stop", "sourceId",
                "loadScale": 1, "speedScale": 1, "durationH", "ambient": null}],
  "meshes":  {meshId: {"capacity": kJ/K, "rOil": →油池热阻 K/kW, "churn": 搅油系数 kW}},
  "bearings":{bId:    {"capacity", "rOil", "f0", "f1", "dm": 节圆 mm}},
  "oilNode": {"capacity"?, "rHousing": 油池→箱体热阻 K/kW}
}

热网络（集中参数一维节点网络）：
  热源：啮合摩擦热 P·(1−η(ν)) → 啮合节点；搅油热 Cc·(ν/νref)·(n/nref) → 油池；
        轴承摩擦（SKF M0/M1 模型）→ 轴承节点。停机段全部热源为 0。
  导热：啮合节点/轴承节点 → 油池 → 箱体 → 环境；箱体→环境电导按散热面积倍乘，
        风扇开启时在 Rfan/Rnat 间切换（自动模式带回差）。
  效率反馈：η(ν)=η0·clip(1−sens·log10(ν/νref), 0.85, 1.02)（νref=该油品 40°C 黏度）。
"""
from __future__ import annotations

import bisect
import json
import math
import time

import loadcase

# ----------------------------- 常量与默认值 -----------------------------

OIL_GRADES = [
    # 牌号, 40°C 黏度, 100°C 黏度（mm²/s，典型值，折线段端点）
    {"grade": "ISO VG 68", "points": [[40, 68], [100, 8.8]]},
    {"grade": "ISO VG 100", "points": [[40, 100], [100, 11.4]]},
    {"grade": "ISO VG 150", "points": [[40, 150], [100, 14.5]]},
    {"grade": "ISO VG 220", "points": [[40, 220], [100, 18.7]]},
    {"grade": "ISO VG 320", "points": [[40, 320], [100, 24.0]]},
]

DEFAULTS = {
    "ambient": 25.0, "initTemp": 30.0, "dt": 30.0,
    "oil": {"rho": 870.0, "volumeL": 3.0, "cp": 1900.0,
            "minVisc": 10.0, "maxVisc": 500.0, "sens": 0.10},
    "housing": {"capacity": 30.0, "area": 0.8, "rNat": 15.0, "rFan": 3.0},
    "fan": {"mode": "auto", "onTemp": 55.0, "offTemp": 45.0,
            "powerKw": 0.25, "minCycleMin": 3.0},
    "limits": {"mesh": 90.0, "bearing": 85.0, "oil": 85.0},
    "mesh": {"capacity": 0.30, "rOil": 0.25, "churn": 0.02},
    "bearing": {"capacity": 0.12, "rOil": 0.40, "f0": 2.0, "f1": 0.0009},
    "oilNode": {"rHousing": 0.08},
    "convRateKpH": 2.0,       # 热平衡收敛：窗口内最大温升速率 K/h
    "maxSamples": 640,        # 返回曲线最多采样点数
}

OIL = "oil"
HOUSING = "housing"
AMBIENT = "ambient"


def _num(v, d=0.0):
    try:
        x = float(v)
        return x if math.isfinite(x) else float(d)
    except (TypeError, ValueError):
        return float(d)


def _r(v, n=3):
    return None if v is None else round(float(v), n)


def oil_by_grade(grade):
    for o in OIL_GRADES:
        if o["grade"] == grade:
            return {"grade": o["grade"], "points": [list(p) for p in o["points"]]}
    return None


# ----------------------------- 来源指纹（过期判定） -----------------------------

def fingerprint(st: dict) -> str:
    """与前端 lcFingerprint 同口径：齿轮 z/m/x/压力角/内齿标记 + 轴位。"""
    shafts = {s["id"]: s for s in st.get("shafts", [])}
    gears = {g["id"]: g for g in st.get("gears", [])}
    items = []
    for e in st.get("meshes", []):
        ga, gb = gears.get(e.get("gearA")), gears.get(e.get("gearB"))
        if not ga or not gb:
            items.append([e.get("id"), "broken"])
            continue
        sa, sb = shafts.get(ga.get("shaftId"), {}), shafts.get(gb.get("shaftId"), {})
        items.append([
            e.get("id"), ga.get("z"), ga.get("module"), ga.get("x", 0) or 0,
            ga.get("pressureAngle", 20) or 20, bool(ga.get("internal")),
            gb.get("z"), gb.get("module"), gb.get("x", 0) or 0,
            gb.get("pressureAngle", 20) or 20, bool(gb.get("internal")),
            sa.get("x", 0) or 0, sa.get("y", 0) or 0,
            sb.get("x", 0) or 0, sb.get("y", 0) or 0])
    items.sort(key=lambda a: str(a[0]))
    return json.dumps(items, ensure_ascii=False, sort_keys=True)


# ----------------------------- 冻结载荷来源 -----------------------------

def _bearing_dm(state: dict, sid: str) -> float:
    """轴承节圆默认值：轴上最小齿轮节径 ×1.25，限幅 20~200 mm。"""
    dms = []
    for g in state.get("gears", []):
        if g.get("shaftId") != sid:
            continue
        if isinstance(g.get("z"), int) and isinstance(g.get("module"), (int, float)):
            dms.append(1.25 * g["z"] * g["module"])
    dm = min(dms) if dms else 40.0
    return round(min(200.0, max(20.0, dm)), 1)


def freeze_sources(sources: list) -> dict:
    """sources: [{"key", "name", "state", "spec", "caseId"?, "version"?}]
    对每个来源跑载荷工况分析，冻结各级损失/转速/轴承载荷，并生成节点默认参数。"""
    frozen = {"sources": {}, "order": [], "meshNames": {}, "shaftNames": {}}
    node_defaults = {}
    mesh_seen = {}
    shaft_seen = {}
    for src in sources:
        st, sp = src.get("state", {}), src.get("spec", {})
        key = str(src.get("key"))
        try:
            res = loadcase.analyze_case(st, sp)
        except Exception as exc:  # noqa: BLE001
            frozen["sources"][key] = {"key": key, "name": src.get("name", key),
                                      "error": str(exc)}
            frozen["order"].append(key)
            continue
        mdata, sdata = {}, {}
        for mid, m in (res.get("meshes") or {}).items():
            mdata[mid] = {
                "name": m["name"], "lossKw": m.get("lossKw", 0.0),
                "powerKw": m.get("powerKw", 0.0), "eta": m.get("efficiency", 0.98),
                "churn": DEFAULTS["mesh"]["churn"],
                "rpmDriver": m.get("speedDriver", 0.0),
                "rpmDriven": m.get("speedDriven", 0.0),
                "internal": m.get("internal", False),
            }
            mesh_seen[mid] = m["name"]
            bid = "mesh:" + mid
            node_defaults.setdefault(bid, {
                "kind": "mesh", "name": m["name"],
                "capacity": DEFAULTS["mesh"]["capacity"],
                "rOil": DEFAULTS["mesh"]["rOil"],
                "churn": DEFAULTS["mesh"]["churn"]})
        for sid, s in (res.get("shafts") or {}).items():
            bs = []
            for b in s.get("bearings", []):
                # 载荷工况轴承 id 形如 b1:<sid> / b2:<sid>；热网络节点统一为 b:<sid>:<1|2>
                num = "".join(ch for ch in str(b["id"]).split(":", 1)[0]
                              if ch.isdigit()) or "1"
                bid = "b:%s:%s" % (sid, num[-1])
                bs.append({"id": bid, "r": b.get("r", 0.0), "Cr": b.get("Cr", 0.0)})
                node_defaults.setdefault(bid, {
                    "kind": "bearing",
                    "name": "%s · 轴承%s" % (s.get("name", sid), num),
                    "capacity": DEFAULTS["bearing"]["capacity"],
                    "rOil": DEFAULTS["bearing"]["rOil"],
                    "f0": DEFAULTS["bearing"]["f0"],
                    "f1": DEFAULTS["bearing"]["f1"],
                    "dm": _bearing_dm(st, sid)})
            sdata[sid] = {"name": s.get("name", sid), "rpm": s.get("rpm", 0.0),
                          "bearings": bs}
            shaft_seen[sid] = s.get("name", sid)
        frozen["sources"][key] = {
            "key": key, "name": src.get("name", key),
            "caseId": src.get("caseId"), "version": src.get("version"),
            "inline": bool(src.get("inline")),
            "fingerprint": fingerprint(st),
            "loadSpec": json.dumps(sp, ensure_ascii=False, sort_keys=True),
            "snapshot": st,
            "inputId": sp.get("inputId"), "inputRpm": (res.get("input") or {}).get("rpm"),
            "pInputKw": (res.get("input") or {}).get("powerKw", 0.0),
            "lossKw": res.get("lossKw", 0.0),
            "durationH": sp.get("duration", 0.0),
            "meshes": mdata, "shafts": sdata,
            "ok": res.get("ok", False), "issues": res.get("issues", []),
        }
        frozen["order"].append(key)
    frozen["meshNames"] = mesh_seen
    frozen["shaftNames"] = shaft_seen
    frozen["defaults"] = node_defaults
    return frozen


# ----------------------------- 规格归一化 -----------------------------

def _merge_spec(spec: dict) -> dict:
    """补齐缺省参数，不覆盖已录入值。"""
    s = {}
    s["ambient"] = _num(spec.get("ambient"), DEFAULTS["ambient"])
    s["initTemp"] = _num(spec.get("initTemp"), s["ambient"])
    s["dt"] = max(1.0, _num(spec.get("dt"), DEFAULTS["dt"]))

    oil_in = spec.get("oil", {}) or {}
    d = DEFAULTS["oil"]
    grade = oil_in.get("grade") or "ISO VG 220"
    preset = oil_by_grade(grade)
    pts = oil_in.get("points")
    if not (isinstance(pts, list) and len(pts) >= 2):
        pts = (preset or oil_by_grade("ISO VG 220"))["points"]
    pts = sorted(([_num(p[0]), max(0.1, _num(p[1]))] for p in pts
                  if isinstance(p, (list, tuple)) and len(p) >= 2),
                 key=lambda p: p[0])
    s["oil"] = {
        "grade": grade if preset else "自定义",
        "points": pts,
        "rho": _num(oil_in.get("rho"), d["rho"]),
        "volumeL": max(0.01, _num(oil_in.get("volumeL"), d["volumeL"])),
        "cp": _num(oil_in.get("cp"), d["cp"]),
        "minVisc": _num(oil_in.get("minVisc"), d["minVisc"]),
        "maxVisc": _num(oil_in.get("maxVisc"), d["maxVisc"]),
        "sens": _num(oil_in.get("sens"), d["sens"]),
    }

    h_in, fan_in = spec.get("housing", {}) or {}, spec.get("fan", {}) or {}
    s["housing"] = {k: _num(h_in.get(k), DEFAULTS["housing"][k])
                    for k in ("capacity", "rNat", "rFan")}
    s["housing"]["area"] = max(0.01, _num(h_in.get("area"), DEFAULTS["housing"]["area"]))
    s["fan"] = {
        "mode": fan_in.get("mode") if fan_in.get("mode") in ("auto", "on", "off")
        else "auto",
        "onTemp": _num(fan_in.get("onTemp"), DEFAULTS["fan"]["onTemp"]),
        "offTemp": _num(fan_in.get("offTemp"), DEFAULTS["fan"]["offTemp"]),
        "powerKw": max(0.0, _num(fan_in.get("powerKw"), DEFAULTS["fan"]["powerKw"])),
        "minCycleMin": max(0.0, _num(fan_in.get("minCycleMin"),
                                     DEFAULTS["fan"]["minCycleMin"])),
    }
    lim_in = spec.get("limits", {}) or {}
    s["limits"] = {k: _num(lim_in.get(k), DEFAULTS["limits"][k])
                   for k in ("mesh", "bearing", "oil")}

    # 时段（过滤掉无来源/零时长的运行段，停机段保留）
    segs = []
    for i, seg in enumerate(spec.get("segments", []) or []):
        kind = seg.get("kind", "run")
        dur = max(0.0, _num(seg.get("durationH"), 0.0))
        segs.append({
            "id": str(seg.get("id") or "seg%d" % (i + 1)),
            "name": str(seg.get("name") or ("运行段 %d" % (i + 1))),
            "kind": "stop" if kind == "stop" else "run",
            "sourceId": str(seg.get("sourceId") or ""),
            "loadScale": max(0.0, _num(seg.get("loadScale"), 1.0)),
            "speedScale": max(0.0, _num(seg.get("speedScale"), 1.0)),
            "durationH": dur,
            "ambient": _num(seg["ambient"], s["ambient"])
            if isinstance(seg.get("ambient"), (int, float)) else None,
        })
    s["segments"] = segs

    mh, bh = spec.get("meshes", {}) or {}, spec.get("bearings", {}) or {}
    # meshes 的键兼容 meshId 与网络节点 id（mesh:<id>）；统一为后者
    def _mesh_key(k):
        return k if str(k).startswith("mesh:") else "mesh:" + str(k)
    s["meshes"] = {_mesh_key(mid): {
        "capacity": max(1e-6, _num(v.get("capacity"), DEFAULTS["mesh"]["capacity"])),
        "rOil": max(1e-6, _num(v.get("rOil"), DEFAULTS["mesh"]["rOil"])),
        "churn": max(0.0, _num(v.get("churn"), DEFAULTS["mesh"]["churn"])),
    } for mid, v in mh.items() if isinstance(v, dict)}
    s["bearings"] = {bid: {
        "capacity": max(1e-6, _num(v.get("capacity"), DEFAULTS["bearing"]["capacity"])),
        "rOil": max(1e-6, _num(v.get("rOil"), DEFAULTS["bearing"]["rOil"])),
        "f0": _num(v.get("f0"), DEFAULTS["bearing"]["f0"]),
        "f1": max(0.0, _num(v.get("f1"), DEFAULTS["bearing"]["f1"])),
        "dm": max(1.0, _num(v.get("dm"), 40.0)),
    } for bid, v in bh.items() if isinstance(v, dict)}
    on = spec.get("oilNode", {}) or {}
    s["oilNode"] = {"rHousing": max(1e-6, _num(on.get("rHousing"),
                                               DEFAULTS["oilNode"]["rHousing"]))}
    return s


# ----------------------------- 黏温曲线与损耗模型 -----------------------------

def oil_visc(points, t: float) -> float:
    """折线段内对 log10(ν) 关于温度线性插值；越界钳到端点。"""
    if t <= points[0][0]:
        return points[0][1]
    if t >= points[-1][0]:
        return points[-1][1]
    for (t0, v0), (t1, v1) in zip(points, points[1:]):
        if t0 <= t <= t1:
            if t1 == t0:
                return v1
            f = (t - t0) / (t1 - t0)
            return 10.0 ** (math.log10(v0) + f * (math.log10(v1) - math.log10(v0)))
    return points[-1][1]


def _bearing_loss_kw(n: float, visc: float, p_load: float, dm: float,
                     f0: float, f1: float) -> float:
    """SKF 摩擦力矩模型：M=M0+M1（Nmm），Q=M·ω/1e6 kW。
    M0 = f0·(ν·n)^(2/3)·dm³·1e-7（νn≥2000），低速段 160·f0·dm³·1e-7。"""
    n = abs(n)
    if n <= 1e-9:
        return 0.0
    vn = visc * n
    if vn >= 2000.0:
        m0 = f0 * (vn ** (2.0 / 3.0)) * dm ** 3 * 1e-7
    else:
        m0 = 160.0 * f0 * dm ** 3 * 1e-7
    m1 = f1 * max(0.0, p_load) * dm
    return (m0 + m1) * 2.0 * math.pi * n / 60.0 / 1e6


# ----------------------------- 热网络主分析 -----------------------------

def analyze_case(spec: dict, frozen: dict | None = None) -> dict:
    frozen = frozen or spec.get("frozen") or {}
    sources = frozen.get("sources", {}) or {}
    defaults = frozen.get("defaults", {}) or {}
    issues = []

    def add(sev, code, msg, **refs):
        issues.append({"severity": sev, "code": code, "message": msg, "refs": refs})

    s = _merge_spec(spec)
    oil = s["oil"]
    nu_ref = oil_visc(oil["points"], oil["points"][0][0])
    c_oil = oil["rho"] * oil["volumeL"] * 1e-3 * oil["cp"] / 1000.0  # kJ/K
    # ρ(kg/m³)·V(L)·1e-3 = kg；×cp(J/kg·K)/1000 = kJ/K

    # —— 节点：油池、箱体、冻结到的全部啮合节点与轴承节点 ——
    caps, kinds, names, limits = {}, {}, {}, {}
    caps[OIL] = c_oil
    caps[HOUSING] = s["housing"]["capacity"]
    kinds.update({OIL: "oil", HOUSING: "housing", AMBIENT: "ambient"})
    names.update({OIL: "油池", HOUSING: "箱体", AMBIENT: "环境"})
    limits[OIL] = s["limits"]["oil"]
    limits[HOUSING] = s["limits"]["oil"] + 10.0
    for nid, d in defaults.items():
        kinds[nid] = d["kind"]
        names[nid] = d["name"]
        limits[nid] = s["limits"]["mesh"] if d["kind"] == "mesh" else s["limits"]["bearing"]
        if d["kind"] == "mesh":
            caps[nid] = s["meshes"].get(nid, {}).get("capacity", d["capacity"])
        else:
            caps[nid] = s["bearings"].get(nid, {}).get("capacity", d["capacity"])

    node_ids = [n for n in caps]
    temp_nodes = node_ids + [AMBIENT]

    # —— 时段校验 ——
    run_segs = [g for g in s["segments"] if g["kind"] == "run"]
    if not s["segments"]:
        add("error", "NO_SEGMENT", "尚未编排任何运行/停机时段")
    for g in s["segments"]:
        if g["kind"] == "run":
            src = sources.get(g["sourceId"])
            if not src or src.get("error"):
                add("error", "BAD_SOURCE", "时段「%s」的冻结来源缺失或计算失败" % g["name"],
                    segId=g["id"])
            elif g["durationH"] <= 0:
                add("error", "ZERO_DURATION", "运行时段「%s」持续时间为 0" % g["name"],
                    segId=g["id"])
    if s["fan"]["mode"] == "auto" and s["fan"]["onTemp"] <= s["fan"]["offTemp"]:
        add("error", "FAN_MISCONFIG",
            "风扇启动温度 %.1f°C 必须高于停机温度 %.1f°C（需有回差）"
            % (s["fan"]["onTemp"], s["fan"]["offTemp"]))

    # —— 边（固定拓扑）：节点→油池、油池→箱体、箱体→环境 ——
    edges = []   # (key, a, b, rFuncName/params)
    for nid in node_ids:
        if nid in (OIL, HOUSING):
            continue
        if kinds[nid] == "mesh":
            r = s["meshes"].get(nid, {}).get("rOil",
                defaults.get(nid, {}).get("rOil", DEFAULTS["mesh"]["rOil"]))
        else:
            r = s["bearings"].get(nid, {}).get("rOil",
                defaults.get(nid, {}).get("rOil", DEFAULTS["bearing"]["rOil"]))
        edges.append((nid + ">oil", nid, OIL, r))
    edges.append(("oil>housing", OIL, HOUSING, s["oilNode"]["rHousing"]))
    # 箱体→环境热阻在迭代中按风扇状态与面积取值
    r_nat = s["housing"]["rNat"] / s["housing"]["area"]
    r_fan = s["housing"]["rFan"] / s["housing"]["area"]

    # —— 时间步（半隐式星形网络无条件稳定：叶节点解析消元，油池/箱体解 2×2）——
    leaf_edges = [(nid, r) for key, nid, bnode, r in edges
                  if nid not in (OIL, HOUSING)]
    r_oh = s["oilNode"]["rHousing"]
    # 为捕捉风扇切换与启停循环，时间步不大于 报告步/2 与 60 s
    total_h = sum(g["durationH"] for g in s["segments"])
    dt_report = max(1.0, s["dt"])
    dt = min(dt_report, 60.0,
             max(2.0, total_h * 3600.0 / 4000.0))
    sub = max(1, int(math.ceil(dt_report / dt)))
    dt = dt_report / sub
    n_steps = int(round(total_h * 3600.0 / dt))
    # 为保证步长整除，按总时长重定 dt
    if n_steps > 0:
        dt = total_h * 3600.0 / n_steps
        sub = max(1, int(round(dt_report / dt)))
        dt = dt_report / sub
        n_steps = int(round(total_h * 3600.0 / dt))

    bands, tcur = [], 0.0
    for g in s["segments"]:
        bands.append({"id": g["id"], "name": g["name"], "kind": g["kind"],
                      "t0": tcur, "t1": tcur + g["durationH"] * 3600.0})
        tcur += g["durationH"] * 3600.0

    def seg_at(t):
        for g in s["segments"]:
            if b_map[g["id"]][0] <= t < b_map[g["id"]][1] - 1e-9:
                return g
        return s["segments"][-1] if s["segments"] else None
    b_map = {g["id"]: (b["t0"], b["t1"]) for g, b in zip(s["segments"], bands)}

    def heat_inputs(seg, nu):
        """返回 {node: Q kW}、{meshId: eta}、总发热。"""
        q = {}
        eta_now = {}
        if not seg or seg["kind"] == "stop":
            return q, eta_now, 0.0
        src = sources.get(seg["sourceId"])
        if not src or src.get("error"):
            return q, eta_now, 0.0
        ls, ss = seg["loadScale"], seg["speedScale"]
        for mid, m in src.get("meshes", {}).items():
            nid = "mesh:" + mid
            if nid not in caps:
                continue
            kf = 1.0 + oil["sens"] * math.log10(max(nu, 1e-6) / nu_ref)
            kf = min(1.0, max(0.90, kf))   # 黏度低于参考值只使油膜变差，η 不超过名义值
            eta_eff = min(1.0, m["eta"] * kf)
            eta_now[mid] = eta_eff
            # 啮合摩擦热入齿轮节点
            q[nid] = q.get(nid, 0.0) + m["powerKw"] * ls * ss * (1.0 - eta_eff)
            # 搅油损失入油池：Cc·(ν/νref)·(n/nref)
            cc = s["meshes"].get(nid, {}).get("churn", m.get("churn",
                DEFAULTS["mesh"]["churn"]))
            nref = max(1e-9, abs(m.get("rpmDriver", 0.0)))
            q[OIL] = q.get(OIL, 0.0) + cc * (nu / nu_ref) * \
                (abs(m.get("rpmDriver", 0.0)) * ss / nref)
        for sid, sd in src.get("shafts", {}).items():
            n_rpm = sd.get("rpm", 0.0) * ss
            for b in sd.get("bearings", []):
                bid = b["id"]
                if bid not in caps:
                    continue
                cfg = s["bearings"].get(bid, {})
                qb = _bearing_loss_kw(n_rpm, nu, b.get("r", 0.0) * ls,
                                      cfg.get("dm", defaults.get(bid, {}).get("dm", 40.0)),
                                      cfg.get("f0", defaults.get(bid, {}).get(
                                          "f0", DEFAULTS["bearing"]["f0"])),
                                      cfg.get("f1", defaults.get(bid, {}).get(
                                          "f1", DEFAULTS["bearing"]["f1"])))
                q[bid] = q.get(bid, 0.0) + qb
        return q, eta_now, sum(q.values())

    # 输出缓冲（稀疏追加，末尾降密）
    times, viscs, fan_arr = [], [], []
    temps = {n: [] for n in node_ids}
    etas = {}                 # meshId -> [eta...]，与样本等长
    flows = {key: [] for key, *_ in edges}
    flows["housing>ambient"] = []
    qin = {n: [] for n in node_ids}

    over_spans = {n: [] for n in node_ids}
    visc_spans = []
    fan_switches = []          # (t, newState)
    fan_on_time = 0.0
    energy_gen_j = 0.0
    over_open = {}
    visc_open = None
    last_eta = {}

    def _push_span(spans, t0, t1):
        if t1 - t0 < dt * 2:
            return
        if spans and t0 - spans[-1][1] <= dt * 2.5:
            spans[-1][1] = t1
        elif len(spans) < 12:
            spans.append([t0, t1])

    def record_sample(tk, fan_state):
        """按当前 T 记录一个输出样本并更新越限/黏度事件。"""
        nonlocal visc_open
        nu = oil_visc(oil["points"], T[OIL])
        times.append(tk)
        for n in node_ids:
            temps[n].append(T[n])
        viscs.append(nu)
        fan_arr.append(fan_state)
        seg = seg_at(tk) if s["segments"] else None
        q, emap, _ = heat_inputs(seg, nu)
        for n in node_ids:
            qin[n].append(q.get(n, 0.0))
        for mid, ev in emap.items():
            last_eta[mid] = ev
        # 每级 eta 全长记录：该级不在热输入段时沿用最近值，首现前为 None
        all_mids = set(last_eta) | set(emap)
        for mid in all_mids:
            etas.setdefault(mid, [])
            etas[mid].append(emap.get(mid, last_eta.get(mid)))
        for key, a, bnode, r in edges:
            tb = T[AMBIENT] if bnode == AMBIENT else T[bnode]
            flows[key].append((T[a] - tb) / r)
        flows["housing>ambient"].append(
            (T[HOUSING] - T[AMBIENT]) / (r_fan if fan_state else r_nat))
        # 越限事件（用采样后的最新值）
        for n in node_ids:
            over = T[n] > limits[n] + 1e-9
            if over and n not in over_open:
                over_open[n] = tk
            elif not over and n in over_open:
                _push_span(over_spans[n], over_open.pop(n), tk)
        vo = nu < oil["minVisc"] - 1e-9 or nu > oil["maxVisc"] + 1e-9
        if vo and visc_open is None:
            visc_open = tk
        elif not vo and visc_open is not None:
            _push_span(visc_spans, visc_open, tk)
            visc_open = None

    # —— 时间推进 ——
    T = {n: s["initTemp"] for n in node_ids}
    T[AMBIENT] = s["ambient"]
    fan_on = 1 if s["fan"]["mode"] == "on" else 0
    if s["fan"]["mode"] == "auto":
        fan_on = 1 if T[HOUSING] >= s["fan"]["onTemp"] else 0

    record_sample(0.0, fan_on)
    for step in range(1, n_steps + 1):
        tk = step * dt
        seg = seg_at(tk) if s["segments"] else None
        amb = seg["ambient"] if (seg and seg["ambient"] is not None) else s["ambient"]
        T[AMBIENT] = amb
        nu = oil_visc(oil["points"], T[OIL])

        # 风扇回差（自动模式在箱体温度上判）
        if s["fan"]["mode"] == "auto":
            if not fan_on and T[HOUSING] >= s["fan"]["onTemp"]:
                fan_on = 1
                fan_switches.append((tk, 1))
            elif fan_on and T[HOUSING] <= s["fan"]["offTemp"]:
                fan_on = 0
                fan_switches.append((tk, 0))
        else:
            want = 1 if s["fan"]["mode"] == "on" else 0
            if want != fan_on:
                fan_on = want
                fan_switches.append((tk, want))

        q, _, qtot = heat_inputs(seg, nu)
        energy_gen_j += qtot * 1000.0 * dt
        if fan_on:
            fan_on_time += dt

        T_init = T.copy()
        # —— 半隐式星形网络（叶节点解析消元，油池/箱体解 2×2，无条件稳定）——
        # 叶节点（齿轮/轴承）只连油池，后向欧拉：
        #   Tx' = tx_n + a·To'，a=dt·Gx/(Cx+dt·Gx)，
        #   tx_n=(Cx·Tx+dt·qx)/(Cx+dt·Gx)
        # 流入油池：Σ Gx(Tx'−To') = Σ Gx·tx_n − α·To'，
        #   α=Σ Gx(1−a)=Σ Cx·Gx/(Cx+dt·Gx)
        g_oh = 1.0 / r_oh
        g_a = 1.0 / (r_fan if fan_on else r_nat)
        alpha = 0.0
        beta_o = 0.0
        leaf_aux = []
        for nid, r in leaf_edges:
            gx = 1.0 / r
            cx = caps[nid]
            a = dt * gx / (cx + dt * gx)
            tx_n = (cx * T_init[nid] + dt * q.get(nid, 0.0)) / (cx + dt * gx)
            alpha += gx * (1.0 - a)
            beta_o += gx * tx_n
            leaf_aux.append((nid, gx, cx, a))

        # 油池：Co(To'−To)/dt = qo + βo − α·To' + g_oh(Tc'−To')
        # 箱体：Ch(Tc'−Tc)/dt = qh + g_oh(To'−Tc') + g_a(Ta−Tc')
        co, ch = caps[OIL], caps[HOUSING]
        qo = q.get(OIL, 0.0)
        qh_fan = s["fan"]["powerKw"] * fan_on * 0.25   # 风扇耗功约 1/4 散入箱壁
        # A·To' + B·Tc' = E；C·To' + D·Tc' = F
        A = co / dt + alpha + g_oh
        B = -g_oh
        E = co * T[OIL] / dt + qo + beta_o
        C = -g_oh
        D = ch / dt + g_oh + g_a
        F = ch * T[HOUSING] / dt + qh_fan + g_a * T[AMBIENT]
        det = A * D - B * C
        if abs(det) < 1e-18:
            add("error", "UNSTABLE", "热网络矩阵奇异，请检查热阻/热容量参数")
            break
        to_new = (E * D - B * F) / det
        tc_new = (A * F - C * E) / det
        T[OIL] = to_new
        T[HOUSING] = tc_new
        for nid, gx, cx, a in leaf_aux:
            tx_n = (cx * T_init[nid] + dt * q.get(nid, 0.0)) / (cx + dt * gx)
            T[nid] = tx_n + a * to_new
            if not math.isfinite(T[nid]):
                add("error", "UNSTABLE", "温度迭代发散，请检查热阻/热容量参数")
                T[nid] = limits[nid]

        if step % sub == 0 or step == n_steps:
            record_sample(tk, fan_on)

    # 收尾未闭合事件
    t_end = total_h * 3600.0
    for n, t0 in list(over_open.items()):
        _push_span(over_spans[n], t0, t_end)
    if visc_open is not None:
        _push_span(visc_spans, visc_open, t_end)

    # —— 越限总时长与诊断 ——
    over_total = {}
    peak = {}
    for n, arr in temps.items():
        over_total[n] = sum(t1 - t0 for t0, t1 in over_spans[n])
        peak[n] = max(arr) if arr else s["initTemp"]
    for n in node_ids:
        if over_total.get(n, 0.0) > 0:
            kind_cn = {"mesh": "齿轮", "bearing": "轴承", "oil": "油池",
                       "housing": "箱体"}.get(kinds[n], "节点")
            span = over_spans[n][0]
            sev = "error" if kinds[n] in ("mesh", "bearing", "oil") else "warning"
            add(sev, "OVER_TEMP",
                "%s节点「%s」超过温度限 %.0f°C：累计 %.1f min，峰值 %.1f°C"
                % (kind_cn, names[n], limits[n], over_total[n] / 60.0, peak[n]),
                node=n, t0=span[0], t1=span[1])
    visc_total = sum(t1 - t0 for t0, t1 in visc_spans)
    if visc_total > 0:
        span = visc_spans[0]
        add("warning", "VISC_OUT",
            "油黏度越出允许区间 [%g, %g] mm²/s：累计 %.1f min（峰值油温 %.1f°C）"
            % (oil["minVisc"], oil["maxVisc"], visc_total / 60.0, peak.get(OIL, 0.0)),
            node=OIL, t0=span[0], t1=span[1])

    # —— 风扇频繁启停 ——
    cycles = len(fan_switches)
    short_cycles = 0
    cycle_spans = []
    prev_t = 0.0
    for t, stv in fan_switches:
        if t - prev_t < s["fan"]["minCycleMin"] * 60.0 and t > 0:
            short_cycles += 1
            if len(cycle_spans) < 12:
                cycle_spans.append([prev_t, t])
        prev_t = t
    if s["fan"]["mode"] == "auto" and short_cycles:
        add("warning", "FAN_CYCLING",
            "风扇频繁启停：%d 次切换短于 %.0f min（共 %d 次切换），应加大启停回差"
            % (short_cycles, s["fan"]["minCycleMin"], cycles),
            t0=cycle_spans[0][0] if cycle_spans else 0,
            t1=cycle_spans[0][1] if cycle_spans else 0)

    # —— 各运行段热平衡收敛 ——
    out_n = len(times)

    def _idx_at(t):
        """不超过 t 的最后一个采样下标。"""
        return max(0, bisect.bisect_right(times, t + 1e-6) - 1)

    balance_segs = []
    for g, band in zip(s["segments"], bands):
        if g["kind"] != "run":
            continue
        t0, t1 = band["t0"], band["t1"]
        win = min(t1 - t0, max(300.0, 0.2 * (t1 - t0))) if t1 - t0 >= 600 else t1 - t0
        i0, i1 = _idx_at(t1 - win), _idx_at(t1)
        rate = 0.0
        if i1 > i0:
            rate = max(abs((temps[n][i1] - temps[n][i0]) / (win / 3600.0))
                       for n in node_ids)
        switched = any(t0 <= t < t1 and (t1 - t) <= win for t, _ in fan_switches)
        converged = rate <= DEFAULTS["convRateKpH"] and not switched and win > 0
        balance_segs.append({"segId": g["id"], "name": g["name"],
                             "converged": converged, "maxRateKpH": _r(rate, 2),
                             "oilEnd": _r(temps[OIL][i1], 2),
                             "housingEnd": _r(temps[HOUSING][i1], 2)})
    if balance_segs and not balance_segs[-1]["converged"] and run_segs:
        b = balance_segs[-1]
        add("warning", "NOT_CONVERGED",
            "末段运行「%s」结束时热平衡未收敛（最大温升速率 %.1f K/h，油温 %.1f°C），"
            "应延长运行时间或加强散热" % (b["name"], b["maxRateKpH"], b["oilEnd"]),
            segId=b["segId"])

    # —— 采样降密 ——
    stride = max(1, int(math.ceil(out_n / DEFAULTS["maxSamples"])))
    if stride > 1:
        idx = list(range(0, out_n, stride))
        if idx[-1] != out_n - 1:
            idx.append(out_n - 1)
    else:
        idx = list(range(out_n))
    times_d = [round(times[i], 1) for i in idx]
    temps_d = {n: [round(temps[n][i], 3) for i in idx] for n in temps}
    viscs_d = [round(viscs[i], 4) for i in idx]
    fan_d = [fan_arr[i] for i in idx]
    flows_d = {k: [round(flows[k][i], 5) for i in idx] for k in flows}
    qin_d = {n: [round(qin[n][i], 5) for i in idx] for n in qin}
    etas_d = {mid: [round(etas[mid][i], 5) if i < len(etas[mid])
                    and isinstance(etas[mid][i], (int, float)) else None
                    for i in idx]
              for mid in etas}
    for mid in etas_d:
        last = None
        for k, v in enumerate(etas_d[mid]):
            if v is None:
                etas_d[mid][k] = last
            else:
                last = v

    fan_kwh = s["fan"]["powerKw"] * fan_on_time / 3600.0
    energy_kwh = energy_gen_j / 3.6e6 + fan_kwh
    node_info = []
    for n in node_ids:
        node_info.append({"id": n, "kind": kinds[n], "name": names[n],
                          "limit": _r(limits[n], 1), "peakTemp": _r(peak[n], 2),
                          "overTimeS": round(over_total.get(n, 0.0), 1),
                          "capacity": _r(caps[n], 4)})
    mesh_info = {mid.split(":", 1)[1]
                 if mid.startswith("mesh:") else mid:
                     {"name": names[mid], "peakTemp": _r(peak[mid], 2),
                      "overTimeS": round(over_total.get(mid, 0.0), 1)}
                 for mid in node_ids if kinds.get(mid) == "mesh"}

    return {
        "ok": not any(i["severity"] == "error" for i in issues),
        "issues": issues,
        "dt": dt_report, "dtInternal": round(dt, 3), "subSteps": sub,
        "nSamples": len(idx), "totalH": round(total_h, 4),
        "times": times_d, "timesH": [round(t / 3600.0, 5) for t in times_d],
        "bands": bands,
        "temps": temps_d, "visc": viscs_d, "eta": etas_d,
        "fan": fan_d, "flows": flows_d, "qin": qin_d,
        "fanInfo": {"cycles": cycles, "shortCycles": short_cycles,
                    "cycleSpans": [[round(a, 1), round(b, 1)] for a, b in cycle_spans],
                    "onTimeH": round(fan_on_time / 3600.0, 4),
                    "energyKwh": round(fan_kwh, 4)},
        "balance": balance_segs,
        "summary": {
            "peakTemp": _r(max(peak.values()), 2) if peak else None,
            "peakNode": max(peak, key=peak.get) if peak else None,
            "overTimeS": round(sum(over_total.values()), 1),
            "viscOverS": round(visc_total, 1),
            "energyKwh": round(energy_kwh, 3),
            "fanKwh": round(fan_kwh, 4),
            "converged": all(b["converged"] for b in balance_segs)
            if balance_segs else False,
        },
        "nodes": node_info,
        "meshInfo": mesh_info,
        "nodeNames": names,
        "oilCap": round(c_oil, 3),
    }


# ----------------------------- 方案搜索 -----------------------------

def search(spec: dict, frozen: dict | None, params: dict) -> dict:
    """锁定现有散热条件时只换油品；否则在油品 × 散热面积 × 风扇启停阈值栅格上
    枚举，按 超限时长 → 峰值温度 → 能耗 → 改动量 排序。"""
    frozen = frozen or spec.get("frozen") or {}
    deadline = time.time() + max(0.5, _num(params.get("timeBudget"), 6.0))
    limit = max(1, int(_num(params.get("limit"), 12)))

    base_s = _merge_spec(spec)
    cur = analyze_case(spec, frozen)

    def grid(lo, hi, step, dft):
        lo, hi, step = _num(lo, dft), _num(hi, dft), max(1e-6, _num(step, dft))
        if hi < lo:
            lo, hi = hi, lo
        vals = []
        v = lo
        while v <= hi + 1e-9:
            vals.append(round(v, 6))
            v += step
        if not vals:
            vals = [dft]
        return vals

    # 油品候选（与当前牌号）
    grades = [base_s["oil"]["grade"]]
    for g in params.get("oils", []) or []:
        if oil_by_grade(g) and g not in grades:
            grades.append(g)
    if params.get("includeCustomName"):
        grades.append("自定义")

    lock_cool = bool(params.get("lockCooling"))

    def with_current(vals, cur):
        """把当前值并入栅格并排序，保证候选列表始终包含“当前配置”可对照。"""
        out = sorted(set(vals + [round(float(cur), 6)]))
        return out

    if lock_cool:
        areas = [base_s["housing"]["area"]]
        on_temps = [base_s["fan"]["onTemp"]]
        off_temps = [base_s["fan"]["offTemp"]]
    else:
        areas = with_current(grid(params.get("areaMin"), params.get("areaMax"),
                     params.get("areaStep"), base_s["housing"]["area"]),
                     base_s["housing"]["area"])
        on_temps = with_current(grid(params.get("fanOnMin"), params.get("fanOnMax"),
                        params.get("fanOnStep"), base_s["fan"]["onTemp"]),
                        base_s["fan"]["onTemp"])
        off_temps = with_current(grid(params.get("fanOffMin"), params.get("fanOffMax"),
                         params.get("fanOffStep"), base_s["fan"]["offTemp"]),
                         base_s["fan"]["offTemp"])
    areas = areas[:9]
    on_temps = on_temps[:9]
    off_temps = off_temps[:9]

    results = []
    seen = set()
    truncated = False
    nodes_n = 0

    def evaluate(grade, area, ton, toff):
        nonlocal truncated
        if time.time() > deadline:
            truncated = True
            return
        key = (grade, round(area, 4), round(ton, 3), round(toff, 3))
        if key in seen or ton <= toff + 1e-9:
            return
        seen.add(key)
        preset = oil_by_grade(grade)
        cand_spec = _deep_merge_spec(spec)
        if preset:
            cand_spec.setdefault("oil", {})["grade"] = grade
            cand_spec["oil"]["points"] = [list(p) for p in preset["points"]]
        cand_spec.setdefault("housing", {})["area"] = area
        cand_spec.setdefault("fan", {})["onTemp"] = ton
        cand_spec["fan"]["offTemp"] = toff
        try:
            r = analyze_case(cand_spec, frozen)
        except Exception:  # noqa: BLE001
            return
        sm = r["summary"]
        change = (0.0 if grade == base_s["oil"]["grade"] else 1.0)
        change += abs(area - base_s["housing"]["area"]) / 0.1
        change += abs(ton - base_s["fan"]["onTemp"]) / 10.0
        change += abs(toff - base_s["fan"]["offTemp"]) / 10.0
        results.append({
            "oilGrade": grade, "area": _r(area, 3),
            "fanOn": _r(ton, 2), "fanOff": _r(toff, 2),
            "overTimeS": sm["overTimeS"], "peakTemp": sm["peakTemp"],
            "energyKwh": sm["energyKwh"], "change": _r(change, 3),
            "converged": sm["converged"],
            "fanCycles": r["fanInfo"]["cycles"],
            "viscOverS": sm["viscOverS"],
            "nIssues": len([i for i in r["issues"] if i["severity"] == "error"]),
            "patch": {"oil": {"grade": grade,
                              "points": [list(p) for p in preset["points"]]
                              if preset else None},
                      "housing": {"area": _r(area, 3)},
                      "fan": {"onTemp": _r(ton, 2), "offTemp": _r(toff, 2)}},
        })

    # 先评估当前配置，保证候选 #1 可对照
    evaluate(base_s["oil"]["grade"], base_s["housing"]["area"],
             base_s["fan"]["onTemp"], base_s["fan"]["offTemp"])
    for grade in grades:
        for area in areas:
            for ton in on_temps:
                for toff in off_temps:
                    nodes_n += 1
                    evaluate(grade, area, ton, toff)
                    if truncated:
                        break
                if truncated:
                    break
            if truncated:
                break
        if truncated:
            break

    results.sort(key=lambda c: (c["nIssues"], c["overTimeS"], c["peakTemp"],
                                c["energyKwh"], c["change"]))
    # 标记当前配置
    for c in results:
        c["current"] = (c["oilGrade"] == base_s["oil"]["grade"]
                        and abs(c["area"] - base_s["housing"]["area"]) < 1e-6
                        and abs(c["fanOn"] - base_s["fan"]["onTemp"]) < 1e-6
                        and abs(c["fanOff"] - base_s["fan"]["offTemp"]) < 1e-6)
    # 当前配置必须出现在候选中（对照用）：排到列表最前并标记
    current = next((c for c in results if c["current"]), None)
    others = [c for c in results if not c["current"]][:limit - 1]
    out = ([current] if current else []) + others
    return {"results": out, "nodes": nodes_n, "truncated": truncated,
            "base": {"overTimeS": cur["summary"]["overTimeS"],
                     "peakTemp": cur["summary"]["peakTemp"],
                     "energyKwh": cur["summary"]["energyKwh"],
                     "converged": cur["summary"]["converged"]}}


def _deep_merge_spec(spec: dict) -> dict:
    import copy
    sp = copy.deepcopy(spec)
    sp.setdefault("oil", {})
    sp.setdefault("housing", {})
    sp.setdefault("fan", {})
    return sp
