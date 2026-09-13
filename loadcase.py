# -*- coding: utf-8 -*-
"""载荷工况：一次轮系受载的独立版本 —— 功率/转矩传播、啮合力合成、
轴承反力与安全余量、分支比例校核与轴系布置搜索。

工况（spec）以一次轮系受载为独立对象：
{
  "inputId":      输入轴 id,
  "inputTorque":  输入转矩 N·m,
  "inputRpm":     输入转速 rpm,
  "duration":     持续时间 h,
  "meshes": { meshId: {"efficiency": 啮合效率 0~1, "loadFactor": 载荷系数 KA} },
  "branches": { shaftId: { meshId: 该分流啮合占轴端功率的比例 } },
  "shafts": { shaftId: {
      "length": 轴长 mm,
      "minGap": 任意两安装面/轴承的最小间距 mm,
      "adjMin": 搜索时每个未锁定位置可向当前位置左侧移动的范围 mm,
      "adjMax": 搜索时可向右移动的范围 mm,
      "faces":    [{"id": "f:<gearId>", "gear": gearId, "x": mm}],
      "bearings": [{"id": "b1:<sid>", "x": mm, "Cr": 径向额定载荷 N, "locked": bool},
                   {"id": "b2:<sid>", ...}],
      "locked": [被锁定的面/轴承 id]   # 也兼容单项 locked 字段
  } }
}

传播模型：
  输入功率 P = T·ω（ω=2πn/60，kW = N·m·rad/s/1000）。
  啮合按 BFS 深度从输入轴定向（动力方向）；分流轴按用户给定比例分配
  功率（默认等分），汇流轴把各路到达功率相加；啮合损失 η 作用在该级：
  P到从动 = P离主动·η。
  主动齿轮上的转矩 T = P级/ω主动，切向力 Ft = T/r（r 用米），
  计算用力 Ft* = KA·Ft；径向力 Fr = Ft*·tanα。
力的方向（复用快照轴位与啮合方向）：
  u 为主动轴中心指向从动轴中心的单位向量，t = u 转 +90°（节圆切向）。
  外啮合：从动受力 −Ft·t（切向）、−Fr·u（指向主动）；
          主动受力 +Ft·t、+Fr·u（径向背离啮合中心）；
  内啮合：切向同上，径向反向（从动 +Fr·u、主动 −Fr·u）。
  两体受力严格等大反向（牛顿第三定律）。
轴系：各安装面横向力作为轴上外载荷，两支点按刚体静力学求反力；
  Rb = −ΣFi(xi−xa)/(xb−xa)，Ra = −ΣFi−Rb；安全余量 = Cr/R，
  L10h = 1e6/(60n)·(Cr/R)^3（球轴承 p=3）。
"""
from __future__ import annotations

import math
import time
from collections import defaultdict, deque
from fractions import Fraction

import kinematics

SEARCH_NODE_CAP = 200_000
BRANCH_TOL = 0.01          # 分支比例闭合容差
DEFAULT_CR = 1000.0        # 轴承径向额定载荷默认 N
L10_P = 3.0                # 球轴承寿命指数


# ----------------------------- 基础工具 -----------------------------

def _num(v, d=0.0):
    return float(v) if isinstance(v, (int, float)) else float(d if d is not None else 0.0)


def _r(v, n=3):
    return None if v is None else round(float(v), n)


def _speed(speeds: dict, sid):
    v = speeds.get(sid)
    if not v:
        return None
    try:
        return Fraction(v["s"])
    except (ValueError, KeyError, TypeError):
        return None


def _gname(g: dict) -> str:
    return g.get("name") or ("内齿圈 z%s" % g.get("z") if g.get("internal")
                             else "齿轮 z%s" % g.get("z"))


def _sname(s: dict) -> str:
    return s.get("name") or ("轴 %s" % s.get("id"))


def mesh_table(state: dict) -> dict:
    """meshId -> 两端齿轮/轴、齿形参数与中心连线方向（跳过残缺啮合）。"""
    gears = {g["id"]: g for g in state.get("gears", [])}
    shafts = {s["id"]: s for s in state.get("shafts", [])}
    out = {}
    for e in state.get("meshes", []):
        ga, gb = gears.get(e.get("gearA")), gears.get(e.get("gearB"))
        if not ga or not gb:
            continue
        if not all(isinstance(g.get("z"), int) and g.get("z") > 0 for g in (ga, gb)):
            continue
        if not isinstance(ga.get("module"), (int, float)) or ga["module"] <= 0:
            continue
        sa, sb = ga.get("shaftId"), gb.get("shaftId")
        A, B = shafts.get(sa), shafts.get(sb)
        ux = uy = 0.0
        if A and B:
            dx, dy = _num(B.get("x")) - _num(A.get("x")), _num(B.get("y")) - _num(A.get("y"))
            d = math.hypot(dx, dy)
            if d > 1e-9:
                ux, uy = dx / d, dy / d
        out[e["id"]] = {
            "id": e["id"], "gearA": ga["id"], "gearB": gb["id"],
            "shaftA": sa, "shaftB": sb,
            "nameA": _gname(ga), "nameB": _gname(gb),
            "zA": ga["z"], "zB": gb["z"], "m": float(ga["module"]),
            "alpha": float(ga.get("pressureAngle") or 20.0),
            "internal": bool(ga.get("internal") or gb.get("internal")),
            "uAB": (ux, uy),
        }
    return out


def default_layouts(state: dict, length_hint: dict | None = None) -> dict:
    """为每根轴生成默认轴向布置：两端轴承、齿轮安装面均布。
    length_hint: {shaftId: 用户给定轴长}，默认几何随该长度生成。"""
    layouts = {}
    gears_by_shaft = defaultdict(list)
    for g in state.get("gears", []):
        if g.get("shaftId") is not None:
            gears_by_shaft[g["shaftId"]].append(g)
    for s in state.get("shafts", []):
        sid = s["id"]
        gs = sorted(gears_by_shaft.get(sid, []), key=lambda g: str(g["id"]))
        n = len(gs)
        auto_len = float(max(50.0, min(200.0, 20.0 * (n + 2))))
        length = float(length_hint.get(sid, auto_len)) if length_hint else auto_len
        length = max(1.0, length)
        if n == 0:
            faces = []
        elif n == 1:
            faces = [{"id": "f:%s" % gs[0]["id"], "gear": gs[0]["id"],
                      "x": round(length / 2, 2)}]
        else:
            x0, x1 = min(20.0, length * 0.25), max(length - 20.0, length * 0.75)
            faces = [{"id": "f:%s" % g["id"], "gear": g["id"],
                      "x": round(x0 + (x1 - x0) * k / (n - 1), 2)}
                     for k, g in enumerate(gs)]
        layouts[sid] = {
            "length": round(length, 2), "minGap": 3.0, "adjMin": 30.0, "adjMax": 30.0,
            "faces": faces,
            "bearings": [
                {"id": "b1:%s" % sid, "x": round(min(8.0, length * 0.15), 2),
                 "Cr": DEFAULT_CR, "locked": False},
                {"id": "b2:%s" % sid, "x": round(max(length - 8.0, length * 0.85), 2),
                 "Cr": DEFAULT_CR, "locked": False}],
            "locked": [],
        }
    return layouts


def _merge_layouts(spec: dict, state: dict) -> dict:
    """补齐缺失的轴布置/面/轴承，不覆盖已录入值。"""
    out = {}
    given = spec.get("shafts", {}) or {}
    hints = {}
    for sid, lay in given.items():
        if isinstance(lay, dict) and isinstance(lay.get("length"), (int, float)) \
                and lay["length"] > 0:
            hints[sid] = float(lay["length"])
    defaults = default_layouts(state, hints)
    for sid, dlay in defaults.items():
        lay = dict(given.get(sid, {})) if isinstance(given.get(sid), dict) else {}
        out[sid] = {
            "length": max(1.0, _num(lay.get("length"), dlay["length"])),
            "minGap": max(0.0, _num(lay.get("minGap"), dlay["minGap"])),
            "adjMin": max(0.0, _num(lay.get("adjMin"), dlay["adjMin"])),
            "adjMax": max(0.0, _num(lay.get("adjMax"), dlay["adjMax"])),
            "locked": list(lay.get("locked", []) or []),
            "faces": [], "bearings": [],
        }
        fdef = {f["id"]: f for f in dlay["faces"]}
        for f in lay.get("faces", []) or []:
            if f.get("id") in fdef:
                fdef[f["id"]] = {**fdef[f["id"]], **f}
        out[sid]["faces"] = [dict(fdef[fid]) for fid in
                             sorted(fdef, key=lambda x: fdef[x]["x"])]
        bdef = {b["id"]: b for b in dlay["bearings"]}
        for b in lay.get("bearings", []) or []:
            if b.get("id") in bdef:
                bdef[b["id"]] = {**bdef[b["id"]], **b}
        out[sid]["bearings"] = [dict(bdef[bid]) for bid in sorted(bdef)]
    return out


# ----------------------------- 动力定向 -----------------------------

def speed_ratios(state: dict, tbl: dict, input_id):
    """从工况实际选中的输入轴开始，沿普通啮合边 BFS 传播精确转速比
    （n轴/n输入；外啮合 −zA/zB，内啮合 +zA/zB）。不依赖全局 inputId，
    因此用户在任一连通分量中选输入轴都能正确传播。
    返回 {shaftId: Fraction}；与输入轴不同分量的轴不在表内。"""
    shafts = {s["id"] for s in state.get("shafts", [])}
    if input_id not in shafts:
        return {}
    adj = defaultdict(list)
    for info in tbl.values():
        sa, sb = info["shaftA"], info["shaftB"]
        if sa in shafts and sb in shafts:
            sign = 1 if info["internal"] else -1
            f = sign * Fraction(info["zA"], info["zB"])   # n_sb / n_sa
            adj[sa].append((sb, f))
            adj[sb].append((sa, Fraction(1, 1) / f))
    ratio = {input_id: Fraction(1, 1)}
    q = deque([input_id])
    while q:
        cur = q.popleft()
        for nb, f in adj.get(cur, ()):
            v = ratio[cur] * f
            if nb in ratio:
                continue
            ratio[nb] = v
            q.append(nb)
    return ratio


def _orient(state, tbl, ratio, input_id, issues):
    """按 BFS 深度把每条啮合边从输入端定向：返回 {meshId: (driverShaft, drivenShaft,
    driverGear, drivenGear, zdriven, depth)} 与每轴深度。ratio 为所选输入轴的转速比。"""
    shafts = {s["id"] for s in state.get("shafts", [])}
    adj = defaultdict(list)
    for mid, info in tbl.items():
        if info["shaftA"] in shafts and info["shaftB"] in shafts:
            adj[info["shaftA"]].append((info["shaftB"], mid))
            adj[info["shaftB"]].append((info["shaftA"], mid))

    depth = {}
    if input_id in shafts:
        depth[input_id] = 0
        q = deque([input_id])
        while q:
            cur = q.popleft()
            for nb, _mid in adj.get(cur, ()):
                if nb not in depth:
                    depth[nb] = depth[cur] + 1
                    q.append(nb)

    oriented = {}
    for mid, info in tbl.items():
        sa, sb = info["shaftA"], info["shaftB"]
        da, db = depth.get(sa), depth.get(sb)
        if da is None or db is None:
            issues.append({"severity": "info", "code": "OFF_PATH",
                           "message": "啮合 %s→%s 不在所选输入轴的动力链上，未参与受载计算"
                           % (info["nameA"], info["nameB"]),
                           "refs": {"mesh": mid}})
            continue
        if da < db:
            drv, dnn, gd, gn, zd = sa, sb, info["gearA"], info["gearB"], info["zB"]
        elif db < da:
            drv, dnn, gd, gn, zd = sb, sa, info["gearB"], info["gearA"], info["zA"]
        else:
            # 同深度（闭环/并联回边）：|转速比|大的一侧为主动（减速时小轮带大轮）
            va = abs(float(ratio.get(sa, 0))) if sa in ratio else 0.0
            vb = abs(float(ratio.get(sb, 0))) if sb in ratio else 0.0
            if va + 1e-12 < vb:
                drv, dnn, gd, gn, zd = sb, sa, info["gearB"], info["gearA"], info["zA"]
            else:
                drv, dnn, gd, gn, zd = sa, sb, info["gearA"], info["gearB"], info["zB"]
            issues.append({"severity": "info", "code": "SAME_LEVEL_EDGE",
                           "message": "啮合 %s→%s 两端距输入轴等深，按转速大小假定动力方向"
                           % (info["nameA"], info["nameB"]),
                           "refs": {"mesh": mid}})
        oriented[mid] = {"driver": drv, "driven": dnn,
                         "gearDriver": gd, "gearDriven": gn, "zDriven": zd,
                         "depth": max(da, db)}
    return oriented, depth


# ----------------------------- 轴承静力学 -----------------------------

def bearing_solve(face_loads, bearings, length, min_gap):
    """face_loads: [{id,x,fx,fy}]；bearings: [{id,x,Cr,locked}]。
    返回两支点反力与超限明细。支点重合（b−a≈0）时无法平衡。"""
    n_bad_range = 0
    viol = []
    for f in face_loads:
        if f["x"] < -1e-9 or f["x"] > length + 1e-9:
            n_bad_range += 1
            viol.append({"code": "FACE_OUT_OF_SHAFT", "face": f["id"],
                         "message": "安装面超出轴长（x=%s mm）" % _r(f["x"], 1)})
    for b in bearings:
        if b["x"] < -1e-9 or b["x"] > length + 1e-9:
            n_bad_range += 1
            viol.append({"code": "BEARING_OUT_OF_SHAFT", "bearing": b["id"],
                         "message": "轴承超出轴长（x=%s mm）" % _r(b["x"], 1)})
    items = [("face", f["id"], f["x"]) for f in face_loads] + \
            [("bearing", b["id"], b["x"]) for b in bearings]
    for i in range(len(items)):
        for j in range(i + 1, len(items)):
            if abs(items[i][2] - items[j][2]) < min_gap - 1e-9:
                viol.append({"code": "TOO_CLOSE",
                             items[i][0]: items[i][1], items[j][0]: items[j][1],
                             "message": "%s 与 %s 间距小于 %.1f mm"
                             % (items[i][1], items[j][1], min_gap)})

    reactions = [None, None]
    coincident = False
    outside = []
    if len(bearings) == 2:
        a, b = bearings[0]["x"], bearings[1]["x"]
        span = b - a
        if abs(span) < 1e-6:
            coincident = True
        else:
            sfx = sum(f["fx"] for f in face_loads)
            sfy = sum(f["fy"] for f in face_loads)
            rbx = -sum(f["fx"] * (f["x"] - a) for f in face_loads) / span
            rby = -sum(f["fy"] * (f["x"] - a) for f in face_loads) / span
            rax, ray = -sfx - rbx, -sfy - rby
            reactions = [
                {"id": bearings[0]["id"], "x": a, "Cr": bearings[0].get("Cr", DEFAULT_CR),
                 "rx": rax, "ry": ray, "r": math.hypot(rax, ray),
                 "locked": bool(bearings[0].get("locked"))},
                {"id": bearings[1]["id"], "x": b, "Cr": bearings[1].get("Cr", DEFAULT_CR),
                 "rx": rbx, "ry": rby, "r": math.hypot(rbx, rby),
                 "locked": bool(bearings[1].get("locked"))},
            ]
            lo, hi = min(a, b), max(a, b)
            for f in face_loads:
                if f["x"] < lo - 1e-9 or f["x"] > hi + 1e-9:
                    outside.append(f["id"])
    else:
        coincident = True
    for r in reactions:
        if r:
            cr = max(1e-9, _num(r.get("Cr"), DEFAULT_CR))
            r["util"] = r["r"] / cr
            r["margin"] = cr / r["r"] if r["r"] > 1e-9 else None
    n_gap = sum(1 for v in viol if v["code"] == "TOO_CLOSE")
    return {"reactions": reactions, "coincident": coincident,
            "outside": outside, "violations": viol,
            "nRange": n_bad_range, "nGap": n_gap}


# ----------------------------- 主分析 -----------------------------

def analyze_case(state: dict, spec: dict) -> dict:
    an = kinematics.analyze(state)   # 仅用于结构诊断，转速从所选输入轴另行传播
    tbl = mesh_table(state)
    shaft_objs = {s["id"]: s for s in state.get("shafts", [])}
    layouts = _merge_layouts(spec, state)
    issues = []

    def add(sev, code, msg, **refs):
        issues.append({"severity": sev, "code": code, "message": msg, "refs": refs})

    input_id = spec.get("inputId")
    T0 = _num(spec.get("inputTorque"), 0.0)
    rpm0 = _num(spec.get("inputRpm"), state.get("inputRpm", 60) or 60)
    duration = max(0.0, _num(spec.get("duration"), 1000.0))
    if input_id not in shaft_objs:
        add("error", "NO_INPUT", "载荷工况尚未指定有效的输入轴")
    if T0 <= 0:
        add("error", "BAD_TORQUE", "输入转矩必须为正数（N·m）")
    if rpm0 <= 0:
        add("error", "BAD_RPM", "输入转速必须为正数（rpm）")

    # 从工况实际选中的输入轴传播转速比（不依赖全局 inputId）
    ratio = speed_ratios(state, tbl, input_id)
    oriented, depth = _orient(state, tbl, ratio, input_id, issues)
    if input_id in shaft_objs and not oriented:
        add("error", "INPUT_IDLE", "输入轴没有任何可达啮合，无法传播载荷")

    # 下游/上游表
    downstream = defaultdict(list)   # shaft -> [(meshId, child)]
    incoming = defaultdict(list)     # shaft -> [meshId]
    for mid, o in oriented.items():
        downstream[o["driver"]].append((mid, o["driven"]))
        incoming[o["driven"]].append(mid)

    # 分支比例：所有“一轴带出多处啮合”的分流点
    branch_spec = spec.get("branches", {}) or {}
    branches_out = []
    ratios = {}                     # meshId -> fraction
    for sid, outs in downstream.items():
        if len(outs) <= 1:
            continue
        mids = [m for m, _ in outs]
        given = branch_spec.get(sid, {}) if isinstance(branch_spec.get(sid), dict) else {}
        vals = []
        for m in mids:
            gv = given.get(m)
            v = float(gv) if isinstance(gv, (int, float)) and gv >= 0 else None
            if v is None:
                v = 1.0 / len(mids)
            vals.append(max(0.0, v))
        s = sum(vals)
        for m, v in zip(mids, vals):
            ratios[m] = v / s if s > 1e-12 else 1.0 / len(mids)
        branches_out.append({"shaftId": sid, "shaftName": _sname(shaft_objs[sid]),
                             "sum": s, "closed": abs(s - 1.0) <= BRANCH_TOL,
                             "meshes": [{"meshId": m,
                                         "name": mesh_label(tbl[m], oriented[m]),
                                         "ratio": ratios[m], "given":
                                             float(given.get(m))
                                             if isinstance(given.get(m), (int, float))
                                             and given.get(m) >= 0 else None}
                                        for m in mids]})
        if abs(s - 1.0) > BRANCH_TOL:
            add("error", "BRANCH_NOT_CLOSED",
                "分支比例不闭合：轴「%s」的各分支比例之和为 %.3f（应为 1）"
                % (_sname(shaft_objs[sid]), s),
                shaft=sid, **{("m%d" % k): m for k, m in enumerate(mids)})
    for sid, outs in downstream.items():
        if len(outs) == 1:
            ratios[outs[0][0]] = 1.0

    # ---- 功率/转矩定点传播（转速比从所选输入轴 BFS 得到）----
    omega0 = 2.0 * math.pi * rpm0 / 60.0
    p_in_kw = T0 * omega0 / 1000.0 if T0 > 0 and rpm0 > 0 else 0.0
    n_ratio = ratio
    mesh_p_kw = {mid: 0.0 for mid in oriented}
    shaft_p = {sid: 0.0 for sid in shaft_objs}
    order = sorted((sid for sid in shaft_objs if sid in depth),
                   key=lambda x: (depth[x], str(x)))
    for _pass in range(max(2, len(order) + 1)):
        changed = 0.0
        for sid in order:
            p = p_in_kw if sid == input_id else 0.0
            for mid in incoming.get(sid, ()):
                info = tbl[mid]
                eta = _mesh_efficiency(spec, mid)
                p += mesh_p_kw[mid] * eta
            if abs(p - shaft_p[sid]) > 1e-9:
                changed = max(changed, abs(p - shaft_p[sid]))
            shaft_p[sid] = p
            for mid, _child in downstream.get(sid, ()):
                mesh_p_kw[mid] = p * ratios.get(mid, 1.0)
        if changed < 1e-9:
            break

    # ---- 啮合结果与安装面载荷 ----
    mesh_res = {}
    face_forces = defaultdict(list)   # gearId -> [(fx,fy)]（一只齿轮理论上一处啮合）
    for mid, o in oriented.items():
        info = tbl[mid]
        ka = _mesh_param(spec, mid, "loadFactor", 1.0)
        eta = _mesh_efficiency(spec, mid)
        sd, sn = o["driver"], o["driven"]
        nd = float(n_ratio.get(sd, Fraction(0))) * rpm0 if sd in n_ratio else 0.0
        nn = float(n_ratio.get(sn, Fraction(0))) * rpm0 if sn in n_ratio else 0.0
        wd = 2.0 * math.pi * abs(nd) / 60.0
        r_d = info["m"] * (info["zA"] if o["gearDriver"] == info["gearA"] else info["zB"]) / 2000.0
        p_kw = mesh_p_kw[mid]
        # 转矩与啮合力按输入轴传入本级的功率（p_kw）正常计算：η 只影响该级输出
        # 与下游（传播时已乘 η），即便 η=0（本级相当于制动/全部耗损），主动轮
        # 仍承受完整齿面力，两齿受力等大反向；powerOutKw/lossKw 体现全部损失。
        torque = p_kw * 1000.0 / wd if wd > 1e-9 and r_d > 0 else 0.0
        ft = torque / r_d if r_d > 0 else 0.0
        ft_star = ft * ka
        fr = ft_star * math.tan(math.radians(info["alpha"]))
        if info["shaftA"] == sd:
            ux, uy = info["uAB"]
        else:
            ux, uy = -info["uAB"][0], -info["uAB"][1]
        tx, ty = -uy, ux
        if info["internal"]:
            fr_drv_x, fr_drv_y = -fr * ux, -fr * uy
            fr_dn_x, fr_dn_y = fr * ux, fr * uy
        else:
            fr_drv_x, fr_drv_y = fr * ux, fr * uy
            fr_dn_x, fr_dn_y = -fr * ux, -fr * uy
        ft_drv_x, ft_drv_y = ft_star * tx, ft_star * ty
        ft_dn_x, ft_dn_y = -ft_star * tx, -ft_star * ty
        face_forces[o["gearDriver"]].append((ft_drv_x + fr_drv_x, ft_drv_y + fr_drv_y,
                                             ft_star, fr))
        face_forces[o["gearDriven"]].append((ft_dn_x + fr_dn_x, ft_dn_y + fr_dn_y,
                                             ft_star, fr))
        mesh_res[mid] = {
            "meshId": mid, "name": mesh_label(info, o),
            "driverShaft": sd, "drivenShaft": sn,
            "driver": info["nameA"] if o["gearDriver"] == info["gearA"] else info["nameB"],
            "driven": info["nameB"] if o["gearDriven"] == info["gearB"] else info["nameA"],
            "internal": info["internal"], "m": info["m"],
            "zDriver": info["zA"] if o["gearDriver"] == info["gearA"] else info["zB"],
            "zDriven": o["zDriven"], "alpha": info["alpha"],
            "efficiency": eta, "loadFactor": ka, "ratio": ratios.get(mid, 1.0),
            "speedDriver": _r(nd, 2), "speedDriven": _r(nn, 2),
            "powerKw": _r(p_kw, 4), "powerOutKw": _r(p_kw * eta, 4),
            "lossKw": _r(p_kw * (1 - eta), 4), "torqueNm": _r(torque, 3),
            "ft": _r(ft_star, 2), "fr": _r(fr, 2),
            "u": [_r(ux, 4), _r(uy, 4)], "t": [_r(tx, 4), _r(ty, 4)],
            "gearDriver": o["gearDriver"], "gearDriven": o["gearDriven"],
        }

    # ---- 轴系合成 ----
    gear_objs = {g["id"]: g for g in state.get("gears", [])}
    # 每根轴上承载齿轮所属的啮合（支点重合等诊断定位相关啮合用）
    gear_to_meshes = defaultdict(list)
    for mid, o in oriented.items():
        gear_to_meshes[o["gearDriver"]].append(mid)
        gear_to_meshes[o["gearDriven"]].append(mid)
    shaft_meshes = defaultdict(list)
    for g in state.get("gears", []):
        for mid in gear_to_meshes.get(g["id"], ()):
            shaft_meshes[g.get("shaftId")].append(mid)
    shafts_out = {}
    all_reactions = []
    for sid, s in shaft_objs.items():
        lay = layouts[sid]
        locks = set(lay.get("locked", []) or [])
        for b in lay["bearings"]:
            if b.get("locked"):
                locks.add(b["id"])
        face_loads = []
        faces_out = []
        for f in lay["faces"]:
            loads = face_forces.get(f["gear"], [(0.0, 0.0, 0.0, 0.0)])
            fx = sum(v[0] for v in loads)
            fy = sum(v[1] for v in loads)
            ftv = sum(v[2] for v in loads)
            frv = sum(v[3] for v in loads)
            g = gear_objs.get(f["gear"], {})
            face_loads.append({"id": f["id"], "x": _num(f.get("x")), "fx": fx, "fy": fy})
            faces_out.append({"id": f["id"], "gear": f["gear"],
                              "name": _gname(g), "x": _r(_num(f.get("x")), 2),
                              "locked": f["id"] in locks,
                              "fx": _r(fx, 2), "fy": _r(fy, 2),
                              "ft": _r(ftv, 2), "fr": _r(frv, 2),
                              "r": _r(math.hypot(fx, fy), 2)})
        bearings_in = [{"id": b["id"], "x": _num(b.get("x")),
                        "Cr": _num(b.get("Cr"), DEFAULT_CR),
                        "locked": b["id"] in locks or b.get("locked")}
                       for b in lay["bearings"]]
        sol = bearing_solve(face_loads, bearings_in, lay["length"], lay["minGap"])
        if sol["coincident"]:
            aff = shaft_meshes.get(sid, [])
            add("error", "BEARINGS_COINCIDENT",
                "轴「%s」两处轴承支点重合或缺失，载荷无法平衡（涉及啮合：%s）"
                % (_sname(s), "、".join(mesh_res[m]["name"] for m in aff)
                   if aff else "无"),
                shaft=sid, **{("m%d" % k): m for k, m in enumerate(aff)})
        for v in sol["violations"]:
            sev = "error" if v["code"] != "TOO_CLOSE" else "warning"
            add(sev, v["code"], "轴「%s」：%s" % (_sname(s), v["message"]), shaft=sid)
        for fid in sol["outside"]:
            add("warning", "LOAD_OUTSIDE_SPAN",
                "轴「%s」安装面 %s 悬在两轴承跨距之外（悬臂载荷）"
                % (_sname(s), fid), shaft=sid)
        bearings_out = []
        for r in sol["reactions"]:
            if not r:
                continue
            if r["util"] > 1.0 + 1e-9:
                add("warning", "BEARING_OVERLOAD",
                    "轴「%s」轴承 %s 反力 %.0f N 超过径向额定载荷 %.0f N（余量 %.2f）"
                    % (_sname(s), r["id"], r["r"], r["Cr"], r["margin"] or 0),
                    shaft=sid, bearing=r["id"])
            n = abs(float(n_ratio.get(sid, Fraction(0)))) * rpm0 if sid in n_ratio else 0
            l10 = (1e6 / (60.0 * n)) * (r["margin"] ** L10_P) \
                if n > 1e-9 and r["margin"] else None
            req_margin = (60.0 * n * duration / 1e6) ** (1.0 / L10_P) \
                if n > 1e-9 and duration > 0 else None
            if l10 is not None and duration > 0 and l10 < duration:
                add("warning", "LIFE_SHORT",
                    "轴「%s」轴承 %s L10 寿命 %.0f h 短于持续时间 %.0f h"
                    "（需余量 ≥ %.2f，实际 %.2f）"
                    % (_sname(s), r["id"], l10, duration,
                       req_margin or 0, r["margin"] or 0),
                    shaft=sid, bearing=r["id"])
            bearings_out.append({"id": r["id"], "x": _r(r["x"], 2),
                                 "Cr": _r(r["Cr"], 1), "locked": r["locked"],
                                 "rx": _r(r["rx"], 2), "ry": _r(r["ry"], 2),
                                 "r": _r(r["r"], 2), "util": _r(r["util"], 3),
                                 "margin": None if r["margin"] is None else _r(r["margin"], 3),
                                 "reqMargin": _r(req_margin, 3) if req_margin else None,
                                 "l10h": None if l10 is None else _r(l10, 1)})
            all_reactions.append((r["r"], sid, r["id"]))
        # 寿命余量 = 实际余量 / 目标寿命所需余量；取每根轴最小值
        life_ratios = [b["margin"] / b["reqMargin"]
                       for b in bearings_out
                       if b["margin"] is not None and b["reqMargin"]]
        margins = [b["margin"] for b in bearings_out
                   if b["margin"] is not None and b["r"] > 0.01]
        ns = abs(float(n_ratio.get(sid, Fraction(0)))) * rpm0 if sid in n_ratio else 0
        shafts_out[sid] = {
            "shaftId": sid, "name": _sname(s), "rpm": _r(ns, 2),
            "powerKw": _r(shaft_p.get(sid, 0.0), 4),
            "torqueNm": _r(shaft_p.get(sid, 0.0) * 1000.0 /
                           (2 * math.pi * ns / 60), 3) if ns > 1e-9 else 0.0,
            "length": _r(lay["length"], 2), "minGap": _r(lay["minGap"], 2),
            "adjMin": _r(lay["adjMin"], 2), "adjMax": _r(lay["adjMax"], 2),
            "faces": sorted(faces_out, key=lambda f: f["x"]),
            "bearings": sorted(bearings_out, key=lambda b: b["x"]),
            "coincident": sol["coincident"], "outside": sol["outside"],
            "violations": sol["violations"],
            "minMargin": _r(min(margins), 3) if margins else None,
            "minLifeMargin": _r(min(life_ratios), 3) if life_ratios else None,
            "peakReaction": _r(max((b["r"] for b in bearings_out), default=0.0), 2),
        }

    peak = max((r[0] for r in all_reactions), default=0.0)
    min_margin = min((shafts_out[sid]["minMargin"] for sid in shafts_out
                      if shafts_out[sid]["minMargin"] is not None), default=None)
    terminal = [sid for sid in order if not downstream.get(sid)]
    p_out = sum(shaft_p.get(sid, 0.0) for sid in terminal)
    p_loss = max(0.0, p_in_kw - p_out)
    return {
        "ok": not any(i["severity"] == "error" for i in issues),
        "issues": issues,
        "inputId": input_id,
        "input": {"torqueNm": _r(T0, 3), "rpm": _r(rpm0, 2),
                  "durationH": _r(duration, 2), "powerKw": _r(p_in_kw, 4),
                  "omega": _r(omega0, 3)},
        "meshes": mesh_res,
        "branches": sorted(branches_out, key=lambda b: str(b["shaftId"])),
        "shafts": shafts_out,
        "peakReaction": _r(peak, 2),
        "minMargin": None if min_margin is None else _r(min_margin, 3),
        "outputPowerKw": _r(p_out, 4),
        "lossKw": _r(p_loss, 4),
        "durationH": _r(duration, 2),
        "meshNames": {mid: r["name"] for mid, r in mesh_res.items()},
        "shaftNames": {sid: _sname(s) for sid, s in shaft_objs.items()},
    }


def _mesh_efficiency(spec, mid):
    """效率：允许显式 0（该级完全耗损，下游功率/转矩/啮合力归零）；
    缺省或非法（非数、越界）才回退 0.98。"""
    ms = (spec.get("meshes", {}) or {}).get(mid, {}) or {}
    v = ms.get("efficiency")
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        return 0.98
    if v < 0.0 or v > 1.0:
        return 0.98
    return float(v)


def _mesh_param(spec, mid, key, default):
    ms = (spec.get("meshes", {}) or {}).get(mid, {}) or {}
    v = ms.get(key)
    if not isinstance(v, (int, float)) or isinstance(v, bool) or v <= 0:
        return default
    if key == "efficiency":
        return min(1.0, float(v))
    return float(v)


def mesh_label(info: dict, o: dict) -> str:
    return "%s → %s" % (
        info["nameA"] if o["gearDriver"] == info["gearA"] else info["nameB"],
        info["nameB"] if o["gearDriven"] == info["gearB"] else info["nameA"])


# ----------------------------- 轴系布置搜索 -----------------------------

def search_layouts(state: dict, spec: dict, params: dict) -> dict:
    """在轴长、最小间距与每元素可调范围内枚举未锁定安装面/轴承位置，
    按 超限数 → 最小余量 → 峰值反力 → 改动量 排序。"""
    shaft_id = params.get("shaftId")
    grid = abs(_num(params.get("grid"), 2.0)) or 2.0
    limit = max(1, int(_num(params.get("limit"), 12)))
    deadline = time.time() + float(_num(params.get("timeBudget"), 6.0))

    base = analyze_case(state, spec)
    layouts = _merge_layouts(spec, state)
    if shaft_id not in layouts:
        return {"results": [], "shaftId": shaft_id, "note": "请选择有效的轴"}
    targets = [shaft_id] if shaft_id else sorted(layouts, key=str)

    out_shafts = {}
    for sid in targets:
        res = _search_one(state, spec, sid, layouts[sid], base, grid, limit, deadline)
        out_shafts[sid] = res
        if time.time() > deadline:
            res["truncated"] = True
            break
    return {"shafts": out_shafts, "note": None}


def _search_one(state, spec, sid, lay, base, grid, limit, deadline):
    sres = base["shafts"].get(sid, {})
    faces0 = sres.get("faces", [])
    bearings0 = sres.get("bearings", [])
    face_loads0 = [{"id": f["id"], "x": f["x"], "fx": f.get("fx", 0), "fy": f.get("fy", 0)}
                   for f in faces0]
    locks = set(lay.get("locked", []) or [])
    for b in lay["bearings"]:
        if b.get("locked"):
            locks.add(b["id"])
    length, min_gap = lay["length"], lay["minGap"]
    adj_min, adj_max = lay["adjMin"], lay["adjMax"]

    elems = []   # {id, kind, x0, cr}
    for f in lay["faces"]:
        elems.append({"id": f["id"], "kind": "face", "x0": _num(f.get("x")),
                      "locked": f["id"] in locks})
    for b in lay["bearings"]:
        elems.append({"id": b["id"], "kind": "bearing", "x0": _num(b.get("x")),
                      "cr": _num(b.get("Cr"), DEFAULT_CR),
                      "locked": b["id"] in locks or b.get("locked")})

    def slots(e):
        if e["locked"]:
            return [round(e["x0"], 4)]
        lo = max(0.0, e["x0"] - adj_min)
        hi = min(length, e["x0"] + adj_max)
        n = int(math.floor((hi - lo) / grid + 1e-9))
        return [round(min(hi, lo + k * grid), 4) for k in range(n + 1)]

    # 可调范围小的元素先放，剪枝更强
    ordered = sorted(range(len(elems)),
                     key=lambda i: (0 if elems[i]["locked"] else 1,
                                    len(slots(elems[i])), elems[i]["x0"]))
    # 目标寿命所需余量（同 analyze_case），用于把寿命不足计入超限
    duration = max(0.0, _num(spec.get("duration"), 1000.0))
    rpm = abs(_num(sres.get("rpm"), 0.0))
    req_margin = (60.0 * rpm * duration / 1e6) ** (1.0 / L10_P) \
        if rpm > 1e-9 and duration > 0 else None
    nodes = [0]
    truncated = False
    results = []

    def evaluate(pos):
        fl = [{"id": f["id"], "x": pos[f["id"]], "fx": f.get("fx", 0.0),
               "fy": f.get("fy", 0.0)} for f in face_loads0]
        bs = [{"id": e["id"], "x": pos[e["id"]], "Cr": e.get("cr", DEFAULT_CR),
               "locked": e["locked"]}
              for e in elems if e["kind"] == "bearing"]
        sol = bearing_solve(fl, bs, length, min_gap)
        n_overload = sum(1 for r in sol["reactions"]
                         if r and r.get("margin") is not None and r["margin"] < 1.0)
        n_life = sum(1 for r in sol["reactions"]
                     if req_margin and r.get("margin") is not None
                     and r["margin"] < req_margin)
        # 安装面悬于两支点跨距之外（悬臂承载）按不利布置计入超限
        n_outside = len(sol["outside"])
        violations = sol["nRange"] + sol["nGap"] + n_overload + n_life + \
            n_outside + (1 if sol["coincident"] else 0)
        margins = [r["margin"] for r in sol["reactions"]
                   if r and r.get("margin") is not None]
        peak = max((r["r"] for r in sol["reactions"] if r), default=0.0)
        change = sum(abs(pos[e["id"]] - e["x0"]) for e in elems)
        return {
            "positions": {k: _r(v, 2) for k, v in pos.items()},
            "violations": violations,
            "nOverload": n_overload, "nLife": n_life, "nGap": sol["nGap"],
            "nRange": sol["nRange"], "nOutside": n_outside,
            "coincident": sol["coincident"],
            "outside": sol["outside"],
            "minMargin": _r(min(margins), 3) if margins else None,
            "reqMargin": _r(req_margin, 3) if req_margin else None,
            "peakReaction": _r(peak, 2),
            "change": _r(change, 2),
            "reactions": {r["id"]: {"r": _r(r["r"], 2), "util": _r(r["util"], 3),
                                    "margin": _r(r["margin"], 3)}
                          for r in sol["reactions"] if r},
        }

    def dfs(k, pos, used):
        nonlocal truncated
        nodes[0] += 1
        if nodes[0] > SEARCH_NODE_CAP or time.time() > deadline:
            truncated = True
            return
        if k == len(ordered):
            cand = evaluate(pos)
            results.append(cand)
            results.sort(key=lambda c: (c["violations"],
                                        -(c["minMargin"] if c["minMargin"] is not None else -1e18),
                                        c["peakReaction"], c["change"]))
            if len(results) > limit * 4:
                del results[limit * 4:]
            return
        e = elems[ordered[k]]
        for x in slots(e):
            if any(abs(x - u) < min_gap - 1e-9 for u in used):
                continue
            pos[e["id"]] = x
            dfs(k + 1, pos, used + [x])
            del pos[e["id"]]
            if truncated:
                return

    dfs(0, {}, [])
    results.sort(key=lambda c: (c["violations"],
                                -(c["minMargin"] if c["minMargin"] is not None else -1e18),
                                c["peakReaction"], c["change"]))
    return {"name": sres.get("name", sid), "results": results[:limit],
            "nodes": nodes[0], "truncated": truncated}
