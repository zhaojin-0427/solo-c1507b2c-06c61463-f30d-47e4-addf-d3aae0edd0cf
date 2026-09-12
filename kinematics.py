# -*- coding: utf-8 -*-
"""轮系运动学核心：转速传播、约束诊断、复位循环、配齿搜索。

状态 JSON 结构：
{
  "shafts": [{"id","name","x","y","locked"}],
  "gears":  [{"id","shaftId","name","z","module","pressureAngle","internal","locked"}],
  "meshes": [{"id","gearA","gearB"}],          # 外/内啮合由齿轮 internal 标志推断
  "coaxRelations": [{"id","shaftA","shaftB"}], # 同轴（同位、各自转速独立）
  "planets": [{"id","name","module","zS","zP","zR","count","phase",
               "fixed","input","output",       # s=太阳轮 r=内齿圈 c=行星架
               "x","y",
               "sunShaftId","ringShaftId","carrierShaftId"}],  # 可空：接入现有轴
  "inputId","outputId","inputRpm"
}
长度单位均为 mm；转速为相对输入轴的分数（输入轴 = 1 转）。
行星级成员在转速图中以伪节点 "<id>:s|r|c" 出现；行星轮自转速度单独按
n_p = n_c − (zS/zP)(n_s − n_c) 计算。
"""
from __future__ import annotations

import math
import time
from collections import defaultdict, deque
from fractions import Fraction
from math import gcd, hypot

import meshing

CENTER_TOL = 0.05  # mm，中心距默认容差


# ----------------------------- 基础工具 -----------------------------

def lcm(a: int, b: int) -> int:
    return a // gcd(a, b) * b if a and b else max(a, b)


def fj(f: Fraction) -> dict:
    """Fraction -> 可 JSON 序列化结构。"""
    return {"s": str(f), "v": float(f)}


def _gname(g: dict) -> str:
    return g.get("name") or ("内齿圈 z%s" % g.get("z") if g.get("internal") else "齿轮 z%s" % g.get("z"))


def _sname(s: dict) -> str:
    return s.get("name") or ("轴%s" % s.get("id"))


# ----------------------------- 行星轮系 -----------------------------

P_MEMBERS = ("s", "r", "c")
P_NAMES = {"s": "太阳轮", "r": "内齿圈", "c": "行星架"}


def p_nodes(pid):
    return [pid + ":" + m for m in P_MEMBERS]


def planet_geom(p):
    """返回行星级几何数据；齿数/模数非法时返回 None。"""
    zs, zp, zr = p.get("zS"), p.get("zP"), p.get("zR")
    m = p.get("module")
    if not all(isinstance(z, int) and z > 0 for z in (zs, zp, zr)):
        return None
    if not isinstance(m, (int, float)) or m <= 0:
        return None
    orbit = m * (zs + zp) / 2.0          # 行星轴轨迹半径
    rp = m * zp / 2.0                    # 行星轮节圆/外圆（齿顶按 +m）
    return {
        "zs": zs, "zp": zp, "zr": zr, "m": m,
        "orbit": orbit, "rp": rp,
        "ringTip": m * (zr - 2) / 2.0,   # 内齿圈齿顶圆（内边界）
        "ringOuter": m * (zr + 2.5) / 2.0,
        "sunTip": m * (zs + 2) / 2.0,
        "chord": 2.0 * orbit * math.sin(math.pi / max(1, int(p.get("count", 1)))),
    }


def planet_spin(zs, zp, speeds, ns_id, nc_id):
    """行星轮绝对自转速度 n_p = n_c − (zS/zP)(n_s − n_c)。"""
    if ns_id not in speeds or nc_id not in speeds:
        return None
    return speeds[nc_id] - Fraction(zs, zp) * (speeds[ns_id] - speeds[nc_id])


def _planet_checks(p, shafts, gears):
    """行星级装配/几何诊断，返回 (issues, geom_or_None)。"""
    pid = p.get("id")
    issues = []
    name = p.get("name") or ("行星级 %s" % pid)
    n = p.get("count")
    if not isinstance(n, int) or not 2 <= n <= 6:
        issues.append(("error", "PLANET_COUNT",
                       "「%s」行星轮数量必须为 2～6 个" % name, {"planet": pid}))
    zs, zp, zr = p.get("zS"), p.get("zP"), p.get("zR")
    m = p.get("module")
    for key, label in (("zS", "太阳轮齿数"), ("zP", "行星轮齿数"), ("zR", "内齿圈齿数")):
        if not isinstance(p.get(key), int) or p.get(key) <= 0:
            issues.append(("error", "BAD_PLANET",
                           "「%s」%s必须为正整数" % (name, label), {"planet": pid}))
    if not isinstance(m, (int, float)) or m <= 0:
        issues.append(("error", "BAD_PLANET",
                       "「%s」模数必须为正数" % name, {"planet": pid}))
    if issues:
        return issues, None

    if zr != zs + 2 * zp:
        issues.append(("error", "WILLIS_GEOM",
                       "「%s」齿数不满足 z圈 = z太阳 + 2z行星：%s ≠ %s + 2×%s = %s"
                       % (name, zr, zs, zp, zs + 2 * zp), {"planet": pid}))
    geom = planet_geom(p)
    if isinstance(n, int) and 2 <= n <= 6 and zr == zs + 2 * zp:
        if (zs + zr) % n != 0:
            issues.append(("error", "ASSEMBLY",
                           "「%s」均布装配条件不满足：(z太阳+z圈)/n = (%s+%s)/%s 非整数，"
                           "第 %s 只行星轮无法在均布相位与太阳轮、内齿圈同时对齿"
                           % (name, zs, zr, n, n), {"planet": pid}))
        net = geom["chord"] - m * (zp + 2)
        if net <= 0:
            issues.append(("error", "PLANET_COLLIDE",
                           "「%s」相邻行星轮顶圆干涉：轴间距 %.2f mm，需大于齿顶圆直径 %.2f mm"
                           % (name, geom["chord"], m * (zp + 2)), {"planet": pid}))
        elif net < 0.15 * m:
            issues.append(("warning", "PLANET_TIGHT",
                           "「%s」行星轮净距仅 %.2f mm（< 0.15m），加工后易蹭齿"
                           % (name, net), {"planet": pid}))
        if geom["sunTip"] >= geom["ringTip"]:
            issues.append(("error", "COAX_CONFLICT",
                           "「%s」同轴尺寸冲突：太阳轮齿顶圆 %.2f mm 已触及内齿圈齿顶圆 %.2f mm"
                           % (name, geom["sunTip"], geom["ringTip"]), {"planet": pid}))

    # 接入轴：存在性、同轴度、与齿圈内腔的尺寸冲突
    att = {}
    for mem, key in (("s", "sunShaftId"), ("r", "ringShaftId"), ("c", "carrierShaftId")):
        sid = p.get(key)
        if sid is not None:
            if sid not in shafts:
                issues.append(("error", "BROKEN_PLANET_SHAFT",
                               "「%s」%s接入的轴已被删除" % (name, P_NAMES[mem]),
                               {"planet": pid}))
            else:
                att[mem] = sid
    px, py = float(p.get("x", 0) or 0), float(p.get("y", 0) or 0)
    for mem, sid in att.items():
        A = shafts[sid]
        d = hypot(A["x"] - px, A["y"] - py)
        if geom and d > 0.05:
            issues.append(("warning", "PLANET_OFFSET",
                           "「%s」%s接入的轴「%s」偏离行星级中心 %.2f mm，可用「吸附轴位」对齐"
                           % (name, P_NAMES[mem], _sname(shafts[sid]), d),
                           {"planet": pid, "shaft": sid}))
    if geom:
        for mem, sid in att.items():
            for g in gears.values():
                if g.get("shaftId") != sid or g.get("internal"):
                    continue
                if not isinstance(g.get("z"), int) or not isinstance(g.get("module"), (int, float)):
                    continue
                out = g["module"] * (g["z"] + 2) / 2.0
                if mem == "r":
                    continue  # 与内齿圈同速的外齿轮在齿圈外，不查内腔
                if out >= geom["ringTip"] - m:
                    issues.append(("error", "COAX_CONFLICT",
                                   "「%s」轴「%s」上的「%s」齿顶圆半径 %.2f mm 超过内齿圈内腔 %.2f mm"
                                   % (name, _sname(shafts[sid]), _gname(g), out, geom["ringTip"]),
                                   {"planet": pid, "shaft": sid, "gear": g.get("id")}))
    return issues, geom


# ----------------------------- 运动学分析 -----------------------------

def analyze(state: dict, center_tol: float = CENTER_TOL) -> dict:
    shafts = {s["id"]: dict(s) for s in state.get("shafts", [])}
    gears = {g["id"]: dict(g) for g in state.get("gears", [])}
    meshes = state.get("meshes", [])
    coax = state.get("coaxRelations", [])
    issues: list[dict] = []

    def add(sev, code, msg, **refs):
        issues.append({"severity": sev, "code": code, "message": msg, "refs": refs})

    # --- 齿轮参数与归属 ---
    for gid, g in gears.items():
        sid = g.get("shaftId")
        if sid not in shafts:
            add("error", "ORPHAN_GEAR", "「%s」没有安装到任何轴" % _gname(g), gear=gid)
        z, m = g.get("z"), g.get("module")
        if not isinstance(z, int) or z <= 0:
            add("error", "BAD_GEAR", "「%s」齿数必须为正整数" % _gname(g), gear=gid)
        if not isinstance(m, (int, float)) or m <= 0:
            add("error", "BAD_GEAR", "「%s」模数必须为正数" % _gname(g), gear=gid)
        if not isinstance(g.get("pressureAngle"), (int, float)) or g.get("pressureAngle", 0) <= 0:
            add("warning", "BAD_GEAR", "「%s」压力角异常" % _gname(g), gear=gid)

    # --- 啮合边 ---
    adj = {sid: [] for sid in shafts}
    edge_info = []
    pair_seen: dict[tuple, str] = {}

    for e in meshes:
        eid = e.get("id")
        a, b = e.get("gearA"), e.get("gearB")
        if a not in gears or b not in gears:
            add("error", "BROKEN_MESH", "存在指向已删除齿轮的啮合约束", mesh=eid)
            continue
        ga, gb = gears[a], gears[b]
        sa, sb = ga.get("shaftId"), gb.get("shaftId")
        if sa not in shafts or sb not in shafts:
            continue
        if sa == sb:
            add("error", "SELF_MESH", "「%s」与「%s」在同一根轴上，不能互相啮合"
                % (_gname(ga), _gname(gb)), mesh=eid)
            continue
        if ga.get("internal") and gb.get("internal"):
            add("error", "TWO_RINGS", "两个内齿圈不能互相啮合（%s、%s）"
                % (_gname(ga), _gname(gb)), mesh=eid)

        za, zb, m = ga.get("z"), gb.get("z"), ga.get("module")
        internal = bool(ga.get("internal") or gb.get("internal"))
        if internal:
            zr, zp = (za, zb) if ga.get("internal") else (zb, za)
            if isinstance(zr, int) and isinstance(zp, int) and zr <= zp:
                add("error", "RING_TOO_SMALL",
                    "内啮合时节圆半径为 m(z圈−z轮)/2：内齿圈 z%s 必须大于小齿轮 z%s"
                    % (zr, zp), mesh=eid)
        if isinstance(ga.get("module"), (int, float)) and isinstance(gb.get("module"), (int, float)) \
                and abs(ga["module"] - gb["module"]) > 1e-9:
            add("error", "MODULE_MISMATCH",
                "模数不合：%s (m=%s) 与 %s (m=%s) 无法啮合"
                % (_gname(ga), ga["module"], _gname(gb), gb["module"]), mesh=eid)
        if abs(ga.get("pressureAngle", 20) - gb.get("pressureAngle", 20)) > 1e-6:
            add("warning", "ALPHA_MISMATCH",
                "压力角不一致：%s (%s°) 与 %s (%s°)"
                % (_gname(ga), ga.get("pressureAngle"), _gname(gb), gb.get("pressureAngle")),
                mesh=eid)

        A, B = shafts[sa], shafts[sb]
        actual = hypot(A["x"] - B["x"], A["y"] - B["y"])
        if isinstance(za, int) and isinstance(zb, int) and isinstance(m, (int, float)):
            # 理论中心距随变位和变化（xΣ=0 时即标准中心距）
            expected = meshing.expected_center(ga, gb)
            if expected is None:
                expected = m * (za + zb) / 2 if not internal else m * abs(za - zb) / 2
            dev = actual - expected
            if abs(dev) > center_tol:
                add("error", "CENTER_DISTANCE",
                    "中心距冲突：%s—%s 应为 %.3f mm，实际 %.3f mm，偏差 %+.3f mm"
                    % (_gname(ga), _gname(gb), expected, actual, dev),
                    mesh=eid)
        else:
            expected, dev = None, None

        key = tuple(sorted((sa, sb)))
        if key in pair_seen:
            add("error", "DUPLICATE",
                "重复约束：轴「%s」与「%s」之间已有一处啮合 (%s)"
                % (_sname(shafts[sa]), _sname(shafts[sb]), pair_seen[key]), mesh=eid)
        else:
            pair_seen[key] = eid

        if isinstance(za, int) and isinstance(zb, int) and za > 0 and zb > 0:
            sign = 1 if internal else -1
            factor = sign * Fraction(za, zb)          # n_sb / n_sa
            adj[sa].append((sb, factor, eid))
            adj[sb].append((sa, Fraction(1, 1) / factor, eid))
        edge_info.append({
            "mesh": eid, "shaftA": sa, "shaftB": sb,
            "gearA": a, "gearB": b, "internal": internal,
            "expected": expected, "actual": round(actual, 6),
            "deviation": None if dev is None else round(dev, 6),
        })

    # --- 同轴关系 ---
    for r in coax:
        rid, a, b = r.get("id"), r.get("shaftA"), r.get("shaftB")
        if a not in shafts or b not in shafts:
            add("error", "BROKEN_COAX", "存在指向已删除轴的同轴约束", coax=rid)
            continue
        A, B = shafts[a], shafts[b]
        d = hypot(A["x"] - B["x"], A["y"] - B["y"])
        if d > center_tol:
            add("warning", "COAX_OFFSET",
                "同轴关系要求「%s」与「%s」同心，当前相距 %.3f mm"
                % (_sname(A), _sname(B), d), coax=rid)

    # --- 行星轮系：Willis 约束 ---
    # 伪节点 "<pid>:s/r/c"；固定件与“接入轴”以系数 1 耦合；
    # 活动的 s/r/c 三者间按转化轮系传动比 (n_s-n_c)/(n_r-n_c) = -z_r/z_s 建边。
    planet_infos = []
    planets = state.get("planets", [])
    for p in planets:
        pid = p.get("id")
        pissues, geom = _planet_checks(p, shafts, gears)
        for sev, code, msg, refs in pissues:
            add(sev, code, msg, **refs)
        fixed, inp, outp = p.get("fixed"), p.get("input"), p.get("output")
        valid_role = set(x for x in (fixed, inp, outp) if x in P_MEMBERS)
        if len(set((fixed, inp, outp))) != 3 or len(valid_role) != 3:
            add("error", "BAD_PLANET_ROLE",
                "「%s」固定件、输入件、输出件必须分别指定为太阳轮/内齿圈/行星架且互不相同"
                % (p.get("name") or ("行星级 %s" % pid)), planet=pid)
        ns_id, nr_id, nc_id = p_nodes(pid)
        for node in (ns_id, nr_id, nc_id):
            adj[node] = []
        if geom is None:
            planet_infos.append({"id": pid, "speeds": {}, "spin": None, "orbit": None})
            continue
        zs, zr = geom["zs"], geom["zr"]

        def wadd(a_node, b_node, fact):
            """双向 Willis 边（转速图）。"""
            adj[a_node].append((b_node, fact, "willis:" + pid))
            adj[b_node].append((a_node, Fraction(1, 1) / fact, "willis:" + pid))

        if fixed == "r":      # n_s/n_c = (zs+zr)/zs
            wadd(nc_id, ns_id, Fraction(zs + zr, zs))
        elif fixed == "s":    # n_r/n_c = (zs+zr)/zr
            wadd(nc_id, nr_id, Fraction(zs + zr, zr))
        elif fixed == "c":    # n_r/n_s = -zs/zr
            wadd(ns_id, nr_id, Fraction(-zs, zr))

        # 接入现有轴：伪节点与轴同速（系数 1，双向）
        for mem, key, node in (("s", "sunShaftId", ns_id),
                               ("r", "ringShaftId", nr_id),
                               ("c", "carrierShaftId", nc_id)):
            sid = p.get(key)
            if sid in shafts:
                adj[node].append((sid, Fraction(1, 1), "pllink:" + pid))
                adj[sid].append((node, Fraction(1, 1), "pllink:" + pid))

        planet_infos.append({
            "id": pid, "name": p.get("name") or "",
            "fixed": fixed, "input": inp, "output": outp,
            "zS": zs, "zP": geom["zp"], "zR": zr, "module": geom["m"],
            "count": p.get("count"), "x": p.get("x", 0), "y": p.get("y", 0),
            "phase": p.get("phase", 0) or 0,
            "nodes": {"s": ns_id, "r": nr_id, "c": nc_id},
            "orbit": geom["orbit"], "ringTip": geom["ringTip"],
            "ringOuter": geom["ringOuter"], "chord": geom["chord"],
            "speeds": {}, "spin": None,
        })

    # --- 转速传播（BFS，分数精确） ---
    # 多源：输入轴/输入成员速度 1；各行星级固定成员（及接入轴）速度 0。
    input_id, output_id = state.get("inputId"), state.get("outputId")
    speeds: dict = {}
    conflicts: set = set()
    seeds = []
    for p in state.get("planets", []):
        pid = p.get("id")
        if p.get("fixed") in P_MEMBERS:
            seeds.append((pid + ":" + p["fixed"], Fraction(0)))

    def bfs(seed_id, seed_val):
        if seed_id not in adj:
            return
        if seed_id in speeds:
            if speeds[seed_id] != seed_val:
                add("error", "LOCKED_TRAIN",
                    "输入件与固定件被连成一体（%s 同时被要求转速 1 和 0），轮系锁死"
                    % seed_id, shaft=seed_id)
            return
        speeds[seed_id] = seed_val
        q = deque([seed_id])
        while q:
            cur = q.popleft()
            for nb, fact, eid in adj[cur]:
                v = speeds[cur] * fact
                if nb in speeds:
                    if speeds[nb] != v and eid not in conflicts:
                        conflicts.add(eid)
                        same_sign = (speeds[nb] > 0) == (v > 0)
                        code = "RATIO_CONFLICT" if same_sign else "DIRECTION_CONFLICT"
                        label = _sname(shafts[nb]) if nb in shafts else \
                            "行星级 %s 的%s" % (nb.split(":")[0],
                                               P_NAMES.get(nb.split(":")[-1], nb))
                        if same_sign:
                            msg = "传动比矛盾：%s 经两条路径推得 %s 与 %s" % (
                                label, speeds[nb], v)
                        else:
                            msg = "转向矛盾：%s 经两条路径转向相反（%s 与 %s），闭合轮系齿数不满足约束" % (
                                label, speeds[nb], v)
                        add("error", code, msg, mesh=eid if not eid.startswith(("willis:", "pllink:")) else None,
                            planet=eid.split(":")[1] if eid.startswith(("willis:", "pllink:")) else None,
                            shaft=nb if nb in shafts else None)
                else:
                    speeds[nb] = v
                    q.append(nb)

    for sid, val in seeds:
        bfs(sid, val)
    if not input_id:
        add("info", "NO_INPUT", "尚未指定输入轴（在轴属性中设置）")
    else:
        bfs(input_id, Fraction(1, 1))

    for sid, s in shafts.items():
        if sid not in speeds and any(g.get("shaftId") == sid for g in gears.values()):
            add("warning", "IDLE_SHAFT", "轴「%s」未连入输入轴的动力链" % _sname(s), shaft=sid)
    if output_id and output_id not in speeds:
        add("warning", "NO_OUTPUT_PATH", "输出轴无法从输入轴到达")
    if not output_id:
        add("info", "NO_OUTPUT", "尚未指定输出轴（在轴属性或行星级编辑器中设置）")

    # --- 总传动比 ---
    ratio = speeds.get(output_id) if output_id else None
    input_rpm = float(state.get("inputRpm", 1) or 1)
    rpms = {sid: input_rpm * float(v) for sid, v in speeds.items()}

    # --- 行星轮自转速度与成员信息 ---
    for info in planet_infos:
        if "nodes" not in info:
            continue
        ns_id = info["nodes"]["s"]
        nc_id = info["nodes"]["c"]
        spin = planet_spin(info["zS"], info["zP"], speeds, ns_id, nc_id)
        for mem, node in info["nodes"].items():
            info["speeds"][mem] = fj(speeds[node]) if node in speeds else None
        info["spin"] = fj(spin) if spin is not None else None

    # --- 整列复位循环 ---
    # 普通齿轮/太阳轮/内齿圈：走过的齿距数 z·n·L 必须为整数；
    # 行星轮按绝对自转 n_p（行星架公转不恢复其齿相位）；
    # 均布行星轴位恢复：行星架转过的角度须为 2π/n 的整数倍，即 n·n_c·L 为整数
    # （n 个相同行星轮虽可互换到视觉重合，但轴位与啮合相位需回到初始槽位）。
    denoms = []
    for gid, g in gears.items():
        sid = g.get("shaftId")
        if sid in speeds and isinstance(g.get("z"), int):
            denoms.append((g["z"] * speeds[sid]).denominator)
    for info in planet_infos:
        if "nodes" not in info:
            continue
        for mem, z in (("s", info["zS"]), ("r", info["zR"])):
            node = info["nodes"][mem]
            if node in speeds:
                denoms.append((z * speeds[node]).denominator)
        spin = planet_spin(info["zS"], info["zP"], speeds,
                           info["nodes"]["s"], info["nodes"]["c"])
        if spin is not None:
            denoms.append((info["zP"] * spin).denominator)
        nc = speeds.get(info["nodes"]["c"])
        count = info.get("count")
        if nc is not None and isinstance(count, int) and 2 <= count <= 6:
            denoms.append((count * nc).denominator)
    L = 1
    for d in denoms:
        L = lcm(L, d)
    shaft_turns = {sid: fj(Fraction(L) * v) for sid, v in speeds.items()}
    cycle = None
    if speeds:
        cycle = {"inputTurns": L, "shaftTurns": shaft_turns,
                 "inputTurnsV": float(L)}

    return {
        "ok": not any(i["severity"] == "error" for i in issues),
        "issues": issues,
        "speeds": {sid: fj(v) for sid, v in speeds.items()},
        "rpms": rpms,
        "ratio": None if ratio is None else fj(ratio),
        "cycle": cycle,
        "edges": edge_info,
        "planets": planet_infos,
    }


# ----------------------------- 轴位重排 -----------------------------

def _mesh_edges_geo(state, gears, shafts):
    out = []
    for e in state.get("meshes", []):
        a, b = e.get("gearA"), e.get("gearB")
        if a not in gears or b not in gears:
            continue
        ga, gb = gears[a], gears[b]
        sa, sb = ga.get("shaftId"), gb.get("shaftId")
        if sa not in shafts or sb not in shafts or sa == sb:
            continue
        if not isinstance(ga.get("z"), int) or not isinstance(gb.get("z"), int):
            continue
        internal = bool(ga.get("internal") or gb.get("internal"))
        req = meshing.expected_center(ga, gb)
        if req is None:
            req = ga["module"] * (ga["z"] + gb["z"]) / 2 if not internal \
                else ga["module"] * abs(ga["z"] - gb["z"]) / 2
        out.append((sa, sb, req, internal))
    return out


def place_shafts(state: dict, locked_ids: set) -> tuple[dict, float, float]:
    """以锁定轴为锚点，按啮合中心距布置其余轴。
    返回 (新坐标, 最大中心距残差 mm, 非锁轴最大位移 mm)。"""
    shafts = {s["id"]: dict(s) for s in state["shafts"]}
    gears = {g["id"]: g for g in state["gears"]}
    old = {sid: (float(s["x"]), float(s["y"])) for sid, s in shafts.items()}

    pos = {sid: old[sid] for sid in locked_ids if sid in old}
    input_id = state.get("inputId")
    if not pos and input_id in old:
        pos[input_id] = old[input_id]
    edges = _mesh_edges_geo(state, gears, shafts)
    neigh = defaultdict(list)
    for a, b, req, internal in edges:
        neigh[a].append((b, req))
        neigh[b].append((a, req))

    guard = 0
    while guard < len(shafts) + 2:
        guard += 1
        progress = False
        for sid in shafts:
            if sid in pos:
                continue
            anchors = [(pos[nb], req) for nb, req in neigh[sid] if nb in pos]
            if not anchors:
                continue
            progress = True
            (x0, y0), r0 = anchors[0]
            if len(anchors) == 1:
                dx, dy = old[sid][0] - x0, old[sid][1] - y0
                d = hypot(dx, dy) or 1.0
                pos[sid] = (x0 + dx / d * r0, y0 + dy / d * r0)
            else:
                (x1, y1), r1 = anchors[1]
                dx, dy, d = x1 - x0, y1 - y0, hypot(x1 - x0, y1 - y0) or 1e-9
                # 两圆交点
                if d > r0 + r1 or d < abs(r0 - r1):
                    t = max(0.0, min(1.0, (d + r0 - r1) / (2 * d))) if d else 0.5
                    pos[sid] = (x0 + dx * t, y0 + dy * t)
                else:
                    a = (r0 * r0 - r1 * r1 + d * d) / (2 * d)
                    h = math.sqrt(max(0.0, r0 * r0 - a * a))
                    mx, my = x0 + a * dx / d, y0 + a * dy / d
                    px, py = -dy / d * h, dx / d * h
                    c1, c2 = (mx + px, my + py), (mx - px, my - py)
                    pos[sid] = c1 if hypot(c1[0] - old[sid][0], c1[1] - old[sid][1]) <= \
                                   hypot(c2[0] - old[sid][0], c2[1] - old[sid][1]) else c2
        if not progress:
            break

    for sid in shafts:
        if sid not in pos:
            pos[sid] = old[sid]

    max_res = 0.0
    for a, b, req, _ in edges:
        d = hypot(pos[a][0] - pos[b][0], pos[a][1] - pos[b][1])
        max_res = max(max_res, abs(d - req))
    max_move = 0.0
    for sid in shafts:
        if sid not in locked_ids:
            max_move = max(max_move, hypot(pos[sid][0] - old[sid][0],
                                           pos[sid][1] - old[sid][1]))
    return {sid: {"x": round(x, 4), "y": round(y, 4)} for sid, (x, y) in pos.items()}, \
        round(max_res, 4), round(max_move, 4)


# ----------------------------- 配齿搜索 -----------------------------

class _UF:
    def __init__(self, items):
        self.p = {x: x for x in items}

    def find(self, x):
        while self.p[x] != x:
            self.p[x] = self.p[self.p[x]]
            x = self.p[x]
        return x

    def union(self, a, b):
        ra, rb = self.find(a), self.find(b)
        if ra != rb:
            self.p[rb] = ra


def _train_component(state):
    gears = {g["id"]: g for g in state.get("gears", [])}
    g2s = {gid: g.get("shaftId") for gid, g in gears.items()}
    nsh = defaultdict(set)
    gpair = []
    for e in state.get("meshes", []):
        a, b = e.get("gearA"), e.get("gearB")
        if a in g2s and b in g2s and g2s[a] != g2s[b]:
            nsh[g2s[a]].add(g2s[b])
            nsh[g2s[b]].add(g2s[a])
            gpair.append((a, b))
    input_id = state.get("inputId")
    reach = set()
    if input_id:
        reach.add(input_id)
        q = deque([input_id])
        while q:
            u = q.popleft()
            for v in nsh[u]:
                if v not in reach:
                    reach.add(v)
                    q.append(v)
    train_gears = {gid for gid, sid in g2s.items() if sid in reach}
    # 只保留动力链内的啮合边（画布上独立的啮合组不参与搜索）
    gpair = [(a, b) for (a, b) in gpair if a in train_gears and b in train_gears]
    return reach, train_gears, gpair


def search(params: dict) -> dict:
    state = params["state"]
    target_s = str(params.get("target", "")).strip().replace(" ", "")
    tol = float(params.get("tolerancePct", 1.0))
    zmin = int(params.get("zMin", 8))
    zmax = int(params.get("zMax", 120))
    modules = sorted({float(x) for x in str(params.get("modules", "1")).replace("，", ",").split(",") if x.strip()})
    center_tol = float(params.get("centerTol", CENTER_TOL))
    result_limit = int(params.get("limit", 150))
    node_cap = int(params.get("nodeCap", 600_000))
    deadline = time.time() + float(params.get("timeBudget", 8.0))

    target = Fraction(target_s)
    tol_frac = Fraction(str(tol)) / 100
    tol_lo = 1 - tol_frac
    tol_hi = 1 + tol_frac
    magnitude_only = not target_s.startswith("-")
    base = analyze(state, center_tol)

    reach, train_gears, gpair = _train_component(state)
    input_id = state.get("inputId")
    output_id = state.get("outputId")
    if output_id not in reach:
        return {"results": [], "truncated": False,
                "note": "输出轴不在输入轴的啮合网络中，无法按目标传动比搜索。"}
    train_gears = {gid for gid in train_gears
                   if any(gid == a or gid == b for a, b in gpair)}
    gears = {g["id"]: g for g in state["gears"] if g["id"] in train_gears}
    shafts = {s["id"]: s for s in state["shafts"]}
    locked_pos = {sid for sid, s in shafts.items() if s.get("locked")}

    # 模数连通组：啮合在一起的齿轮模数必须相同
    uf = _UF(train_gears)
    for a, b in gpair:
        uf.union(a, b)
    groups = defaultdict(list)
    for gid in train_gears:
        groups[uf.find(gid)].append(gid)

    group_mods: dict = {}
    for _, members in groups.items():
        allowed = set(modules)
        for gid in members:
            g = gears[gid]
            if g.get("locked"):
                allowed &= {float(g["module"])}
        if not allowed:
            locked_m = sorted({gears[gid]["module"] for gid in members if gears[gid].get("locked")})
            return {"results": [], "truncated": False,
                    "note": "存在锁定齿轮的模数 %s 不在可用模数列表 %s 中，无解。"
                            % (locked_m, modules)}
        group_mods[tuple(sorted(members))] = sorted(allowed)

    z_domain = {}
    for gid, g in gears.items():
        if g.get("locked"):
            z_domain[gid] = (int(g["z"]),)
        else:
            z_domain[gid] = tuple(range(zmin, zmax + 1))

    # 变量齿轮（未锁定）按其啮合度数从高到低排列，尽早产生剪枝
    degree = defaultdict(int)
    for a, b in gpair:
        degree[a] += 1
        degree[b] += 1
    var_gears = sorted((gid for gid in train_gears if not gears[gid].get("locked")),
                       key=lambda g: (-degree[g], str(g)))
    group_list = list(group_mods.items())     # [(members_tuple, [module,...]), ...]

    nodes = [0]
    truncated = [False]
    raw = []
    seen = set()
    HARD_CODES = {"RATIO_CONFLICT", "DIRECTION_CONFLICT", "TWO_RINGS",
                  "SELF_MESH", "BAD_GEAR", "ORPHAN_GEAR", "RING_TOO_SMALL"}

    def stop():
        if nodes[0] > node_cap or time.time() > deadline:
            truncated[0] = True
            return True
        return False

    def prune(assign):
        """assign: {gid: (module, z_or_None)}；只检查已赋齿数的啮合边。"""
        for a, b in gpair:
            if a not in assign or b not in assign:
                continue
            ma, za = assign[a]
            mb, zb = assign[b]
            if za is None or zb is None:
                continue
            if abs(ma - mb) > 1e-12:
                return False
            ga, gb = gears[a], gears[b]
            internal = ga.get("internal") or gb.get("internal")
            if internal:
                zr, zp = (za, zb) if ga.get("internal") else (zb, za)
                if zr <= zp:
                    return False
            sa, sb = ga["shaftId"], gb["shaftId"]
            if sa in locked_pos and sb in locked_pos:
                A, B = shafts[sa], shafts[sb]
                dist = hypot(A["x"] - B["x"], A["y"] - B["y"])
                req = meshing.expected_center(dict(ga, z=za, module=ma),
                                              dict(gb, z=zb, module=mb))
                if req is None:
                    req = ma * (za + zb) / 2 if not internal else ma * abs(za - zb) / 2
                if abs(dist - req) > center_tol:
                    return False
        return True

    # ---- 增量传播：BFS 树序枚举，避免每叶重跑全量传播 ----
    # 以输入轴为根对“轴啮合图”BFS，确定各轴父边
    par_shaft = {input_id: None}
    par_edge = {}
    q = deque([input_id])
    shaft_level = {input_id: 0}
    edge_order = []          # (mesh索引, 父轴, 子轴)
    cross_edges = []         # (mesh索引) 两端轴都在更早层
    while q:
        cur = q.popleft()
        for k, (a, b) in enumerate(gpair):
            ga, gb = gears[a], gears[b]
            other = None
            if ga["shaftId"] == cur and gb["shaftId"] not in par_shaft:
                other = gb["shaftId"]
            elif gb["shaftId"] == cur and ga["shaftId"] not in par_shaft:
                other = ga["shaftId"]
            if other is not None:
                par_shaft[other] = cur
                par_edge[other] = k
                shaft_level[other] = shaft_level[cur] + 1
                edge_order.append((k, cur, other))
                q.append(other)
    for k in range(len(gpair)):
        a, b = gpair[k]
        ga, gb = gears[a], gears[b]
        if par_edge.get(ga["shaftId"]) == k or par_edge.get(gb["shaftId"]) == k:
            continue
        cross_edges.append(k)

    # 变量齿轮的枚举顺序：先父边上的，再随 BFS 层推进；锁定齿轮不在枚举中
    gear_on_edge = defaultdict(list)
    for k, (a, b) in enumerate(gpair):
        gear_on_edge[k].extend([a, b])
    order_gears = []
    seen_order = set()
    for k, _, _ in edge_order:
        for gid in gear_on_edge[k]:
            if not gears[gid].get("locked") and gid not in seen_order:
                seen_order.add(gid)
                order_gears.append(gid)
    for k in cross_edges:
        for gid in gear_on_edge[k]:
            if not gears[gid].get("locked") and gid not in seen_order:
                seen_order.add(gid)
                order_gears.append(gid)
    # 理论上 order_gears 应包含全部变量齿轮
    for gid in train_gears:
        if not gears[gid].get("locked") and gid not in seen_order:
            order_gears.append(gid)

    # ---- 搜索专用轻量布置（只依赖齿数与模数，按 BFS 树布置） ----
    locked_xy = {sid: (float(shafts[sid]["x"]), float(shafts[sid]["y"])) for sid in locked_pos}
    old_xy = {sid: (float(s["x"]), float(s["y"])) for sid, s in shafts.items()}
    tree_edges = [(cur, other, k) for k, cur, other in edge_order]
    cross_idx = cross_edges

    def place_light(assign):
        """返回 (positions, max_res, max_move)。树边精确布置，残差校验全部边。"""
        pos = dict(locked_xy)
        if input_id not in pos:
            pos[input_id] = old_xy[input_id]
        edge_req = {}
        for k, (a, b) in enumerate(gpair):
            ga, gb = gears[a], gears[b]
            m, za, zb = assign[a][0], assign[a][1], assign[b][1]
            internal = bool(ga.get("internal") or gb.get("internal"))
            req = meshing.expected_center(dict(ga, z=za, module=m),
                                          dict(gb, z=zb, module=m))
            edge_req[k] = req if req is not None else \
                (m * (za + zb) / 2 if not internal else m * abs(za - zb) / 2)
        for parent, child, k in tree_edges:
            if child in pos or parent not in pos:
                continue
            req = edge_req[k]
            x0, y0 = pos[parent]
            ox, oy = old_xy[child]
            dx, dy = ox - x0, oy - y0
            d = hypot(dx, dy) or 1.0
            pos[child] = (x0 + dx / d * req, y0 + dy / d * req)
        for sid in shafts:
            if sid not in pos:
                pos[sid] = old_xy[sid]
        # 残差：树边按构造精确；只需校验闭合（cross）边
        max_res = 0.0
        for k in cross_idx:
            a, b = gpair[k]
            sa, sb = gears[a]["shaftId"], gears[b]["shaftId"]
            d = hypot(pos[sa][0] - pos[sb][0], pos[sa][1] - pos[sb][1])
            max_res = max(max_res, abs(d - edge_req[k]))
        max_move = 0.0
        for sid in shafts:
            if sid not in locked_pos:
                max_move = max(max_move,
                               hypot(pos[sid][0] - old_xy[sid][0],
                                     pos[sid][1] - old_xy[sid][1]))
        positions = {sid: {"x": round(x, 4), "y": round(y, 4)}
                     for sid, (x, y) in pos.items()}
        return positions, round(max_res, 4), round(max_move, 4)

    def complete(assign, sp_out):
        """assign: {gid:(m,z)}；sp_out: 各轴整数对转速 (num, den)。"""
        changed = {}
        for gid, (m, z) in assign.items():
            g0 = gears[gid]
            if not g0.get("locked") and (
                    int(g0["z"]) != z or abs(float(g0["module"]) - float(m)) > 1e-12):
                changed[gid] = (int(g0["z"]), float(g0["module"]))
        placed = place_light(assign)
        if placed is None:
            return
        positions, max_res, max_move = placed
        if max_res > center_tol:
            return  # 几何不可行（如两锁定圆无交点），候选不可用

        # L：使所有 z*n 为整数的最小输入转数；n=num/den（未归约），分母为 den/gcd(z*num,den)
        L = 1
        for gid, (m, z) in assign.items():
            sid = gears[gid]["shaftId"]
            if sid in sp_out:
                pn, pd = sp_out[sid]
                L = lcm(L, pd // gcd(z * pn, pd))

        replaced = []
        for gid, (z0, m0) in changed.items():
            z1, m1 = assign[gid][1], assign[gid][0]
            replaced.append({"id": gid, "name": _gname(gears[gid]),
                             "z0": z0, "m0": m0, "z1": z1, "m1": m1,
                             "internal": bool(gears[gid].get("internal"))})
        on, od = sp_out[output_id]
        rr = Fraction(on, od)
        err = (abs(abs(rr) - abs(target)) / abs(target)) if magnitude_only \
            else abs(rr - target) / abs(target)
        base_cycle = (base.get("cycle") or {}).get("inputTurnsV", 1) or 1
        raw.append({
            "gears": {gid: {"z": assign[gid][1], "module": assign[gid][0]} for gid in changed},
            "shafts": positions,
            "ratio": fj(rr),
            "errorPct": round(float(err) * 100, 5),
            "maxMove": max_move,
            "maxResidual": max_res,
            "maxZ": max(z for _, z in assign.values()),
            "replaced": replaced,
            "cycle": str(L),
            "cycleV": float(L),
            "cycleRatio": round(float(L) / base_cycle, 4),
        })

    # 输出路径：从输入到输出经过的树边，记录 (父轴齿轮id, 子轴齿轮id, 内啮合?)
    out_path_edges = []
    if output_id in par_shaft:
        cur = output_id
        while par_shaft[cur] is not None:
            k = par_edge[cur]
            a, b = gpair[k]
            ga, gb = gears[a], gears[b]
            if ga["shaftId"] == par_shaft[cur]:
                gp_id, gc_id, internal = a, b, bool(ga.get("internal") or gb.get("internal"))
            else:
                gp_id, gc_id, internal = b, a, bool(ga.get("internal") or gb.get("internal"))
            out_path_edges.append((gp_id, gc_id, internal))
            cur = par_shaft[cur]
    out_path_edges.reverse()
    out_path_gears = set(x for e in out_path_edges for x in e[:2])

    def propagate(gid, assign, sp):
        """整数对转速传播：sp[shaft] = (num, den, sign=±1 合入 num 符号)。
        不做 gcd 归约；比较时叉乘。返回 (ok, touched)。"""
        stack = [gid]
        touched = set()
        while stack:
            cur_gear = stack.pop()
            for k in gear_of[cur_gear]:
                a, b = gpair[k]
                ga, gb = gears[a], gears[b]
                za, zb = assign.get(a, (None, None))[1], assign.get(b, (None, None))[1]
                if za is None or zb is None:
                    continue
                internal = bool(ga.get("internal") or gb.get("internal"))
                sa, sb = ga["shaftId"], gb["shaftId"]
                sgn = 1 if internal else -1
                # v_b = sgn*za/zb*v_a ;  v_a = sgn*zb/za*v_b
                candidates = ((sa, sb, sgn * za, zb), (sb, sa, sgn * zb, za))
                for drive, driven, pn, qn in candidates:
                    if drive not in sp:
                        continue
                    dn, dd = sp[drive]
                    v = (dn * pn, dd * qn)
                    if driven in sp:
                        en, ed = sp[driven]
                        if en * v[1] != v[0] * ed:
                            return False, touched
                    else:
                        sp[driven] = v
                        touched.add(driven)
                        for other in shaft_gears[driven]:
                            if other not in (a, b) and \
                                    assign.get(other, (None, None))[1] is not None:
                                stack.append(other)
        return True, touched

    # 容差目标上下界（预计算为整数分数，供叉乘）
    _at = abs(target)
    _lb, _hb = _at * tol_lo, _at * tol_hi
    tol_lo_n, tol_lo_d = _lb.numerator, _lb.denominator
    tol_hi_n, tol_hi_d = _hb.numerator, _hb.denominator

    def range_prune(assign):
        """输出树路径上：已定齿数贡献固定，未定齿数取 [zmin,zmax]，
        要求输出总比值的绝对值与目标容差区间有交集；符号已定时校验符号。"""
        if not out_path_edges:
            return True
        lo_n, lo_d, hi_n, hi_d = 1, 1, 1, 1
        fixed_sign = 1
        sign_known = True
        for gp_id, gc_id, internal in out_path_edges:
            if not internal:
                fixed_sign *= -1
            zp = assign.get(gp_id, (None, None))[1]
            zc = assign.get(gc_id, (None, None))[1]
            if zp is None:
                sign_known = False
            if zc is None:
                sign_known = False
            if zp is None and zc is None:
                lo_n, lo_d, hi_n, hi_d = lo_n * zmin, lo_d * zmax, hi_n * zmax, hi_d * zmin
            elif zp is None:
                lo_n, lo_d, hi_n, hi_d = lo_n * zmin, lo_d * zc, hi_n * zmax, hi_d * zc
            elif zc is None:
                lo_n, lo_d, hi_n, hi_d = lo_n * zp, lo_d * zmax, hi_n * zp, hi_d * zmin
            else:
                lo_n, lo_d, hi_n, hi_d = lo_n * zp, lo_d * zc, hi_n * zp, hi_d * zc
        if lo_n * hi_d > hi_n * lo_d:
            lo_n, hi_n = hi_n, lo_n
            lo_d, hi_d = hi_d, lo_d
        # hi >= 下界 且 lo <= 上界，整数叉乘（上下界已预算）
        if hi_n * tol_lo_d < tol_lo_n * hi_d:
            return False
        if lo_n * tol_hi_d > tol_hi_n * lo_d:
            return False
        if sign_known and not magnitude_only:
            if (fixed_sign >= 0) != (target >= 0):
                return False
        return True

    # 齿轮 -> 所在啮合边索引；轴 -> 齿轮
    gear_of = defaultdict(list)
    for k, (a, b) in enumerate(gpair):
        gear_of[a].append(k)
        gear_of[b].append(k)
    shaft_gears = defaultdict(list)
    for gid, g in gears.items():
        shaft_gears[g["shaftId"]].append(gid)

    def dfs_teeth(idx, assign, sp):
        if stop():
            return
        if idx == len(order_gears):
            complete(assign, sp)
            return
        nodes[0] += 1
        gid = order_gears[idx]
        module_val = assign[gid][0]
        # 输出轴转速若已确定，整个子树只能微调误差？——只能在叶判定，其余齿轮可能
        # 位于与输出无关的已连部分；此处先不提前剪。
        for z in z_domain[gid]:
            assign[gid] = (module_val, z)
            if not prune(assign):
                continue
            if not range_prune(assign):
                continue
            ok, touched = propagate(gid, assign, sp)
            if ok:
                # 输出轴速度一旦算出即可按容差剪枝
                out_v = sp.get(output_id)
                if out_v is not None:
                    on, od = out_v
                    # |on/od - t| <= |t|*tol/100，叉乘比较
                    tn, td = target.numerator, target.denominator
                    # 比较 |on*td| 与 |tn*od| 的相对误差
                    lhs = abs(abs(on * td) - abs(tn * od)) * 100
                    rhs = abs(tn * od) * tol
                    if lhs <= rhs:
                        dfs_teeth(idx + 1, assign, sp)
                else:
                    dfs_teeth(idx + 1, assign, sp)
                if truncated[0]:
                    return
            for sid in touched:
                sp.pop(sid, None)
        assign[gid] = (module_val, None)

    def dfs_modules(gi, assign):
        """先为每个模数连通组选定模数（锁定齿轮已固定）。"""
        if stop():
            return
        if gi == len(group_list):
            # 先从锁定齿轮做一轮初始传播（整数对转速）
            sp0 = {input_id: (1, 1)}
            touched_all = set()
            for gid in sorted(train_gears,
                              key=lambda x: shaft_level.get(gears[x]["shaftId"], 99)):
                if gears[gid].get("locked"):
                    ok, touched = propagate(gid, assign, sp0)
                    touched_all |= touched
                    if not ok:
                        return
            dfs_teeth(0, assign, sp0)
            return
        members, mods = group_list[gi]
        for m in mods:
            for gid in members:
                assign[gid] = (m, int(gears[gid]["z"]) if gears[gid].get("locked") else None)
            if prune(assign):
                dfs_modules(gi + 1, assign)
                if truncated[0]:
                    return
            for gid in members:
                if gears[gid].get("locked"):
                    assign[gid] = (float(gears[gid]["module"]), int(gears[gid]["z"]))
                else:
                    assign.pop(gid, None)

    # 初始 assign：锁定齿轮
    init_assign = {gid: (float(gears[gid]["module"]), int(gears[gid]["z"]))
                   for gid in train_gears if gears[gid].get("locked")}
    dfs_modules(0, dict(init_assign))

    raw.sort(key=lambda c: (c["errorPct"], c["maxMove"], c["maxZ"], c["maxResidual"]))
    return {
        "results": raw[:result_limit],
        "totalMatched": len(raw),
        "truncated": truncated[0],
        "nodes": nodes[0],
        "target": str(target),
        "note": None,
    }

# ----------------------------- 行星级配齿搜索 -----------------------------

# 六种角色配置下的输出/输入传动比：
#   n_out/n_in，f = zR/zS（齿数比）
#   r 固定、c→s：1+f   s 固定、c→r：1+1/f
#   r 固定、s→c：1/(1+f)   s 固定、r→c：f/(1+f)
#   c 固定、s→r：-1/f   c 固定、r→s：-f
PLANET_CONFIGS = [
    {"fixed": "r", "input": "c", "output": "s", "key": "rcs"},
    {"fixed": "r", "input": "s", "output": "c", "key": "rsc"},
    {"fixed": "s", "input": "c", "output": "r", "key": "scr"},
    {"fixed": "s", "input": "r", "output": "c", "key": "src"},
    {"fixed": "c", "input": "s", "output": "r", "key": "csr"},
    {"fixed": "c", "input": "r", "output": "s", "key": "crs"},
]


def _planet_config_ratio(cfg, zs, zr):
    f = Fraction(zr, zs)
    key = cfg["key"]
    if key == "rcs":
        return 1 + f
    if key == "rsc":
        return 1 / (1 + f)
    if key == "scr":
        return 1 + 1 / f
    if key == "src":
        return f / (1 + f)
    if key == "csr":
        return -1 / f
    return -f  # crs


def search_planets(params: dict) -> dict:
    """枚举满足目标传动比、齿数范围、行星数与外径上限的可装配行星级。"""
    target_s = str(params.get("target", "")).strip().replace(" ", "")
    tol = float(params.get("tolerancePct", 1.0))
    zmin = int(params.get("zMin", 12))
    zmax = int(params.get("zMax", 120))
    modules = sorted({float(x) for x in
                      str(params.get("modules", "1")).replace("，", ",").split(",")
                      if x.strip()})
    counts = sorted({int(x) for x in
                     str(params.get("counts", "3")).replace("，", ",").split(",")
                     if x.strip() and 2 <= int(x) <= 6} or {3})
    max_outer = float(params.get("maxOuter", 1e9))
    clearance = float(params.get("minClearance", 0.0))  # 相邻行星顶圆最小净距 mm
    result_limit = int(params.get("limit", 200))
    deadline = time.time() + float(params.get("timeBudget", 8.0))
    magnitude_only = not target_s.startswith("-")

    target = Fraction(target_s)
    tol_frac = Fraction(str(tol)) / 100
    tol_lo, tol_hi = 1 - tol_frac, 1 + tol_frac
    _at = abs(target)

    results = []
    nodes = 0
    truncated = False
    # 以 zS、zP、n、配置 为变量；zR 由 z圈=z太阳+2z行星 决定，再校验装配/外径/净距
    for mdl in modules:
        for cfg in PLANET_CONFIGS:
            for zs in range(max(zmin, 1), zmax + 1):
                for zp in range(max(zmin, 1), zmax + 1):
                    zr = zs + 2 * zp
                    if zr > zmax:
                        break
                    nodes += 1
                    if nodes % 4096 == 0 and time.time() > deadline:
                        truncated = True
                        break
                    rr = _planet_config_ratio(cfg, zs, zr)
                    if magnitude_only and (rr < 0) != (target < 0) and not (
                            target > 0 and rr > 0):
                        pass
                    mag = abs(rr)
                    err = (abs(mag - _at) / _at) if magnitude_only \
                        else abs(rr - target) / abs(target)
                    if not magnitude_only and (rr < 0) != (target < 0):
                        continue
                    if err > tol_frac:
                        continue
                    outer = mdl * (zr + 2.5)
                    if outer > max_outer:
                        continue
                    orbit = mdl * (zs + zp) / 2.0
                    for n in counts:
                        if (zs + zr) % n:
                            continue
                        chord = 2 * orbit * math.sin(math.pi / n)
                        net = chord - mdl * (zp + 2)
                        if net < clearance:
                            continue
                        # 复位循环：以输入成员转 1 转为基准，求 zS·n_s、zR·n_r、
                        # zP·n_p（行星绝对自转）同时为整数，且均布行星轴位恢复
                        # （n·n_c 为整数）所需的最小输入转数
                        speeds = {}
                        speeds[cfg["input"]] = Fraction(1)
                        speeds[cfg["fixed"]] = Fraction(0)
                        speeds[cfg["output"]] = rr
                        spin = speeds["c"] - Fraction(zs, zp) * (speeds["s"] - speeds["c"])
                        L = 1
                        for z, mem in ((zs, "s"), (zr, "r")):
                            L = lcm(L, (z * speeds[mem]).denominator)
                        L = lcm(L, (zp * spin).denominator)
                        L = lcm(L, (n * speeds["c"]).denominator)
                        results.append({
                            "module": mdl, "zS": zs, "zP": zp, "zR": zr,
                            "count": n,
                            "fixed": cfg["fixed"], "input": cfg["input"],
                            "output": cfg["output"],
                            "ratio": fj(rr), "errorPct": round(float(err) * 100, 5),
                            "outerD": round(outer, 3),
                            "orbitR": round(orbit, 3),
                            "netGap": round(net, 3),
                            "cycle": str(L), "cycleV": float(L),
                            "maxZ": zr,
                        })
                if truncated:
                    break
            if truncated:
                break
        if truncated:
            break

    results.sort(key=lambda c: (c["errorPct"], c["outerD"], c["cycleV"], c["maxZ"]))
    return {
        "results": results[:result_limit],
        "totalMatched": len(results),
        "truncated": truncated,
        "nodes": nodes,
        "target": str(target),
        "note": None,
    }
