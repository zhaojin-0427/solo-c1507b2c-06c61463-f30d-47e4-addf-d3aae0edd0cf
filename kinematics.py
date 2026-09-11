# -*- coding: utf-8 -*-
"""轮系运动学核心：转速传播、约束诊断、复位循环、配齿搜索。

状态 JSON 结构：
{
  "shafts": [{"id","name","x","y","locked"}],
  "gears":  [{"id","shaftId","name","z","module","pressureAngle","internal","locked"}],
  "meshes": [{"id","gearA","gearB"}],          # 外/内啮合由齿轮 internal 标志推断
  "coaxRelations": [{"id","shaftA","shaftB"}], # 同轴（同位、各自转速独立）
  "inputId","outputId","inputRpm"
}
长度单位均为 mm；转速为相对输入轴的分数（输入轴 = 1 转）。
"""
from __future__ import annotations

import math
import time
from collections import defaultdict, deque
from fractions import Fraction
from math import gcd, hypot

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

    # --- 转速传播（BFS，分数精确） ---
    input_id, output_id = state.get("inputId"), state.get("outputId")
    speeds: dict = {}
    conflicts: set = set()

    if not input_id or input_id not in shafts:
        add("info", "NO_INPUT", "尚未指定输入轴（在轴属性中设置）")
    else:
        speeds[input_id] = Fraction(1, 1)
        q = deque([input_id])
        while q:
            cur = q.popleft()
            for nb, fact, eid in adj[cur]:
                v = speeds[cur] * fact
                if nb in speeds:
                    if speeds[nb] != v and eid not in conflicts:
                        conflicts.add(eid)
                        same_sign = (speeds[nb] > 0) == (v > 0)
                        code = "RATIO_CONFLICT" if same_sign else "DIRECTION_CONFLICT"
                        if same_sign:
                            msg = "传动比矛盾：轴「%s」经两条路径推得 %s 与 %s" % (
                                _sname(shafts[nb]), speeds[nb], v)
                        else:
                            msg = "转向矛盾：轴「%s」经两条路径转向相反（%s 与 %s），闭合轮系齿数不满足约束" % (
                                _sname(shafts[nb]), speeds[nb], v)
                        add("error", code, msg, mesh=eid, shaft=nb)
                else:
                    speeds[nb] = v
                    q.append(nb)

    for sid, s in shafts.items():
        if sid not in speeds and any(g.get("shaftId") == sid for g in gears.values()):
            add("warning", "IDLE_SHAFT", "轴「%s」未连入输入轴的动力链" % _sname(s), shaft=sid)
    if output_id and output_id not in speeds:
        add("warning", "NO_OUTPUT_PATH", "输出轴无法从输入轴到达")
    if not output_id or output_id not in shafts:
        add("info", "NO_OUTPUT", "尚未指定输出轴（在轴属性中设置）")

    # --- 总传动比 ---
    ratio = speeds.get(output_id) if output_id else None
    input_rpm = float(state.get("inputRpm", 1) or 1)
    rpms = {sid: input_rpm * float(v) for sid, v in speeds.items()}

    # --- 整列复位循环：每只齿轮走过的齿距数 z·n·L 必须为整数 ---
    denoms = []
    for gid, g in gears.items():
        sid = g.get("shaftId")
        if sid in speeds and isinstance(g.get("z"), int):
            denoms.append((g["z"] * speeds[sid]).denominator)
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

    def complete(assign, sp_out):
        """assign: {gid:(m,z)}；sp_out: 各轴最终分数转速。"""
        changed = {}
        light_gears = []
        for gid, (m, z) in assign.items():
            g0 = gears[gid]
            if not g0.get("locked") and (
                    int(g0["z"]) != z or abs(float(g0["module"]) - float(m)) > 1e-12):
                changed[gid] = (int(g0["z"]), float(g0["module"]))
            light_gears.append({"id": gid, "shaftId": g0["shaftId"], "z": z,
                                "module": m, "internal": bool(g0.get("internal"))})
        train_set = set(train_gears)
        light_gears += [g for g in state.get("gears", []) if g["id"] not in train_set]
        light_state = {"shafts": list(shafts.values()), "gears": light_gears,
                       "meshes": state.get("meshes", [])}
        positions, max_res, max_move = place_shafts(light_state, locked_pos)

        L = 1
        for gid, (m, z) in assign.items():
            sid = gears[gid]["shaftId"]
            if sid in sp_out:
                L = lcm(L, (Fraction(z) * sp_out[sid]).denominator)

        replaced = []
        for gid, (z0, m0) in changed.items():
            z1, m1 = assign[gid][1], assign[gid][0]
            replaced.append({"id": gid, "name": _gname(gears[gid]),
                             "z0": z0, "m0": m0, "z1": z1, "m1": m1,
                             "internal": bool(gears[gid].get("internal"))})
        rr = sp_out[output_id]
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

    def propagate(gid, assign, sp):
        """齿轮 gid 齿数刚确定（assign 已更新），推进所有与之相关且对方已定的边。
        返回 (ok, new_speeds)，new_speeds 供回溯。"""
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
                sign = 1 if internal else -1
                candidates = [
                    (sa, sb, Fraction(za, zb)),              # v_b = sign*za/zb*v_a
                    (sb, sa, Fraction(zb, za)),              # v_a = sign*zb/za*v_b
                ]
                for drive, driven, frac in candidates:
                    if drive not in sp:
                        continue
                    v = sp[drive] * (Fraction(sign, 1) * frac)
                    if driven in sp:
                        if sp[driven] != v:
                            return False, touched
                    else:
                        sp[driven] = v
                        touched.add(driven)
                        for other in shaft_gears[driven]:
                            if other not in (a, b) and \
                                    assign.get(other, (None, None))[1] is not None:
                                stack.append(other)
        return True, touched

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
            ok, touched = propagate(gid, assign, sp)
            if ok:
                # 输出轴速度一旦算出即可按容差剪枝（之后齿数不再经过输出路径）
                out_v = sp.get(output_id)
                if out_v is not None:
                    r = abs(out_v) if magnitude_only else out_v
                    t = abs(target) if magnitude_only else target
                    if abs(r - t) <= t * (tol / 100.0):
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
            # 先从锁定齿轮做一轮初始传播
            sp0 = {input_id: Fraction(1, 1)}
            touched_all = set()
            for gid in sorted(train_gears, key=lambda x: shaft_level.get(gears[x]["shaftId"], 99)):
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
