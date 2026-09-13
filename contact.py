# -*- coding: utf-8 -*-
"""齿对接触周期与整齿装配相位分析。

数据约定（字段随齿轮对象保存在项目 state 里）：
  zeroDeg      零号齿方向（度）：θ=0（轴未转）时，0 号齿中心线在固定坐标系中的方向角
  offset       整齿装配偏移（齿距数，整数）：带键基准到实际装入位置的整齿转动，
               齿 i 在固定系中的中心线角 = zeroDeg + 360°·(i + offset)/z + θ轴
  phaseLocked  带键齿轮：装配相位锁定，枚举候选时偏移不变
  watched      关注齿序号列表（0 基）
  marks        {齿序号: "wear"|"chip"|"repair"} 磨损/崩角/修补标记

啮合事件模型（顺着传动方向）：
  以输入轴为根沿普通齿轮啮合图 BFS，深度小的一侧齿轮为主动；节点（两节圆切点）
  在固定系中不动。主动轴正向（逆时针）转过一个齿距时，原来落后一个齿距的齿进入
  节点，故事件 k=0,1,… 时
    主动齿 = (i0A − k) mod zA
    从动齿 = 外啮合 (i0B + k) mod zB（从动轴反转）
            内啮合 (i0B − k) mod zB（从动轴同向）
  若主动轴相对输入反转，整体乘 s = sign(nA)。
  节点方向：外啮合节点在两心连线上，主动看从动为 φ、从动看主动为 φ+180；
  内啮合两节圆内切，从任一轮中心看节点都在 φ+180 方向。
  事件间隔 = 主动轮一个齿距，输入轴转角 = 360°·k / (zA·|nA|)。
  因而外啮合长期只在 iA+iB ≡ c (mod gcd(zA,zB)) 的齿对子集上循环，
  内啮合为 iB−iA ≡ c，可到达齿对数 = zA·zB/gcd。
"""
from __future__ import annotations

import math
import time
from fractions import Fraction
from math import gcd

import kinematics

PERIOD_CAP = 60_000      # 单啮合展开周期事件数上限（超过则不生成矩阵）
EVENT_CAP = 1_000_000    # 观察窗口事件总数上限
MARK_LIST_CAP = 400      # 单啮合返回的标记/关注相遇条目上限
CANDIDATE_LIMIT = 120
MARK_KINDS = ("wear", "chip", "repair")
MARK_LABEL = {"wear": "磨损", "chip": "崩角", "repair": "修补"}


# ----------------------------- 工具 -----------------------------

def _fj(f: Fraction) -> dict:
    return kinematics.fj(f)


def _speed(speeds: dict, sid) -> Fraction | None:
    """kinematics.analyze 返回的 speeds 是 {s: "p/q", v: float}。"""
    v = speeds.get(sid)
    if not v:
        return None
    try:
        return Fraction(v["s"])
    except (ValueError, KeyError):
        return None


def _gear_int(g, key, default=0):
    v = g.get(key, default)
    return v if isinstance(v, int) else default


def _watched(g) -> list[int]:
    z = g.get("z")
    out = []
    seen = set()
    if isinstance(z, int):
        for t in g.get("watched", []) or []:
            if isinstance(t, int) and 0 <= t < z and t not in seen:
                seen.add(t)
                out.append(t)
    return out


def _marks(g) -> dict[int, str]:
    z = g.get("z")
    out = {}
    if isinstance(z, int):
        for t, k in (g.get("marks") or {}).items():
            try:
                ti = int(t)
            except (TypeError, ValueError):
                continue
            if 0 <= ti < z and k in MARK_KINDS:
                out[ti] = k
    return out


def _mod1(x: float) -> float:
    return x - math.floor(x)


def _gear_int_offset(v) -> int:
    return v if isinstance(v, int) else 0


def _i0(phi_deg: float, zero_deg, offset: int, z: int) -> int:
    """节点方向 phiDeg 处，当前最靠近节点的齿序号。

    齿 i 的固定方向角 = zero + 360·(i + offset)/z；令其等于 phi：
    i ≈ (phi − zero)·z/360 − offset，就近取整后模 z（直接对整值取模，
    不能先取小数部分，否则负整数齿号会被错误折回 0）。
    """
    x = (phi_deg - (zero_deg or 0)) * z / 360.0 - (offset or 0)
    return int(math.floor(x + 0.5)) % z


def _pair_index(k, i0a, i0b, za, zb, internal, s=1):
    ia = (i0a - s * k) % za
    ib = (i0b - s * k) % zb if internal else (i0b + s * k) % zb
    return ia, ib


def _crt2(r1: int, m1: int, r2: int, m2: int):
    """解 k ≡ r1 (mod m1)，k ≡ r2 (mod m2) 的最小非负解；无解返回 None。"""
    r1 %= m1
    r2 %= m2
    g = gcd(m1, m2)
    if (r2 - r1) % g:
        return None
    if m1 == 1 and m2 == 1:
        return 0
    a, b, c = m1 // g, m2 // g, (r2 - r1) // g
    if b == 1:
        t0 = 0
    else:
        t0 = (c * pow(a % b, -1, b)) % b
    period = m1 // g * m2
    return (r1 + m1 * t0) % period


# ----------------------------- 啮合上下文 -----------------------------

def _build_contexts(state: dict, an: dict):
    """返回 (issues, contexts)：只含动力链内、齿数/转速有效的普通齿轮啮合。

    每个 context 含主动/从动判定（按 BFS 深度）、节点方向、初始齿号、转速等。
    """
    issues = []
    shafts = {s["id"]: s for s in state.get("shafts", [])}
    gears = {g["id"]: g for g in state.get("gears", [])}
    speeds = an.get("speeds", {})
    input_id = state.get("inputId")

    valid_meshes = []
    adj: dict = {sid: [] for sid in shafts}
    for e in state.get("meshes", []):
        ga, gb = gears.get(e.get("gearA")), gears.get(e.get("gearB"))
        if not ga or not gb:
            continue
        sa, sb = ga.get("shaftId"), gb.get("shaftId")
        if sa not in shafts or sb not in shafts or sa == sb:
            continue
        if not (isinstance(ga.get("z"), int) and isinstance(gb.get("z"), int)
                and ga["z"] > 0 and gb["z"] > 0):
            continue
        if ga.get("internal") and gb.get("internal"):
            continue
        internal = bool(ga.get("internal") or gb.get("internal"))
        if internal:
            zr = ga["z"] if ga.get("internal") else gb["z"]
            zp = gb["z"] if ga.get("internal") else ga["z"]
            if zr <= zp:
                continue
        valid_meshes.append((e, ga, gb, sa, sb, internal))
        adj[sa].append(sb)
        adj[sb].append(sa)

    # 以输入轴为根的 BFS 深度（用于沿传动方向选主动轮与事件排序）
    depth = {}
    if input_id in adj:
        depth[input_id] = 0
        q = [input_id]
        head = 0
        while head < len(q):
            cur = q[head]
            head += 1
            for nb in adj[cur]:
                if nb not in depth:
                    depth[nb] = depth[cur] + 1
                    q.append(nb)

    contexts = []
    for order, (e, ga, gb, sa, sb, internal) in enumerate(valid_meshes):
        na, nb = _speed(speeds, sa), _speed(speeds, sb)
        if na is None or nb is None or na == 0:
            issues.append({"mesh": e.get("id"),
                           "message": "啮合不在输入轴动力链内或主动轴静止，未参与齿对展开"})
            continue
        # 深度小者为主动；同级（闭合边）按啮合表顺序取 gearA 侧。
        # 注意：主动/从动按传动方向判定，与啮合边的 gearA/gearB 存储顺序无关。
        da, db = depth.get(sa, 1 << 30), depth.get(sb, 1 << 30)
        if da < db or (da == db and da < (1 << 30)):
            drv, dnv, sd, sn, n_shaft_signed = ga, gb, sa, sb, na
        elif db < da:
            drv, dnv, sd, sn, n_shaft_signed = gb, ga, sb, sa, nb
        else:
            drv, dnv, sd, sn, n_shaft_signed = ga, gb, sa, sb, na
        n_shaft = abs(n_shaft_signed)
        s_sign = 1 if n_shaft_signed > 0 else -1
        Pd, Pn = shafts[sd], shafts[sn]
        phi = math.degrees(math.atan2(Pn["y"] - Pd["y"], Pn["x"] - Pd["x"]))
        za, zb = drv["z"], dnv["z"]
        offa = _gear_int_offset(drv.get("offset"))
        offb = _gear_int_offset(dnv.get("offset"))
        # 节点方向（两轮中心指向切点的固定方向角）：
        # 外啮合切点在两心连线上：主动侧 phi、从动侧 phi+180；
        # 内啮合两节圆内切，从两轮中心看切点同向——齿圈驱动时为 phi，
        # 小齿轮驱动时为 phi+180。
        if internal:
            node_a = node_b = phi + (0.0 if drv.get("internal") else 180.0)
        else:
            node_a, node_b = phi, phi + 180.0
        i0a = _i0(node_a, drv.get("zeroDeg", 0), offa, za)
        i0b = _i0(node_b, dnv.get("zeroDeg", 0), offb, zb)
        contexts.append({
            "mesh": e, "ga": ga, "gb": gb, "drv": drv, "dnv": dnv,
            "internal": internal, "depth": min(depth.get(sd, order), depth.get(sn, order)),
            "order": order,
            "phi": phi, "nodeA": node_a, "nodeB": node_b,
            "za": za, "zb": zb,
            "shaftRate": n_shaft, "shaftSign": s_sign,
            "rate": za * n_shaft,
            "i0a": i0a, "i0b": i0b, "offa": offa, "offb": offb,
        })
    contexts.sort(key=lambda c: (c["depth"], c["order"]))
    return issues, contexts


# ----------------------------- 齿对接触分析 -----------------------------

def analyze_teeth(state: dict, observe_turns: float | None = None) -> dict:
    """逐啮合展开齿对，返回周期、可达子集、相遇次数、接触矩阵与关注齿相遇角。"""
    an = kinematics.analyze(state)
    issues = list(an.get("issues", []))
    if observe_turns is None:
        observe_turns = state.get("observeTurns", 2)
    try:
        T = Fraction(str(float(observe_turns)))
        if T <= 0:
            raise ValueError
    except (ValueError, ZeroDivisionError):
        T = Fraction(2)

    ctx_issues, contexts = _build_contexts(state, an)
    for it in ctx_issues:
        issues.append({"severity": "info", "code": "TOOTH_SKIP",
                       "message": it["message"], "refs": {"mesh": it["mesh"]}})

    out_meshes = []
    total_pairs_all = 0
    total_reach_all = 0
    truncated = False

    for c in contexts:
        za, zb, g = c["za"], c["zb"], gcd(c["za"], c["zb"])
        P = za // g * zb                    # 每对齿的重复周期（事件数）
        total_pairs = za * zb
        reach = P
        total_pairs_all += total_pairs
        total_reach_all += reach
        rate = c["rate"]
        s_sign = c["shaftSign"]
        period_turns = Fraction(P) / rate
        N = int(T * rate)                  # 观察窗口内事件数 k=0..N-1
        cap_hit = False
        if N > EVENT_CAP:
            N = EVENT_CAP
            cap_hit = truncated = True

        watch_a, watch_b = _watched(c["drv"]), _watched(c["dnv"])
        marks_a, marks_b = _marks(c["drv"]), _marks(c["dnv"])
        watch_set_a, watch_set_b = set(watch_a), set(watch_b)

        cells = []
        watch_meetings = []
        marked_meetings = []
        period_ok = P <= PERIOD_CAP
        if not period_ok:
            truncated = True
        else:
            base, rem = divmod(N, P)
            seen_pairs = 0
            for k in range(P):
                ia, ib = _pair_index(k, c["i0a"], c["i0b"], za, zb,
                                     c["internal"], s_sign)
                seen_pairs += 1
                if k < N:
                    cnt = base + (1 if k < rem else 0)
                    if cnt > 0:
                        cells.append([ia, ib, cnt, round(360.0 * k / float(rate), 6)])
                if watch_set_a and watch_set_b and ia in watch_set_a and ib in watch_set_b \
                        and k < N and len(watch_meetings) < MARK_LIST_CAP:
                    watch_meetings.append(_meeting(c, ia, ib, k, N, P, marks_a, marks_b))
                if (ia in marks_a or ib in marks_b) and k < N \
                        and len(marked_meetings) < MARK_LIST_CAP:
                    marked_meetings.append(_meeting(c, ia, ib, k, N, P, marks_a, marks_b))

        drv, dnv = c["drv"], c["dnv"]
        def _gname(gg, zz):
            return gg.get("name") or (
                "内齿圈 z%s" % zz if gg.get("internal") else "齿轮 z%s" % zz)
        out_meshes.append({
            "meshId": c["mesh"].get("id"),
            # 一律按传动方向：A=主动、B=从动（与啮合边存储顺序无关）
            "gearA": drv["id"], "gearB": dnv["id"],
            "storedGearA": c["ga"]["id"], "storedGearB": c["gb"]["id"],
            "nameA": _gname(drv, za), "nameB": _gname(dnv, zb),
            "driverGear": drv["id"], "drivenGear": dnv["id"],
            "internal": c["internal"], "depth": c["depth"],
            "zA": za, "zB": zb,
            "rate": _fj(rate), "shaftRate": _fj(c["shaftRate"]),
            "shaftSign": s_sign,
            "phiDeg": round(c["phi"], 6),
            "nodeA": round(c["nodeA"], 6), "nodeB": round(c["nodeB"], 6),
            "zeroA": _gear_int(drv, "zeroDeg", 0) or 0,
            "zeroB": _gear_int(dnv, "zeroDeg", 0) or 0,
            "offA": c["offa"], "offB": c["offb"],
            "i0A": c["i0a"], "i0B": c["i0b"],
            "events": N,
            "totalPairs": total_pairs, "reachPairs": reach,
            "huntGroups": g, "subsetOnly": g > 1,
            "periodEvents": P,
            "periodInputTurns": _fj(period_turns),
            "residue": int(((c["i0b"] - c["i0a"]) if c["internal"]
                            else (c["i0a"] + c["i0b"])) % g) if g else 0,
            "matrix": cells if period_ok else [],
            "matrixTruncated": not period_ok,
            "watchMeetings": watch_meetings,
            "markedMeetings": marked_meetings,
        })

    return {
        "ok": not any(i.get("severity") == "error" for i in issues),
        "issues": issues,
        "observeTurns": float(T),
        "meshes": out_meshes,
        "totalPairs": total_pairs_all,
        "reachPairs": total_reach_all,
        "eventCap": EVENT_CAP,
        "truncated": truncated,
        "watchTeeth": {g["id"]: _watched(g) for g in state.get("gears", []) if _watched(g)},
        "marks": {g["id"]: {str(t): k for t, k in _marks(g).items()}
                  for g in state.get("gears", []) if _marks(g)},
    }


def _meeting(c, ia, ib, k0, N, P, marks_a, marks_b):
    rate = c["rate"]
    count = 1 + (N - 1 - k0) // P if k0 < N else 0
    return {
        "ia": ia, "ib": ib,
        "gearA": c["drv"]["id"], "gearB": c["dnv"]["id"],
        "firstEvent": k0,
        "firstDeg": round(360.0 * k0 / float(rate), 6),
        "firstTurns": _fj(Fraction(k0) / rate if rate else Fraction(0)),
        "count": count,
        "markA": marks_a.get(ia), "markB": marks_b.get(ib),
    }


# ----------------------------- 装配相位枚举 -----------------------------

def _mesh_score(c, za, zb, i0a, i0b, watch_a, watch_b, N):
    """给定初始齿号后，统计观察窗内关注齿相遇次数与最早相遇输入转数（None=不相遇）。

    事件 k：ia = i0a − k (mod za)；外啮合 ib = i0b + k (mod zb)，
    内啮合 ib = i0b − k (mod zb)。s=±1 不改变 gcd 同余类，故无需单列。
    """
    if not watch_a or not watch_b or N <= 0:
        return 0, None
    g = gcd(za, zb)
    P = za // g * zb
    count = 0
    earliest = None
    rate = c["rate"]
    internal = c["internal"]
    for ia in watch_a:
        # k ≡ i0a − ia (mod za)
        r1 = (i0a - ia) % za
        for ib in watch_b:
            # 外啮合 k ≡ ib − i0b (mod zb)；内啮合 k ≡ i0b − ib (mod zb)
            r2 = ((ib - i0b) if not internal else (i0b - ib)) % zb
            k0 = _crt2(r1, za, r2, zb)
            if k0 is None or k0 >= N:
                continue
            count += 1 + (N - 1 - k0) // P
            turns0 = Fraction(k0) / rate
            if earliest is None or turns0 < earliest:
                earliest = turns0
    return count, earliest


def enumerate_assembly(state: dict, offset_range: int = 3,
                       observe_turns: float | None = None,
                       time_budget: float = 6.0,
                       node_cap: int = 300_000) -> dict:
    """锁定带键齿轮相位，枚举其余可达齿轮的整齿装配偏移候选。

    排序：关注齿窗内相遇次数少 → 首次相遇晚（不相遇最优）→ 相位改动小。
    """
    an = kinematics.analyze(state)
    if observe_turns is None:
        observe_turns = state.get("observeTurns", 2)
    T = Fraction(str(float(observe_turns)))
    _, contexts = _build_contexts(state, an)
    if not contexts:
        return {"candidates": [], "nodes": 0, "truncated": False,
                "note": "动力链内没有可展开的普通齿轮啮合。"}

    offset_range = max(0, min(12, int(offset_range)))

    # 枚举变量：动力链内出现、未“带键锁定”的齿轮
    gears = {g["id"]: g for g in state.get("gears", [])}
    used = []
    seen = set()
    for c in contexts:
        for role in ("drv", "dnv"):
            gid = c[role]["id"]
            if gid not in seen:
                seen.add(gid)
                used.append(c[role])
    locked = {g["id"]: _gear_int_offset(g.get("offset"))
              for g in used if g.get("phaseLocked")}
    variables = [g for g in used if not g.get("phaseLocked")]
    if not variables:
        return {"candidates": [], "nodes": 0, "truncated": False,
                "note": "动力链内齿轮均已「带键锁定」，没有可枚举的装配相位。"}

    domains = {}
    for g in variables:
        cur = _gear_int_offset(g.get("offset"))
        domains[g["id"]] = list(range(cur - offset_range, cur + offset_range + 1))

    watch = {g["id"]: _watched(g) for g in used}

    def i0_with(c, role, offsets):
        g = c[role]
        gid = g["id"]
        off = offsets.get(gid, _gear_int_offset(g.get("offset")))
        phi = c["nodeA"] if role == "drv" else c["nodeB"]
        return _i0(phi, g.get("zeroDeg", 0), off, g["z"])

    N_by_mesh = {id(c): int(T * c["rate"]) for c in contexts}

    deadline = time.time() + time_budget
    nodes = [0]
    truncated = False
    results = []
    result_keys = set()

    # 当前装配先入列，作为排序基线
    current_offsets = {g["id"]: _gear_int_offset(g.get("offset")) for g in used}

    def evaluate(offsets):
        """返回 (关注齿窗内相遇总次数, 全局最早相遇输入转数 Fraction 或 None)。"""
        total_count = 0
        earliest_global = None
        for c in contexts:
            i0a = i0_with(c, "drv", offsets)
            i0b = i0_with(c, "dnv", offsets)
            cnt, turns0 = _mesh_score(c, c["za"], c["zb"], i0a, i0b,
                                      watch.get(c["drv"]["id"], []),
                                      watch.get(c["dnv"]["id"], []),
                                      N_by_mesh[id(c)])
            total_count += cnt
            if turns0 is not None and (earliest_global is None or turns0 < earliest_global):
                earliest_global = turns0
        return total_count, earliest_global

    def record(offsets):
        cnt, turns0 = evaluate(offsets)
        change = sum(abs(offsets[g["id"]] - _gear_int_offset(g.get("offset")))
                     for g in variables)
        key = tuple(offsets[g["id"]] for g in variables)
        if key in result_keys:
            return
        result_keys.add(key)
        results.append({
            # 仅回传相对当前装配的整齿增量，供前端叠加
            "deltas": {g["id"]: offsets[g["id"]] - _gear_int_offset(g.get("offset"))
                       for g in variables if offsets[g["id"]] != _gear_int_offset(g.get("offset"))},
            "watchMeetings": cnt,
            "firstTurns": None if turns0 is None else _fj(turns0),
            "firstDeg": None if turns0 is None else round(360.0 * float(turns0), 6),
            "firstSort": float("inf") if turns0 is None else float(turns0),
            "neverMeet": turns0 is None,
            "phaseChange": change,
            "isCurrent": all(offsets[g["id"]] == _gear_int_offset(g.get("offset"))
                             for g in variables),
            "names": {g["id"]: g.get("name") or ("齿轮 z%s" % g.get("z")) for g in used},
        })

    record(dict(current_offsets))

    order = list(variables)
    assigned = dict(locked)

    def dfs(idx):
        nonlocal truncated
        if nodes[0] > node_cap or time.time() > deadline:
            truncated = True
            return
        if idx == len(order):
            record(dict(assigned))
            return
        g = order[idx]
        for off in domains[g["id"]]:
            nodes[0] += 1
            assigned[g["id"]] = off
            # 关注齿相遇数要在全部啮合确定后才能汇总（同一根轴上的齿轮会
            # 同时影响多处啮合），不能做部分和剪枝；节点/时间预算足以兜底。
            dfs(idx + 1)
            if truncated:
                break
        assigned.pop(g["id"], None)

    dfs(0)

    INF_SORT = 10 ** 12
    results.sort(key=lambda r: (
        r["watchMeetings"],
        -(r["firstSort"] if r["firstSort"] < INF_SORT else INF_SORT),
        r["phaseChange"],
        tuple(sorted(r["deltas"].items())),
    ))
    for r in results:
        r.pop("firstSort", None)
    # 剪枝按当前最优相遇数，结果可能超量，截断即可
    limited = results[:CANDIDATE_LIMIT]
    return {
        "candidates": limited,
        "totalMatched": len(results),
        "nodes": nodes[0],
        "truncated": truncated,
        "offsetRange": offset_range,
        "observeTurns": float(T),
        "locked": [{"id": gid, "offset": off,
                    "name": gears[gid].get("name") or ("齿轮 z%s" % gears[gid].get("z"))}
                   for gid, off in locked.items()],
        "variables": [{"id": g["id"], "name": g.get("name") or ("齿轮 z%s" % g.get("z")),
                       "z": g.get("z"), "offset": _gear_int_offset(g.get("offset"))}
                      for g in variables],
        "note": None,
    }
