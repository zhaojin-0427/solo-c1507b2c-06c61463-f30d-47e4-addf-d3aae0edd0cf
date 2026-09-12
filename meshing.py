# -*- coding: utf-8 -*-
"""渐开线圆柱齿轮啮合校核与成对变位搜索。

齿形参数（随项目保存在齿轮对象上，旧项目缺省即标准齿形）：
  ha  齿顶高系数（标准 1.0）
  c   顶隙系数（标准 0.25）
  x   变位系数（标准 0）

check_pair(gearA, gearB, center, eps_min) 根据**实际中心距**计算：
  工作压力角 αw、啮合线起止点（B2 啮入 / B1 啮出）、重合度 εα、
  两端滑动率，并定位根切、齿顶干涉、重合度不足与几何条件无解。
返回的 view 字段为放大视图预计算几何：小轮（内啮合时为外齿轮）中心在原点、
另一轮中心在 +x 轴上，作用线过节点 P、方向 d，线上点 Q(s) = P + s·d，
s ∈ [sStart, sEnd] 为实际啮合段，接触点随转角沿该线移动。

search_shifts(params) 在锁定约束下枚举成对变位 (x1, x2) 与可调整中心距，
按 违规数 → 重合度余量 → 中心距改动 → 变位总量 排序。
"""
from __future__ import annotations

import math
import time

DEFAULT_HA = 1.0      # 标准齿顶高系数
DEFAULT_C = 0.25      # 标准顶隙系数
EPS_MIN_DEFAULT = 1.2  # 重合度下限默认值
SLIDING_WARN = 3.0     # 滑动率告警阈值
BACKLASH_TOL = 0.005   # 变位和偏差在此范围内视为无侧隙


def inv(a: float) -> float:
    """渐开线函数 inv(a) = tan(a) − a（a 为弧度）。"""
    return math.tan(a) - a


def solve_inv(target: float, guess: float = 0.35) -> float:
    """由渐开线函数值反解角度（牛顿迭代）。"""
    a = min(1.4, max(1e-4, guess))
    for _ in range(40):
        t = math.tan(a)
        f = t - a - target
        d = t * t
        if d < 1e-12:
            break
        na = a - f / d
        if na <= 1e-6:
            na = a * 0.5
        elif na >= 1.5:
            na = (a + 1.5) * 0.5
        a = na
        if abs(f) < 1e-13:
            break
    return a


def _num(v, default):
    return float(v) if isinstance(v, (int, float)) else float(default)


def gear_profile(g: dict) -> dict:
    """单齿轮齿形轮廓参数；缺省字段按标准齿形。参数非法抛 ValueError。"""
    z = g.get("z")
    m = g.get("module")
    if not isinstance(z, int) or z <= 0:
        raise ValueError("齿数必须为正整数")
    if not isinstance(m, (int, float)) or m <= 0:
        raise ValueError("模数必须为正数")
    m = float(m)
    alpha = math.radians(_num(g.get("pressureAngle"), 20.0) or 20.0)
    if not (1.0 < math.degrees(alpha) < 45.0):
        raise ValueError("压力角异常")
    ha = _num(g.get("ha"), DEFAULT_HA)
    c = _num(g.get("c"), DEFAULT_C)
    x = _num(g.get("x"), 0.0)
    internal = bool(g.get("internal"))
    r = m * z / 2.0
    rb = r * math.cos(alpha)
    if internal:
        ra = m * (z / 2.0 - ha + x)        # 内齿圈齿顶圆（内边界）
        rf = m * (z / 2.0 + ha + c + x)
        x_min = None                       # 内齿圈不按齿条刀具根切校核
    else:
        ra = m * (z / 2.0 + ha + x)
        rf = m * (z / 2.0 - ha - c + x)
        x_min = ha - z * math.sin(alpha) ** 2 / 2.0   # 最少变位（根切界限）
    return {"z": z, "m": m, "alpha": alpha, "ha": ha, "c": c, "x": x,
            "internal": internal, "r": r, "rb": rb, "ra": ra, "rf": rf,
            "xMin": x_min}


def _shift_terms(p1: dict, p2: dict):
    """返回 (internal, zΣ, xΣ)。内啮合时 p1 须为外齿轮、p2 为内齿圈。"""
    if p1["internal"] or p2["internal"]:
        zsum = p2["z"] - p1["z"]
        xsum = p2["x"] - p1["x"]
        return True, zsum, xsum
    return False, p1["z"] + p2["z"], p1["x"] + p2["x"]


def _shifted_center(ga: dict, gb: dict):
    """变位和对应的无侧隙理论中心距；不可解（inv αw ≤ 0）返回 None。"""
    try:
        p1, p2 = gear_profile(ga), gear_profile(gb)
    except ValueError:
        return None
    if p1["internal"] and p2["internal"]:
        return None
    if p1["internal"]:
        p1, p2 = p2, p1
    internal, zsum, xsum = _shift_terms(p1, p2)
    if zsum <= 0:
        return None
    alpha = p1["alpha"]
    a0 = p1["m"] * zsum / 2.0
    inv_w = inv(alpha) + 2.0 * math.tan(alpha) * xsum / zsum
    if inv_w <= 1e-12:
        return None
    aw = solve_inv(inv_w, alpha)
    return a0 * math.cos(alpha) / math.cos(aw)


def expected_center(ga: dict, gb: dict):
    """考虑变位的理论中心距（供中心距诊断）；变位和不可解时退回标准中心距。"""
    a = _shifted_center(ga, gb)
    if a is not None:
        return a
    try:
        p1, p2 = gear_profile(ga), gear_profile(gb)
    except ValueError:
        return None
    if p1["internal"] and p2["internal"]:
        return None
    if p1["internal"]:
        p1, p2 = p2, p1
    zsum = (p2["z"] - p1["z"]) if (p1["internal"] or p2["internal"]) else p1["z"] + p2["z"]
    if zsum <= 0:
        return None
    return p1["m"] * zsum / 2.0


def _r6(v):
    return None if v is None else round(v, 6)


def check_pair(ga: dict, gb: dict, center: float, eps_min: float = EPS_MIN_DEFAULT) -> dict:
    """按实际中心距校核一对渐开线齿轮啮合。gearA/gearB 为齿轮对象（可缺省齿形参数）。"""
    issues = []

    def add(sev, code, msg):
        issues.append({"severity": sev, "code": code, "message": msg})

    def fail(code, msg):
        add("error", code, msg)
        return {"ok": False, "issues": issues, "type": None, "map": None,
                "params": None, "geom": None, "view": None}

    try:
        pa, pb = gear_profile(ga), gear_profile(gb)
    except ValueError as exc:
        return fail("BAD_GEAR", str(exc))
    if pa["internal"] and pb["internal"]:
        return fail("TWO_RINGS", "两个内齿圈不能互相啮合")
    # 统一 p1 = 几何首轮（内啮合时为外齿轮/小齿轮，p2 为内齿圈）
    swapped = False
    p1, p2 = pa, pb
    if p1["internal"]:
        p1, p2 = p2, p1
        swapped = True
    gmap = {"g1": "B" if swapped else "A", "g2": "A" if swapped else "B"}
    internal = p2["internal"]
    m = p1["m"]
    if abs(p1["m"] - p2["m"]) > 1e-9:
        return fail("MODULE_MISMATCH",
                    "模数不合：m=%s 与 m=%s 无法啮合" % (p1["m"], p2["m"]))
    if abs(p1["alpha"] - p2["alpha"]) > 1e-6:
        add("warning", "ALPHA_MISMATCH",
            "压力角不一致：%.4g° 与 %.4g°，校核按 %.4g° 计算"
            % (math.degrees(p1["alpha"]), math.degrees(p2["alpha"]), math.degrees(p1["alpha"])))
    alpha = p1["alpha"]
    z1, z2 = p1["z"], p2["z"]
    x1, x2 = p1["x"], p2["x"]
    if internal and z2 <= z1:
        return fail("RING_TOO_SMALL",
                    "内啮合要求内齿圈齿数大于小齿轮：z圈 %s 必须大于 z轮 %s" % (z2, z1))
    _, zsum, xsum = _shift_terms(p1, p2)
    a0 = m * zsum / 2.0
    a = float(center)
    if not (a > 0):
        return fail("GEOM_IMPOSSIBLE", "中心距必须为正数")

    tan_a = math.tan(alpha)
    inv_a = inv(alpha)
    # 当前变位和对应的无侧隙中心距
    inv_w_req = inv_a + 2.0 * tan_a * xsum / zsum
    a_req = None
    alpha_w_req = None
    if inv_w_req <= 1e-12:
        add("warning", "SHIFT_IMPOSSIBLE",
            "变位和 xΣ=%.3f 使无侧隙啮合角非正，几何条件无解（变位和过小）" % xsum)
    else:
        alpha_w_req = solve_inv(inv_w_req, alpha)
        a_req = a0 * math.cos(alpha) / math.cos(alpha_w_req)

    # 由实际中心距求工作压力角
    cos_w = a0 * math.cos(alpha) / a
    alpha_w = None
    if cos_w > 1.0 + 1e-9:
        add("error", "GEOM_IMPOSSIBLE",
            "实际中心距 %.3f mm 小于可装配下限 %.3f mm（基圆连心线长），几何条件无解"
            % (a, a0 * math.cos(alpha)))
    else:
        alpha_w = 0.0 if cos_w >= 1.0 else math.acos(cos_w)

    geom = {"a0": _r6(a0), "aActual": _r6(a), "aReq": _r6(a_req),
            "alphaW": None, "alphaWReq": None if alpha_w_req is None else _r6(math.degrees(alpha_w_req)),
            "xSum": _r6(xsum), "xSumNeeded": None, "backlashMm": None,
            "r1": _r6(p1["r"]), "r2": _r6(p2["r"]),
            "rb1": _r6(p1["rb"]), "rb2": _r6(p2["rb"]),
            "ra1": _r6(p1["ra"]), "ra2": _r6(p2["ra"]),
            "rf1": _r6(p1["rf"]), "rf2": _r6(p2["rf"]),
            "r1w": None, "r2w": None,
            "pe": _r6(math.pi * m * math.cos(alpha)),
            "gAlpha": None, "epsAlpha": None,
            "sStart": None, "sEnd": None, "gN1": None, "gN2": None,
            "u1": None, "u2": None, "t12": None,
            "xMin1": _r6(p1["xMin"]), "xMin2": _r6(p2["xMin"]),
            "eta1Start": None, "eta1End": None,
            "eta2Start": None, "eta2End": None,
            "eta1Max": None, "eta2Max": None}
    params = {"m": m, "alphaDeg": _r6(math.degrees(alpha)),
              "z1": z1, "z2": z2, "x1": _r6(x1), "x2": _r6(x2),
              "ha1": _r6(p1["ha"]), "ha2": _r6(p2["ha"]),
              "c1": _r6(p1["c"]), "c2": _r6(p2["c"])}
    base = {"issues": issues, "type": "internal" if internal else "external",
            "map": gmap, "params": params}

    # 根切（仅外齿轮；内齿圈不按齿条刀具根切校核）
    for idx, p in ((1, p1), (2, p2)):
        if p["xMin"] is not None and p["x"] < p["xMin"] - 1e-9:
            z_min = 2.0 * p["ha"] / math.sin(alpha) ** 2
            add("error", "UNDERCUT",
                "根切：z%s 轮 x=%.3f 低于根切界限 x_min=%.3f（标准齿形最少齿数约 %.1f）"
                % (p["z"], p["x"], p["xMin"], z_min))

    if alpha_w is None:
        return dict(base, ok=False, geom=geom, view=None)

    geom["alphaW"] = _r6(math.degrees(alpha_w))
    # 实际中心距下无侧隙所需变位和 → 侧隙状态
    xsum_need = zsum * (inv(alpha_w) - inv_a) / (2.0 * tan_a)
    dxs = xsum_need - xsum
    jt = 2.0 * m * tan_a * dxs
    geom["xSumNeeded"] = _r6(xsum_need)
    geom["backlashMm"] = _r6(jt)
    if dxs > BACKLASH_TOL:
        add("info", "BACKLASH_LOOSE",
            "当前变位和小于实际中心距所需：存在圆周侧隙约 %.3f mm（所需 xΣ=%.3f，当前 %.3f）"
            % (jt, xsum_need, xsum))
    elif dxs < -BACKLASH_TOL:
        add("warning", "BACKLASH_TIGHT",
            "当前变位和大于实际中心距所需：负侧隙约 %.3f mm，装配过盈易卡死（所需 xΣ=%.3f，当前 %.3f）"
            % (-jt, xsum_need, xsum))

    # 齿顶圆与基圆关系
    u1 = u2 = None
    if p1["ra"] < p1["rb"] - 1e-9:
        add("error", "TIP_BELOW_BASE",
            "z%s 轮齿顶圆 %.3f 低于基圆 %.3f（变位过小或齿顶高系数异常），几何条件无解"
            % (z1, p1["ra"], p1["rb"]))
    else:
        u1 = math.sqrt(max(0.0, p1["ra"] ** 2 - p1["rb"] ** 2))
    if p2["ra"] < p2["rb"] - 1e-9:
        if internal:
            add("warning", "RING_TIP_LOW",
                "内齿圈齿顶圆 %.3f 低于基圆 %.3f，有效渐开线不足，按基圆起算"
                % (p2["ra"], p2["rb"]))
            u2 = 0.0
        else:
            add("error", "TIP_BELOW_BASE",
                "z%s 轮齿顶圆 %.3f 低于基圆 %.3f，几何条件无解" % (z2, p2["ra"], p2["rb"]))
    else:
        u2 = math.sqrt(max(0.0, p2["ra"] ** 2 - p2["rb"] ** 2))
    if u1 is None or u2 is None:
        return dict(base, ok=False, geom=geom, view=None)

    cos_w = math.cos(alpha_w)
    sin_w = math.sin(alpha_w)
    tan_w = sin_w / cos_w if cos_w > 1e-12 else 1e12
    r1w = p1["rb"] / cos_w
    r2w = p2["rb"] / cos_w
    t12 = a * sin_w                      # 两切点 N1、N2 间距
    g_n1 = p1["rb"] * tan_w              # N1 在作用线上相对 P 的坐标（外啮合取负）
    g_n2 = p2["rb"] * tan_w
    if internal:
        # B1（小齿轮齿顶，啮出）在 P 的负侧，B2（内齿圈齿顶，啮入）在正侧
        s_start = g_n1 - u1
        s_end = g_n2 - u2
        n1_pos, n2_pos = g_n1, g_n2      # N1、N2 在 P 同侧
    else:
        s_start = g_n2 - u2              # B2：齿轮2齿顶进入啮合
        s_end = u1 - g_n1                # B1：齿轮1齿顶退出啮合
        n1_pos, n2_pos = -g_n1, g_n2     # N1、N2 分居 P 两侧
    g_alpha = s_end - s_start
    geom.update({"r1w": _r6(r1w), "r2w": _r6(r2w),
                 "sStart": _r6(s_start), "sEnd": _r6(s_end),
                 "gN1": _r6(n1_pos), "gN2": _r6(n2_pos),
                 "u1": _r6(u1), "u2": _r6(u2), "t12": _r6(t12),
                 "gAlpha": _r6(g_alpha)})

    # 齿顶干涉
    if internal:
        # 内啮合：两齿顶圆相交时，校核交点处的角度条件（齿先干涉公式）
        ra1, ra2 = p1["ra"], p2["ra"]
        rb1, rb2 = p1["rb"], p2["rb"]
        if ra1 > 1e-9 and ra2 > 1e-9 and abs(ra1 - ra2) < a < ra1 + ra2:
            th1 = math.acos(max(-1.0, min(1.0, (ra1 ** 2 + a ** 2 - ra2 ** 2) / (2 * ra1 * a))))
            th2 = math.acos(max(-1.0, min(1.0, (ra2 ** 2 + a ** 2 - ra1 ** 2) / (2 * ra2 * a))))
            # 齿顶圆低于基圆时按基圆起算（αa 钳位为 0）
            aa1 = math.acos(max(-1.0, min(1.0, rb1 / ra1)))
            aa2 = math.acos(max(-1.0, min(1.0, rb2 / ra2)))
            lhs = th1 + inv(alpha_w) - inv(aa1)
            rhs = (z2 / z1) * (th2 + inv(alpha_w) - inv(aa2))
            if lhs < rhs - 1e-9:
                add("error", "TIP_INTERFERENCE",
                    "齿顶干涉：小齿轮齿顶与内齿圈齿顶相碰（判据 %.4f < %.4f），"
                    "多因齿数差过小或齿顶过高" % (lhs, rhs))
    else:
        # 外啮合：啮合起止点不得越过对方基圆切点（渐开线干涉）
        if u2 > t12 + 1e-9:
            add("error", "TIP_INTERFERENCE",
                "齿顶干涉：z%s 轮齿顶越过对方基圆切点（齿顶展长 %.3f > 切点距 %.3f），"
                "将挖入对方齿根过渡曲线" % (z2, u2, t12))
        if u1 > t12 + 1e-9:
            add("error", "TIP_INTERFERENCE",
                "齿顶干涉：z%s 轮齿顶越过对方基圆切点（齿顶展长 %.3f > 切点距 %.3f），"
                "将挖入对方齿根过渡曲线" % (z1, u1, t12))

    # 重合度
    pe = math.pi * m * math.cos(alpha)
    eps = g_alpha / pe if g_alpha > 0 else None
    if eps is None or eps <= 1e-9:
        add("error", "NO_CONTACT",
            "啮合线长度为 %.3f mm ≤ 0：齿顶圆不足以形成啮合区间，几何条件无解" % g_alpha)
    else:
        geom["epsAlpha"] = _r6(eps)
        if eps < 1.0:
            add("error", "LOW_CONTACT_RATIO",
                "重合度 εα=%.3f < 1，传动不连续（脱啮）" % eps)
        elif eps < eps_min:
            add("warning", "LOW_CONTACT_RATIO",
                "重合度不足：εα=%.3f 低于下限 %.2f" % (eps, eps_min))

    # 滑动率（接触点在作用线上坐标 g 处）
    def eta(gpos):
        n1k = abs(gpos - n1_pos)
        n2k = abs(gpos - n2_pos)
        e1 = None if n1k < 1e-9 else 1.0 - (z1 / z2) * (n2k / n1k)
        e2 = None if n2k < 1e-9 else 1.0 - (z2 / z1) * (n1k / n2k)
        return e1, e2

    if eps is not None and eps > 0:
        e1s, e2s = eta(s_start)
        e1e, e2e = eta(s_end)
        geom.update({"eta1Start": _r6(e1s), "eta1End": _r6(e1e),
                     "eta2Start": _r6(e2s), "eta2End": _r6(e2e)})
        cands1 = [v for v in (e1s, e1e) if v is not None]
        cands2 = [v for v in (e2s, e2e) if v is not None]
        if cands1:
            geom["eta1Max"] = _r6(max(cands1, key=abs))
        if cands2:
            geom["eta2Max"] = _r6(max(cands2, key=abs))
        emax = max([abs(v) for v in cands1 + cands2], default=0.0)
        if emax > SLIDING_WARN:
            add("warning", "SLIDING_HIGH",
                "滑动率过大：|η|max=%.2f 超过 %.1f，齿面磨损加剧" % (emax, SLIDING_WARN))

    # 放大视图几何：p1 中心在原点，p2 中心在 +x 轴
    if internal:
        P = (-r1w, 0.0)
        n_hat = (-cos_w, sin_w)
    else:
        P = (r1w, 0.0)
        n_hat = (cos_w, -sin_w)
    d = (sin_w, cos_w)
    N1 = (p1["rb"] * n_hat[0], p1["rb"] * n_hat[1])
    N2 = (a + (p2["rb"] * n_hat[0] if internal else -p2["rb"] * n_hat[0]),
          (p2["rb"] * n_hat[1] if internal else -p2["rb"] * n_hat[1]))

    def q(s):
        return [_r6(P[0] + s * d[0]), _r6(P[1] + s * d[1])]

    view = {"O1": [0.0, 0.0], "O2": [_r6(a), 0.0],
            "P": [_r6(P[0]), _r6(P[1])], "d": [_r6(d[0]), _r6(d[1])],
            "N1": [_r6(N1[0]), _r6(N1[1])], "N2": [_r6(N2[0]), _r6(N2[1])],
            "Bstart": q(s_start), "Bend": q(s_end),
            "r1": geom["r1"], "r2": geom["r2"],
            "rb1": geom["rb1"], "rb2": geom["rb2"],
            "ra1": geom["ra1"], "ra2": geom["ra2"],
            "rf1": geom["rf1"], "rf2": geom["rf2"],
            "r1w": _r6(r1w), "r2w": _r6(r2w),
            "pe": geom["pe"], "sStart": geom["sStart"], "sEnd": geom["sEnd"],
            "gN1": geom["gN1"], "gN2": geom["gN2"],
            "rbDrive": geom["rb1"]}
    ok = not any(i["severity"] == "error" for i in issues)
    return dict(base, ok=ok, geom=geom, view=view)


# ----------------------------- 成对变位搜索 -----------------------------

def search_shifts(params: dict) -> dict:
    """枚举成对变位 (x1, x2)；中心距未锁定时按变位和反算无侧隙中心距并限幅。

    排序：违规数 → 重合度余量(降) → 中心距改动 → 变位总量。
    """
    ga = params.get("gearA", {})
    gb = params.get("gearB", {})
    center = float(params.get("center", 0))
    eps_min = float(params.get("epsMin", EPS_MIN_DEFAULT))
    x_min = float(params.get("xMin", -0.6))
    x_max = float(params.get("xMax", 1.0))
    x_step = float(params.get("xStep", 0.05)) or 0.05
    da_max = float(params.get("daMax", 0.8))
    lock_a = bool(params.get("lockA"))
    lock_b = bool(params.get("lockB"))
    lock_center = bool(params.get("lockCenter"))
    limit = int(params.get("limit", 60))
    deadline = time.time() + float(params.get("timeBudget", 6.0))

    def grid(cur, locked):
        if locked:
            return [round(_num(cur, 0.0), 4)]
        n = max(0, int(round((x_max - x_min) / x_step)))
        return [round(x_min + i * x_step, 4) for i in range(n + 1)]

    xs_a = grid(ga.get("x"), lock_a)
    xs_b = grid(gb.get("x"), lock_b)

    results = []
    seen = set()
    nodes = 0
    truncated = False
    for xa in xs_a:
        for xb in xs_b:
            nodes += 1
            if nodes % 512 == 0 and time.time() > deadline:
                truncated = True
                break
            g1 = dict(ga, x=xa)
            g2 = dict(gb, x=xb)
            a = center
            if not lock_center:
                a_req = _shifted_center(g1, g2)
                if a_req is None:
                    continue
                if abs(a_req - center) > da_max + 1e-9:
                    continue
                a = a_req
            chk = check_pair(g1, g2, a, eps_min)
            viol = sum(1 for i in chk["issues"] if i["severity"] == "error")
            viol += sum(1 for i in chk["issues"] if i["code"] == "BACKLASH_TIGHT")
            gm = chk.get("geom") or {}
            eps = gm.get("epsAlpha")
            margin = (eps - eps_min) if eps is not None else None
            key = (round(xa, 4), round(xb, 4), round(a, 4))
            if key in seen:
                continue
            seen.add(key)
            results.append({
                "xA": round(xa, 4), "xB": round(xb, 4),
                "a": round(a, 4), "da": round(a - center, 4),
                "eps": eps, "margin": _r6(margin) if margin is not None else None,
                "violations": viol,
                "issues": [i["message"] for i in chk["issues"]
                           if i["severity"] == "error" or i["code"] == "BACKLASH_TIGHT"][:3],
                "alphaW": gm.get("alphaW"),
                "eta1Max": gm.get("eta1Max"), "eta2Max": gm.get("eta2Max"),
                "view": chk.get("view"),
            })
        if truncated:
            break

    results.sort(key=lambda c: (
        c["violations"],
        -(c["margin"] if c["margin"] is not None else -1e9),
        abs(c["da"]),
        abs(c["xA"]) + abs(c["xB"])))
    return {"results": results[:limit],
            "totalMatched": len(results),
            "truncated": truncated,
            "nodes": nodes,
            "note": None}
