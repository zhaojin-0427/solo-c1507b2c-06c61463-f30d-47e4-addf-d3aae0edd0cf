# -*- coding: utf-8 -*-
"""扭转振动工况：以**一次转速扫描**为独立版本（草稿 → 已采用流转）。

建立草稿时从当前轮系与一份已存载荷工况（或当前载荷草稿）**冻结动力路径、
转速比与平均转矩**（``freeze``）；随后为各轴填转动惯量，为啮合、联轴器与
负载端填扭转刚度/阻尼，并设置输入转速范围、扫速步长与驱动端/负载端激励阶次。
草稿/版本不随源轮系或源载荷自动重算，只按来源指纹标记过期。

力学模型（集中参数扭振系统，全部角度为物理弧度，惯量/刚度可在各轴本地录入，
组装时统一折算到参考轴＝冻结来源的输入轴，r_i = n_i/n_输入）：
  节点：每根动力路径上的轴一个惯量节点；驱动端（电机）、负载端（输出叶轮/
        工作台）与飞轮为联轴器节点（飞轮采用后直接并入所在轴惯量）。
  边：
    - 啮合边：刚度/阻尼按主动轴侧录入；广义坐标下 k_g=k/r主²，
      从动坐标带方向号 σ（外啮合 −1、内啮合 +1）。
    - 联轴器边：同轴 r 相同、σ=+1，k_g=k/r²。
  方程：J q̈ + C q̇ + K q = Q（谐和激励 T_a·cos(ωt)，ω=h·2πn节点/60）。
  特征：广义对称特征值 Kφ=ωn²Jφ（Cholesky + Jacobi），刚体模态 ωn=0；
        模态阻尼比 ζ=φᵀCφ/(2ωn)（φ 按 φᵀJφ=1 归一）。
  频响：每个转速样本、每个激励阶次复高斯消元求 Θ；边动态转矩幅
        Tdyn=|k+iωc|·|Δφ物理|，多阶次按平方和开根合成；
        放大系数 = (平均转矩 + Tdyn)/平均转矩。

规格 spec：
{
  "primarySource": 冻结来源 key,
  "shafts":   {shaftId: {"inertia": kg·m²（该轴本地）, "locked": bool}},
  "meshes":   {meshId:  {"stiffness": N·m/rad（主动侧）, "damping": N·m·s/rad,
                        "locked": bool}},
  "couplings":[{ "id", "kind": "driver"/"load", "shaftId",
                 "inertia", "stiffness", "damping", "locked" }],
  "addedFlywheels": [{"shaftId", "inertia"}],   # 已采用的飞轮（展示用，惯量已并入轴）
  "scan":     {"rpmMin", "rpmMax", "rpmStep"},
  "excitation": {"nodeId": 联轴器节点 id, "orders": [{"h": 阶次, "amp": N·m}]},
  "limits":   {"zetaMin": 模态阻尼比下限, "torqueFactor": 动态转矩限（×平均）}
}
"""
from __future__ import annotations

import json
import math
import time
from collections import defaultdict, deque
from fractions import Fraction

import loadcase
import thermal

# ----------------------------- 常量 -----------------------------

STEEL_RHO = 7850.0
TOOTH_K = 2.0e4          # 轮齿综合刚度系数（N/mm 齿宽 / μm 变形折算后），见 default_mesh_k
FACEWIDTH_PER_M = 10.0   # 默认齿宽 b = 10·模数 mm（仅用于默认刚度/惯量估算，可改）
MESH_ZETA = 0.04         # 默认啮合阻尼比
COUPLING_ZETA = 0.05     # 默认联轴器阻尼比
MAX_SAMPLES = 240
SEARCH_NODE_CAP = 60_000


def _num(v, d=0.0):
    try:
        x = float(v)
        return x if math.isfinite(x) else float(d)
    except (TypeError, ValueError):
        return float(d)


def _pos(v, d):
    x = _num(v, d)
    return x if x > 0 else float(d)


def _r(v, n=4):
    return None if v is None else round(float(v), n)


# ----------------------------- 来源指纹（与前端 tvFingerprint 一致） -----------------------------

def canonical_dumps(obj) -> str:
    return thermal.canonical_dumps(obj)


def train_fingerprint(st: dict) -> str:
    """源轮系指纹：齿轮 z/m/x/压力角/内齿标记 + 轴位（复用热平衡同一规范）。"""
    return thermal.fingerprint(st)


def load_spec_fingerprint(spec: dict) -> str:
    return canonical_dumps(spec or {})


# ----------------------------- 默认值估算 -----------------------------

def _gear_blank_inertia(g: dict) -> float:
    """钢齿坯近似 J=½ρπbr⁴（kg·m²），齿宽 b=10m、节圆半径 r=mz/2。"""
    m = float(g.get("module") or 0)
    z = g.get("z")
    if m <= 0 or not isinstance(z, int) or z <= 0:
        return 0.0
    r = m * z / 2000.0
    b = FACEWIDTH_PER_M * m / 1000.0
    return 0.5 * STEEL_RHO * math.pi * b * r ** 4


def default_mesh_k(z_driver: int, module: float) -> float:
    """主动侧扭转刚度 kθ=K·r²（N·m/rad）：K=2e4·b N/mm，r 为主动轮节圆半径 mm。"""
    b = FACEWIDTH_PER_M * module
    r = module * z_driver / 2.0
    return TOOTH_K * b * r * r / 1000.0


# ----------------------------- 冻结载荷来源 -----------------------------

def freeze_sources(sources: list) -> dict:
    """sources: [{"key","name","state","spec","caseId"?,"version"?,"inline"?}]
    扭振一次只对应一份平均转矩工况；仍按 sources 映射返回以便与其它工况同构。"""
    frozen = {"sources": {}, "order": []}
    for src in sources:
        st, sp = src.get("state", {}), src.get("spec", {})
        key = str(src.get("key"))
        entry = {
            "key": key, "name": src.get("name", key),
            "caseId": src.get("caseId"), "version": src.get("version"),
            "inline": bool(src.get("inline")),
            "snapshot": st, "loadSpec": sp,
            "trainFp": train_fingerprint(st),
            "loadSpecFp": load_spec_fingerprint(sp),
        }
        try:
            res = loadcase.analyze_case(st, sp)
            tbl = loadcase.mesh_table(st)
            input_id = sp.get("inputId")
            ratio = loadcase.speed_ratios(st, tbl, input_id)
            issues = []
            oriented, depth = loadcase._orient(st, tbl, ratio, input_id, issues)
            shaft_objs = {s["id"]: s for s in st.get("shafts", [])}
            downstream = defaultdict(list)
            for mid, o in oriented.items():
                downstream[o["driver"]].append((mid, o["driven"]))
            terminals = [sid for sid in depth if not downstream.get(sid)]
            # 各轴齿坯惯量估算（默认值用）
            gear_j = defaultdict(float)
            for g in st.get("gears", []):
                gear_j[g.get("shaftId")] += _gear_blank_inertia(g)
            shafts = []
            for sid, frac in ratio.items():
                s = shaft_objs.get(sid, {})
                rr = float(frac)
                shafts.append({
                    "id": sid, "name": loadcase._sname(s),
                    "ratio": rr, "depth": depth.get(sid, 0),
                    "rpm": _r(abs(rr) * _num(sp.get("inputRpm"), 0.0), 3),
                    "torqueNm": (res.get("shafts", {}).get(sid) or {}).get("torqueNm", 0.0),
                    "gearInertia": _r(gear_j.get(sid, 0.0), 8),
                })
            meshes = []
            for mid, o in oriented.items():
                info = tbl[mid]
                mr = res.get("meshes", {}).get(mid, {})
                meshes.append({
                    "id": mid, "name": loadcase.mesh_label(info, o),
                    "driver": o["driver"], "driven": o["driven"],
                    "zDriver": info["zA"] if o["gearDriver"] == info["gearA"] else info["zB"],
                    "zDriven": o["zDriven"], "module": info["m"],
                    "internal": info["internal"], "depth": o["depth"],
                    "torqueNm": mr.get("torqueNm", 0.0),
                    "speedDriver": mr.get("speedDriver", 0.0),
                    "speedDriven": mr.get("speedDriven", 0.0),
                })
            entry.update({
                "ok": res.get("ok", False), "issues": res.get("issues", []),
                "inputId": input_id,
                "rpm0": _num(sp.get("inputRpm"), 0.0),
                "torqueInNm": (res.get("input") or {}).get("torqueNm", 0.0),
                "ratios": {sid: float(f) for sid, f in ratio.items()},
                "shafts": sorted(shafts, key=lambda x: (x["depth"], str(x["id"]))),
                "meshes": sorted(meshes, key=lambda x: (x["depth"], str(x["id"]))),
                "terminals": sorted(terminals, key=str),
                "shaftNames": {sid: loadcase._sname(s) for sid, s in shaft_objs.items()},
            })
        except Exception as exc:  # noqa: BLE001
            entry["ok"] = False
            entry["error"] = str(exc)
        frozen["sources"][key] = entry
        frozen["order"].append(key)
    return frozen


# ----------------------------- 默认规格 -----------------------------

def default_spec(frozen: dict) -> dict:
    """冻结后生成一份可改的默认 spec：惯量/刚度按齿坯几何估算，联轴器自动布置。"""
    key = (frozen.get("order") or [""])[0]
    src = (frozen.get("sources") or {}).get(key, {})
    ratios = src.get("ratios", {})
    shafts_in = src.get("shafts", [])
    meshes_in = src.get("meshes", [])

    gear_j = {s["id"]: max(1e-8, _num(s.get("gearInertia"), 1e-8)) for s in shafts_in}
    shafts = {}
    for s in shafts_in:
        j = max(1e-8, _num(s.get("gearInertia"), 1e-8))
        shafts[s["id"]] = {"inertia": _r(j, 8), "locked": False}

    mesh_defs = {}
    for m in meshes_in:
        k = max(1.0, default_mesh_k(m["zDriver"], m["module"]))
        j_d = gear_j.get(m["driver"], 1e-6)
        c = 2.0 * MESH_ZETA * math.sqrt(max(1e-12, j_d * k))
        mesh_defs[m["id"]] = {"stiffness": _r(k, 3), "damping": _r(c, 6),
                             "locked": False}

    # 每根轴最近的啮合刚度（联轴器默认取其 1/4，呈“软联轴器”）
    nearest_k = defaultdict(list)
    for m in meshes_in:
        nearest_k[m["driver"]].append(mesh_defs[m["id"]]["stiffness"])
        nearest_k[m["driven"]].append(mesh_defs[m["id"]]["stiffness"])
    couplings = []

    def coupling(cid, kind, sid, j):
        ks = nearest_k.get(sid) or [1000.0]
        k = max(1.0, 0.25 * (min(ks) if ks else 1000.0))
        c = 2.0 * COUPLING_ZETA * math.sqrt(max(1e-12, j * k))
        return {"id": cid, "kind": kind, "shaftId": sid,
                "inertia": _r(j, 8), "stiffness": _r(k, 3),
                "damping": _r(c, 6), "locked": False}

    input_id = src.get("inputId")
    if input_id in shafts:
        j_in = gear_j.get(input_id, 1e-6)
        couplings.append(coupling("c:driver", "driver", input_id,
                                  max(20.0 * j_in, 5e-4)))
    for k_i, sid in enumerate(src.get("terminals", [])):
        if sid in shafts and sid != input_id:
            j_t = gear_j.get(sid, 1e-6)
            couplings.append(coupling("c:load%d" % k_i, "load", sid,
                                      max(10.0 * j_t, 2e-4)))

    rpm0 = max(1.0, _num(src.get("rpm0"), 60.0))
    lo = max(50.0, round(rpm0 * 0.25 / 10) * 10)
    hi = round(rpm0 * 2.5 / 10) * 10
    step = max(1.0, round((hi - lo) / 120))
    t_in = max(1.0, _num(src.get("torqueInNm"), 1.0))
    return {
        "primarySource": key,
        "shafts": shafts, "meshes": mesh_defs, "couplings": couplings,
        "addedFlywheels": [],
        "scan": {"rpmMin": lo, "rpmMax": hi, "rpmStep": step},
        "excitation": {"nodeId": "c:driver",
                       "orders": [{"h": 1.0, "amp": _r(0.1 * t_in, 4)}]},
        "limits": {"zetaMin": 0.03, "torqueFactor": 1.5},
    }


def _merge_spec(spec: dict, frozen: dict) -> dict:
    """用默认 spec 补缺，不覆盖已录入值；保留刷新冻结后仍存在的轴/啮合/联轴器。"""
    base = default_spec(frozen)
    s = json.loads(json.dumps(base))
    if not isinstance(spec, dict):
        return s
    key = spec.get("primarySource") or s["primarySource"]
    src = (frozen.get("sources") or {}).get(key) or \
        (frozen.get("sources") or {}).get(s["primarySource"]) or {}
    s["primarySource"] = src.get("key", s["primarySource"])

    valid_shafts = {x["id"] for x in src.get("shafts", [])}
    valid_meshes = {x["id"] for x in src.get("meshes", [])}
    given_s = spec.get("shafts", {}) or {}
    s["shafts"] = {sid: {
        "inertia": max(1e-12, _pos(given_s.get(sid, {}).get("inertia"),
                                   s["shafts"][sid]["inertia"])),
        "locked": bool((given_s.get(sid) or {}).get("locked")),
    } for sid in s["shafts"] if sid in valid_shafts}
    given_m = spec.get("meshes", {}) or {}
    s["meshes"] = {mid: {
        "stiffness": max(1e-9, _pos(given_m.get(mid, {}).get("stiffness"),
                                    s["meshes"][mid]["stiffness"])),
        "damping": max(0.0, _num((given_m.get(mid) or {}).get("damping"),
                                 s["meshes"][mid]["damping"])),
        "locked": bool((given_m.get(mid) or {}).get("locked")),
    } for mid in s["meshes"] if mid in valid_meshes}

    base_c = {c["id"]: c for c in s["couplings"]}
    kept, seen_kinds = [], set()
    for c in spec.get("couplings", []) or []:
        cid = str(c.get("id") or "")
        b = base_c.get(cid)
        if not b or c.get("shaftId") not in valid_shafts:
            continue
        kept.append({
            "id": cid, "kind": b["kind"], "shaftId": c.get("shaftId"),
            "inertia": max(1e-12, _pos(c.get("inertia"), b["inertia"])),
            "stiffness": max(1e-9, _pos(c.get("stiffness"), b["stiffness"])),
            "damping": max(0.0, _num(c.get("damping"), b["damping"])),
            "locked": bool(c.get("locked")),
        })
        seen_kinds.add((b["kind"], c.get("shaftId")))
    for cid, b in base_c.items():
        if (b["kind"], b["shaftId"]) not in seen_kinds and \
                not any(x["id"] == cid for x in kept):
            kept.append(dict(b))
    s["couplings"] = sorted(kept, key=lambda c: (c["kind"] != "driver", c["id"]))

    # 已采用飞轮：刷新后轴仍在则保留（惯量已并入 shafts.inertia，这里仅记录展示）
    s["addedFlywheels"] = [dict(f) for f in (spec.get("addedFlywheels") or [])
                           if f.get("shaftId") in valid_shafts]

    scan_in = spec.get("scan", {}) or {}
    lo = _pos(scan_in.get("rpmMin"), s["scan"]["rpmMin"])
    hi = _pos(scan_in.get("rpmMax"), s["scan"]["rpmMax"])
    if hi <= lo:
        lo, hi = s["scan"]["rpmMin"], s["scan"]["rpmMax"]
    step = max(1e-6, _pos(scan_in.get("rpmStep"), s["scan"]["rpmStep"]))
    s["scan"] = {"rpmMin": lo, "rpmMax": hi, "rpmStep": step}

    ex_in = spec.get("excitation", {}) or {}
    cids = {c["id"] for c in s["couplings"]}
    default_ex = next((c["id"] for c in s["couplings"] if c["kind"] == "driver"),
                      s["couplings"][0]["id"] if s["couplings"] else None)
    node_id = ex_in.get("nodeId") if ex_in.get("nodeId") in cids else default_ex
    orders = []
    for o in ex_in.get("orders", []) or []:
        h = _num(o.get("h"), 0.0)
        if h > 0:
            orders.append({"h": h, "amp": max(0.0, _num(o.get("amp"), 0.0))})
    if not orders:
        orders = list(s["excitation"]["orders"])
    s["excitation"] = {"nodeId": node_id, "orders": orders}

    lim = spec.get("limits", {}) or {}
    s["limits"] = {
        "zetaMin": max(0.0, _num(lim.get("zetaMin"), s["limits"]["zetaMin"])),
        "torqueFactor": max(1.0, _num(lim.get("torqueFactor"),
                                      s["limits"]["torqueFactor"])),
    }
    return s


# ----------------------------- 纯 Python 线性代数 -----------------------------

def _cholesky(M):
    """对称正定 M 的 Cholesky 下三角 L（M=L Lᵀ）。"""
    n = len(M)
    L = [[0.0] * n for _ in range(n)]
    for i in range(n):
        for j in range(i + 1):
            v = M[i][j] - sum(L[i][k] * L[j][k] for k in range(j))
            if i == j:
                L[i][j] = math.sqrt(max(1e-30, v))
            else:
                L[i][j] = v / L[j][j]
    return L


def _fwd_solve(L, b):
    """解 Lx=b（L 下三角）。"""
    n = len(L)
    x = [0.0] * n
    for i in range(n):
        x[i] = (b[i] - sum(L[i][k] * x[k] for k in range(i))) / L[i][i]
    return x


def _jacobi(A, tol=1e-11, max_sweeps=80):
    """实对称矩阵 A 的循环 Jacobi 特征分解，返回 (升序特征值, 特征向量列)。"""
    n = len(A)
    V = [[1.0 if i == j else 0.0 for j in range(n)] for i in range(n)]
    B = [row[:] for row in A]
    for _ in range(max_sweeps):
        off = sum(B[i][j] ** 2 for i in range(n) for j in range(n) if i != j)
        if off < tol:
            break
        for p in range(n - 1):
            for q in range(p + 1, n):
                apq = B[p][q]
                if abs(apq) < 1e-14:
                    continue
                tau = (B[q][q] - B[p][p]) / (2.0 * apq)
                t = (1.0 if tau >= 0 else -1.0) / \
                    (abs(tau) + math.sqrt(1.0 + tau * tau))
                c = 1.0 / math.sqrt(1.0 + t * t)
                s = t * c
                for k in range(n):
                    akp, akq = B[k][p], B[k][q]
                    B[k][p] = c * akp - s * akq
                    B[k][q] = s * akp + c * akq
                for k in range(n):
                    apk, aqp = B[p][k], B[q][k]
                    B[p][k] = c * apk - s * aqp
                    B[q][k] = s * apk + c * aqp
                for k in range(n):
                    vkp, vkq = V[k][p], V[k][q]
                    V[k][p] = c * vkp - s * vkq
                    V[k][q] = s * vkp + c * vkq
    eig = sorted((B[i][i], [V[k][i] for k in range(n)]) for i in range(n))
    return [e[0] for e in eig], [e[1] for e in eig]


def _eigen_modes(M, K):
    """广义对称特征值 Kφ=ω²Mφ。返回 [{omega, hz, zetaPhi(phi)} 占位, phi]，
    特征向量按 φᵀMφ=1 归一；刚体/病态模态 ω 钳为 0。"""
    n = len(M)
    L = _cholesky(M)
    Linv = [_fwd_solve(L, [1.0 if i == j else 0.0 for j in range(n)])
            for i in range(n)]   # 行：L⁻¹ 的行（前向代入单位列）
    # A = L⁻¹ K L⁻ᵀ；Linv_rows[i][k] = (L⁻¹)[i][k]
    A = [[0.0] * n for _ in range(n)]
    for i in range(n):
        for j in range(n):
            acc = 0.0
            for a in range(n):
                Lia = Linv[i][a]
                if Lia == 0:
                    continue
                for b in range(n):
                    if K[a][b] and Linv[j][b]:
                        acc += Lia * K[a][b] * Linv[j][b]
            A[i][j] = acc
    vals, vecs = _jacobi(A)
    modes = []
    for lam, v in zip(vals, vecs):
        # 原问题特征向量 φ = L⁻ᵀ v：φ_k = Σ_i (L⁻¹)[i][k] v_i
        phi = [0.0] * n
        for k in range(n):
            phi[k] = sum(Linv[i][k] * v[i] for i in range(n))
        mu = sum(M[i][j] * phi[i] * phi[j] for i in range(n) for j in range(n))
        if mu > 0:
            scale = 1.0 / math.sqrt(mu)
            phi = [x * scale for x in phi]
        omega = math.sqrt(lam) if lam > 1e-8 else 0.0
        modes.append({"omega": omega, "hz": omega / (2.0 * math.pi), "phi": phi})
    modes.sort(key=lambda m: m["omega"])
    return modes


def _solve_complex(A, b):
    """复矩阵高斯消元（部分主元），n 很小。"""
    n = len(A)
    M = [row[:] for row in A]
    y = b[:]
    for k in range(n):
        piv = max(range(k, n), key=lambda r: abs(M[r][k]))
        if abs(M[piv][k]) < 1e-30:
            raise ValueError("扭振频响矩阵奇异")
        if piv != k:
            M[k], M[piv] = M[piv], M[k]
            y[k], y[piv] = y[piv], y[k]
        for i in range(k + 1, n):
            f = M[i][k] / M[k][k]
            if f == 0:
                continue
            M[i][k] = 0j
            for j in range(k + 1, n):
                M[i][j] -= f * M[k][j]
            y[i] -= f * y[k]
    x = [0j] * n
    for i in range(n - 1, -1, -1):
        x[i] = (y[i] - sum(M[i][j] * x[j] for j in range(i + 1, n))) / M[i][i]
    return x


# ----------------------------- 模型组装 -----------------------------

def build_model(spec: dict, src: dict):
    """组装折算到输入轴的集中参数模型。节点顺序：驱动联轴器 → 各轴（按深度）
    → 负载联轴器，便于读图。返回节点/边/矩阵与索引映射。"""
    ratios = src.get("ratios", {})
    shaft_in = {x["id"]: x for x in src.get("shafts", [])}
    meshes_in = {x["id"]: x for x in src.get("meshes", [])}

    nodes, idx = [], {}

    def add_node(nid, kind, name, sid, inertia, depth):
        r = ratios.get(sid, 0.0)
        if nid not in idx:
            idx[nid] = len(nodes)
            nodes.append({"id": nid, "kind": kind, "name": name,
                          "shaftId": sid, "inertia": inertia,
                          "r": r, "depth": depth})

    for c in spec["couplings"]:
        s = shaft_in.get(c["shaftId"])
        if s:
            add_node(c["id"], c["kind"],
                     "驱动端" if c["kind"] == "driver" else "负载端·" + s["name"],
                     c["shaftId"], c["inertia"], s["depth"])
    for s in src.get("shafts", []):
        add_node("s:" + s["id"], "shaft", s["name"], s["id"],
                 spec["shafts"].get(s["id"], {}).get("inertia",
                                                     max(1e-12, s.get("gearInertia", 1e-8))),
                 s["depth"])
    n = len(nodes)
    M = [[0.0] * n for _ in range(n)]
    for nd in nodes:
        rr = nd["r"] if nd["r"] else 1.0
        M[idx[nd["id"]]][idx[nd["id"]]] = max(1e-14, nd["inertia"] * rr * rr)

    K = [[0.0] * n for _ in range(n)]
    C = [[0.0] * n for _ in range(n)]
    edges = []

    def add_spring(eid, kind, name, id_a, shaft_a, id_b, shaft_b, sigma,
                   k_phys, c_phys, extra=None):
        ia, ib = idx[id_a], idx[id_b]
        # 广义坐标取 q̃=r·θ（物理角 θ=q̃/r，r 带转向符号）：
        # 物理变形 Δ=q̃a/ra − σ q̃b/rb，两端对角项分别为 k/ra²、k/rb²，
        # 交叉项为 −σ·k/(ra·rb)（不可假定两端转速比相同）。
        ra = ratios.get(shaft_a, 0.0) or 1.0
        rb = ratios.get(shaft_b, 0.0) or 1.0
        K[ia][ia] += k_phys / (ra * ra)
        K[ib][ib] += k_phys / (rb * rb)
        K[ia][ib] -= sigma * k_phys / (ra * rb)
        K[ib][ia] -= sigma * k_phys / (ra * rb)
        C[ia][ia] += c_phys / (ra * ra)
        C[ib][ib] += c_phys / (rb * rb)
        C[ia][ib] -= sigma * c_phys / (ra * rb)
        C[ib][ia] -= sigma * c_phys / (ra * rb)
        edge = {"id": eid, "kind": kind, "name": name,
                "a": ia, "b": ib, "sigma": sigma,
                "ra": ratios.get(shaft_a, 1.0), "rb": ratios.get(shaft_b, 1.0),
                "k": k_phys, "c": c_phys}
        if extra:
            edge.update(extra)
        edges.append(edge)

    for m in src.get("meshes", []):
        cfg = spec["meshes"].get(m["id"], {})
        add_spring("m:" + m["id"], "mesh", m["name"],
                   "s:" + m["driver"], m["driver"],
                   "s:" + m["driven"], m["driven"],
                   1.0 if m["internal"] else -1.0,
                   max(1e-9, _pos(cfg.get("stiffness"), 1.0)),
                   max(0.0, _num(cfg.get("damping"), 0.0)),
                   {"meshId": m["id"], "meanTorque": max(0.0, m.get("torqueNm", 0.0)),
                    "locked": bool(cfg.get("locked"))})
    for c in spec["couplings"]:
        add_spring("c:" + c["id"], "coupling",
                   ("驱动联轴器" if c["kind"] == "driver"
                    else "负载联轴器·" + shaft_in[c["shaftId"]]["name"]),
                   c["id"], c["shaftId"], "s:" + c["shaftId"], c["shaftId"],
                   1.0, max(1e-9, _pos(c.get("stiffness"), 1.0)),
                   max(0.0, _num(c.get("damping"), 0.0)),
                   {"locked": bool(c.get("locked"))})
    return nodes, edges, M, K, C, idx


# ----------------------------- 转速扫描主分析 -----------------------------

def analyze_case(spec: dict, frozen: dict | None = None) -> dict:
    frozen = frozen or spec.get("frozen") or {}
    key = spec.get("primarySource") if isinstance(spec, dict) else None
    src = (frozen.get("sources") or {}).get(key)
    if not src:
        order = frozen.get("order") or []
        src = (frozen.get("sources") or {}).get(order[0]) if order else None
    issues = []

    def add(sev, code, msg, **refs):
        issues.append({"severity": sev, "code": code, "message": msg, "refs": refs})

    if not src:
        return {"ok": False, "issues": [{"severity": "error", "code": "NO_SOURCE",
                "message": "尚未冻结载荷来源（先从载荷版本建立草稿）", "refs": {}}]}
    if src.get("error"):
        return {"ok": False, "issues": [{"severity": "error", "code": "BAD_SOURCE",
                "message": "冻结来源计算失败：%s" % src["error"], "refs": {}}]}
    s = _merge_spec(spec, frozen)
    nodes, edges, M, K, C, idx = build_model(s, src)
    n = len(nodes)

    # —— 固有模态 ——
    modes_raw = _eigen_modes(M, K)
    modes = []
    for mi, mm in enumerate(modes_raw):
        phi = mm["phi"]
        omega = mm["omega"]
        modal_c = sum(C[i][j] * phi[i] * phi[j]
                      for i in range(n) for j in range(n))
        zeta = modal_c / (2.0 * omega) if omega > 1e-9 else 0.0
        zeta = max(0.0, zeta)
        # 物理扭振幅（按最大广义分量归一），用于模态形状图
        phys = []
        for nd, q in zip(nodes, phi):
            rr = nd["r"] if nd["r"] else 1.0
            phys.append(q / rr)
        amax = max((abs(v) for v in phys), default=1.0) or 1.0
        shape = [_r(v / amax, 4) for v in phys]
        modes.append({"index": mi, "hz": _r(mm["hz"], 3),
                      "omega": _r(omega, 5), "zeta": _r(zeta, 5),
                      "rigid": omega <= 1e-9, "shape": shape,
                      "nodeId": max(range(n), key=lambda ni: abs(shape[ni]))
                      if n else None})
    # 模态主导节点索引 → 节点 id
    for m in modes:
        if m["nodeId"] is not None:
            m["nodeId"] = nodes[m["nodeId"]]["id"]

    zeta_min = s["limits"]["zetaMin"]
    under = [m for m in modes if not m["rigid"] and m["zeta"] < zeta_min]
    for m in under:
        add("warning", "LOW_DAMPING",
            "第 %d 阶固有频率 %.2f Hz 模态阻尼比 ζ=%.4f 低于下限 %.3f，共振放大风险高"
            % (m["index"], m["hz"], m["zeta"], zeta_min),
            mode=m["index"])

    # —— 转速扫描样本（粗扫步长 + 各阶共振转速附近加密，避免漏掉窄共振峰）——
    lo, hi, step = s["scan"]["rpmMin"], s["scan"]["rpmMax"], s["scan"]["rpmStep"]
    ex_node0 = s["excitation"]["nodeId"]
    ex_r0 = abs(nodes[idx[ex_node0]]["r"]) if ex_node0 in idx else 1.0
    sample_set = []
    x = lo
    while x <= hi + 1e-9:
        sample_set.append(round(x, 4))
        x += step
    if not sample_set or sample_set[-1] < hi - 1e-9:
        sample_set.append(round(hi, 4))
    refine_hs = [od["h"] for od in s["excitation"]["orders"]] or [1.0]
    for mm in modes:
        if mm["rigid"]:
            continue
        nc_base = 60.0 * mm["hz"] / ex_r0
        beta = max(3.0 * max(mm["zeta"], 1e-4), 0.02)
        for h in refine_hs:
            nc = nc_base / h
            for f in (-2.0, -1.5, -1.0, -0.67, -0.4, -0.2, -0.1, 0.0,
                      0.1, 0.2, 0.4, 0.67, 1.0, 1.5, 2.0):
                v = nc * (1.0 + f * beta)
                if lo - 1e-9 <= v <= hi + 1e-9:
                    sample_set.append(round(v, 3))
    rpms = sorted(set(sample_set))
    if len(rpms) > MAX_SAMPLES:
        # 保留两端后等距抽取（共振加密点多在中部，抽取后仍远密于粗扫）
        keep = [rpms[0]] + [rpms[int(k * (len(rpms) - 1) / (MAX_SAMPLES - 1))]
                            for k in range(1, MAX_SAMPLES - 1)] + [rpms[-1]]
        rpms = sorted(set(keep))
    n_samp = len(rpms)

    ex_node = s["excitation"]["nodeId"]
    ex_i = idx.get(ex_node)
    ex_r = abs(nodes[ex_i]["r"]) if ex_i is not None else 1.0

    # 每阶次：节点物理幅/相位、边动态转矩；多阶次动态分量平方和开根
    per_order = []
    edge_dyn_sq = [[0.0] * n_samp for _ in edges]
    node_amp_sq = [[0.0] * n_samp for _ in nodes]
    for od in s["excitation"]["orders"]:
        h, amp = od["h"], od["amp"]
        na = [[0.0] * n_samp for _ in nodes]
        np_ = [[0.0] * n_samp for _ in nodes]
        ed = [[0.0] * n_samp for _ in edges]
        for k_i, rpm in enumerate(rpms):
            omega = h * 2.0 * math.pi * ex_r * rpm / 60.0
            A = [[complex(K[i][j] - omega * omega * M[i][j], omega * C[i][j])
                  for j in range(n)] for i in range(n)]
            Q = [0j] * n
            if ex_i is not None and amp > 0:
                # 物理谐和转矩 T 的广义力 Q=T/r（虚功：T·δθ=T·δq̃/r）
                Q[ex_i] = complex(amp / max(ex_r, 1e-12), 0.0)
            try:
                theta = _solve_complex(A, Q)
            except ValueError:
                theta = [0j] * n
            for ni, nd in enumerate(nodes):
                rr = abs(nd["r"]) if nd["r"] else 1.0
                z = theta[ni] / (nd["r"] if nd["r"] else 1.0)
                a = abs(z)
                na[ni][k_i] = a
                np_[ni][k_i] = (math.degrees(math.atan2(z.imag, z.real))
                                if a > 1e-30 else 0.0)
                node_amp_sq[ni][k_i] += a * a
            for ei, e in enumerate(edges):
                za = theta[e["a"]] / (e["ra"] if e["ra"] else 1.0)
                zb = theta[e["b"]] / (e["rb"] if e["rb"] else 1.0)
                delta = za - e["sigma"] * zb
                dyn = abs(complex(e["k"], omega * e["c"]) * delta)
                ed[ei][k_i] = dyn
                edge_dyn_sq[ei][k_i] += dyn * dyn
        per_order.append({"h": h, "amp": amp, "nodeId": ex_node,
                          "nodeAmp": na, "nodePhase": np_, "edgeDyn": ed})

    max_mean = max((e.get("meanTorque", 0.0) for e in edges
                    if e["kind"] == "mesh"), default=0.0)
    t_factor = s["limits"]["torqueFactor"]
    torque_curves, max_dyn_all = [], 0.0
    for ei, e in enumerate(edges):
        if e["kind"] != "mesh":
            continue
        mean = e.get("meanTorque", 0.0)
        limit = t_factor * mean if mean > 1e-9 else t_factor * max_mean
        dyn = [math.sqrt(v) for v in edge_dyn_sq[ei]]
        total = [mean + d for d in dyn]
        ampf = [(mean + d) / mean if mean > 1e-9 else (0.0 if d < 1e-12 else float("inf"))
                for d in dyn]
        max_dyn_all = max(max_dyn_all, max(total, default=0.0))
        # 超限连续区间（样本下标 → rpm）
        spans, open_i = [], None
        for k_i, t in enumerate(total):
            over = limit > 0 and t > limit
            if over and open_i is None:
                open_i = k_i
            elif not over and open_i is not None:
                if k_i - 1 > open_i:
                    spans.append([_r(rpms[open_i], 2), _r(rpms[k_i - 1], 2)])
                open_i = None
        if open_i is not None:
            spans.append([_r(rpms[open_i], 2), _r(rpms[-1], 2)])
        if spans:
            add("warning", "TORQUE_OVER",
                "啮合「%s」动态转矩超过 %.2f 倍平均转矩：%s rpm 等 %d 个区间"
                % (e["name"], t_factor,
                   "、".join("%g~%g" % (a, b) for a, b in spans[:3]), len(spans)),
                mesh=e.get("meshId"))
        torque_curves.append({"meshId": e.get("meshId"), "name": e["name"],
                              "mean": _r(mean, 4), "limit": _r(limit, 4),
                              "dyn": [_r(x, 5) for x in dyn],
                              "total": [_r(x, 5) for x in total],
                              "ampFactor": [None if not math.isfinite(x) else _r(x, 3)
                                            for x in ampf],
                              "spans": spans, "locked": e.get("locked", False)})

    # 联轴器动态转矩（动画/展示，不参与超限判定）
    coupling_curves = []
    for ei, e in enumerate(edges):
        if e["kind"] != "coupling":
            continue
        coupling_curves.append({"edgeId": e["id"], "name": e["name"],
                                "dyn": [_r(math.sqrt(v), 5) for v in edge_dyn_sq[ei]],
                                "locked": e.get("locked", False)})

    # —— Campbell：穿越转速、共振带、阻尼不足带 ——
    crossings, zones = [], []
    band_floor = 0.05
    for mi, m in enumerate(modes):
        if m["rigid"]:
            continue
        for od in per_order:
            h = od["h"]
            nc = 60.0 * m["hz"] / (h * ex_r)
            if not (lo - 0.05 * (hi - lo) <= nc <= hi + 0.05 * (hi - lo)):
                continue
            beta = max(2.0 * m["zeta"], band_floor)
            z0, z1 = nc * (1 - beta), nc * (1 + beta)
            in_range = z1 >= lo and z0 <= hi
            # 带内峰值放大系数
            peak = 0.0
            for k_i, rpm in enumerate(rpms):
                if z0 <= rpm <= z1:
                    for tc in torque_curves:
                        if tc["mean"] > 1e-9:
                            peak = max(peak, tc["total"][k_i] / tc["mean"])
            danger = peak >= t_factor
            underdamped = m["zeta"] < zeta_min
            crossings.append({"rpm": _r(nc, 2), "hz": m["hz"], "mode": mi,
                              "h": h, "zeta": m["zeta"], "inRange": in_range,
                              "band": [_r(max(lo, z0), 2), _r(min(hi, z1), 2)],
                              "peakAmp": _r(peak, 3), "danger": danger,
                              "underdamped": underdamped})
            if in_range:
                zones.append({"mode": mi, "h": h,
                              "rpm0": _r(max(lo, z0), 2),
                              "rpm1": _r(min(hi, z1), 2),
                              "danger": danger, "underdamped": underdamped})
                if danger:
                    add("warning", "RESONANCE",
                        "第 %d 阶（%.2f Hz）与 %g 阶激励在 %.0f rpm 相交，带内放大 %.2f×"
                        % (mi, m["hz"], h, nc, peak),
                        mode=mi, rpm=_r(nc, 2))

    # —— 节点当前曲线（平方和幅，供默认游标读数） ——
    node_curves = [{"nodeId": nd["id"], "name": nd["name"], "kind": nd["kind"],
                    "shaftId": nd["shaftId"], "r": _r(nd["r"], 6),
                    "inertia": _r(nd["inertia"], 8), "locked":
                        (s["shafts"].get(nd["shaftId"], {}).get("locked", False)
                         if nd["kind"] == "shaft" else
                         next((c.get("locked") for c in s["couplings"]
                               if c["id"] == nd["id"]), False)),
                    "amp": [_r(math.sqrt(a), 8) for a in node_amp_sq[ni]]}
                   for ni, nd in enumerate(nodes)]

    return {
        "ok": not any(i["severity"] == "error" for i in issues),
        "issues": issues,
        "source": {"key": src["key"], "name": src["name"],
                   "inputId": src.get("inputId"), "rpm0": src.get("rpm0"),
                   "torqueInNm": src.get("torqueInNm")},
        "rpms": [_r(x, 2) for x in rpms],
        "nodes": node_curves,
        "edges": [{"id": e["id"], "kind": e["kind"], "name": e["name"],
                   "a": nodes[e["a"]]["id"], "b": nodes[e["b"]]["id"],
                   "sigma": e["sigma"], "meshId": e.get("meshId"),
                   "meanTorque": _r(e.get("meanTorque", 0.0), 4),
                   "locked": e.get("locked", False)} for e in edges],
        "modes": modes,
        "orders": per_order,
        "torqueCurves": torque_curves,
        "couplingCurves": coupling_curves,
        "crossings": crossings,
        "zones": zones,
        "limits": s["limits"],
        "summary": {
            "nModes": len([m for m in modes if not m["rigid"]]),
            "nResonanceZones": len([z for z in zones if z["danger"]]),
            "nZones": len(zones),
            "nUnderDamped": len(under),
            "nOverLimit": sum(len(tc["spans"]) for tc in torque_curves),
            "maxTorque": _r(max_dyn_all, 4),
            "maxAmpFactor": _r(max((a for tc in torque_curves for a in tc["ampFactor"]
                                   if a is not None), default=0.0), 3),
            "freqRange": [modes and modes[-1]["hz"] or 0],
        },
    }


# ----------------------------- 避振方案搜索 -----------------------------

def search(spec: dict, frozen: dict | None, params: dict) -> dict:
    """在飞轮惯量（加到指定轴）、未锁定联轴器刚度/阻尼倍率候选中枚举，
    按 危险共振区数 → 最大动态转矩 → 附加惯量 → 改动量 排序。"""
    frozen = frozen or (spec.get("frozen") if isinstance(spec, dict) else None) or {}
    deadline = time.time() + max(1.0, _num(params.get("timeBudget"), 20.0))
    limit = max(1, int(_num(params.get("limit"), 12)))
    base_s = _merge_spec(spec, frozen)
    cur = analyze_case(base_s, frozen)

    def vals(key, dft):
        raw = params.get(key)
        out = []
        for v in raw or []:
            try:
                fv = float(v)
            except (TypeError, ValueError):
                continue
            if fv > 0 and fv not in out:
                out.append(fv)
        return out or dft

    j_cands = vals("flywheelJ", [0.0])
    if 0.0 not in j_cands:
        j_cands = [0.0] + j_cands
    k_mults = vals("kMult", [1.0])
    c_mults = vals("cMult", [1.0])
    # 锁定轴不可作为飞轮安装位（其惯量不允许被搜索改变）
    fw_shafts = [sid for sid in (params.get("flywheelShafts") or [])
                 if sid in base_s["shafts"]
                 and not base_s["shafts"][sid].get("locked")]
    nodes_n = 0
    truncated = False

    base_j = {sid: v["inertia"] for sid, v in base_s["shafts"].items()}
    sum_j = sum(base_j.values()) or 1.0
    unlocked_c = [c for c in base_s["couplings"] if not c.get("locked")]
    k0 = {c["id"]: c["stiffness"] for c in base_s["couplings"]}
    c0 = {c["id"]: c["damping"] for c in base_s["couplings"]}

    results, seen = [], set()

    def evaluate(j_sid, j_add, km, cm):
        nonlocal nodes_n, truncated
        # 指定安装轴但附加惯量为 0 时与“不加飞轮”等价，跳过避免重复当前候选
        if j_sid and j_add <= 0:
            return
        if time.time() > deadline:
            truncated = True
            return
        key = (j_sid or "", round(j_add, 9), round(km, 5), round(cm, 5))
        if key in seen:
            return
        seen.add(key)
        nodes_n += 1
        cand = json.loads(json.dumps(base_s))
        added = []
        if j_add > 0 and j_sid:
            cand["shafts"][j_sid]["inertia"] = base_j[j_sid] + j_add
            added.append({"shaftId": j_sid, "inertia": _r(j_add, 8)})
        for c in cand["couplings"]:
            if not c.get("locked"):
                c["stiffness"] = k0[c["id"]] * km
                c["damping"] = c0[c["id"]] * cm
        try:
            r = analyze_case(cand, frozen)
        except Exception:  # noqa: BLE001
            return
        sm = r["summary"]
        change = abs(km - 1.0) + abs(cm - 1.0)
        if j_add > 0:
            change += 1.0 + j_add / sum_j
        patch = {"couplings": {c["id"]: {
            "stiffness": _r(k0[c["id"]] * km, 4),
            "damping": _r(c0[c["id"]] * cm, 6)} for c in unlocked_c},
            "flywheelShaft": j_sid, "flywheelJ": _r(j_add, 8) if j_add else 0.0}
        results.append({
            "flywheelShaft": j_sid, "flywheelJ": _r(j_add, 8) if j_add else 0.0,
            "kMult": km, "cMult": cm,
            "nResonanceZones": sm["nResonanceZones"],
            "nOverLimit": sm["nOverLimit"],
            "nUnderDamped": sm["nUnderDamped"],
            "maxTorque": sm["maxTorque"], "maxAmpFactor": sm["maxAmpFactor"],
            "addedJ": _r(j_add, 8), "change": _r(change, 4),
            "crossings": r["crossings"], "zones": r["zones"],
            "torqueCurves": r["torqueCurves"], "rpms": r["rpms"],
            "modes": r["modes"], "summary": sm,
            "patch": patch,
        })

    # 当前配置（无飞轮、倍率 1）始终先评估
    evaluate(None, 0.0, 1.0, 1.0)
    for j_sid in (fw_shafts or [None]):
        for j_add in j_cands:
            for km in k_mults:
                for cm in c_mults:
                    evaluate(j_sid, j_add if j_sid else 0.0, km, cm)
                    if truncated:
                        break
                if truncated:
                    break
            if truncated:
                break
        if truncated:
            break

    results.sort(key=lambda c: (c["nResonanceZones"], c["maxTorque"],
                                c["addedJ"], c["change"],
                                c["flywheelShaft"] or "", c["kMult"], c["cMult"]))
    for c in results:
        c["current"] = (c["flywheelJ"] == 0.0 and c["kMult"] == 1.0
                        and c["cMult"] == 1.0)
    return {"results": results[:limit], "nodes": nodes_n, "truncated": truncated,
            "base": {"nResonanceZones": cur["summary"]["nResonanceZones"],
                     "nOverLimit": cur["summary"]["nOverLimit"],
                     "maxTorque": cur["summary"]["maxTorque"],
                     "maxAmpFactor": cur["summary"]["maxAmpFactor"]}}
