# -*- coding: utf-8 -*-
"""轮系配齿台 — Flask + SQLite 后端。"""
import json
import os
import sqlite3
import time

from flask import Flask, g, jsonify, render_template, request

import kinematics
import meshing
import contact
import backlash
import loadcase
import thermal

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
DB_PATH = os.path.join(BASE_DIR, "geartrain.db")

app = Flask(__name__)


# ----------------------------- 数据库 -----------------------------

def db():
    if "db" not in g:
        g.db = sqlite3.connect(DB_PATH)
        g.db.row_factory = sqlite3.Row
        g.db.execute("PRAGMA foreign_keys = ON")
    return g.db


@app.teardown_appcontext
def close_db(_exc):
    con = g.pop("db", None)
    if con:
        con.close()


def init_db():
    con = sqlite3.connect(DB_PATH)
    con.executescript("""
    CREATE TABLE IF NOT EXISTS project (
        id INTEGER PRIMARY KEY CHECK (id = 1),
        state TEXT NOT NULL,
        baseline_id INTEGER,
        updated_at REAL
    );
    CREATE TABLE IF NOT EXISTS alternatives (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        name TEXT NOT NULL,
        baseline_id INTEGER,
        target TEXT,
        ratio TEXT,
        error_pct REAL,
        state TEXT NOT NULL,
        created_at REAL
    );
    CREATE TABLE IF NOT EXISTS backlash_cases (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        name TEXT NOT NULL,
        version INTEGER NOT NULL,
        note TEXT,
        spec TEXT NOT NULL,
        snapshot TEXT NOT NULL,
        solution TEXT,
        created_at REAL
    );
    CREATE TABLE IF NOT EXISTS load_cases (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        name TEXT NOT NULL,
        version INTEGER NOT NULL,
        note TEXT,
        spec TEXT NOT NULL,
        snapshot TEXT NOT NULL,
        solution TEXT,
        created_at REAL
    );
    CREATE TABLE IF NOT EXISTS thermal_cases (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        name TEXT NOT NULL,
        version INTEGER NOT NULL,
        note TEXT,
        spec TEXT NOT NULL,
        frozen TEXT NOT NULL,
        solution TEXT,
        created_at REAL
    );
    """)
    con.commit()
    con.close()


def load_project():
    row = db().execute("SELECT state FROM project WHERE id = 1").fetchone()
    return json.loads(row["state"]) if row else None


def save_project(state, baseline_id=None):
    con = db()
    now = time.time()
    con.execute(
        """INSERT INTO project (id, state, baseline_id, updated_at)
           VALUES (1, ?, ?, ?)
           ON CONFLICT(id) DO UPDATE SET
             state=excluded.state,
             baseline_id=excluded.baseline_id,
             updated_at=excluded.updated_at""",
        (json.dumps(state, ensure_ascii=False), baseline_id, now))
    con.commit()


# ----------------------------- 页面 -----------------------------

@app.route("/")
def index():
    return render_template("index.html")


@app.get("/api/project")
def api_get_project():
    state = load_project()
    row = db().execute("SELECT baseline_id FROM project WHERE id = 1").fetchone()
    baseline = None
    bid = row["baseline_id"] if row else None
    if bid is not None:
        brow = db().execute("SELECT id, name, state FROM alternatives WHERE id = ?",
                            (bid,)).fetchone()
        if brow:
            baseline = {"id": brow["id"], "name": brow["name"],
                        "state": json.loads(brow["state"])}
    return jsonify({"state": state, "baseline": baseline})


@app.post("/api/project")
def api_save_project():
    body = request.get_json(force=True)
    state = body.get("state")
    if not isinstance(state, dict):
        return jsonify({"error": "state 必须为对象"}), 400
    row = db().execute("SELECT baseline_id FROM project WHERE id = 1").fetchone()
    save_project(state, row["baseline_id"] if row else None)
    return jsonify({"ok": True})


@app.post("/api/analyze")
def api_analyze():
    body = request.get_json(force=True)
    try:
        return jsonify(kinematics.analyze(body.get("state", {}),
                                          float(body.get("centerTol", kinematics.CENTER_TOL))))
    except Exception as exc:  # noqa: BLE001
        return jsonify({"ok": False, "error": str(exc)}), 400


@app.post("/api/search")
def api_search():
    body = request.get_json(force=True)
    try:
        return jsonify(kinematics.search(body))
    except Exception as exc:  # noqa: BLE001
        return jsonify({"results": [], "note": "搜索参数有误：%s" % exc}), 400


@app.post("/api/search-planets")
def api_search_planets():
    body = request.get_json(force=True)
    try:
        return jsonify(kinematics.search_planets(body))
    except Exception as exc:  # noqa: BLE001
        return jsonify({"results": [], "note": "搜索参数有误：%s" % exc}), 400


@app.post("/api/mesh-check")
def api_mesh_check():
    """渐开线啮合校核：按实际中心距计算工作压力角、啮合线、重合度与滑动率。"""
    body = request.get_json(force=True)
    try:
        return jsonify(meshing.check_pair(
            body.get("gearA", {}), body.get("gearB", {}),
            float(body.get("center", 0)),
            float(body.get("epsMin", meshing.EPS_MIN_DEFAULT))))
    except Exception as exc:  # noqa: BLE001
        return jsonify({"ok": False, "issues": [
            {"severity": "error", "code": "BAD_REQUEST",
             "message": "校核参数有误：%s" % exc}]}), 400


@app.post("/api/mesh-shifts")
def api_mesh_shifts():
    """成对变位与可调整中心距搜索。"""
    body = request.get_json(force=True)
    try:
        return jsonify(meshing.search_shifts(body))
    except Exception as exc:  # noqa: BLE001
        return jsonify({"results": [], "note": "搜索参数有误：%s" % exc}), 400


@app.post("/api/teeth")
def api_teeth():
    """齿对接触周期：沿传动方向展开齿对事件、可达子集、接触矩阵、关注齿相遇角。"""
    body = request.get_json(force=True)
    try:
        turns = body.get("observeTurns")
        turns = float(turns) if turns is not None else None
        return jsonify(contact.analyze_teeth(body.get("state", {}), turns))
    except Exception as exc:  # noqa: BLE001
        return jsonify({"ok": False, "error": str(exc)}), 400


@app.post("/api/teeth-assembly")
def api_teeth_assembly():
    """锁定带键齿轮相位，枚举其余齿轮的整齿装配偏移候选。"""
    body = request.get_json(force=True)
    try:
        turns = body.get("observeTurns")
        turns = float(turns) if turns is not None else None
        return jsonify(contact.enumerate_assembly(
            body.get("state", {}),
            offset_range=int(body.get("offsetRange", 3)),
            observe_turns=turns,
            time_budget=float(body.get("timeBudget", 6.0)),
        ))
    except Exception as exc:  # noqa: BLE001
        return jsonify({"candidates": [], "note": "装配枚举参数有误：%s" % exc}), 400


@app.post("/api/alternatives")
def api_create_alt():
    body = request.get_json(force=True)
    name = (body.get("name") or "").strip() or ("方案 %s" % time.strftime("%m-%d %H:%M"))
    con = db()
    cur = con.execute(
        """INSERT INTO alternatives (name, baseline_id, target, ratio, error_pct,
                                     state, created_at)
           VALUES (?, ?, ?, ?, ?, ?, ?)""",
        (name, body.get("baselineId"), body.get("target"), body.get("ratio"),
         body.get("errorPct"),
         json.dumps(body.get("state"), ensure_ascii=False), time.time()))
    con.commit()
    return jsonify({"id": cur.lastrowid, "ok": True})


@app.get("/api/alternatives")
def api_list_alts():
    rows = db().execute(
        "SELECT id, name, baseline_id, target, ratio, error_pct, created_at "
        "FROM alternatives ORDER BY id DESC").fetchall()
    return jsonify([dict(r) for r in rows])


@app.get("/api/alternatives/<int:alt_id>")
def api_get_alt(alt_id):
    row = db().execute("SELECT * FROM alternatives WHERE id = ?", (alt_id,)).fetchone()
    if not row:
        return jsonify({"error": "不存在"}), 404
    d = dict(row)
    d["state"] = json.loads(d["state"])
    return jsonify(d)


@app.delete("/api/alternatives/<int:alt_id>")
def api_del_alt(alt_id):
    db().execute("DELETE FROM alternatives WHERE id = ?", (alt_id,))
    db().commit()
    return jsonify({"ok": True})


@app.post("/api/baseline/<int:alt_id>")
def api_set_baseline(alt_id):
    row = db().execute("SELECT id FROM alternatives WHERE id = ?", (alt_id,)).fetchone()
    if not row:
        return jsonify({"error": "方案不存在"}), 404
    con = db()
    prow = con.execute("SELECT state FROM project WHERE id = 1").fetchone()
    if not prow:
        return jsonify({"error": "请先保存当前项目"}), 400
    con.execute("UPDATE project SET baseline_id = ? WHERE id = 1", (alt_id,))
    con.commit()
    return jsonify({"ok": True})


@app.post("/api/baseline/clear")
def api_clear_baseline():
    con = db()
    con.execute("UPDATE project SET baseline_id = NULL WHERE id = 1")
    con.commit()
    return jsonify({"ok": True})


# ----------------------------- 回程间隙工况 -----------------------------

@app.post("/api/backlash/analyze")
def api_backlash_analyze():
    """回程间隙工况计算：各级间隙折算到观察轴、相容区间与换面角。"""
    body = request.get_json(force=True)
    try:
        return jsonify(backlash.analyze_case(
            body.get("state", {}), body.get("spec", {})))
    except Exception as exc:  # noqa: BLE001
        return jsonify({"ok": False, "error": str(exc)}), 400


@app.post("/api/backlash/search")
def api_backlash_search():
    """间隙组合搜索：等级 × 中心距调整 × 变位和，按最坏空程/冲突数/改动量排序。"""
    body = request.get_json(force=True)
    try:
        return jsonify(backlash.search(
            body.get("state", {}), body.get("spec", {}), body))
    except Exception as exc:  # noqa: BLE001
        return jsonify({"results": [], "note": "搜索参数有误：%s" % exc}), 400


@app.get("/api/backlash/cases")
def api_backlash_cases():
    rows = db().execute(
        "SELECT id, name, version, note, created_at FROM backlash_cases "
        "ORDER BY name, version DESC").fetchall()
    return jsonify([dict(r) for r in rows])


@app.post("/api/backlash/cases")
def api_backlash_case_save():
    """另存工况版本：同名工况版本号递增，不改写原轮系方案。"""
    body = request.get_json(force=True)
    name = (body.get("name") or "").strip() or "换向工况"
    spec = body.get("spec")
    snapshot = body.get("snapshot")
    if not isinstance(spec, dict) or not isinstance(snapshot, dict):
        return jsonify({"error": "spec 与 snapshot 必须为对象"}), 400
    con = db()
    row = con.execute(
        "SELECT COALESCE(MAX(version), 0) AS v FROM backlash_cases WHERE name = ?",
        (name,)).fetchone()
    version = row["v"] + 1
    cur = con.execute(
        """INSERT INTO backlash_cases (name, version, note, spec, snapshot, solution, created_at)
           VALUES (?, ?, ?, ?, ?, ?, ?)""",
        (name, version, body.get("note"),
         json.dumps(spec, ensure_ascii=False),
         json.dumps(snapshot, ensure_ascii=False),
         json.dumps(body.get("solution"), ensure_ascii=False)
         if body.get("solution") is not None else None,
         time.time()))
    con.commit()
    return jsonify({"id": cur.lastrowid, "version": version, "ok": True})


@app.get("/api/backlash/cases/<int:case_id>")
def api_backlash_case_get(case_id):
    row = db().execute("SELECT * FROM backlash_cases WHERE id = ?", (case_id,)).fetchone()
    if not row:
        return jsonify({"error": "不存在"}), 404
    d = dict(row)
    d["spec"] = json.loads(d["spec"])
    d["snapshot"] = json.loads(d["snapshot"])
    d["solution"] = json.loads(d["solution"]) if d["solution"] else None
    return jsonify(d)


@app.delete("/api/backlash/cases/<int:case_id>")
def api_backlash_case_del(case_id):
    db().execute("DELETE FROM backlash_cases WHERE id = ?", (case_id,))
    db().commit()
    return jsonify({"ok": True})


# ----------------------------- 载荷工况 -----------------------------

@app.post("/api/loadcase/analyze")
def api_loadcase_analyze():
    """载荷工况：功率/转矩传播、啮合力合成、轴承反力与安全余量。"""
    body = request.get_json(force=True)
    try:
        return jsonify(loadcase.analyze_case(
            body.get("state", {}), body.get("spec", {})))
    except Exception as exc:  # noqa: BLE001
        return jsonify({"ok": False, "issues": [
            {"severity": "error", "code": "BAD_REQUEST",
             "message": "载荷计算参数有误：%s" % exc, "refs": {}}]}), 400


@app.post("/api/loadcase/search")
def api_loadcase_search():
    """轴系布置搜索：轴长/最小间距/可调范围内枚举安装面与轴承位置。"""
    body = request.get_json(force=True)
    try:
        return jsonify(loadcase.search_layouts(
            body.get("state", {}), body.get("spec", {}), body))
    except Exception as exc:  # noqa: BLE001
        return jsonify({"results": [], "shafts": {},
                        "note": "搜索参数有误：%s" % exc}), 400


@app.get("/api/loadcase/cases")
def api_loadcase_cases():
    rows = db().execute(
        "SELECT id, name, version, note, created_at FROM load_cases "
        "ORDER BY name, version DESC").fetchall()
    return jsonify([dict(r) for r in rows])


@app.post("/api/loadcase/cases")
def api_loadcase_case_save():
    """另存载荷工况版本：同名工况版本号递增，不改写原轮系方案。"""
    body = request.get_json(force=True)
    name = (body.get("name") or "").strip() or "受载工况"
    spec = body.get("spec")
    snapshot = body.get("snapshot")
    if not isinstance(spec, dict) or not isinstance(snapshot, dict):
        return jsonify({"error": "spec 与 snapshot 必须为对象"}), 400
    con = db()
    row = con.execute(
        "SELECT COALESCE(MAX(version), 0) AS v FROM load_cases WHERE name = ?",
        (name,)).fetchone()
    version = row["v"] + 1
    cur = con.execute(
        """INSERT INTO load_cases (name, version, note, spec, snapshot, solution, created_at)
           VALUES (?, ?, ?, ?, ?, ?, ?)""",
        (name, version, body.get("note"),
         json.dumps(spec, ensure_ascii=False),
         json.dumps(snapshot, ensure_ascii=False),
         json.dumps(body.get("solution"), ensure_ascii=False)
         if body.get("solution") is not None else None,
         time.time()))
    con.commit()
    return jsonify({"id": cur.lastrowid, "version": version, "ok": True})


@app.get("/api/loadcase/cases/<int:case_id>")
def api_loadcase_case_get(case_id):
    row = db().execute("SELECT * FROM load_cases WHERE id = ?", (case_id,)).fetchone()
    if not row:
        return jsonify({"error": "不存在"}), 404
    d = dict(row)
    d["spec"] = json.loads(d["spec"])
    d["snapshot"] = json.loads(d["snapshot"])
    d["solution"] = json.loads(d["solution"]) if d["solution"] else None
    return d


@app.delete("/api/loadcase/cases/<int:case_id>")
def api_loadcase_case_del(case_id):
    db().execute("DELETE FROM load_cases WHERE id = ?", (case_id,))
    db().commit()
    return jsonify({"ok": True})


# ----------------------------- 热平衡工况 -----------------------------

@app.get("/api/thermal/oils")
def api_thermal_oils():
    """可选油品黏温曲线（牌号 + 40/100°C 折线段）。"""
    return jsonify({"oils": thermal.OIL_GRADES})


@app.post("/api/thermal/freeze")
def api_thermal_freeze():
    """从已保存（或当前草稿）的载荷工况冻结各级损失/转速/持续时间与节点默认参数。
    body: {"sources": [{"key","name","state","spec","caseId","version","inline"}]}"""
    body = request.get_json(force=True)
    sources = body.get("sources", [])
    if not isinstance(sources, list) or not sources:
        return jsonify({"error": "sources 必须为非空数组"}), 400
    clean = []
    for src in sources:
        if not isinstance(src, dict) or not isinstance(src.get("state"), dict):
            continue
        clean.append({"key": str(src.get("key")), "name": src.get("name"),
                      "state": src["state"], "spec": src.get("spec", {}),
                      "caseId": src.get("caseId"), "version": src.get("version"),
                      "inline": bool(src.get("inline"))})
    return jsonify(thermal.freeze_sources(clean))


@app.post("/api/thermal/analyze")
def api_thermal_analyze():
    """热网络时间步迭代：温度、黏温反馈效率、热流、风扇状态与四类问题时段。"""
    body = request.get_json(force=True)
    try:
        return jsonify(thermal.analyze_case(body.get("spec", {}), body.get("frozen")))
    except Exception as exc:  # noqa: BLE001
        return jsonify({"ok": False, "issues": [
            {"severity": "error", "code": "BAD_REQUEST",
             "message": "热平衡参数有误：%s" % exc, "refs": {}}]}), 400


@app.post("/api/thermal/search")
def api_thermal_search():
    """油品 × 散热片面积 × 风扇阈值搜索，按超限时长/峰值/能耗/改动量排序。"""
    body = request.get_json(force=True)
    try:
        return jsonify(thermal.search(body.get("spec", {}), body.get("frozen"), body))
    except Exception as exc:  # noqa: BLE001
        return jsonify({"results": [], "note": "搜索参数有误：%s" % exc}), 400


@app.get("/api/thermal/cases")
def api_thermal_cases():
    rows = db().execute(
        "SELECT id, name, version, note, created_at FROM thermal_cases "
        "ORDER BY name, version DESC").fetchall()
    return jsonify([dict(r) for r in rows])


@app.post("/api/thermal/cases")
def api_thermal_case_save():
    """另存热平衡工况版本：同名版本号递增；solution 保留时间步与全部温度曲线。"""
    body = request.get_json(force=True)
    name = (body.get("name") or "").strip() or "热平衡工况"
    spec, frozen = body.get("spec"), body.get("frozen")
    if not isinstance(spec, dict) or not isinstance(frozen, dict):
        return jsonify({"error": "spec 与 frozen 必须为对象"}), 400
    con = db()
    row = con.execute(
        "SELECT COALESCE(MAX(version), 0) AS v FROM thermal_cases WHERE name = ?",
        (name,)).fetchone()
    version = row["v"] + 1
    cur = con.execute(
        """INSERT INTO thermal_cases (name, version, note, spec, frozen, solution, created_at)
           VALUES (?, ?, ?, ?, ?, ?, ?)""",
        (name, version, body.get("note"),
         json.dumps(spec, ensure_ascii=False),
         json.dumps(frozen, ensure_ascii=False),
         json.dumps(body.get("solution"), ensure_ascii=False)
         if body.get("solution") is not None else None,
         time.time()))
    con.commit()
    return jsonify({"id": cur.lastrowid, "version": version, "ok": True})


@app.get("/api/thermal/cases/<int:case_id>")
def api_thermal_case_get(case_id):
    row = db().execute("SELECT * FROM thermal_cases WHERE id = ?", (case_id,)).fetchone()
    if not row:
        return jsonify({"error": "不存在"}), 404
    d = dict(row)
    d["spec"] = json.loads(d["spec"])
    d["frozen"] = json.loads(d["frozen"])
    d["solution"] = json.loads(d["solution"]) if d["solution"] else None
    return d


@app.delete("/api/thermal/cases/<int:case_id>")
def api_thermal_case_del(case_id):
    db().execute("DELETE FROM thermal_cases WHERE id = ?", (case_id,))
    db().commit()
    return jsonify({"ok": True})


init_db()

if __name__ == "__main__":
    app.run(host="127.0.0.1", port=5000, debug=True)
