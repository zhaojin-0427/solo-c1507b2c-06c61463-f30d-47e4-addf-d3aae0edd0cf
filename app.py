# -*- coding: utf-8 -*-
"""轮系配齿台 — Flask + SQLite 后端。"""
import json
import os
import sqlite3
import time

from flask import Flask, g, jsonify, render_template, request

import kinematics
import meshing

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


init_db()

if __name__ == "__main__":
    app.run(host="127.0.0.1", port=5000, debug=True)
