/* 轮系配齿台 — 原生 JS 前端。运动学规则与 kinematics.py 保持一致。 */
'use strict';

const SVGNS = 'http://www.w3.org/2000/svg';
const SCALE = 3;                 // 基础 px / mm
const CENTER_TOL = 0.05;

/* ---------------- 分数（BigInt 精确） ---------------- */
const bGcd = (a, b) => { a = a < 0n ? -a : a; b = b < 0n ? -b : b; while (b) [a, b] = [b, a % b]; return a; };
function fr(n, d = 1n) {
  n = BigInt(n); d = BigInt(d);
  if (d === 0n) return null;
  if (d < 0n) { n = -n; d = -d; }
  const g = bGcd(n, d);
  return { n: n / g, d: d / g };
}
const fMul = (a, b) => fr(a.n * b.n, a.d * b.d);
const fAdd = (a, b) => fr(a.n * b.d + b.n * a.d, a.d * b.d);
const fSub = (a, b) => fr(a.n * b.d - b.n * a.d, a.d * b.d);
const fInv = a => fr(a.d, a.n);
const fNum = a => Number(a.n) / Number(a.d);
const fStr = a => a.d === 1n ? a.n.toString() : `${a.n}/${a.d}`;
function parseFr(s) {
  const t = String(s).trim();
  if (t.includes('/')) { const [n, d] = t.split('/'); return fr(n.trim(), d.trim()); }
  return fr(t);
}
const bLcm = (a, b) => a / bGcd(a, b) * b;

/* ---------------- DOM 工具 ---------------- */
const $ = s => document.querySelector(s);
function svgEl(tag, attrs = {}, parent) {
  const node = document.createElementNS(SVGNS, tag);
  for (const [k, v] of Object.entries(attrs)) {
    if (k === 'text') node.textContent = v;
    else node.setAttribute(k, v);
  }
  if (parent) parent.appendChild(node);
  return node;
}
function h(tag, attrs = {}, parent, text) {
  const node = document.createElement(tag);
  for (const [k, v] of Object.entries(attrs)) {
    if (k.startsWith('on')) node.addEventListener(k.slice(2), v);
    else if (k === 'class') node.className = v;
    else node.setAttribute(k, v);
  }
  if (text != null) node.textContent = text;
  if (parent) parent.appendChild(node);
  return node;
}
const uid = p => `${p}${++counters[p] || (counters[p] = 1)}`;
const counters = { s: 0, g: 0, m: 0, c: 0, p: 0 };
function bumpIds(state) {
  for (const list of [['shafts', 's'], ['gears', 'g'], ['meshes', 'm'],
                      ['coaxRelations', 'c'], ['planets', 'p']]) {
    const p = list[1];
    counters[p] = Math.max(counters[p], 0);
    (state[list[0]] || []).forEach(o => {
      const n = parseInt(String(o.id).replace(/^\D+/, ''), 10);
      if (Number.isFinite(n)) counters[p] = Math.max(counters[p], n);
    });
  }
}

/* ---------------- 全局状态 ---------------- */
let state = emptyState();
let selection = null;           // {type, id}
let analysis = null;
let baseline = null;            // {id,name,state}
let candidates = [];
let selectedCand = -1;
let planetCandidates = [];
let selectedPlanetId = null;
let motionTheta = 0;            // 输入轴累计转角（度），统一驱动所有运动
let dirty = false;
let saveTimer = null;
let stateVersion = 0;
let previewCache = null;

const view = { zoom: 1, panX: 60, panY: 200 };

function emptyState() {
  return { shafts: [], gears: [], meshes: [], coaxRelations: [], planets: [],
           inputId: null, outputId: null, inputRpm: 60 };
}
const byId = (list, id) => (list || []).find(o => o.id === id);
const shaftMap = () => Object.fromEntries(state.shafts.map(s => [s.id, s]));
const gearMap = () => Object.fromEntries(state.gears.map(g => [g.id, g]));
const planetMap = st => Object.fromEntries((st.planets || []).map(p => [p.id, p]));
const P_MEMBERS = ['s', 'r', 'c'];
const P_LABEL = { s: '太阳轮', r: '内齿圈', c: '行星架' };
const pNode = (pid, mem) => `${pid}:${mem}`;
const isPNode = id => typeof id === 'string' && /^p\d+:/.test(id);
function splitPNode(id) {
  const k = String(id).indexOf(':');
  return k < 0 ? null : { pid: id.slice(0, k), mem: id.slice(k + 1) };
}

/* 行星级几何（与 kinematics.py planet_geom 一致） */
function planetGeom(p) {
  if (![p.zS, p.zP, p.zR].every(Number.isInteger) || !(p.module > 0)) return null;
  const { zS, zR, zP, module: m } = p;
  const count = Math.max(1, p.count | 0);
  const orbit = m * (zS + zP) / 2;
  return {
    zS, zP, zR, m, orbit,
    rp: m * zP / 2,
    ringTip: m * (zR - 2) / 2, ringOuter: m * (zR + 2.5) / 2,
    sunTip: m * (zS + 2) / 2,
    chord: 2 * orbit * Math.sin(Math.PI / count),
  };
}

/* ---------------- 本地运动学分析（与 kinematics.py 同构） ---------------- */
function analyzeLocal(st, centerTol = CENTER_TOL) {
  const issues = [];
  const add = (severity, code, message, refs = {}) => issues.push({ severity, code, message, refs });
  const shafts = Object.fromEntries(st.shafts.map(s => [s.id, { ...s }]));
  const gears = Object.fromEntries(st.gears.map(g => [g.id, { ...g }]));
  const gname = g => g.name || (g.internal ? `内齿圈 z${g.z}` : `齿轮 z${g.z}`);
  const sname = s => s.name || `轴${s.id}`;

  for (const g of Object.values(gears)) {
    if (!shafts[g.shaftId]) add('error', 'ORPHAN_GEAR', `「${gname(g)}」没有安装到任何轴`, { gear: g.id });
    if (!Number.isInteger(g.z) || g.z <= 0) add('error', 'BAD_GEAR', `「${gname(g)}」齿数必须为正整数`, { gear: g.id });
    if (!(g.module > 0)) add('error', 'BAD_GEAR', `「${gname(g)}」模数必须为正数`, { gear: g.id });
    if (!(g.pressureAngle > 0)) add('warning', 'BAD_GEAR', `「${gname(g)}」压力角异常`, { gear: g.id });
  }

  const adj = new Map();
  st.shafts.forEach(s => adj.set(s.id, []));
  const edges = [];
  const pairSeen = new Map();

  for (const e of st.meshes) {
    const ga = gears[e.gearA], gb = gears[e.gearB];
    if (!ga || !gb) { add('error', 'BROKEN_MESH', '存在指向已删除齿轮的啮合约束', { mesh: e.id }); continue; }
    const sa = ga.shaftId, sb = gb.shaftId;
    if (!shafts[sa] || !shafts[sb]) continue;
    if (sa === sb) { add('error', 'SELF_MESH', `「${gname(ga)}」与「${gname(gb)}」在同一根轴上，不能互相啮合`, { mesh: e.id }); continue; }
    if (ga.internal && gb.internal) add('error', 'TWO_RINGS', `两个内齿圈不能互相啮合（${gname(ga)}、${gname(gb)}）`, { mesh: e.id });
    const internal = ga.internal || gb.internal;
    if (internal) {
      const zr = ga.internal ? ga.z : gb.z, zp = ga.internal ? gb.z : ga.z;
      if (Number.isInteger(zr) && Number.isInteger(zp) && zr <= zp)
        add('error', 'RING_TOO_SMALL', `内啮合时节圆半径为 m(z圈−z轮)/2：内齿圈 z${zr} 必须大于小齿轮 z${zp}`, { mesh: e.id });
    }
    if (ga.module > 0 && gb.module > 0 && Math.abs(ga.module - gb.module) > 1e-9)
      add('error', 'MODULE_MISMATCH', `模数不合：${gname(ga)} (m=${ga.module}) 与 ${gname(gb)} (m=${gb.module}) 无法啮合`, { mesh: e.id });
    if (Math.abs((ga.pressureAngle || 20) - (gb.pressureAngle || 20)) > 1e-6)
      add('warning', 'ALPHA_MISMATCH', `压力角不一致：${gname(ga)} (${ga.pressureAngle}°) 与 ${gname(gb)} (${gb.pressureAngle}°)`, { mesh: e.id });

    const A = shafts[sa], B = shafts[sb];
    const actual = Math.hypot(A.x - B.x, A.y - B.y);
    let expected = null, deviation = null;
    if (Number.isInteger(ga.z) && Number.isInteger(gb.z) && ga.module > 0) {
      // 理论中心距随变位和变化（xΣ=0 时即标准中心距），与后端 meshing.expected_center 一致
      expected = Involute.expectedCenter(ga, gb);
      if (expected == null)
        expected = internal ? ga.module * Math.abs(ga.z - gb.z) / 2 : ga.module * (ga.z + gb.z) / 2;
      deviation = actual - expected;
      if (Math.abs(deviation) > centerTol)
        add('error', 'CENTER_DISTANCE',
          `中心距冲突：${gname(ga)}—${gname(gb)} 应为 ${expected.toFixed(3)} mm，实际 ${actual.toFixed(3)} mm，偏差 ${deviation >= 0 ? '+' : ''}${deviation.toFixed(3)} mm`,
          { mesh: e.id });
    }
    const key = [sa, sb].sort().join('|');
    if (pairSeen.has(key))
      add('error', 'DUPLICATE', `重复约束：轴「${sname(shafts[sa])}」与「${sname(shafts[sb])}」之间已有一处啮合`, { mesh: e.id });
    else pairSeen.set(key, e.id);

    if (Number.isInteger(ga.z) && Number.isInteger(gb.z) && ga.z > 0 && gb.z > 0) {
      const f = fr((internal ? 1 : -1) * ga.z, gb.z);
      adj.get(sa).push({ to: sb, f, mesh: e.id });
      adj.get(sb).push({ to: sa, f: fInv(f), mesh: e.id });
    }
    edges.push({ mesh: e.id, shaftA: sa, shaftB: sb, gearA: ga.id, gearB: gb.id,
                 internal, expected, actual, deviation });
  }

  for (const r of st.coaxRelations) {
    const A = shafts[r.shaftA], B = shafts[r.shaftB];
    if (!A || !B) { add('error', 'BROKEN_COAX', '存在指向已删除轴的同轴约束', { coax: r.id }); continue; }
    const d = Math.hypot(A.x - B.x, A.y - B.y);
    if (d > centerTol)
      add('warning', 'COAX_OFFSET', `同轴关系要求「${sname(A)}」与「${sname(B)}」同心，当前相距 ${d.toFixed(3)} mm`, { coax: r.id });
  }

  /* ---------------- 行星轮系 ---------------- */
  const planetInfos = [];
  for (const p of (st.planets || [])) {
    const pid = p.id;
    const nm = p.name || `行星级 ${pid}`;
    const pissues = [];
    const padd = (severity, code, message) => { add(severity, code, message, { planet: pid }); pissues.push(code); };
    if (!Number.isInteger(p.count) || p.count < 2 || p.count > 6)
      padd('error', 'PLANET_COUNT', `「${nm}」行星轮数量必须为 2～6 个`);
    for (const [key, label] of [['zS', '太阳轮齿数'], ['zP', '行星轮齿数'], ['zR', '内齿圈齿数']]) {
      if (!Number.isInteger(p[key]) || p[key] <= 0)
        padd('error', 'BAD_PLANET', `「${nm}」${label}必须为正整数`);
    }
    if (!(p.module > 0)) padd('error', 'BAD_PLANET', `「${nm}」模数必须为正数`);
    const roles = [p.fixed, p.input, p.output];
    if (roles.some(r => !P_MEMBERS.includes(r)) || new Set(roles).size !== 3)
      padd('error', 'BAD_PLANET_ROLE', `「${nm}」固定件、输入件、输出件必须分别指定为太阳轮/内齿圈/行星架且互不相同`);

    const G = planetGeom(p);
    if (G) {
      if (p.zR !== p.zS + 2 * p.zP)
        padd('error', 'WILLIS_GEOM',
          `「${nm}」齿数不满足 z圈 = z太阳 + 2z行星：${p.zR} ≠ ${p.zS} + 2×${p.zP} = ${p.zS + 2 * p.zP}`);
      if (Number.isInteger(p.count) && p.count >= 2 && p.count <= 6 && p.zR === p.zS + 2 * p.zP) {
        if ((p.zS + p.zR) % p.count !== 0)
          padd('error', 'ASSEMBLY',
            `「${nm}」均布装配条件不满足：(z太阳+z圈)/n = (${p.zS}+${p.zR})/${p.count} 非整数`);
        const net = G.chord - p.module * (p.zP + 2);
        if (net <= 0)
          padd('error', 'PLANET_COLLIDE',
            `「${nm}」相邻行星轮顶圆干涉：轴间距 ${G.chord.toFixed(2)} mm，需大于齿顶圆直径 ${(p.module * (p.zP + 2)).toFixed(2)} mm`);
        else if (net < 0.15 * p.module)
          padd('warning', 'PLANET_TIGHT', `「${nm}」行星轮净距仅 ${net.toFixed(2)} mm（< 0.15m），加工后易蹭齿`);
        if (G.sunTip >= G.ringTip)
          padd('error', 'COAX_CONFLICT',
            `「${nm}」同轴尺寸冲突：太阳轮齿顶圆 ${G.sunTip.toFixed(2)} mm 已触及内齿圈齿顶圆 ${G.ringTip.toFixed(2)} mm`);
      }
    }

    // 接入轴
    const att = {};
    for (const [mem, key] of [['s', 'sunShaftId'], ['r', 'ringShaftId'], ['c', 'carrierShaftId']]) {
      const sid = p[key];
      if (sid != null) {
        if (!shafts[sid]) add('error', 'BROKEN_PLANET_SHAFT', `「${nm}」${P_LABEL[mem]}接入的轴已被删除`, { planet: pid });
        else {
          att[mem] = sid;
          const A = shafts[sid];
          const d = Math.hypot(A.x - (p.x || 0), A.y - (p.y || 0));
          if (G && d > 0.05)
            add('warning', 'PLANET_OFFSET',
              `「${nm}」${P_LABEL[mem]}接入的轴「${sname(A)}」偏离行星级中心 ${d.toFixed(2)} mm`, { planet: pid, shaft: sid });
        }
      }
    }
    if (G) {
      for (const mem of ['s', 'c']) {
        const sid = att[mem];
        if (!sid) continue;
        for (const g of Object.values(gears)) {
          if (g.shaftId !== sid || g.internal || !(g.module > 0) || !Number.isInteger(g.z)) continue;
          const out = g.module * (g.z + 2) / 2;
          if (out >= G.ringTip - p.module)
            add('error', 'COAX_CONFLICT',
              `「${nm}」轴「${sname(shafts[sid])}」上的「${gname(g)}」齿顶圆半径 ${out.toFixed(2)} mm 超过内齿圈内腔 ${G.ringTip.toFixed(2)} mm`,
              { planet: pid, shaft: sid, gear: g.id });
        }
      }
    }

    // 转速图：伪节点 + Willis 边 + 接轴边
    const ns = pNode(pid, 's'), nr = pNode(pid, 'r'), nc = pNode(pid, 'c');
    const nodes = { s: ns, r: nr, c: nc };
    [ns, nr, nc].forEach(n => { if (!adj.has(n)) adj.set(n, []); });
    const wadd = (a, b, f) => {
      adj.get(a).push({ to: b, f, mesh: 'willis:' + pid });
      adj.get(b).push({ to: a, f: fInv(f), mesh: 'willis:' + pid });
    };
    if (G && !pissues.includes('BAD_PLANET_ROLE')) {
      if (p.fixed === 'r') wadd(nc, ns, fr(p.zS + p.zR, p.zS));
      else if (p.fixed === 's') wadd(nc, nr, fr(p.zS + p.zR, p.zR));
      else if (p.fixed === 'c') wadd(ns, nr, fr(-p.zS, p.zR));
    }
    for (const [mem, key] of [['s', 'sunShaftId'], ['r', 'ringShaftId'], ['c', 'carrierShaftId']]) {
      const sid = p[key];
      if (shafts[sid]) {
        adj.get(nodes[mem]).push({ to: sid, f: fr(1), mesh: 'pllink:' + pid });
        adj.get(sid).push({ to: nodes[mem], f: fr(1), mesh: 'pllink:' + pid });
      }
    }
    planetInfos.push({ id: pid, p, geom: G, nodes, att, speeds: {} });
  }

  const speeds = new Map();
  const depth = new Map();
  const edgeLevel = new Map();
  const conflicts = new Set();

  const nodeLabel = node => {
    const sp = splitPNode(node);
    return sp ? `行星级 ${sp.pid} 的${P_LABEL[sp.mem] || node}` : sname(shafts[node]);
  };
  const bfs = (seed, val) => {
    if (!adj.has(seed)) return;
    if (speeds.has(seed)) {
      const old = speeds.get(seed);
      if (old.n !== val.n || old.d !== val.d)
        add('error', 'LOCKED_TRAIN', `输入件与固定件被连成一体（${nodeLabel(seed)} 同时被要求转速 1 和 0），轮系锁死`,
          splitPNode(seed) ? { planet: splitPNode(seed).pid } : { shaft: seed });
      return;
    }
    speeds.set(seed, val);
    depth.set(seed, 0);
    const q = [seed];
    while (q.length) {
      const cur = q.shift();
      for (const nb of adj.get(cur)) {
        const v = fMul(speeds.get(cur), nb.f);
        if (speeds.has(nb.to)) {
          const old = speeds.get(nb.to);
          if (old.n !== v.n || old.d !== v.d) {
            if (conflicts.has(nb.mesh)) continue;
            conflicts.add(nb.mesh);
            const sameSign = (old.n > 0n) === (v.n > 0n);
            const ref = {};
            if (nb.mesh.startsWith('willis:') || nb.mesh.startsWith('pllink:')) ref.planet = nb.mesh.split(':')[1];
            else ref.mesh = nb.mesh;
            if (!isPNode(nb.to)) ref.shaft = nb.to;
            add('error', sameSign ? 'RATIO_CONFLICT' : 'DIRECTION_CONFLICT',
              sameSign
                ? `传动比矛盾：${nodeLabel(nb.to)} 经两条路径推得 ${fStr(old)} 与 ${fStr(v)}`
                : `转向矛盾：${nodeLabel(nb.to)} 经两条路径转向相反（${fStr(old)} 与 ${fStr(v)}），闭合轮系齿数不满足约束`,
              ref);
          }
        } else {
          speeds.set(nb.to, v);
          depth.set(nb.to, depth.get(cur) + 1);
          edgeLevel.set(nb.mesh, depth.get(nb.to));
          q.push(nb.to);
        }
      }
    }
  };
  for (const p of (st.planets || []))
    if (P_MEMBERS.includes(p.fixed)) bfs(pNode(p.id, p.fixed), fr(0));
  if (st.inputId) bfs(st.inputId, fr(1));
  else add('info', 'NO_INPUT', '尚未指定输入轴（在轴属性或行星级编辑器中设置）');

  for (const s of st.shafts) {
    if (!speeds.has(s.id) && st.gears.some(g => g.shaftId === s.id))
      add('warning', 'IDLE_SHAFT', `轴「${sname(s)}」未连入输入轴的动力链`, { shaft: s.id });
  }
  if (st.outputId && !speeds.has(st.outputId)) add('warning', 'NO_OUTPUT_PATH', '输出轴无法从输入轴到达');
  if (!st.outputId) add('info', 'NO_OUTPUT', '尚未指定输出轴（在轴属性或行星级编辑器中设置）');

  const ratio = st.outputId ? speeds.get(st.outputId) || null : null;
  const rpm = st.inputRpm || 1;
  const rpms = {};
  for (const [sid, v] of speeds) rpms[sid] = fNum(v) * rpm;

  /* 行星轮自转 n_p = n_c − (zS/zP)(n_s − n_c)，并回填成员信息 */
  for (const info of planetInfos) {
    for (const mem of P_MEMBERS) {
      const node = info.nodes[mem];
      info.speeds[mem] = speeds.has(node) ? speeds.get(node) : null;
    }
    const ns = info.speeds.s, nc = info.speeds.c;
    // n_p = n_c − (zS/zP)(n_s − n_c)
    info.spin = (ns && nc && info.geom)
      ? fSub(nc, fMul(fr(info.p.zS, info.p.zP), fSub(ns, nc)))
      : null;
  }

  let L = 1n;
  for (const g of st.gears) {
    const v = speeds.get(g.shaftId);
    if (v && Number.isInteger(g.z)) L = bLcm(L, fMul(fr(g.z), v).d);
  }
  for (const info of planetInfos) {
    if (!info.geom) continue;
    if (info.speeds.s) L = bLcm(L, fMul(fr(info.p.zS), info.speeds.s).d);
    if (info.speeds.r) L = bLcm(L, fMul(fr(info.p.zR), info.speeds.r).d);
    if (info.spin) L = bLcm(L, fMul(fr(info.p.zP), info.spin).d);
    // 均布行星轴位恢复：行星架转 n_c·L 须为 1/n 的整数倍，即 n·n_c·L 为整数
    if (info.speeds.c && Number.isInteger(info.p.count))
      L = bLcm(L, fMul(fr(info.p.count), info.speeds.c).d);
  }
  const shaftTurns = {};
  for (const [sid, v] of speeds) shaftTurns[sid] = { s: fStr(fMul(fr(L), v)), v: Number(L) * fNum(v) };
  const cycle = speeds.size ? { inputTurns: L.toString(), inputTurnsV: Number(L), shaftTurns } : null;

  const levels = [];
  for (const [mid, lv] of edgeLevel) {
    while (levels.length < lv) levels.push([]);
    levels[lv - 1].push(mid);
  }

  return {
    ok: !issues.some(i => i.severity === 'error'),
    issues,
    speeds: Object.fromEntries([...speeds].map(([k, v]) => [k, { s: fStr(v), v: fNum(v) }])),
    rpms, ratio: ratio ? { s: fStr(ratio), v: fNum(ratio) } : null,
    cycle, edges, levels, planets: planetInfos,
  };
}

/* ---------------- 示例 ---------------- */
function sampleState() {
  const st = emptyState();
  st.shafts = [
    { id: 's1', name: '输入轴', x: 0, y: 0, locked: false },
    { id: 's2', name: '中间轴', x: 48, y: 0, locked: false },
    { id: 's3', name: '输出轴', x: 88, y: 0, locked: false },
  ];
  st.gears = [
    { id: 'g1', shaftId: 's1', name: '主动轮', z: 24, module: 1, pressureAngle: 20, internal: false, locked: false },
    { id: 'g2', shaftId: 's2', name: '大轮', z: 72, module: 1, pressureAngle: 20, internal: false, locked: false },
    { id: 'g3', shaftId: 's2', name: '二级小轮', z: 20, module: 1, pressureAngle: 20, internal: false, locked: false },
    { id: 'g4', shaftId: 's3', name: '输出轮', z: 60, module: 1, pressureAngle: 20, internal: false, locked: false },
  ];
  st.meshes = [
    { id: 'm1', gearA: 'g1', gearB: 'g2' },
    { id: 'm2', gearA: 'g3', gearB: 'g4' },
  ];
  st.coaxRelations = [];
  st.planets = [];
  st.inputId = 's1';
  st.outputId = 's3';
  st.inputRpm = 60;
  return st;
}

/* ---------------- 渲染 ---------------- */
const layers = {
  rel: $('#layer-relations'), planets: $('#layer-planets'),
  prevBase: $('#layer-preview-baseline'),
  prev: $('#layer-preview'), drag: $('#layer-drag'),
  gears: $('#layer-gears'), shafts: $('#layer-shafts'),
};

function gearRadii(g) {
  const rp = g.module * g.z / 2;
  const ha = g.ha ?? 1, c = g.c ?? 0.25, x = g.x ?? 0;
  if (g.internal) return { rp, outer: g.module * (g.z / 2 + ha + c + x), tip: g.module * (g.z / 2 - ha + x), internal: true };
  return { rp, outer: g.module * (g.z / 2 + ha + x), root: Math.max(0.5, g.module * (g.z / 2 - ha - c + x)), internal: false };
}

function clearLayer(l) { while (l.firstChild) l.removeChild(l.firstChild); }

function selectedMeshBad() {
  return new Set(analysis.issues.filter(i => i.severity === 'error' && i.refs.mesh).map(i => i.refs.mesh));
}

function render() {
  analysis = analyzeLocal(state);
  motionTheta = motionTheta || 0;
  Object.values(layers).forEach(clearLayer);

  const S = shaftMap(), G = gearMap();
  const badMeshes = selectedMeshBad();

  /* 同轴关系 */
  for (const r of state.coaxRelations) {
    const A = S[r.shaftA], B = S[r.shaftB];
    if (!A || !B) continue;
    const g0 = svgEl('g', { class: 'coax', 'data-coax': r.id }, layers.rel);
    svgEl('line', { x1: A.x, y1: A.y, x2: B.x, y2: B.y,
      class: 'coax-ring' + (selection && selection.type === 'coax' && selection.id === r.id ? ' relation-selected' : ''),
      'data-coax': r.id, stroke: 'transparent', 'stroke-width': 3, 'pointer-events': 'stroke' }, g0);
    svgEl('line', { x1: A.x, y1: A.y, x2: B.x, y2: B.y,
      class: 'coax-ring' + (selection && selection.type === 'coax' && selection.id === r.id ? ' relation-selected' : ''),
      'data-coax': r.id, 'pointer-events': 'none' }, g0);
    for (const P of [A, B])
      svgEl('circle', { cx: P.x, cy: P.y, r: 3.4, class: 'coax-ring', 'data-coax': r.id }, g0);
  }

  /* 啮合线 */
  for (const e of state.meshes) {
    const ga = G[e.gearA], gb = G[e.gearB];
    if (!ga || !gb) continue;
    const A = S[ga.shaftId], B = S[gb.shaftId];
    if (!A || !B) continue;
    const internal = ga.internal || gb.internal;
    const cls = 'mesh-line' + (internal ? ' internal' : '') +
      (badMeshes.has(e.id) ? ' mesh-bad' : '') +
      (selection && selection.type === 'mesh' && selection.id === e.id ? ' relation-selected' : '');
    svgEl('line', { x1: A.x, y1: A.y, x2: B.x, y2: B.y, class: cls,
      'data-mesh': e.id, 'pointer-events': 'stroke', 'stroke-width': 1.1 }, layers.rel);
  }

  /* 齿轮（按轴分组为转子） */
  for (const s of state.shafts) {
    const sgears = state.gears.filter(g => g.shaftId === s.id)
      .sort((a, b) => gearRadii(b).outer - gearRadii(a).outer);
    if (!sgears.length) continue;
    const rotor = svgEl('g', { class: 'rotor', 'data-shaft': s.id }, layers.gears);
    for (const g of sgears) {
      const R = gearRadii(g);
      const gg = svgEl('g', { class: 'gear' + (g.locked ? ' gear-locked' : '') +
        (selection && selection.type === 'gear' && selection.id === g.id ? ' gear-selected' : '') }, rotor);
      if (g.internal) {
        svgEl('circle', { r: R.outer, class: 'gear-body internal', fill: 'url(#hatch)' }, gg);
        svgEl('circle', { r: R.tip, class: 'root-circle', fill: 'none', stroke: '#8d6e63', 'stroke-width': 0.35 }, gg);
      } else {
        svgEl('circle', { r: R.outer, class: 'gear-body', stroke: 'var(--gear)' }, gg);
        svgEl('circle', { r: R.root, fill: 'none', stroke: '#b9c6d8', 'stroke-width': 0.3 }, gg);
      }
      svgEl('circle', { r: R.rp, class: 'pitch-circle', stroke: g.internal ? '#8d6e63' : '#7d93b5', 'stroke-width': 0.25 }, gg);
      const tickN = Math.min(g.z, 48);
      for (let i = 0; i < tickN; i++) {
        const a = 2 * Math.PI * i / tickN;
        const r1 = g.internal ? R.tip + 0.3 : R.root,
              r2 = g.internal ? R.outer - 0.3 : R.outer;
        svgEl('line', {
          x1: Math.cos(a) * r1, y1: Math.sin(a) * r1,
          x2: Math.cos(a) * r2, y2: Math.sin(a) * r2,
          stroke: g.internal ? '#8d6e63' : '#9db1cc', 'stroke-width': 0.18,
        }, gg);
      }
      svgEl('circle', { r: R.outer + 2, class: 'gear-hit', 'data-gear': g.id,
        'pointer-events': 'all' }, gg);
      svgEl('text', { class: 'gear-label', y: R.outer + 4.2, text: g.name || `z${g.z}` }, gg);
    }
  }

  /* 轴标 */
  for (const s of state.shafts) {
    const g0 = svgEl('g', { class: 'shaft-static' + (s.locked ? ' shaft-locked' : ''),
      'data-shaft': s.id }, layers.shafts);
    const sgears = state.gears.filter(g => g.shaftId === s.id);
    const maxR = sgears.length ? Math.max(...sgears.map(g => gearRadii(g).outer)) : 4;
    const cls = 'shaft-peg' +
      (s.id === state.inputId ? ' shaft-input' : s.id === state.outputId ? ' shaft-output' : '');
    svgEl('circle', { r: 1.9, class: cls, 'data-shaft': s.id, 'pointer-events': 'all' }, g0);
    svgEl('circle', { r: 3.2, fill: 'none', stroke: 'transparent', 'stroke-width': 2,
      'data-shaft': s.id, 'pointer-events': 'all', style: 'cursor:pointer' }, g0);
    const label = `${s.name || s.id}${s.locked ? ' 🔒' : ''}`;
    svgEl('text', { class: 'shaft-label', y: -maxR - 2.2, text: label }, g0);
    if (selection && selection.type === 'shaft' && selection.id === s.id)
      svgEl('circle', { r: 3.6, fill: 'none', stroke: 'var(--hl)', 'stroke-width': 0.5 }, g0);
    g0.setAttribute('transform', `translate(${s.x} ${s.y})`);
  }

  renderPlanets();
  renderBaselineOverlay();
  renderCandidatePreview();
  applyWorldTransform();
  updateRotorTransforms();
  updatePlanetTransforms();
  renderPanels();
}

/* ---------------- 行星级 SVG ---------------- */
function gearTicks(parent, r1, r2, z, color) {
  const n = Math.min(z, 48);
  for (let i = 0; i < n; i++) {
    const a = 2 * Math.PI * i / n;
    svgEl('line', {
      x1: Math.cos(a) * r1, y1: Math.sin(a) * r1,
      x2: Math.cos(a) * r2, y2: Math.sin(a) * r2,
      stroke: color || '#9db1cc', 'stroke-width': 0.18,
    }, parent);
  }
}
function roleBadge(p, mem) {
  if (p.fixed === mem) return '🔒固定';
  if (p.input === mem && state.inputId === pNode(p.id, mem)) return '▶输入';
  if (p.output === mem && state.outputId === pNode(p.id, mem)) return '⬇输出';
  return '';
}

function renderPlanets() {
  clearLayer(layers.planets);
  for (const p of state.planets || []) {
    const G = planetGeom(p);
    const sel = selection && selection.type === 'planet' && selection.id === p.id;
    const grp = svgEl('g', { class: 'planet-stage' + (sel ? ' planet-selected' : ''),
      'data-planet': p.id }, layers.planets);
    svgEl('text', { class: 'planet-title', y: (G ? -G.ringOuter - 6 : -20),
      text: p.name || `行星级 ${p.id}`, 'data-planet-hit': p.id }, grp);

    if (!G) {
      svgEl('circle', { r: 10, class: 'planet-badring', 'data-planet-hit': p.id }, grp);
      svgEl('text', { class: 'planet-warn', y: 18, text: '齿数/模数无效', 'data-planet-hit': p.id }, grp);
      grp.setAttribute('transform', `translate(${p.x || 0} ${p.y || 0})`);
      continue;
    }
    const count = Math.max(2, Math.min(6, p.count | 0));
    const phase = (p.phase || 0) * Math.PI / 180;

    /* 内齿圈：外圆不透明，遮住后方背景轮系；齿顶圆内为空腔 */
    const ring = svgEl('g', { class: 'pl-ring', 'data-pl-rotor': 'r' }, grp);
    svgEl('circle', { r: G.ringOuter, class: 'pl-ring-outer',
      fill: 'var(--bg)', 'data-planet-hit': p.id }, ring);
    svgEl('circle', { r: G.ringTip, class: 'pl-ring-tip', 'data-planet-hit': p.id }, ring);
    svgEl('circle', { r: G.m * p.zR / 2, class: 'pitch-circle pl-pitch',
      stroke: 'var(--gear-in)', 'data-planet-hit': p.id }, ring);
    gearTicks(ring, G.ringTip + 0.3, G.ringOuter - 0.3, p.zR, '#b08968');

    /* 行星轴轨迹 */
    svgEl('circle', { r: G.orbit, class: 'pl-orbit', 'data-planet-hit': p.id }, grp);

    /* 行星架（含均布的行星轴与行星轮；行星轮作为架的子节点，
       公转天然与架同相同步，自转由内部 data-pl-planet-spin 组承担） */
    const carrier = svgEl('g', { class: 'pl-carrier', 'data-pl-rotor': 'c' }, grp);
    for (let i = 0; i < count; i++) {
      const a = phase + 2 * Math.PI * i / count;
      const ax = Math.cos(a) * G.orbit, ay = Math.sin(a) * G.orbit;
      svgEl('line', { x1: 0, y1: 0, x2: ax, y2: ay, class: 'pl-arm' }, carrier);
    }

    /* 太阳轮 */
    const sun = svgEl('g', { class: 'pl-sun', 'data-pl-rotor': 's' }, grp);
    const sunOutR = G.m * (p.zS + 2) / 2, sunRoot = Math.max(0.5, G.m * (p.zS - 2.5) / 2);
    svgEl('circle', { r: sunOutR, class: 'pl-sun-body', 'data-planet-hit': p.id }, sun);
    svgEl('circle', { r: sunRoot, fill: 'none', stroke: '#b9c6d8', 'stroke-width': 0.3 }, sun);
    svgEl('circle', { r: G.m * p.zS / 2, class: 'pitch-circle pl-pitch',
      stroke: '#7d93b5', 'data-planet-hit': p.id }, sun);
    gearTicks(sun, sunRoot, sunOutR, p.zS);
    svgEl('line', { x1: -sunOutR * 0.8, y1: 0, x2: sunOutR * 0.8, y2: 0,
      class: 'pl-spinmark', stroke: 'var(--gear)', 'stroke-width': 0.5 }, sun);

    /* 行星轮（位于架旋转组内：translate 到轴位，内部再按自转旋转） */
    for (let i = 0; i < count; i++) {
      const a = phase + 2 * Math.PI * i / count;
      const ax = Math.cos(a) * G.orbit, ay = Math.sin(a) * G.orbit;
      const pOrbit = svgEl('g', { class: 'pl-planet-orbit', 'data-pl-planet': i }, carrier);
      pOrbit.setAttribute('transform', `translate(${ax} ${ay})`);
      const pSpin = svgEl('g', { class: 'pl-planet-spin', 'data-pl-planet-spin': i }, pOrbit);
      const pOut = G.m * (p.zP + 2) / 2, pRoot = Math.max(0.5, G.m * (p.zP - 2.5) / 2);
      svgEl('circle', { r: pOut, class: 'pl-planet-body', 'data-planet-hit': p.id }, pSpin);
      svgEl('circle', { r: pRoot, fill: 'none', stroke: '#b9c6d8', 'stroke-width': 0.3 }, pSpin);
      svgEl('circle', { r: G.rp, class: 'pitch-circle', stroke: '#7d93b5' }, pSpin);
      gearTicks(pSpin, pRoot, pOut, p.zP);
      svgEl('line', { x1: -pOut * 0.8, y1: 0, x2: pOut * 0.8, y2: 0,
        class: 'pl-spinmark', stroke: 'var(--gear)', 'stroke-width': 0.5 }, pSpin);
    }
    /* 行星轴销（随架公转的轴点）与中心销 */
    for (let i = 0; i < count; i++) {
      const a = phase + 2 * Math.PI * i / count;
      svgEl('circle', { cx: Math.cos(a) * G.orbit, cy: Math.sin(a) * G.orbit,
        r: 1.3, class: 'pl-axle' }, carrier);
    }
    svgEl('circle', { r: 1.8, class: 'pl-axle pl-center' }, grp);

    /* 角色徽标 */
    const badgeR = G.ringOuter;
    const mkBadge = (mem, x, y) => {
      const b = roleBadge(p, mem);
      if (b) svgEl('text', { class: 'pl-badge pl-badge-' + mem, x, y, text: b }, grp);
    };
    mkBadge('s', 0, 0);
    mkBadge('r', 0, badgeR - 2);
    mkBadge('c', G.orbit * 0.5, -2.6);

    grp.setAttribute('transform', `translate(${p.x || 0} ${p.y || 0})`);
  }

  renderPlanetCandidateOverlay();
}

function renderPlanetCandidateOverlay() {
  const st = planetPreviewState();
  const c = planetCandidates[selectedPlanetPreview];
  if (!st || !c) return;
  const target0 = byId(state.planets, selectedPlanetId) || state.planets[0];
  const p = target0 && byId(st.planets, target0.id);
  if (!p || !planetGeom(p)) return;
  const G = planetGeom(p);
  const g0 = svgEl('g', { class: 'pl-cand-overlay',
    transform: `translate(${p.x || 0} ${p.y || 0})` }, layers.prev);
  svgEl('circle', { r: G.m * p.zR / 2, class: 'pl-cand-ring' }, g0);
  svgEl('circle', { r: G.orbit, class: 'pl-cand-orbit' }, g0);
  svgEl('circle', { r: G.m * p.zS / 2, class: 'pl-cand-sun' }, g0);
  const phase = (p.phase || 0) * Math.PI / 180;
  for (let i = 0; i < p.count; i++) {
    const a = phase + 2 * Math.PI * i / p.count;
    svgEl('circle', { cx: Math.cos(a) * G.orbit, cy: Math.sin(a) * G.orbit,
      r: G.rp, class: 'pl-cand-planet' }, g0);
  }
  svgEl('text', { class: 'pl-cand-label', y: -G.ringOuter - 6,
    text: `候选预览 i=${c.ratio.s}（m=${c.module} z${c.zS}/${c.zP}×${c.count}/${c.zR}）` }, g0);
}

function updatePlanetTransforms() {
  const prevSt = planetPreviewState();
  const st = prevSt || candidatePreviewState() || state;
  const an = analyzeLocal(st);
  const theta = motionTheta || 0;
  for (const info of an.planets || []) {
    const p = byId(st.planets, info.id);
    if (!p) continue;
    const grp = document.querySelector(`#layer-planets [data-planet="${p.id}"]`);
    if (!grp) continue;
    grp.setAttribute('transform', `translate(${p.x || 0} ${p.y || 0})`);
    const ang = mem => {
      const sp = info.speeds[mem];   // 原始 BigInt 分数 {n,d}
      return sp ? fNum(sp) * theta : 0;
    };
    const rNode = grp.querySelector('[data-pl-rotor="r"]');
    const sNode = grp.querySelector('[data-pl-rotor="s"]');
    const cNode = grp.querySelector('[data-pl-rotor="c"]');
    if (rNode) rNode.setAttribute('transform', `rotate(${ang('r')})`);
    if (sNode) sNode.setAttribute('transform', `rotate(${ang('s')})`);
    if (cNode) cNode.setAttribute('transform', `rotate(${ang('c')})`);
    // 行星轮自转子组嵌套在公转的行星架组内，须用相对架的自转 n_p − n_c，
    // 使其在固定坐标系中的绝对姿态恰好为 Willis 给出的 n_p。
    const carrierV = info.speeds.c ? fNum(info.speeds.c) : 0;
    const spinRel = info.spin ? (fNum(info.spin) - carrierV) * theta : 0;
    grp.querySelectorAll('[data-pl-planet-spin]').forEach(n =>
      n.setAttribute('transform', `rotate(${spinRel})`));
  }
}

function renderBaselineOverlay() {
  if (!baseline) return;
  const S = Object.fromEntries(baseline.state.shafts.map(s => [s.id, s]));
  for (const g of baseline.state.gears) {
    const s = S[g.shaftId];
    if (!s || !(g.module > 0) || !Number.isInteger(g.z)) continue;
    const rp = g.module * g.z / 2;
    const grp = svgEl('g', { transform: `translate(${s.x} ${s.y})` }, layers.prevBase);
    svgEl('circle', { r: rp, class: 'pitch-circle preview-baseline' }, grp);
  }
  for (const p of baseline.state.planets || []) {
    const G = planetGeom(p);
    if (!G) continue;
    const grp = svgEl('g', { transform: `translate(${p.x || 0} ${p.y || 0})` }, layers.prevBase);
    svgEl('circle', { r: G.m * p.zR / 2, class: 'pitch-circle preview-baseline' }, grp);
    svgEl('circle', { r: G.orbit, class: 'pitch-circle preview-baseline' }, grp);
    svgEl('circle', { r: G.m * p.zS / 2, class: 'pitch-circle preview-baseline' }, grp);
    const phase = (p.phase || 0) * Math.PI / 180;
    for (let i = 0; i < p.count; i++) {
      const a = phase + 2 * Math.PI * i / p.count;
      svgEl('circle', { cx: Math.cos(a) * G.orbit, cy: Math.sin(a) * G.orbit,
        r: G.rp, class: 'pitch-circle preview-baseline' }, grp);
    }
  }
}

function candidatePreviewState() {
  if (selectedCand < 0 || !candidates[selectedCand]) return null;
  if (previewCache && previewCache.ver === stateVersion && previewCache.idx === selectedCand)
    return previewCache.st;
  const c = candidates[selectedCand];
  const st = JSON.parse(JSON.stringify(state));
  for (const g of st.gears) if (c.gears[g.id]) Object.assign(g, c.gears[g.id]);
  for (const s of st.shafts) if (c.shafts[s.id]) { s.x = c.shafts[s.id].x; s.y = c.shafts[s.id].y; }
  previewCache = { ver: stateVersion, idx: selectedCand, st };
  return st;
}

function renderCandidatePreview() {
  const st = candidatePreviewState();
  if (!st) return;
  const S = Object.fromEntries(st.shafts.map(s => [s.id, s]));
  const curS = shaftMap();

  /* 旧节圆（红虚线，当前位置）+ 新节圆（蓝，随轴旋转） */
  const oldLayer = svgEl('g', {}, layers.prev);
  for (const g0 of state.gears) {
    if (!candidates[selectedCand].gears[g0.id]) continue;
    const s = curS[g0.shaftId];
    const rp = g0.module * g0.z / 2;
    const grp = svgEl('g', { transform: `translate(${s.x} ${s.y})` }, oldLayer);
    svgEl('circle', { r: rp, class: 'pitch-circle preview-old' }, grp);
  }
  for (const s of st.shafts) {
    const sgears = st.gears.filter(g => g.shaftId === s.id);
    if (!sgears.length) continue;
    const rotor = svgEl('g', { class: 'preview-rotor', 'data-shaft': s.id }, layers.prev);
    for (const g of sgears) {
      const R = gearRadii(g);
      const changed = !!candidates[selectedCand].gears[g.id];
      if (changed)
        svgEl('circle', { r: R.outer, fill: 'rgba(91,141,239,.08)', stroke: 'var(--accent)', 'stroke-width': 0.35 }, rotor);
      svgEl('circle', { r: R.rp, class: 'pitch-circle', stroke: changed ? 'var(--accent)' : '#9db1cc',
        'stroke-width': changed ? 0.4 : 0.25 }, rotor);
    }
    rotor.setAttribute('transform', `translate(${s.x} ${s.y})`);
  }
}

function applyWorldTransform() {
  $('#world').setAttribute('transform',
    `translate(${view.panX} ${view.panY}) scale(${SCALE * view.zoom})`);
}
function updateRotorTransforms() {
  const previewOn = !!candidatePreviewState();
  const sel = previewOn ? '.preview-rotor' : '.rotor';
  const st = previewOn ? candidatePreviewState() : state;
  const an = previewOn ? analyzeLocal(st) : analysis;
  const theta = motionTheta || 0;
  document.querySelectorAll(sel).forEach(node => {
    const sid = node.getAttribute('data-shaft');
    const s = byId(st.shafts, sid);
    if (!s) return;
    const sp = an.speeds[sid];
    const a = sp ? sp.v * theta : 0;
    node.setAttribute('transform', `translate(${s.x} ${s.y}) rotate(${a})`);
  });
}

/* ---------------- 面板：分析 ---------------- */
function renderPanels() {
  renderAnalysisPanel();
  renderProps();
  renderPlanetPanel();
  renderSearchPanel();
}

function renderAnalysisPanel() {
  const box = $('#ratio-box');
  if (analysis.ratio) {
    const r = analysis.ratio, inv = fInv(parseFr(r.s));
    const sign = r.v >= 0 ? '同向（正转）' : '反向（反转）';
    box.classList.toggle('bad', !analysis.ok);
    box.innerHTML =
      `<div>总传动比 <b>i = n输出/n输入 = <span class="big">${r.s}</span></b>` +
      ` ≈ ${r.v.toFixed(6)}</div>` +
      `<div>减速比 n输入/n输出 = ${fStr(inv)}；输出轴与输入轴${sign}</div>`;
  } else {
    box.classList.add('bad');
    box.innerHTML = `<div class="big">—</div><div>指定输入轴与输出轴后显示总传动比</div>`;
  }

  const errs = analysis.issues.filter(i => i.severity === 'error');
  const warns = analysis.issues.filter(i => i.severity === 'warning');
  const badge = $('#issue-count');
  badge.textContent = errs.length ? `${errs.length} 错 / ${warns.length} 警` : warns.length ? `${warns.length} 警` : '全部正常';
  badge.className = 'badge ' + (errs.length ? 'err' : warns.length ? 'warn' : '');

  const ul = $('#issue-list');
  ul.innerHTML = '';
  if (!analysis.issues.length) {
    h('li', { class: 'info' }, ul, '✓ 未发现问题');
  }
  for (const it of analysis.issues) {
    const li = h('li', { class: it.severity }, ul);
    h('span', { class: 'sev' }, li,
      it.severity === 'error' ? '✗ 错误' : it.severity === 'warning' ? '⚠ 警告' : 'ℹ 提示');
    li.appendChild(document.createTextNode(it.message));
    li.addEventListener('click', () => focusIssue(it));
  }

  const head = $('#cycle-head');
  if (analysis.cycle) {
    const L = analysis.cycle.inputTurns;
    head.textContent = `整列恢复初始啮合：输入转 ${L} 转（各轴/齿轮与行星轮转过的齿数同时为整数）`;
  } else head.textContent = '';

  const tb = $('#shaft-table tbody');
  tb.innerHTML = '';
  const trh = h('tr', {}, tb);
  ['轴', '相对转速', '转速 rpm', `循环内转数`, '方向'].forEach(t => h('th', {}, trh, t));
  for (const s of state.shafts) {
    const sp = analysis.speeds[s.id];
    const tr = h('tr', { style: 'cursor:pointer' }, tb);
    tr.addEventListener('click', () => { selection = { type: 'shaft', id: s.id }; render(); renderProps(); });
    h('td', {}, tr, (s.name || s.id) + (s.locked ? ' 🔒' : ''));
    if (sp) {
      h('td', {}, tr, sp.s);
      h('td', {}, tr, analysis.rpms[s.id].toFixed(2));
      const ct = analysis.cycle.shaftTurns[s.id];
      h('td', {}, tr, `${ct.s}`);
      h('td', { class: sp.v >= 0 ? 'cw' : 'ccw' }, tr, sp.v >= 0 ? '↻ 正' : '↺ 反');
    } else h('td', { colspan: 4, class: 'muted' }, tr, '未连入动力链');
  }

  const ps = $('#planet-speeds');
  ps.innerHTML = '';
  for (const info of analysis.planets || []) {
    if (!info.geom) continue;
    const p = byId(state.planets, info.id);
    h('h3', { style: 'cursor:pointer' }, ps,
      `${p.name || '行星级 ' + p.id}（×${p.count} 只 z${p.zP} 行星轮）`).onclick =
      () => { selection = { type: 'planet', id: p.id }; selectedPlanetId = p.id; render(); renderPlanetPanel(); };
    const tbl = h('table', { class: 'shaft-table' }, ps);
    const body = h('tbody', {}, tbl);
    const rows = [
      ['太阳轮', info.speeds.s],
      ['内齿圈', info.speeds.r],
      ['行星架（公转）', info.speeds.c],
      ['行星轮（自转）', info.spin],
    ];
    for (const [label, sp] of rows) {
      const tr = h('tr', {}, body);
      h('td', {}, tr, label);
      if (sp) {
        const node = label === '太阳轮' ? pNode(p.id, 's') : label === '内齿圈' ? pNode(p.id, 'r')
          : label === '行星架（公转）' ? pNode(p.id, 'c') : null;
        const sv = fNum(sp);
        h('td', {}, tr, fStr(sp));
        h('td', {}, tr, node && analysis.rpms[node] != null ? analysis.rpms[node].toFixed(2) : '—');
        let turns = '—';
        if (node && analysis.cycle.shaftTurns[node]) turns = analysis.cycle.shaftTurns[node].s;
        else if (label === '行星轮（自转）')
          turns = fStr(fMul(fr(analysis.cycle.inputTurns), sp));
        h('td', {}, tr, turns);
        h('td', { class: sv >= 0 ? 'cw' : 'ccw' }, tr, sv >= 0 ? '↻ 正' : '↺ 反');
      } else h('td', { colspan: 4, class: 'muted' }, tr, '未连入动力链');
    }
  }
}

function focusIssue(it) {
  const refs = it.refs || {};
  if (refs.planet) {
    selection = { type: 'planet', id: refs.planet };
    selectedPlanetId = refs.planet;
    render(); renderPlanetPanel(); switchTab('planet');
    return;
  }
  if (refs.mesh) { selection = { type: 'mesh', id: refs.mesh }; render(); renderProps(); }
  else if (refs.gear) { selection = { type: 'gear', id: refs.gear }; render(); renderProps(); }
  else if (refs.shaft) { selection = { type: 'shaft', id: refs.shaft }; render(); renderProps(); }
  else if (refs.coax) { selection = { type: 'coax', id: refs.coax }; render(); renderProps(); }
}

/* ---------------- 面板：属性 ---------------- */
function labeledInput(body, label, value, oninput, type = 'number', extra = {}) {
  if (type === 'checkbox') {
    const f = h('div', { class: 'field checkbox-field' }, body);
    const inp = h('input', { type: 'checkbox', checked: !!value,
      onchange: e => oninput(e.target.checked) }, f);
    f.appendChild(document.createTextNode(label));
    return inp;
  }
  const f = h('div', { class: 'field' }, body);
  h('label', {}, f, label);
  const inp = h('input', Object.assign(
    { type, value: value == null ? '' : value,
      oninput: e => oninput(type === 'number' ? e.target.value : e.target.value) },
    extra), f);
  return inp;
}

function renderProps() {
  const body = $('#props-body');
  body.innerHTML = '';
  body.className = '';
  if (!selection) {
    body.className = 'muted';
    body.textContent = '在画布上选择轴、齿轮、啮合或同轴关系后在此编辑。';
    return;
  }
  const commit = () => { markDirty(); render(); };

  if (selection.type === 'shaft') {
    const s = byId(state.shafts, selection.id);
    if (!s) { selection = null; return renderProps(); }
    h('h3', {}, body, '轴');
    labeledInput(body, '名称', s.name, v => { s.name = v || null; markDirty(); render(); }, 'text');
    const row = h('div', { class: 'field-row' }, body);
    labeledInput(row, 'X (mm)', s.x, v => { s.x = parseFloat(v) || 0; commit(); });
    labeledInput(row, 'Y (mm)', s.y, v => { s.y = parseFloat(v) || 0; commit(); });
    labeledInput(body, '锁定轴位（搜索时不动）', s.locked, v => { s.locked = v; commit(); }, 'checkbox');
    const io = h('div', { class: 'io-btns' }, body);
    h('button', { class: s.id === state.inputId ? 'active' : '', onclick: () => {
      state.inputId = s.id; resetMotion(); commit();
    } }, io, s.id === state.inputId ? '✓ 输入轴' : '设为输入轴');
    h('button', { class: s.id === state.outputId ? 'active' : '', onclick: () => {
      state.outputId = s.id; commit();
    } }, io, s.id === state.outputId ? '✓ 输出轴' : '设为输出轴');
    if (s.id === state.inputId)
      labeledInput(body, '输入转速 rpm', state.inputRpm, v => { state.inputRpm = parseFloat(v) || 0; commit(); });
    h('button', { class: 'danger', onclick: () => deleteShaft(s.id) }, body, '删除该轴及其齿轮');
  } else if (selection.type === 'gear') {
    const g = byId(state.gears, selection.id);
    if (!g) { selection = null; return renderProps(); }
    h('h3', {}, body, '齿轮');
    labeledInput(body, '名称', g.name, v => { g.name = v || null; markDirty(); render(); }, 'text');
    labeledInput(body, '齿数 z（正整数）', g.z, v => { g.z = parseInt(v, 10); commit(); });
    labeledInput(body, '模数 m (mm)', g.module, v => { g.module = parseFloat(v) || 0; commit(); }, 'number', { step: 0.1 });
    labeledInput(body, '压力角 (°)', g.pressureAngle, v => { g.pressureAngle = parseFloat(v) || 0; commit(); }, 'number', { step: 1 });
    labeledInput(body, '内齿圈（内啮合）', g.internal, v => { g.internal = v; commit(); }, 'checkbox');
    labeledInput(body, '锁定（搜索时不替换）', g.locked, v => { g.locked = v; commit(); }, 'checkbox');
    const f = h('div', { class: 'field' }, body);
    h('label', {}, f, '安装到轴');
    const sel = h('select', { onchange: e => { g.shaftId = e.target.value; commit(); } }, f);
    for (const s of state.shafts) {
      const opt = h('option', { value: s.id }, sel, s.name || s.id);
      if (s.id === g.shaftId) opt.selected = true;
    }
    const rp = g.module * g.z / 2;
    h('p', { class: 'muted small' }, body, `节圆半径 r' = m·z/2 = ${rp.toFixed(2)} mm（${g.internal ? '内齿，节圆为齿圈分度圆' : '外齿'}）`);
    h('button', { class: 'danger', onclick: () => deleteGear(g.id) }, body, '删除该齿轮');
  } else if (selection.type === 'mesh') {
    const e = byId(state.meshes, selection.id);
    const ga = e && byId(state.gears, e.gearA), gb = e && byId(state.gears, e.gearB);
    if (!e) { selection = null; return renderProps(); }
    h('h3', {}, body, ga.internal || gb.internal ? '内啮合' : '外啮合');
    h('p', {}, body, `${ga.name || '齿轮 ' + ga.z} (z=${ga.z}, m=${ga.module}) ⚙ ${gb.name || '齿轮 ' + gb.z} (z=${gb.z}, m=${gb.module})`);
    const ei = analysis.edges.find(x => x.mesh === e.id);
    if (ei && ei.expected != null)
      h('p', { class: 'muted small' }, body,
        `理论中心距 ${ei.expected.toFixed(3)} mm；实际 ${ei.actual.toFixed(3)} mm；偏差 ${ei.deviation >= 0 ? '+' : ''}${ei.deviation.toFixed(3)} mm`);
    h('button', { class: 'danger', onclick: () => {
      state.meshes = state.meshes.filter(x => x.id !== e.id);
      selection = null; commit();
    } }, body, '删除啮合');
  } else if (selection.type === 'coax') {
    const r = byId(state.coaxRelations, selection.id);
    if (!r) { selection = null; return renderProps(); }
    const A = byId(state.shafts, r.shaftA), B = byId(state.shafts, r.shaftB);
    h('h3', {}, body, '同轴关系');
    h('p', {}, body, `「${A.name || A.id}」与「${B.name || B.id}」同位安装；两轴转速各自独立，齿轮可在轴间传递。`);
    h('button', { class: 'danger', onclick: () => {
      state.coaxRelations = state.coaxRelations.filter(x => x.id !== r.id);
      selection = null; commit();
    } }, body, '解除同轴');
  }
}

/* ---------------- 面板：行星轮系 ---------------- */
function renderPlanetPanel() {
  const sel = $('#pl-select');
  if (!sel) return;
  const cur = selectedPlanetId;
  sel.innerHTML = '';
  for (const p of state.planets || []) {
    const o = h('option', { value: p.id }, sel, p.name || `行星级 ${p.id}`);
    if (p.id === cur) o.selected = true;
  }
  if (!state.planets || !state.planets.length) { sel.disabled = true; } else sel.disabled = false;
  if (!selectedPlanetId && state.planets.length) selectedPlanetId = state.planets[0].id;
  if (cur && !byId(state.planets || [], cur))
    selectedPlanetId = state.planets[0]?.id || null;

  const body = $('#pl-body');
  body.innerHTML = '';
  body.classList.remove('muted');
  const p = byId(state.planets || [], selectedPlanetId);
  if (!p) {
    body.className = 'muted';
    body.textContent = '还没有行星级：在顶部切到「行星级」工具后点击画布空白处放置，或点「＋ 新行星级」。';
    return;
  }
  const recompute = () => { resetMotion(); markDirty(); render(); };
  h('h3', {}, body, p.name || `行星级 ${p.id}`);
  labeledInput(body, '名称', p.name, v => { p.name = v || null; markDirty(); render(); }, 'text');

  const r1 = h('div', { class: 'field-row' }, body);
  labeledInput(r1, '模数 m (mm)', p.module, v => { p.module = parseFloat(v) || 0; recompute(); }, 'number', { step: 0.1 });
  labeledInput(r1, '行星轮数量 n（2～6）', p.count, v => { p.count = Math.max(2, Math.min(6, parseInt(v, 10) || 2)); recompute(); });
  const r2 = h('div', { class: 'field-row' }, body);
  labeledInput(r2, '太阳轮齿数 zS', p.zS, v => { p.zS = parseInt(v, 10); recompute(); });
  labeledInput(r2, '行星轮齿数 zP', p.zP, v => { p.zP = parseInt(v, 10); recompute(); });
  labeledInput(r2, '内齿圈齿数 zR', p.zR, v => { p.zR = parseInt(v, 10); recompute(); });

  if (Number.isInteger(p.zS) && Number.isInteger(p.zP)) {
    const zWant = p.zS + 2 * p.zP;
    h('p', { class: 'small ' + (p.zR === zWant ? 'muted' : 'err-text') }, body,
      `Willis 几何要求 z圈 = z太阳 + 2z行星 = ${zWant}；` +
      (p.zR === zWant ? '当前满足。' : `当前 zR=${p.zR}，`),
    );
    if (p.zR !== zWant)
      h('button', { onclick: () => { p.zR = zWant; recompute(); } }, body, `将 zR 改为 ${zWant}`);
  }

  /* 固定/输入/输出角色：选择时自动对调，保证三者互不相同 */
  const attKey = { s: 'sunShaftId', r: 'ringShaftId', c: 'carrierShaftId' };
  const ownNode = mem => pNode(p.id, mem);
  const belongsToThis = id => P_MEMBERS.some(mem => id === ownNode(mem));
  // 角色改变后同步全局 IO：独立行星级直接指向构件；接入现有轴的成员指向真实轴；
  // 但绝不覆盖已有的外部轴输入（如把太阳轮接到原齿轮列输出轴后，整体从原输入轴算起）
  let syncGlobalIOForRole = () => {};
  const roleBox = h('div', { class: 'pl-roles' }, body);
  const roleRow = (role, label, val, setter) => {
    const f = h('div', { class: 'field' }, roleBox);
    h('label', {}, f, label);
    const s = h('select', {}, f);
    for (const mem of P_MEMBERS) {
      const o = h('option', { value: mem }, s, P_LABEL[mem]);
      if (val === mem) o.selected = true;
    }
    s.addEventListener('change', () => {
      const v = s.value;
      const others = { fixed: ['input', 'output'], input: ['fixed', 'output'], output: ['fixed', 'input'] }[role];
      if (p[others[0]] === v) p[others[0]] = p[role];
      else if (p[others[1]] === v) p[others[1]] = p[role];
      setter(v);
      syncGlobalIOForRole();
      recompute();
    });
  };
  roleRow('fixed', '固定件（速度 0）', p.fixed, v => { p.fixed = v; });
  roleRow('input', '输入件', p.input, v => { p.input = v; });
  roleRow('output', '输出件', p.output, v => { p.output = v; });
  // 角色改变时同步全局 IO：仅当全局 IO 未被既有齿轮列占用（独立行星级），
  // 或当前本就指向该行星级构件时才更新；接入轴后输入保持在原外部轴。
  syncGlobalIOForRole = () => {
    const inAtt = p[attKey[p.input]], outAtt = p[attKey[p.output]];
    if (state.inputId == null || belongsToThis(state.inputId))
      state.inputId = inAtt || ownNode(p.input);
    if (state.outputId == null || belongsToThis(state.outputId))
      state.outputId = outAtt || ownNode(p.output);
  };
  h('p', { class: 'muted small' }, body,
    '独立行星级：全局输入/输出随角色指向构件；把构件接入现有轴后，全局输入/输出保持在原轮系轴上，整体传动比自动串联重算。');

  /* 装配条件快览 */
  const G = planetGeom(p);
  if (G && Number.isInteger(p.count) && p.count >= 2 && p.count <= 6) {
    const assy = (p.zS + p.zR) % p.count === 0;
    const net = G.chord - p.module * (p.zP + 2);
    const facts = h('div', { class: 'pl-facts' }, body);
    h('div', { class: assy ? 'ok-text' : 'err-text' }, facts,
      `${assy ? '✓' : '✗'} 均布装配：(zS+zR)/n = ${p.zS + p.zR}/${p.count} = ${((p.zS + p.zR) / p.count).toFixed(2)}`);
    h('div', { class: net > 0 ? 'ok-text' : 'err-text' }, facts,
      `${net > 0 ? '✓' : '✗'} 相邻行星净距：${net.toFixed(2)} mm（轴间距 ${G.chord.toFixed(2)}，顶圆直径 ${(p.module * (p.zP + 2)).toFixed(2)}）`);
    h('div', { class: 'muted small' }, facts,
      `节圆：太阳 r=${(p.module * p.zS / 2).toFixed(2)}，行星 r=${G.rp.toFixed(2)}，` +
      `内齿圈 r=${(p.module * p.zR / 2).toFixed(2)}；行星轴轨迹半径 ${G.orbit.toFixed(2)}；外径约 ${(2 * G.ringOuter).toFixed(1)} mm`);
  }

  /* 初相位 */
  labeledInput(body, '初相位（°，第一只行星轮相对 X 轴）', p.phase || 0,
    v => { p.phase = parseFloat(v) || 0; markDirty(); render(); }, 'number', { step: 5 });

  /* 接入现有轴 */
  h('h3', {}, body, '接入现有轴');
  const attRow = (mem, key) => {
    const f = h('div', { class: 'field' }, body);
    h('label', {}, f, `${P_LABEL[mem]}与哪根轴同速（可空）`);
    const s = h('select', { onchange: e => {
      p[key] = e.target.value || null;
      // 接轴只建立同速链接，绝不抢占全局输入（否则把太阳轮接到原齿轮列输出轴后，
      // 会把该轴误当输入源，只显示级比 1/4）。输出成员接入时全局输出落到真实轴。
      if (p[key] && state.outputId === ownNode(mem)) state.outputId = p[key];
      recompute();
    } }, f);
    h('option', { value: '' }, s, '— 不接入 —');
    for (const sh of state.shafts) {
      const o = h('option', { value: sh.id }, s, sh.name || sh.id);
      if (p[key] === sh.id) o.selected = true;
    }
  };
  attRow('s', 'sunShaftId');
  attRow('r', 'ringShaftId');
  attRow('c', 'carrierShaftId');
  const attBtns = h('div', { class: 'field-row' }, body);
  h('button', { onclick: () => {
    // 吸附接入轴到行星级中心
    for (const key of ['sunShaftId', 'ringShaftId', 'carrierShaftId']) {
      const sh = p[key] && byId(state.shafts, p[key]);
      if (sh) { sh.x = p.x; sh.y = p.y; }
    }
    recompute();
  } }, attBtns, '吸附接入轴到中心');
  h('p', { class: 'muted small' }, body,
    '接入后若全局输入仍为原齿轮列的轴，整体传动比会从该轴串到行星级输出（如 1/9 × 1/4 = 1/36）。');

  const posRow = h('div', { class: 'field-row' }, body);
  labeledInput(posRow, '中心 X (mm)', p.x, v => { p.x = parseFloat(v) || 0; recompute(); });
  labeledInput(posRow, '中心 Y (mm)', p.y, v => { p.y = parseFloat(v) || 0; recompute(); });

  h('button', { class: 'danger', onclick: () => deletePlanet(p.id) }, body, '删除该行星级');
}

$('#pl-select')?.addEventListener('change', e => {
  selectedPlanetId = e.target.value;
  selection = { type: 'planet', id: selectedPlanetId };
  render();
});
$('#btn-pl-add')?.addEventListener('click', () => addPlanet(0, 0));

/* ---------------- 删除 / 放置 ---------------- */
function deleteShaft(sid) {
  const gids = new Set(state.gears.filter(g => g.shaftId === sid).map(g => g.id));
  state.gears = state.gears.filter(g => g.shaftId !== sid);
  state.meshes = state.meshes.filter(e => !gids.has(e.gearA) && !gids.has(e.gearB));
  state.coaxRelations = state.coaxRelations.filter(r => r.shaftA !== sid && r.shaftB !== sid);
  for (const p of state.planets || []) {
    for (const key of ['sunShaftId', 'ringShaftId', 'carrierShaftId'])
      if (p[key] === sid) p[key] = null;
    for (const mem of P_MEMBERS)
      if (state.inputId === pNode(p.id, mem) || state.outputId === pNode(p.id, mem)) break;
  }
  state.shafts = state.shafts.filter(s => s.id !== sid);
  if (state.inputId === sid) state.inputId = null;
  if (state.outputId === sid) state.outputId = null;
  selection = null;
  resetMotion();
  markDirty(); render();
}
function deleteGear(gid) {
  state.meshes = state.meshes.filter(e => e.gearA !== gid && e.gearB !== gid);
  state.gears = state.gears.filter(g => g.id !== gid);
  selection = null;
  markDirty(); render();
}

function addShaft(x, y) {
  const s = { id: uid('s'), name: '', x: +x.toFixed(2), y: +y.toFixed(2), locked: false };
  state.shafts.push(s);
  selection = { type: 'shaft', id: s.id };
  markDirty(); render();
  return s;
}
function addGearOn(sid) {
  const g = { id: uid('g'), shaftId: sid, name: '', z: 24, module: 1,
    pressureAngle: 20, internal: false, locked: false, ha: 1, c: 0.25, x: 0 };
  state.gears.push(g);
  selection = { type: 'gear', id: g.id };
  markDirty(); render();
}
function defaultPlanet(x, y) {
  return { id: uid('p'), name: '', module: 1, zS: 24, zP: 24, zR: 72,
    count: 3, phase: 0, fixed: 'r', input: 's', output: 'c',
    x: +x.toFixed(2), y: +y.toFixed(2),
    sunShaftId: null, ringShaftId: null, carrierShaftId: null };
}
function addPlanet(x, y) {
  const p = defaultPlanet(x, y);
  state.planets.push(p);
  selection = { type: 'planet', id: p.id };
  selectedPlanetId = p.id;
  // 放置时：只有全局 IO 未被既有轮系占用，才让新行星级充当动力源；
  // 已存在齿轮列时保留其输入轴，行星级经接轴串联。
  if (state.inputId == null) state.inputId = pNode(p.id, p.input);
  if (state.outputId == null) state.outputId = pNode(p.id, p.output);
  resetMotion();
  markDirty(); render();
  switchTab('planet');
  return p;
}
function deletePlanet(pid) {
  state.planets = state.planets.filter(p => p.id !== pid);
  for (const mem of P_MEMBERS) {
    const node = pNode(pid, mem);
    if (state.inputId === node) state.inputId = null;
    if (state.outputId === node) state.outputId = null;
  }
  if (selectedPlanetId === pid) selectedPlanetId = state.planets[0]?.id || null;
  if (selection && selection.type === 'planet' && selection.id === pid) selection = null;
  resetMotion();
  markDirty(); render();
}

/* ---------------- 指针交互 ---------------- */
const svg = $('#canvas');
let pointer = null;   // {action, ...}

function eventWorld(e) {
  const rect = svg.getBoundingClientRect();
  const u = SCALE * view.zoom;
  return {
    x: (e.clientX - rect.left - view.panX) / u,
    y: (e.clientY - rect.top - view.panY) / u,
    sx: e.clientX - rect.left, sy: e.clientY - rect.top,
  };
}
function mode() { return document.querySelector('input[name="mode"]:checked').value; }
function hitKind(node) {
  while (node && node !== svg) {
    if (node.dataset && node.dataset.gear) return { type: 'gear', id: node.dataset.gear };
    if (node.dataset && node.dataset.shaft) return { type: 'shaft', id: node.dataset.shaft };
    if (node.dataset && node.dataset.mesh) return { type: 'mesh', id: node.dataset.mesh };
    if (node.dataset && node.dataset.coax) return { type: 'coax', id: node.dataset.coax };
    if (node.dataset && node.dataset.planetHit) return { type: 'planet', id: node.dataset.planetHit };
    node = node.parentNode;
  }
  return null;
}

/* 松开时 e.target 在拖动中可能不是命中元素，用 elementFromPoint 兜底，
   并按优先顺序返回（关系线可点时不应被透明齿轮热区盖住）。 */
function hitAtPoint(clientX, clientY, prefer = null) {
  const direct = hitKind(document.elementFromPoint(clientX, clientY));
  if (!direct || !prefer) return direct;
  if (direct.type === prefer) return direct;
  const el = document.elementFromPoint(clientX, clientY);
  if (el && el.style) el.style.pointerEvents = 'none';
  const second = hitKind(document.elementFromPoint(clientX, clientY));
  if (el && el.style) el.style.pointerEvents = '';
  return second && second.type === prefer ? second : direct;
}

svg.addEventListener('pointerdown', e => {
  /* 不使用 setPointerCapture：捕获后 pointerup 的 target 会变成 SVG，
     导致“点轴加齿轮/拖放建啮合”全部命中失败。 */
  if (e.button !== 0) return;
  const w = eventWorld(e);
  const hit = hitKind(e.target);
  const m = mode();
  svg.classList.toggle('mode-mesh', m === 'mesh');
  svg.classList.toggle('mode-coax', m === 'coax');
  svg.classList.toggle('mode-planet', m === 'planet');
  const hint = $('#hint');
  if (hint) {
    hint.textContent = m === 'planet'
      ? '行星级模式：点击空白处放置新行星级；拖动可移动已有行星级。配置在右侧「行星轮系」页。'
      : '提示：「放轴/齿轮」模式下点击空白处放轴，点击已有轴为其添加齿轮；Delete 键删除所选对象';
  }

  if (m === 'planet') {
    if (!hit) addPlanet(w.x, w.y);
    else if (hit.type === 'planet') {
      const p = byId(state.planets, hit.id);
      selection = hit;
      pointer = { action: 'move-planet', pid: hit.id,
        dx: w.x - (p.x || 0), dy: w.y - (p.y || 0), startX: w.sx, startY: w.sy, moved: false };
    }
    return;
  }
  if (m === 'mesh') {
    if (hit && hit.type === 'gear') {
      pointer = { action: 'mesh', from: hit.id, x0: w.sx, y0: w.sy, wx: w.x, wy: w.y };
    }
    return;
  }
  if (m === 'coax') {
    if (hit && hit.type === 'shaft') {
      pointer = { action: 'coax', from: hit.id, wx: w.x, wy: w.y };
    }
    return;
  }
  /* place 模式 */
  if (hit && hit.type === 'planet') {
    const p = byId(state.planets, hit.id);
    selection = hit;
    pointer = { action: 'move-planet', pid: hit.id,
      dx: w.x - (p.x || 0), dy: w.y - (p.y || 0), startX: w.sx, startY: w.sy, moved: false };
    render();
  } else if (hit && (hit.type === 'gear' || hit.type === 'shaft')) {
    const sid = hit.type === 'gear' ? byId(state.gears, hit.id).shaftId : hit.id;
    const s = byId(state.shafts, sid);
    selection = hit;
    pointer = { action: s.locked ? null : 'move-shaft', sid,
      dx: w.x - s.x, dy: w.y - s.y, startX: w.sx, startY: w.sy, moved: false };
    /* 注意：按下时不 render，避免事件目标节点被移除导致 pointerup 命中失败 */
  } else if (hit && (hit.type === 'mesh' || hit.type === 'coax')) {
    selection = hit; render();
  } else {
    pointer = { action: 'pan-or-place', pan0x: view.panX, pan0y: view.panY,
      sx: w.sx, sy: w.sy, wx: w.x, wy: w.y, moved: false, down: true };
  }
});

svg.addEventListener('pointermove', e => {
  if (!pointer) return;
  const w = eventWorld(e);
  if (pointer.action === 'move-shaft') {
    const s = byId(state.shafts, pointer.sid);
    if (Math.abs(w.sx - pointer.startX) + Math.abs(w.sy - pointer.startY) > 3) pointer.moved = true;
    s.x = +(w.x - pointer.dx).toFixed(3);
    s.y = +(w.y - pointer.dy).toFixed(3);
    render();
  } else if (pointer.action === 'move-planet') {
    const p = byId(state.planets, pointer.pid);
    if (Math.abs(w.sx - pointer.startX) + Math.abs(w.sy - pointer.startY) > 3) pointer.moved = true;
    p.x = +(w.x - pointer.dx).toFixed(3);
    p.y = +(w.y - pointer.dy).toFixed(3);
    render();
  } else if (pointer.action === 'pan-or-place') {
    if (Math.abs(w.sx - pointer.sx) + Math.abs(w.sy - pointer.sy) > 6) pointer.moved = true;
    if (pointer.moved) {
      view.panX = pointer.pan0x + (w.sx - pointer.sx);
      view.panY = pointer.pan0y + (w.sy - pointer.sy);
      applyWorldTransform();
    }
  } else if (pointer.action === 'mesh' || pointer.action === 'coax') {
    pointer.wx = w.x; pointer.wy = w.y;
    clearLayer(layers.drag);
    const fromGear = pointer.action === 'mesh' ? byId(state.gears, pointer.from) : null;
    const fromShaft = pointer.action === 'mesh'
      ? byId(state.shafts, fromGear.shaftId) : byId(state.shafts, pointer.from);
    svgEl('line', { x1: fromShaft.x, y1: fromShaft.y, x2: w.x, y2: w.y, class: 'drag-line' }, layers.drag);
  }
});

svg.addEventListener('pointerup', e => {
  if (!pointer) return;
  const w = eventWorld(e);
  const want = pointer.action === 'mesh' ? 'gear'
    : pointer.action === 'coax' ? 'shaft' : null;
  const hit = hitAtPoint(e.clientX, e.clientY, want) || hitKind(e.target);
  const p = pointer;
  pointer = null;
  clearLayer(layers.drag);

  if (p.action === 'pan-or-place' && !p.moved) {
    if (!hit) addShaft(w.x, w.y);
    else { selection = hit; render(); }
  } else if (p.action === 'move-shaft' && !p.moved) {
    /* 单击已有轴：点中轴（或该轴上齿轮）且该轴尚无齿轮时直接装一个齿轮 */
    let sid = null;
    if (hit && hit.type === 'shaft') sid = hit.id;
    else if (hit && hit.type === 'gear') sid = byId(state.gears, hit.id).shaftId;
    const s = sid && byId(state.shafts, sid);
    if (s && !state.gears.some(g => g.shaftId === s.id)) addGearOn(s.id);
    else if (hit) { selection = hit; render(); }
  } else if (p.action === 'move-shaft' && p.moved) {
    markDirty();
  } else if (p.action === 'move-planet' && !p.moved) {
    if (hit && hit.type === 'planet') { selection = { type: 'planet', id: hit.id }; selectedPlanetId = hit.id; render(); renderPlanetPanel(); }
  } else if (p.action === 'move-planet' && p.moved) {
    markDirty();
  } else if (p.action === 'mesh' && hit && hit.type === 'gear') {
    if (hit.id === p.from) return;
    if (state.meshes.some(x =>
      (x.gearA === p.from && x.gearB === hit.id) || (x.gearB === p.from && x.gearA === hit.id))) {
      flashHint('这对齿轮已经存在啮合（重复约束会在诊断中报错）');
      return;
    }
    state.meshes.push({ id: uid('m'), gearA: p.from, gearB: hit.id });
    resetMotion();
    markDirty(); render();
  } else if (p.action === 'coax' && hit && hit.type === 'shaft') {
    if (hit.id === p.from) return;
    if (state.coaxRelations.some(x =>
      (x.shaftA === p.from && x.shaftB === hit.id) || (x.shaftB === p.from && x.shaftA === hit.id))) {
      flashHint('这两根轴已是同轴关系'); return;
    }
    state.coaxRelations.push({ id: uid('c'), shaftA: p.from, shaftB: hit.id });
    /* 同轴自动吸附到同一位置（取中点） */
    const A = byId(state.shafts, p.from), B = byId(state.shafts, hit.id);
    const mx = (A.x + B.x) / 2, my = (A.y + B.y) / 2;
    if (!A.locked) { A.x = mx; A.y = my; }
    if (!B.locked) { B.x = mx; B.y = my; }
    markDirty(); render();
  }
});

svg.addEventListener('wheel', e => {
  e.preventDefault();
  const rect = svg.getBoundingClientRect();
  const sx = e.clientX - rect.left, sy = e.clientY - rect.top;
  const u0 = SCALE * view.zoom;
  const wx = (sx - view.panX) / u0, wy = (sy - view.panY) / u0;
  view.zoom = Math.max(0.2, Math.min(8, view.zoom * (e.deltaY < 0 ? 1.12 : 0.89)));
  const u1 = SCALE * view.zoom;
  view.panX = sx - wx * u1;
  view.panY = sy - wy * u1;
  applyWorldTransform();
}, { passive: false });

/* 指针在 SVG 外松开也要结束手势（避免拖轴/拖连线卡住） */
document.addEventListener('pointerup', e => {
  if (!pointer) return;
  if (svg.contains(e.target)) return;  // SVG 自身的 pointerup 已处理
  const p = pointer;
  pointer = null;
  clearLayer(layers.drag);
  if (p.action === 'move-shaft' && p.moved) markDirty();
  if (p.action === 'move-planet' && p.moved) markDirty();
});

/* 单击已有轴的“加齿轮”快捷：双击轴 */
svg.addEventListener('dblclick', e => {
  if (mode() !== 'place') return;
  const hit = hitKind(e.target);
  if (hit && hit.type === 'shaft') addGearOn(hit.id);
});

function flashHint(msg) {
  const hint = $('#hint');
  const old = hint.textContent;
  hint.textContent = msg;
  setTimeout(() => { hint.textContent = old; }, 2200);
}

document.addEventListener('keydown', e => {
  if ((e.key === 'Delete' || e.key === 'Backspace') && selection &&
      !['INPUT', 'TEXTAREA', 'SELECT'].includes(document.activeElement.tagName)) {
    if (selection.type === 'shaft') deleteShaft(selection.id);
    else if (selection.type === 'gear') deleteGear(selection.id);
    else if (selection.type === 'planet') deletePlanet(selection.id);
    else if (selection.type === 'mesh') {
      state.meshes = state.meshes.filter(x => x.id !== selection.id);
      selection = null; markDirty(); render();
    } else if (selection.type === 'coax') {
      state.coaxRelations = state.coaxRelations.filter(x => x.id !== selection.id);
      selection = null; markDirty(); render();
    }
  }
});

/* ---------------- 播放 ---------------- */
let playMode = 'none';   // none | step | spin
let playT0 = 0;
let lastTs = 0;

/* ---------------- 运动：统一由输入转角 motionTheta（度）驱动 ---------------- */
function resetMotion() {
  motionTheta = 0;
  const sl = $('#angle-slider');
  if (sl) sl.value = 0;
  const ro = $('#angle-readout');
  if (ro) ro.textContent = '0°';
}

function applyMotionTheta(theta) {
  motionTheta = theta;
  const ro = $('#angle-readout');
  if (ro) {
    const turns = theta / 360;
    ro.textContent = `${theta.toFixed(0)}°（输入轴 ${turns.toFixed(3)} 转）`;
  }
  updateRotorTransforms();
  updatePlanetTransforms();
}

function activePlayState() {
  const pp = planetPreviewState();
  if (pp) return { st: pp, an: analyzeLocal(pp), preview: true };
  const prev = candidatePreviewState();
  return prev ? { st: prev, an: analyzeLocal(prev), preview: true }
              : { st: state, an: analysis, preview: false };
}

function frame(ts) {
  if (playMode === 'none') return;
  const dt = lastTs ? (ts - lastTs) / 1000 : 0;
  lastTs = ts;
  const { st, an } = activePlayState();

  if (playMode === 'spin') {
    // 输入轴按其 rpm 推进；其余构件（含行星轮自转/公转）由转速比随动
    const rpm = st.inputRpm || 60;
    applyMotionTheta(motionTheta + rpm * 6 * dt);
    if ($('#angle-loop').checked && an.cycle && motionTheta >= 360 * an.cycle.inputTurnsV)
      applyMotionTheta(motionTheta - 360 * an.cycle.inputTurnsV);
  } else if (playMode === 'step') {
    const lv = Math.max(1, an.levels.length);
    const dur = 0.85 * (lv + 0.5);
    let p = ((performance.now() - playT0) / 1000) % (dur + 0.5);
    if (p > dur) p = dur;
    const phase = p / dur;
    applyMotionTheta(720 * phase);   // 输入轴两转逐级传播
    const reached = Math.floor(phase * (lv + 0.001));
    document.querySelectorAll('.mesh-line').forEach(line => {
      const mid = line.dataset.mesh;
      let lvl = -1;
      an.levels.forEach((arr, i) => { if (arr.includes(mid)) lvl = i; });
      line.classList.toggle('edge-active', lvl === reached - 1);
      line.classList.toggle('edge-pending', lvl >= reached);
    });
  }
  const sl = $('#angle-slider');
  if (sl && Math.abs(parseFloat(sl.value) - motionTheta) > 1) {
    const v = Math.max(-720, Math.min(720, motionTheta));
    if (motionTheta >= -720 && motionTheta <= 720) sl.value = v;
  }
  requestAnimationFrame(frame);
}

function setPlay(m) {
  playMode = playMode === m ? 'none' : m;
  $('#btn-play').classList.toggle('active', playMode === 'step');
  $('#btn-spin').classList.toggle('active', playMode === 'spin');
  if (playMode === 'none') {
    document.querySelectorAll('.mesh-line').forEach(l => l.classList.remove('edge-active', 'edge-pending'));
    resetMotion();
    render();
    return;
  }
  if (playMode === 'step') playT0 = performance.now();
  lastTs = 0;
  requestAnimationFrame(frame);
}
$('#btn-play').addEventListener('click', () => setPlay('step'));
$('#btn-spin').addEventListener('click', () => setPlay('spin'));
$('#angle-slider').addEventListener('input', e => {
  if (playMode !== 'none') setPlay('none');
  applyMotionTheta(parseFloat(e.target.value));
});

/* ---------------- 搜索面板 ---------------- */
$('#btn-search').addEventListener('click', async () => {
  const btn = $('#btn-search');
  btn.disabled = true; btn.textContent = '搜索中…';
  $('#search-meta').textContent = '正在枚举齿数组合';
  try {
    const resp = await fetch('/api/search', {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        state,
        target: $('#s-target').value,
        tolerancePct: parseFloat($('#s-tol').value) || 0,
        zMin: parseInt($('#s-zmin').value, 10),
        zMax: parseInt($('#s-zmax').value, 10),
        modules: $('#s-modules').value,
        centerTol: parseFloat($('#s-centertol').value) || CENTER_TOL,
      }),
    });
    const data = await resp.json();
    candidates = data.results || [];
    selectedCand = candidates.length ? 0 : -1;
    $('#search-meta').textContent =
      (data.note ? data.note + ' ' : '') +
      (data.totalMatched != null ? `命中 ${data.totalMatched} 组，显示前 ${candidates.length}；枚举节点 ${data.nodes}${data.truncated ? '（达到搜索上限已截断）' : ''}` : '');
    render();
  } finally {
    btn.disabled = false; btn.textContent = '搜索替换组合';
  }
});

function renderSearchPanel() {
  renderPlanetCandidates();
  const ol = $('#candidate-list');
  ol.innerHTML = '';
  candidates.forEach((c, i) => {
    const li = h('li', { class: 'cand' + (i === selectedCand ? ' selected' : '') }, ol);
    li.addEventListener('click', () => {
      selectedCand = i === selectedCand ? -1 : i;
      resetMotion(); render();
    });
    const head = h('div', { class: 'head' }, li);
    h('span', {}, head, `#${i + 1}  i = ${c.ratio.s} ≈ ${c.ratio.v.toFixed(5)}`);
    h('span', { class: 'err' }, head, `误差 ${c.errorPct}%`);
    h('div', { class: 'meta' }, li,
      `最大齿数 z=${c.maxZ}；非锁轴最大位移 ${c.maxMove} mm；中心距残差 ${c.maxResidual} mm；复位循环 ${c.cycle} 转（×${c.cycleRatio}）`);
    const rep = h('div', { class: 'repl' }, li);
    rep.innerHTML = '替换：' + c.replaced.map(r =>
      `${r.name} ${r.z0}<small>齿/m${r.m0}</small>→<b>${r.z1}</b><small>齿/m${r.m1}</small>`).join('；');
    const acts = h('div', { class: 'actions' }, li);
    h('button', { onclick: e => { e.stopPropagation(); selectedCand = i; resetMotion(); render(); } },
      acts, i === selectedCand ? '✓ 预览中' : '叠加预览');
    h('button', { class: 'primary', onclick: e => { e.stopPropagation(); applyCandidate(i); } },
      acts, '采用此方案');
  });
}

function applyCandidate(i) {
  const c = candidates[i];
  if (!c) return;
  for (const g of state.gears) if (c.gears[g.id]) Object.assign(g, c.gears[g.id]);
  for (const s of state.shafts) if (c.shafts[s.id]) { s.x = c.shafts[s.id].x; s.y = c.shafts[s.id].y; }
  selectedCand = -1;
  candidates = [];
  resetMotion();
  setPlay('none');
  markDirty(); render();
  flashHint('已采用候选方案，可在「方案库」另存并与基线对照');
}

/* ---------------- 行星级配齿搜索 ---------------- */
$('#btn-ps-search')?.addEventListener('click', async () => {
  const btn = $('#btn-ps-search');
  btn.disabled = true; btn.textContent = '搜索中…';
  $('#ps-meta').textContent = '正在枚举可装配行星级';
  try {
    const resp = await fetch('/api/search-planets', {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        target: $('#ps-target').value,
        tolerancePct: parseFloat($('#ps-tol').value) || 0,
        zMin: parseInt($('#ps-zmin').value, 10),
        zMax: parseInt($('#ps-zmax').value, 10),
        modules: $('#ps-modules').value,
        counts: $('#ps-counts').value,
        maxOuter: parseFloat($('#ps-outer').value) || 1e9,
      }),
    });
    const data = await resp.json();
    planetCandidates = data.results || [];
    $('#ps-meta').textContent =
      (data.note ? data.note + ' ' : '') +
      (data.totalMatched != null
        ? `命中 ${data.totalMatched} 组，显示前 ${planetCandidates.length}；枚举 ${data.nodes}${data.truncated ? '（超时已截断）' : ''}`
        : '');
    render();
  } finally {
    btn.disabled = false; btn.textContent = '搜索可装配行星级';
  }
});

function renderPlanetCandidates() {
  const ol = $('#ps-list');
  if (!ol) return;
  ol.innerHTML = '';
  const roleTxt = c =>
    `${c.fixed === 'r' ? '齿圈固定' : c.fixed === 's' ? '太阳固定' : '架固定'}，` +
    `${P_LABEL[c.input]}输入→${P_LABEL[c.output]}输出`;
  planetCandidates.forEach((c, i) => {
    if (!c || !c.ratio) return;
    const li = h('li', { class: 'cand' }, ol);
    const head = h('div', { class: 'head' }, li);
    h('span', {}, head, `#${i + 1}  i = ${c.ratio.s} ≈ ${Number(c.ratio.v).toFixed(5)}`);
    h('span', { class: 'err' }, head, `误差 ${c.errorPct}%`);
    h('div', { class: 'meta' }, li,
      `m=${c.module}：zS ${c.zS} / zP ${c.zP} ×${c.count} / zR ${c.zR}；${roleTxt(c)}`);
    h('div', { class: 'meta' }, li,
      `外径 ⌀${c.outerD} mm；行星净距 ${c.netGap} mm；轨迹 r=${c.orbitR}；重复啮合周期 ${c.cycle} 转；最大齿 z=${c.maxZ}`);
    const acts = h('div', { class: 'actions' }, li);
    h('button', { onclick: () => previewPlanetCandidate(i) }, acts, '叠加预览');
    h('button', { onclick: e => { e.stopPropagation(); selectedPlanetPreview = -1; planetPreviewCache = null; resetMotion(); render(); } },
      acts, '清除预览');
    h('button', { class: 'primary', onclick: () => applyPlanetCandidate(i) }, acts, '采用到行星级');
    h('button', { onclick: () => { addPlanetFromCandidate(c); } }, acts, '建成新行星级');
  });
}

function planetCandidateState(c) {
  // 在第一个（或选中的）行星级上叠加候选，返回临时 state 用于预览
  const target0 = byId(state.planets || [], selectedPlanetId) || state.planets[0];
  if (!target0) return null;
  const st = JSON.parse(JSON.stringify(state));
  const p = byId(st.planets, target0.id);
  Object.assign(p, { module: c.module, zS: c.zS, zP: c.zP, zR: c.zR, count: c.count,
    fixed: c.fixed, input: c.input, output: c.output });
  st.inputId = pNode(p.id, c.input);
  st.outputId = pNode(p.id, c.output);
  return st;
}

function previewPlanetCandidate(i) {
  const c = planetCandidates[i];
  const st = planetCandidateState(c);
  if (!st) { flashHint('请先放置一个行星级'); return; }
  selectedPlanetPreview = i;
  planetPreviewCache = { st, ver: stateVersion };
  resetMotion();
  render();
  const an = analyzeLocal(st);
  const errs = an.issues.filter(x => x.severity === 'error').length;
  flashHint(`叠加预览：i=${c.ratio.s}，可装配，当前诊断 ${errs} 个错误（蓝色虚线为候选）`);
}

let selectedPlanetPreview = -1;
let planetPreviewCache = null;
function planetPreviewState() {
  if (selectedPlanetPreview < 0 || !planetCandidates[selectedPlanetPreview]) return null;
  if (planetPreviewCache && planetPreviewCache.ver === stateVersion)
    return planetPreviewCache.st;
  const st = planetCandidateState(planetCandidates[selectedPlanetPreview]);
  planetPreviewCache = { st, ver: stateVersion };
  return st;
}

function applyPlanetCandidate(i) {
  const c = planetCandidates[i];
  let p = byId(state.planets || [], selectedPlanetId) || state.planets[0];
  if (!p) { addPlanetFromCandidate(c); return; }
  Object.assign(p, { module: c.module, zS: c.zS, zP: c.zP, zR: c.zR, count: c.count,
    fixed: c.fixed, input: c.input, output: c.output });
  const attKey = { s: 'sunShaftId', r: 'ringShaftId', c: 'carrierShaftId' };
  state.inputId = p[attKey[c.input]] || pNode(p.id, c.input);
  state.outputId = p[attKey[c.output]] || pNode(p.id, c.output);
  selectedPlanetId = p.id;
  selectedPlanetPreview = -1;
  resetMotion();
  markDirty(); render();
  flashHint(`已采用：zS=${c.zS}, zP=${c.zP}, zR=${c.zR}，i=${c.ratio.s}`);
}

function addPlanetFromCandidate(c) {
  // 找一个不与现有行星级重叠的位置
  const k = state.planets.length;
  const p = defaultPlanet(0, 0);
  Object.assign(p, { module: c.module, zS: c.zS, zP: c.zP, zR: c.zR, count: c.count,
    fixed: c.fixed, input: c.input, output: c.output,
    x: (k % 3) * 120, y: Math.floor(k / 3) * 120 });
  state.planets.push(p);
  selectedPlanetId = p.id;
  state.inputId = pNode(p.id, c.input);
  state.outputId = pNode(p.id, c.output);
  selection = { type: 'planet', id: p.id };
  resetMotion();
  markDirty(); render();
  switchTab('planet');
}

function normalizeState(st) {
  st.shafts = st.shafts || [];
  st.gears = st.gears || [];
  st.meshes = st.meshes || [];
  st.coaxRelations = st.coaxRelations || [];
  st.planets = st.planets || [];
  if (st.inputRpm == null) st.inputRpm = 60;
  for (const g of st.gears) {
    /* 旧项目没有齿形参数时按标准齿形打开 */
    if (g.ha == null) g.ha = 1.0;
    if (g.c == null) g.c = 0.25;
    if (g.x == null) g.x = 0.0;
  }
  for (const p of st.planets) {
    if (p.phase == null) p.phase = 0;
    for (const k of ['sunShaftId', 'ringShaftId', 'carrierShaftId'])
      if (p[k] === undefined) p[k] = null;
  }
  return st;
}

/* ---------------- 方案库 ---------------- */
async function refreshLibrary() {
  const [alts] = await Promise.all([(await fetch('/api/alternatives')).json()]);
  const ul = $('#alt-list');
  ul.innerHTML = '';
  const line = $('#baseline-line');
  line.textContent = baseline
    ? `当前基线：#${baseline.id} 「${baseline.name}」（画布上绿色虚线节圆为基线）。`
    : '尚未设置基线：另存方案后可将其设为基线，之后任何修改都能与其对照。';
  for (const a of alts) {
    const li = h('li', { class: baseline && baseline.id === a.id ? 'is-baseline' : '' }, ul);
    h('span', { class: 'name' }, li, `${a.id}. ${a.name}${baseline && baseline.id === a.id ? ' ★基线' : ''}`);
    h('span', { class: 'ratio' }, li, a.ratio ? `i=${a.ratio}` : '');
    h('button', { title: '载入到画布', onclick: () => loadAlternative(a.id) }, li, '载入');
    h('button', { onclick: () => setBaseline(a.id) }, li, '设为基线');
    h('button', { class: 'danger', onclick: () => deleteAlternative(a.id) }, li, '删');
  }
}

async function loadAlternative(id) {
  const a = await (await fetch(`/api/alternatives/${id}`)).json();
  state = normalizeState(a.state);
  bumpIds(state);
  selection = null; candidates = []; selectedCand = -1; resetMotion();
  markDirty(); render();
  switchTab('analysis');
  flashHint(`已载入方案「${a.name}」，绿色虚线为基线节圆`);
}
async function setBaseline(id) {
  await fetch(`/api/baseline/${id}`, { method: 'POST' });
  await reloadProjectMeta();
  refreshLibrary();
  render();
}
async function deleteAlternative(id) {
  await fetch(`/api/alternatives/${id}`, { method: 'DELETE' });
  await reloadProjectMeta();
  refreshLibrary();
}

$('#btn-save-alt').addEventListener('click', async () => {
  const name = $('#alt-name').value.trim();
  await fetch('/api/alternatives', {
    method: 'POST', headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({
      name, state, baselineId: baseline ? baseline.id : null,
      target: $('#s-target').value,
      ratio: analysis.ratio ? analysis.ratio.s : null,
      errorPct: null,
    }),
  });
  $('#alt-name').value = '';
  refreshLibrary();
  flashHint('方案已另存到 SQLite');
});

$('#btn-clear-baseline').addEventListener('click', async () => {
  await fetch('/api/baseline/clear', { method: 'POST' });
  await reloadProjectMeta();
  refreshLibrary();
  render();
});

/* ---------------- 持久化 ---------------- */
function markDirty() {
  dirty = true;
  stateVersion++;
  const tag = $('#save-state');
  tag.textContent = '未保存…';
  tag.className = 'savestate dirty';
  clearTimeout(saveTimer);
  saveTimer = setTimeout(saveProject, 700);
}
async function saveProject() {
  await fetch('/api/project', {
    method: 'POST', headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ state }),
  });
  dirty = false;
  const tag = $('#save-state');
  tag.textContent = '✓ 已保存';
  tag.className = 'savestate';
}
async function reloadProjectMeta() {
  const data = await (await fetch('/api/project')).json();
  baseline = data.baseline || null;
  return data;
}

/* ---------------- Tab / 顶部按钮 ---------------- */
function switchTab(name) {
  document.querySelectorAll('.tab').forEach(t => t.classList.toggle('active', t.dataset.tab === name));
  document.querySelectorAll('.panel').forEach(p => p.classList.toggle('active', p.id === `panel-${name}`));
  if (name === 'library') refreshLibrary();
}
document.querySelectorAll('.tab').forEach(t =>
  t.addEventListener('click', () => switchTab(t.dataset.tab)));

$('#btn-sample').addEventListener('click', () => {
  state = sampleState();
  bumpIds(state);
  selection = null; candidates = []; selectedCand = -1; resetMotion();
  view.zoom = 1; view.panX = 80; view.panY = svg.clientHeight / 2;
  markDirty(); render();
});
$('#btn-clear').addEventListener('click', () => {
  if (!state.shafts.length || confirm('确定清空画布上的所有轴、齿轮与约束？')) {
    state = emptyState();
    selection = null; candidates = []; selectedCand = -1; resetMotion();
    markDirty(); render();
  }
});

/* ---------------- 启动 ---------------- */
(async function init() {
  view.panY = svg.clientHeight / 2;
  const data = await reloadProjectMeta();
  if (data.state && ((data.state.shafts || []).length || (data.state.planets || []).length)) {
    state = normalizeState(data.state);
    bumpIds(state);
  } else {
    state = sampleState();
    bumpIds(state);
    markDirty();
  }
  render();
})();
