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
const counters = { s: 0, g: 0, m: 0, c: 0 };
function bumpIds(state) {
  for (const list of [['shafts', 's'], ['gears', 'g'], ['meshes', 'm'], ['coaxRelations', 'c']]) {
    const p = list[1];
    counters[p] = Math.max(counters[p], 0);
    state[list[0]].forEach(o => {
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
let angles = new Map();         // shaftId -> 度
let dirty = false;
let saveTimer = null;
let stateVersion = 0;
let previewCache = null;

const view = { zoom: 1, panX: 60, panY: 200 };

function emptyState() {
  return { shafts: [], gears: [], meshes: [], coaxRelations: [],
           inputId: null, outputId: null, inputRpm: 60 };
}
const byId = (list, id) => list.find(o => o.id === id);
const shaftMap = () => Object.fromEntries(state.shafts.map(s => [s.id, s]));
const gearMap = () => Object.fromEntries(state.gears.map(g => [g.id, g]));

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

  const speeds = new Map();
  const depth = new Map();
  const edgeLevel = new Map();
  const conflicts = new Set();
  if (st.inputId && shafts[st.inputId]) {
    speeds.set(st.inputId, fr(1));
    depth.set(st.inputId, 0);
    const q = [st.inputId];
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
            add('error', sameSign ? 'RATIO_CONFLICT' : 'DIRECTION_CONFLICT',
              sameSign
                ? `传动比矛盾：轴「${sname(shafts[nb.to])}」经两条路径推得 ${fStr(old)} 与 ${fStr(v)}`
                : `转向矛盾：轴「${sname(shafts[nb.to])}」经两条路径转向相反（${fStr(old)} 与 ${fStr(v)}），闭合轮系齿数不满足约束`,
              { mesh: nb.mesh, shaft: nb.to });
          }
        } else {
          speeds.set(nb.to, v);
          depth.set(nb.to, depth.get(cur) + 1);
          edgeLevel.set(nb.mesh, depth.get(nb.to));
          q.push(nb.to);
        }
      }
    }
  } else add('info', 'NO_INPUT', '尚未指定输入轴（在轴属性中设置）');

  for (const s of st.shafts) {
    if (!speeds.has(s.id) && st.gears.some(g => g.shaftId === s.id))
      add('warning', 'IDLE_SHAFT', `轴「${sname(s)}」未连入输入轴的动力链`, { shaft: s.id });
  }
  if (st.outputId && !speeds.has(st.outputId)) add('warning', 'NO_OUTPUT_PATH', '输出轴无法从输入轴到达');
  if (!st.outputId || !shafts[st.outputId]) add('info', 'NO_OUTPUT', '尚未指定输出轴（在轴属性中设置）');

  const ratio = st.outputId ? speeds.get(st.outputId) || null : null;
  const rpm = st.inputRpm || 1;
  const rpms = {};
  for (const [sid, v] of speeds) rpms[sid] = fNum(v) * rpm;

  let L = 1n;
  for (const g of st.gears) {
    const v = speeds.get(g.shaftId);
    if (v && Number.isInteger(g.z)) L = bLcm(L, fMul(fr(g.z), v).d);
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
    cycle, edges, levels,
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
  st.inputId = 's1';
  st.outputId = 's3';
  st.inputRpm = 60;
  return st;
}

/* ---------------- 渲染 ---------------- */
const layers = {
  rel: $('#layer-relations'), prevBase: $('#layer-preview-baseline'),
  prev: $('#layer-preview'), drag: $('#layer-drag'),
  gears: $('#layer-gears'), shafts: $('#layer-shafts'),
};

function gearRadii(g) {
  const rp = g.module * g.z / 2;
  if (g.internal) return { rp, outer: rp + 1.25 * g.module, tip: rp - g.module, internal: true };
  return { rp, outer: rp + g.module, root: Math.max(0.5, rp - 1.25 * g.module), internal: false };
}

function clearLayer(l) { while (l.firstChild) l.removeChild(l.firstChild); }

function selectedMeshBad() {
  return new Set(analysis.issues.filter(i => i.severity === 'error' && i.refs.mesh).map(i => i.refs.mesh));
}

function render() {
  analysis = analyzeLocal(state);
  angles = new Map([...angles].filter(([sid]) => byId(state.shafts, sid)));
  Object.values(layers).forEach(clearLayer);

  const S = shaftMap(), G = gearMap();
  const badMeshes = selectedMeshBad();

  /* 同轴关系 */
  for (const r of state.coaxRelations) {
    const A = S[r.shaftA], B = S[r.shaftB];
    if (!A || !B) continue;
    const g0 = svgEl('g', { class: 'coax', 'data-coax': r.id }, layers.rel);
    svgEl('line', { x1: A.x, y1: A.y, x2: B.x, y2: B.y,
      stroke: 'transparent', 'stroke-width': 3, 'pointer-events': 'stroke',
      'data-coax': r.id, style: 'cursor:pointer' }, g0);
    svgEl('line', { x1: A.x, y1: A.y, x2: B.x, y2: B.y,
      class: 'coax-ring' + (selection && selection.type === 'coax' && selection.id === r.id ? ' relation-selected' : ''),
      'data-coax': r.id }, g0);
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

  renderBaselineOverlay();
  renderCandidatePreview();
  applyWorldTransform();
  updateRotorTransforms();
  renderPanels();
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
  document.querySelectorAll(sel).forEach(node => {
    const sid = node.getAttribute('data-shaft');
    const s = byId(st.shafts, sid);
    if (!s) return;
    const a = angles.get(sid) || 0;
    node.setAttribute('transform', `translate(${s.x} ${s.y}) rotate(${a})`);
  });
}

/* ---------------- 面板：分析 ---------------- */
function renderPanels() {
  renderAnalysisPanel();
  renderProps();
  renderSearchPanel();
  renderLibrary();
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
    head.textContent = `整列恢复初始啮合：输入轴转 ${L} 转（各轴转过的齿数同时为整数）`;
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
}

function focusIssue(it) {
  const refs = it.refs || {};
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
      state.inputId = s.id; angles.clear(); commit();
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

/* ---------------- 删除 / 放置 ---------------- */
function deleteShaft(sid) {
  const gids = new Set(state.gears.filter(g => g.shaftId === sid).map(g => g.id));
  state.gears = state.gears.filter(g => g.shaftId !== sid);
  state.meshes = state.meshes.filter(e => !gids.has(e.gearA) && !gids.has(e.gearB));
  state.coaxRelations = state.coaxRelations.filter(r => r.shaftA !== sid && r.shaftB !== sid);
  state.shafts = state.shafts.filter(s => s.id !== sid);
  if (state.inputId === sid) state.inputId = null;
  if (state.outputId === sid) state.outputId = null;
  selection = null;
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
    pressureAngle: 20, internal: false, locked: false };
  state.gears.push(g);
  selection = { type: 'gear', id: g.id };
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
    node = node.parentNode;
  }
  return null;
}

svg.addEventListener('pointerdown', e => {
  svg.setPointerCapture(e.pointerId);
  const w = eventWorld(e);
  const hit = hitKind(e.target);
  const m = mode();
  svg.classList.toggle('mode-mesh', m === 'mesh');
  svg.classList.toggle('mode-coax', m === 'coax');

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
  if (hit && (hit.type === 'gear' || hit.type === 'shaft')) {
    const sid = hit.type === 'gear' ? byId(state.gears, hit.id).shaftId : hit.id;
    const s = byId(state.shafts, sid);
    selection = hit;
    render();
    pointer = { action: s.locked ? null : 'move-shaft', sid,
      dx: w.x - s.x, dy: w.y - s.y, startX: w.sx, startY: w.sy, moved: false };
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
  const hit = hitKind(e.target);
  const p = pointer;
  pointer = null;
  clearLayer(layers.drag);

  if (p.action === 'pan-or-place' && !p.moved) {
    if (!hit) addShaft(w.x, w.y);
    else { selection = hit; render(); }
  } else if (p.action === 'move-shaft' && !p.moved) {
    /* 单击已有轴：若点中的是轴且该轴上无齿轮则直接加齿轮；点中齿轮只选中 */
    if (hit && hit.type === 'shaft') {
      const s = byId(state.shafts, hit.id);
      if (s && !state.gears.some(g => g.shaftId === s.id)) addGearOn(s.id);
    }
  } else if (p.action === 'move-shaft' && p.moved) {
    markDirty();
  } else if (p.action === 'mesh' && hit && hit.type === 'gear') {
    if (hit.id === p.from) return;
    if (state.meshes.some(x =>
      (x.gearA === p.from && x.gearB === hit.id) || (x.gearB === p.from && x.gearA === hit.id))) {
      flashHint('这对齿轮已经存在啮合（重复约束会在诊断中报错）');
      return;
    }
    state.meshes.push({ id: uid('m'), gearA: p.from, gearB: hit.id });
    angles.clear();
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

function activePlayState() {
  const prev = candidatePreviewState();
  return prev ? { st: prev, an: analyzeLocal(prev), preview: true }
              : { st: state, an: analysis, preview: false };
}

function frame(ts) {
  if (playMode === 'none') return;
  const dt = lastTs ? (ts - lastTs) / 1000 : 0;
  lastTs = ts;
  const { st, an, preview } = activePlayState();

  if (playMode === 'spin') {
    for (const s of st.shafts) {
      const rpm = an.rpms[s.id] || 0;
      angles.set(s.id, (angles.get(s.id) || 0) + rpm * 6 * dt);
    }
  } else if (playMode === 'step') {
    const lv = Math.max(1, an.levels.length);
    const dur = 0.85 * (lv + 0.5);
    let p = ((performance.now() - playT0) / 1000) % (dur + 0.5);
    if (p > dur) p = dur;
    const phase = p / dur;
    for (const s of st.shafts) {
      const sp = an.speeds[s.id];
      if (sp) angles.set(s.id, sp.v * 720 * phase);
    }
    const reached = Math.floor(phase * (lv + 0.001));
    document.querySelectorAll('.mesh-line').forEach(line => {
      const mid = line.dataset.mesh;
      let lvl = -1;
      an.levels.forEach((arr, i) => { if (arr.includes(mid)) lvl = i; });
      line.classList.toggle('edge-active', lvl === reached - 1);
      line.classList.toggle('edge-pending', lvl >= reached);
    });
  }
  updateRotorTransforms();
  requestAnimationFrame(frame);
}

function setPlay(m) {
  playMode = playMode === m ? 'none' : m;
  $('#btn-play').classList.toggle('active', playMode === 'step');
  $('#btn-spin').classList.toggle('active', playMode === 'spin');
  if (playMode === 'none') {
    document.querySelectorAll('.mesh-line').forEach(l => l.classList.remove('edge-active', 'edge-pending'));
    angles = new Map();
    render();
    return;
  }
  if (playMode === 'step') playT0 = performance.now();
  lastTs = 0;
  requestAnimationFrame(frame);
}
$('#btn-play').addEventListener('click', () => setPlay('step'));
$('#btn-spin').addEventListener('click', () => setPlay('spin'));

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
  const ol = $('#candidate-list');
  ol.innerHTML = '';
  candidates.forEach((c, i) => {
    const li = h('li', { class: 'cand' + (i === selectedCand ? ' selected' : '') }, ol);
    li.addEventListener('click', () => {
      selectedCand = i === selectedCand ? -1 : i;
      angles.clear(); render();
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
    h('button', { onclick: e => { e.stopPropagation(); selectedCand = i; angles.clear(); render(); } },
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
  angles.clear();
  setPlay('none');
  markDirty(); render();
  flashHint('已采用候选方案，可在「方案库」另存并与基线对照');
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
  state = a.state;
  bumpIds(state);
  selection = null; candidates = []; selectedCand = -1; angles.clear();
  markDirty(); render();
  switchTab('analysis');
  flashHint(`已载入方案「${a.name}」，绿色虚线为基线节圆`);
}
async function setBaseline(id) {
  await fetch(`/api/baseline/${id}`, { method: 'POST' });
  await reloadProjectMeta();
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
  selection = null; candidates = []; selectedCand = -1; angles.clear();
  view.zoom = 1; view.panX = 80; view.panY = svg.clientHeight / 2;
  markDirty(); render();
});
$('#btn-clear').addEventListener('click', () => {
  if (!state.shafts.length || confirm('确定清空画布上的所有轴、齿轮与约束？')) {
    state = emptyState();
    selection = null; candidates = []; selectedCand = -1; angles.clear();
    markDirty(); render();
  }
});

/* ---------------- 启动 ---------------- */
(async function init() {
  view.panY = svg.clientHeight / 2;
  const data = await reloadProjectMeta();
  if (data.state && (data.state.shafts || []).length) {
    state = data.state;
    bumpIds(state);
  } else {
    state = sampleState();
    bumpIds(state);
    markDirty();
  }
  render();
})();
