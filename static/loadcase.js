/* 载荷工况工作区 — 原生 JS/SVG；功率/转矩传播与轴承反力由后端 loadcase.py 完成。
   草稿 = {name, spec, snapshot}：spec 记录一次轮系受载的输入转矩/转速/持续时间、
   各啮合效率与载荷系数、分支比例与各轴轴向布置；snapshot 为建立时的项目快照，
   源轮系齿轮(z/m/x/压力角)或轴位变化时只标记过期。 */
'use strict';

const LC_KEY = 'loadcaseDraftV1';
const LC_DEFAULTS = { efficiency: 0.98, loadFactor: 1.0, torque: 0, rpm: 100,
  duration: 1000, minGap: 3, adj: 30, cr: 1000 };

let lcDraft = null;       // {name, spec, snapshot}
let lcResult = null;
let lcToken = 0;
let lcTimer = null;
let lcVersions = [];
let lcSearchRes = {};     // {shaftId: {results, nodes, truncated}}
let lcMeshKey = '';
let lcShaftKey = '';
const lcDrag = { id: null, shaftId: null, svg: null, x0: 0, locked: false };

/* ---------------- 快照与过期 ---------------- */
function lcFingerprint(st) {
  const S = Object.fromEntries((st.shafts || []).map(s => [s.id, s]));
  const G = Object.fromEntries((st.gears || []).map(g => [g.id, g]));
  const items = [];
  for (const e of (st.meshes || [])) {
    const ga = G[e.gearA], gb = G[e.gearB];
    if (!ga || !gb) { items.push([e.id, 'broken']); continue; }
    const sa = S[ga.shaftId] || {}, sb = S[gb.shaftId] || {};
    items.push([e.id, ga.z, ga.module, ga.x || 0, ga.pressureAngle || 20,
      gb.z, gb.module, gb.x || 0, gb.pressureAngle || 20,
      +sa.x || 0, +sa.y || 0, +sb.x || 0, +sb.y || 0]);
  }
  items.sort((a, b) => String(a[0]).localeCompare(String(b[0])));
  return JSON.stringify(items);
}

function lcStale() {
  return !!(lcDraft && lcFingerprint(lcDraft.snapshot) !== lcFingerprint(state));
}

function lcDefaultLayouts() {
  const byShaft = {};
  for (const s of state.shafts) byShaft[s.id] = [];
  for (const g of state.gears) if (byShaft[g.shaftId]) byShaft[g.shaftId].push(g);
  const shafts = {};
  for (const s of state.shafts) {
    const gs = byShaft[s.id].sort((a, b) => String(a.id).localeCompare(String(b.id)));
    const n = gs.length;
    const length = Math.max(50, Math.min(200, 20 * (n + 2)));
    const faces = n === 1 ? [{ id: 'f:' + gs[0].id, gear: gs[0].id, x: length / 2 }]
      : gs.map((g, k) => ({ id: 'f:' + g.id, gear: g.id,
        x: n ? +(20 + (length - 40) * k / (n - 1)).toFixed(2) : length / 2 }));
    shafts[s.id] = {
      length, minGap: LC_DEFAULTS.minGap, adjMin: LC_DEFAULTS.adj, adjMax: LC_DEFAULTS.adj,
      faces,
      bearings: [
        { id: 'b1:' + s.id, x: 8, Cr: LC_DEFAULTS.cr, locked: false },
        { id: 'b2:' + s.id, x: +(length - 8).toFixed(2), Cr: LC_DEFAULTS.cr, locked: false }],
      locked: [],
    };
  }
  return shafts;
}

function lcBuildDraft() {
  const meshes = {};
  for (const e of state.meshes)
    meshes[e.id] = { efficiency: LC_DEFAULTS.efficiency, loadFactor: LC_DEFAULTS.loadFactor };
  lcDraft = {
    name: (lcDraft && lcDraft.name) || '受载工况',
    spec: {
      inputId: state.inputId, inputTorque: LC_DEFAULTS.torque,
      inputRpm: state.inputRpm || LC_DEFAULTS.rpm, duration: LC_DEFAULTS.duration,
      meshes, branches: {}, shafts: lcDefaultLayouts(),
    },
    snapshot: JSON.parse(JSON.stringify(state)),
  };
  lcReset();
  lcSaveDraft(); lcAnalyzeSoon(); lcRender();
  flashHint('已从项目快照建立载荷工况草稿');
}

/* 刷新快照：保留仍存在啮合/轴的参数，新增给默认值，消失的丢弃 */
function lcRefreshSnapshot() {
  if (!lcDraft) return lcBuildDraft();
  const oldM = lcDraft.spec.meshes || {};
  const meshes = {};
  for (const e of state.meshes) meshes[e.id] = oldM[e.id] || {
    efficiency: LC_DEFAULTS.efficiency, loadFactor: LC_DEFAULTS.loadFactor };
  const defs = lcDefaultLayouts();
  const oldS = lcDraft.spec.shafts || {};
  const shafts = {};
  for (const sid of Object.keys(defs)) {
    const o = oldS[sid];
    if (!o) { shafts[sid] = defs[sid]; continue; }
    const fOld = Object.fromEntries((o.faces || []).map(f => [f.id, f]));
    const bOld = Object.fromEntries((o.bearings || []).map(b => [b.id, b]));
    shafts[sid] = {
      length: +o.length > 0 ? +o.length : defs[sid].length,
      minGap: +o.minGap >= 0 ? +o.minGap : defs[sid].minGap,
      adjMin: +o.adjMin >= 0 ? +o.adjMin : defs[sid].adjMin,
      adjMax: +o.adjMax >= 0 ? +o.adjMax : defs[sid].adjMax,
      locked: (o.locked || []).filter(id =>
        defs[sid].faces.some(f => f.id === id) || defs[sid].bearings.some(b => b.id === id)),
      faces: defs[sid].faces.map(f => ({ ...f, ...(fOld[f.id] || {}) })),
      bearings: defs[sid].bearings.map(b => ({ ...b, ...(bOld[b.id] || {}) })),
    };
  }
  const ids = new Set(state.shafts.map(s => s.id));
  Object.assign(lcDraft.spec, { meshes, shafts });
  if (!ids.has(lcDraft.spec.inputId)) lcDraft.spec.inputId = state.inputId;
  // 清理指向已消失啮合/轴的分支
  const mids = new Set(state.meshes.map(m => m.id));
  for (const sid of Object.keys(lcDraft.spec.branches || {})) {
    if (!ids.has(sid)) { delete lcDraft.spec.branches[sid]; continue; }
    for (const mid of Object.keys(lcDraft.spec.branches[sid]))
      if (!mids.has(mid)) delete lcDraft.spec.branches[sid][mid];
  }
  lcDraft.snapshot = JSON.parse(JSON.stringify(state));
  lcReset();
  lcSaveDraft(); lcAnalyzeSoon(); lcRender();
  flashHint('已按当前项目刷新工况快照');
}

function lcReset() {
  lcResult = null; lcSearchRes = {}; lcMeshKey = ''; lcShaftKey = '';
}

function lcSaveDraft() {
  try { localStorage.setItem(LC_KEY, JSON.stringify(lcDraft)); } catch (e) { /* ignore */ }
}

function lcSpecChanged() {
  lcSearchRes = {};
  lcSaveDraft();
  lcAnalyzeSoon();
}

/* ---------------- 后端计算 ---------------- */
function lcAnalyzeSoon() {
  clearTimeout(lcTimer);
  lcTimer = setTimeout(lcAnalyze, 150);
}

async function lcAnalyze() {
  if (!lcDraft) return;
  const token = ++lcToken;
  try {
    const resp = await fetch('/api/loadcase/analyze', {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ state: lcDraft.snapshot, spec: lcDraft.spec }),
    });
    const data = await resp.json();
    if (token !== lcToken) return;
    lcResult = data;
  } catch (e) {
    if (token === lcToken) lcResult = { ok: false, issues: [
      { severity: 'error', message: '计算失败：' + e }] };
  }
  lcRender();
}

/* ---------------- 渲染 ---------------- */
function lcRender() {
  const has = !!lcDraft;
  $('#lc-body').style.display = has ? '' : 'none';
  $('#lc-empty').style.display = has ? 'none' : '';
  if (!has) return;
  const nameInp = $('#lc-name');
  if (document.activeElement !== nameInp) nameInp.value = lcDraft.name || '';
  $('#lc-stale').style.display = lcStale() ? '' : 'none';
  lcFillInputs();
  lcRenderMeshCards();
  lcRenderBranches();
  lcRenderShafts();
  lcRenderSummary();
  lcRenderIssues();
  lcRenderSearch();
}

function lcSchedule() { if (lcDraft) $('#lc-stale').style.display = lcStale() ? '' : 'none'; }
function lcTabShown() { lcLoadVersions(); lcRender(); }

function lcFillInputs() {
  const sp = lcDraft.spec;
  const sel = $('#lc-input');
  const S = lcDraft.snapshot.shafts || [];
  const key = sel.value + '|' + S.map(s => s.id).join(',');
  if (key !== sel.dataset.key) {
    sel.dataset.key = key;
    sel.innerHTML = '';
    for (const s of S) {
      const o = h('option', { value: s.id }, sel, s.name || s.id);
      if (s.id === sp.inputId) o.selected = true;
    }
  }
  for (const [id, k] of [['#lc-torque', 'inputTorque'], ['#lc-rpm', 'inputRpm'],
                         ['#lc-duration', 'duration']]) {
    const inp = $(id);
    if (document.activeElement !== inp) inp.value = sp[k] ?? '';
  }
}

function lcMeshStructKey() {
  return JSON.stringify(Object.keys(lcDraft.spec.meshes || {}).sort()) +
    '|' + JSON.stringify((lcDraft.snapshot.shafts || []).map(s => s.id).sort()) +
    '|' + lcFingerprint(lcDraft.snapshot);
}

function lcRenderMeshCards() {
  const key = lcMeshStructKey();
  if (key === lcMeshKey) { lcUpdateMeshResults(); return; }
  lcMeshKey = key;
  const wrap = $('#lc-meshes');
  wrap.innerHTML = '';
  const G = Object.fromEntries((lcDraft.snapshot.gears || []).map(g => [g.id, g]));
  for (const e of (lcDraft.snapshot.meshes || [])) {
    const ms = lcDraft.spec.meshes[e.id];
    if (!ms) continue;
    const ga = G[e.gearA] || {}, gb = G[e.gearB] || {};
    const internal = ga.internal || gb.internal;
    const card = h('div', { class: 'lc-mesh-card', 'data-mesh': e.id }, wrap);
    const head = h('div', { class: 'bl-mesh-head' }, card);
    h('span', {}, head, `${ga.name || 'z' + ga.z} → ${gb.name || 'z' + gb.z}`);
    h('span', { class: 'muted small' }, head,
      `m=${ga.module} · z${ga.z}/z${gb.z} · ${internal ? '内啮合' : '外啮合'}`);
    const grid = h('div', { class: 'bl-mesh-grid' }, card);
    const mkNum = (label, val, step, onch) => {
      const lab = h('label', {}, grid, label);
      const inp = h('input', { type: 'number', step: String(step), value: val }, lab);
      inp.addEventListener('input', () => { onch(parseFloat(inp.value)); lcSpecChanged(); });
      return inp;
    };
    mkNum('效率 η（0~1）', ms.efficiency ?? 0.98, 0.01,
      v => { ms.efficiency = Math.min(1, Math.max(0, v || 0)); });
    mkNum('载荷系数 KA', ms.loadFactor ?? 1, 0.05,
      v => { ms.loadFactor = Math.max(0, v || 0); });
    h('div', { class: 'lc-mesh-res muted', 'data-res': e.id }, card, '');
  }
  lcUpdateMeshResults();
}

function lcUpdateMeshResults() {
  const meshes = (lcResult && lcResult.meshes) || {};
  document.querySelectorAll('#lc-meshes .lc-mesh-card').forEach(card => {
    const mid = card.getAttribute('data-mesh');
    const r = meshes[mid];
    const line = card.querySelector('[data-res]');
    card.classList.toggle('off-path', lcResult && !r);
    if (line) line.textContent = r
      ? `T ${r.torqueNm} N·m · Ft ${r.ft} N · Fr ${r.fr} N · 功率 ${r.powerKw}→${r.powerOutKw} kW`
      : (lcResult ? '不在输入动力路径上' : '');
  });
}

/* ---------------- 分支比例 ---------------- */
function lcRenderBranches() {
  const wrap = $('#lc-branches');
  const branches = (lcResult && lcResult.branches) || [];
  wrap.innerHTML = '';
  if (!branches.length) {
    wrap.style.display = 'none';
    return;
  }
  wrap.style.display = '';
  h('h3', {}, wrap, '分支载荷比例');
  const br = lcDraft.spec.branches || (lcDraft.spec.branches = {});
  for (const b of branches) {
    const card = h('div', { class: 'lc-branch-card' + (b.closed ? '' : ' conflict'),
      'data-shaft': b.shaftId }, wrap);
    const head = h('div', { class: 'bl-mesh-head' }, card);
    h('span', {}, head, `分流轴「${b.shaftName}」`);
    h('span', { class: b.closed ? 'lc-ok' : 'lc-err' }, head,
      `Σ = ${b.sum.toFixed(3)}` + (b.closed ? ' 闭合' : ' 不闭合'));
    const grid = h('div', { class: 'bl-mesh-grid' }, card);
    if (!br[b.shaftId]) br[b.shaftId] = {};
    for (const m of b.meshes) {
      const lab = h('label', {}, grid, (lcResult.meshNames || {})[m.meshId] || m.meshId);
      const inp = h('input', { type: 'number', step: '0.05', min: '0', max: '1',
        value: m.given == null ? '' : m.given }, lab);
      inp.placeholder = m.ratio.toFixed(3) + '（默认等分）';
      inp.addEventListener('input', () => {
        const v = parseFloat(inp.value);
        if (Number.isFinite(v) && v >= 0) br[b.shaftId][m.meshId] = v;
        else delete br[b.shaftId][m.meshId];
        lcSpecChanged();
      });
    }
  }
}

/* ---------------- 轴系侧视图（SVG，三层：轴/力/可拖动元素） ---------------- */
function lcRenderShafts() {
  const key = JSON.stringify((lcDraft.snapshot.shafts || []).map(s => s.id).sort());
  const wrap = $('#lc-shafts');
  if (key !== lcShaftKey) {
    lcShaftKey = key;
    wrap.innerHTML = '';
    for (const s of (lcDraft.snapshot.shafts || []))
      lcBuildShaftCard(wrap, s.id);
  }
  for (const s of (lcDraft.snapshot.shafts || []))
    lcUpdateShaftCard(s.id);
}

function lcLayout(sid) { return (lcDraft.spec.shafts || {})[sid]; }

function lcBuildShaftCard(wrap, sid) {
  const lay = lcLayout(sid);
  if (!lay) return;
  const card = h('div', { class: 'lc-shaft-card', 'data-shaft': sid }, wrap);
  const head = h('div', { class: 'lc-shaft-head' }, card);
  h('span', { class: 'lc-shaft-title', 'data-title': '' }, head);
  h('span', { class: 'lc-shaft-stat muted', 'data-stat': '' }, head);
  const grid = h('div', { class: 'lc-shaft-grid' }, card);
  const mkNum = (label, val, step, onch) => {
    const lab = h('label', {}, grid, label);
    const inp = h('input', { type: 'number', step: String(step), value: val }, lab);
    inp.addEventListener('input', () => { onch(parseFloat(inp.value)); lcSpecChanged(); });
    return inp;
  };
  mkNum('轴长 mm', lay.length, 5, v => { if (v > 0) lay.length = v; });
  mkNum('最小间距 mm', lay.minGap, 1, v => { lay.minGap = Math.max(0, v || 0); });
  mkNum('可调范围 −mm', lay.adjMin, 5, v => { lay.adjMin = Math.max(0, v || 0); });
  mkNum('可调范围 +mm', lay.adjMax, 5, v => { lay.adjMax = Math.max(0, v || 0); });

  const svgWrap = h('div', { class: 'lc-svg-wrap' }, card);
  const svg = svgEl('svg', { class: 'lc-svg', xmlns: 'http://www.w3.org/2000/svg' }, svgWrap);
  const defs = svgEl('defs', {}, svg);
  const mk = svgEl('marker', { id: 'lc-arrow-' + sid, markerWidth: 8, markerHeight: 8,
    refX: 6, refY: 3, orient: 'auto', markerUnits: 'strokeWidth' }, defs);
  svgEl('path', { d: 'M0,0 L7,3 L0,6 Z', fill: 'var(--err)' }, mk);
  const mkB = svgEl('marker', { id: 'lc-arrowb-' + sid, markerWidth: 8, markerHeight: 8,
    refX: 6, refY: 3, orient: 'auto', markerUnits: 'strokeWidth' }, defs);
  svgEl('path', { d: 'M0,0 L7,3 L0,6 Z', fill: 'var(--accent)' }, mkB);
  svgEl('g', { class: 'lc-layer-shaft' }, svg);
  svgEl('g', { class: 'lc-layer-loads' }, svg);
  svgEl('g', { class: 'lc-layer-items' }, svg);

  const row = h('div', { class: 'lc-shaft-actions' }, card);
  h('button', { onclick: () => lcSearchShaft(sid) }, row, '搜索该轴布置');
  h('div', { class: 'lc-search-list', 'data-search': sid }, card);
  card.dataset.itemkey = '';
}

const LC_VIEW = { ml: 26, mr: 26, axisY: 50, rowH: 104 };

function lcScale(svg, L) {
  const W = Math.max(260, Math.min(620, 120 + L * 3.2));
  svg.setAttribute('viewBox', `0 0 ${W} ${LC_VIEW.rowH}`);
  svg.dataset.length = L;
  return { W, X: x => LC_VIEW.ml + (W - LC_VIEW.ml - LC_VIEW.mr) *
      Math.max(0, Math.min(L, x)) / L };
}

function lcUpdateShaftCard(sid) {
  const card = document.querySelector(`#lc-shafts .lc-shaft-card[data-shaft="${CSS.escape(sid)}"]`);
  if (!card) return;
  const lay = lcLayout(sid);
  const res = lcResult && lcResult.shafts ? lcResult.shafts[sid] : null;
  const sname = (lcResult && lcResult.shaftNames && lcResult.shaftNames[sid]) || sid;
  card.querySelector('[data-title]').textContent = `轴「${sname}」`;
  card.querySelector('[data-stat]').textContent = res
    ? `${res.rpm} rpm · ${res.powerKw} kW · T ${res.torqueNm} N·m · 峰值反力 ${res.peakReaction} N` +
      ` · 强度余量 ${res.minMargin ?? '—'} · 寿命余量 ${res.minLifeMargin ?? '—'}`
    : '';
  card.classList.toggle('conflict', !!res && (res.coincident ||
    res.violations.some(v => v.code !== 'TOO_CLOSE')));

  const svg = card.querySelector('.lc-svg');
  const L = Math.max(1, +lay.length || 1);
  const { X } = lcScale(svg, L);
  const { axisY } = LC_VIEW;
  const locks = new Set(lay.locked || []);
  lay.bearings.forEach(b => { if (b.locked) locks.add(b.id); });

  // —— 第 1 层：轴体与标尺（可随时重画，不影响拖动元素的 pointer capture）——
  const lShaft = svg.querySelector('.lc-layer-shaft');
  lShaft.innerHTML = '';
  svgEl('line', { x1: LC_VIEW.ml, y1: axisY, x2: +svg.viewBox.baseVal.width - LC_VIEW.mr, y2: axisY,
    stroke: 'var(--shaft)', 'stroke-width': 4, 'stroke-linecap': 'round' }, lShaft);
  const tickStep = L <= 60 ? 10 : L <= 120 ? 20 : 50;
  for (let t = 0; t <= L + 1e-9; t += tickStep) {
    svgEl('line', { x1: X(t), y1: axisY - 5, x2: X(t), y2: axisY + 5,
      stroke: 'var(--muted)', 'stroke-width': 0.7 }, lShaft);
    svgEl('text', { x: X(t), y: LC_VIEW.rowH - 4, class: 'lc-tick', text: t }, lShaft)
      .setAttribute('text-anchor', 'middle');
  }

  // —— 第 3 层：可拖动元素，仅在结构（面/轴承 id 与归属）变化时重建 ——
  const lItems = svg.querySelector('.lc-layer-items');
  const itemSig = lay.faces.map(f => 'F' + f.id).join(',') + '|' +
    lay.bearings.map(b => 'B' + b.id).join(',');
  if (card.dataset.itemkey !== itemSig) {
    card.dataset.itemkey = itemSig;
    lItems.innerHTML = '';
    lay.bearings.forEach((b, i) => lcDrawBearing(lItems, sid, b, i, axisY, locks));
    for (const f of lay.faces) lcDrawFace(lItems, sid, f, axisY, locks);
  }
  // 位置/锁定态/标签随数据更新（拖动中的该轴不做位置回写，避免与指针打架）
  const draggingThis = lcDrag.shaftId === sid && lcDrag.id;
  const fResMap = Object.fromEntries((res ? res.faces : []).map(f => [f.id, f]));
  const bResMap = Object.fromEntries((res ? res.bearings : []).map(b => [b.id, b]));
  lItems.querySelectorAll('[data-id]').forEach(g => {
    const id = g.dataset.id;
    const item = lay.faces.find(f => f.id === id) || lay.bearings.find(b => b.id === id);
    if (!item) return;
    const base = parseFloat(g.dataset.px);
    if (!(draggingThis && id === lcDrag.id))
      g.setAttribute('transform', `translate(${X(item.x) - base},0)`);
    g.classList.toggle('locked', locks.has(id));
    const lockEl = g.querySelector('.lc-lock-sym');
    if (lockEl) lockEl.style.display = locks.has(id) ? '' : 'none';
    const fr = fResMap[id];
    if (fr) {
      const nm = g.querySelector('[data-nlabel]'), gt = g.querySelector('[data-gtext]');
      if (nm) nm.textContent = fr.name || '';
      if (gt) gt.textContent = (fr.name || '').replace(/^.*?(z\d+).*$/, '$1').slice(0, 5);
    }
    const br = bResMap[id];
    if (br) {
      const rl = g.querySelector('[data-rlabel]');
      if (rl) rl.textContent = br.r.toFixed(0) + 'N';
      g.setAttribute('data-tip',
        `${id}：反力 ${br.r} N · Cr ${br.Cr} N · 余量 ${br.margin}` +
        (br.reqMargin != null ? `（寿命需≥${br.reqMargin}）` : '') +
        ` · L10 ${br.l10h ?? '—'} h`);
      const poly = g.querySelector('polygon');
      const over = br.util > 1;
      const lifeBad = br.reqMargin != null && br.margin != null && br.margin < br.reqMargin;
      if (poly) {
        poly.setAttribute('fill', over ? '#fdecea' : lifeBad ? '#fff6e5' : '#eef3fe');
        poly.setAttribute('stroke', over ? 'var(--err)' : lifeBad ? 'var(--hl)' : 'var(--accent)');
      }
      if (rl) rl.setAttribute('fill', over ? 'var(--err)' : lifeBad ? 'var(--warn)' : 'var(--ink)');
    }
  });

  // —— 第 2 层：力箭头（齿轮力向上橙；轴承反力向下蓝）——
  const lLoads = svg.querySelector('.lc-layer-loads');
  lLoads.innerHTML = '';
  const arrowLen = r => Math.min(30, 4 + Math.log10(1 + Math.max(0, r)) * 12);
  for (const f of (res ? res.faces : [])) {
    if (!(f.r > 0.01)) continue;
    const len = arrowLen(f.r);
    svgEl('line', { x1: X(f.x), y1: axisY - 24, x2: X(f.x), y2: axisY - 24 - len,
      stroke: 'var(--hl)', 'stroke-width': 1.4,
      'marker-end': `url(#lc-arrow-${sid})` }, lLoads);
    const t = svgEl('text', { x: X(f.x) + 2, y: axisY - 27 - len,
      class: 'lc-force-label', fill: 'var(--hl)', text: f.r.toFixed(0) + 'N' }, lLoads);
  }
  for (const b of (res ? res.bearings : [])) {
    if (!(b.r > 0.01)) continue;
    const len = arrowLen(b.r);
    svgEl('line', { x1: X(b.x), y1: axisY + 26, x2: X(b.x), y2: axisY + 26 + len,
      stroke: 'var(--accent)', 'stroke-width': 1.2,
      'marker-end': `url(#lc-arrowb-${sid})` }, lLoads);
    svgEl('text', { x: X(b.x) + 2, y: axisY + 29 + len,
      class: 'lc-force-label', fill: 'var(--accent)',
      text: b.r.toFixed(0) + 'N' }, lLoads);
  }
}

function lcDrawBearing(parent, sid, b, i, axisY, locks) {
  const side = i === 0 ? 1 : -1;
  const g = svgEl('g', { class: 'lc-drag', 'data-id': b.id, 'data-shaft': sid,
    cursor: 'ew-resize' }, parent);
  g.dataset.px = '0';
  svgEl('polygon', { points: `-7,${side * 7} 7,${side * 7} 0,${side * 1}`,
    fill: '#eef3fe', stroke: 'var(--accent)', 'stroke-width': 1 }, g);
  for (let k = -1; k <= 1; k++)
    svgEl('line', { x1: k * 4 - 3, y1: side * 11, x2: k * 4 + 3, y2: side * 5,
      stroke: 'var(--muted)', 'stroke-width': 0.7 }, g);
  const rl = svgEl('text', { class: 'lc-tick', y: side * 22,
    'text-anchor': 'middle', 'data-rlabel': '1', text: '' }, g);
  svgEl('text', { class: 'lc-lock-sym lc-lock', x: 9, y: side * 7 - 4,
    text: '🔒' }, g);
  lcBindDrag(g);
}

function lcDrawFace(parent, sid, f, axisY, locks) {
  const g = svgEl('g', { class: 'lc-drag', 'data-id': f.id, 'data-shaft': sid,
    cursor: 'ew-resize' }, parent);
  g.dataset.px = '0';
  svgEl('rect', { x: -6, y: axisY - 9, width: 12, height: 18, rx: 2,
    fill: '#f2ebe0', stroke: 'var(--gear-in)', 'stroke-width': 1.1 }, g);
  svgEl('text', { class: 'lc-face-g', y: axisY + 3, 'data-gtext': '1' }, g)
    .setAttribute('text-anchor', 'middle');
  svgEl('text', { class: 'lc-tick', y: axisY - 13, 'text-anchor': 'middle',
    'data-nlabel': '1' }, g);
  svgEl('text', { class: 'lc-lock-sym lc-lock', x: 8, y: axisY - 4,
    text: '🔒' }, g);
  lcBindDrag(g);
}

function lcSvgToX(svg, clientX) {
  const rect = svg.getBoundingClientRect();
  const vb = svg.viewBox.baseVal;
  const px = (clientX - rect.left) / rect.width * vb.width;
  const L = +svg.dataset.length;
  return Math.max(0, Math.min(L,
    (px - LC_VIEW.ml) / (vb.width - LC_VIEW.ml - LC_VIEW.mr) * L));
}

function lcBindDrag(g) {
  g.addEventListener('pointerdown', e => {
    e.preventDefault(); e.stopPropagation();
    const sid = g.dataset.shaft, id = g.dataset.id;
    const lay = lcLayout(sid);
    const locks = new Set(lay.locked || []);
    lay.bearings.forEach(b => { if (b.locked) locks.add(b.id); });
    if (locks.has(id)) { flashHint('该位置已锁定'); return; }
    g.setPointerCapture(e.pointerId);
    lcDrag.id = id; lcDrag.shaftId = sid;
    lcDrag.svg = g.closest('svg'); lcDrag.moved = false;
  });
  g.addEventListener('pointermove', e => {
    if (lcDrag.id !== g.dataset.id) return;
    const x = +lcSvgToX(lcDrag.svg, e.clientX).toFixed(1);
    const lay = lcLayout(lcDrag.shaftId);
    const item = lay.faces.find(f => f.id === lcDrag.id)
      || lay.bearings.find(b => b.id === lcDrag.id);
    if (!item) return;
    item.x = x; lcDrag.moved = true;
    const L = +lay.length;
    const { X } = lcScale(lcDrag.svg, L);
    g.setAttribute('transform', `translate(${X(x)},0)`);
    if (!lcDrag.timer) {
      lcDrag.timer = setTimeout(() => {
        lcDrag.timer = null;
        lcSaveDraft(); lcAnalyzeSoon();
        // 仅刷新力箭头层，不重建拖动元素
        const card = lcDrag.svg.closest('.lc-shaft-card');
        const sid2 = card.getAttribute('data-shaft');
        lcUpdateShaftCard(sid2);
      }, 90);
    }
  });
  const end = e => {
    if (lcDrag.id !== g.dataset.id) return;
    const sid = lcDrag.shaftId;
    clearTimeout(lcDrag.timer);
    lcDrag.id = null; lcDrag.shaftId = null; lcDrag.timer = null;
    lcSaveDraft(); lcAnalyzeSoon();
    lcUpdateShaftCard(sid);
  };
  g.addEventListener('pointerup', end);
  g.addEventListener('pointercancel', end);
  g.addEventListener('dblclick', e => {
    e.preventDefault(); e.stopPropagation();
    const sid0 = g.dataset.shaft, id0 = g.dataset.id;
    const lay0 = lcLayout(sid0);
    const isB = lay0.bearings.some(b => b.id === id0);
    let lockArr = lay0.locked || (lay0.locked = []);
    const k = lockArr.indexOf(id0);
    if (k >= 0) lockArr.splice(k, 1); else lockArr.push(id0);
    // 与单项 locked 字段保持同步
    const item0 = lay0.faces.find(f => f.id === id0) || lay0.bearings.find(b => b.id === id0);
    if (isB) item0.locked = k < 0;
    lcSaveDraft(); lcAnalyzeSoon(); lcUpdateShaftCard(sid0);
    flashHint(k >= 0 ? '已解除锁定' : '已锁定该位置（搜索时不移动）');
  });
}

/* ---------------- 汇总与诊断 ---------------- */
function lcRenderSummary() {
  const box = $('#lc-summary'), badge = $('#lc-badge');
  if (!lcResult) { box.textContent = '正在计算…'; box.classList.add('muted'); badge.textContent = ''; return; }
  box.classList.remove('muted');
  const errs = (lcResult.issues || []).filter(i => i.severity === 'error');
  if (errs.length) {
    box.innerHTML = errs.map(i => `<b>${i.message}</b>`).join('；');
    badge.textContent = '无法平衡'; badge.className = 'badge err';
    return;
  }
  const overload = Object.values(lcResult.shafts || {}).flatMap(s =>
    s.bearings.filter(b => b.util > 1).map(b => `${s.name} ${b.id}`));
  const lifeShort = Object.values(lcResult.shafts || {}).flatMap(s =>
    s.bearings.filter(b => b.reqMargin && b.margin != null && b.margin < b.reqMargin)
      .map(b => `${s.name} ${b.id}(${b.l10h}h)`));
  badge.textContent = overload.length ? `${overload.length} 处超载`
    : lifeShort.length ? `${lifeShort.length} 处寿命不足` : '正常';
  badge.className = 'badge ' + ((overload.length || lifeShort.length) ? 'warn' : '');
  box.innerHTML = `输入 ${lcResult.input.torqueNm} N·m @ ${lcResult.input.rpm} rpm` +
    ` = <b>${lcResult.input.powerKw} kW</b>；末端输出合计 <b>${lcResult.outputPowerKw} kW</b>` +
    `（损失 ${lcResult.lossKw} kW）；持续 ${lcResult.durationH} h；` +
    `全列峰值轴承反力 <b>${lcResult.peakReaction} N</b>，最小安全余量 <b>${lcResult.minMargin ?? '—'}</b>` +
    (overload.length ? `；<b style="color:var(--err)">超载轴承：${overload.join('、')}</b>` : '') +
    (lifeShort.length ? `；<b style="color:var(--warn)">寿命不足：${lifeShort.join('、')}</b>` : '');
}

function lcRenderIssues() {
  const ul = $('#lc-issues');
  ul.innerHTML = '';
  if (!lcResult) return;
  const sevRank = { error: 0, warning: 1, info: 2 };
  const list = (lcResult.issues || []).slice()
    .sort((a, b) => sevRank[a.severity] - sevRank[b.severity]);
  if (!list.length) { h('li', { class: 'info' }, ul, '✓ 功率流闭合，各支点载荷均可平衡'); return; }
  for (const it of list) {
    const li = h('li', { class: it.severity }, ul);
    h('span', { class: 'sev' }, li,
      it.severity === 'error' ? '✗ 错误' : it.severity === 'warning' ? '⚠ 警告' : 'ℹ 提示');
    li.appendChild(document.createTextNode(it.message));
    li.addEventListener('click', () => lcLocate(it));
  }
}

function lcLocate(it) {
  const refs = it.refs || {};
  const sid = refs.shaft;
  const card = sid && document.querySelector(
    `#lc-shafts .lc-shaft-card[data-shaft="${CSS.escape(sid)}"]`);
  if (card) {
    card.scrollIntoView({ behavior: 'smooth', block: 'center' });
    card.classList.add('locate-flash');
    setTimeout(() => card.classList.remove('locate-flash'), 1600);
  }
  const mid = refs.mesh || refs.m0;
  const mc = mid && document.querySelector(`#lc-meshes .lc-mesh-card[data-mesh="${CSS.escape(mid)}"]`);
  if (mc) {
    mc.scrollIntoView({ behavior: 'smooth', block: 'center' });
    mc.classList.add('locate-flash');
    setTimeout(() => mc.classList.remove('locate-flash'), 1600);
  }
}

/* ---------------- 布置搜索 ---------------- */
async function lcSearchShaft(sid) {
  if (!lcDraft) return;
  const lay = lcLayout(sid);
  const list = document.querySelector(`[data-search="${CSS.escape(sid)}"]`);
  list.textContent = '搜索中…';
  try {
    const resp = await fetch('/api/loadcase/search', {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        state: lcDraft.snapshot, spec: lcDraft.spec,
        shaftId: sid,
        grid: parseFloat($('#lc-grid').value) || 2,
        limit: 10, timeBudget: 6,
      }),
    });
    const data = await resp.json();
    lcSearchRes[sid] = data.shafts && data.shafts[sid];
  } catch (e) { list.textContent = '搜索失败：' + e; return; }
  lcRenderSearch();
}

function lcRenderSearch() {
  for (const sid of Object.keys(lcSearchRes)) {
    const list = document.querySelector(`[data-search="${CSS.escape(sid)}"]`);
    if (!list) continue;
    list.innerHTML = '';
    const r = lcSearchRes[sid];
    if (!r) continue;
    const meta = h('div', { class: 'muted small' }, list,
      `枚举 ${r.nodes} 个节点${r.truncated ? '（达到上限已截断）' : ''}，按超限→最小余量→峰值反力→改动量排序`);
    (r.results || []).forEach((c, i) => {
      const li = h('div', { class: 'cand lc-cand' }, list);
      const head = h('div', { class: 'head' }, li);
      h('span', {}, head, `#${i + 1} ` +
        (c.violations ? `超限 ${c.violations}` : '无超限') +
        ` · 最小余量 ${c.minMargin ?? '—'} · 峰值 ${c.peakReaction} N`);
      h('span', { class: 'muted small' }, head, `改动 ${c.change} mm`);
      const tags = [];
      if (c.nOverload) tags.push(`超载 ${c.nOverload}`);
      if (c.nLife) tags.push(`寿命不足 ${c.nLife}`);
      if (c.nGap) tags.push(`间距 ${c.nGap}`);
      if (c.nRange) tags.push(`越界 ${c.nRange}`);
      if (c.nOutside) tags.push(`悬臂面 ${c.nOutside}`);
      if (tags.length) h('div', { class: 'meta', style: 'color:var(--err)' }, li, tags.join('；'));
      h('div', { class: 'meta' }, li,
        Object.entries(c.positions).map(([id, x]) => `${id.replace(/^[fb]\d?:/, '')}:${x}`).join('  '));
      const acts = h('div', { class: 'actions' }, li);
      h('button', { class: 'primary', onclick: () => lcApplyCandidate(sid, c) },
        acts, '采用并更新布置');
    });
  }
}

function lcApplyCandidate(sid, c) {
  const lay = lcLayout(sid);
  for (const f of lay.faces) if (c.positions[f.id] != null) f.x = c.positions[f.id];
  for (const b of lay.bearings) if (c.positions[b.id] != null) b.x = c.positions[b.id];
  lcSpecChanged();
  lcRender();
  flashHint('已采用布置方案（可继续调整或另存版本）');
}

/* ---------------- 版本 ---------------- */
async function lcLoadVersions() {
  try {
    lcVersions = await (await fetch('/api/loadcase/cases')).json();
  } catch (e) { lcVersions = []; }
  const sel = $('#lc-versions');
  const cur = sel.value;
  sel.innerHTML = '';
  if (!lcVersions.length) h('option', { value: '' }, sel, '— 尚无已存版本 —');
  for (const v of lcVersions) {
    const o = h('option', { value: v.id }, sel,
      `${v.name} · v${v.version}${v.note ? `（${v.note}）` : ''}`);
    if (String(v.id) === cur) o.selected = true;
  }
}

async function lcSaveVersion() {
  if (!lcDraft) return;
  const resp = await fetch('/api/loadcase/cases', {
    method: 'POST', headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({
      name: lcDraft.name || '受载工况',
      spec: lcDraft.spec, snapshot: lcDraft.snapshot,
      note: null,
    }),
  });
  const data = await resp.json();
  if (data.ok) {
    flashHint(`已另存载荷工况版本 v${data.version}（原轮系方案未改动）`);
    lcLoadVersions();
  }
}

/* ---------------- 事件绑定与启动 ---------------- */
function lcBind() {
  $('#btn-lc-new').addEventListener('click', lcBuildDraft);
  $('#btn-lc-refresh').addEventListener('click', lcRefreshSnapshot);
  $('#lc-name').addEventListener('input', e => {
    if (lcDraft) { lcDraft.name = e.target.value; lcSaveDraft(); }
  });
  $('#lc-input').addEventListener('change', e => {
    lcDraft.spec.inputId = e.target.value || null; lcSpecChanged();
  });
  $('#lc-torque').addEventListener('input', e => {
    lcDraft.spec.inputTorque = Math.max(0, parseFloat(e.target.value) || 0); lcSpecChanged();
  });
  $('#lc-rpm').addEventListener('input', e => {
    lcDraft.spec.inputRpm = Math.max(0, parseFloat(e.target.value) || 0); lcSpecChanged();
  });
  $('#lc-duration').addEventListener('input', e => {
    lcDraft.spec.duration = Math.max(0, parseFloat(e.target.value) || 0); lcSpecChanged();
  });
  $('#btn-lc-save').addEventListener('click', lcSaveVersion);
  $('#btn-lc-load').addEventListener('click', async () => {
    const id = $('#lc-versions').value;
    if (!id) return;
    const v = await (await fetch(`/api/loadcase/cases/${id}`)).json();
    if (!v.spec) return;
    lcDraft = { name: v.name, spec: v.spec, snapshot: v.snapshot };
    lcReset();
    lcSaveDraft(); lcAnalyzeSoon(); lcRender();
    flashHint(`已载入载荷工况「${v.name}」v${v.version}（源轮系变化仅标记过期）`);
  });
  $('#btn-lc-del').addEventListener('click', async () => {
    const id = $('#lc-versions').value;
    if (!id) return;
    await fetch(`/api/loadcase/cases/${id}`, { method: 'DELETE' });
    lcLoadVersions();
  });
}

(function lcInit() {
  try {
    const raw = localStorage.getItem(LC_KEY);
    if (raw) lcDraft = JSON.parse(raw);
  } catch (e) { lcDraft = null; }
  lcBind();
  lcRender();
  if (lcDraft) lcAnalyze();
  lcLoadVersions();
})();
