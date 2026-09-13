/* 回程间隙工况工作区 — 原生 JS/SVG；计算由后端 backlash.py 完成。
   草稿 = {name, spec, snapshot}：spec 记录一次换向的输入/观察轴、换向角与各啮合
   侧隙参数；snapshot 为建立时的项目快照，源轮系齿轮或轴位变化时标记过期。 */
'use strict';

const BL_GRADES = [
  { id: 'g0', name: '消隙（零背隙）', factor: 0.0 },
  { id: 'g1', name: '精密级 0.03m', factor: 0.03 },
  { id: 'g2', name: '标准级 0.06m', factor: 0.06 },
  { id: 'g3', name: '宽松级 0.10m', factor: 0.10 },
  { id: 'g4', name: '大间隙级 0.16m', factor: 0.16 },
];
const BL_GRADE_MAP = Object.fromEntries(BL_GRADES.map(g => [g.id, g]));
const BL_KEY = 'backlashDraftV1';

let blDraft = null;          // {name, spec, snapshot}
let blResult = null;         // 最近一次 /api/backlash/analyze 结果
let blToken = 0;
let blTimer = null;
let blAngle = 0;             // 当前换向角（度，输入轴）
let blPlaying = false;
let blPlayT0 = 0;
let blSvgRefs = null;
let blMeshKey = '';
let blVersions = [];
let blSearchRes = [];
let blSelSol = -1;
let blSolSpec = null, blSolResult = null;   // 叠加预览中的搜索方案
let blCmpResult = null, blCmpLabel = '';    // 对照基线版本的分析结果

/* ---------------- 快照与过期 ---------------- */
function blFingerprint(st) {
  const S = Object.fromEntries((st.shafts || []).map(s => [s.id, s]));
  const G = Object.fromEntries((st.gears || []).map(g => [g.id, g]));
  const items = [];
  for (const e of (st.meshes || [])) {
    const ga = G[e.gearA], gb = G[e.gearB];
    if (!ga || !gb) { items.push([e.id, 'broken']); continue; }
    const sa = S[ga.shaftId] || {}, sb = S[gb.shaftId] || {};
    items.push([e.id, ga.z, ga.module, ga.x || 0, gb.z, gb.module, gb.x || 0,
      +sa.x || 0, +sa.y || 0, +sb.x || 0, +sb.y || 0]);
  }
  items.sort((a, b) => String(a[0]).localeCompare(String(b[0])));
  return JSON.stringify(items);
}

function blStale() {
  return !!(blDraft && blFingerprint(blDraft.snapshot) !== blFingerprint(state));
}

function blDefaultJn(e) {
  const G = Object.fromEntries(state.gears.map(g => [g.id, g]));
  const ga = G[e.gearA];
  const m = ga && ga.module > 0 ? ga.module : 1;
  return +(BL_GRADE_MAP.g2.factor * m).toFixed(3);
}

function blBuildDraft() {
  const meshes = {};
  for (const e of state.meshes) {
    meshes[e.id] = { jn: blDefaultJn(e), jnLocked: false, grade: 'g2',
      dA: 0, xSum: 0, centerTol: 0.05, ecc: 0.02, flank: 'drive' };
  }
  blDraft = {
    name: (blDraft && blDraft.name) || '换向工况',
    spec: { inputId: state.inputId, observeId: state.outputId,
            reversalDeg: 30, meshes },
    snapshot: JSON.parse(JSON.stringify(state)),
  };
  blResetResult();
  blSaveDraft(); blAnalyzeSoon(); blRender();
  flashHint('已从项目快照建立回程间隙工况草稿');
}

/* 刷新快照：保留仍存在啮合的用户参数，新增啮合给默认值，消失的丢弃 */
function blRefreshSnapshot() {
  if (!blDraft) return blBuildDraft();
  const old = blDraft.spec.meshes || {};
  const meshes = {};
  for (const e of state.meshes) meshes[e.id] = old[e.id] || {
    jn: blDefaultJn(e), jnLocked: false, grade: 'g2',
    dA: 0, xSum: 0, centerTol: 0.05, ecc: 0.02, flank: 'drive' };
  blDraft.spec.meshes = meshes;
  const ids = new Set(state.shafts.map(s => s.id));
  if (!ids.has(blDraft.spec.inputId)) blDraft.spec.inputId = state.inputId;
  if (!ids.has(blDraft.spec.observeId)) blDraft.spec.observeId = state.outputId;
  blDraft.snapshot = JSON.parse(JSON.stringify(state));
  blResetResult();
  blSaveDraft(); blAnalyzeSoon(); blRender();
  flashHint('已按当前项目刷新工况快照');
}

function blResetResult() {
  blResult = null; blSvgRefs = null; blMeshKey = '';
  blSearchRes = []; blSelSol = -1; blSolSpec = null; blSolResult = null;
  blAngle = 0; blPlaying = false;
}

function blSaveDraft() {
  try { localStorage.setItem(BL_KEY, JSON.stringify(blDraft)); } catch (e) { /* ignore */ }
}

/* ---------------- 后端计算 ---------------- */
function blAnalyzeSoon() {
  clearTimeout(blTimer);
  blTimer = setTimeout(blAnalyze, 250);
}

async function blAnalyze() {
  if (!blDraft) return;
  const token = ++blToken;
  try {
    const resp = await fetch('/api/backlash/analyze', {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ state: blDraft.snapshot, spec: blDraft.spec }),
    });
    const data = await resp.json();
    if (token !== blToken) return;
    blResult = data;
  } catch (e) {
    if (token === blToken) blResult = { ok: false, issues: [
      { severity: 'error', message: '计算失败：' + e }] };
  }
  blAngle = 0;
  blRender();
}

function blSpecChanged() {
  blSolSpec = null; blSolResult = null; blSelSol = -1;
  blSaveDraft();
  blAnalyzeSoon();
}

/* ---------------- 面板渲染 ---------------- */
function blMaxAngle() {
  if (!blResult) return 30;
  return Math.max(blResult.reversalDeg || 30, (blResult.totalSwitchDeg || 0) * 1.15, 5);
}

function blRender() {
  const has = !!blDraft;
  $('#bl-body').style.display = has ? '' : 'none';
  $('#bl-empty').style.display = has ? 'none' : '';
  if (!has) return;
  const nameInp = $('#bl-name');
  if (document.activeElement !== nameInp) nameInp.value = blDraft.name || '';
  blUpdateStale();
  blFillShaftSelects();
  blRenderMeshCards();
  blUpdateMeshCardResults();
  blRenderSummary();
  blRenderStageList();
  blDrawSVG();
  const sl = $('#bl-angle');
  sl.max = String(blMaxAngle());
  sl.value = String(Math.min(blAngle, blMaxAngle()));
  blUpdateMotion();
  blRenderSearchResults();
  blDrawCompare();
}

function blUpdateStale() {
  $('#bl-stale').style.display = blStale() ? '' : 'none';
}

/* 由 app.js renderPanels() 调用：源轮系变化时刷新过期标记 */
function blSchedule() {
  if (!blDraft) return;
  blUpdateStale();
}

function blTabShown() {
  blLoadVersions();
  blRender();
}

function blFillShaftSelects() {
  const S = blDraft.snapshot.shafts || [];
  for (const [id, key] of [['#bl-input', 'inputId'], ['#bl-observe', 'observeId']]) {
    const sel = $(id);
    const cur = blDraft.spec[key];
    sel.innerHTML = '';
    for (const s of S) {
      const o = h('option', { value: s.id }, sel, s.name || s.id);
      if (s.id === cur) o.selected = true;
    }
  }
  const rev = $('#bl-reversal');
  if (document.activeElement !== rev) rev.value = blDraft.spec.reversalDeg ?? 30;
}

function blMeshStructKey() {
  return JSON.stringify(Object.keys(blDraft.spec.meshes || {}).sort()) +
    '|' + blFingerprint(blDraft.snapshot);
}

function blRenderMeshCards() {
  const key = blMeshStructKey();
  if (key === blMeshKey) return;
  blMeshKey = key;
  const wrap = $('#bl-meshes');
  wrap.innerHTML = '';
  const G = Object.fromEntries((blDraft.snapshot.gears || []).map(g => [g.id, g]));
  for (const e of (blDraft.snapshot.meshes || [])) {
    const ms = blDraft.spec.meshes[e.id];
    if (!ms) continue;
    const ga = G[e.gearA] || {}, gb = G[e.gearB] || {};
    const internal = ga.internal || gb.internal;
    const card = h('div', { class: 'bl-mesh-card', 'data-mesh': e.id }, wrap);
    const head = h('div', { class: 'bl-mesh-head' }, card);
    h('span', {}, head, `${ga.name || 'z' + ga.z} → ${gb.name || 'z' + gb.z}`);
    h('span', { class: 'muted small' }, head,
      `m=${ga.module} · z${ga.z}/z${gb.z} · ${internal ? '内啮合' : '外啮合'}`);
    const grid = h('div', { class: 'bl-mesh-grid' }, card);
    const mkNum = (label, val, step, onch) => {
      const lab = h('label', {}, grid, label);
      const inp = h('input', { type: 'number', step: String(step), value: val }, lab);
      inp.addEventListener('input', () => { onch(parseFloat(inp.value)); blSpecChanged(); });
      return inp;
    };
    mkNum('法向侧隙 jn mm（实测）', ms.jn, 0.005, v => { ms.jn = v || 0; });
    const cbLab = h('label', { class: 'cb' }, grid);
    const cb = h('input', { type: 'checkbox' }, cbLab);
    cb.checked = !!ms.jnLocked;
    cbLab.appendChild(document.createTextNode('锁定实测间隙'));
    cb.addEventListener('change', () => { ms.jnLocked = cb.checked; blMeshKey = ''; blSpecChanged(); blRender(); });
    const gLab = h('label', {}, grid, '间隙等级');
    const gSel = h('select', {}, gLab);
    for (const g of BL_GRADES) {
      const o = h('option', { value: g.id }, gSel, g.name);
      if ((ms.grade || 'g2') === g.id) o.selected = true;
    }
    gSel.disabled = !!ms.jnLocked;
    gSel.addEventListener('change', () => { ms.grade = gSel.value; blSpecChanged(); });
    mkNum('中心距公差 ±mm', ms.centerTol, 0.01, v => { ms.centerTol = Math.max(0, v || 0); });
    mkNum('偏心量 mm', ms.ecc, 0.005, v => { ms.ecc = Math.max(0, v || 0); });
    const fLab = h('label', {}, grid, '初始贴合齿面');
    const fSel = h('select', {}, fLab);
    for (const [v, t] of [['drive', '工作面（换向需越隙）'], ['coast', '非工作面（不换面）']]) {
      const o = h('option', { value: v }, fSel, t);
      if ((ms.flank || 'drive') === v) o.selected = true;
    }
    fSel.addEventListener('change', () => { ms.flank = fSel.value; blSpecChanged(); });
    h('div', { class: 'bl-mesh-contrib muted', 'data-contrib': e.id }, card, '');
  }
}

function blUpdateMeshCardResults() {
  const conflict = new Set((blResult && blResult.conflictMeshes) || []);
  const onPath = new Set(Object.keys((blResult && blResult.meshNames) || {}));
  const stageMap = Object.fromEntries(((blResult && blResult.stages) || []).map(s => [s.meshId, s]));
  document.querySelectorAll('#bl-meshes .bl-mesh-card').forEach(card => {
    const mid = card.getAttribute('data-mesh');
    card.classList.toggle('conflict', conflict.has(mid));
    card.classList.toggle('off-path', !!blResult && blResult.ok !== undefined && !onPath.has(mid) && !!blResult.stages);
    const line = card.querySelector('[data-contrib]');
    const st = stageMap[mid];
    if (line) {
      line.textContent = st
        ? `换面角 ${st.phiIn}°（输入）→ 空程贡献 ${st.phiObs}°（观察轴，${st.phiObsMin}~${st.phiObsMax}）`
        : (onPath.has(mid) ? '' : '不在输入→观察路径上，未参与计算');
    }
  });
}

function blRenderSummary() {
  const box = $('#bl-summary');
  const badge = $('#bl-badge');
  if (!blResult) {
    box.textContent = '正在计算…'; box.classList.add('muted');
    badge.textContent = ''; badge.className = 'badge';
    return;
  }
  box.classList.remove('muted');
  const errs = (blResult.issues || []).filter(i => i.severity === 'error');
  if (errs.length) {
    box.textContent = errs.map(i => i.message).join('；');
    badge.textContent = '无法计算'; badge.className = 'badge err';
    return;
  }
  if (!blResult.compatible) {
    const names = (blResult.conflictMeshes || []).map(mid =>
      (blResult.meshNames || {})[mid] || mid).join('、');
    box.innerHTML = `闭环/分支路径的相容区间 <b>无交集</b>（各路径区间：` +
      blResult.paths.map(p => `[${p.lostMin}°, ${p.lostMax}°]`).join(' ∩ ') +
      `）；<b>冲突啮合：${names || '—'}</b>。可增大侧隙或调整初始贴合面。`;
    badge.textContent = '区间冲突'; badge.className = 'badge err';
    return;
  }
  const multi = (blResult.paths || []).length > 1;
  box.innerHTML = `观察轴空程：最小 <b>${blResult.lostMin}°</b>，最大 <b>${blResult.lostMax}°</b>` +
    `（折算输入轴 ${blResult.lostMinIn}° ~ ${blResult.lostMaxIn}°）；` +
    `路径 ${blResult.paths.length} 条` +
    (multi ? `，相容区间 [${blResult.lostMin}°, ${blResult.lostMax}°]` : '') +
    `；全部换面需输入 ${blResult.totalSwitchDeg}°` +
    ((blResult.reversalDeg || 0) < blResult.totalSwitchDeg
      ? ` <b style="color:var(--err)">超过换向角 ${blResult.reversalDeg}°！</b>` : '');
  badge.textContent = '正常'; badge.className = 'badge';
}

function blRenderStageList() {
  const ol = $('#bl-stages');
  ol.innerHTML = '';
  for (const [i, st] of ((blResult && blResult.stages) || []).entries()) {
    const li = h('li', {}, ol);
    li.textContent = `级${i + 1} ${st.driver}→${st.driven}：` +
      (st.factor === 0
        ? '初始贴合非工作面，换向不换面（贡献 0）'
        : `${st.startDeg}° 离开工作面 → ${st.switchDeg}° 换面完成（输入轴）；` +
          `贡献空程 ${st.phiObs}°（观察轴，${st.phiObsMin}~${st.phiObsMax}）；` +
          `jn ${st.jnMin}~${st.jnMax} mm`);
  }
  if (blResult && (blResult.paths || []).length > 1) {
    const li = h('li', { class: 'muted' }, ol);
    li.textContent = `闭环/分支：共 ${blResult.paths.length} 条路径参与相容区间，侧栏按主路径（级数最少）展开。`;
  }
}

/* ---------------- SVG 换向动画 ---------------- */
function blDrawSVG() {
  const svg = $('#bl-svg');
  svg.innerHTML = '';
  blSvgRefs = null;
  const stages = (blResult && blResult.stages) || [];
  if (!blResult || !stages.length || !(blResult.totalSwitchDeg >= 0)) {
    svg.style.display = 'none';
    return;
  }
  svg.style.display = '';
  const maxA = blMaxAngle();
  const H = 52 + stages.length * 64 + 6;
  svg.setAttribute('viewBox', `0 0 360 ${H}`);
  const refs = { stages: [] };
  const x0 = 30, x1 = 352;
  const X = a => x0 + (x1 - x0) * Math.min(1, Math.max(0, a / maxA));
  refs.X = X;

  refs.summary = svgEl('text', { x: 8, y: 12, class: 'bl-svg-label' }, svg);

  /* 时间轴：0..maxA，橙色=越隙区，蓝色=观察轴随动区 */
  const tlY = 34;
  const ts = blResult.totalSwitchDeg || 0;
  svgEl('rect', { x: X(0), y: tlY - 3, width: Math.max(0, X(ts) - X(0)), height: 6,
    fill: 'rgba(224,142,69,.28)', rx: 2 }, svg);
  svgEl('rect', { x: X(ts), y: tlY - 3, width: Math.max(0, X(maxA) - X(ts)), height: 6,
    fill: 'rgba(91,141,239,.20)', rx: 2 }, svg);
  svgEl('line', { x1: x0, y1: tlY, x2: x1, y2: tlY, stroke: '#b7ad9c', 'stroke-width': 0.8 }, svg);
  const rev = blResult.reversalDeg || 0;
  if (rev > 0) {
    svgEl('line', { x1: X(rev), y1: tlY - 8, x2: X(rev), y2: tlY + 8,
      stroke: 'var(--err)', 'stroke-width': 1, 'stroke-dasharray': '2 2' }, svg);
    svgEl('text', { x: X(rev), y: tlY + 16, class: 'bl-tick-label', text: `换向角 ${rev}°` }, svg);
  }
  for (const st of stages) {
    svgEl('line', { x1: X(st.switchDeg), y1: tlY - 5, x2: X(st.switchDeg), y2: tlY + 5,
      stroke: 'var(--gear)', 'stroke-width': 1 }, svg);
    svgEl('text', { x: X(st.switchDeg), y: tlY - 8, class: 'bl-tick-label',
      text: `${st.switchDeg}°` }, svg);
  }
  refs.cursor = svgEl('line', { x1: X(0), y1: tlY - 9, x2: X(0), y2: tlY + 9,
    stroke: 'var(--err)', 'stroke-width': 1.6 }, svg);

  /* 各级条带 */
  stages.forEach((st, i) => {
    const y0 = 52 + i * 64;
    const g = svgEl('g', {}, svg);
    svgEl('text', { x: 8, y: y0 + 9, class: 'bl-svg-label',
      text: `级${i + 1} ${st.driver} → ${st.driven}` }, g);
    const stateTxt = svgEl('text', { x: 8, y: y0 + 19, class: 'bl-svg-state' }, g);
    svgEl('line', { x1: 60, y1: y0 + 38, x2: 300, y2: y0 + 38,
      stroke: '#c9c0b0', 'stroke-width': 0.8 }, g);

    /* 主动轮（左）与从动轮（右），旋转刻线随换向角转动 */
    const mkWheel = cx => {
      svgEl('circle', { cx, cy: y0 + 38, r: 13, fill: '#fffdf8',
        stroke: 'var(--gear)', 'stroke-width': 1.1 }, g);
      const rot = svgEl('g', {}, g);
      svgEl('line', { x1: cx, y1: y0 + 38, x2: cx, y2: y0 + 27,
        stroke: 'var(--gear)', 'stroke-width': 1.3 }, rot);
      return rot;
    };
    const dRot = mkWheel(46);
    const rRot = mkWheel(314);

    /* 齿面示意：从动齿（上，固定）与主动齿（下，随越隙右移） */
    svgEl('polygon', { points: `172,${y0 + 22} 188,${y0 + 22} 184,${y0 + 36} 176,${y0 + 36}`,
      fill: '#d8d0c2', stroke: 'var(--gear-in)', 'stroke-width': 0.7 }, g);
    const driverTooth = svgEl('polygon', { points: '',
      fill: '#e8dfd0', stroke: 'var(--gear)', 'stroke-width': 0.8 }, g);
    const leftDot = svgEl('circle', { cx: 172, cy: y0 + 37, r: 2.2, fill: 'var(--ok)' }, g);
    const rightDot = svgEl('circle', { cx: 188, cy: y0 + 37, r: 2.2, fill: 'var(--accent)' }, g);
    svgEl('text', { x: 180, y: y0 + 62, class: 'bl-tick-label',
      text: `jn ${st.jnMin}~${st.jnMax} mm（示意，未按比例）` }, g)
      .setAttribute('text-anchor', 'middle');
    refs.stages.push({ y0, stateTxt, dRot, rRot, driverTooth, leftDot, rightDot });
  });
  blSvgRefs = refs;
}

function blUpdateMotion() {
  if (!blSvgRefs || !blResult) return;
  const theta = blAngle;
  const stages = blResult.stages || [];
  const ts = blResult.totalSwitchDeg || 0;
  blSvgRefs.cursor.setAttribute('x1', blSvgRefs.X(theta));
  blSvgRefs.cursor.setAttribute('x2', blSvgRefs.X(theta));
  blSvgRefs.summary.textContent =
    `输入轴反向 ${theta.toFixed(1)}° · ` +
    (theta >= ts ? '观察轴随动中' : `空程中（全部换面需 ${ts}°）`);
  stages.forEach((st, i) => {
    const r = blSvgRefs.stages[i];
    const span = st.switchDeg - st.startDeg;
    const p = st.factor === 0 ? 1
      : span <= 1e-9 ? (theta >= st.switchDeg ? 1 : 0)
      : Math.max(0, Math.min(1, (theta - st.startDeg) / span));
    /* 主动齿位置：p=0 贴工作面（左），p=1 贴非工作面（右） */
    const x = 160 + p * 28;
    r.driverTooth.setAttribute('points',
      `${x},${r.y0 + 54} ${x + 12},${r.y0 + 54} ${x + 8},${r.y0 + 38} ${x + 4},${r.y0 + 38}`);
    /* 主动轴自 startDeg 起转，从动轴自 switchDeg 起转 */
    const dAng = -Math.max(0, theta - st.startDeg) * (st.nDriver ?? 1);
    r.dRot.setAttribute('transform', `rotate(${dAng} 46 ${r.y0 + 38})`);
    const nDriven = i + 1 < stages.length ? stages[i + 1].nDriver : blResult.nObserve;
    const rAng = -Math.max(0, theta - st.switchDeg) * (nDriven ?? 1);
    r.rRot.setAttribute('transform', `rotate(${rAng} 314 ${r.y0 + 38})`);
    r.leftDot.style.display = p <= 0.001 ? '' : 'none';
    r.rightDot.style.display = p >= 0.999 ? '' : 'none';
    let txt, fill;
    if (st.factor === 0) { txt = '初始贴合非工作面，不换面'; fill = 'var(--muted)'; }
    else if (theta < st.startDeg) { txt = '贴合工作面（待换向）'; fill = 'var(--ok)'; }
    else if (theta < st.switchDeg) { txt = `齿面离开·越隙中 ${(p * 100).toFixed(0)}%`; fill = 'var(--hl)'; }
    else { txt = '已换面，带动后级'; fill = 'var(--accent)'; }
    r.stateTxt.textContent = txt;
    r.stateTxt.setAttribute('fill', fill);
  });
  document.querySelectorAll('#bl-stages li').forEach((li, i) => {
    const st = stages[i];
    if (!st) return;
    li.classList.toggle('cur', theta >= st.startDeg && theta < st.switchDeg && st.factor !== 0);
    li.classList.toggle('done', theta >= st.switchDeg);
  });
  const ro = $('#bl-angle-readout');
  if (ro) ro.textContent = `换向角 ${theta.toFixed(1)}°（输入轴）`;
}

function blSetAngle(a) {
  blAngle = a;
  const sl = $('#bl-angle');
  if (sl && Math.abs(parseFloat(sl.value) - a) > 0.05) sl.value = String(a);
  blUpdateMotion();
}

function blPlayFrame(ts) {
  if (!blPlaying) return;
  if (!blPlayT0) blPlayT0 = ts;
  const maxA = blMaxAngle();
  const dur = 3600, hold = 800, cycle = dur + hold + 500;
  const t = (ts - blPlayT0) % cycle;
  blSetAngle(t < dur ? maxA * t / dur : t < dur + hold ? maxA : 0);
  requestAnimationFrame(blPlayFrame);
}

/* ---------------- 搜索 ---------------- */
async function blSearch() {
  if (!blDraft) return;
  const btn = $('#btn-bl-search');
  btn.disabled = true; btn.textContent = '搜索中…';
  $('#bls-meta').textContent = '正在枚举间隙组合';
  const grades = [...document.querySelectorAll('#bls-grades input:checked')]
    .map(inp => inp.value);
  try {
    const resp = await fetch('/api/backlash/search', {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        state: blDraft.snapshot, spec: blDraft.spec,
        daMax: parseFloat($('#bls-da').value) || 0.3,
        daSteps: parseInt($('#bls-dasteps').value, 10) || 3,
        xMin: parseFloat($('#bls-xmin').value) || 0,
        xMax: parseFloat($('#bls-xmax').value) || 0,
        xStep: parseFloat($('#bls-xstep').value) || 0.1,
        grades,
      }),
    });
    const data = await resp.json();
    blSearchRes = data.results || [];
    blSelSol = -1; blSolSpec = null; blSolResult = null;
    $('#bls-meta').textContent =
      (data.note ? data.note + ' ' : '') +
      (data.totalMatched != null
        ? `命中 ${data.totalMatched} 组，显示前 ${blSearchRes.length}；枚举节点 ${data.nodes}${data.truncated ? '（达到上限已截断）' : ''}`
        : '');
    blRenderSearchResults();
    blDrawCompare();
  } finally {
    btn.disabled = false; btn.textContent = '搜索组合';
  }
}

function blTrialSpec(cand) {
  const spec = JSON.parse(JSON.stringify(blDraft.spec));
  for (const [mid, o] of Object.entries(cand.meshes || {})) {
    if (!spec.meshes[mid]) spec.meshes[mid] = {};
    Object.assign(spec.meshes[mid], o);
  }
  return spec;
}

function blRenderSearchResults() {
  const ol = $('#bls-list');
  ol.innerHTML = '';
  const names = (blResult && blResult.meshNames) || {};
  blSearchRes.forEach((c, i) => {
    const li = h('li', { class: 'cand' + (i === blSelSol ? ' selected' : '') }, ol);
    const head = h('div', { class: 'head' }, li);
    h('span', {}, head, `#${i + 1} ` +
      (c.compatible ? `最坏空程 ${c.worst}°` : `区间冲突（缺口 ${c.gap}°）`));
    h('span', { class: 'err', style: c.conflicts ? '' : 'color:var(--ok)' }, head,
      c.conflicts ? `冲突 ${c.conflicts}` : '无冲突');
    h('div', { class: 'meta' }, li,
      `观察轴空程 ${c.lostMin}°~${c.lostMax}°；改动量 ${c.change} mm`);
    const ch = h('div', { class: 'meta' }, li);
    ch.textContent = '调整：' + Object.entries(c.meshes || {}).map(([mid, o]) => {
      const g = BL_GRADE_MAP[o.grade];
      return `${names[mid] || mid} ${g ? g.name.split(' ')[0] : o.grade}` +
        `${o.dA ? ` Δa${o.dA > 0 ? '+' : ''}${o.dA}` : ''}` +
        `${o.xSum ? ` xΣ${o.xSum > 0 ? '+' : ''}${o.xSum}` : ''}`;
    }).join('；');
    const acts = h('div', { class: 'actions' }, li);
    h('button', { onclick: e => { e.stopPropagation(); blPreviewSol(i); } },
      acts, i === blSelSol ? '✓ 叠加中' : '叠加基线');
    h('button', { class: 'primary', onclick: e => { e.stopPropagation(); blSaveSolution(i); } },
      acts, '另存版本');
  });
}

async function blPreviewSol(i) {
  const c = blSearchRes[i];
  if (!c || !blDraft) return;
  if (blSelSol === i) {
    blSelSol = -1; blSolSpec = null; blSolResult = null;
    blRenderSearchResults(); blDrawCompare();
    return;
  }
  blSelSol = i;
  blSolSpec = blTrialSpec(c);
  try {
    const resp = await fetch('/api/backlash/analyze', {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ state: blDraft.snapshot, spec: blSolSpec }),
    });
    blSolResult = await resp.json();
  } catch (e) { blSolResult = null; }
  blRenderSearchResults();
  blDrawCompare();
  flashHint('已叠加搜索方案与基线对照（未写入轮系方案）');
}

async function blSaveSolution(i) {
  const c = blSearchRes[i];
  if (!c || !blDraft) return;
  const spec = blSelSol === i && blSolSpec ? blSolSpec : blTrialSpec(c);
  await blSaveVersion(spec, c,
    `搜索方案：最坏空程 ${c.compatible ? c.worst + '°' : '冲突'}，改动量 ${c.change} mm`);
}

/* ---------------- 对照图 ---------------- */
function blDrawCompare() {
  const svg = $('#bl-compare');
  svg.innerHTML = '';
  const totals = [];
  if (blResult && blResult.compatible && blResult.lostMax != null)
    totals.push({ label: '草稿', lo: blResult.lostMin, hi: blResult.lostMax, color: 'var(--accent)' });
  if (blSolResult && blSolResult.compatible && blSolResult.lostMax != null)
    totals.push({ label: '选中方案', lo: blSolResult.lostMin, hi: blSolResult.lostMax, color: 'var(--hl)' });
  if (blCmpResult && blCmpResult.compatible && blCmpResult.lostMax != null)
    totals.push({ label: blCmpLabel, lo: blCmpResult.lostMin, hi: blCmpResult.lostMax, color: 'var(--ok)' });
  const stageRows = [];
  if (blSolResult && blResult) {
    const solMap = Object.fromEntries((blSolResult.stages || []).map(s => [s.meshId, s]));
    for (const st of (blResult.stages || [])) {
      const so = solMap[st.meshId];
      stageRows.push({ label: `${st.driver}→${st.driven}`,
        a: st.phiObsMax || 0, b: so ? (so.phiObsMax || 0) : null });
    }
  }
  if (!totals.length && !stageRows.length) { svg.style.display = 'none'; return; }
  svg.style.display = '';
  const x0 = 76, x1 = 352;
  let y = 14;
  const maxTot = Math.max(0.001, ...totals.map(t => t.hi));
  svgEl('text', { x: 4, y: y, class: 'bl-svg-label', text: '观察轴空程区间对照' }, svg);
  y += 6;
  for (const t of totals) {
    y += 15;
    const X = v => x0 + (x1 - x0) * v / (maxTot * 1.08);
    svgEl('text', { x: 4, y: y + 3, class: 'bl-tick-label', text: t.label }, svg)
      .setAttribute('text-anchor', 'start');
    svgEl('rect', { x: X(t.lo), y: y - 3, width: Math.max(1.5, X(t.hi) - X(t.lo)),
      height: 7, fill: t.color, opacity: 0.75, rx: 2 }, svg);
    svgEl('text', { x: X(t.hi) + 3, y: y + 3, class: 'bl-tick-label',
      text: `${t.lo}~${t.hi}°` }, svg);
  }
  if (stageRows.length) {
    y += 16;
    svgEl('text', { x: 4, y, class: 'bl-svg-label',
      text: '每级贡献（观察轴°，蓝=草稿 / 橙=选中方案）' }, svg);
    const maxSt = Math.max(0.001, ...stageRows.flatMap(r => [r.a, r.b || 0]));
    const XS = v => x0 + (x1 - x0) * v / (maxSt * 1.08);
    for (const r of stageRows) {
      y += 15;
      svgEl('text', { x: 4, y: y + 3, class: 'bl-tick-label', text: r.label }, svg)
        .setAttribute('text-anchor', 'start');
      svgEl('rect', { x: x0, y: y - 4, width: Math.max(1, XS(r.a) - x0), height: 4.5,
        fill: 'var(--accent)', opacity: 0.8 }, svg);
      if (r.b != null)
        svgEl('rect', { x: x0, y: y + 1, width: Math.max(1, XS(r.b) - x0), height: 4.5,
          fill: 'var(--hl)', opacity: 0.85 }, svg);
    }
  }
  svg.setAttribute('viewBox', `0 0 360 ${y + 8}`);
}

/* ---------------- 版本 ---------------- */
async function blLoadVersions() {
  try {
    blVersions = await (await fetch('/api/backlash/cases')).json();
  } catch (e) { blVersions = []; }
  const sel = $('#bl-versions');
  const cur = sel.value;
  sel.innerHTML = '';
  if (!blVersions.length) h('option', { value: '' }, sel, '— 尚无已存版本 —');
  for (const v of blVersions) {
    const o = h('option', { value: v.id }, sel,
      `${v.name} · v${v.version}${v.note ? `（${v.note}）` : ''}`);
    if (String(v.id) === cur) o.selected = true;
  }
  const cmp = $('#bl-cmp-version');
  const cmpCur = cmp.value;
  cmp.innerHTML = '';
  h('option', { value: '' }, cmp, '— 不对照 —');
  for (const v of blVersions) {
    const o = h('option', { value: v.id }, cmp, `${v.name} · v${v.version}`);
    if (String(v.id) === cmpCur) o.selected = true;
  }
}

async function blSaveVersion(spec, solution, note) {
  if (!blDraft) return;
  const resp = await fetch('/api/backlash/cases', {
    method: 'POST', headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({
      name: blDraft.name || '换向工况',
      spec, snapshot: blDraft.snapshot,
      solution: solution || null, note: note || null,
    }),
  });
  const data = await resp.json();
  if (data.ok) {
    flashHint(`已另存工况版本 v${data.version}（原轮系方案未改动）`);
    blLoadVersions();
  }
}

/* ---------------- 事件绑定与启动 ---------------- */
function blBind() {
  $('#btn-bl-new').addEventListener('click', blBuildDraft);
  $('#btn-bl-refresh').addEventListener('click', blRefreshSnapshot);
  $('#bl-name').addEventListener('input', e => {
    if (blDraft) { blDraft.name = e.target.value; blSaveDraft(); }
  });
  $('#bl-input').addEventListener('change', e => {
    blDraft.spec.inputId = e.target.value || null; blSpecChanged();
  });
  $('#bl-observe').addEventListener('change', e => {
    blDraft.spec.observeId = e.target.value || null; blSpecChanged();
  });
  $('#bl-reversal').addEventListener('input', e => {
    blDraft.spec.reversalDeg = Math.max(1, parseFloat(e.target.value) || 30);
    blSpecChanged();
  });
  $('#bl-angle').addEventListener('input', e => {
    if (blPlaying) { blPlaying = false; $('#btn-bl-play').classList.remove('active'); }
    blSetAngle(parseFloat(e.target.value) || 0);
  });
  $('#btn-bl-play').addEventListener('click', () => {
    blPlaying = !blPlaying;
    $('#btn-bl-play').classList.toggle('active', blPlaying);
    if (blPlaying) { blPlayT0 = 0; requestAnimationFrame(blPlayFrame); }
  });
  $('#btn-bl-search').addEventListener('click', blSearch);
  $('#btn-bl-save').addEventListener('click', () => {
    if (!blDraft) return;
    blSaveVersion(blDraft.spec, blSelSol >= 0 ? blSearchRes[blSelSol] : null, null);
  });
  $('#btn-bl-load').addEventListener('click', async () => {
    const id = $('#bl-versions').value;
    if (!id) return;
    const v = await (await fetch(`/api/backlash/cases/${id}`)).json();
    if (!v.spec) return;
    blDraft = { name: v.name, spec: v.spec, snapshot: v.snapshot };
    blResetResult();
    blSaveDraft(); blAnalyzeSoon(); blRender();
    flashHint(`已载入工况「${v.name}」v${v.version}`);
  });
  $('#btn-bl-del').addEventListener('click', async () => {
    const id = $('#bl-versions').value;
    if (!id) return;
    await fetch(`/api/backlash/cases/${id}`, { method: 'DELETE' });
    blLoadVersions();
  });
  $('#bl-cmp-version').addEventListener('change', async e => {
    const id = e.target.value;
    blCmpResult = null;
    if (id) {
      const v = await (await fetch(`/api/backlash/cases/${id}`)).json();
      if (v.spec) {
        try {
          const resp = await fetch('/api/backlash/analyze', {
            method: 'POST', headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ state: v.snapshot, spec: v.spec }),
          });
          blCmpResult = await resp.json();
          blCmpLabel = `${v.name} v${v.version}`;
        } catch (err) { blCmpResult = null; }
      }
    }
    blDrawCompare();
  });
  /* 间隙等级勾选 */
  const gw = $('#bls-grades');
  for (const g of BL_GRADES) {
    const lab = h('label', {}, gw);
    const cb = h('input', { type: 'checkbox', value: g.id }, lab);
    if (g.id !== 'g0') cb.checked = true;
    lab.appendChild(document.createTextNode(g.name));
  }
}

(function blInit() {
  try {
    const raw = localStorage.getItem(BL_KEY);
    if (raw) blDraft = JSON.parse(raw);
  } catch (e) { blDraft = null; }
  blBind();
  blRender();
  if (blDraft) blAnalyze();
  blLoadVersions();
})();
