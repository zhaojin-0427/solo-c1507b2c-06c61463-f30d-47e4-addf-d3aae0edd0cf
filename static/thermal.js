/* 热平衡工况工作区 — 原生 JS/SVG。
   草稿 = {name, spec, frozen}：frozen 从载荷工况版本（或当前载荷草稿）冻结
   各级啮合损失、转速、轴承载荷与持续时间；spec 填写环境、热容量、热阻、油品、
   散热面积与风扇阈值。按时间步的温度/热流/效率曲线由后端 thermal.py 计算。
   来源载荷或源轮系变化时只标记过期，不自动重算。 */
'use strict';

const TH_KEY = 'thermalDraftV1';
const TH_DEFAULTS = {
  ambient: 25, initTemp: 30, dt: 30,
  oil: { grade: 'ISO VG 220', rho: 870, volumeL: 3, cp: 1900,
         minVisc: 10, maxVisc: 500, sens: 0.10 },
  housing: { capacity: 30, area: 0.8, rNat: 15, rFan: 3 },
  fan: { mode: 'auto', onTemp: 55, offTemp: 45, powerKw: 0.25, minCycleMin: 3 },
  limits: { mesh: 90, bearing: 85, oil: 85 },
  oilNode: { rHousing: 0.08 },
};
const TH_NODE_COLORS = {
  oil: '#b06a00', housing: '#6d5a40', mesh: '#3d5a80', bearing: '#5b8def',
};

let thDraft = null;        // {name, spec, frozen}
let thResult = null;
let thToken = 0;
let thTimer = null;
let thVersions = [];
let thOils = [];
let thCursor = -1;         // 时间游标样本下标
let thFanOverride = '';    // '' 按方案 / 'on' / 'off'
let thSelectedNode = null; // 联动查看热流的节点
let thStructKey = '';
let thSearchRes = null;

/* ---------------- 指纹（与 thermal.fingerprint 逐字节一致） ---------------- */
/* 规范数值：6 位小数取整，整数去掉小数点；与后端 _norm_num 同口径 */
function thNormNum(v, dflt) {
  let f = Number(v);
  if (!Number.isFinite(f)) f = Number.isFinite(+dflt) ? +dflt : 0;
  f = Math.round(f * 1e6) / 1e6;
  if (Object.is(f, -0)) f = 0;
  return Number.isInteger(f) ? f : f;
}

/* 紧凑、递归按 key 排序的 JSON；与后端 canonical_dumps 输出一致 */
function thCanonicalStringify(obj) {
  if (obj === null || typeof obj !== 'object') return JSON.stringify(obj);
  if (Array.isArray(obj)) return '[' + obj.map(thCanonicalStringify).join(',') + ']';
  const keys = Object.keys(obj).sort();
  return '{' + keys.map(k => JSON.stringify(k) + ':' + thCanonicalStringify(obj[k])).join(',') + '}';
}

function thFingerprint(st) {
  const S = Object.fromEntries((st.shafts || []).map(s => [s.id, s]));
  const G = Object.fromEntries((st.gears || []).map(g => [g.id, g]));
  const items = [];
  for (const e of (st.meshes || [])) {
    const ga = G[e.gearA], gb = G[e.gearB];
    if (!ga || !gb) { items.push([String(e.id), 'broken']); continue; }
    const sa = S[ga.shaftId] || {}, sb = S[gb.shaftId] || {};
    items.push([
      String(e.id),
      thNormNum(ga.z), thNormNum(ga.module), thNormNum(ga.x || 0),
      thNormNum(ga.pressureAngle || 20), !!ga.internal,
      thNormNum(gb.z), thNormNum(gb.module), thNormNum(gb.x || 0),
      thNormNum(gb.pressureAngle || 20), !!gb.internal,
      thNormNum(sa.x || 0), thNormNum(sa.y || 0),
      thNormNum(sb.x || 0), thNormNum(sb.y || 0)]);
  }
  items.sort((a, b) => a[0] < b[0] ? -1 : a[0] > b[0] ? 1 : 0);
  return thCanonicalStringify(items);
}

function thSourceStale() {
  if (!thDraft) return false;
  for (const src of Object.values(thDraft.frozen.sources || {})) {
    if (src.error) continue;
    if (src.inline) {
      // 内联来源：源轮系快照与载荷参数都与当前载荷草稿一致才不过期
      if (typeof lcDraft === 'undefined' || !lcDraft) return true;
      if (src.fingerprint !== thFingerprint(lcDraft.snapshot)) return true;
      if (src.loadSpec && src.loadSpec !== thCanonicalStringify(lcDraft.spec)) return true;
    } else {
      // 已保存载荷版本不可变：仅当当前项目轮系与该版本快照不一致时提示过期
      if (src.fingerprint !== thFingerprint(state)) return true;
    }
  }
  return false;
}

/* ---------------- 草稿建立 / 冻结 ---------------- */
async function thLoadOils() {
  if (thOils.length) return;
  try { thOils = (await (await fetch('/api/thermal/oils')).json()).oils || []; }
  catch (e) { thOils = []; }
}

async function thBuildSources(inlineCurrent) {
  // 来源 = 已保存载荷版本；勾选当前载荷草稿时追加一个内联来源
  const caseRows = await (await fetch('/api/loadcase/cases')).json();
  const sources = [];
  const seenName = new Set();
  for (const row of caseRows) {
    const v = await (await fetch(`/api/loadcase/cases/${row.id}`)).json();
    if (!v.spec || !v.snapshot) continue;
    const key = 'lc' + row.id + 'v' + row.version;
    sources.push({ key, name: `${row.name} v${row.version}`,
      state: v.snapshot, spec: v.spec, caseId: row.id, version: row.version });
    seenName.add(row.name);
  }
  if (inlineCurrent && typeof lcDraft !== 'undefined' && lcDraft && lcDraft.spec) {
    sources.push({ key: 'inline', name: '当前载荷草稿（未另存）',
      state: lcDraft.snapshot, spec: lcDraft.spec, inline: true });
  }
  return sources;
}

async function thNewDraft() {
  // 来源 = 已保存载荷版本；勾选「内联当前载荷草稿」时把未另存草稿也作为一个来源
  const caseRows = await (await fetch('/api/loadcase/cases')).json();
  const inlineCurrent = $('#th-inline').checked;
  if (!caseRows.length && (!inlineCurrent || typeof lcDraft === 'undefined' || !lcDraft)) {
    alert('还没有已保存的载荷版本。请先在「载荷工况」页保存版本，或勾选「同时内联当前载荷草稿」。');
    return;
  }
  const sources = await thBuildSources(inlineCurrent);
  if (!sources.length) {
    alert('没有可用的载荷来源。');
    return;
  }
  await thFreezeAndInit(sources, true);
}

async function thRefreeze() {
  if (!thDraft) return;
  const oldKeys = thDraft.spec.segments
    .filter(g => g.kind === 'run').map(g => g.sourceId);
  const caseRows = await (await fetch('/api/loadcase/cases')).json();
  const sources = [];
  for (const srcIn of thDraft.frozen.order.map(k => thDraft.frozen.sources[k])) {
    if (srcIn.inline && typeof lcDraft !== 'undefined' && lcDraft) {
      sources.push({ key: srcIn.key, name: srcIn.name, state: lcDraft.snapshot,
        spec: lcDraft.spec, inline: true });
    } else if (srcIn.caseId) {
      const v = await (await fetch(`/api/loadcase/cases/${srcIn.caseId}`)).json();
      if (v.spec) sources.push({ key: srcIn.key, name: srcIn.name,
        state: v.snapshot, spec: v.spec, caseId: v.id, version: v.version });
    }
  }
  // 新出现的已存版本不自动加入（避免悄悄改变热输入），只刷新已有来源
  if (!sources.length) { alert('原有冻结来源都已不存在，请重新建立草稿。'); return; }
  const resp = await fetch('/api/thermal/freeze', {
    method: 'POST', headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ sources }),
  });
  thDraft.frozen = await resp.json();
  thDraft.spec.segments = thDraft.spec.segments.filter(
    g => g.kind !== 'run' || thDraft.frozen.sources[g.sourceId]);
  thReset();
  thSaveDraft(); thAnalyzeSoon(); thRender();
  flashHint('已重新冻结来源（各级损失/转速/轴承载荷已更新）');
}

async function thFreezeAndInit(sources, defaultSeg) {
  const resp = await fetch('/api/thermal/freeze', {
    method: 'POST', headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ sources }),
  });
  const frozen = await resp.json();
  const sp = thDefaultSpec(frozen);
  if (defaultSeg && frozen.order.length) {
    const first = frozen.sources[frozen.order[0]];
    sp.segments = [{ id: 'seg1', name: '连续运行', kind: 'run',
      sourceId: first.key, loadScale: 1, speedScale: 1,
      durationH: first.durationH || 1, ambient: null }];
  }
  // 节点参数默认值（热容量/热阻/搅油/轴承模型）
  sp.meshes = {}; sp.bearings = {};
  for (const [nid, d] of Object.entries(frozen.defaults || {})) {
    if (d.kind === 'mesh') sp.meshes[nid] = { capacity: d.capacity, rOil: d.rOil, churn: d.churn };
    else sp.bearings[nid] = { capacity: d.capacity, rOil: d.rOil, f0: d.f0, f1: d.f1, dm: d.dm };
  }
  thDraft = { name: (thDraft && thDraft.name) || '热平衡工况', spec: sp, frozen };
  thReset();
  thSaveDraft(); thAnalyzeSoon(); thRender();
  flashHint('已从载荷工况冻结热平衡草稿');
}

function thDefaultSpec() {
  return JSON.parse(JSON.stringify({
    ambient: TH_DEFAULTS.ambient, initTemp: TH_DEFAULTS.initTemp, dt: TH_DEFAULTS.dt,
    oil: { ...TH_DEFAULTS.oil }, housing: { ...TH_DEFAULTS.housing },
    fan: { ...TH_DEFAULTS.fan }, limits: { ...TH_DEFAULTS.limits },
    oilNode: { ...TH_DEFAULTS.oilNode }, segments: [],
    meshes: {}, bearings: {},
  }));
}

function thReset() {
  thResult = null; thSearchRes = null; thCursor = -1; thStructKey = '';
  thSelectedNode = null; thFanOverride = '';
  $('#th-search-oils').dataset.built = '';
  $('#th-visc-points').dataset.n = '';   // 强制按新草稿的折点重建行
  document.querySelectorAll('input[name="th-fan-override"]').forEach(r => { r.checked = false; });
}
let thDraftTimer = null;   // 服务端草稿防抖
let thDraftLoading = false; // 正在从服务端载入，抑制回写
function thSaveDraft() {
  if (!thDraft || thDraftLoading) return;
  // 本地立即保存（离线/刷新当前页即时恢复）
  try { localStorage.setItem(TH_KEY, JSON.stringify(thDraft)); } catch (e) { /* ignore */ }
  // 服务端防抖保存（换浏览器/清站点数据后可恢复）
  clearTimeout(thDraftTimer);
  thDraftTimer = setTimeout(thPushDraft, 800);
}
async function thPushDraft() {
  if (!thDraft) return;
  try {
    await fetch('/api/thermal/draft', {
      method: 'PUT', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ draft: { name: thDraft.name,
        spec: thDraft.spec, frozen: thDraft.frozen } }),
    });
  } catch (e) { /* 网络失败时本地副本仍在 */ }
}
function thSpecChanged() { thSearchRes = null; thSaveDraft(); thAnalyzeSoon(); }

/* ---------------- 后端计算 ---------------- */
function thAnalyzeSoon() {
  clearTimeout(thTimer);
  thTimer = setTimeout(thAnalyze, 180);
}
async function thAnalyze() {
  if (!thDraft) return;
  const token = ++thToken;
  try {
    const resp = await fetch('/api/thermal/analyze', {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ spec: thDraft.spec, frozen: thDraft.frozen }),
    });
    const data = await resp.json();
    if (token !== thToken) return;
    thResult = data;
    if (thCursor < 0 || thCursor >= (data.times || []).length)
      thCursor = (data.times || []).length - 1;
  } catch (e) {
    if (token === thToken) thResult = { ok: false, issues: [
      { severity: 'error', code: 'FETCH', message: '计算失败：' + e, refs: {} }] };
  }
  thRenderChartOnly();
  thRenderResults();
}

/* ---------------- 渲染总入口 ---------------- */
function thRender() {
  const has = !!thDraft;
  $('#th-body').style.display = has ? '' : 'none';
  $('#th-empty').style.display = has ? 'none' : '';
  if (!has) return;
  const nameInp = $('#th-name');
  if (document.activeElement !== nameInp) nameInp.value = thDraft.name || '';
  $('#th-stale').style.display = thSourceStale() ? '' : 'none';
  thFillScalarInputs();
  thRenderSources();
  thRenderSegments();
  thRenderNodeParams();
  thRenderViscPoints();
  thRenderViscCurve();
  thRenderSearchOils();
  thRenderResults();
}

function thSchedule() {
  if (thDraft) $('#th-stale').style.display = thSourceStale() ? '' : 'none';
}
function thTabShown() { thLoadVersions(); thLoadOils().then(thRender); thRender(); }

function thFillScalarInputs() {
  const sp = thDraft.spec;
  const set = (sel, v) => {
    const inp = $(sel);
    if (document.activeElement !== inp) inp.value = v ?? '';
  };
  set('#th-ambient', sp.ambient); set('#th-init', sp.initTemp); set('#th-dt', sp.dt);
  // 油品下拉
  const sel = $('#th-oil-grade');
  if (sel.dataset.built !== '1') {
    sel.dataset.built = '1'; sel.innerHTML = '';
    for (const o of thOils.length ? thOils : []) h('option', { value: o.grade }, sel, o.grade);
    h('option', { value: '自定义' }, sel, '自定义（沿用已存折点）');
  }
  if (document.activeElement !== sel) sel.value = (sp.oil.points && sp.oil.grade) || sp.oil.grade;
  set('#th-oil-vol', sp.oil.volumeL); set('#th-oil-rho', sp.oil.rho);
  set('#th-oil-cp', sp.oil.cp);
  set('#th-visc-min', sp.oil.minVisc); set('#th-visc-max', sp.oil.maxVisc);
  set('#th-oil-sens', sp.oil.sens);
  set('#th-h-cap', sp.housing.capacity); set('#th-h-area', sp.housing.area);
  set('#th-rnat', sp.housing.rNat); set('#th-rfan', sp.housing.rFan);
  set('#th-roh', sp.oilNode.rHousing);
  const fm = $('#th-fan-mode');
  if (document.activeElement !== fm) fm.value = sp.fan.mode;
  set('#th-fan-on', sp.fan.onTemp); set('#th-fan-off', sp.fan.offTemp);
  set('#th-fan-p', sp.fan.powerKw); set('#th-fan-min', sp.fan.minCycleMin);
  set('#th-lim-mesh', sp.limits.mesh); set('#th-lim-bearing', sp.limits.bearing);
  set('#th-lim-oil', sp.limits.oil);
}

/* ---------------- 来源与时段 ---------------- */
function thRenderSources() {
  const wrap = $('#th-sources');
  wrap.innerHTML = '';
  const order = thDraft.frozen.order || [];
  if (!order.length) { h('div', { class: 'muted small' }, wrap, '无冻结来源'); return; }
  for (const key of order) {
    const src = thDraft.frozen.sources[key];
    const used = thDraft.spec.segments.some(g => g.kind === 'run' && g.sourceId === key);
    const card = h('div', { class: 'th-source-card' + (used ? '' : ' unused') }, wrap);
    const head = h('div', { class: 'bl-mesh-head' }, card);
    h('span', {}, head, src.name + (src.inline ? '（草稿）' : ''));
    h('span', { class: 'muted small' }, head,
      `输入 ${src.pInputKw ?? '?'} kW · 损失 ${src.lossKw ?? '?'} kW · ${Object.keys(src.meshes || {}).length} 级` +
      (src.error ? ' · 计算失败' : ''));
  }
}

function thRenderSegments() {
  const wrap = $('#th-segments');
  wrap.innerHTML = '';
  const sp = thDraft.spec;
  sp.segments.forEach((g, i) => {
    const card = h('div', { class: 'th-seg-card ' + g.kind, 'data-i': i }, wrap);
    const head = h('div', { class: 'bl-mesh-head' }, card);
    const nameInp = h('input', { class: 'th-seg-name', value: g.name }, head);
    nameInp.addEventListener('input', () => { g.name = nameInp.value; thSaveDraft(); thRenderResults(); });
    h('span', { class: 'th-seg-kind' }, head, g.kind === 'stop' ? '停机' : '运行');
    h('button', { class: 'danger', onclick: () => {
      sp.segments.splice(i, 1); thSpecChanged(); thRender(); } }, head, '删除');
    const grid = h('div', { class: 'bl-mesh-grid' }, card);
    if (g.kind === 'run') {
      const labS = h('label', {}, grid, '冻结来源');
      const selS = h('select', {}, labS);
      for (const key of (thDraft.frozen.order || [])) {
        const attrs = { value: key };
        if (key === g.sourceId) attrs.selected = 'selected';
        h('option', attrs, selS, thDraft.frozen.sources[key].name);
      }
      selS.addEventListener('change', () => { g.sourceId = selS.value; thSpecChanged(); thRender(); });
    }
    const mkNum = (label, val, step, onch) => {
      const lab = h('label', {}, grid, label);
      const inp = h('input', { type: 'number', step: String(step), value: val }, lab);
      inp.addEventListener('input', () => { onch(parseFloat(inp.value)); thSpecChanged(); });
    };
    mkNum('持续 h', g.durationH, 0.5, v => { g.durationH = Math.max(0, v || 0); });
    if (g.kind === 'run') {
      mkNum('载荷比例', g.loadScale ?? 1, 0.1, v => { g.loadScale = Math.max(0, v || 0); });
      mkNum('转速比例', g.speedScale ?? 1, 0.1, v => { g.speedScale = Math.max(0, v || 0); });
    }
    mkNum('段环境温 °C（空=默认）', g.ambient == null ? '' : g.ambient, 1,
      v => { g.ambient = Number.isFinite(v) ? v : null; });
  });
}

/* ---------------- 节点热参数 ---------------- */
function thRenderNodeParams() {
  const key = JSON.stringify(Object.keys(thDraft.frozen.defaults || {}).sort());
  const wrap = $('#th-nodes');
  if (key === thStructKey) return;
  thStructKey = key;
  wrap.innerHTML = '';
  const groups = [['mesh', '齿轮啮合节点'], ['bearing', '轴承节点']];
  for (const [kind, title] of groups) {
    const ids = Object.entries(thDraft.frozen.defaults || {})
      .filter(([, d]) => d.kind === kind).map(([id]) => id);
    if (!ids.length) continue;
    h('div', { class: 'th-node-group-title muted small' }, wrap, title);
    for (const id of ids) {
      const d = thDraft.frozen.defaults[id];
      const store = kind === 'mesh'
        ? (thDraft.spec.meshes[id] || (thDraft.spec.meshes[id] = {
            capacity: d.capacity, rOil: d.rOil, churn: d.churn }))
        : (thDraft.spec.bearings[id] || (thDraft.spec.bearings[id] = {
            capacity: d.capacity, rOil: d.rOil, f0: d.f0, f1: d.f1, dm: d.dm }));
      const card = h('div', { class: 'th-node-card' }, wrap);
      h('span', { class: 'th-node-name' }, card, d.name);
      const mk = (label, val, step, onch) => {
        const lab = h('label', {}, card, label);
        const inp = h('input', { type: 'number', step: String(step), value: val }, lab);
        inp.addEventListener('input', () => { onch(parseFloat(inp.value)); thSpecChanged(); });
      };
      mk('C kJ/K', store.capacity, 0.02, v => { store.capacity = Math.max(1e-6, v || 0); });
      mk('R→油 K/kW', store.rOil, 0.02, v => { store.rOil = Math.max(1e-6, v || 0); });
      if (kind === 'mesh') {
        mk('搅油系数 kW', store.churn, 0.005, v => { store.churn = Math.max(0, v || 0); });
      } else {
        mk('节圆 dm mm', store.dm, 1, v => { store.dm = Math.max(1, v || 1); });
        mk('f0', store.f0, 0.1, v => { store.f0 = v; });
        mk('f1', store.f1, 0.0001, v => { store.f1 = Math.max(0, v || 0); });
      }
    }
  }
}

/* 用一组新折点原地替换草稿内容并标记折点结构版本，
   保证折点行随后按新值重建（牌号切换/采用候选/载入版本时用）。 */
function thSetPoints(newPoints) {
  const pts = thDraft.spec.oil.points;
  const valid = (newPoints || [])
    .filter(p => Number.isFinite(+p[0]) && Number.isFinite(+p[1]) && +p[1] > 0)
    .map(p => [+p[0], +p[1]])
    .sort((a, b) => a[0] - b[0]);
  pts.length = valid.length;
  valid.forEach((p, i) => { pts[i] = p; });
  const wrap = $('#th-visc-points');
  wrap.dataset.n = '';   // 长度/引用变化后强制重建折点行
  return pts;
}

/* ---------------- 黏温折点录入 ---------------- */
/* 折点始终规范化为 thDraft.spec.oil.points（温度升序、黏度为正）。
   编辑时只原地改值并原地排序，绝不替换数组引用——行内 input 闭包持有
   该数组，替换会导致第二次输入写到游离的旧数组、草稿得不到更新。 */
function thValidPoints() {
  return (thDraft.spec.oil.points || [])
    .filter(p => Number.isFinite(+p[0]) && Number.isFinite(+p[1]) && +p[1] > 0)
    .map(p => [+p[0], +p[1]])
    .sort((a, b) => a[0] - b[0]);
}

/* 原地把草稿中的折点同步为 valid（同长度改值、排序；增删才改变长度），
   返回有效折点；不替换数组引用。 */
function thSyncPoints(valid) {
  const pts = thDraft.spec.oil.points;
  pts.length = valid.length;
  valid.forEach((p, i) => { pts[i] = p; });
  return pts;
}

function thRenderViscPoints() {
  const wrap = $('#th-visc-points');
  const pts = thDraft.spec.oil.points || [];
  // 仅在折点数量（结构）变化时重建行；普通键入只回显、不抢焦点
  if (wrap.dataset.n === String(pts.length)) {
    wrap.querySelectorAll('.th-point-row').forEach((row, i) => {
      const p = pts[i] || [null, null];
      const a = row.querySelector('input[data-t]'), b = row.querySelector('input[data-v]');
      if (document.activeElement !== a && a.value !== String(p[0] ?? '')) a.value = p[0] ?? '';
      if (document.activeElement !== b && b.value !== String(p[1] ?? '')) b.value = p[1] ?? '';
    });
    return;
  }
  wrap.dataset.n = String(pts.length);
  wrap.innerHTML = '';
  pts.forEach((p, i) => {
    const row = h('div', { class: 'th-point-row' }, wrap);
    const mk = (dataKey, val, ph) => {
      const attrs = { type: 'number', step: dataKey === 't' ? '1' : '0.1', placeholder: ph };
      attrs['data-' + dataKey] = '';
      const inp = h('input', attrs, row);
      inp.value = val ?? '';
      inp.addEventListener('input', () => {
        const v = parseFloat(inp.value);
        const idx = dataKey === 't' ? 0 : 1;
        // 只在合法时原地写回同一个 points 数组（闭包持有的稳定引用）
        if (Number.isFinite(v) && (idx === 0 || v > 0)) pts[i][idx] = v;
        const valid = thValidPoints();
        if (valid.length >= 2) {
          thSyncPoints(valid);   // 原地排序，引用不变
          // 排序可能改变本行位置：值回显到正确的行（焦点仍在当前 input）
          wrap.querySelectorAll('.th-point-row').forEach((r2, j) => {
            const ta = r2.querySelector('input[data-t]'), va = r2.querySelector('input[data-v]');
            if (document.activeElement !== ta) ta.value = valid[j][0];
            if (document.activeElement !== va) va.value = valid[j][1];
          });
        }
        thOnOilPointsEdited();
      });
      return inp;
    };
    mk('t', p[0], '°C');
    h('span', { class: 'th-point-sep' }, row, '—');
    mk('v', p[1], 'mm²/s');
    const del = h('button', { type: 'button', class: 'danger th-point-del' }, row, '删');
    del.addEventListener('click', () => {
      pts.splice(i, 1);
      wrap.dataset.n = '';   // 强制按新长度重建
      thOnOilPointsEdited(); thRender();
    });
  });
}

/* 用户改动折点后的统一处理：至少 2 个有效折点才参与计算并标记牌号；
   每次输入都保存草稿（防抖写服务端）并刷新曲线。 */
function thOnOilPointsEdited() {
  const valid = thValidPoints();
  if (valid.length < 2) {
    thSaveDraft();
    thRenderViscCurve();
    return;
  }
  thSyncPoints(valid);
  const matchesPreset = thOils.some(o => o.grade === thDraft.spec.oil.grade &&
    thPointsEqual(o.points, thDraft.spec.oil.points));
  if (!matchesPreset) thDraft.spec.oil.grade = '自定义';
  const sel = $('#th-oil-grade');
  if (sel && sel.value !== thDraft.spec.oil.grade) sel.value = thDraft.spec.oil.grade;
  thSaveDraft();
  thAnalyzeSoon();
  thRenderViscCurve();
}

function thPointsEqual(a, b) {
  if (!a || !b || a.length !== b.length) return false;
  return a.every((p, i) => Math.abs(p[0] - b[i][0]) < 1e-9 && Math.abs(p[1] - b[i][1]) < 1e-9);
}

/* ---------------- 黏温曲线 SVG ---------------- */
function thRenderViscCurve() {
  const svg = $('#th-visc-svg');
  const W = 300, H = 90, ml = 34, mr = 8, mt = 8, mb = 18;
  const pts = (thDraft.spec.oil.points || [])
    .filter(p => Number.isFinite(+p[0]) && Number.isFinite(+p[1]) && +p[1] > 0);
  svg.setAttribute('viewBox', `0 0 ${W} ${H}`);
  svg.innerHTML = '';
  if (pts.length < 2) {
    svgEl('text', { x: W / 2, y: H / 2, class: 'lc-tick',
      text: '至少需要 2 个有效折点' }, svg).setAttribute('text-anchor', 'middle');
    return;
  }
  const temps = [], viscs = [];
  for (let T = 20; T <= 120; T += 2) {
    temps.push(T); viscs.push(thOilVisc(pts, T));
  }
  const tMin = 20, tMax = 120;
  const lMin = Math.log10(Math.min(...viscs, thDraft.spec.oil.minVisc) * 0.8);
  const lMax = Math.log10(Math.max(...viscs, thDraft.spec.oil.maxVisc) * 1.2);
  const X = T => ml + (W - ml - mr) * (T - tMin) / (tMax - tMin);
  const Y = v => mt + (H - mt - mb) * (1 - (Math.log10(v) - lMin) / (lMax - lMin));
  svg.setAttribute('viewBox', `0 0 ${W} ${H}`);
  svg.innerHTML = '';
  // 允许黏度区间
  svgEl('rect', { x: X(tMin), y: Y(thDraft.spec.oil.maxVisc),
    width: X(tMax) - X(tMin),
    height: Y(thDraft.spec.oil.minVisc) - Y(thDraft.spec.oil.maxVisc),
    fill: '#e9f3ea', opacity: 0.7 }, svg);
  const d = temps.map((T, i) => `${i ? 'L' : 'M'}${X(T).toFixed(1)},${Y(viscs[i]).toFixed(1)}`).join('');
  svgEl('path', { d, fill: 'none', stroke: '#b06a00', 'stroke-width': 1.6 }, svg);
  for (const [T, v] of pts)
    svgEl('circle', { cx: X(T), cy: Y(v), r: 2.6, fill: '#b06a00' }, svg);
  // 当前油温点
  if (thResult) {
    const T = thResult.temps.oil[thCursor >= 0 ? thCursor : thResult.temps.oil.length - 1];
    const v = thOilVisc(pts, T);
    if (T >= tMin && T <= tMax)
      svgEl('circle', { cx: X(T), cy: Y(v), r: 3.4, fill: '#c62828' }, svg);
  }
  svgEl('text', { x: 2, y: 12, class: 'lc-tick', text: 'mm²/s' }, svg);
  svgEl('text', { x: ml, y: H - 5, class: 'lc-tick', text: '20°C' }, svg);
  svgEl('text', { x: W - mr, y: H - 5, class: 'lc-tick', text: '120°C' }, svg)
    .setAttribute('text-anchor', 'end');
}

function thOilVisc(points, t) {
  const ps = (points || [])
    .filter(p => Number.isFinite(+p[0]) && Number.isFinite(+p[1]) && +p[1] > 0)
    .map(p => [+p[0], +p[1]])
    .sort((a, b) => a[0] - b[0]);
  if (ps.length < 2) return Number.NaN;
  if (t <= ps[0][0]) return ps[0][1];
  if (t >= ps[ps.length - 1][0]) return ps[ps.length - 1][1];
  for (let i = 0; i < ps.length - 1; i++) {
    const [t0, v0] = ps[i], [t1, v1] = ps[i + 1];
    if (t >= t0 && t <= t1) {
      const f = (t - t0) / (t1 - t0);
      return Math.pow(10, Math.log10(v0) + f * (Math.log10(v1) - Math.log10(v0)));
    }
  }
  return ps[ps.length - 1][1];
}

/* ---------------- 结果：汇总/问题/热网络/时间轴 ---------------- */
function thRenderResults() {
  thRenderSummary();
  thRenderIssues();
  thRenderBalance();
  thRenderChartOnly();
}

function thRenderSummary() {
  const box = $('#th-summary'), badge = $('#th-badge');
  if (!thResult) { box.textContent = '正在计算…'; box.classList.add('muted'); badge.textContent = ''; return; }
  box.classList.remove('muted');
  const errs = (thResult.issues || []).filter(i => i.severity === 'error');
  const sm = thResult.summary || {};
  if (errs.length) {
    badge.textContent = '参数错误'; badge.className = 'badge err';
    box.innerHTML = errs.map(i => `<b>${i.message}</b>`).join('；');
    return;
  }
  const warns = (thResult.issues || []).filter(i => i.severity === 'warning');
  badge.textContent = warns.length ? `${warns.length} 项告警` : '热平衡正常';
  badge.className = 'badge ' + (warns.length ? 'warn' : '');
  box.innerHTML = `总时长 <b>${thResult.totalH} h</b>（内部步 ${thResult.dtInternal} s，` +
    `${thResult.nSamples} 个样本）；峰值 <b>${sm.peakTemp}°C</b>` +
    `（${(thResult.nodeNames || {})[sm.peakNode] || sm.peakNode}）；` +
    `累计超温 <b>${(sm.overTimeS / 60).toFixed(1)} min</b>；` +
    `黏度越界 <b>${(sm.viscOverS / 60).toFixed(1)} min</b>；` +
    `总能耗 <b>${sm.energyKwh} kWh</b>（风扇 ${thResult.fanInfo.energyKwh} kWh，` +
    `运行 ${thResult.fanInfo.onTimeH} h，切换 ${thResult.fanInfo.cycles} 次）；` +
    `末段热平衡 <b style="color:var(--${sm.converged ? 'ok' : 'warn'})">` +
    `${sm.converged ? '已收敛' : '未收敛'}</b>`;
}

function thRenderIssues() {
  const ul = $('#th-issues');
  ul.innerHTML = '';
  if (!thResult) return;
  const sevRank = { error: 0, warning: 1, info: 2 };
  const list = (thResult.issues || []).slice().sort((a, b) => sevRank[a.severity] - sevRank[b.severity]);
  if (!list.length) { h('li', { class: 'info' }, ul, '✓ 全程无超温/黏度越界，热平衡收敛，风扇无频繁启停'); return; }
  for (const it of list) {
    const li = h('li', { class: it.severity }, ul);
    h('span', { class: 'sev' }, li,
      it.severity === 'error' ? '✗ 超温' : '⚠ 时段');
    li.appendChild(document.createTextNode(it.message));
    li.addEventListener('click', () => thLocate(it));
  }
}

function thLocate(it) {
  const refs = it.refs || {};
  // 点击问题条目：时间游标跳到问题开始时刻，节点联动选中
  if (refs.t0 != null && thResult) {
    const k = thTimeToIndex(refs.t0);
    if (k >= 0) { thCursor = k; thRenderChartOnly(); }
  }
  if (refs.node) { thSelectedNode = refs.node; thRenderNetwork(); thRenderChartOnly(); }
}

function thTimeToIndex(t) {
  const ts = thResult.times;
  let lo = 0, hi = ts.length - 1;
  while (lo < hi) {
    const mid = (lo + hi + 1) >> 1;
    if (ts[mid] <= t) lo = mid; else hi = mid - 1;
  }
  return lo;
}

function thRenderBalance() {
  const wrap = $('#th-balance');
  wrap.innerHTML = '';
  if (!thResult || !(thResult.balance || []).length) return;
  for (const b of thResult.balance) {
    const card = h('div', { class: 'th-bal-card' + (b.converged ? '' : ' bad') }, wrap);
    h('span', {}, card, `「${b.name}」`);
    h('span', { class: b.converged ? 'lc-ok' : 'lc-err' }, card,
      b.converged ? '✓ 收敛' : '✗ 未收敛');
    h('span', { class: 'muted small' }, card,
      `末段速率 ${b.maxRateKpH} K/h · 油温 ${b.oilEnd}°C · 箱体 ${b.housingEnd}°C`);
    card.addEventListener('click', () => {
      const band = (thResult.bands || []).find(x => x.id === b.segId);
      if (band) { thCursor = thTimeToIndex(band.t1); thRenderChartOnly(); }
    });
  }
}

/* ---------------- 热网络 SVG（节点 + 当前热流箭头） ---------------- */
function thRenderNetwork() {
  const svg = $('#th-net-svg');
  if (!thResult) { svg.innerHTML = ''; return; }
  const nodes = thResult.nodes || [];
  const gears = nodes.filter(n => n.kind === 'mesh');
  const bearings = nodes.filter(n => n.kind === 'bearing');
  const W = 560;
  const perRow = list => Math.min(4, Math.max(1, list.length));
  const rowsN = Math.ceil(gears.length / perRow(gears)) +
                Math.ceil(bearings.length / perRow(bearings)) + 1;
  const rowH = 48;
  const topY = 40;
  const H = topY + rowsN * rowH + 90;
  svg.setAttribute('viewBox', `0 0 ${W} ${H}`);
  svg.innerHTML = '';

  const pos = {};
  const place = (list, rowBase) => {
    const cols = perRow(list);
    list.forEach((n, i) => {
      const r = Math.floor(i / cols), c = i % cols;
      pos[n.id] = [W * (c + 1) / (cols + 1), topY + (rowBase + r) * rowH];
    });
    return Math.ceil(list.length / cols);
  };
  const gearRows = place(gears, 0);
  const bearRows = place(bearings, gearRows);
  const oilY = topY + (gearRows + bearRows) * rowH + 22;
  const houseY = oilY + 78;
  pos.oil = [W / 2, oilY];
  pos.housing = [W / 2, houseY];

  const k = thCursor >= 0 ? thCursor : (thResult.times.length - 1);
  const T = id => (thResult.temps[id] || [])[k];
  const flow = key => (thResult.flows[key] || [])[k] || 0;

  // 边：叶→油池（热流为正表示流向油池），油→箱，箱→环境
  for (const n of [...gears, ...bearings]) {
    const [x1, y1] = pos[n.id], [x2, y2] = pos.oil;
    const f = Math.abs(flow(n.id + '>oil'));
    thHeatArrow(svg, x1, y1 + 12, x2, y2 - 22, f);
  }
  thHeatArrow(svg, pos.oil[0], pos.oil[1] + 20, pos.housing[0], houseY - 22,
    Math.abs(flow('oil>housing')));
  const fanOn = thFanStateAt(k);
  thHeatArrow(svg, pos.housing[0], houseY + 18, pos.housing[0], H - 18,
    Math.abs(flow('housing>ambient')), fanOn ? '#5b8def' : '#b08968');
  svgEl('text', { x: pos.housing[0] + 6, y: H - 4, class: 'lc-tick',
    text: fanOn ? '风扇开 → 环境' : '自然对流 → 环境', fill: fanOn ? '#5b8def' : '#837a6d' }, svg);

  const drawNode = (id, r) => {
    const [x, y] = pos[id];
    const n = nodes.find(z => z.id === id) || { name: thResult.nodeNames[id], kind: id };
    const t = T(id);
    const over = n.limit != null && t > n.limit;
    const g = svgEl('g', { transform: `translate(${x},${y})`, class: 'th-node',
      cursor: 'pointer' }, svg);
    g.addEventListener('click', () => {
      thSelectedNode = thSelectedNode === id ? null : id;
      thRenderNetwork(); thRenderChartOnly();
    });
    svgEl('circle', { r, fill: over ? '#fdecea' : '#fffdf8',
      stroke: over ? '#c62828' : TH_NODE_COLORS[n.kind] || '#666',
      'stroke-width': thSelectedNode === id ? 2.6 : 1.4 }, g);
    svgEl('text', { y: 2, class: 'th-node-temp', text: t == null ? '' : t.toFixed(1) + '°' }, g)
      .setAttribute('text-anchor', 'middle');
    svgEl('text', { y: r + 11, class: 'lc-tick', text: (n.name || id).slice(0, 12) }, g)
      .setAttribute('text-anchor', 'middle');
  };
  for (const n of gears) drawNode(n.id, 13);
  for (const n of bearings) drawNode(n.id, 11);
  drawNode('oil', 20);
  drawNode('housing', 18);
}

function thFanStateAt(k) {
  // 后端 result.fan 已反映当前 spec.fan.mode（含手动开关触发的整段重算）
  return (thResult.fan || [])[k] ? 1 : 0;
}

function thHeatArrow(svg, x1, y1, x2, y2, kw, color) {
  const w = Math.min(4.5, 0.6 + Math.log10(1 + Math.max(0, kw)) * 2.2);
  svgEl('line', { x1, y1, x2, y2, stroke: color || '#e08e45',
    'stroke-width': Math.max(0.4, w), opacity: 0.75,
    'marker-end': '' }, svg);
  const mx = (x1 + x2) / 2, my = (y1 + y2) / 2;
  svgEl('text', { x: mx + 2, y: my - 2, class: 'th-flow-label',
    text: kw >= 0.01 ? kw.toFixed(2) + ' kW' : '' }, svg);
}

/* ---------------- 温度时间轴（可拖游标 + 段背景 + 问题区间） ---------------- */
function thChartGeom() {
  const W = 560, H = 220, ml = 36, mr = 10, mt = 12, mb = 34;
  const times = thResult.timesH || [0];
  const tEnd = Math.max(1e-9, times[times.length - 1]);
  const allT = [thDraft.spec.ambient, thDraft.spec.initTemp];
  for (const id of Object.keys(thResult.temps)) allT.push(...thResult.temps[id]);
  let tMin = Math.min(...allT), tMax = Math.max(...allT);
  for (const n of (thResult.nodes || [])) tMax = Math.max(tMax, n.limit);
  if (tMax - tMin < 5) { tMin -= 3; tMax += 3; }
  const pad = (tMax - tMin) * 0.08;
  tMin -= pad; tMax += pad;
  return { W, H, ml, mr, mt, mb, tEnd, tMin, tMax,
    X: hh => ml + (W - ml - mr) * (hh / tEnd),
    Y: T => mt + (H - mt - mb) * (1 - (T - tMin) / (tMax - tMin)) };
}

function thVisibleSeries() {
  // 默认显示油/箱体/全部齿轮节点；轴承仅在热网络点选后追加其曲线
  if (!thDraft._series) thDraft._series = {};
  const out = [];
  for (const n of thResult.nodes || []) {
    if (n.kind === 'bearing') {
      if (thSelectedNode === n.id) out.push(n);
    } else out.push(n);
  }
  return out;
}

function thRenderChartOnly() {
  const svg = $('#th-chart');
  if (!thDraft || !thResult || !thResult.times) {
    if (svg) svg.innerHTML = '';
    return;
  }
  const g = thChartGeom();
  const { W, H, ml, mr, mt, mb, X, Y, tMin, tMax, tEnd } = g;
  svg.setAttribute('viewBox', `0 0 ${W} ${H}`);
  svg.innerHTML = '';
  const layers = { band: svgEl('g', {}, svg), grid: svgEl('g', {}, svg),
    limit: svgEl('g', {}, svg), curve: svgEl('g', {}, svg),
    cursor: svgEl('g', {}, svg) };
  svg._thGeom = g;

  // 时段背景
  for (const b of thResult.bands || []) {
    const x1 = X(b.t0 / 3600), x2 = X(b.t1 / 3600);
    svgEl('rect', { x: x1, y: mt, width: Math.max(1, x2 - x1), height: H - mt - mb,
      fill: b.kind === 'stop' ? '#ece4d6' : '#f3f6fc', opacity: 0.8 }, layers.band);
    svgEl('text', { x: (x1 + x2) / 2, y: mt + 10, class: 'lc-tick',
      text: b.name }, layers.band).setAttribute('text-anchor', 'middle');
  }
  // 网格与坐标
  for (let i = 0; i <= 4; i++) {
    const T = tMin + (tMax - tMin) * i / 4, y = Y(T);
    svgEl('line', { x1: ml, y1: y, x2: W - mr, y2: y, stroke: '#e0d9cb',
      'stroke-width': 0.5 }, layers.grid);
    svgEl('text', { x: ml - 3, y: y + 3, class: 'lc-tick', text: T.toFixed(0) }, layers.grid)
      .setAttribute('text-anchor', 'end');
  }
  for (let i = 0; i <= 5; i++) {
    const hh = tEnd * i / 5;
    svgEl('text', { x: X(hh), y: H - mb + 12, class: 'lc-tick',
      text: hh >= 1 ? hh.toFixed(1) + 'h' : Math.round(hh * 60) + 'm' }, layers.grid)
      .setAttribute('text-anchor', 'middle');
  }
  // 温度限虚线
  for (const [id, lim] of [['mesh', thDraft.spec.limits.mesh],
                           ['bearing', thDraft.spec.limits.bearing],
                           ['oil', thDraft.spec.limits.oil]]) {
    if (lim >= tMin && lim <= tMax)
      svgEl('line', { x1: ml, y1: Y(lim), x2: W - mr, y2: Y(lim),
        stroke: TH_NODE_COLORS[id], 'stroke-width': 0.7, 'stroke-dasharray': '4 3',
        opacity: 0.6 }, layers.limit);
  }
  // 温度曲线
  const times = thResult.timesH;
  for (const n of thVisibleSeries()) {
    const arr = thResult.temps[n.id];
    if (!arr) continue;
    const d = arr.map((T, i) => `${i ? 'L' : 'M'}${X(times[i]).toFixed(1)},${Y(T).toFixed(1)}`).join('');
    const sel = thSelectedNode === n.id;
    svgEl('path', { d, fill: 'none', stroke: TH_NODE_COLORS[n.kind] || '#666',
      'stroke-width': sel ? 2.4 : n.kind === 'oil' ? 2.0 : 1.0,
      opacity: sel ? 1 : n.kind === 'oil' ? 0.95 : 0.55 }, layers.curve);
  }
  // 黏度曲线（虚线橙色，log 量纲映射）
  const visc = thResult.visc;
  if (visc) {
    const vMin = Math.min(...visc, thDraft.spec.oil.minVisc);
    const vMax = Math.max(...visc, thDraft.spec.oil.maxVisc);
    const Yv = v => mt + (H - mt - mb) * (1 - (Math.log10(v) - Math.log10(vMin)) /
      (Math.log10(vMax) - Math.log10(vMin) || 1));
    const dv = visc.map((v, i) => `${i ? 'L' : 'M'}${X(times[i]).toFixed(1)},${Yv(v).toFixed(1)}`).join('');
    svgEl('path', { d: dv, fill: 'none', stroke: '#c98a1f', 'stroke-width': 1.2,
      'stroke-dasharray': '5 3', opacity: 0.8 }, layers.curve);
  }

  // 游标（拖动时只移动该组，不整体重绘）
  const k = Math.max(0, Math.min(thCursor, times.length - 1));
  thCursor = k;
  const cg = svgEl('g', { class: 'th-cursor' }, layers.cursor);
  svgEl('line', { x1: 0, y1: mt, x2: 0, y2: H - mb, stroke: '#c62828',
    'stroke-width': 1 }, cg);
  svgEl('rect', { x: -6, y: mt - 2, width: 12, height: 8, rx: 2,
    fill: '#c62828', cursor: 'ew-resize' }, cg);
  cg.setAttribute('transform', `translate(${X(times[k])},0)`);

  thUpdateCursorReadout();
  thRenderLegend();
  thRenderNetwork();
  thRenderViscCurve();
}

/* 时间轴游标拖动（事件只绑一次） */
function thBindChartDrag() {
  const svg = $('#th-chart');
  let dragging = false;
  const toK = clientX => {
    const { ml, W, mr, tEnd } = svg._thGeom;
    const rect = svg.getBoundingClientRect();
    const vb = svg.viewBox.baseVal;
    const px = (clientX - rect.left) / rect.width * vb.width;
    const hh = (px - ml) / (W - ml - mr) * tEnd;
    const times = thResult.timesH;
    let lo = 0, hi = times.length - 1;
    while (lo < hi) {
      const mid = (lo + hi) >> 1;
      if (times[mid] < hh) lo = mid + 1; else hi = mid;
    }
    return Math.max(0, Math.min(times.length - 1, lo));
  };
  svg.addEventListener('pointerdown', e => {
    if (!thResult || !svg._thGeom) return;
    dragging = true; svg.setPointerCapture(e.pointerId);
    thCursor = toK(e.clientX); thRenderChartOnly();
  });
  svg.addEventListener('pointermove', e => {
    if (!dragging || !svg._thGeom) return;
    thCursor = toK(e.clientX);
    const cg = svg.querySelector('.th-cursor');
    if (cg) cg.setAttribute('transform',
      `translate(${svg._thGeom.X(thResult.timesH[thCursor])},0)`);
    thUpdateCursorReadout(); thRenderNetwork(); thRenderViscCurve();
  });
  const end = () => { dragging = false; };
  svg.addEventListener('pointerup', end);
  svg.addEventListener('pointercancel', end);
}

function thUpdateCursorReadout() {
  const box = $('#th-cursor-readout');
  if (!thResult || !box) return;
  const k = Math.max(0, Math.min(thCursor, thResult.times.length - 1));
  const t = thResult.timesH[k];
  const fan = thFanStateAt(k);
  const node = thSelectedNode;
  let txt = `t = ${t >= 1 ? t.toFixed(2) + ' h' : Math.round(t * 60) + ' min'} · ` +
    `油温 ${thResult.temps.oil[k].toFixed(1)}°C · 黏度 ${thResult.visc[k].toFixed(1)} mm²/s · ` +
    `风扇 ${fan ? '开' : '关'}`;
  if (node && thResult.temps[node]) {
    const q = (thResult.qin[node] || [])[k] || 0;
    const fkey = node + '>oil';
    const f = (thResult.flows[fkey] || [])[k] || 0;
    txt += ` · ${thResult.nodeNames[node]}：${thResult.temps[node][k].toFixed(1)}°C，` +
      `热源 ${q.toFixed(3)} kW，→油 ${f.toFixed(3)} kW`;
  }
  box.textContent = txt;
}

function thRenderLegend() {
  const wrap = $('#th-legend');
  wrap.innerHTML = '';
  const items = [{ c: TH_NODE_COLORS.oil, t: '油温' },
    { c: TH_NODE_COLORS.housing, t: '箱体' },
    { c: TH_NODE_COLORS.mesh, t: '齿轮节点' },
    { c: TH_NODE_COLORS.bearing, t: '轴承（点网络节点查看）' },
    { c: '#c98a1f', t: '油黏度（虚线）', dash: true }];
  for (const it of items) {
    const sp = h('span', { class: 'th-lg-item' }, wrap);
    h('i', { class: 'th-lg-line' + (it.dash ? ' dash' : ''),
      style: `background:${it.c}` }, sp);
    sp.appendChild(document.createTextNode(it.t));
  }
}

/* ---------------- 搜索 ---------------- */
function thRenderSearchOils() {
  const wrap = $('#th-search-oils');
  if (wrap.dataset.built === '1') return;
  wrap.dataset.built = '1';
  wrap.innerHTML = '';
  const cur = thDraft.spec.oil.grade;
  const grades = (thOils.map(o => o.grade));
  for (const g of grades) {
    const lab = h('label', {}, wrap);
    const cb = h('input', { type: 'checkbox', value: g }, lab);
    // 全部油品默认参与（当前油品由后端始终包含，不依赖勾选）
    cb.checked = true;
    lab.appendChild(document.createTextNode(' ' + g));
  }
  // 搜索范围默认值
  const setV = (sel, v) => { const i = $(sel); if (!i.value) i.value = v; };
  const a = thDraft.spec.housing.area;
  setV('#th-a-min', (a * 0.6).toFixed(2));
  setV('#th-a-max', (a * 1.8).toFixed(2));
  setV('#th-a-step', Math.max(0.1, +(a * 0.4).toFixed(2)));
  setV('#th-on-min', Math.max(35, thDraft.spec.fan.onTemp - 10));
  setV('#th-on-max', thDraft.spec.fan.onTemp + 10);
  setV('#th-on-step', 5);
  setV('#th-off-min', Math.max(30, thDraft.spec.fan.offTemp - 10));
  setV('#th-off-max', thDraft.spec.fan.offTemp + 5);
  setV('#th-off-step', 5);
}

async function thRunSearch() {
  if (!thDraft) return;
  const oils = [...document.querySelectorAll('#th-search-oils input:checked')]
    .map(cb => cb.value);
  const body = {
    spec: thDraft.spec, frozen: thDraft.frozen, oils,
    lockCooling: $('#th-lock-cool').checked,
    areaMin: parseFloat($('#th-a-min').value),
    areaMax: parseFloat($('#th-a-max').value),
    areaStep: parseFloat($('#th-a-step').value),
    fanOnMin: parseFloat($('#th-on-min').value),
    fanOnMax: parseFloat($('#th-on-max').value),
    fanOnStep: parseFloat($('#th-on-step').value),
    fanOffMin: parseFloat($('#th-off-min').value),
    fanOffMax: parseFloat($('#th-off-max').value),
    fanOffStep: parseFloat($('#th-off-step').value),
    limit: 12, timeBudget: 15,
  };
  $('#ths-meta').textContent = '搜索中…';
  const resp = await fetch('/api/thermal/search', {
    method: 'POST', headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(body),
  });
  thSearchRes = await resp.json();
  thRenderSearchList();
}

function thRenderSearchList() {
  const list = $('#ths-list'), meta = $('#ths-meta');
  list.innerHTML = '';
  if (!thSearchRes) return;
  const r = thSearchRes;
  meta.textContent = `枚举 ${r.nodes} 个组合${r.truncated ? '（达到时限截断）' : ''}；` +
    `当前：超温 ${(r.base.overTimeS / 60).toFixed(1)} min · 峰值 ${r.base.peakTemp}°C · ` +
    `${r.base.energyKwh} kWh`;
  (r.results || []).forEach((c, i) => {
    const li = h('li', { class: 'cand lc-cand' + (c.current ? ' current-cand' : '') }, list);
    const head = h('div', { class: 'head' }, li);
    h('span', {}, head, `#${i + 1} ${c.oilGrade} · 面积 ${c.area} m² · 风扇 ${c.fanOn}/${c.fanOff}°C` +
      (c.current ? '（当前）' : ''));
    h('span', { class: 'muted small' }, head, `改动量 ${c.change}`);
    h('div', { class: 'meta' }, li,
      `超温 ${(c.overTimeS / 60).toFixed(1)} min · 峰值 ${c.peakTemp}°C · ` +
      `黏度越界 ${(c.viscOverS / 60).toFixed(1)} min · 风扇切换 ${c.fanCycles} 次 · ` +
      `能耗 ${c.energyKwh} kWh · ${c.converged ? '已收敛' : '未收敛'}`);
    const acts = h('div', { class: 'actions' }, li);
    h('button', { class: 'primary', onclick: () => thApplyCandidate(c) }, acts, '采用并另存版本');
  });
}

function thApplyCandidate(c) {
  const sp = thDraft.spec;
  if (c.patch.oil.points) {
    sp.oil.grade = c.patch.oil.grade;
    thSetPoints(c.patch.oil.points);
  }
  sp.housing.area = c.patch.housing.area;
  sp.fan.onTemp = c.patch.fan.onTemp;
  sp.fan.offTemp = c.patch.fan.offTemp;
  $('#th-visc-points').dataset.n = '';
  thSaveDraft();
  // 采用后先重算，待解返回再另存版本（solution 为采用方案的完整时间步曲线）
  (async () => {
    await thAnalyze();
    await thSaveVersion(true);
    thRender();
    flashHint('已采用搜索方案并另存热平衡版本');
  })();
}

/* ---------------- 版本 ---------------- */
async function thLoadVersions() {
  try {
    thVersions = await (await fetch('/api/thermal/cases')).json();
  } catch (e) { thVersions = []; }
  const sel = $('#th-versions');
  const cur = sel.value;
  sel.innerHTML = '';
  if (!thVersions.length) h('option', { value: '' }, sel, '— 尚无已存版本 —');
  for (const v of thVersions) {
    const o = h('option', { value: v.id }, sel,
      `${v.name} · v${v.version}${v.note ? `（${v.note}）` : ''}`);
    if (String(v.id) === cur) o.selected = true;
  }
}

async function thSaveVersion(fromCandidate) {
  if (!thDraft) return;
  // 版本保存时把最近一次完整解（时间步 + 全部温度曲线）存入 solution
  const resp = await fetch('/api/thermal/cases', {
    method: 'POST', headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({
      name: thDraft.name || '热平衡工况',
      spec: thDraft.spec, frozen: thDraft.frozen,
      solution: thResult, note: null,
    }),
  });
  const data = await resp.json();
  if (data.ok) {
    flashHint(`已另存热平衡版本 v${data.version}（含来源快照、时间步与全部温度曲线）`);
    thLoadVersions();
  }
}

/* ---------------- 事件绑定 ---------------- */
function thBind() {
  thBindChartDrag();
  $('#btn-th-new').addEventListener('click', () => thNewDraft(true));
  $('#btn-th-refresh').addEventListener('click', thRefreeze);
  $('#th-name').addEventListener('input', e => {
    if (thDraft) { thDraft.name = e.target.value; thSaveDraft(); }
  });
  const bindNum = (sel, path, onch) => {
    $(sel).addEventListener('input', e => {
      if (!thDraft) return;
      const v = parseFloat(e.target.value);
      onch(thDraft.spec, v, e.target.value);
      thSpecChanged();
    });
  };
  bindNum('#th-ambient', null, (sp, v) => { sp.ambient = v || 0; });
  bindNum('#th-init', null, (sp, v) => { sp.initTemp = v || 0; });
  bindNum('#th-dt', null, (sp, v) => { sp.dt = Math.max(1, v || 30); });
  bindNum('#th-oil-vol', null, (sp, v) => { sp.oil.volumeL = Math.max(0.01, v || 0); });
  bindNum('#th-oil-rho', null, (sp, v) => { sp.oil.rho = v || 0; });
  bindNum('#th-oil-cp', null, (sp, v) => { sp.oil.cp = v || 0; });
  bindNum('#th-visc-min', null, (sp, v) => { sp.oil.minVisc = v || 0; });
  bindNum('#th-visc-max', null, (sp, v) => { sp.oil.maxVisc = v || 0; });
  bindNum('#th-oil-sens', null, (sp, v) => { sp.oil.sens = Math.max(0, v || 0); });
  bindNum('#th-h-cap', null, (sp, v) => { sp.housing.capacity = Math.max(1e-6, v || 0); });
  bindNum('#th-h-area', null, (sp, v) => { sp.housing.area = Math.max(0.01, v || 0); });
  bindNum('#th-rnat', null, (sp, v) => { sp.housing.rNat = Math.max(1e-6, v || 0); });
  bindNum('#th-rfan', null, (sp, v) => { sp.housing.rFan = Math.max(1e-6, v || 0); });
  bindNum('#th-roh', null, (sp, v) => { sp.oilNode.rHousing = Math.max(1e-6, v || 0); });
  $('#th-fan-mode').addEventListener('change', e => {
    if (thDraft) { thDraft.spec.fan.mode = e.target.value; thSpecChanged(); }
  });
  bindNum('#th-fan-on', null, (sp, v) => { sp.fan.onTemp = v || 0; });
  bindNum('#th-fan-off', null, (sp, v) => { sp.fan.offTemp = v || 0; });
  bindNum('#th-fan-p', null, (sp, v) => { sp.fan.powerKw = Math.max(0, v || 0); });
  bindNum('#th-fan-min', null, (sp, v) => { sp.fan.minCycleMin = Math.max(0, v || 0); });
  bindNum('#th-lim-mesh', null, (sp, v) => { sp.limits.mesh = v || 0; });
  bindNum('#th-lim-bearing', null, (sp, v) => { sp.limits.bearing = v || 0; });
  bindNum('#th-lim-oil', null, (sp, v) => { sp.limits.oil = v || 0; });

  $('#th-oil-grade').addEventListener('change', e => {
    if (!thDraft) return;
    const preset = thOils.find(o => o.grade === e.target.value);
    if (preset) {
      thDraft.spec.oil.grade = preset.grade;
      thSetPoints(preset.points);
    } else {
      thDraft.spec.oil.grade = '自定义';
    }
    thSpecChanged(); thRender();
  });

  $('#btn-th-add-point').addEventListener('click', () => {
    if (!thDraft) return;
    const pts = thDraft.spec.oil.points;
    const lastT = pts.length ? pts[pts.length - 1][0] : 40;
    const lastV = pts.length ? pts[pts.length - 1][1] : 100;
    // 新折点默认在末端升温 20°C、黏度按经验减半
    pts.push([+lastT + 20, Math.max(1, +(lastV / 2).toFixed(2))]);
    $('#th-visc-points').dataset.n = '';   // 强制重建
    thOnOilPointsEdited(); thRender();
  });

  $('#btn-th-add-run').addEventListener('click', () => {
    if (!thDraft) return;
    const key = (thDraft.frozen.order || [])[0];
    thDraft.spec.segments.push({ id: 'seg' + Date.now(), name: '运行段',
      kind: 'run', sourceId: key, loadScale: 1, speedScale: 1,
      durationH: 1, ambient: null });
    thSpecChanged(); thRender();
  });
  $('#btn-th-add-stop').addEventListener('click', () => {
    if (!thDraft) return;
    thDraft.spec.segments.push({ id: 'seg' + Date.now(), name: '停机段',
      kind: 'stop', sourceId: '', loadScale: 1, speedScale: 1,
      durationH: 1, ambient: null });
    thSpecChanged(); thRender();
  });

  document.querySelectorAll('input[name="th-fan-override"]').forEach(r =>
    r.addEventListener('change', () => {
      const checked = [...document.querySelectorAll('input[name="th-fan-override"]')]
        .find(x => x.checked);
      thFanOverride = checked ? checked.value : '';
      if (!thDraft) return;
      if (thFanOverride) {
        if (thDraft.spec.fan.mode === 'auto' ||
            (thDraft.spec.fan.mode !== thFanOverride && !thDraft._fanModeBeforeOverride))
          thDraft._fanModeBeforeOverride = thDraft.spec.fan.mode;
        thDraft.spec.fan.mode = thFanOverride;   // 强制开/关：整段重算
      } else if (thDraft._fanModeBeforeOverride) {
        thDraft.spec.fan.mode = thDraft._fanModeBeforeOverride;  // 恢复自动/原方案
        delete thDraft._fanModeBeforeOverride;
      }
      $('#th-fan-mode').value = thDraft.spec.fan.mode;
      thSpecChanged();
    }));

  $('#btn-th-search').addEventListener('click', thRunSearch);
  $('#btn-th-save').addEventListener('click', () => thSaveVersion(false));
  $('#btn-th-reload-draft').addEventListener('click', async () => {
    const d = await thLoadServerDraft();
    if (!d) { flashHint('服务端还没有保存的热平衡草稿'); return; }
    thDraft = d;
    $('#th-oil-grade').dataset.built = '';
    thReset();
    try { localStorage.setItem(TH_KEY, JSON.stringify(d)); } catch (e) { /* ignore */ }
    thAnalyzeSoon(); thRender(); thLoadVersions();
    flashHint('已从服务端恢复热平衡草稿');
  });
  $('#btn-th-load').addEventListener('click', async () => {
    const id = $('#th-versions').value;
    if (!id) return;
    const v = await (await fetch(`/api/thermal/cases/${id}`)).json();
    if (!v.spec) return;
    thDraft = { name: v.name, spec: v.spec, frozen: v.frozen };
    $('#th-oil-grade').dataset.built = '';
    thReset();
    thSaveDraft(); thAnalyzeSoon(); thRender();
    flashHint(`已载入热平衡「${v.name}」v${v.version}（来源变化仅标记过期）`);
  });
  $('#btn-th-del').addEventListener('click', async () => {
    const id = $('#th-versions').value;
    if (!id) return;
    await fetch(`/api/thermal/cases/${id}`, { method: 'DELETE' });
    thLoadVersions();
  });
}

/* ---------------- 启动（服务端草稿优先，本地为离线后备） ---------------- */
async function thLoadServerDraft() {
  try {
    const d = await (await fetch('/api/thermal/draft')).json();
    if (d && d.draft && d.draft.spec) return d.draft;
  } catch (e) { /* 网络失败退到本地 */ }
  return null;
}

(async function thInit() {
  thBind();
  await thLoadOils();
  // 服务端草稿优先（跨浏览器/清站点数据后可恢复）；否则用本地离线副本
  thDraftLoading = true;
  const server = await thLoadServerDraft();
  if (server) {
    thDraft = server;
    try { localStorage.setItem(TH_KEY, JSON.stringify(server)); } catch (e) { /* ignore */ }
  } else {
    try {
      const raw = localStorage.getItem(TH_KEY);
      if (raw) thDraft = JSON.parse(raw);
    } catch (e) { thDraft = null; }
    if (thDraft) thPushDraft();   // 服务端无草稿时把本地副本补传上去
  }
  thDraftLoading = false;
  thRender();
  if (thDraft) thAnalyze();
  thLoadVersions();
})();
