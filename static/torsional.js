/* 扭转振动工作区 — 原生 JS/SVG。
   草稿 = {name, spec, frozen}：frozen 从载荷版本（或内联当前载荷草稿）冻结
   动力路径、转速比与平均转矩；spec 填各轴转动惯量、啮合/联轴器扭转刚度阻尼、
   转速扫描范围与驱动/负载端激励阶次。固有模态、Campbell 穿越、啮合动态转矩与
   放大系数由后端 torsional.py 计算。采用后冻结输入/结果/曲线，源轮系或载荷变化
   只标记过期。 */
'use strict';

const TV_KEY = 'torsionalDraftV1';

let tvDraft = null;        // {name, spec, frozen}
let tvResult = null;
let tvToken = 0;
let tvTimer = null;
let tvVersions = [];
let tvCursor = -1;         // 转速样本下标
let tvSelectedMode = -1;
let tvSearchRes = null;
let tvStructKey = '';
let tvDraftTimer = null;
let tvDraftLoading = false;

/* ---------------- 指纹（与后端 thermal.fingerprint / canonical_dumps 逐字节一致） ---------------- */
function tvNormNum(v, dflt) {
  let f = Number(v);
  if (!Number.isFinite(f)) f = Number.isFinite(+dflt) ? +dflt : 0;
  f = Math.round(f * 1e6) / 1e6;
  if (Object.is(f, -0)) f = 0;
  return Number.isInteger(f) ? f : f;
}
function tvCanonical(obj) {
  if (obj === null || typeof obj !== 'object') return JSON.stringify(obj);
  if (Array.isArray(obj)) return '[' + obj.map(tvCanonical).join(',') + ']';
  const keys = Object.keys(obj).sort();
  return '{' + keys.map(k => JSON.stringify(k) + ':' + tvCanonical(obj[k])).join(',') + '}';
}
function tvTrainFingerprint(st) {
  const S = Object.fromEntries((st.shafts || []).map(s => [s.id, s]));
  const G = Object.fromEntries((st.gears || []).map(g => [g.id, g]));
  const items = [];
  for (const e of (st.meshes || [])) {
    const ga = G[e.gearA], gb = G[e.gearB];
    if (!ga || !gb) { items.push([String(e.id), 'broken']); continue; }
    const sa = S[ga.shaftId] || {}, sb = S[gb.shaftId] || {};
    items.push([
      String(e.id),
      tvNormNum(ga.z), tvNormNum(ga.module), tvNormNum(ga.x || 0),
      tvNormNum(ga.pressureAngle || 20), !!ga.internal,
      tvNormNum(gb.z), tvNormNum(gb.module), tvNormNum(gb.x || 0),
      tvNormNum(gb.pressureAngle || 20), !!gb.internal,
      tvNormNum(sa.x || 0), tvNormNum(sa.y || 0),
      tvNormNum(sb.x || 0), tvNormNum(sb.y || 0)]);
  }
  items.sort((a, b) => a[0] < b[0] ? -1 : a[0] > b[0] ? 1 : 0);
  return tvCanonical(items);
}
function tvPrimarySource() {
  if (!tvDraft) return null;
  const key = tvDraft.spec.primarySource;
  return (tvDraft.frozen.sources || {})[key] || null;
}
function tvSourceStale() {
  const src = tvPrimarySource();
  if (!src) return false;
  if (src.inline) {
    if (typeof lcDraft === 'undefined' || !lcDraft) return true;
    if (src.trainFp !== tvTrainFingerprint(lcDraft.snapshot)) return true;
    if (src.loadSpecFp && src.loadSpecFp !== tvCanonical(lcDraft.spec)) return true;
  } else if (src.trainFp !== tvTrainFingerprint(state)) {
    return true;
  }
  return false;
}

/* ---------------- 草稿建立 / 冻结 ---------------- */
async function tvBuildSources(inlineCurrent) {
  const caseRows = await (await fetch('/api/loadcase/cases')).json();
  const sources = [];
  for (const row of caseRows) {
    const v = await (await fetch(`/api/loadcase/cases/${row.id}`)).json();
    if (!v.spec || !v.snapshot) continue;
    sources.push({ key: 'lc' + row.id + 'v' + row.version,
      name: `${row.name} v${row.version}`, state: v.snapshot, spec: v.spec,
      caseId: row.id, version: row.version });
  }
  if (inlineCurrent && typeof lcDraft !== 'undefined' && lcDraft && lcDraft.spec)
    sources.push({ key: 'inline', name: '当前载荷草稿（未另存）',
      state: lcDraft.snapshot, spec: lcDraft.spec, inline: true });
  return sources;
}

async function tvNewDraft() {
  const inlineCurrent = $('#tv-inline').checked;
  const sources = await tvBuildSources(inlineCurrent);
  if (!sources.length) {
    alert('还没有可用的载荷来源。请先在「载荷工况」页保存版本，或勾选「同时内联当前载荷草稿」。');
    return;
  }
  // 取所选/第一个来源作为平均转矩来源
  const frozen = await (await fetch('/api/torsional/freeze', {
    method: 'POST', headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ sources }),
  })).json();
  const firstKey = (frozen.order || [])[0];
  const src = frozen.sources[firstKey];
  if (!src || src.error || !src.ok) {
    alert('冻结来源的载荷分析未通过：' + (src && src.error || '未知错误') +
      '\n请先在载荷工况页修正该版本（输入转矩/转速、分支比例等）。');
    return;
  }
  const spec = await (await fetch('/api/torsional/default-spec', {
    method: 'POST', headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ frozen }),
  })).json();
  spec.primarySource = firstKey;
  tvDraft = { name: (tvDraft && tvDraft.name) || '扭振工况', spec, frozen };
  tvReset();
  tvSaveDraft(); tvAnalyzeSoon(); tvRender();
  flashHint('已冻结动力路径/转速比/平均转矩，建立扭振草稿');
}

async function tvRefreeze() {
  if (!tvDraft) return;
  const key = tvDraft.spec.primarySource;
  const old = tvDraft.frozen.sources[key];
  if (!old) return;
  let sources;
  if (old.inline && typeof lcDraft !== 'undefined' && lcDraft) {
    sources = [{ key: old.key, name: old.name, state: lcDraft.snapshot,
      spec: lcDraft.spec, inline: true }];
  } else if (old.caseId) {
    const v = await (await fetch(`/api/loadcase/cases/${old.caseId}`)).json();
    if (!v.spec) { alert('原载荷版本已不存在，请重新建立草稿。'); return; }
    sources = [{ key: old.key, name: old.name, state: v.snapshot, spec: v.spec,
      caseId: v.id, version: v.version }];
  } else return;
  const frozen = await (await fetch('/api/torsional/freeze', {
    method: 'POST', headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ sources }),
  })).json();
  // 用后端默认值补缺（保留已录入的惯量/刚度/扫描/激励）
  tvDraft.frozen = frozen;
  const merged = await (await fetch('/api/torsional/default-spec', {
    method: 'POST', headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ frozen }),
  })).json();
  tvReconcileSpec(merged);
  tvReset();
  tvSaveDraft(); tvAnalyzeSoon(); tvRender();
  flashHint('已重新冻结来源（动力路径/转速比/平均转矩已更新）');
}

/* 重新冻结后把默认结构与已录入值合并（后端 _merge_spec 在分析时也会兜底，
   这里保证前端表单结构完整：新增轴/啮合给默认值，消失的剔除）。 */
function tvReconcileSpec(base) {
  const sp = tvDraft.spec;
  sp.primarySource = base.primarySource;
  const ns = {};
  for (const [sid, v] of Object.entries(base.shafts))
    ns[sid] = sp.shafts && sp.shafts[sid] ? sp.shafts[sid] : v;
  sp.shafts = ns;
  const nm = {};
  for (const [mid, v] of Object.entries(base.meshes))
    nm[mid] = sp.meshes && sp.meshes[mid] ? sp.meshes[mid] : v;
  sp.meshes = nm;
  const keepC = [], have = new Set();
  for (const c of (sp.couplings || [])) {
    const b = base.couplings.find(x => x.id === c.id);
    if (b) { keepC.push({ ...c, kind: b.kind }); have.add(c.id); }
  }
  for (const b of base.couplings) if (!have.has(b.id)) keepC.push(b);
  sp.couplings = keepC;
  const ids = new Set(base.couplings.map(c => c.id));
  sp.addedFlywheels = (sp.addedFlywheels || []).filter(f => sp.shafts[f.shaftId]);
  if (!ids.has(sp.excitation && sp.excitation.nodeId))
    sp.excitation = base.excitation;
}

function tvReset() {
  tvResult = null; tvSearchRes = null; tvCursor = -1; tvSelectedMode = -1;
  tvStructKey = '';
}

/* ---------------- 草稿持久化 ---------------- */
function tvSaveDraft() {
  if (!tvDraft || tvDraftLoading) return;
  try { localStorage.setItem(TV_KEY, JSON.stringify(tvDraft)); } catch (e) { /* ignore */ }
  clearTimeout(tvDraftTimer);
  tvDraftTimer = setTimeout(tvPushDraft, 800);
}
async function tvPushDraft() {
  if (!tvDraft) return;
  try {
    await fetch('/api/torsional/draft', {
      method: 'PUT', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ draft: tvDraft }),
    });
  } catch (e) { /* 本地副本仍在 */ }
}
function tvSpecChanged() { tvSearchRes = null; tvSaveDraft(); tvAnalyzeSoon(); }

/* ---------------- 后端分析 ---------------- */
function tvAnalyzeSoon() {
  clearTimeout(tvTimer);
  tvTimer = setTimeout(tvAnalyze, 180);
}
async function tvAnalyze() {
  if (!tvDraft) return;
  const token = ++tvToken;
  try {
    const resp = await fetch('/api/torsional/analyze', {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ spec: tvDraft.spec, frozen: tvDraft.frozen }),
    });
    const data = await resp.json();
    if (token !== tvToken) return;
    tvResult = data;
    if (tvCursor < 0 || tvCursor >= (data.rpms || []).length)
      tvCursor = (data.rpms || []).length - 1;
  } catch (e) {
    if (token === tvToken) tvResult = { ok: false, issues: [
      { severity: 'error', code: 'FETCH', message: '计算失败：' + e }] };
  }
  tvRenderResults();
}

/* ---------------- 渲染总入口 ---------------- */
function tvRender() {
  const has = !!tvDraft;
  $('#tv-body').style.display = has ? '' : 'none';
  $('#tv-empty').style.display = has ? 'none' : '';
  if (!has) return;
  const nameInp = $('#tv-name');
  if (document.activeElement !== nameInp) nameInp.value = tvDraft.name || '';
  $('#tv-stale').style.display = tvSourceStale() ? '' : 'none';
  tvFillInputs();
  tvRenderSource();
  tvRenderOrders();
  tvRenderShafts();
  tvRenderSprings();
  tvRenderSearchShafts();
  tvRenderResults();
}
function tvSchedule() {
  if (tvDraft) $('#tv-stale').style.display = tvSourceStale() ? '' : 'none';
}
function tvTabShown() { tvLoadVersions(); tvRender(); }

function tvFillInputs() {
  const sp = tvDraft.spec;
  const set = (sel, v) => {
    const inp = $(sel);
    if (document.activeElement !== inp) inp.value = v ?? '';
  };
  set('#tv-rpm-min', sp.scan.rpmMin); set('#tv-rpm-max', sp.scan.rpmMax);
  set('#tv-rpm-step', sp.scan.rpmStep);
  set('#tv-zeta-min', sp.limits.zetaMin); set('#tv-tfactor', sp.limits.torqueFactor);
  const sel = $('#tv-ex-node');
  const opts = sp.couplings.map(c => [c.id,
    (c.kind === 'driver' ? '驱动端' : '负载端') + '（' +
    ((tvPrimarySource() || {}).shaftNames || {})[c.shaftId] + '）']);
  const key = opts.map(o => o[0]).join('|');
  if (sel.dataset.key !== key) {
    sel.dataset.key = key; sel.innerHTML = '';
    for (const [id, label] of opts) h('option', { value: id }, sel, label);
  }
  if (document.activeElement !== sel) sel.value = sp.excitation.nodeId || '';
}

function tvRenderSource() {
  const box = $('#tv-source');
  const src = tvPrimarySource();
  box.innerHTML = '';
  if (!src) { box.textContent = '冻结来源缺失'; return; }
  const card = h('div', { class: 'th-source-card' + (src.ok ? '' : ' unused') }, box);
  const head = h('div', { class: 'bl-mesh-head' }, card);
  h('span', {}, head, src.name + (src.inline ? '（载荷草稿）' : ''));
  h('span', { class: 'muted small' }, head,
    `输入 ${src.rpm0} rpm · 平均转矩 ${src.torqueInNm} N·m · ${src.shafts.length} 轴 · ${src.meshes.length} 啮合`);
  h('div', { class: 'muted small' }, card,
    '动力路径、转速比与平均转矩已冻结；后续修改源轮系或载荷不会自动重算，只标记过期。');
}

/* ---------------- 激励阶次 ---------------- */
function tvRenderOrders() {
  const wrap = $('#tv-orders');
  const orders = tvDraft.spec.excitation.orders;
  if (wrap.dataset.n === String(orders.length)) {
    wrap.querySelectorAll('.tv-order-row').forEach((row, i) => {
      const o = orders[i];
      const hi = row.querySelector('[data-h]'), ai = row.querySelector('[data-amp]');
      if (document.activeElement !== hi) hi.value = o.h;
      if (document.activeElement !== ai) ai.value = o.amp;
    });
    return;
  }
  wrap.dataset.n = String(orders.length);
  wrap.innerHTML = '';
  orders.forEach((o, i) => {
    const row = h('div', { class: 'tv-order-row' }, wrap);
    const lh = h('label', {}, row, '阶次 h');
    const hi = h('input', { type: 'number', step: '0.5', min: '0.1', 'data-h': '' }, lh);
    hi.value = o.h;
    hi.addEventListener('input', () => { const v = parseFloat(hi.value);
      if (v > 0) { o.h = v; tvSpecChanged(); } });
    const la = h('label', {}, row, '幅值 N·m');
    const ai = h('input', { type: 'number', step: '0.1', min: '0', 'data-amp': '' }, la);
    ai.value = o.amp;
    ai.addEventListener('input', () => { o.amp = Math.max(0, parseFloat(ai.value) || 0); tvSpecChanged(); });
    h('button', { class: 'danger', onclick: () => {
      orders.splice(i, 1); wrap.dataset.n = ''; tvSpecChanged(); tvRender(); } }, row, '删');
  });
}

/* ---------------- 轴惯量 / 弹簧参数 ---------------- */
function tvRenderShafts() {
  const src = tvPrimarySource();
  const wrap = $('#tv-shafts');
  const key = src.shafts.map(s => s.id).join('|') + '|' +
    tvDraft.spec.addedFlywheels.map(f => f.shaftId + ':' + f.inertia).join(',');
  if (key === wrap.dataset.key) return;
  wrap.dataset.key = key;
  wrap.innerHTML = '';
  const names = src.shaftNames || {};
  for (const s of src.shafts) {
    const cfg = tvDraft.spec.shafts[s.id];
    const fw = (tvDraft.spec.addedFlywheels || []).filter(f => f.shaftId === s.id);
    const card = h('div', { class: 'tv-shaft-card' + (cfg.locked ? ' locked' : ''),
      title: '双击锁定/解锁（锁定轴不接受搜索飞轮）' }, wrap);
    const head = h('div', { class: 'bl-mesh-head' }, card);
    h('span', {}, head, `${s.name} · r=${(+s.ratio).toFixed(4)} · ${Math.round(s.rpm)} rpm` +
      (cfg.locked ? ' 🔒' : ''));
    h('span', { class: 'muted small' }, head,
      `齿坯估算 J≈${s.gearInertia.toExponential(2)}` +
      (fw.length ? ` · 含飞轮 +${fw.reduce((a, f) => a + f.inertia, 0).toExponential(2)}` : ''));
    const grid = h('div', { class: 'bl-mesh-grid' }, card);
    const lab = h('label', {}, grid, '转动惯量 J kg·m²');
    const inp = h('input', { type: 'number', step: '0.00001' }, lab);
    inp.value = cfg.inertia;
    inp.addEventListener('input', () => {
      const v = parseFloat(inp.value);
      if (v > 0) { cfg.inertia = v; tvSpecChanged(); }
    });
    card.addEventListener('dblclick', () => {
      cfg.locked = !cfg.locked;
      wrap.dataset.key = ''; tvSaveDraft(); tvRender();
      flashHint(cfg.locked ? `已锁定轴「${names[s.id] || s.id}」` : '已解锁该轴');
    });
  }
}

function tvSpringStructKey() {
  const sp = tvDraft.spec;
  return JSON.stringify(Object.keys(sp.meshes).sort()) + '|' +
    sp.couplings.map(c => c.id + ':' + c.kind + ':' + c.shaftId).join(',');
}

function tvRenderSprings() {
  const wrap = $('#tv-springs');
  const key = tvSpringStructKey();
  if (key === tvStructKey) return;
  tvStructKey = key;
  wrap.innerHTML = '';
  const src = tvPrimarySource();
  const mByName = Object.fromEntries(src.meshes.map(m => [m.id, m]));
  const sNames = src.shaftNames || {};
  for (const [mid, cfg] of Object.entries(tvDraft.spec.meshes)) {
    const m = mByName[mid];
    const card = h('div', { class: 'tv-spring-card' + (cfg.locked ? ' locked' : '') }, wrap);
    const head = h('div', { class: 'bl-mesh-head' }, card);
    h('span', {}, head, '啮合 ' + (m ? m.name : mid) + (cfg.locked ? ' 🔒' : ''));
    h('span', { class: 'muted small' }, head,
      m ? `z${m.zDriver}/z${m.zDriven} · 平均 T ${m.torqueNm} N·m` : '');
    tvSpringGrid(card, cfg, { k: 'stiffness', c: 'damping' }, tvSpecChanged);
    h('button', { class: cfg.locked ? '' : 'danger', onclick: () => {
      cfg.locked = !cfg.locked; tvStructKey = ''; tvSaveDraft(); tvRender();
    } }, card, cfg.locked ? '解锁（允许搜索）' : '锁定（不参与搜索）');
  }
  for (const cpl of tvDraft.spec.couplings) {
    const card = h('div', { class: 'tv-spring-card coupling' + (cpl.locked ? ' locked' : '') }, wrap);
    const head = h('div', { class: 'bl-mesh-head' }, card);
    h('span', {}, head, (cpl.kind === 'driver' ? '驱动端联轴器' :
      '负载端联轴器·' + (sNames[cpl.shaftId] || cpl.shaftId)) + (cpl.locked ? ' 🔒' : ''));
    h('span', { class: 'muted small' }, head, `端惯量 J ${cpl.inertia} kg·m²`);
    tvSpringGrid(card, cpl, { k: 'stiffness', c: 'damping', j: 'inertia' }, tvSpecChanged);
    h('button', { class: cpl.locked ? '' : 'danger', onclick: () => {
      cpl.locked = !cpl.locked; tvStructKey = ''; tvSaveDraft(); tvRender();
    } }, card, cpl.locked ? '解锁（允许搜索）' : '锁定（不参与搜索）');
  }
}

function tvSpringGrid(card, cfg, fields, onch) {
  const grid = h('div', { class: 'bl-mesh-grid' }, card);
  const mk = (key, label, step) => {
    const lab = h('label', {}, grid, label);
    const inp = h('input', { type: 'number', step: String(step) }, lab);
    inp.value = cfg[key];
    inp.addEventListener('input', () => {
      const v = parseFloat(inp.value);
      if (Number.isFinite(v) && (key === 'damping' ? v >= 0 : v > 0)) {
        cfg[key] = v; onch();
      }
    });
  };
  if (fields.j) mk('inertia', '端转动惯量 kg·m²', 0.00001);
  mk('stiffness', '扭转刚度 N·m/rad', 10);
  mk('damping', '扭转阻尼 N·m·s/rad', 0.001);
}

/* ---------------- 结果：汇总 / 问题 ---------------- */
function tvRenderResults() {
  tvRenderSummary();
  tvRenderIssues();
  tvRenderCampbell();
  tvRenderModeList();
  tvRenderModeShape();
  tvRenderShaftAnim();
  tvRenderSearchList();
}

function tvRenderSummary() {
  const box = $('#tv-summary'), badge = $('#tv-badge');
  if (!tvResult) { box.textContent = '正在计算…'; box.classList.add('muted'); badge.textContent = ''; return; }
  box.classList.remove('muted');
  const errs = (tvResult.issues || []).filter(i => i.severity === 'error');
  if (errs.length) {
    badge.textContent = '参数错误'; badge.className = 'badge err';
    box.innerHTML = errs.map(i => `<b>${i.message}</b>`).join('；');
    return;
  }
  const sm = tvResult.summary;
  const warns = (tvResult.issues || []).filter(i => i.severity === 'warning');
  badge.textContent = warns.length ? `${warns.length} 项告警` : '扭振正常';
  badge.className = 'badge ' + (warns.length ? 'warn' : '');
  box.innerHTML =
    `扫描 <b>${tvResult.rpms[0]}~${tvResult.rpms[tvResult.rpms.length - 1]} rpm</b>` +
    `（${tvResult.rpms.length} 样本）；弹性模态 <b>${sm.nModes}</b> 阶；` +
    `扫描范围内共振带 <b>${sm.nZones}</b> 个（其中动态转矩超限危险区 <b style="color:var(--err)">${sm.nResonanceZones}</b>）；` +
    `阻尼不足模态 <b>${sm.nUnderDamped}</b> 阶；动态转矩超限区间 <b>${sm.nOverLimit}</b> 段；` +
    `全列峰值总转矩 <b>${sm.maxTorque} N·m</b>，最大放大系数 <b>${sm.maxAmpFactor}×</b>`;
}

function tvRenderIssues() {
  const ul = $('#tv-issues');
  ul.innerHTML = '';
  if (!tvResult) return;
  const sevRank = { error: 0, warning: 1, info: 2 };
  const list = (tvResult.issues || []).slice()
    .sort((a, b) => sevRank[a.severity] - sevRank[b.severity]);
  if (!list.length) { h('li', { class: 'info' }, ul, '✓ 扫描范围内无危险共振穿越，阻尼充足，动态转矩未超限'); return; }
  for (const it of list) {
    const li = h('li', { class: it.severity }, ul);
    h('span', { class: 'sev' }, li,
      it.code === 'LOW_DAMPING' ? '⚠ 欠阻尼' : it.code === 'RESONANCE' ? '⚠ 共振' : '⚠ 超限');
    li.appendChild(document.createTextNode(it.message));
    li.addEventListener('click', () => tvLocate(it));
  }
}

function tvLocate(it) {
  const refs = it.refs || {};
  if (refs.rpm != null) {
    const k = tvRpmToIndex(refs.rpm);
    if (k >= 0) { tvCursor = k; tvRenderCampbell(); tvRenderShaftAnim(); }
  }
  if (refs.mode != null) { tvSelectedMode = refs.mode; tvRenderModeList(); tvRenderModeShape(); tvRenderCampbell(); }
}
function tvRpmToIndex(rpm) {
  const arr = tvResult.rpms;
  let best = 0, bd = Infinity;
  for (let i = 0; i < arr.length; i++) {
    const d = Math.abs(arr[i] - rpm);
    if (d < bd) { bd = d; best = i; }
  }
  return best;
}

/* ---------------- Campbell 图 ---------------- */
function tvCampbellGeom() {
  const W = 620, H = 240, ml = 42, mr = 12, mt = 12, mb = 30;
  const rpms = tvResult.rpms;
  const x0 = rpms[0], x1 = rpms[rpms.length - 1];
  // y 轴：激励频率与固有频率（Hz）；斜线 f=h·r·n/60
  const exNode = tvDraft.spec.excitation.nodeId;
  const exN = (tvResult.nodes || []).find(n => n.nodeId === exNode);
  const exR = Math.abs(exN ? exN.r : 1);
  let yMax = 0;
  for (const m of tvResult.modes) if (!m.rigid) yMax = Math.max(yMax, m.hz);
  for (const o of tvDraft.spec.excitation.orders)
    yMax = Math.max(yMax, o.h * exR * x1 / 60);
  yMax *= 1.08;
  return { W, H, ml, mr, mt, mb, x0, x1, yMax, exR,
    X: rpm => ml + (W - ml - mr) * (rpm - x0) / (x1 - x0),
    Y: f => mt + (H - mt - mb) * (1 - f / yMax) };
}

function tvRenderCampbell() {
  const svg = $('#tv-campbell');
  if (!tvDraft || !tvResult || !tvResult.rpms || !tvResult.rpms.length) { svg.innerHTML = ''; return; }
  const g = tvCampbellGeom();
  const { W, H, ml, mr, mt, mb, X, Y, x0, x1, yMax, exR } = g;
  svg.setAttribute('viewBox', `0 0 ${W} ${H}`);
  svg.innerHTML = '';
  svg._tvGeom = g;
  const layers = { zone: svgEl('g', {}, svg), grid: svgEl('g', {}, svg),
    line: svgEl('g', {}, svg), cross: svgEl('g', {}, svg),
    cursor: svgEl('g', {}, svg) };

  // 危险共振区（rpm 带背景）+ 各啮合动态转矩超限区间
  for (const z of tvResult.zones || []) {
    if (!z.danger) continue;
    svgEl('rect', { x: X(z.rpm0), y: mt, width: Math.max(2, X(z.rpm1) - X(z.rpm0)),
      height: H - mt - mb, fill: '#fdecea', opacity: 0.75 }, layers.zone);
  }
  for (const tc of tvResult.torqueCurves || []) {
    for (const [a, b] of (tc.spans || [])) {
      const aa = Math.max(x0, a), bb = Math.min(x1, b);
      if (bb <= aa) continue;
      svgEl('rect', { x: X(aa), y: H - mb - 14, width: Math.max(2, X(bb) - X(aa)),
        height: 6, fill: '#e08e45', opacity: 0.85 }, layers.zone);
    }
  }

  // 网格
  for (let i = 0; i <= 4; i++) {
    const f = yMax * i / 4, y = Y(f);
    svgEl('line', { x1: ml, y1: y, x2: W - mr, y2: y, stroke: '#e0d9cb', 'stroke-width': 0.5 }, layers.grid);
    svgEl('text', { x: ml - 3, y: y + 3, class: 'lc-tick', text: f.toFixed(0) }, layers.grid)
      .setAttribute('text-anchor', 'end');
  }
  for (let i = 0; i <= 5; i++) {
    const rpm = x0 + (x1 - x0) * i / 5;
    svgEl('text', { x: X(rpm), y: H - mb + 13, class: 'lc-tick', text: Math.round(rpm) }, layers.grid)
      .setAttribute('text-anchor', 'middle');
  }
  svgEl('text', { x: 6, y: 12, class: 'lc-tick', text: 'Hz' }, layers.grid);
  svgEl('text', { x: W - mr, y: H - 4, class: 'lc-tick', text: '输入转速 rpm' }, layers.grid)
    .setAttribute('text-anchor', 'end');

  // 固有频率横线（刚体模态不画）
  const colors = ['#3d5a80', '#2e7d32', '#7b5ea7', '#b06a00', '#5b8def', '#c62828'];
  tvResult.modes.forEach((m, i) => {
    if (m.rigid) return;
    const sel = tvSelectedMode === m.index;
    const under = m.zeta < tvDraft.spec.limits.zetaMin;
    svgEl('line', { x1: ml, y1: Y(m.hz), x2: W - mr, y2: Y(m.hz),
      stroke: under ? '#c62828' : colors[i % colors.length],
      'stroke-width': sel ? 2.2 : 1.0,
      'stroke-dasharray': under ? '6 3' : 'none', opacity: sel ? 1 : 0.8,
      class: 'tv-fn-line', 'data-mode': m.index,
      style: 'cursor:pointer' }, layers.line);
    const t = svgEl('text', { x: W - mr - 2, y: Y(m.hz) - 2, class: 'tv-fn-label',
      text: `f${m.index}=${m.hz}Hz ζ=${m.zeta.toFixed(3)}`, 'data-mode': m.index,
      style: 'cursor:pointer' }, layers.line);
    t.setAttribute('text-anchor', 'end');
  });
  layers.line.querySelectorAll('[data-mode]').forEach(el => el.addEventListener('click', () => {
    const mi = +el.getAttribute('data-mode');
    tvSelectedMode = tvSelectedMode === mi ? -1 : mi;
    tvRenderModeList(); tvRenderModeShape(); tvRenderCampbell();
  }));

  // 激励阶次斜线 f = h·r_ex·n/60（r_ex 是激励端相对输入轴的转速比绝对值）
  tvDraft.spec.excitation.orders.forEach((o, k) => {
    const fa = o.h * exR * x0 / 60, fb = o.h * exR * x1 / 60;
    svgEl('line', { x1: X(x0), y1: Y(fa), x2: X(x1), y2: Y(fb),
      stroke: '#e08e45', 'stroke-width': 1.2, 'stroke-dasharray': '5 3' }, layers.line);
    svgEl('text', { x: X(x1), y: Y(fb) - 2, class: 'tv-order-label',
      text: `${o.h}阶` }, layers.line).setAttribute('text-anchor', 'end');
  });

  // 穿越点
  for (const c of tvResult.crossings || []) {
    if (!c.inRange) continue;
    const f = c.hz;
    svgEl('circle', { cx: X(c.rpm), cy: Y(f), r: c.danger ? 4.2 : 3,
      fill: c.danger ? '#c62828' : (c.underdamped ? '#b26a00' : '#fff'),
      stroke: c.danger ? '#c62828' : '#666', 'stroke-width': 1.2 }, layers.cross);
  }

  // 游标
  const k = Math.max(0, Math.min(tvCursor, tvResult.rpms.length - 1));
  tvCursor = k;
  const cg = svgEl('g', { class: 'tv-cursor' }, layers.cursor);
  svgEl('line', { x1: 0, y1: mt, x2: 0, y2: H - mb, stroke: '#c62828', 'stroke-width': 1 }, cg);
  svgEl('rect', { x: -6, y: mt - 2, width: 12, height: 8, rx: 2, fill: '#c62828',
    cursor: 'ew-resize' }, cg);
  cg.setAttribute('transform', `translate(${X(tvResult.rpms[k])},0)`);
  // 游标与各阶斜线的交点（当前激励频率）
  for (const o of tvDraft.spec.excitation.orders) {
    const f = o.h * exR * tvResult.rpms[k] / 60;
    if (f <= yMax)
      svgEl('circle', { cx: 0, cy: Y(f), r: 2.6, fill: '#e08e45' }, cg);
  }
  tvUpdateCursorReadout();
}

/* Campbell 游标拖动（只绑一次） */
function tvBindCampbellDrag() {
  const svg = $('#tv-campbell');
  let dragging = false;
  const toK = clientX => {
    const gm = svg._tvGeom;
    const rect = svg.getBoundingClientRect();
    const vb = svg.viewBox.baseVal;
    const px = (clientX - rect.left) / rect.width * vb.width;
    const rpm = gm.x0 + (px - gm.ml) / (gm.W - gm.ml - gm.mr) * (gm.x1 - gm.x0);
    let lo = 0, hi = tvResult.rpms.length - 1, best = 0, bd = Infinity;
    tvResult.rpms.forEach((v, i) => { const d = Math.abs(v - rpm); if (d < bd) { bd = d; best = i; } });
    return best;
  };
  svg.addEventListener('pointerdown', e => {
    if (!tvResult || !svg._tvGeom) return;
    dragging = true; svg.setPointerCapture(e.pointerId);
    tvCursor = toK(e.clientX); tvRenderCampbell(); tvRenderShaftAnim();
  });
  svg.addEventListener('pointermove', e => {
    if (!dragging || !svg._tvGeom) return;
    tvCursor = toK(e.clientX);
    const gm = svg._tvGeom;
    const cg = svg.querySelector('.tv-cursor');
    if (cg) cg.setAttribute('transform',
      `translate(${gm.X(tvResult.rpms[tvCursor])},0)`);
    tvUpdateCursorReadout(); tvRenderShaftAnim();
  });
  const end = () => { dragging = false; };
  svg.addEventListener('pointerup', end);
  svg.addEventListener('pointercancel', end);
}

function tvUpdateCursorReadout() {
  const box = $('#tv-campbell-readout');
  if (!tvResult || !box) return;
  const k = Math.max(0, Math.min(tvCursor, tvResult.rpms.length - 1));
  const rpm = tvResult.rpms[k];
  const exNode = tvDraft.spec.excitation.nodeId;
  const exN = (tvResult.nodes || []).find(n => n.nodeId === exNode);
  const exR = Math.abs(exN ? exN.r : 1);
  const fs = tvDraft.spec.excitation.orders.map(o =>
    `${o.h}阶 ${(o.h * exR * rpm / 60).toFixed(1)}Hz`).join('，');
  // 最近的固有频率穿越
  let near = '';
  for (const m of tvResult.modes) {
    if (m.rigid) continue;
    for (const o of tvDraft.spec.excitation.orders) {
      const nc = 60 * m.hz / (o.h * exR);
      if (Math.abs(nc - rpm) / Math.max(nc, 1) < 0.03)
        near += `；靠近 f${m.index}×${o.h}阶穿越点 ${nc.toFixed(0)} rpm`;
    }
  }
  box.textContent = `n = ${rpm} rpm · 激励频率：${fs}${near}`;
  box.className = near ? 'tv-cursor-readout danger' : 'tv-cursor-readout';
}

/* ---------------- 模态列表与模态形状图 ---------------- */
function tvRenderModeList() {
  const ol = $('#tv-modes');
  ol.innerHTML = '';
  if (!tvResult) return;
  tvResult.modes.forEach(m => {
    if (m.rigid) return;
    const under = m.zeta < tvDraft.spec.limits.zetaMin;
    const li = h('li', { class: 'tv-mode-item' +
      (tvSelectedMode === m.index ? ' selected' : '') + (under ? ' under' : '') }, ol);
    li.addEventListener('click', () => {
      tvSelectedMode = tvSelectedMode === m.index ? -1 : m.index;
      tvRenderModeList(); tvRenderModeShape(); tvRenderCampbell();
    });
    h('span', { class: 'tv-mode-name' }, li,
      `第 ${m.index} 阶 · ${m.hz} Hz` + (under ? ' · ζ不足' : ''));
    h('span', { class: under ? 'lc-err' : 'muted small' }, li,
      `ζ = ${m.zeta.toFixed(4)}` + (under ? ` < ${tvDraft.spec.limits.zetaMin}` : ''));
  });
}

function tvRenderModeShape() {
  const svg = $('#tv-mode-svg');
  svg.innerHTML = '';
  if (!tvResult || tvSelectedMode < 0) {
    svg.setAttribute('viewBox', '0 0 620 30');
    svgEl('text', { x: 310, y: 18, class: 'lc-tick',
      text: tvResult ? '点击上方模态查看振型（物理轴角，相对幅归一）' : '' }, svg)
      .setAttribute('text-anchor', 'middle');
    return;
  }
  const m = tvResult.modes.find(x => x.index === tvSelectedMode);
  if (!m) return;
  const nodes = tvResult.nodes;
  const W = 620, H = 170, ml = 30, mr = 12, mt = 18, mb = 40;
  svg.setAttribute('viewBox', `0 0 ${W} ${H}`);
  const x = i => ml + (W - ml - mr) * (i + 0.5) / nodes.length;
  const y0 = (mt + H - mb) / 2;
  const scale = Math.min(46, (H - mt - mb) / 2 - 8);
  // 零轴
  svgEl('line', { x1: ml, y1: y0, x2: W - mr, y2: y0, stroke: '#c9c0b1', 'stroke-width': 0.8 }, svg);
  const pts = m.shape.map((v, i) => [x(i), y0 - v * scale]);
  const d = pts.map((p, i) => `${i ? 'L' : 'M'}${p[0].toFixed(1)},${p[1].toFixed(1)}`).join('');
  svgEl('path', { d, fill: 'none', stroke: '#3d5a80', 'stroke-width': 1.6 }, svg);
  nodes.forEach((n, i) => {
    const kindColor = n.kind === 'driver' ? '#2e7d32' : n.kind === 'load' ? '#b08968' : '#5b8def';
    svgEl('circle', { cx: pts[i][0], cy: pts[i][1], r: 4, fill: kindColor,
      stroke: '#fffdf8', 'stroke-width': 1 }, svg);
    const lbl = (n.kind === 'shaft' ? n.name : (n.kind === 'driver' ? '驱动端' : '负载端'))
      .slice(0, 6);
    svgEl('text', { x: pts[i][0], y: H - mb + 14, class: 'lc-tick', text: lbl }, svg)
      .setAttribute('text-anchor', 'middle');
    svgEl('text', { x: pts[i][0], y: pts[i][1] + (m.shape[i] >= 0 ? -7 : 13),
      class: 'tv-mode-val', text: m.shape[i].toFixed(2) }, svg)
      .setAttribute('text-anchor', 'middle');
  });
  svgEl('text', { x: 6, y: 12, class: 'lc-tick',
    text: `f${m.index}=${m.hz} Hz，ζ=${m.zeta.toFixed(4)}（节点按动力路径排列）` }, svg);
}

/* ---------------- 轴系扭转动画 ---------------- */
function tvNodeX(i, n, ml, mr, W) {
  return ml + (W - ml - mr) * (i + 0.5) / n;
}

function tvRenderShaftAnim() {
  const svg = $('#tv-shaft-svg');
  if (!tvDraft || !tvResult || !tvResult.rpms || !tvResult.rpms.length) { svg.innerHTML = ''; return; }
  const nodes = tvResult.nodes, edges = tvResult.edges;
  const k = Math.max(0, Math.min(tvCursor, tvResult.rpms.length - 1));
  const W = 620, H = 190, ml = 34, mr = 20, cy = 92;
  svg.setAttribute('viewBox', `0 0 ${W} ${H}`);
  svg.innerHTML = '';
  const xs = nodes.map((_, i) => tvNodeX(i, nodes.length, ml, mr, W));
  const ampMax = Math.max(1e-9, ...nodes.map(n => Math.max(...(n.amp || [0]))));
  const phase = (n) => {
    // 后端 orders 顺序与 spec.excitation.orders 一致；取第一阶的节点相位
    const od = tvResult.orders[0];
    return od ? od.nodePhase[nodes.indexOf(n)][k] : 0;
  };
  const amp = n => (n.amp || [0])[k] || 0;

  // 边（弹簧）：颜色按该转速下动态转矩相对平均/限值
  const edgeById = Object.fromEntries(edges.map(e => [e.id, e]));
  const xOf = id => {
    const i = nodes.findIndex(n => n.nodeId === id);
    return i >= 0 ? xs[i] : null;
  };
  for (const tc of tvResult.torqueCurves) {
    const e = edgeById['m:' + tc.meshId];
    if (!e) continue;
    const x1 = xOf(e.a), x2 = xOf(e.b);
    if (x1 == null || x2 == null) continue;
    const total = tc.total[k], lim = tc.limit;
    const over = total > lim;
    svgEl('line', { x1, y1: cy, x2, y2: cy,
      stroke: over ? '#c62828' : '#8d9bb3', 'stroke-width': over ? 3 : 2,
      'stroke-dasharray': '5 3' }, svg);
    svgEl('text', { x: (x1 + x2) / 2, y: cy - 8, class: 'tv-edge-label' + (over ? ' over' : ''),
      text: `${tc.total[k].toFixed(2)} / ${tc.mean} N·m` }, svg)
      .setAttribute('text-anchor', 'middle');
  }
  for (const cc of tvResult.couplingCurves || []) {
    const e = edgeById[cc.edgeId];
    if (!e) continue;
    const x1 = xOf(e.a), x2 = xOf(e.b);
    if (x1 == null || x2 == null) continue;
    svgEl('line', { x1, y1: cy, x2, y2: cy, stroke: '#b08968', 'stroke-width': 2 }, svg);
    svgEl('text', { x: (x1 + x2) / 2, y: cy + 16, class: 'muted', 'font-size': 8,
      text: `联轴器 Tdyn ${cc.dyn[k].toFixed(2)}` }, svg).setAttribute('text-anchor', 'middle');
  }

  // 节点圆盘 + 扭转角红弧
  nodes.forEach((n, i) => {
    const a = amp(n), ph = (phase(n) * Math.PI / 180);
    const rMag = 16 + 10 * Math.sqrt(Math.min(1, a / ampMax));
    const color = n.kind === 'driver' ? '#2e7d32' : n.kind === 'load' ? '#b08968'
      : (tvDraft.spec.shafts[n.shaftId] || {}).locked ? '#b26a00' : '#5b8def';
    const g = svgEl('g', { transform: `translate(${xs[i]},${cy})` }, svg);
    svgEl('circle', { r: rMag, fill: '#fffdf8', stroke: color, 'stroke-width': 1.6 }, g);
    // 扭角弧（按振幅缩放，固定相位读数）
    const ang = Math.max(4, Math.min(150, a / ampMax * 90));
    const arcR = rMag + 5;
    const a0 = ph - ang * Math.PI / 360, a1 = ph + ang * Math.PI / 360;
    const P = aa => [arcR * Math.sin(aa), -arcR * Math.cos(aa)];
    const p0 = P(a0), p1 = P(a1);
    svgEl('path', {
      d: `M ${p0[0].toFixed(1)} ${p0[1].toFixed(1)} A ${arcR} ${arcR} 0 0 1 ${p1[0].toFixed(1)} ${p1[1].toFixed(1)}`,
      fill: 'none', stroke: '#c62828', 'stroke-width': 1.6 }, g);
    // 基准刻线 + 振荡刻线
    svgEl('line', { x1: 0, y1: -rMag + 2, x2: 0, y2: rMag - 2, stroke: '#c9c0b1', 'stroke-width': 0.7 }, g);
    svgEl('line', { x1: arcR * Math.sin(ph), y1: -arcR * Math.cos(ph),
      x2: (rMag - 3) * Math.sin(ph), y2: -(rMag - 3) * Math.cos(ph),
      stroke: '#c62828', 'stroke-width': 1.2 }, g);
    const lbl = (n.kind === 'shaft' ? n.name : n.kind === 'driver' ? '驱动端' : '负载端').slice(0, 6);
    svgEl('text', { x: 0, y: rMag + 14, class: 'tv-node-label', text: lbl }, g)
      .setAttribute('text-anchor', 'middle');
    svgEl('text', { x: 0, y: 3, class: 'tv-node-amp',
      text: a >= 0.001 ? a.toExponential(1) : a.toFixed(4) }, g)
      .setAttribute('text-anchor', 'middle');
  });
  svgEl('text', { x: 6, y: 14, class: 'lc-tick',
    text: `红弧＝该节点扭振角幅（相对最大值缩放），盘内为弧度幅值；弹簧红色＝动态转矩超限` }, svg);
  tvRenderCursorDetail(k);
}

function tvRenderCursorDetail(k) {
  const box = $('#tv-cursor-detail');
  if (!tvResult) return;
  const od = tvResult.orders[0];
  const lines = [];
  tvResult.nodes.forEach((n, i) => {
    const a = od.nodeAmp[i][k];
    const ph = od.nodePhase[i][k];
    lines.push(`${n.kind === 'shaft' ? n.name : n.name}：幅 ${a.toExponential(2)} rad，相位 ${ph.toFixed(0)}°`);
  });
  for (const tc of tvResult.torqueCurves) {
    lines.push(`啮合「${tc.name}」：动态 ${tc.dyn[k].toFixed(3)}，平均 ${tc.mean}，` +
      `总 ${tc.total[k].toFixed(3)} N·m，放大 ${tc.ampFactor[k] == null ? '—' : tc.ampFactor[k] + '×'}`);
  }
  box.innerHTML = lines.map(t => `<div>${t}</div>`).join('');
}

/* ---------------- 搜索 ---------------- */
function tvRenderSearchShafts() {
  if (!tvDraft) return;
  const wrap = $('#tv-fw-shafts');
  const src = tvPrimarySource();
  const key = src.shafts.map(s => s.id + ':' + (tvDraft.spec.shafts[s.id].locked ? 1 : 0)).join('|');
  if (wrap.dataset.key === key) return;
  wrap.dataset.key = key;
  wrap.innerHTML = '';
  for (const s of src.shafts) {
    const locked = tvDraft.spec.shafts[s.id].locked;
    const lab = h('label', { class: locked ? 'tv-fw-locked' : '' }, wrap,
      (locked ? '🔒 ' : '') + s.name);
    if (!locked) {
      const cb = h('input', { type: 'checkbox', value: s.id }, lab);
      cb.checked = true;
      lab.insertBefore(cb, lab.firstChild);
    }
  }
}

async function tvRunSearch() {
  if (!tvDraft) return;
  const shafts = [...document.querySelectorAll('#tv-fw-shafts input:checked')].map(c => c.value);
  const parse = v => String(v).split(',').map(x => parseFloat(x.trim()))
    .filter(x => Number.isFinite(x) && x > 0);
  const body = {
    spec: tvDraft.spec, frozen: tvDraft.frozen,
    flywheelShafts: shafts,
    flywheelJ: parse($('#tv-fw-j').value),
    kMult: parse($('#tv-k-mult').value),
    cMult: parse($('#tv-c-mult').value),
    limit: 12, timeBudget: 20,
  };
  $('#tvs-meta').textContent = '搜索中…';
  const resp = await fetch('/api/torsional/search', {
    method: 'POST', headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(body),
  });
  tvSearchRes = await resp.json();
  tvRenderSearchList();
}

function tvRenderSearchList() {
  const list = $('#tvs-list'), meta = $('#tvs-meta');
  if (!list) return;
  list.innerHTML = '';
  if (!tvSearchRes) return;
  const r = tvSearchRes;
  meta.textContent = `枚举 ${r.nodes} 个组合${r.truncated ? '（达到时限截断）' : ''}；` +
    `当前：危险共振区 ${r.base.nResonanceZones} · 超限段 ${r.base.nOverLimit} · ` +
    `峰值总转矩 ${r.base.maxTorque} N·m（${r.base.maxAmpFactor}×）`;
  (r.results || []).forEach((c, i) => {
    const li = h('li', { class: 'cand lc-cand' + (c.current ? ' current-cand' : '') }, list);
    const head = h('div', { class: 'head' }, li);
    const fwTxt = c.flywheelJ > 0
      ? `飞轮 +${c.flywheelJ} kg·m² 于「${(tvPrimarySource().shaftNames || {})[c.flywheelShaft] || c.flywheelShaft}」`
      : '不加飞轮';
    h('span', {}, head, `#${i + 1} ${fwTxt} · k×${c.kMult} · c×${c.cMult}` +
      (c.current ? '（当前）' : ''));
    h('span', { class: 'muted small' }, head, `改动量 ${c.change}`);
    h('div', { class: 'meta' }, li,
      `危险共振区 ${c.nResonanceZones} · 超限段 ${c.nOverLimit} · 欠阻尼 ${c.nUnderDamped} 阶 · ` +
      `峰值总转矩 ${c.maxTorque} N·m（${c.maxAmpFactor}×） · 附加惯量 ${c.addedJ}`);
    const acts = h('div', { class: 'actions' }, li);
    h('button', { class: 'primary', onclick: () => tvApplyCandidate(c) }, acts, '采用并另存版本');
  });
}

async function tvApplyCandidate(c) {
  const sp = tvDraft.spec;
  // 联轴器刚度/阻尼（只改未锁定的，后端 patch 也只含未锁定项）
  for (const [cid, p] of Object.entries(c.patch.couplings || {})) {
    const cpl = sp.couplings.find(x => x.id === cid);
    if (cpl && !cpl.locked) { cpl.stiffness = p.stiffness; cpl.damping = p.damping; }
  }
  // 飞轮：惯量直接并入所在轴（锁定轴后端已排除），并记录到 addedFlywheels
  if (c.patch.flywheelJ > 0 && c.patch.flywheelShaft) {
    const sid = c.patch.flywheelShaft;
    if (sp.shafts[sid] && !sp.shafts[sid].locked) {
      sp.shafts[sid].inertia += c.patch.flywheelJ;
      sp.addedFlywheels = sp.addedFlywheels || [];
      sp.addedFlywheels.push({ shaftId: sid, inertia: c.patch.flywheelJ });
    }
  }
  tvSaveDraft();
  await tvAnalyze();           // 等待采用方案的完整解
  await tvSaveVersion(true);   // 冻结输入、计算结果与曲线
  tvStructKey = '';
  tvRender();
  flashHint('已采用避振方案并另存扭振版本（输入/结果/曲线已冻结）');
}

/* ---------------- 版本 ---------------- */
async function tvLoadVersions() {
  try { tvVersions = await (await fetch('/api/torsional/cases')).json(); }
  catch (e) { tvVersions = []; }
  const sel = $('#tv-versions');
  const cur = sel.value;
  sel.innerHTML = '';
  if (!tvVersions.length) h('option', { value: '' }, sel, '— 尚无已存版本 —');
  const curFp = state ? tvTrainFingerprint(state) : null;
  for (const v of tvVersions) {
    // 版本过期：已保存载荷版本不可变，仅当当前项目轮系与其冻结快照不一致
    const stale = curFp && v.train_fp && v.train_fp !== curFp;
    const o = h('option', { value: v.id }, sel,
      `${v.name} · v${v.version}${v.note ? `（${v.note}）` : ''}${stale ? ' ⚠过期' : ''}`);
    if (String(v.id) === cur) o.selected = true;
  }
}

async function tvSaveVersion(fromCandidate) {
  if (!tvDraft || !tvResult) return;
  const resp = await fetch('/api/torsional/cases', {
    method: 'POST', headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({
      name: tvDraft.name || '扭振工况',
      spec: tvDraft.spec, frozen: tvDraft.frozen,
      solution: tvResult, note: null,
    }),
  });
  const data = await resp.json();
  if (data.ok) {
    flashHint(`已另存扭振版本 v${data.version}（冻结输入、扫描结果与全部曲线）`);
    tvLoadVersions();
  }
}

/* ---------------- 事件绑定 ---------------- */
function tvBind() {
  tvBindCampbellDrag();
  $('#btn-tv-new').addEventListener('click', tvNewDraft);
  $('#btn-tv-refreeze').addEventListener('click', tvRefreeze);
  $('#tv-name').addEventListener('input', e => {
    if (tvDraft) { tvDraft.name = e.target.value; tvSaveDraft(); }
  });
  const num = (sel, fn) => $(sel).addEventListener('input', e => {
    if (!tvDraft) return;
    const v = parseFloat(e.target.value);
    if (Number.isFinite(v)) { fn(tvDraft.spec, v); tvSpecChanged(); }
  });
  num('#tv-rpm-min', (sp, v) => { sp.scan.rpmMin = v; });
  num('#tv-rpm-max', (sp, v) => { sp.scan.rpmMax = v; });
  num('#tv-rpm-step', (sp, v) => { sp.scan.rpmStep = Math.max(1, v); });
  num('#tv-zeta-min', (sp, v) => { sp.limits.zetaMin = Math.max(0, v); });
  num('#tv-tfactor', (sp, v) => { sp.limits.torqueFactor = Math.max(1, v); });
  $('#tv-ex-node').addEventListener('change', e => {
    if (tvDraft) { tvDraft.spec.excitation.nodeId = e.target.value; tvSpecChanged(); }
  });
  $('#btn-tv-add-order').addEventListener('click', () => {
    if (!tvDraft) return;
    const hs = tvDraft.spec.excitation.orders.map(o => o.h);
    let h = 1;
    while (hs.includes(h)) h += 1;
    tvDraft.spec.excitation.orders.push({ h, amp: 0 });
    $('#tv-orders').dataset.n = '';
    tvSpecChanged(); tvRender();
  });
  $('#btn-tv-search').addEventListener('click', tvRunSearch);
  $('#btn-tv-save').addEventListener('click', () => tvSaveVersion(false));
  $('#btn-tv-reload-draft').addEventListener('click', async () => {
    const d = await tvLoadServerDraft();
    if (!d) { flashHint('服务端还没有保存的扭振草稿'); return; }
    tvDraft = d;
    tvReset();
    try { localStorage.setItem(TV_KEY, JSON.stringify(d)); } catch (e) { /* ignore */ }
    tvAnalyzeSoon(); tvRender(); tvLoadVersions();
    flashHint('已从服务端恢复扭振草稿');
  });
  $('#btn-tv-load').addEventListener('click', async () => {
    const id = $('#tv-versions').value;
    if (!id) return;
    const v = await (await fetch(`/api/torsional/cases/${id}`)).json();
    if (!v.spec) return;
    tvDraft = { name: v.name, spec: v.spec, frozen: v.frozen };
    tvReset();
    // 已采用版本优先展示冻结的解，再异步复核
    if (v.solution) { tvResult = v.solution; }
    tvSaveDraft();
    if (!v.solution) tvAnalyzeSoon();
    else tvAnalyze();
    tvRender();
    flashHint(`已载入扭振「${v.name}」v${v.version}（输入/结果/曲线已冻结，来源变化仅标记过期）`);
  });
  $('#btn-tv-del').addEventListener('click', async () => {
    const id = $('#tv-versions').value;
    if (!id) return;
    await fetch(`/api/torsional/cases/${id}`, { method: 'DELETE' });
    tvLoadVersions();
  });
}

async function tvLoadServerDraft() {
  try {
    const d = await (await fetch('/api/torsional/draft')).json();
    if (d && d.draft && d.draft.spec) return d.draft;
  } catch (e) { /* 退到本地 */ }
  return null;
}

(async function tvInit() {
  tvBind();
  tvDraftLoading = true;
  const server = await tvLoadServerDraft();
  if (server) {
    tvDraft = server;
    try { localStorage.setItem(TV_KEY, JSON.stringify(server)); } catch (e) { /* ignore */ }
  } else {
    try {
      const raw = localStorage.getItem(TV_KEY);
      if (raw) tvDraft = JSON.parse(raw);
    } catch (e) { tvDraft = null; }
    if (tvDraft) tvPushDraft();
  }
  tvDraftLoading = false;
  tvRender();
  if (tvDraft) tvAnalyze();
  tvLoadVersions();
})();
