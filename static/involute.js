/* 渐开线啮合纯几何（无 DOM 依赖）：与 meshing.py 同公式，供前端诊断与接触点动画。
   在浏览器中挂到 window.Involute；在 node 中挂到 globalThis（供一致性测试）。 */
(function (root) {
  'use strict';

  function invOf(a) { return Math.tan(a) - a; }

  function solveInv(target, guess) {
    let a = Math.min(1.4, Math.max(1e-4, guess || 0.35));
    for (let i = 0; i < 40; i++) {
      const t = Math.tan(a);
      const f = t - a - target;
      const d = t * t;
      if (d < 1e-12) break;
      let na = a - f / d;
      if (na <= 1e-6) na = a * 0.5;
      else if (na >= 1.5) na = (a + 1.5) * 0.5;
      a = na;
      if (Math.abs(f) < 1e-13) break;
    }
    return a;
  }

  /* 考虑变位和的理论（无侧隙）中心距；参数非法返回 null，变位和不可解退回标准中心距。
     与 meshing.expected_center 保持一致。 */
  function expectedCenter(ga, gb) {
    const m = +ga.module;
    const alpha = ((+ga.pressureAngle) || 20) * Math.PI / 180;
    if (!(m > 0) || !Number.isInteger(ga.z) || !Number.isInteger(gb.z)) return null;
    const internal = !!(ga.internal || gb.internal);
    if (ga.internal && gb.internal) return null;
    let zsum, xsum;
    if (internal) {
      const ring = ga.internal ? ga : gb, pin = ga.internal ? gb : ga;
      zsum = ring.z - pin.z;
      xsum = (+ring.x || 0) - (+pin.x || 0);
    } else {
      zsum = ga.z + gb.z;
      xsum = (+ga.x || 0) + (+gb.x || 0);
    }
    if (zsum <= 0) return null;
    const a0 = m * zsum / 2;
    const invW = invOf(alpha) + 2 * Math.tan(alpha) * xsum / zsum;
    if (invW <= 1e-12) return a0;
    const aw = solveInv(invW, alpha);
    return a0 * Math.cos(alpha) / Math.cos(aw);
  }

  /* 当前转角 phi（弧度，齿轮 g1 自转）下，作用线段 [sStart, sEnd] 内的接触点 s 坐标列表。
     接触点随转角沿作用线移动，每隔基圆节距 pe 重复。 */
  function contactPositions(view, phi) {
    const sStart = view.sStart, sEnd = view.sEnd, pe = view.pe, rb = view.rbDrive;
    if (!(pe > 0) || !(sEnd > sStart)) return [];
    let s0 = (rb * phi - sStart) % pe;
    s0 = ((s0 % pe) + pe) % pe;
    const out = [];
    for (let s = sStart + s0; s <= sEnd + 1e-9; s += pe) out.push(s);
    return out;
  }

  /* 把作用线段按同时承载对数分区（用于着色：1 对 / 2 对 / 3 对）。 */
  function contactZones(view) {
    const sStart = view.sStart, sEnd = view.sEnd, pe = view.pe;
    if (!(pe > 0) || !(sEnd > sStart)) return [];
    const cuts = new Set([sStart, sEnd]);
    for (let k = 1; k < 32; k++) {
      const a = sStart + k * pe, b = sEnd - k * pe;
      let more = false;
      if (a < sEnd - 1e-12) { cuts.add(a); more = true; }
      if (b > sStart + 1e-12) { cuts.add(b); more = true; }
      if (!more) break;
    }
    const pts = [...cuts].sort((x, y) => x - y);
    const zones = [];
    for (let i = 0; i + 1 < pts.length; i++) {
      const mid = (pts[i] + pts[i + 1]) / 2;
      let count = 0;
      for (let k = -16; k <= 16; k++) {
        const s = mid + k * pe;
        if (s >= sStart - 1e-9 && s <= sEnd + 1e-9) count++;
      }
      zones.push({ s0: pts[i], s1: pts[i + 1], count });
    }
    return zones;
  }

  root.Involute = { invOf, solveInv, expectedCenter, contactPositions, contactZones };
})(typeof window !== 'undefined' ? window : globalThis);
