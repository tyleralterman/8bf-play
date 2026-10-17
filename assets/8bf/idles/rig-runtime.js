// rig-runtime.js — the free lane of the sprite-studio bake-off.
// A small skeletal-animation player for rigspec files written by studio/engine/rigkit.py: bone hierarchy with
// Spine-compatible math (y up, degrees CCW), weighted meshes, region attachments, two-bone IK with softness,
// bezier/linear/stepped keys, crossfades between animations, events, and simple springs for secondary motion.
// It knows nothing about Spine's code; it only follows the same documented conventions so the same rig data can be
// compared fairly against Spine's official runtime.
(function (root) {
  'use strict';
  const DEG = Math.PI / 180;
  const cosd = (d) => Math.cos(d * DEG), sind = (d) => Math.sin(d * DEG);

  // ---------------------------------------------------------------- curves
  function bezierY(t, t0, v0, cx1, cy1, cx2, cy2, t1, v1) {
    // Solve x(s) = t on the cubic (t0,cx1,cx2,t1), return y(s) on (v0,cy1,cy2,v1).
    let lo = 0, hi = 1, s = (t - t0) / ((t1 - t0) || 1e-9);
    for (let i = 0; i < 24; i++) {
      const u = 1 - s;
      const x = u * u * u * t0 + 3 * u * u * s * cx1 + 3 * u * s * s * cx2 + s * s * s * t1;
      if (Math.abs(x - t) < 1e-7) break;
      if (x < t) lo = s; else hi = s;
      s = (lo + hi) / 2;
    }
    const u = 1 - s;
    return u * u * u * v0 + 3 * u * u * s * cy1 + 3 * u * s * s * cy2 + s * s * s * v1;
  }

  // Sample a timeline (array of keys with named value fields) at time t. Returns array of values.
  function sample(keys, t, fields) {
    const n = keys.length;
    if (t <= keys[0].time) return fields.map((f) => keys[0][f] ?? 0);
    if (t >= keys[n - 1].time) return fields.map((f) => keys[n - 1][f] ?? 0);
    let i = 0;
    while (i < n - 2 && keys[i + 1].time <= t) i++;
    const a = keys[i], b = keys[i + 1];
    if (a.curve === 'stepped') return fields.map((f) => a[f] ?? 0);
    const out = [];
    for (let k = 0; k < fields.length; k++) {
      const f = fields[k], v0 = a[f] ?? 0, v1 = b[f] ?? 0;
      if (Array.isArray(a.curve)) {
        const c = a.curve;
        out.push(bezierY(t, a.time, v0, c[k * 4], c[k * 4 + 1], c[k * 4 + 2], c[k * 4 + 3], b.time, v1));
      } else {
        out.push(v0 + (v1 - v0) * ((t - a.time) / ((b.time - a.time) || 1e-9)));
      }
    }
    return out;
  }

  // ---------------------------------------------------------------- skeleton
  class Bone {
    constructor(d, parent) {
      this.data = d; this.parent = parent; this.name = d.name;
      this.setup();
      this.a = 1; this.b = 0; this.c = 0; this.d = 1; this.worldX = 0; this.worldY = 0;
    }
    setup() { const d = this.data; this.x = d.x || 0; this.y = d.y || 0; this.rotation = d.rotation || 0; this.scaleX = d.scaleX ?? 1; this.scaleY = d.scaleY ?? 1; }
    update() {
      const r = this.rotation, sx = this.scaleX, sy = this.scaleY;
      const la = cosd(r) * sx, lb = cosd(r + 90) * sy, lc = sind(r) * sx, ld = sind(r + 90) * sy;
      const p = this.parent;
      if (!p) { this.a = la; this.b = lb; this.c = lc; this.d = ld; this.worldX = this.x; this.worldY = this.y; return; }
      this.worldX = p.a * this.x + p.b * this.y + p.worldX;
      this.worldY = p.c * this.x + p.d * this.y + p.worldY;
      this.a = p.a * la + p.b * lc; this.b = p.a * lb + p.b * ld;
      this.c = p.c * la + p.d * lc; this.d = p.c * lb + p.d * ld;
    }
    worldRotation() { return Math.atan2(this.c, this.a) / DEG; }
    worldToLocalRotation(worldRot) { // bone-local rotation that yields the given world rotation
      const p = this.parent; if (!p) return worldRot;
      const pr = Math.atan2(p.c, p.a) / DEG; return worldRot - pr;
    }
  }

  class Skeleton {
    constructor(spec) {
      this.spec = spec;
      this.bones = []; this.boneMap = {};
      for (const d of spec.bones) {
        const b = new Bone(d, d.parent ? this.boneMap[d.parent] : null);
        this.bones.push(b); this.boneMap[d.name] = b;
      }
      this.slots = spec.slots.map((s) => ({ data: s, name: s.name, bone: this.boneMap[s.bone], attachmentName: s.attachment, r: 1, g: 1, b: 1, a: 1 }));
      this.slotMap = {}; for (const s of this.slots) this.slotMap[s.name] = s;
      this.ik = (spec.ik || []).map((c) => ({ data: c, bones: c.bones.map((n) => this.boneMap[n]), target: this.boneMap[c.target], mix: c.mix ?? 1, softness: c.softness || 0, bend: c.bendPositive ? 1 : -1, stretch: !!c.stretch, compress: !!c.compress }));
      this.springs = (spec.physics || []).map((p) => ({ data: p, bone: this.boneMap[p.bone], angle: 0, vel: 0, px: null, py: null, vx: 0, vy: 0 }));
      this.x = 0; this.y = 0; this.scale = 1;
    }
    setToSetupPose() {
      for (const b of this.bones) b.setup();
      for (const s of this.slots) { s.attachmentName = s.data.attachment; s.r = s.g = s.b = s.a = 1; }
      for (const c of this.ik) c.mix = c.data.mix ?? 1;
    }
    updateWorld(dt) {
      for (const b of this.bones) b.update();
      for (const c of this.ik) this.applyIK(c);
      if (this.springs.length) this.applySprings(dt);
    }
    // Two-bone IK in the spirit of Spine's (law of cosines, bend direction, softness). One-bone IK aims the bone.
    applyIK(c) {
      if (c.mix <= 0) return;
      const t = c.target;
      if (c.bones.length === 1) {
        const b = c.bones[0];
        const want = Math.atan2(t.worldY - b.worldY, t.worldX - b.worldX) / DEG;
        const cur = b.worldRotation();
        b.rotation += shortest(want - cur) * c.mix;
        const L = b.data.length;
        if (L > 0 && (c.stretch || c.compress)) { // scale along the bone to reach the target exactly
          const d = Math.hypot(t.worldX - b.worldX, t.worldY - b.worldY) / (Math.hypot(b.a, b.c) || 1) * b.scaleX;
          if ((c.compress && d < L) || (c.stretch && d > L)) b.scaleX *= 1 + (d / L - 1) * c.mix;
        }
        this.updateFrom(b);
        return;
      }
      const [b1, b2] = c.bones;
      const l1 = b1.data.length, l2 = b2.data.length;
      let dx = t.worldX - b1.worldX, dy = t.worldY - b1.worldY;
      let dist = Math.hypot(dx, dy) || 1e-6;
      if (c.softness > 0) { // Spine-style soft approach to full extension (C1 continuous)
        const sd = dist - (l1 + l2) + c.softness;
        if (sd > 0) { const p = Math.min(1, sd / (c.softness * 2)) - 1; dist -= sd - c.softness * (1 - p * p); }
      }
      dist = Math.min(dist, l1 + l2 - 1e-6);
      const cosA = Math.max(-1, Math.min(1, (l1 * l1 + dist * dist - l2 * l2) / (2 * l1 * dist)));
      const base = Math.atan2(dy, dx) / DEG;
      const a1 = base - c.bend * Math.acos(cosA) / DEG; // bendPositive → joint on the clockwise side (Spine convention)
      const jx = b1.worldX + l1 * cosd(a1), jy = b1.worldY + l1 * sind(a1);
      const a2 = Math.atan2(t.worldY - jy, t.worldX - jx) / DEG;
      b1.rotation += shortest(a1 - b1.worldRotation()) * c.mix; this.updateFrom(b1);
      b2.rotation += shortest(a2 - b2.worldRotation()) * c.mix; this.updateFrom(b2);
    }
    updateFrom(b) { // update b and all descendants
      b.update();
      for (const o of this.bones) if (o !== b && isDescendant(o, b)) o.update();
    }
    // Springs (our own model, not Spine's): a follower point chases the bone's animated tip on a damped spring
    // (frequency in Hz, damping ratio zeta), and the bone turns toward the follower. It lags when the body moves,
    // overshoots when it stops, and settles, which is secondary motion. With no time passing (scrubbing), the last
    // offsets hold.
    applySprings(dt) {
      const h = 1 / 120;
      for (const s of this.springs) {
        const b = s.bone, p = s.data, L = b.data.length || 20;
        const ox = b.worldX, oy = b.worldY, tx = ox + b.a * L, ty = oy + b.c * L;
        if (s.px === null) { s.px = tx; s.py = ty; s.vx = 0; s.vy = 0; s.acc = 0; s.off = 0; }
        if (dt > 0) {
          const w = 2 * Math.PI * (p.freq ?? 2.2), k = w * w, c = 2 * (p.zeta ?? 0.3) * w;
          s.acc = Math.min(s.acc + dt, 0.1);
          while (s.acc >= h) {
            s.acc -= h;
            s.vx += (k * (tx - s.px) - c * s.vx) * h; s.vy += (k * (ty - s.py) - c * s.vy) * h;
            s.px += s.vx * h; s.py += s.vy * h;
          }
          const lim = p.limitDeg ?? 40;
          // a soft limit: a hard clamp stops the swing dead in one frame (a 40°/frame whip); tanh eases into it
          s.off = lim * Math.tanh(shortest(Math.atan2(s.py - oy, s.px - ox) / DEG - Math.atan2(ty - oy, tx - ox) / DEG) / lim);
        }
        b.rotation += s.off * (p.mix ?? 1);
        this.updateFrom(b);
      }
    }
  }
  function isDescendant(o, b) { for (let p = o.parent; p; p = p.parent) if (p === b) return true; return false; }
  function shortest(d) { d %= 360; if (d > 180) d -= 360; if (d < -180) d += 360; return d; }

  // ---------------------------------------------------------------- animation
  // Which properties an animation keys, the way Spine groups them: translate/translatex/translatey share x and y.
  const PROP_IDS = { rotate: ['r'], translate: ['x', 'y'], translatex: ['x'], translatey: ['y'], scale: ['sx', 'sy'], scalex: ['sx'], scaley: ['sy'] };
  function keyedProps(anim) {
    if (anim._keyed) return anim._keyed;
    const k = new Set();
    for (const bn in anim.bones) for (const tl in anim.bones[bn]) for (const id of PROP_IDS[tl] || [tl]) k.add(bn + '.' + id);
    for (const sn in anim.slots) for (const tl in anim.slots[sn]) k.add('slot:' + sn + '.' + tl);
    for (const cn in anim.ik || {}) k.add('ik:' + cn);
    Object.defineProperty(anim, '_keyed', { value: k, enumerable: false });
    return k;
  }

  function applyAnimation(sk, anim, t, alpha, mixTo = null, alphaMix = 1) {
    // Values are relative to the setup pose (Spine semantics). alpha < 1 blends from the current pose.
    // When this is the animation being mixed OUT (mixTo = the incoming one), properties the incoming animation also
    // keys are held at full strength (it blends over them), and the rest fade to the setup pose by alphaMix. That is
    // what Spine's AnimationState does, so nothing pops when the old animation is dropped.
    const to = mixTo ? keyedProps(mixTo) : null;
    const A = (key, ids) => (!to ? alpha : ids.some((id) => to.has(key + '.' + id)) ? alpha : alphaMix);
    for (const bn in anim.bones) {
      const b = sk.boneMap[bn]; if (!b) continue;
      const tl = anim.bones[bn];
      let a;
      if (tl.rotate) { a = A(bn, ['r']); const [v] = sample(tl.rotate, t, ['value']); b.rotation += ((b.data.rotation || 0) + v - b.rotation) * a; }
      if (tl.translate) { a = A(bn, ['x', 'y']); const [x, y] = sample(tl.translate, t, ['x', 'y']); b.x += ((b.data.x || 0) + x - b.x) * a; b.y += ((b.data.y || 0) + y - b.y) * a; }
      if (tl.translatex) { a = A(bn, ['x']); const [x] = sample(tl.translatex, t, ['value']); b.x += ((b.data.x || 0) + x - b.x) * a; }
      if (tl.translatey) { a = A(bn, ['y']); const [y] = sample(tl.translatey, t, ['value']); b.y += ((b.data.y || 0) + y - b.y) * a; }
      if (tl.scale) { a = A(bn, ['sx', 'sy']); const [x, y] = sample(tl.scale, t, ['x', 'y']); b.scaleX += ((b.data.scaleX ?? 1) * x - b.scaleX) * a; b.scaleY += ((b.data.scaleY ?? 1) * y - b.scaleY) * a; }
    }
    for (const sn in anim.slots) {
      const s = sk.slotMap[sn]; if (!s) continue;
      const tl = anim.slots[sn];
      if (tl.attachment) {
        let name = s.data.attachment;
        for (const k of tl.attachment) if (k.time <= t + 1e-6) name = k.name;
        if (alpha >= 0.5 && (!to || alphaMix >= 0.5)) s.attachmentName = name;
      }
      if (tl.rgba) { const a = A('slot:' + sn, ['rgba']); const [r, g, b2, al] = sample(tl.rgba, t, ['r', 'g', 'b', 'a']); s.r += (r - s.r) * a; s.g += (g - s.g) * a; s.b += (b2 - s.b) * a; s.a += (al - s.a) * a; }
    }
    for (const cn in anim.ik || {}) {
      const c = sk.ik.find((x) => x.data.name === cn); if (!c) continue;
      const a = !to ? alpha : to.has('ik:' + cn) ? alpha : alphaMix;
      const [m] = sample(anim.ik[cn], t, ['mix']); c.mix += (m - c.mix) * a;
    }
  }

  class Player {
    constructor(spec) {
      this.spec = spec; this.skeleton = new Skeleton(spec);
      this.track = null; this.from = null; this.mixTime = 0; this.mixDur = 0; this.listeners = [];
      this.timeScale = 1;
    }
    setAnimation(name, loop = true, mix = 0.1) {
      const anim = this.spec.animations[name];
      if (!anim) throw new Error('no animation ' + name);
      if (this.track && mix > 0) { this.from = this.track; this.mixTime = 0; this.mixDur = mix; } else this.from = null;
      this.track = { name, anim, loop, time: 0, lastTime: 0 };
      return this.track;
    }
    onEvent(fn) { this.listeners.push(fn); }
    update(dt) {
      dt *= this.timeScale;
      const sk = this.skeleton;
      sk.setToSetupPose();
      if (this.from) {
        const f = this.from; f.time += dt;
        const ft = f.loop ? f.time % f.anim.duration : Math.min(f.time, f.anim.duration);
        this.mixTime += dt;
        const mix = Math.min(1, this.mixTime / this.mixDur);
        applyAnimation(sk, f.anim, ft, 1, this.track.anim, 1 - mix);
        if (this.mixTime >= this.mixDur) this.from = null;
      }
      const tr = this.track;
      if (tr) {
        const prev = tr.time; tr.time += dt;
        const d = tr.anim.duration;
        const t = tr.loop ? tr.time % d : Math.min(tr.time, d);
        const alpha = this.from ? Math.min(1, this.mixTime / this.mixDur) : 1; // same mix fraction the old one used
        applyAnimation(sk, tr.anim, t, alpha);
        for (const e of tr.anim.events || []) {
          const p0 = tr.loop ? prev % d : prev, crossed = tr.loop && (tr.time % d) < p0 ? (e.time > p0 || e.time <= t) : (e.time > p0 && e.time <= t);
          if (crossed) this.listeners.forEach((fn) => fn(e.name, tr.name));
        }
      }
      sk.updateWorld(dt);
    }
    // World-space vertices for a slot's current attachment (x, y interleaved; y up).
    slotVertices(slot, out) {
      const att = this.attachment(slot); if (!att) return null;
      const sk = this.skeleton;
      if (att.type === 'mesh') {
        const v = att.vertices, n = att.uvs.length / 2;
        if (!out || out.length !== n * 2) out = new Float32Array(n * 2);
        if (v.length === att.uvs.length) { // unweighted: relative to the slot's bone
          const b = slot.bone;
          for (let i = 0; i < n; i++) { const x = v[i * 2], y = v[i * 2 + 1]; out[i * 2] = b.a * x + b.b * y + b.worldX; out[i * 2 + 1] = b.c * x + b.d * y + b.worldY; }
          return out;
        }
        let p = 0;
        for (let i = 0; i < n; i++) {
          const cnt = v[p++]; let wx = 0, wy = 0;
          for (let k = 0; k < cnt; k++) {
            const b = sk.bones[v[p]], x = v[p + 1], y = v[p + 2], w = v[p + 3]; p += 4;
            wx += (b.a * x + b.b * y + b.worldX) * w; wy += (b.c * x + b.d * y + b.worldY) * w;
          }
          out[i * 2] = wx; out[i * 2 + 1] = wy;
        }
        return out;
      }
      // region: 4 corners around (x, y) rotated by the attachment rotation, in bone space
      const b = slot.bone, hw = att.width / 2, hh = att.height / 2, r = att.rotation || 0;
      const cr = cosd(r), sr = sind(r);
      const corners = [[-hw, -hh], [hw, -hh], [hw, hh], [-hw, hh]];
      if (!out || out.length !== 8) out = new Float32Array(8);
      corners.forEach(([cx, cy], i) => {
        const lx = att.x + cx * cr - cy * sr, ly = att.y + cx * sr + cy * cr;
        out[i * 2] = b.a * lx + b.b * ly + b.worldX; out[i * 2 + 1] = b.c * lx + b.d * ly + b.worldY;
      });
      return out;
    }
    attachment(slot) {
      const name = slot.attachmentName; if (!name) return null;
      const d = this.spec.attachments[slot.name]; return d ? d[name] : null;
    }
  }

  const api = { Player, Skeleton, sample, bezierY };
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
  root.RigRuntime = api;
})(typeof window !== 'undefined' ? window : globalThis);
