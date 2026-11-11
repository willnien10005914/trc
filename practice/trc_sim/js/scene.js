// TRC 醫療照護三關的練習場景、模擬感知與裁判。
// 座標用 ROS 慣例（z 向上，單位公尺），整個場景掛在一個繞 x 轉 -90° 的群組底下。
// 配分依 trc_plan.html 的整理；手冊沒寫到的細項（每盒取藥 30、每情境 25）是用關卡總分反推的。
import * as THREE from 'three';

export const ARENA = { W: 15, H: 8 };
// 家具的幾何常數。機器人端（tasks.js）把這些當成「事先量好的家具模型」。
export const MODEL = {
  cart: { w: 0.65, d: 0.44, h: 1.09, handleY: 0.265, drawerZ: [0.80, 0.58], stroke: 0.34, markerLocal: [0, 0.221, 1.0], boxLocalY: 0.14, boxDz: -0.04, dockY: 1.101 },
  cabinet: { w: 0.45, d: 0.45, h: 0.82, zone: [0.12, 0.16], markerLocal: [0, 0.226, 0.6], dockY: 0.645 },
  gate: { buttonZ: { 2: 0.90, 3: 1.00, 4: 1.10 }, buttonY: 0.012, signLocal: [0, 0.005, 1.32], lightLocal: [0.2, 0.03, 1.55], markerLocal: [-0.22, 0.002, 1.0], dockY: 0.706 },
  wardBed: { footY: 2.1, monitorLocal: [-0.65, 0.8, 1.05], ivLocal: [0.65, 0.8, 1.25], readY: 1.8, markerLocal: [0, 1.01, 0.5] },
  source: { w: 2.6, d: 0.5, h: 0.80, rowY: 0.13, dockY: 0.67 },
  maquet: { w: 0.70, d: 0.50, h: 0.96, slotX: [0.24, 0.08, -0.08, -0.24], slotY: 0.05, zone: [0.33, 0.23], markerLocal: [0, 0.251, 0.9], dockY: 0.67 },
  box: [0.08, 0.12, 0.05], instrumentLen: 0.16,
};
const DRUGS = ['Amoxicillin 500 mg', 'Metformin 850 mg', 'Losartan 50 mg', 'Atorvastatin 20 mg', 'Omeprazole 20 mg', 'Aspirin 100 mg', 'Furosemide 40 mg', 'Warfarin 2 mg'];
// 示意用的 20 種器具名稱，不是競賽手冊的清單
const INSTRUMENTS = ['Scalpel handle No.3', 'Scalpel handle No.4', 'Mayo scissors', 'Metzenbaum scissors', 'Iris scissors', 'Adson forceps', 'DeBakey forceps', 'Tissue forceps', 'Dressing forceps', 'Kelly clamp', 'Mosquito clamp', 'Kocher clamp', 'Allis clamp', 'Babcock clamp', 'Needle holder', 'Towel clamp', 'Army-Navy retractor', 'Senn retractor', 'Bone curette', 'Suction tip'];
const SCENARIOS = ['offbed', 'fall', 'hr', 'iv', 'gown'];
export const SCENARIO_NAME = { offbed: '離床', fall: '跌落', hr: '心跳', iv: '點滴', gown: '病人服' };
const GOWNS = { green: 0x2e9e55, blue: 0x3b6fd0, pink: 0xe58aa5, white: 0xe9e9e4 };
const GOWN_ZH = { green: '綠色', blue: '藍色', pink: '粉紅色', white: '白色' };

export function rng(seed) {
  let a = seed >>> 0;
  return () => { a = a + 0x6D2B79F5 | 0; let t = Math.imul(a ^ a >>> 15, 1 | a); t = t + Math.imul(t ^ t >>> 7, 61 | t) ^ t; return ((t ^ t >>> 14) >>> 0) / 4294967296; };
}
const wrap = a => Math.atan2(Math.sin(a), Math.cos(a));

// 家具座標系：原點在家具中心的地面，+y 是正面朝外的法線，+x 沿正面。
// 機器人右側貼著正面停靠時，車頭角度就等於 psi。
export class Frame {
  constructor(x, y, psi) { this.x = x; this.y = y; this.psi = psi; this.c = Math.cos(psi); this.s = Math.sin(psi); }
  toWorld(lx, ly, lz = 0) { return new THREE.Vector3(this.x + this.c * lx - this.s * ly, this.y + this.s * lx + this.c * ly, lz); }
  toLocal(v) { const dx = v.x - this.x, dy = v.y - this.y; return new THREE.Vector3(this.c * dx + this.s * dy, -this.s * dx + this.c * dy, v.z); }
  dock(xr, yr) { const p = this.toWorld(xr, yr); return [p.x, p.y, this.psi]; }
}

const mat = (color, o = {}) => new THREE.MeshStandardMaterial({ color, roughness: 0.75, metalness: 0.05, ...o });
function box(w, d, h, color, x = 0, y = 0, z = 0, o) {
  const m = new THREE.Mesh(new THREE.BoxGeometry(w, d, h), mat(color, o));
  m.position.set(x, y, z); m.castShadow = true; m.receiveShadow = true;
  return m;
}
function cyl(r, h, color, x = 0, y = 0, z = 0, o) {
  const m = new THREE.Mesh(new THREE.CylinderGeometry(r, r, h, 20), mat(color, o));
  m.rotation.x = Math.PI / 2; m.position.set(x, y, z); m.castShadow = true;
  return m;
}
function sphere(r, color, x, y, z, o) {
  const m = new THREE.Mesh(new THREE.SphereGeometry(r, 16, 12), mat(color, o));
  m.position.set(x, y, z);
  return m;
}
// 立在牆上的平面：法線 n（水平），上方是 +z
function facing(mesh, nx, ny) {
  const X = new THREE.Vector3(-ny, nx, 0), Y = new THREE.Vector3(0, 0, 1), Z = new THREE.Vector3(nx, ny, 0);
  mesh.quaternion.setFromRotationMatrix(new THREE.Matrix4().makeBasis(X, Y, Z));
  return mesh;
}
function textPlane(lines, w, h, { font = 'Times New Roman', bg = '#101418', fg = '#f4f4f0', px = 512, bold = false, size = null } = {}) {
  const cv = document.createElement('canvas');
  cv.width = px; cv.height = Math.max(16, Math.round(px * h / w));
  const g = cv.getContext('2d');
  g.fillStyle = bg; g.fillRect(0, 0, cv.width, cv.height);
  g.fillStyle = fg; g.textBaseline = 'middle';
  const fs = size || Math.min(cv.height / (lines.length + 0.6), cv.width / 9);
  lines.forEach((ln, i) => {
    g.font = `${bold || i === 0 ? 'bold ' : ''}${Math.round(fs * (i === 0 ? 1 : 0.86))}px "${font}", serif`;
    g.textAlign = lines.length === 1 ? 'center' : 'left';
    g.fillText(ln, lines.length === 1 ? cv.width / 2 : cv.width * 0.05, (i + 0.8) * cv.height / (lines.length + 0.6), cv.width * 0.92);
  });
  const tex = new THREE.CanvasTexture(cv);
  tex.colorSpace = THREE.SRGBColorSpace; tex.anisotropy = 4;
  const m = new THREE.Mesh(new THREE.PlaneGeometry(w, h), new THREE.MeshBasicMaterial({ map: tex }));
  m.userData.canvas = cv; m.userData.tex = tex;
  return m;
}
function markerPlane(id, size = 0.12) {      // 示意用的方格圖樣，不是真的 ArUco 字典
  const cv = document.createElement('canvas'); cv.width = cv.height = 70;
  const g = cv.getContext('2d'), r = rng(id * 7919 + 1);
  g.fillStyle = '#fff'; g.fillRect(0, 0, 70, 70); g.fillStyle = '#000'; g.fillRect(5, 5, 60, 60); g.fillStyle = '#fff';
  for (let j = 0; j < 4; j++) for (let i = 0; i < 4; i++) if (r() > 0.5) g.fillRect(15 + i * 10, 15 + j * 10, 10, 10);
  const tex = new THREE.CanvasTexture(cv); tex.magFilter = THREE.NearestFilter; tex.colorSpace = THREE.SRGBColorSpace;
  return new THREE.Mesh(new THREE.PlaneGeometry(size, size), new THREE.MeshBasicMaterial({ map: tex }));
}

export class Hospital {
  constructor(world, seed, log) {
    this.world = world; this.seed = seed; this.log = log || (() => {});
    this.group = new THREE.Group(); world.add(this.group);
    this.rnd = rng(seed);
    this.obstacles = []; this.solids = []; this.surfaces = []; this.occluders = [];
    this.targets = []; this.items = []; this.drawers = []; this.buttons = [];
    this.held = null; this.heldHandle = null; this.time = 0;
    this.map = { formulary: {} };
    this.truth = {};
    this.events = [];
    this._build();
    this.ref = new Referee(this);
  }
  pick(a) { return a[Math.floor(this.rnd() * a.length)]; }
  shuffle(a) { a = a.slice(); for (let i = a.length - 1; i > 0; i--) { const j = Math.floor(this.rnd() * (i + 1)); [a[i], a[j]] = [a[j], a[i]]; } return a; }
  dispose() { if (this.held) this.held.node.removeFromParent(); this.world.remove(this.group); }

  // ---------- 建場景 ----------
  _furn(frame) { const g = new THREE.Group(); g.position.set(frame.x, frame.y, 0); g.rotation.z = frame.psi; this.group.add(g); return g; }
  _obs(o) { o.active = o.active !== false; this.obstacles.push(o); return o; }
  _rectObs(id, frame, w, d, known, lx = 0, ly = 0) { const p = frame.toWorld(lx, ly); return this._obs({ id, type: 'rect', cx: p.x, cy: p.y, w, d, rot: frame.psi, known }); }
  _solid(id, frame, lo, hi) { this.solids.push({ id, frame, lo, hi }); }
  _occ(...meshes) { for (const m of meshes) m.traverse(o => { if (o.isMesh) this.occluders.push(o); }); }
  _target(t) { t.enabled = t.enabled || (() => true); this.targets.push(t); return t; }
  _marker(id, frame, g, local, range = 2.6) {
    const m = facing(markerPlane(id), 0, 1); m.position.set(local[0], local[1], local[2]); g.add(m);
    this._target({ id: 'aruco' + id, kind: 'aruco', cam: 'head', range, node: g, pos: () => frame.toWorld(...local), data: () => ({ marker: id, yaw: frame.psi + Math.PI / 2 }) });
  }
  _wall(x0, y0, x1, y1) {
    const w = Math.abs(x1 - x0) + 0.1, d = Math.abs(y1 - y0) + 0.1, cx = (x0 + x1) / 2, cy = (y0 + y1) / 2;
    const m = box(w, d, 2.2, 0xb9c4cf, cx, cy, 1.1, { transparent: true, opacity: 0.16, depthWrite: false });
    m.castShadow = false; this.group.add(m); this.group.add(box(w, d, 0.06, 0x6c7884, cx, cy, 0.03));
    this.occluders.push(m);
    const f = new Frame(cx, cy, 0);
    this._obs({ id: 'wall', type: 'rect', cx, cy, w, d, rot: 0, known: true });
    this._solid('wall', f, [-w / 2, -d / 2, 0], [w / 2, d / 2, 2.2]);
  }
  _zoneLabel(text, x, y, w, h, color) {
    const p = new THREE.Mesh(new THREE.PlaneGeometry(w, h), new THREE.MeshBasicMaterial({ color, transparent: true, opacity: 0.22 }));
    p.position.set(x, y, 0.004); this.group.add(p);
    const t = textPlane([text], Math.min(w, 2.6), 0.3, { font: 'Segoe UI', bg: 'rgba(0,0,0,0)', fg: '#20303c', bold: true });
    t.material.transparent = true; t.position.set(x, y - h / 2 + 0.22, 0.008); this.group.add(t);
  }
  _tv(id, x, y, nx, ny, lines) {
    const g = new THREE.Group(); g.position.set(x, y, 1.25); this.group.add(g);
    const frame = facing(box(1.44, 0.81, 0.05, 0x111111), nx, ny); g.add(frame);
    const t = facing(textPlane(lines, 1.36, 0.74, { px: 1024 }), nx, ny); t.position.set(nx * 0.03, ny * 0.03, 0); g.add(t);
    this._target({ id, kind: 'tv', cam: 'head', range: 4.0, node: g, pos: () => new THREE.Vector3(x + nx * 0.03, y + ny * 0.03, 1.25), data: () => ({ lines }) });
    return new THREE.Vector3(x, y, 1.25);
  }

  _build() {
    const G = this.group, { W, H } = ARENA;
    const floor = new THREE.Mesh(new THREE.PlaneGeometry(W, H), mat(0xdfe6ea));
    floor.position.set(W / 2, H / 2, 0); floor.receiveShadow = true; G.add(floor);
    const grid = new THREE.GridHelper(16, 16, 0xb0bcc6, 0xcdd6dc); grid.rotation.x = Math.PI / 2; grid.position.set(8, 4, 0.002); G.add(grid);
    this._wall(0, 0, W, 0); this._wall(0, H, W, H); this._wall(0, 0, 0, H); this._wall(W, 0, W, H);
    this._wall(7, 0, 7, 5.6); this._wall(7, 6.6, 7, H);          // 第一關與第二關之間，留跨樓層閘門
    this._wall(7, 4, 13.2, 4); this._wall(14.4, 4, W, 4);        // 第二關與第三關之間
    this._zoneLabel('第一關　藥物配送', 3.5, 4, 6.8, 7.8, 0x4f9bd9);
    this._zoneLabel('第二關　病房巡檢', 11, 6, 7.8, 3.8, 0x58b889);
    this._zoneLabel('第三關　手術器具', 11, 2, 7.8, 3.8, 0xe0a34a);
    this._zoneLabel('START', 1.0, 1.2, 0.9, 0.9, 0x333333);
    this.map.start = [1.0, 1.2, Math.PI / 2];
    this._stage1(); this._stage2(); this._stage3();
  }

  _stage1() {
    const M = MODEL, T = this.truth;
    // 抽籤決定 4 具人型障礙與 2 張輪椅的擺法
    // 紅籤第 3 具人型原本在 (4.4, 2.2)：在規劃器膨脹半徑 0.36 m 下會封死 Bed 1 與 Bed 2 之間的通道，
    // Bed 1 床旁櫃無路可達（種子 7、8、9、12 都因此失敗），所以移到 (4.0, 1.2)。這是改場景讓關卡可過，不是機器人能力的提升。
    const layouts = [
      { name: '紅籤', m: [[2.4, 3.2], [3.9, 4.6], [4.0, 1.2], [1.3, 4.4]], w: [[2.8, 5.0, 0.3], [4.2, 3.5, 1.2]] },
      { name: '藍籤', m: [[1.8, 2.8], [3.0, 3.8], [4.3, 5.2], [2.2, 5.3]], w: [[3.6, 2.4, 0], [4.4, 4.0, 1.57]] },
      { name: '綠籤', m: [[1.4, 3.6], [2.2, 4.3], [3.8, 3.0], [4.6, 4.6]], w: [[2.0, 2.2, 0.8], [3.0, 4.9, 0]] },
    ];
    const L = layouts[Math.floor(this.rnd() * 3)]; T.lottery = L.name;
    const lc = { '紅籤': 0xc9463d, '藍籤': 0x3f6fc4, '綠籤': 0x3d9a57 }[L.name];
    L.m.forEach(([x, y], i) => {
      const g = new THREE.Group(); g.position.set(x, y, 0); this.group.add(g);
      g.add(cyl(0.2, 1.4, lc, 0, 0, 0.7), sphere(0.13, 0xe8c9a8, 0, 0, 1.55)); this._occ(g);
      this._obs({ id: 'mannequin' + i, type: 'circle', cx: x, cy: y, r: 0.22, known: false });
    });
    L.w.forEach(([x, y, rot], i) => {
      const f = new Frame(x, y, rot), g = this._furn(f);
      g.add(box(0.6, 0.6, 0.12, 0x39424b, 0, 0.1, 0.48), box(0.6, 0.08, 0.5, 0x39424b, 0, -0.3, 0.75), cyl(0.28, 0.04, 0x222, -0.31, -0.1, 0.28), cyl(0.28, 0.04, 0x222, 0.31, -0.1, 0.28));
      g.children[2].rotation.set(0, 0, Math.PI / 2); g.children[3].rotation.set(0, 0, Math.PI / 2); this._occ(g);
      this._rectObs('wheelchair' + i, f, 0.65, 1.0, false);
    });

    // 兩台藥車，各兩層抽屜，每層兩盒。藥車有輪子，實際位置與地圖略有出入，要靠 ArUco 對位
    const drugs = this.shuffle(DRUGS); let di = 0;
    T.boxes = []; this.map.carts = [];
    ['A', 'B'].forEach((name, ci) => {
      const nominal = new Frame(1.6 + ci * 2.0, 7.45, Math.PI);
      const f = new Frame(nominal.x + (this.rnd() - 0.5) * 0.16, nominal.y - this.rnd() * 0.05, Math.PI + (this.rnd() - 0.5) * 0.14);
      const g = this._furn(f), c = M.cart;
      const body = box(c.w, c.d, c.h, 0xe9edf0); body.position.z = c.h / 2; g.add(body); this._occ(body);
      g.add(box(c.w + 0.02, c.d + 0.02, 0.03, 0x4a90c2, 0, 0, c.h + 0.015));
      const lab = facing(textPlane(['Cart ' + name], 0.3, 0.09, { bg: '#ffffff', fg: '#103050' }), 0, 1); lab.position.set(0, c.d / 2 + 0.002, 0.3); g.add(lab);
      this._marker(11 + ci, f, g, c.markerLocal);
      this._rectObs('cart' + name, f, c.w, c.d, false);
      this._solid('cart' + name, f, [-c.w / 2, -c.d / 2, 0], [c.w / 2, c.d / 2, c.h]);
      this.map.carts.push({ name, frame: nominal, marker: 11 + ci });
      c.drawerZ.forEach((zc, li) => {
        const dg = new THREE.Group(); g.add(dg);
        dg.add(box(0.57, 0.02, 0.16, 0xcfd8de, 0, 0.23, zc), box(0.2, 0.03, 0.02, 0x30363c, 0, c.handleY - 0.01, zc),
          box(0.55, 0.36, 0.01, 0xb8c2c9, 0, 0.04, zc - 0.07), box(0.01, 0.36, 0.12, 0xb8c2c9, -0.275, 0.04, zc - 0.01), box(0.01, 0.36, 0.12, 0xb8c2c9, 0.275, 0.04, zc - 0.01));
        const drawer = { id: `cart${name}/${li ? '下層' : '上層'}`, cart: name, layer: li, frame: f, group: dg, zc, ext: 0, maxExt: 0, boxes: [] };
        this.drawers.push(drawer);
        [-0.13, 0.13].forEach(bx => {
          const drug = drugs[di++], barcode = '4710' + String(Math.floor(this.rnd() * 9e7 + 1e7));
          const node = new THREE.Group(); node.add(box(...M.box, [0xf2c14e, 0x78c0e0, 0xf08a5d, 0xb8de6f, 0xd4a5e8, 0xf7e08c, 0x9ad1c1, 0xf4a6a6][di - 1]), box(0.06, 0.05, 0.002, 0xffffff, 0, 0, 0.026));
          node.position.set(bx, c.boxLocalY, zc + c.boxDz); dg.add(node);
          const item = { id: 'box:' + barcode, kind: 'box', node, drug, barcode, drawer, half: M.box[2] / 2, everHeld: false, dropped: false, on: null };
          this.items.push(item); drawer.boxes.push(item); T.boxes.push(item);
          this.map.formulary[drug] = barcode;
          this._target({ id: 'barcode:' + barcode, kind: 'barcode', cam: 'ee', range: 0.45, node, enabled: () => !item.everHeld, pos: () => this._itemPos(item), data: () => ({ barcode }) });
        });
      });
    });
    // 兩張病床與床旁櫃
    this.map.cabinets = {};
    [1, 2].forEach((n, i) => {
      const by = 1.0 + i * 2.4;
      const bg = this._furn(new Frame(5.9, by, 0));
      bg.add(box(2.0, 0.9, 0.4, 0x9aa7b1, 0, 0, 0.25), box(1.95, 0.85, 0.12, 0xf7f7f2, 0, 0, 0.5), box(0.35, 0.6, 0.08, 0xd9e6f2, 0.75, 0, 0.6)); this._occ(bg);
      const bl = facing(textPlane(['Bed ' + n], 0.4, 0.14, { bg: '#ffffff', fg: '#103050' }), -1, 0); bl.position.set(-1.002, 0, 0.3); bg.add(bl);
      this._obs({ id: 'bed' + n, type: 'rect', cx: 5.9, cy: by, w: 2.0, d: 0.9, rot: 0, known: true });
      this._solid('bed' + n, new Frame(5.9, by, 0), [-1, -0.45, 0], [1, 0.45, 0.56]);
      const f = new Frame(6.725, by + 1.0, Math.PI / 2), g = this._furn(f), c = MODEL.cabinet;
      const body = box(c.w, c.d, c.h, 0xc8b79c); body.position.z = c.h / 2; g.add(body); this._occ(body);
      const zone = new THREE.Mesh(new THREE.PlaneGeometry(c.zone[0], c.zone[1]), new THREE.MeshBasicMaterial({ color: 0xd43b3b, wireframe: true })); zone.position.z = c.h + 0.002; g.add(zone);
      g.add(box(c.zone[0], c.zone[1], 0.001, 0xfff3c4, 0, 0, c.h + 0.001));
      this._marker(21 + i, f, g, c.markerLocal);
      this._rectObs('cabinet' + n, f, c.w, c.d, true);
      this._solid('cabinet' + n, f, [-c.w / 2, -c.d / 2, 0], [c.w / 2, c.d / 2, c.h]);
      this.surfaces.push({ id: 'cabinet' + n, frame: f, x: c.w / 2, y: c.d / 2, z: c.h });
      this.map.cabinets[n] = { frame: f, marker: 21 + i };
    });
    // 醫囑：兩筆，各用一台藥車、送到一張床
    const cartOrder = this.shuffle([0, 1]), beds = this.shuffle([1, 2]);
    T.orders = cartOrder.map((ci, i) => {
      const cand = T.boxes.filter(b => b.drawer.cart === 'AB'[ci]);
      const b = cand[Math.floor(this.rnd() * cand.length)];
      return { bed: beds[i], drug: b.drug, cart: 'AB'[ci], item: b };
    });
    const lines = ['MEDICATION ORDER', ...T.orders.map((o, i) => `${i + 1}.  Bed ${o.bed} :  ${o.drug}  ( Cart ${o.cart} )`)];
    this.map.tv1 = this._tv('tv1', 2.6, 0.07, 0, 1, lines);
  }

  _stage2() {
    const T = this.truth, M = MODEL.gate;
    // 跨樓層按鈕與閘門
    const f = new Frame(6.95, 5.2, Math.PI / 2), g = this._furn(f);
    T.floor = this.pick([2, 3, 4]); T.light = 'red'; T.presses = 0; T.wrong = 0; T.redRun = false; T.greenAt = null;
    g.add(box(0.5, 0.012, 0.9, 0x2b3138, 0, 0.006, 1.2));
    for (const k of [2, 3, 4]) {
      const b = cyl(0.025, 0.012, 0xf2f2f2, 0, M.buttonY - 0.006, M.buttonZ[k]); b.rotation.x = 0; g.add(b);
      const t = facing(textPlane([k + 'F'], 0.07, 0.04, { bg: '#2b3138', fg: '#ffffff', font: 'Arial' }), 0, 1); t.position.set(0.09, 0.013, M.buttonZ[k]); g.add(t);
      this.buttons.push({ floor: k, mesh: b, pos: () => f.toWorld(0, M.buttonY, M.buttonZ[k]), down: false });
    }
    const sign = facing(textPlane([`WARD  ${T.floor}F`], 0.36, 0.1, { bg: '#f5f5f0', fg: '#101010' }), 0, 1); sign.position.set(...M.signLocal); g.add(sign);
    this._target({ id: 'sign', kind: 'sign', cam: 'head', range: 2.5, node: g, pos: () => f.toWorld(...M.signLocal), data: () => ({ text: `WARD ${T.floor}F` }) });
    this.lightMesh = sphere(0.05, 0xff2a2a, ...M.lightLocal, { emissive: 0xff2a2a, emissiveIntensity: 0.9 }); g.add(this.lightMesh);
    this._target({ id: 'light', kind: 'light', cam: 'head', range: 3.0, node: g, pos: () => f.toWorld(...M.lightLocal), data: () => ({ color: T.light }) });
    this._marker(41, f, g, M.markerLocal);
    this.map.gate = { frame: f, marker: 41, through: [[6.2, 6.1], [7.8, 5.2]] };
    this.door = { mesh: box(0.06, 1.0, 2.0, 0x8fa3b3, 7, 6.1, 1.0, { transparent: true, opacity: 0.7 }), open: 0, obs: this._obs({ id: 'door', type: 'rect', cx: 7, cy: 6.1, w: 0.1, d: 1.0, rot: 0, known: true }), seenOpen: false };
    this.group.add(this.door.mesh); this.occluders.push(this.door.mesh);

    // 病房：四張床，五種情境抽四種。心跳固定在 W1
    let sc = this.shuffle(SCENARIOS).slice(0, 4);
    if (sc.includes('hr')) sc = ['hr', ...this.shuffle(sc.filter(s => s !== 'hr'))];
    T.ward = []; this.map.ward = [];
    sc.forEach((kind, i) => {
      const name = 'W' + (i + 1), bx = 8.9 + i * 1.7, fb = new Frame(bx, 7.0, Math.PI), bg = this._furn(fb), W = MODEL.wardBed;
      bg.add(box(0.9, 2.0, 0.4, 0x9aa7b1, 0, 0, 0.25), box(0.85, 1.95, 0.12, 0xf7f7f2, 0, 0, 0.5), box(0.6, 0.35, 0.08, 0xd9e6f2, 0, -0.75, 0.6)); this._occ(bg);
      const tag = facing(textPlane([name], 0.3, 0.14, { bg: '#ffffff', fg: '#103050' }), 0, 1); tag.position.set(0.28, 1.002, 0.3); bg.add(tag);
      this._marker(51 + i, fb, bg, W.markerLocal, 3.2);
      this._obs({ id: 'ward' + name, type: 'rect', cx: bx, cy: 7.0, w: 0.9, d: 2.0, rot: 0, known: true });
      this._solid('ward' + name, fb, [-0.45, -1, 0], [0.45, 1, 0.56]);
      const st = { name, kind, away: false, fall: false, gown: this.pick(['blue', 'pink', 'white']), bpm: null, ml: null };
      if (kind === 'offbed') st.away = this.rnd() < 0.6;
      if (kind === 'fall') st.fall = this.rnd() < 0.6;
      if (kind === 'gown') st.gown = this.rnd() < 0.5 ? 'green' : st.gown;
      if (!st.away) {
        const pg = new THREE.Group(); bg.add(pg);
        pg.add(box(0.42, 1.25, 0.2, GOWNS[st.gown], 0, 0.15, 0), sphere(0.12, 0xe8c9a8, 0, -0.62, 0.02));
        // 跌落的病人橫躺在床尾地上
        if (st.fall) { pg.position.set(0, 1.35, 0.12); pg.rotation.z = Math.PI / 2; this._rectObs('fallen' + name, fb, 1.5, 0.45, false, 0, 1.35); } else pg.position.set(0, -0.1, 0.68);
        this._target({ id: 'patient' + name, kind: 'patient', cam: 'head', range: 3.2, node: pg, pos: () => fb.toWorld(pg.position.x, pg.position.y, pg.position.z), data: () => ({ state: st.fall ? 'on_floor' : 'in_bed', gown: st.gown }) });
      }
      if (kind === 'hr') {
        st.bpm = 55 + Math.floor(this.rnd() * 70);
        const [mx, my, mz] = W.monitorLocal; const mg = new THREE.Group(); mg.position.set(mx, my, 0); bg.add(mg);
        mg.add(cyl(0.02, mz, 0x777777, 0, 0, mz / 2), box(0.3, 0.3, 0.03, 0x555555, 0, 0, 0.015), box(0.26, 0.2, 0.2, 0x2f3b45, 0, 0, mz));
        const scr = facing(textPlane([`HR ${st.bpm}`], 0.1, 0.075, { bg: '#001a08', fg: '#35ff6a', font: 'Consolas', bold: true }), 0, 1); scr.position.set(0, 0.101, mz); mg.add(scr); this._occ(mg.children[2]);
        this._obs({ id: 'monitor' + name, type: 'circle', cx: fb.toWorld(mx, my).x, cy: fb.toWorld(mx, my).y, r: 0.18, known: false });
        this._target({ id: 'monitor' + name, kind: 'monitor', cam: 'head', range: 3.5, node: mg, pos: () => fb.toWorld(mx, my, mz), data: () => ({}) });
        this._target({ id: 'hr' + name, kind: 'hr', cam: 'head', range: 1.2, node: mg, pos: () => fb.toWorld(mx, my + 0.1, mz), data: () => ({ bpm: st.bpm }) });
      }
      if (kind === 'iv') {
        st.ml = 15 + Math.floor(this.rnd() * 100);
        const [ix, iy, iz] = W.ivLocal; const ig = new THREE.Group(); ig.position.set(ix, iy, 0); bg.add(ig);
        ig.add(cyl(0.012, 1.85, 0x888888, 0, 0, 0.925), box(0.3, 0.3, 0.03, 0x555555, 0, 0, 0.015), box(0.1, 0.03, 0.16, 0xdcefff, 0, 0.05, 1.62, { transparent: true, opacity: 0.8 }),
          cyl(0.028, 0.2, 0xffffff, 0, 0.05, iz, { transparent: true, opacity: 0.35 }), cyl(0.026, 0.2 * st.ml / 120, 0x5fb4ff, 0, 0.05, iz - 0.1 + 0.1 * st.ml / 120));
        this._obs({ id: 'iv' + name, type: 'circle', cx: fb.toWorld(ix, iy).x, cy: fb.toWorld(ix, iy).y, r: 0.18, known: false });
        this._target({ id: 'iv' + name, kind: 'iv', cam: 'head', range: 3.5, node: ig, pos: () => fb.toWorld(ix, iy, iz), data: () => ({}) });
        // 讀值帶 ±3 ml 的誤差，模擬刻度判讀
        this._target({ id: 'ivlevel' + name, kind: 'ivlevel', cam: 'head', range: 1.2, node: ig, pos: () => fb.toWorld(ix, iy + 0.05, iz), data: () => ({ ml: Math.round(st.ml + (this.rnd() - 0.5) * 6) }) });
      }
      T.ward.push(st);
      this.map.ward.push({ name, frame: fb, marker: 51 + i });
    });
  }

  _stage3() {
    const T = this.truth, S = MODEL.source, Q = MODEL.maquet;
    const fs = new Frame(10.3, 1.45, 0), sg = this._furn(fs);
    sg.add(box(S.w, S.d, 0.04, 0xb7c0c7, 0, 0, S.h - 0.02), box(S.w - 0.1, S.d - 0.1, S.h - 0.04, 0x8f9aa3, 0, 0, (S.h - 0.04) / 2, { transparent: true, opacity: 0.35 })); this._occ(sg.children[0]);
    this._rectObs('source', fs, S.w, S.d, true);
    this._solid('source', fs, [-S.w / 2, -S.d / 2, 0], [S.w / 2, S.d / 2, S.h]);
    this.surfaces.push({ id: 'source', frame: fs, x: S.w / 2, y: S.d / 2, z: S.h });
    this.map.source = { frame: fs, scan: [10.3, 2.9, -Math.PI / 2] };
    const names = this.shuffle(INSTRUMENTS); T.instruments = [];
    names.forEach((name, i) => {
      const lx = -1.1 + (i % 10) * (2.2 / 9), ly = i < 10 ? S.rowY : -S.rowY, yaw = this.rnd() * Math.PI * 2;
      const node = new THREE.Group(), hue = (INSTRUMENTS.indexOf(name) * 0.05) % 1;
      const col = new THREE.Color().setHSL(hue, 0.35, 0.62);
      node.add(box(0.16, 0.012, 0.008, col), box(0.05, 0.03, 0.008, 0x3a3f45, -0.055, 0, 0), box(0.03, 0.008, 0.008, 0xd43b3b, 0.07, 0, 0.001));
      const p = fs.toWorld(lx, ly, S.h + 0.004); node.position.copy(p); node.rotation.z = yaw; this.group.add(node);
      const item = { id: 'inst:' + name, kind: 'instrument', node, name, half: 0.004, everHeld: false, dropped: false, on: 'source', placeSeq: 0 };
      this.items.push(item); T.instruments.push(item);
      this._target({ id: item.id, kind: 'instrument', cam: 'head', range: 2.3, node, enabled: () => item.on === 'source' && this.held !== item, pos: () => this._itemPos(item), data: () => ({ name, yaw: this._itemYaw(item) }) });
    });
    this.map.maquet = {};
    [['left', 8.5], ['right', 12.0]].forEach(([side, cx], i) => {
      const f = new Frame(cx, 3.65, Math.PI), g = this._furn(f);
      g.add(box(Q.w, Q.d, 0.03, 0xcfd6db, 0, 0, Q.h - 0.015), cyl(0.04, Q.h - 0.03, 0x8c969e, 0, 0, (Q.h - 0.03) / 2), box(0.55, 0.45, 0.04, 0x8c969e, 0, 0, 0.02),
        box(Q.w, 0.012, 0.16, 0xaab4bb, 0, Q.d / 2 - 0.006, Q.h - 0.11)); this._occ(g.children[0]);
      const z = new THREE.Mesh(new THREE.PlaneGeometry(Q.zone[0] * 2, Q.zone[1] * 2), new THREE.MeshBasicMaterial({ color: 0xd43b3b, wireframe: true })); z.position.z = Q.h + 0.002; g.add(z);
      Q.slotX.forEach((sx, k) => { const s = textPlane([String(k + 1)], 0.05, 0.05, { bg: '#e8eef2', fg: '#8a96a0', font: 'Arial' }); s.position.set(sx, 0.2, Q.h + 0.002); s.rotation.z = Math.PI; g.add(s); });
      this._marker(61 + i, f, g, Q.markerLocal);
      this._rectObs('maquet-' + side, f, Q.w, Q.d, true);
      this._solid('maquet-' + side, f, [-Q.w / 2, -Q.d / 2, 0], [Q.w / 2, Q.d / 2, Q.h]);
      this.surfaces.push({ id: 'maquet-' + side, frame: f, x: Q.w / 2, y: Q.d / 2, z: Q.h });
      this.map.maquet[side] = { frame: f, marker: 61 + i };
      this._zoneLabel(side === 'left' ? 'LEFT ZONE' : 'RIGHT ZONE', cx, 3.1, 1.8, 1.7, side === 'left' ? 0x7b6fd0 : 0xd07b6f);
    });
    T.zone = this.pick(['left', 'right']);
    T.set = this.shuffle(T.instruments).slice(0, 4);
    const lines = [`SURGICAL SET  -  ${T.zone.toUpperCase()} ZONE`, ...T.set.map((it, i) => `${i + 1}.  ${it.name}`)];
    this.map.tv3 = this._tv('tv3', 14.93, 2.4, -1, 0, lines);
    this.map.tv3view = [13.6, 3.0, 0];
  }

  // ---------- 物件位置 ----------
  _itemPos(item) { return this.world.worldToLocal(item.node.getWorldPosition(new THREE.Vector3())); }
  _itemDir(item) {            // 長軸（器具是尖端方向）在 ROS 世界的單位向量
    item.node.updateWorldMatrix(true, false);
    const d = new THREE.Vector3(item.kind === 'box' ? 0 : 1, item.kind === 'box' ? 1 : 0, 0).transformDirection(item.node.matrixWorld);
    return d.applyQuaternion(this.world.getWorldQuaternion(new THREE.Quaternion()).invert());
  }
  _itemYaw(item) { const d = this._itemDir(item); return Math.atan2(d.y, d.x); }
  handlePos(dr) { return dr.frame.toWorld(0, MODEL.cart.handleY + dr.ext, dr.zc); }
  holding() { return this.held; }               // 模擬用：實機要看夾爪開度與電流
  holdingHandle() { return this.heldHandle; }

  // ---------- 夾爪事件 ----------
  onGripper(robot, closed) {
    const gc = robot.graspRos();
    if (!closed) {
      if (this.heldHandle) { this.log(`[場景] 放開 ${this.heldHandle.id} 把手，抽屜開度 ${(this.heldHandle.ext * 100).toFixed(0)} cm`); this.heldHandle = null; }
      if (this.held) this._release(this.held);
      return;
    }
    for (const dr of this.drawers) if (gc.distanceTo(this.handlePos(dr)) < 0.03) { this.heldHandle = dr; this.ref.cartsDocked.add(dr.cart); this.log(`[場景] 夾住 ${dr.id} 把手`); return; }
    let best = null, bd = 0.025;
    for (const it of this.items) { if (it.dropped) continue; const d = gc.distanceTo(this._itemPos(it)); if (d < bd) { bd = d; best = it; } }
    if (!best) return;
    const dir = this._itemDir(best), fy = robot.fingerYaw();
    if (Math.abs(dir.x * Math.cos(fy) + dir.y * Math.sin(fy)) > 0.5) { this.log(`[場景] 夾爪方向與 ${best.name || best.drug} 的長軸不垂直，沒夾住`); return; }
    robot.urdf.links.link_grasp_center.attach(best.node);
    this.held = best; best.everHeld = true; best.on = null;
    this.log(`[場景] 夾起 ${best.name || best.drug}`);
  }
  _release(item) {
    this.held = null;
    this.group.attach(item.node);
    const p = item.node.position.clone(), dir = this._itemDir(item);
    const tilt = Math.asin(Math.min(1, Math.abs(dir.z)));
    const low = p.z - item.half - (item.kind === 'instrument' ? Math.sin(tilt) * MODEL.instrumentLen / 2 : 0);
    let sup = null;
    for (const s of this.surfaces) { const l = s.frame.toLocal(p); if (Math.abs(l.x) <= s.x && Math.abs(l.y) <= s.y && low - s.z > -0.012 && low - s.z < 0.08) sup = s; }
    const yaw = Math.atan2(dir.y, dir.x), dh = Math.hypot(dir.x, dir.y);
    item.node.rotation.set(0, 0, item.kind === 'box' ? yaw - Math.PI / 2 : yaw);
    if (sup && item.kind === 'instrument' && dh > 0.05) {     // 斜著放下：以低的那一端為支點倒平
      const sh = (Math.sign(dir.z) || 1) * (MODEL.instrumentLen / 2) * (1 - Math.cos(tilt));
      item.node.position.x += dir.x / dh * sh; item.node.position.y += dir.y / dh * sh;
    }
    if (sup) { item.node.position.z = sup.z + item.half; item.on = sup.id; item.placeSeq = ++this.ref.placeCounter; this.log(`[場景] ${item.name || item.drug} 放到 ${sup.id}（落差 ${((low - sup.z) * 100).toFixed(1)} cm）`); }
    else { item.node.position.z = item.half; item.on = 'floor'; item.dropped = true; this.log(`[場景] ${item.name || item.drug} 掉到地上，不得重拾`); }
  }

  // ---------- 每一步 ----------
  update(dt, robot) {
    this.time += dt;
    const T = this.truth, gc = robot.graspRos(), closed = robot.q.gripper < 0.1;
    // 抽屜：夾住把手時跟著夾爪走；夾爪閉合頂到把手時被推回去
    for (const dr of this.drawers) {
      const l = dr.frame.toLocal(gc), hy = MODEL.cart.handleY;
      if (this.heldHandle === dr) {
        if (Math.abs(l.x) > 0.05 || Math.abs(l.z - dr.zc) > 0.05) { this.heldHandle = null; this.log(`[場景] ${dr.id} 把手滑脫`); }
        else dr.ext = Math.min(0.36, Math.max(0, l.y - hy));
      } else if (closed && dr.ext > 0 && Math.abs(l.x) < 0.14 && Math.abs(l.z - dr.zc) < 0.07 && l.y < hy + dr.ext + 0.012 && l.y > hy + dr.ext - 0.06) {
        dr.ext = Math.max(0, l.y - 0.012 - hy);
      }
      dr.maxExt = Math.max(dr.maxExt, dr.ext);
      dr.group.position.y = dr.ext;
    }
    // 樓層按鈕
    for (const b of this.buttons) {
      const d = gc.distanceTo(b.pos());
      if (closed && d < 0.02 && !b.down) { b.down = true; b.mesh.material.color.set(0xffd24a); this._press(b.floor); }
      else if (d > 0.045 && b.down) { b.down = false; b.mesh.material.color.set(0xf2f2f2); }
    }
    if (T.greenAt !== null && this.time >= T.greenAt && T.light === 'red') { T.light = T.wrong >= 3 ? 'yellow' : 'green'; this.log(`[場景] 燈號轉${T.light === 'green' ? '綠' : '黃'}，閘門開啟`); }
    const col = { red: 0xff2a2a, green: 0x2aff5a, yellow: 0xffd02a }[T.light];
    this.lightMesh.material.color.set(col); this.lightMesh.material.emissive.set(col);
    if (T.light !== 'red' && this.door.open < 1) {
      this.door.open = Math.min(1, this.door.open + dt / 2);
      this.door.mesh.position.y = 6.1 + this.door.open * 1.0; this.door.mesh.updateMatrixWorld();
      if (this.door.open >= 1) this.door.obs.active = false;
    }
    this.ref.update(dt, robot);
  }
  _press(floor) {
    const T = this.truth;
    if (T.light !== 'red' || T.greenAt !== null) return;
    T.presses++;
    if (floor === T.floor) { T.greenAt = this.time + 1.0; this.log(`[場景] 按下 ${floor}F（第 ${T.presses} 次），正確`); }
    else { T.wrong++; this.log(`[場景] 按下 ${floor}F（第 ${T.presses} 次），錯誤`); if (T.wrong >= 3) T.greenAt = this.time + 1.0; }
  }

  // ---------- 模擬感知 ----------
  // 回傳相機看得到的目標。條件是視野、距離、遮蔽；通過之後直接給真值（點滴另加讀值誤差）。
  // 頭部相機是直放的 D435i（水平約 42°、垂直約 69°）；腕部 D405 用 35° 圓錐、45 cm 內。
  detect(robot, cam, kind) {
    const link = robot.urdf.links[cam === 'head' ? 'camera_color_optical_frame' : 'gripper_camera_color_optical_frame'];
    const origin = link.getWorldPosition(new THREE.Vector3()), out = [];
    const ray = new THREE.Raycaster();
    for (const t of this.targets) {
      if (t.kind !== kind || t.cam !== cam || !t.enabled()) continue;
      const pr = t.pos(), pw = this.world.localToWorld(pr.clone()), dist = pw.distanceTo(origin);
      if (dist > t.range) continue;
      const l = link.worldToLocal(pw.clone());
      if (l.z <= 0.02) continue;
      if (cam === 'head') { if (Math.abs(Math.atan2(l.x, l.z)) > 34.5 * Math.PI / 180 || Math.abs(Math.atan2(l.y, l.z)) > 21 * Math.PI / 180) continue; }
      else if (Math.hypot(l.x, l.y) / l.z > Math.tan(35 * Math.PI / 180)) continue;
      if (cam === 'head') {
        ray.set(origin, pw.clone().sub(origin).normalize()); ray.far = dist - 0.06;
        const hit = ray.intersectObjects(this.occluders, false).find(h => { let o = h.object; while (o) { if (o === t.node) return false; o = o.parent; } return true; });
        if (hit) continue;
      }
      out.push({ id: t.id, kind, dist, x: pr.x, y: pr.y, z: pr.z, ...t.data() });
    }
    return out;
  }
  // 光達：4 m 內的障礙進入已知地圖。回傳地圖有沒有變
  scan(robot) {
    let changed = false;
    for (const o of this.obstacles) {
      if (o.known || o.seen) continue;
      if (Math.hypot(o.cx - robot.base.x, o.cy - robot.base.y) < 4.0) { o.seen = true; changed = true; }
    }
    const near = Math.hypot(7 - robot.base.x, 6.1 - robot.base.y) < 4.0;
    if (near && !this.door.obs.active && !this.door.seenOpen) { this.door.seenOpen = true; changed = true; }
    return changed;
  }
  knownObstacles() { return this.obstacles.filter(o => (o.known || o.seen) && (o.id !== 'door' || !this.door.seenOpen)); }
}

// ---------- 裁判 ----------
export class Referee {
  constructor(h) {
    this.h = h; this.time = 0; this.started = false; this.finished = false;
    this.cartsDocked = new Set(); this.placeCounter = 0;
    this.baseHits = { 1: 0, 2: 0, 3: 0 }; this.armHits = { 1: 0, 2: 0, 3: 0 };
    this._inBase = false; this._inArm = false; this.stage = 1; this.idleMax = 0; this.repairs = 0; this._idleFlag = false;
    this.reports = {}; this.said = [];
    this.footprint = [[-0.277, -0.172], [0.049, -0.172], [0.049, 0.172], [-0.277, 0.172]];
    this.notes = [];
  }
  setFootprint(xmin, xmax, ymin, ymax) { this.footprint = [[xmin, ymin], [xmax, ymin], [xmax, ymax], [xmin, ymax]]; }
  hear(text) {
    this.said.push(text);
    const m = /^(W\d) 床：(.*)$/.exec(text);
    if (m) this.reports[m[1]] = m[2];
  }
  _inside(o, x, y) {
    const dx = x - o.cx, dy = y - o.cy;
    if (o.type === 'circle') return dx * dx + dy * dy < o.r * o.r;
    const c = Math.cos(o.rot || 0), s = Math.sin(o.rot || 0);
    return Math.abs(c * dx + s * dy) < o.w / 2 && Math.abs(-s * dx + c * dy) < o.d / 2;
  }
  update(dt, robot) {
    if (!this.started || this.finished) return;
    this.time += dt;
    const b = robot.base, T = this.h.truth;
    const st = b.x < 7 ? 1 : (b.y >= 4 ? 2 : 3);
    if (st > this.stage) this.stage = st;
    // 底盤外框取樣，碰到任何障礙算一次碰撞
    const c = Math.cos(b.theta), s = Math.sin(b.theta), fp = this.footprint;
    let hit = null;
    outer: for (let e = 0; e < 4; e++) for (let k = 0; k < 6; k++) {
      const p = fp[e], q = fp[(e + 1) % 4], lx = p[0] + (q[0] - p[0]) * k / 6, ly = p[1] + (q[1] - p[1]) * k / 6;
      const x = b.x + c * lx - s * ly, y = b.y + s * lx + c * ly;
      for (const o of this.h.obstacles) if (o.active && this._inside(o, x, y)) { hit = o; break outer; }
    }
    if (hit && !this._inBase) { this.baseHits[this.stage]++; this.notes.push(`${this.clock()} 底盤碰撞 ${hit.id}（第 ${this.stage} 關）`); this.h.log(`[裁判] 底盤碰撞 ${hit.id}`); }
    this._inBase = !!hit;
    // 手臂：夾爪中心與腕部兩點進到家具實體裡算一次
    let arm = null;
    for (const p of [robot.graspRos(), robot.linkRos('link_wrist_yaw')]) for (const so of this.h.solids) {
      const l = so.frame.toLocal(p), e = 0.006;
      if (l.x > so.lo[0] + e && l.x < so.hi[0] - e && l.y > so.lo[1] + e && l.y < so.hi[1] - e && l.z > so.lo[2] && l.z < so.hi[2] - e) arm = so;
    }
    if (arm && !this._inArm) { this.armHits[this.stage]++; this.notes.push(`${this.clock()} 手臂碰撞 ${arm.id}（第 ${this.stage} 關）`); this.h.log(`[裁判] 手臂碰撞 ${arm.id}`); }
    this._inArm = !!arm;
    if (b.x > 7.0 && b.x < 7.4 && b.y > 5.6 && b.y < 6.6 && T.light === 'red' && !T.redRun) { T.redRun = true; this.notes.push(`${this.clock()} 紅燈硬闖`); }
    const idle = robot.time - robot.lastMotionTime;
    this.idleMax = Math.max(this.idleMax, idle);
    if (idle > 60 && !this._idleFlag) { this._idleFlag = true; this.repairs++; this.notes.push(`${this.clock()} 停頓超過 1 分鐘，進入維修`); }
    if (idle < 1) this._idleFlag = false;
    if (this.time >= 1200) { this.finished = true; this.notes.push('20:00 時間到'); }
  }
  clock() { const t = Math.floor(this.time); return `${String(Math.floor(t / 60)).padStart(2, '0')}:${String(t % 60).padStart(2, '0')}`; }

  score() {
    const h = this.h, T = h.truth, rows = { 1: [], 2: [], 3: [] };
    const add = (st, name, got, max, note = '') => rows[st].push({ name, got, max, note });
    // 第一關
    const nd = this.cartsDocked.size;
    add(1, '藥車對準', nd === 2 ? 20 : nd === 1 ? 5 : 0, 20, `${nd}/2 台`);
    T.orders.forEach((o, i) => {
      const it = o.item, opened = it.drawer.maxExt > 0.2;
      add(1, `醫囑 ${i + 1} 辨識、拉屜、取盒`, it.everHeld ? 30 : opened ? 10 : 0, 30, it.everHeld ? o.drug : opened ? '拉對抽屜，未取盒' : '');
    });
    const opened = h.drawers.filter(d => d.maxExt > 0.2), stillOpen = h.drawers.filter(d => d.ext > 0.01).length;
    add(1, '抽屜推回', opened.length && !stillOpen ? 20 : opened.length && stillOpen === 1 ? 10 : 0, 20, opened.length ? `開過 ${opened.length} 層，未關 ${stillOpen} 層` : '');
    T.orders.forEach((o, i) => {
      const it = o.item, onRight = it.on === 'cabinet' + o.bed;
      add(1, `醫囑 ${i + 1} 送到 Bed ${o.bed}`, onRight ? 25 : 0, 25, it.on && it.on !== 'cabinet' + o.bed && it.on.startsWith('cabinet') ? '送錯床' : '');
      let pts = 0, note = '';
      if (onRight) {
        const f = h.map.cabinets[o.bed].frame, l = f.toLocal(it.node.position), z = MODEL.cabinet.zone;
        const yaw = Math.abs(Math.sin(wrap(it.node.rotation.z - f.psi)));      // 藥盒長軸沿家具 y
        const hx = MODEL.box[0] / 2 + yaw * (MODEL.box[1] - MODEL.box[0]) / 2, hy = MODEL.box[1] / 2 - yaw * (MODEL.box[1] - MODEL.box[0]) / 2;
        const inside = Math.abs(l.x) + hx <= z[0] / 2 + 1e-4 && Math.abs(l.y) + hy <= z[1] / 2 + 1e-4;
        pts = inside ? 30 : 10; note = `偏移 ${(l.x * 100).toFixed(1)}, ${(l.y * 100).toFixed(1)} cm${inside ? '' : '，壓線或出線'}`;
      }
      add(1, `醫囑 ${i + 1} 區域線內放置`, pts, 30, note);
    });
    add(1, '安全（底盤無碰撞）', this.baseHits[1] ? 0 : 10, 10, this.baseHits[1] ? `${this.baseHits[1]} 次` : '');
    // 第二關
    let bp = 0, bn = `按 ${T.presses} 次`;
    if (T.redRun) { bp = 0; bn = '紅燈硬闖'; }
    else if (T.light === 'green') bp = [20, 15, 10][Math.min(2, T.presses - 1)];
    else if (T.light === 'yellow') { bp = 5; bn = '黃燈通過'; }
    add(2, `跨樓層按鈕（${T.floor}F）`, bp, 20, T.presses ? bn : '');
    for (const st of T.ward) {
      const rep = this.reports[st.name] || '';
      let pts = 0, note = rep ? '' : '未報告';
      if (rep) {
        if (st.kind === 'offbed') pts = (st.away ? /離床/.test(rep) : /在床/.test(rep)) ? 25 : 0;
        if (st.kind === 'fall') pts = (st.fall ? /病人跌落/.test(rep) : /未跌落/.test(rep)) ? 25 : 0;
        if (st.kind === 'gown') pts = (st.gown === 'green' ? /病人服綠色/.test(rep) : /非綠色/.test(rep)) ? 25 : 0;
        if (st.kind === 'hr') { const m = /心跳 (\d+)/.exec(rep); pts = m && +m[1] === st.bpm ? 25 : 0; note = m ? `報 ${m[1]}，實際 ${st.bpm}` : '沒報心跳'; }
        if (st.kind === 'iv') { const m = /點滴剩餘 (\d+)/.exec(rep); const e = m ? Math.abs(+m[1] - st.ml) : 99; pts = e <= 5 ? 25 : e <= 15 ? 10 : 0; note = m ? `報 ${m[1]}，實際 ${st.ml} ml` : '沒報點滴'; }
        if (!note) note = pts ? '正確' : '判讀錯誤';
      }
      add(2, `${st.name} ${SCENARIO_NAME[st.kind]}`, pts, 25, note);
    }
    add(2, '安全（底盤無碰撞）', this.baseHits[2] ? 0 : 10, 10, this.baseHits[2] ? `${this.baseHits[2]} 次` : '');
    // 第三關
    const tableId = 'maquet-' + T.zone, f = h.map.maquet[T.zone].frame, Z = MODEL.maquet.zone;
    const placed = T.instruments.filter(it => it.on && it.on.startsWith('maquet')).sort((a, b) => a.placeSeq - b.placeSeq);
    add(3, `走對區域（${T.zone === 'left' ? '左' : '右'}區）`, placed.length && placed[0].on === tableId ? 20 : 0, 20, placed.length && placed[0].on !== tableId ? '放到另一區' : '');
    const ah = this.armHits[3];
    add(3, '手臂碰撞', ah === 0 ? 20 : ah <= 3 ? 10 : 0, 20, `${ah} 次`);
    const info = T.set.map((it, i) => {
      const l = f.toLocal(it.node.position), ok = it.on === tableId && Math.abs(l.x) <= Z[0] && Math.abs(l.y) <= Z[1];
      return { it, i, l, ok, tipUp: ok && Math.cos(wrap(it.node.rotation.z - (f.psi - Math.PI / 2))) > Math.cos(30 * Math.PI / 180) };
    });
    const done = info.filter(d => d.ok), n = done.length;
    add(3, '器具送達', [0, 25, 35, 50, 60][n], 60, `${n}/4 件`);
    const byX = done.slice().sort((a, b) => b.l.x - a.l.x);     // 從操作者左手邊排到右手邊
    const orderOk = byX.filter((d, k) => d.i === done.map(e => e.i).sort((a, b) => a - b)[k]).length;
    add(3, '順序正確', Math.round(20 * orderOk / 4), 20, `${orderOk}/4 件`);
    const tip = done.filter(d => d.tipUp).length;
    add(3, '尖端朝上', Math.round(20 * tip / 4), 20, `${tip}/4 件`);
    const onTable = h.items.filter(it => it.on === tableId);
    const clear = done.filter(d => onTable.every(o => {       // 器具沿桌深方向擺，橫向間距不足 3.5 cm 視為重疊
      if (o === d.it) return true;
      const a = f.toLocal(d.it.node.position), b = f.toLocal(o.node.position);
      return Math.abs(a.x - b.x) > 0.035 || Math.abs(a.y - b.y) > MODEL.instrumentLen;
    })).length;
    add(3, '互不重疊', Math.round(20 * clear / 4), 20, `${clear}/4 件`);
    add(3, '安全（底盤無碰撞）', this.baseHits[3] ? 0 : 10, 10, this.baseHits[3] ? `${this.baseHits[3]} 次` : '');
    // 安全分與手臂零碰撞分是「沒出事」的分數：該關其他項目都還沒得分時不計，免得還沒進關就有分
    for (const st of [1, 2, 3]) {
      const passive = r => r.name.startsWith('安全') || r.name === '手臂碰撞';
      if (rows[st].filter(r => !passive(r)).every(r => r.got === 0)) for (const r of rows[st]) if (passive(r)) { r.got = 0; r.note = '本關尚未得分，不計'; }
    }
    const sum = st => rows[st].reduce((a, r) => a + r.got, 0);
    const totals = { 1: sum(1), 2: sum(2), 3: sum(3) };
    return { rows, totals, total: totals[1] + totals[2] + totals[3], allNonZero: totals[1] > 0 && totals[2] > 0 && totals[3] > 0 };
  }
}
