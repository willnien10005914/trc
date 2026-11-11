// 自動過三關的任務腳本。
// 這一層只做三件事：呼叫 RobotClient（robot.js，方法名同 stretch_ai）、
// 讀事先給的地圖與家具模型（hospital.map、MODEL）、向模擬感知要相機看得到的東西（hospital.detect）。
// 它不讀場景的真值；抽籤結果、醫囑、樓層、病房情境、器具位置都要自己看出來。
import * as THREE from 'three';
import { AStar } from './planner.js';
import { ARENA, MODEL, Frame } from './scene.js';

const V3 = THREE.Vector3;
const wrap = a => Math.atan2(Math.sin(a), Math.cos(a));
const DOWN = -1.57;     // 俯抓
const PICK = -1.2;      // 抽屜內取盒、床旁櫃放盒：俯抓搆不到 62 cm 外，腕稍微抬起
const PLACE = -0.55;    // 96 cm 器具桌：俯抓最高只到 95 cm，只能斜持放下

export class Mission {
  constructor(robot, hospital) {
    this.robot = robot; this.h = hospital;
    this.planner = new AStar(ARENA.W, ARENA.H, 0.05);
    this.inflate = 0.36;      // 底盤外接圓半徑（含導航姿態的夾爪）加餘裕
    this.path = []; this.failures = []; this.stage = 0; this.done = false;
  }
  note(msg) { this.robot._log('# ' + msg); }
  fail(e) { this.failures.push(e.message); this.note('✗ ' + e.message); }

  // ---------- 導航：A* 規劃，逐段 move_base_to，光達看到新障礙就重排 ----------
  async navigate(goal, label) {
    const r = this.robot, h = this.h;
    if (!r.in_navigation_mode()) await r.move_to_nav_posture();
    this.note(`導航到${label}`);
    for (let attempt = 0; attempt < 40; attempt++) {
      h.scan(r);
      this.planner.build(h.knownObstacles(), this.inflate);
      const path = this.planner.plan([r.base.x, r.base.y], goal);
      if (!path) throw new Error(`A* 找不到到${label}的路徑`);
      const pts = [];
      for (let i = 1; i < path.length; i++) {
        const a = path[i - 1], b = path[i], n = Math.max(1, Math.ceil(Math.hypot(b[0] - a[0], b[1] - a[1]) / 1.0));
        for (let k = 1; k <= n; k++) pts.push([a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n]);
      }
      this.path = [[r.base.x, r.base.y], ...pts];
      let replan = false;
      for (let i = 0; i < pts.length; i++) {
        if (h.scan(r)) { replan = true; break; }
        const p = pts[i], last = i === pts.length - 1;
        const far = Math.hypot(p[0] - r.base.x, p[1] - r.base.y) > 0.05;
        await r.move_base_to([p[0], p[1], last ? goal[2] : (far ? Math.atan2(p[1] - r.base.y, p[0] - r.base.x) : r.base.theta)]);
        this.path = [[r.base.x, r.base.y], ...pts.slice(i + 1)];
      }
      if (!replan) { this.path = []; return; }
      this.note('光達看到新障礙，重新規劃');
    }
    throw new Error(`到${label}途中重排次數過多`);
  }

  // ---------- 感知 ----------
  async look_at(p) {
    const r = this.robot, tb = r.urdf.worldToLocal(r.world.localToWorld(p.clone()));
    let pan = Math.atan2(tb.y, tb.x);
    if (pan > 1.5) pan -= 2 * Math.PI;
    const tilt = Math.atan2(tb.z - 1.313, Math.hypot(tb.x, tb.y)) + 0.092;    // 相機光軸在 tilt=0 時略朝下
    await r.head_to(pan, Math.min(0, tilt));
  }
  see(cam, kind) {
    const out = this.h.detect(this.robot, cam, kind);
    this.robot._log(`obs = robot.get_${cam === 'head' ? 'head' : 'ee'}_rgbd()`, `模擬感知 ${kind}：${out.length ? out.length + ' 個' : '沒看到'}`);
    return out;
  }
  // 用 ArUco 求家具的實際座標系
  async locate(marker, nominal, local) {
    await this.look_at(nominal.toWorld(...local));
    const d = this.see('head', 'aruco').find(a => a.marker === marker);
    if (!d) throw new Error(`看不到 ArUco ${marker}`);
    const psi = d.yaw - Math.PI / 2, c = Math.cos(psi), s = Math.sin(psi);
    return new Frame(d.x - (c * local[0] - s * local[1]), d.y - (s * local[0] + c * local[1]), psi);
  }

  // ---------- 手臂 ----------
  async reach(p, { pitch = 0, roll = 0, liftFirst = false } = {}) {
    const r = this.robot, ik = r.ik_reach(p, { pitch, roll });
    if (!ik.ok) throw new Error(`逆解超出行程（lift ${ik.lift.toFixed(2)} m、arm ${ik.arm.toFixed(2)} m）`);
    if (liftFirst) { const cur = r.get_six_joints(); await r.arm_to([cur[0], ik.q[1], cur[2], roll, pitch, 0]); }
    await r.arm_to(ik.q);
  }
  async retract() { const q = this.robot.get_six_joints(); q[2] = 0.01; await this.robot.arm_to(q); }
  // 俯抓時，兩指連線要轉到 yaw 方向所需的腕部 roll
  rollForFinger(yaw, pitch) {
    const r = this.robot, saved = { ...r.q };
    r.q.wrist_pitch = pitch; r.q.wrist_roll = 0; r.apply(); const f0 = r.fingerYaw();
    r.q.wrist_roll = 0.2; r.apply(); const s = Math.sign(wrap(r.fingerYaw() - f0)) || 1;
    r.q = saved; r.apply();
    let roll = wrap((yaw - f0) * s);
    if (roll > Math.PI / 2) roll -= Math.PI;
    if (roll < -Math.PI / 2) roll += Math.PI;
    return roll;
  }
  // 手上器具的尖端方向（夾爪座標系），用夾取當下看到的方位與自身運動學推得
  rememberTip(yaw) {
    const r = this.robot, link = r.urdf.links.link_grasp_center;
    const wq = r.world.getWorldQuaternion(new THREE.Quaternion()), lq = link.getWorldQuaternion(new THREE.Quaternion());
    this.tipLocal = new V3(Math.cos(yaw), Math.sin(yaw), 0).applyQuaternion(wq).applyQuaternion(lq.invert());
  }
  rollForTip(yaw, pitch) {
    const r = this.robot, link = r.urdf.links.link_grasp_center, saved = { ...r.q };
    const wqi = r.world.getWorldQuaternion(new THREE.Quaternion()).invert(), lq = new THREE.Quaternion();
    let best = 0, bv = -2;
    r.q.wrist_pitch = pitch;
    for (let a = -3.1; a <= 3.1; a += 0.02) {
      r.q.wrist_roll = a; r.apply();
      const d = this.tipLocal.clone().applyQuaternion(link.getWorldQuaternion(lq)).applyQuaternion(wqi);
      const v = d.x * Math.cos(yaw) + d.y * Math.sin(yaw);
      if (v > bv) { bv = v; best = a; }
    }
    r.q = saved; r.apply();
    return best;
  }

  // ---------- 總流程 ----------
  async run() {
    const r = this.robot, h = this.h;
    h.ref.started = true;
    for (const [n, fn] of [[1, this.stage1], [2, this.stage2], [3, this.stage3]]) {
      this.stage = n;
      try { await fn.call(this); } catch (e) { if (e.message === 'stopped') throw e; this.fail(e); await this.recover(); }
    }
    r.say('三關任務結束。');
    h.ref.finished = true; this.done = true;
  }
  async recover() {
    const r = this.robot;
    try { if (this.h.holdingHandle()) await r.open_gripper(); await r.move_to_nav_posture(); } catch (e) { if (e.message === 'stopped') throw e; }
  }

  // ---------- 第一關：藥物配送 ----------
  async stage1() {
    const r = this.robot, map = this.h.map;
    r.say('第一關，藥物配送，開始。');
    await r.move_to_nav_posture();
    await this.look_at(map.tv1);
    const tv = this.see('head', 'tv')[0];
    if (!tv) throw new Error('看不到醫囑螢幕');
    const orders = tv.lines.map(l => /^\d\.\s+Bed (\d) :\s+(.+?)\s+\( Cart (\w) \)$/.exec(l)).filter(Boolean).map(m => ({ bed: +m[1], drug: m[2], cart: m[3] }));
    r.say(`讀到 ${orders.length} 筆醫囑。`);
    for (const o of orders) {
      try { await this.deliver(o); } catch (e) { if (e.message === 'stopped') throw e; this.fail(new Error(`Bed ${o.bed} ${o.drug}：${e.message}`)); await this.recover(); }
    }
  }
  async deliver(o) {
    const r = this.robot, h = this.h, map = h.map, C = MODEL.cart, K = MODEL.cabinet;
    const cart = map.carts.find(c => c.name === o.cart), barcode = map.formulary[o.drug];
    if (!cart || !barcode) throw new Error('醫囑對不到藥車或處方集');
    await this.navigate(cart.frame.dock(0, 1.6), `藥車 ${o.cart} 前`);
    const f = await this.locate(cart.marker, cart.frame, C.markerLocal);
    await this.navigate(f.dock(0, C.dockY), `藥車 ${o.cart} 停靠點`);
    await r.move_to_manip_posture();
    let got = false;
    for (let li = 0; li < C.drawerZ.length && !got; li++) {
      const zc = C.drawerZ[li], out = C.handleY + C.stroke;
      this.note(`拉開藥車 ${o.cart} ${li ? '下層' : '上層'}抽屜`);
      await r.open_gripper();
      await this.reach(f.toWorld(0, C.handleY + 0.10, zc), { liftFirst: true });
      await this.reach(f.toWorld(0, C.handleY, zc));
      await r.close_gripper();
      if (!h.holdingHandle()) { this.note('沒夾到把手，換下一層'); continue; }
      await this.reach(f.toWorld(0, out, zc));                       // 收臂把抽屜拉出
      await r.open_gripper();
      await this.reach(f.toWorld(0, out, zc + 0.22));                // 抬高離開把手
      const tray = f.toWorld(0, C.boxLocalY + C.stroke, zc + C.boxDz);
      await this.reach(tray.clone().setZ(tray.z + 0.15), { pitch: PICK });
      const hit = this.see('ee', 'barcode').find(c => c.barcode === barcode);   // 腕部相機讀條碼
      if (hit) {
        this.note(`條碼 ${barcode} 對到 ${o.drug}`);
        await this.reach(new V3(hit.x, hit.y, hit.z + 0.10), { pitch: PICK });
        await this.reach(new V3(hit.x, hit.y, hit.z), { pitch: PICK });
        await r.close_gripper();
        got = !!h.holding() && h.holding().barcode === barcode;
        await this.reach(new V3(hit.x, hit.y, hit.z + 0.16), { pitch: PICK });
      } else { this.note('這層沒有指定藥盒'); await r.close_gripper(); }
      this.note('把抽屜推回');
      await this.reach(f.toWorld(0, out + 0.04, zc + 0.20));
      await this.reach(f.toWorld(0, out + 0.04, zc));
      await this.reach(f.toWorld(0, C.handleY + 0.012, zc));
      await this.reach(f.toWorld(0, C.handleY + 0.08, zc));
    }
    await r.move_to_nav_posture();
    if (!got) throw new Error('沒取到藥盒');
    const cab = map.cabinets[o.bed];
    await this.navigate(cab.frame.dock(0, 1.3), `Bed ${o.bed} 床旁櫃前`);
    const fc = await this.locate(cab.marker, cab.frame, K.markerLocal);
    await this.navigate(fc.dock(0, K.dockY), `Bed ${o.bed} 床旁櫃停靠點`);
    await r.move_to_manip_posture();
    const z = K.h + MODEL.box[2] / 2 + 0.006;
    await this.reach(fc.toWorld(0, 0, z + 0.09), { pitch: PICK, liftFirst: true });
    await this.reach(fc.toWorld(0, 0, z), { pitch: PICK });
    await r.open_gripper();
    await this.reach(fc.toWorld(0, 0, z + 0.09), { pitch: PICK });
    await r.move_to_nav_posture();
    r.say(`Bed ${o.bed} 的 ${o.drug} 已放到床旁櫃。`);
  }

  // ---------- 第二關：跨樓層按鈕與病房巡檢 ----------
  async stage2() {
    const r = this.robot, g = this.h.map.gate, G = MODEL.gate;
    r.say('第二關，病房巡檢。');
    await this.navigate(g.frame.dock(0, 1.3), '樓層按鈕前');
    const f = await this.locate(g.marker, g.frame, G.markerLocal);
    await this.look_at(f.toWorld(...G.signLocal));
    const sign = this.see('head', 'sign')[0], m = sign && /(\d)F/.exec(sign.text);
    if (!m) throw new Error('讀不到樓層指示');
    const floor = +m[1];
    await this.navigate(f.dock(0, G.dockY), '按鈕停靠點');
    await r.move_to_manip_posture();
    await r.close_gripper();
    let color = 'red';
    for (let tries = 0; tries < 3 && color === 'red'; tries++) {
      const z = G.buttonZ[floor];
      await this.reach(f.toWorld(0, 0.08, z), { liftFirst: true });
      await this.reach(f.toWorld(0, 0.004, z));                      // 指尖點按
      await this.reach(f.toWorld(0, 0.08, z));
      await r.sleep(1.5);
      await this.look_at(f.toWorld(...G.lightLocal));
      const l = this.see('head', 'light')[0];
      color = l ? l.color : 'red';
    }
    await r.move_to_nav_posture();
    if (color === 'red') { r.say('燈號仍是紅燈，不通過。'); throw new Error('跨樓層按鈕未成功，紅燈不通過'); }
    r.say(`${floor} 樓按鈕完成，燈號${color === 'green' ? '綠燈' : '黃燈'}，通過。`);
    await r.sleep(2.2);                                               // 等閘門全開
    await this.navigate([g.through[1][0], g.through[1][1], 0], '病房入口');
    for (const bed of this.h.map.ward) {
      try { await this.inspect(bed); } catch (e) { if (e.message === 'stopped') throw e; this.fail(new Error(`${bed.name}：${e.message}`)); await this.recover(); }
    }
  }
  async inspect(bed) {
    const r = this.robot, f = bed.frame, W = MODEL.wardBed, face = f.psi - Math.PI / 2;
    const foot = f.toWorld(0, W.footY);
    await this.navigate([foot.x, foot.y, face], `${bed.name} 床尾`);
    const seen = { tag: false, patient: null, monitor: false, iv: false };
    const mine = d => Math.abs(f.toLocal(new V3(d.x, d.y, d.z)).x) <= 0.9;     // 只算這個床位範圍內的東西
    for (const [pan, tilt] of [[0, -0.35], [0, -0.95], [0.5, -0.4], [-0.5, -0.4]]) {   // 掃視，對照 stretch.app.head_sweep
      await r.head_to(pan, tilt);
      if (this.see('head', 'aruco').some(a => a.marker === bed.marker)) seen.tag = true;
      for (const p of this.see('head', 'patient')) if (mine(p)) seen.patient = p;
      if (this.see('head', 'monitor').some(mine)) seen.monitor = true;
      if (this.see('head', 'iv').some(mine)) seen.iv = true;
    }
    if (!seen.tag) throw new Error('床尾標籤沒讀到，不報告以免認錯床');
    const parts = [];
    if (!seen.patient) parts.push('病人離床');
    else if (seen.patient.state === 'on_floor') parts.push('病人跌落');
    else parts.push('病人在床', '未跌落', seen.patient.gown === 'green' ? '病人服綠色' : '病人服非綠色');
    if (seen.monitor) {
      const p = f.toWorld(W.monitorLocal[0], W.readY);
      await this.navigate([p.x, p.y, face], `${bed.name} 監視器前`);
      await this.look_at(f.toWorld(W.monitorLocal[0], W.monitorLocal[1] + 0.1, W.monitorLocal[2]));
      const hr = this.see('head', 'hr')[0];
      if (hr) parts.push(`心跳 ${hr.bpm} bpm`); else this.note('監視器數字讀不到');
    }
    if (seen.iv) {
      const p = f.toWorld(W.ivLocal[0], W.readY);
      await this.navigate([p.x, p.y, face], `${bed.name} 點滴架前`);
      await this.look_at(f.toWorld(W.ivLocal[0], W.ivLocal[1] + 0.05, W.ivLocal[2]));
      const iv = this.see('head', 'ivlevel')[0];
      if (iv) parts.push(`點滴剩餘 ${iv.ml} ml`); else this.note('點滴刻度讀不到');
    }
    r.say(`${bed.name} 床：${parts.join('，')}。`);
  }

  // ---------- 第三關：手術器具 ----------
  async stage3() {
    const r = this.robot, map = this.h.map, Q = MODEL.maquet;
    r.say('第三關，手術器具。');
    await this.navigate(map.tv3view, '第三關螢幕前');
    await this.look_at(map.tv3);
    const tv = this.see('head', 'tv')[0], zm = tv && /(LEFT|RIGHT) ZONE/.exec(tv.lines[0]);
    if (!zm) throw new Error('看不到器具螢幕');
    const zone = zm[1].toLowerCase(), names = tv.lines.slice(1).map(l => l.replace(/^\d\.\s+/, ''));
    r.say(`指定${zone === 'left' ? '左' : '右'}區，器具 ${names.length} 件。`);
    await this.navigate(map.source.scan, '器具桌掃描點');
    const found = {};
    for (const pan of [-0.75, -0.38, 0, 0.38, 0.75]) {
      await r.head_to(pan, -0.38);
      for (const d of this.see('head', 'instrument')) found[d.name] = d;
    }
    this.note(`器具桌上認出 ${Object.keys(found).length} 種器具`);
    const mq = map.maquet[zone];
    await this.navigate(mq.frame.dock(0, 1.25), `${zone === 'left' ? '左' : '右'}區器具桌前`);
    const fm = await this.locate(mq.marker, mq.frame, Q.markerLocal);
    for (let k = 0; k < names.length; k++) {
      const d = found[names[k]];
      if (!d) { this.fail(new Error(`器具桌上沒認出 ${names[k]}`)); continue; }
      try { await this.moveInstrument(d, fm, k); } catch (e) {
        if (e.message === 'stopped') throw e;
        this.fail(new Error(`${names[k]}：${e.message}`));
        try { await this.retract(); await r.open_gripper(); } catch (e2) { if (e2.message === 'stopped') throw e2; }
        await this.recover();
      }
    }
    r.say('器具擺放完成。');
  }
  async moveInstrument(d, fm, k) {
    const r = this.robot, h = this.h, S = MODEL.source, Q = MODEL.maquet, fs = h.map.source.frame;
    const north = fs.toLocal(new V3(d.x, d.y, 0)).y > 0;
    const side = north ? fs : new Frame(fs.x, fs.y, fs.psi + Math.PI);          // 器具靠哪一側就從哪一側靠近
    const lx = side.toLocal(new V3(d.x, d.y, 0)).x;
    await this.navigate(side.dock(lx + 0.02, S.dockY), `器具桌 ${d.name}`);
    await r.move_to_manip_posture();
    await r.open_gripper();
    const roll = this.rollForFinger(d.yaw + Math.PI / 2, DOWN);                 // 兩指跨過器具的寬度
    await this.reach(new V3(d.x, d.y, d.z + 0.10), { pitch: DOWN, roll, liftFirst: true });
    await this.reach(new V3(d.x, d.y, d.z + 0.003), { pitch: DOWN, roll });
    await r.close_gripper();
    if (!h.holding() || h.holding().name !== d.name) throw new Error('沒夾到器具');
    this.rememberTip(d.yaw);
    await this.reach(new V3(d.x, d.y, d.z + 0.12), { pitch: DOWN, roll });
    await r.move_to_nav_posture();
    const sx = Q.slotX[k];
    await this.navigate(fm.dock(sx + 0.02, Q.dockY), `器具桌第 ${k + 1} 格`);
    await r.move_to_manip_posture();
    if (!h.holding()) throw new Error('途中掉落');
    // 尖端朝上 = 朝桌子深處。斜持時器具與桌面夾角是 90° 減腕的俯角，放開後以低端為支點倒平，
    // 中心會往尖端方向移 (L/2)(1-cos)，所以瞄準點先往回補。
    const up = fm.psi - Math.PI / 2, tilt = Math.PI / 2 + PLACE, L = MODEL.instrumentLen;
    const rollP = this.rollForTip(up, PLACE);
    const back = (L / 2) * (1 - Math.cos(tilt));
    const target = fm.toWorld(sx, Q.slotY + back, Q.h + (L / 2) * Math.sin(tilt) + 0.012);
    await this.reach(target, { pitch: PLACE, roll: rollP, liftFirst: true });
    await r.open_gripper();
    await this.retract();
    await r.move_to_nav_posture();
    r.say(`第 ${k + 1} 件 ${d.name} 已放好。`);
  }
}
