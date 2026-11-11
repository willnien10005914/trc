// Stretch 的網頁模擬版 RobotClient。
// 方法名稱、參數順序與預設姿態對照 stretch_ai：
//   src/stretch/agent/zmq_client.py      (RobotClient 的方法)
//   src/stretch/motion/constants.py      (STRETCH_NAVIGATION_Q、STRETCH_PREGRASP_Q、look_* 、關節名)
//   src/stretch/motion/kinematics.py     (GRIPPER_OPEN / GRIPPER_CLOSED)
//   src/stretch/config/control/motion_profiles.yaml (關節速度)
// 這裡只有運動學，沒有動力學、沒有感測噪聲。
import * as THREE from 'three';
import URDFLoader from '../vendor/URDFLoader.js';

export const URDF_URL = '../stretch_ai/src/stretch/config/urdf/stretch.urdf';
export const MESH_PKG = '../stretch_urdf/stretch_urdf/SE3';

// constants.py
export const ROS_ARM_JOINTS = ['joint_arm_l0', 'joint_arm_l1', 'joint_arm_l2', 'joint_arm_l3'];
export const STRETCH_GRASP_FRAME = 'link_grasp_center';
export const STRETCH_CAMERA_FRAME = 'camera_color_optical_frame';
export const EE_CAMERA_FRAME = 'gripper_camera_color_optical_frame';
const D2R = Math.PI / 180;
export const look_at_ee = [-Math.PI / 2, -Math.PI / 4];
export const look_front = [0.0, -Math.PI / 4];
// [lift, arm, wrist_roll, wrist_pitch, wrist_yaw, head_pan, head_tilt]
const STRETCH_NAVIGATION_Q = { lift: 0.6, arm: 0.01, wrist_roll: 0, wrist_pitch: -1.5, wrist_yaw: 0, head_pan: 0, head_tilt: -65 * D2R };
const STRETCH_PREGRASP_Q = { lift: 0.78, arm: 0.01, wrist_roll: 0, wrist_pitch: -1.5, wrist_yaw: 0, head_pan: -Math.PI / 2, head_tilt: -Math.PI / 4 };
// kinematics.py
export const GRIPPER_OPEN = 0.6, GRIPPER_CLOSED = -0.3, GRIPPER_CLOSED_LOOSE = 0.0;

// URDF 的關節極限（stretch.urdf）
export const LIMITS = {
  lift: [0.0, 1.1], arm: [0.0, 0.52],
  wrist_yaw: [-1.75, 4.0], wrist_pitch: [-1.57, 0.56], wrist_roll: [-3.14, 3.14],
  head_pan: [-3.9, 1.5], head_tilt: [-1.53, 0.79], gripper: [GRIPPER_CLOSED, GRIPPER_OPEN],
};
// motion_profiles.yaml：lift.max、arm.fast、base.fast；腕與頭用 default
export const SPEED = {
  lift: 0.15, arm: 0.30, wrist_yaw: 2.0, wrist_pitch: 2.0, wrist_roll: 3.0,
  head_pan: 3.0, head_tilt: 3.0, gripper: 1.5,
  base_v: 0.25, base_w: 1.0, base_manip_v: 0.12,
};
const WHEEL_R = 0.0508, WHEEL_SEP = 0.3407;

const wrap = a => Math.atan2(Math.sin(a), Math.cos(a));
const clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));
const f2 = v => (Math.abs(v) < 5e-4 ? 0 : v).toFixed(3).replace(/\.?0+$/, '') || '0';
const arr = a => '[' + a.map(f2).join(', ') + ']';

export async function loadStretch(onProgress) {
  const stats = { requested: 0, loaded: 0, failed: [] };
  const manager = new THREE.LoadingManager();
  const loader = new URDFLoader(manager);
  loader.packages = { stretch_description: MESH_PKG };
  const orig = loader.loadMeshCb;
  loader.loadMeshCb = (path, mgr, done) => {
    stats.requested++;
    orig(path, mgr, (obj, err) => { if (obj && !err) stats.loaded++; done(obj, err); });
  };
  manager.onError = url => stats.failed.push(url);
  manager.onProgress = (url, n, total) => onProgress && onProgress(n, total);
  const allDone = new Promise(res => { manager.onLoad = res; });
  const urdf = await loader.loadAsync(URDF_URL);
  await allDone;
  let tris = 0;
  urdf.traverse(o => {
    if (o.isMesh) {
      o.castShadow = true;
      const g = o.geometry;
      tris += (g.index ? g.index.count : g.attributes.position.count) / 3;
    }
  });
  stats.triangles = Math.round(tris);
  return { urdf, stats };
}

export class RobotClient {
  constructor(urdf, world) {
    this.urdf = urdf;       // URDFRobot，根是 base_link
    this.world = world;     // z 向上的 ROS 世界群組
    world.add(urdf);
    for (const n of ['joint_gripper_finger_left', 'joint_gripper_finger_right']) urdf.joints[n].ignoreLimits = true;
    this.base = { x: 0, y: 0, theta: 0 };
    this.q = { lift: 0.6, arm: 0.01, wrist_roll: 0, wrist_pitch: -1.5, wrist_yaw: 0, head_pan: 0, head_tilt: -65 * D2R, gripper: GRIPPER_OPEN };
    this._mode = 'navigation';
    this._manipOrigin = { x: 0, y: 0, theta: 0 };
    this._jobs = [];
    this._wheel = [0, 0];
    this.time = 0;
    this.lastMotionTime = 0;
    this.linSpeed = 0;
    this.maxLinSpeed = 0;
    this.odometer = 0;
    this._finish = false;
    this.onLog = null;      // (pythonLine, note)
    this.onSay = null;      // (text)
    this.onGripper = null;  // (closed:boolean) 由場景決定夾到什麼
    this.apply();
  }

  // ---------- 內部 ----------
  _log(line, note) { if (this.onLog) this.onLog(line, note); }
  apply() {
    const u = this.urdf, q = this.q;
    u.position.set(this.base.x, this.base.y, 0);
    u.rotation.set(0, 0, this.base.theta);
    u.setJointValue('joint_lift', q.lift);
    for (const j of ROS_ARM_JOINTS) u.setJointValue(j, q.arm / 4);
    u.setJointValue('joint_wrist_yaw', q.wrist_yaw);
    u.setJointValue('joint_wrist_pitch', q.wrist_pitch);
    u.setJointValue('joint_wrist_roll', q.wrist_roll);
    u.setJointValue('joint_head_pan', q.head_pan);
    u.setJointValue('joint_head_tilt', q.head_tilt);
    const f = q.gripper * 0.55;
    u.setJointValue('joint_gripper_finger_left', f);
    u.setJointValue('joint_gripper_finger_right', f);
    u.setJointValue('joint_left_wheel', this._wheel[0]);
    u.setJointValue('joint_right_wheel', this._wheel[1]);
    u.updateMatrixWorld(true);
  }
  _job(step) {
    return new Promise((resolve, reject) => { this._jobs.push({ step, resolve, reject }); });
  }
  tick(dt) {
    this.time += dt;
    this.linSpeed = 0;
    const before = JSON.stringify([this.base, this.q]);
    const rest = [];
    for (const j of this._jobs) {
      let done = false;
      try { done = j.step(dt); } catch (e) { j.reject(e); continue; }
      if (done) j.resolve(true); else rest.push(j);
    }
    this._jobs = rest;
    if (JSON.stringify([this.base, this.q]) !== before) this.lastMotionTime = this.time;
    this.maxLinSpeed = Math.max(this.maxLinSpeed, this.linSpeed);
    this.apply();
  }
  _joints(targets) {
    const t = {};
    for (const k in targets) t[k] = clamp(targets[k], LIMITS[k][0], LIMITS[k][1]);
    return this._job(dt => {
      let done = true;
      for (const k in t) {
        const d = t[k] - this.q[k], s = SPEED[k] * dt;
        if (Math.abs(d) <= s) this.q[k] = t[k]; else { this.q[k] += Math.sign(d) * s; done = false; }
      }
      return done;
    });
  }
  _drive(dist, dt, vmax) {      // 沿車頭方向走 dist（可為負），回傳實際位移
    const s = clamp(dist, -vmax * dt, vmax * dt);
    this.base.x += Math.cos(this.base.theta) * s;
    this.base.y += Math.sin(this.base.theta) * s;
    this._wheel[0] += s / WHEEL_R; this._wheel[1] += s / WHEEL_R;
    this.linSpeed = Math.abs(s) / dt;
    this.odometer += Math.abs(s);
    return s;
  }
  _turn(dth, dt) {
    const s = clamp(dth, -SPEED.base_w * dt, SPEED.base_w * dt);
    this.base.theta = wrap(this.base.theta + s);
    this._wheel[0] -= s * WHEEL_SEP / 2 / WHEEL_R; this._wheel[1] += s * WHEEL_SEP / 2 / WHEEL_R;
    return s;
  }
  // 差速底盤的 goto：轉向目標、直行、轉到最終角度。近距離且目標在後方時倒車
  // （對照 config/control/noplan_velocity_*.yaml 的 max_rev_dist: 0.5）。
  _goto(gx, gy, gth) {
    let phase = 0, reverse = null;
    return this._job(dt => {
      const dx = gx - this.base.x, dy = gy - this.base.y, dist = Math.hypot(dx, dy);
      if (phase === 0) {
        if (dist < 0.004) { phase = 2; return false; }
        let head = Math.atan2(dy, dx);
        if (reverse === null) reverse = dist < 0.5 && Math.abs(wrap(head - this.base.theta)) > Math.PI / 2;
        if (reverse) head = wrap(head + Math.PI);
        const e = wrap(head - this.base.theta);
        if (Math.abs(e) < 1e-3) phase = 1; else this._turn(e, dt);
        return false;
      }
      if (phase === 1) {
        const along = dx * Math.cos(this.base.theta) + dy * Math.sin(this.base.theta);
        if (Math.abs(along) < 1e-4) { phase = 2; return false; }
        this._drive(along, dt, SPEED.base_v);
        return false;
      }
      const e = wrap(gth - this.base.theta);
      if (Math.abs(e) < 1e-3) { this.base.theta = wrap(gth); return true; }
      this._turn(e, dt);
      return false;
    });
  }

  // ---------- stretch_ai RobotClient 對應方法 ----------
  in_manipulation_mode() { return this._mode === 'manipulation'; }
  in_navigation_mode() { return this._mode === 'navigation'; }
  get_base_pose() { return [this.base.x, this.base.y, this.base.theta]; }
  get_pan_tilt() { return [this.q.head_pan, this.q.head_tilt]; }
  get_gripper_position() { return this.q.gripper; }
  _baseX() {
    const o = this._manipOrigin;
    return (this.base.x - o.x) * Math.cos(o.theta) + (this.base.y - o.y) * Math.sin(o.theta);
  }
  // [base_x, lift, arm, wrist_roll, wrist_pitch, wrist_yaw]
  get_six_joints() {
    const q = this.q;
    return [this._baseX(), q.lift, q.arm, q.wrist_roll, q.wrist_pitch, q.wrist_yaw];
  }
  async switch_to_navigation_mode() {
    this._log('robot.switch_to_navigation_mode()');
    this._mode = 'navigation';
  }
  async switch_to_manipulation_mode() {
    this._log('robot.switch_to_manipulation_mode()');
    this._mode = 'manipulation';
    this._manipOrigin = { ...this.base };
  }
  async move_to_nav_posture() {
    this._log('robot.move_to_nav_posture()');
    this._mode = 'navigation';
    // 先收臂再動升降與腕，避免帶著伸出的手臂掃過家具
    await this._joints({ arm: STRETCH_NAVIGATION_Q.arm });
    await this._joints(STRETCH_NAVIGATION_Q);
  }
  async move_to_manip_posture() {
    this._log('robot.move_to_manip_posture()');
    this._mode = 'manipulation';
    this._manipOrigin = { ...this.base };
    await this._joints({ arm: STRETCH_PREGRASP_Q.arm });
    await this._joints(STRETCH_PREGRASP_Q);
  }
  async move_base_to(xyt, { relative = false } = {}) {
    if (!this.in_navigation_mode()) throw new Error('move_base_to() is only allowed in navigation mode');
    let [x, y, t] = xyt;
    this._log(`robot.move_base_to(${arr(xyt)}${relative ? ', relative=True' : ''})`);
    if (relative) {       // xyt_base_to_global
      const b = this.base, c = Math.cos(b.theta), s = Math.sin(b.theta);
      [x, y, t] = [b.x + c * xyt[0] - s * xyt[1], b.y + s * xyt[0] + c * xyt[1], b.theta + xyt[2]];
    }
    await this._goto(x, y, t);
  }
  async arm_to(joint_angles, { gripper = null, head = null } = {}) {
    if (!this.in_manipulation_mode()) throw new Error('Robot must be in manipulation mode to move the arm');
    if (joint_angles.length !== 6) throw new Error('joint_angles must be 6 dimensional: base_x, lift, arm, wrist roll, wrist pitch, wrist yaw');
    const extra = (gripper !== null ? `, gripper=${f2(gripper)}` : '') + (head ? `, head=${arr(head)}` : '');
    this._log(`robot.arm_to(${arr(joint_angles)}${extra})`);
    const [bx, lift, arm, roll, pitch, yaw] = joint_angles;
    const h = head || look_at_ee;     // arm_to 未指定 head 時會看向末端
    const targets = { lift, arm, wrist_roll: roll, wrist_pitch: pitch, wrist_yaw: yaw, head_pan: h[0], head_tilt: clamp(h[1], -Math.PI / 2, 0) };
    if (gripper !== null) targets.gripper = gripper;
    const jobs = [this._joints(targets)];
    const o = this._manipOrigin;
    jobs.push(this._job(dt => {
      const d = bx - this._baseX();
      if (Math.abs(d) < 1e-4) return true;
      this.base.theta = o.theta;
      this._drive(d, dt, SPEED.base_manip_v);
      return false;
    }));
    await Promise.all(jobs);
    return true;
  }
  async head_to(head_pan, head_tilt) {
    this._log(`robot.head_to(${f2(head_pan)}, ${f2(head_tilt)}, blocking=True)`);
    await this._joints({ head_pan, head_tilt: clamp(head_tilt, -Math.PI / 2, 0) });
  }
  async look_front() { this._log('robot.look_front()'); await this._joints({ head_pan: look_front[0], head_tilt: look_front[1] }); }
  async look_at_ee() { this._log('robot.look_at_ee()'); await this._joints({ head_pan: look_at_ee[0], head_tilt: look_at_ee[1] }); }
  async gripper_to(target) {
    this._log(`robot.gripper_to(${f2(target)})`);
    await this._joints({ gripper: target });
  }
  async open_gripper() {
    this._log('robot.open_gripper()');
    await this._joints({ gripper: GRIPPER_OPEN });
    if (this.onGripper) this.onGripper(false);
    return true;
  }
  async close_gripper({ loose = false } = {}) {
    this._log(`robot.close_gripper(${loose ? 'loose=True' : ''})`);
    await this._joints({ gripper: loose ? GRIPPER_CLOSED_LOOSE : GRIPPER_CLOSED });
    if (this.onGripper) this.onGripper(true);
    return true;
  }
  say(text) {
    this._log(`robot.say(${JSON.stringify(text)})`);
    if (this.onSay) this.onSay(text);
  }
  async sleep(sec) {        // 對應腳本裡的 time.sleep()
    const t1 = this.time + sec;
    await this._job(() => this.time >= t1);
  }
  stop() {
    this._log('robot.stop()');
    this._finish = true;
    for (const j of this._jobs) j.reject(new Error('stopped'));
    this._jobs = [];
  }

  // ---------- 正逆運動學（用載入的 URDF 直接算）----------
  toRos(v) { return this.world.worldToLocal(v); }
  linkRos(name) { return this.toRos(this.urdf.links[name].getWorldPosition(new THREE.Vector3())); }
  graspRos() { return this.linkRos(STRETCH_GRASP_FRAME); }
  // 在底座座標系下，給定 lift/arm/腕角時夾爪中心的位置。不改變顯示中的姿態。
  fkGraspInBase(over) {
    const saved = { ...this.q };
    Object.assign(this.q, over);
    this.apply();
    const p = this.urdf.worldToLocal(this.urdf.links[STRETCH_GRASP_FRAME].getWorldPosition(new THREE.Vector3()));
    this.q = saved;
    this.apply();
    return p;
  }
  // Stretch 的三個平移自由度（base_x、lift、arm）對應底座系的 x、z、-y，
  // 所以逆解就是先定腕角，再把夾爪中心的殘差補到這三軸上
  // （stretch_ai 的 motion/simple_ik.py 也是同樣的結構）。
  ik_reach(targetRos, { pitch = 0, yaw = 0, roll = 0 } = {}) {
    const tw = this.world.localToWorld(targetRos.clone());
    const tb = this.urdf.worldToLocal(tw);              // 目標在目前底座系
    const p = this.fkGraspInBase({ lift: 0.5, arm: 0.2, wrist_pitch: pitch, wrist_yaw: yaw, wrist_roll: roll });
    const lift = 0.5 + (tb.z - p.z), arm = 0.2 + (p.y - tb.y), dx = tb.x - p.x;
    const ok = lift >= LIMITS.lift[0] - 1e-6 && lift <= LIMITS.lift[1] + 1e-6 && arm >= LIMITS.arm[0] - 1e-6 && arm <= LIMITS.arm[1] + 1e-6;
    return { ok, q: [this._baseX() + dx, lift, arm, roll, pitch, yaw], lift, arm, dx };
  }
  // 夾爪兩指尖連線在世界座標的方位角（俯抓時決定要夾物體的哪個方向）
  fingerYaw() {
    const a = this.linkRos('link_gripper_fingertip_left'), b = this.linkRos('link_gripper_fingertip_right');
    return Math.atan2(b.y - a.y, b.x - a.x);
  }
  // 整機包絡（公尺）：寬 = ROS y、深 = ROS x、高 = ROS z
  envelope(over) {
    const saved = { ...this.q }, sb = { ...this.base };
    Object.assign(this.q, over || {});
    this.base = { x: 0, y: 0, theta: 0 };
    this.apply();
    const box = new THREE.Box3().setFromObject(this.urdf);
    const lo = this.toRos(box.min.clone()), hi = this.toRos(box.max.clone());
    this.q = saved; this.base = sb; this.apply();
    return {
      W: Math.abs(hi.y - lo.y), D: Math.abs(hi.x - lo.x), H: Math.max(hi.z, lo.z),
      xmin: Math.min(lo.x, hi.x), xmax: Math.max(lo.x, hi.x), ymin: Math.min(lo.y, hi.y), ymax: Math.max(lo.y, hi.y),
    };
  }
}
