// 頁面主程式：載入 URDF、建場景、跑模擬迴圈、更新右側面板。
// 網址參數：seed=N 場景種子、speed=N、autorun=1 載入後自動開始、
// headless=1 不等畫面更新直接跑完並把結果寫進 <pre id="out">、shotAt=秒 跑到該模擬時間就停住（截圖用）。
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { loadStretch, RobotClient, LIMITS, URDF_URL, MESH_PKG } from './robot.js';
import { Hospital, ARENA, MODEL, SCENARIO_NAME } from './scene.js';
import { Mission } from './tasks.js';

const P = new URLSearchParams(location.search);
const HEADLESS = P.has('headless'), SHOT_AT = +(P.get('shotAt') || 0);
const $ = id => document.getElementById(id);
const DT = 1 / 60;
const NAV_Q = { lift: 0.6, arm: 0.01, wrist_roll: 0, wrist_pitch: -1.5, wrist_yaw: 0 };
const JOINT_ZH = { lift: '升降 lift', arm: '伸縮 arm', wrist_yaw: '腕 yaw', wrist_pitch: '腕 pitch', wrist_roll: '腕 roll', head_pan: '頭 pan', head_tilt: '頭 tilt', gripper: '夾爪' };

let urdf, stats, facts, robot, hospital, mission;
let running = false, paused = false, finished = false;
let speed = +(P.get('speed') || 4), seed = +(P.get('seed') || 1);
const logLines = [], errors = [];
window.addEventListener('error', e => { errors.push(String(e.message)); dump(); });
window.addEventListener('unhandledrejection', e => { errors.push(String(e.reason && e.reason.stack || e.reason)); dump(); });

// ---------- three.js ----------
const view = $('view');
const QUIET = HEADLESS && !SHOT_AT;      // 只要分數、不看畫面
const renderer = new THREE.WebGLRenderer({ antialias: !QUIET, preserveDrawingBuffer: HEADLESS });
renderer.domElement.className = 'gl';
view.prepend(renderer.domElement);
renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
renderer.shadowMap.enabled = !QUIET;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
const scene = new THREE.Scene();
scene.background = new THREE.Color(0xcfd8df);
const world = new THREE.Group();          // ROS 座標（z 向上）→ three.js（y 向上）
world.rotation.x = -Math.PI / 2;
scene.add(world);
scene.add(new THREE.HemisphereLight(0xffffff, 0x8899aa, 1.5));
const sun = new THREE.DirectionalLight(0xffffff, 1.6);
sun.position.set(4, 14, 3); sun.target.position.set(7.5, 0, -4); sun.castShadow = true;
sun.shadow.mapSize.set(2048, 2048);
Object.assign(sun.shadow.camera, { left: -10, right: 10, top: 7, bottom: -7, near: 1, far: 40 });
scene.add(sun, sun.target);
const cam = new THREE.PerspectiveCamera(50, 1, 0.05, 80);
cam.position.set(7.5, 10, 6);
const controls = new OrbitControls(cam, renderer.domElement);
controls.target.set(7.5, 0, -4); controls.maxPolarAngle = Math.PI / 2 - 0.02;
// 頭部相機：D435i 直放，垂直視角約 69°、水平約 42°
const headCam = new THREE.PerspectiveCamera(69, Math.tan(21 * Math.PI / 180) / Math.tan(34.5 * Math.PI / 180), 0.05, 30);
headCam.quaternion.setFromRotationMatrix(new THREE.Matrix4().makeBasis(new THREE.Vector3(0, -1, 0), new THREE.Vector3(-1, 0, 0), new THREE.Vector3(0, 0, -1)));

let viewMode = 'follow', vw = 0, vh = 0;
function setView(mode) {
  viewMode = mode;
  if (mode === 'top') { controls.target.set(7.5, 0, -4); cam.position.set(7.5, 15.5, -3.99); }
  else if (mode === 'free') { controls.target.set(7.5, 0, -4); cam.position.set(7.5, 10, 6); }
  else if (robot) { controls.target.set(robot.base.x, 0.7, -robot.base.y); cam.position.set(robot.base.x - 1.2, 4.2, -robot.base.y + 4.4); }
}
function render() {
  const w = view.clientWidth, h = view.clientHeight;
  if (w !== vw || h !== vh) { vw = w; vh = h; renderer.setSize(w, h, false); cam.aspect = w / h; cam.updateProjectionMatrix(); }
  if (viewMode === 'follow' && robot) {
    const t = new THREE.Vector3(robot.base.x, 0.7, -robot.base.y), d = t.clone().sub(controls.target);
    cam.position.add(d); controls.target.copy(t);
  }
  controls.update();
  renderer.setScissorTest(false); renderer.setViewport(0, 0, w, h);
  renderer.render(scene, cam);
  const iw = 172, ih = 308, x = w - iw - 10, y = h - 30 - ih;
  if (robot && y > 0) {
    renderer.setScissorTest(true); renderer.setScissor(x, y, iw, ih); renderer.setViewport(x, y, iw, ih);
    renderer.render(scene, headCam);
    renderer.setScissorTest(false);
  }
}

// ---------- 日誌 ----------
const clock = () => { const t = Math.floor(hospital ? hospital.ref.time : 0); return `${String(Math.floor(t / 60)).padStart(2, '0')}:${String(t % 60).padStart(2, '0')}`; };
function addLog(text, cls = '') {
  const line = `${clock()} ${text}`;
  logLines.push(line);
  if (QUIET) return;
  const el = $('log'), d = document.createElement('div');
  d.innerHTML = `<span class="t">${clock()}</span> `;
  const s = document.createElement('span'); s.className = cls; s.textContent = text; d.appendChild(s);
  el.appendChild(d);
  while (el.childElementCount > 900) el.removeChild(el.firstChild);
  const tab = $('tLog'); if (tab.classList.contains('on')) tab.scrollTop = tab.scrollHeight;
}
let sayUntil = 0;
function showSay(text) {
  $('say').textContent = '🔊 ' + text; $('say').style.display = 'block'; sayUntil = performance.now() + 4000;
  if ($('cTts').checked && window.speechSynthesis) { const u = new SpeechSynthesisUtterance(text); u.lang = 'zh-TW'; u.rate = 1.2; speechSynthesis.speak(u); }
}

// ---------- 場景與機器人 ----------
function setup() {
  if (robot) robot.stop();
  if (hospital) hospital.dispose();
  logLines.length = 0; $('log').innerHTML = '';
  hospital = new Hospital(world, seed, msg => addLog(msg, 'e'));
  robot = new RobotClient(urdf, world);
  [robot.base.x, robot.base.y, robot.base.theta] = hospital.map.start;
  robot.apply();
  robot.onLog = (line, note) => addLog(line + (note ? '   # ' + note : ''), line.startsWith('# ✗') ? 'x' : line.startsWith('#') ? 'c' : line.startsWith('robot.say') ? 's' : '');
  robot.onSay = text => { hospital.ref.hear(text); showSay(text); };
  robot.onGripper = closed => hospital.onGripper(robot, closed);
  mission = new Mission(robot, hospital);
  scene.updateMatrixWorld(true);
  running = false; paused = false; finished = false;
  $('bRun').disabled = false; $('bPause').disabled = true; $('bPause').textContent = '暫停';
  setView(viewMode);
  addLog(`# 場景種子 ${seed}，障礙抽到${hospital.truth.lottery}`, 'c');
  syncSliders(); renderScore();
}
function start() {
  if (running || finished) return;
  running = true; $('bRun').disabled = true; $('bPause').disabled = false;
  robot._log('robot = RobotClient()', '模擬版，介面同 stretch.agent.RobotClient');
  mission.run().then(() => { finish(); }, e => { if (e.message !== 'stopped') { errors.push(String(e.stack || e)); addLog('✗ ' + e.message, 'x'); finish(); } });
}
function finish() {
  running = false; finished = true; $('bPause').disabled = true;
  renderScore(); syncSliders(); dump();
}
function step() {
  robot.tick(DT);
  hospital.update(DT, robot);
  if (hospital.ref.finished && !mission.done && running) { robot.stop(); addLog('# 20 分鐘到，停止', 'x'); finish(); }
}

// ---------- URDF 測試 ----------
function computeFacts() {
  const r = new RobotClient(urdf, world), z = { lift: 0, arm: 0, wrist_roll: 0, wrist_yaw: 0 };
  const span = pitch => { const p = r.fkGraspInBase({ ...z, wrist_pitch: pitch }); return { zmin: p.z, zmax: p.z + LIMITS.lift[1], ymin: -p.y, ymax: -p.y + LIMITS.arm[1] }; };
  const joints = Object.values(urdf.joints), movable = joints.filter(j => j.jointType !== 'fixed');
  r.q = { ...r.q, head_pan: 0, head_tilt: 0 }; r.apply();
  facts = {
    links: Object.keys(urdf.links).length, joints: joints.length, movable,
    horiz: span(0), down: span(-1.57), pick: span(-1.2), tilt: span(-0.55),
    envNav: r.envelope(NAV_Q), envMax: r.envelope({ lift: 1.1, arm: 0.52, wrist_pitch: 0, wrist_yaw: 0, wrist_roll: 0 }),
    camZ: r.linkRos('camera_color_optical_frame').z,
  };
}
const cm = v => (v * 100).toFixed(1);
function renderUrdf() {
  const f = facts, e = f.envNav, m = f.envMax, L = MODEL.instrumentLen;
  const lim = (v, max) => `<td class="n">${cm(v)}</td><td class="n">${max}</td><td class="${v * 100 <= max ? 'ok' : 'bad'}">${v * 100 <= max ? '符合' : '超出'}</td>`;
  const need = (name, how, h, s) => { const ok = h <= s.zmax + 1e-6 && h >= s.zmin; return `<tr><td>${name}</td><td>${how}</td><td class="n">${cm(h)}</td><td class="n">${cm(s.zmax)}</td><td class="${ok ? 'ok' : 'bad'}">${ok ? '搆得到' : '搆不到'}</td></tr>`; };
  const tiltNeed = MODEL.maquet.h + (L / 2) * Math.sin(Math.PI / 2 - 0.55) + 0.012;
  $('tUrdf').innerHTML = `
  <h3>載入結果</h3>
  <table>
    <tr><td>URDF</td><td><code>${URDF_URL.replace('../', '')}</code></td></tr>
    <tr><td>網格</td><td><code>${MESH_PKG.replace('../', '')}/meshes</code></td></tr>
    <tr><td>連桿 / 關節</td><td>${f.links} / ${f.joints}（可動 ${f.movable.length}）</td></tr>
    <tr><td>網格檔</td><td class="${stats.failed.length ? 'bad' : 'ok'}">${stats.loaded} / ${stats.requested} 載入${stats.failed.length ? '，失敗：' + stats.failed.join('、') : ''}</td></tr>
    <tr><td>三角面</td><td>${stats.triangles.toLocaleString()}</td></tr>
    <tr><td>頭部相機高度</td><td>${cm(f.camZ)} cm</td></tr>
  </table>
  <h3>外形與 TRC 上限（cm）</h3>
  <table><thead><tr><th>項目</th><th class="n">Stretch</th><th class="n">上限</th><th></th></tr></thead>
    <tr><td>寬（導航姿態）</td>${lim(e.W, 65)}</tr>
    <tr><td>深</td>${lim(e.D, 80)}</tr>
    <tr><td>高</td>${lim(e.H, 175)}</tr>
    <tr><td>手臂全伸時的寬</td>${lim(m.W, 200)}</tr>
  </table>
  <h3>夾爪中心可達範圍（cm）</h3>
  <table><thead><tr><th>腕部姿態</th><th class="n">高度</th><th class="n">離底盤中心的側向距離</th></tr></thead>
    ${[['水平（pitch 0）', f.horiz], ['斜持（pitch −0.55）', f.tilt], ['近俯抓（pitch −1.2）', f.pick], ['俯抓（pitch −1.57）', f.down]].map(([n, s]) => `<tr><td>${n}</td><td class="n">${cm(s.zmin)} – ${cm(s.zmax)}</td><td class="n">${cm(s.ymin)} – ${cm(s.ymax)}</td></tr>`).join('')}
  </table>
  <h3>場景需求對照（cm）</h3>
  <table><thead><tr><th>動作</th><th>姿態</th><th class="n">需要</th><th class="n">最高</th><th></th></tr></thead>
    ${need('藥車上層抽屜把手', '水平', MODEL.cart.drawerZ[0], f.horiz)}
    ${need('床旁櫃頂放藥盒', '近俯抓', MODEL.cabinet.h + 0.031 + 0.09, f.pick)}
    ${need('樓層按鈕最高一顆', '水平', MODEL.gate.buttonZ[4], f.horiz)}
    ${need('器具桌 96 cm，俯抓平放', '俯抓', MODEL.maquet.h + 0.005, f.down)}
    ${need('器具桌 96 cm，斜持放下', '斜持', tiltNeed, f.tilt)}
    ${need('器具桌升到 135.5 cm', '水平', 1.355 + 0.005, f.horiz)}
  </table>
  <p class="note">Stretch 原機升降行程 1.1 m、伸縮 0.52 m。器具桌在最低的 96 cm 時俯抓差約 1 cm 搆不到，只能把腕抬到約 −32° 斜持後放下；桌子升高後完全搆不到。rover 規劃的升降 0.45–1.55 m、伸縮 0.75 m 是為了補這一段。</p>
  <h3>關節滑桿</h3>
  <p class="note">自動流程沒在跑的時候可以拖。極限取自 URDF；夾爪用 stretch_ai 的開合值（−0.3 – 0.6）。</p>
  <div id="sliders">${Object.keys(LIMITS).map(k => `<div class="sl"><span>${JOINT_ZH[k]}</span><input type="range" data-j="${k}" min="${LIMITS[k][0]}" max="${LIMITS[k][1]}" step="0.005"><output id="o_${k}"></output></div>`).join('')}</div>
  <details><summary>URDF 內的可動關節</summary><table><thead><tr><th>關節</th><th>型式</th><th class="n">下限</th><th class="n">上限</th></tr></thead>
    ${f.movable.map(j => `<tr><td><code>${j.name}</code></td><td>${j.jointType}</td><td class="n">${j.limit.lower}</td><td class="n">${j.limit.upper}</td></tr>`).join('')}</table>
    <p class="note"><code>joint_fake</code> 是 stretch_ai 為了逆解把底盤前後移動當成一個滑動關節。兩個手指關節在這份 URDF 的極限是 0，頁面裡為了顯示開合而解除。</p></details>`;
  for (const inp of $('tUrdf').querySelectorAll('input[type=range]')) inp.addEventListener('input', () => {
    if (running) return;
    robot.q[inp.dataset.j] = +inp.value; robot.apply(); $('o_' + inp.dataset.j).textContent = (+inp.value).toFixed(3);
  });
}
function syncSliders() {
  if (!robot) return;
  for (const inp of $('tUrdf').querySelectorAll('input[type=range]')) {
    const k = inp.dataset.j; inp.disabled = running; inp.value = robot.q[k]; $('o_' + k).textContent = robot.q[k].toFixed(3);
  }
}

// ---------- 計分面板 ----------
function renderScore() {
  if (!hospital) return;
  const ref = hospital.ref, sc = ref.score(), T = hospital.truth, max = { 1: 220, 2: 130, 3: 170 }, nm = { 1: '第一關　藥物配送', 2: '第二關　病房巡檢', 3: '第三關　手術器具' };
  const stageTable = st => `<h3>${nm[st]}　<span class="${sc.totals[st] ? 'ok' : 'mut'}">${sc.totals[st]}</span> / ${max[st]}</h3><table>${sc.rows[st].map(r => `<tr><td>${r.name}</td><td class="mut">${r.note}</td><td class="n ${r.got === r.max ? 'ok' : r.got ? 'warn' : 'mut'}">${r.got} / ${r.max}</td></tr>`).join('')}</table>`;
  const state = finished ? '已結束' : running ? (paused ? '暫停中' : `進行中，第 ${Math.max(1, mission.stage)} 關`) : '待命';
  $('tScore').innerHTML = `
  <div class="kpi">
    <div><b>${ref.clock()}</b><span>計時 / 20:00</span></div>
    <div><b>${sc.total}</b><span>總分 / 520</span></div>
    <div><b>${robot.odometer.toFixed(1)}</b><span>里程 m</span></div>
    <div><b>${robot.maxLinSpeed.toFixed(2)}</b><span>最高速 m/s</span></div>
  </div>
  <p class="note">狀態：${state}。抽籤：${T.lottery}。三關皆有分數（龍騰獎門檻）：<b class="${sc.allNonZero ? 'ok' : 'mut'}">${sc.allNonZero ? '是' : '尚未'}</b>。</p>
  ${stageTable(1)}${stageTable(2)}${stageTable(3)}
  ${ref.notes.length || mission.failures.length ? `<h3>事件</h3><div class="note">${[...ref.notes, ...mission.failures.map(f => '✗ ' + f)].map(n => `<div class="${n.startsWith('✗') ? 'bad' : ''}">${n}</div>`).join('')}</div>` : ''}
  <details><summary>場景答案（裁判用，機器人讀不到）</summary><div class="note">
    <div>醫囑：${T.orders.map(o => `Bed ${o.bed} ← ${o.drug}（Cart ${o.cart} ${o.item.drawer.layer ? '下層' : '上層'}）`).join('；')}</div>
    <div>巡檢樓層：${T.floor}F</div>
    <div>病房：${T.ward.map(w => `${w.name} ${SCENARIO_NAME[w.kind]}＝${w.kind === 'hr' ? w.bpm + ' bpm' : w.kind === 'iv' ? w.ml + ' ml' : w.kind === 'offbed' ? (w.away ? '離床' : '在床') : w.kind === 'fall' ? (w.fall ? '跌落' : '未跌落') : (w.gown === 'green' ? '綠色' : '非綠色')}`).join('；')}</div>
    <div>器具：${T.zone === 'left' ? '左' : '右'}區，${T.set.map(i => i.name).join(' → ')}</div></div></details>
  <p class="note">配分依 trc_plan 的整理。每盒取藥 30 分、每個巡檢情境 25 分是用關卡總分反推的；20 種器具名稱是示意。</p>`;
}

// ---------- 小地圖 ----------
function drawMini() {
  const cv = $('mini'), g = cv.getContext('2d'), s = 20, X = x => x * s, Y = y => (ARENA.H - y) * s;
  g.clearRect(0, 0, cv.width, cv.height);
  for (const o of hospital.obstacles) {
    if (!o.active) continue;
    g.fillStyle = (o.known || o.seen) ? '#55626e' : '#cfd6dc';       // 淺色＝光達還沒看到
    if (o.type === 'circle') { g.beginPath(); g.arc(X(o.cx), Y(o.cy), o.r * s, 0, 7); g.fill(); }
    else { g.save(); g.translate(X(o.cx), Y(o.cy)); g.rotate(-(o.rot || 0)); g.fillRect(-o.w * s / 2, -o.d * s / 2, o.w * s, o.d * s); g.restore(); }
  }
  if (mission.path.length > 1) { g.strokeStyle = '#1f6fb5'; g.lineWidth = 1.5; g.beginPath(); mission.path.forEach((p, i) => i ? g.lineTo(X(p[0]), Y(p[1])) : g.moveTo(X(p[0]), Y(p[1]))); g.stroke(); }
  const b = robot.base;
  g.save(); g.translate(X(b.x), Y(b.y)); g.rotate(-b.theta); g.fillStyle = '#d43b3b';
  g.beginPath(); g.moveTo(7, 0); g.lineTo(-5, 4.5); g.lineTo(-5, -4.5); g.closePath(); g.fill(); g.restore();
}

// ---------- 無頭模式輸出 ----------
function dump() {
  if (!HEADLESS && !SHOT_AT) return;
  const out = { seed, errors };
  if (hospital && mission) try {
    const sc = hospital.ref.score(), T = hospital.truth;
    Object.assign(out, {
      done: mission.done, time: +hospital.ref.time.toFixed(1), total: sc.total, totals: sc.totals,
      lost: [1, 2, 3].flatMap(st => sc.rows[st].filter(r => r.got < r.max).map(r => `${st}:${r.name} ${r.got}/${r.max} ${r.note}`)),
      failures: mission.failures, notes: hospital.ref.notes, lottery: T.lottery,
      maxSpeed: +robot.maxLinSpeed.toFixed(3), odometer: +robot.odometer.toFixed(1), idleMax: +hospital.ref.idleMax.toFixed(1),
      scene: logLines.filter(l => l.includes('[場景]') || l.includes('[裁判]') || l.includes('✗')).slice(-60),
      tail: P.has('tail') ? logLines.slice(-(+P.get('tail'))) : undefined,
    });
  } catch (e) { out.dumpError = String(e.stack || e); out.tail = logLines.slice(-25); }
  $('out').textContent = 'RESULT ' + JSON.stringify(out);
  if (finished || errors.length) post('RESULT', out);
}
// 測試用：網址帶 report=URL 時把進度與結果 POST 出去
function post(kind, obj) {
  if (!P.get('report')) return;
  fetch(P.get('report'), { method: 'POST', body: `${kind} ${P.get('tag') || ''} ${JSON.stringify(obj)}` }).catch(() => {});
}

// ---------- 主迴圈 ----------
let lastUi = 0;
async function loop() {
  for (;;) {
    if (running && !paused) {
      const n = HEADLESS ? +(P.get('chunk') || 900) : speed;
      for (let i = 0; i < n && running && !paused; i++) {
        step();
        for (let k = 0; k < 14; k++) await null;      // 讓任務腳本的 await 接續，排下一個動作
        if (SHOT_AT && hospital.ref.time >= SHOT_AT) { paused = true; dump(); }
      }
    }
    const now = performance.now();
    if (!HEADLESS) {
      render();
      if (now - lastUi > 250) { lastUi = now; renderScore(); drawMini(); if (running) syncSliders(); }
      if (now > sayUntil) $('say').style.display = 'none';
      await new Promise(r => requestAnimationFrame(r));
    } else {
      // 軟體繪圖很慢：無頭模式只在結束（或停在 shotAt）時畫一張就收工
      if (finished || paused) { syncSliders(); renderScore(); drawMini(); render(); return; }
      post('PROGRESS', { sim: +hospital.ref.time.toFixed(1), wall: Math.round(now), stage: mission.stage, last: logLines[logLines.length - 1] });
      await new Promise(r => setTimeout(r, 0));
    }
  }
}

// ---------- 啟動 ----------
for (const b of document.querySelectorAll('nav button')) b.addEventListener('click', () => {
  document.querySelectorAll('nav button, .tab').forEach(e => e.classList.remove('on'));
  b.classList.add('on'); $(b.dataset.tab).classList.add('on');
});
$('bRun').addEventListener('click', start);
$('bPause').addEventListener('click', () => { paused = !paused; $('bPause').textContent = paused ? '繼續' : '暫停'; });
$('bReset').addEventListener('click', setup);
$('iSeed').addEventListener('change', () => { seed = Math.max(1, Math.floor(+$('iSeed').value || 1)); setup(); });
$('sSpeed').addEventListener('change', () => { speed = +$('sSpeed').value; });
$('sView').addEventListener('change', () => setView($('sView').value));
$('iSeed').value = seed;
if ([1, 4, 10, 30].includes(speed)) $('sSpeed').value = speed;
if (P.get('view')) { $('sView').value = P.get('view'); viewMode = P.get('view'); }
if (P.get('tab')) document.querySelector(`nav button[data-tab="${P.get('tab')}"]`)?.click();

try {
  const res = await loadStretch((n, total) => { $('load').textContent = `載入 Stretch URDF 與網格… ${n} / ${total}`; });
  urdf = res.urdf; stats = res.stats;
  scene.updateMatrixWorld(true);
  computeFacts();
  urdf.links.camera_color_optical_frame.add(headCam);
  renderUrdf();
  setup();
  $('load').style.display = 'none';
  $('bReset').disabled = false;
  if (P.has('autorun') || HEADLESS) start();
  loop();
} catch (e) {
  errors.push(String(e.stack || e));
  $('load').textContent = '載入失敗：' + e.message + '。請確認是用 http 伺服器開啟，且 practice 底下有 stretch_ai 與 stretch_urdf。';
  dump();
}
