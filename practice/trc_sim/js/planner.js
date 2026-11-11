// 2D 佔用格 A*。結構對照 stretch_ai 的 src/stretch/motion/algo/a_star.py：
// to_pt / to_xy、8 鄰域、靠近障礙加罰分、is_in_line_of_sight、clean_path、起終點被佔用時找最近空格。
export class AStar {
  constructor(width, height, resolution = 0.05) {
    this.res = resolution;
    this.W = Math.ceil(width / resolution);
    this.H = Math.ceil(height / resolution);
    this.dist = new Float32Array(this.W * this.H);   // 到最近障礙的距離（已扣掉機器人半徑）
    this.soft = 0.25;                                // 這個距離內加罰分，路徑會離障礙遠一點
  }
  to_pt(x, y) { return [Math.min(this.W - 1, Math.max(0, Math.floor(x / this.res))), Math.min(this.H - 1, Math.max(0, Math.floor(y / this.res)))]; }
  to_xy(i, j) { return [(i + 0.5) * this.res, (j + 0.5) * this.res]; }
  // obstacles: {type:'rect',cx,cy,w,d,rot} 或 {type:'circle',cx,cy,r}
  build(obstacles, inflate) {
    const { W, H, dist } = this;
    dist.fill(1e3);
    for (const o of obstacles) {
      const reach = inflate + this.soft + (o.type === 'circle' ? o.r : Math.hypot(o.w, o.d) / 2);
      const [i0, j0] = this.to_pt(o.cx - reach, o.cy - reach), [i1, j1] = this.to_pt(o.cx + reach, o.cy + reach);
      const c = Math.cos(o.rot || 0), s = Math.sin(o.rot || 0);
      for (let j = j0; j <= j1; j++) for (let i = i0; i <= i1; i++) {
        const [x, y] = this.to_xy(i, j), dx = x - o.cx, dy = y - o.cy;
        let d;
        if (o.type === 'circle') d = Math.hypot(dx, dy) - o.r;
        else {
          const lx = Math.abs(c * dx + s * dy) - o.w / 2, ly = Math.abs(-s * dx + c * dy) - o.d / 2;
          d = Math.hypot(Math.max(lx, 0), Math.max(ly, 0)) + Math.min(Math.max(lx, ly), 0);
        }
        d -= inflate;
        const k = j * W + i;
        if (d < dist[k]) dist[k] = d;
      }
    }
  }
  point_is_occupied(i, j) { return i < 0 || j < 0 || i >= this.W || j >= this.H || this.dist[j * this.W + i] <= 0; }
  is_in_line_of_sight(ax, ay, bx, by, margin = 0.02) {
    const n = Math.max(1, Math.ceil(Math.hypot(bx - ax, by - ay) / (this.res * 0.5)));
    for (let k = 0; k <= n; k++) {
      const [i, j] = this.to_pt(ax + (bx - ax) * k / n, ay + (by - ay) * k / n);
      if (i < 0 || j < 0 || i >= this.W || j >= this.H || this.dist[j * this.W + i] <= margin) return false;
    }
    return true;
  }
  get_unoccupied_neighbor(i, j) {
    if (!this.point_is_occupied(i, j)) return [i, j];
    for (let r = 1; r < 30; r++) {
      let best = null, bd = 1e9;
      for (let dj = -r; dj <= r; dj++) for (let di = -r; di <= r; di++) {
        if (Math.max(Math.abs(di), Math.abs(dj)) !== r || this.point_is_occupied(i + di, j + dj)) continue;
        const d = di * di + dj * dj;
        if (d < bd) { bd = d; best = [i + di, j + dj]; }
      }
      if (best) return best;
    }
    return null;
  }
  run_astar(s, g) {
    const { W, H, dist } = this, N = W * H;
    const cost = new Float64Array(N).fill(Infinity), from = new Int32Array(N).fill(-1), closed = new Uint8Array(N);
    const sk = s[1] * W + s[0], gk = g[1] * W + g[0];
    const heap = [];    // 二元堆 [f, k]
    const push = (f, k) => { heap.push([f, k]); let i = heap.length - 1; while (i > 0) { const p = (i - 1) >> 1; if (heap[p][0] <= heap[i][0]) break; [heap[p], heap[i]] = [heap[i], heap[p]]; i = p; } };
    const pop = () => { const top = heap[0], last = heap.pop(); if (heap.length) { heap[0] = last; let i = 0; for (;;) { const l = 2 * i + 1, r = l + 1; let m = i; if (l < heap.length && heap[l][0] < heap[m][0]) m = l; if (r < heap.length && heap[r][0] < heap[m][0]) m = r; if (m === i) break; [heap[m], heap[i]] = [heap[i], heap[m]]; i = m; } } return top; };
    cost[sk] = 0; push(0, sk);
    while (heap.length) {
      const [, k] = pop();
      if (k === gk) break;
      if (closed[k]) continue;        // 堆裡的舊項目
      closed[k] = 1;
      const i = k % W, j = (k - i) / W;
      for (let dj = -1; dj <= 1; dj++) for (let di = -1; di <= 1; di++) {
        if (!di && !dj) continue;
        const ni = i + di, nj = j + dj;
        if (ni < 0 || nj < 0 || ni >= W || nj >= H) continue;
        const nk = nj * W + ni, d = dist[nk];
        if (d <= 0 || closed[nk]) continue;
        if (di && dj && (dist[j * W + ni] <= 0 || dist[nj * W + i] <= 0)) continue;   // 不切角
        const punish = d < this.soft ? 1 + 4 * (this.soft - d) / this.soft : 1;
        const c = cost[k] + Math.hypot(di, dj) * punish;
        if (c < cost[nk]) { cost[nk] = c; from[nk] = k; push(c + Math.hypot(g[0] - ni, g[1] - nj), nk); }
      }
    }
    if (from[gk] < 0 && gk !== sk) return null;
    const path = [];
    for (let k = gk; k >= 0; k = from[k]) { path.push([k % W, Math.floor(k / W)]); if (k === sk) break; }
    return path.reverse();
  }
  clean_path(pts) {       // 視線可達就直接連過去
    const out = [pts[0]];
    const dAt = p => { const [a, b] = this.to_pt(p[0], p[1]); return this.dist[b * this.W + a]; };
    let i = 0;
    while (i < pts.length - 1) {
      let j = pts.length - 1;
      // 端點本身就貼近家具（停靠點）時，餘裕跟著放寬，否則永遠連不起來
      const ok = k => this.is_in_line_of_sight(pts[i][0], pts[i][1], pts[k][0], pts[k][1], Math.min(0.06, Math.max(0, Math.min(dAt(pts[i]), dAt(pts[k])) - 0.01)));
      while (j > i + 1 && !ok(j)) j--;
      out.push(pts[j]); i = j;
    }
    return out;
  }
  // 回傳世界座標的路徑點（含起點與精確終點），找不到回傳 null
  plan(start, goal) {
    const s = this.get_unoccupied_neighbor(...this.to_pt(start[0], start[1]));
    const g = this.get_unoccupied_neighbor(...this.to_pt(goal[0], goal[1]));
    if (!s || !g) return null;
    const cells = this.run_astar(s, g);
    if (!cells) return null;
    const pts = cells.map(c => this.to_xy(c[0], c[1]));
    pts[0] = [start[0], start[1]];
    pts.push([goal[0], goal[1]]);
    return this.clean_path(pts);
  }
}
