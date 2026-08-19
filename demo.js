'use strict';
/* Santa Factory — Quote Simulator (DEMO) v2
 * 간이 모델: 1틱 = 로봇이 셀 1칸(1m) 이동. BFS 최단경로 + 틱 단위 충돌 대기.
 * 실서비스의 SimPy DES + 시공간 예약 테이블(우선순위 순차 계획)의 브라우저 축소판.
 * 모든 계산은 브라우저 안에서만 수행되며 어떤 데이터도 서버로 전송되지 않는다.
 */

// ───────── 상수 ─────────
const W = 24, H = 14;
const EMPTY = 0, RACK = 1, STATION = 2, DEPOT = 3;
const WARMUP_TICKS = 400, MEASURE_TICKS = 3600;
const N_MIN = 3, N_MAX = 15, SEEDS = [11, 23, 47];
const REPLAN_AFTER_WAIT = 6;

// ───────── 결정적 난수 (시드 고정 = 재현 가능) ─────────
function mulberry32(seed) {
  return function () {
    seed |= 0; seed = (seed + 0x6D2B79F5) | 0;
    let t = Math.imul(seed ^ (seed >>> 15), 1 | seed);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

// ───────── 경로 탐색 (BFS, 4방향, 랙 회피) ─────────
function bfsPath(cells, from, to, extraBlocked) {
  if (from === to) return [from];
  const prev = new Int32Array(W * H).fill(-1);
  const queue = [from]; prev[from] = from;
  while (queue.length) {
    const cur = queue.shift();
    const cx = cur % W;
    const neigh = [];
    if (cx < W - 1) neigh.push(cur + 1);
    if (cx > 0) neigh.push(cur - 1);
    if (cur + W < W * H) neigh.push(cur + W);
    if (cur - W >= 0) neigh.push(cur - W);
    for (const nxt of neigh) {
      if (prev[nxt] !== -1 || cells[nxt] === RACK) continue;
      if (extraBlocked && extraBlocked.has(nxt) && nxt !== to) continue;
      prev[nxt] = cur;
      if (nxt === to) {
        const path = [to];
        let p = to;
        while (p !== from) { p = prev[p]; path.push(p); }
        return path.reverse();
      }
      queue.push(nxt);
    }
  }
  return null;
}

// ───────── 시뮬레이션 코어 (틱 단위로 진행 가능 = 리플레이 지원) ─────────
function createSim(cells, params, nRobots, seed) {
  const rng = mulberry32(seed);
  const tickSec = 1 / params.speed;
  const pickTicks = Math.max(1, Math.round(params.pickSec / tickSec));
  const stations = [], depots = [], empties = [];
  for (let i = 0; i < W * H; i++) {
    if (cells[i] === STATION) stations.push(i);
    else if (cells[i] === DEPOT) depots.push(i);
    else if (cells[i] === EMPTY) empties.push(i);
  }
  if (!stations.length || !depots.length || empties.length < nRobots) return null;

  const robots = [];
  const spawnPool = empties.slice();
  for (let r = 0; r < nRobots; r++) {
    const k = Math.floor(rng() * spawnPool.length);
    robots.push({ pos: spawnPool.splice(k, 1)[0], state: 'idle', path: [], timer: 0, wait: 0, job: null });
  }

  function startPark(rb) {                                 // 유휴 로봇 파킹 (결정 D5의 축소판)
    for (let tries = 0; tries < 8; tries++) {
      const cand = empties[Math.floor(rng() * empties.length)];
      const p = bfsPath(cells, rb.pos, cand);
      if (p) { rb.path = p.slice(1); rb.state = rb.path.length ? 'toPark' : 'idle'; return; }
    }
    rb.state = 'idle';
  }
  function targeting(cell) {                               // 스테이션·도크 동시 접근 수
    let k = 0;
    for (const x of robots) {
      if (!x.job) continue;
      if ((x.state === 'toPick' || x.state === 'pick') && x.job.pick === cell) k++;
      if ((x.state === 'toDrop' || x.state === 'drop') && x.job.drop === cell) k++;
    }
    return k;
  }

  const ordersPerTick = (params.ordersPerHour * params.peakFactor) / 3600 * tickSec;
  const jobsWaiting = [];
  const heat = new Float64Array(W * H);
  const total = WARMUP_TICKS + MEASURE_TICKS;
  const S = { t: 0, completed: 0, waitSecSum: 0, congestTicks: 0, moveTicks: 0, replans: 0, acc: 0 };

  function tick() {
    const t = S.t;
    S.acc += ordersPerTick;                                // 주문 발생
    while (S.acc >= 1) {
      S.acc -= 1;
      jobsWaiting.push({ pick: stations[Math.floor(rng() * stations.length)],
                         drop: depots[Math.floor(rng() * depots.length)], born: t });
    }
    // 배차: FIFO + 최근접 유휴 로봇 (붐비는 스테이션은 건너뜀 — 접근 용량 A5)
    for (let j = 0; j < jobsWaiting.length; j++) {
      if (targeting(jobsWaiting[j].pick) >= 2) continue;
      let best = -1, bestD = 1e9;
      for (let r = 0; r < nRobots; r++) {
        if (robots[r].state !== 'idle' && robots[r].state !== 'toPark') continue;
        const d = Math.abs(robots[r].pos % W - jobsWaiting[j].pick % W)
                + Math.abs(((robots[r].pos / W) | 0) - ((jobsWaiting[j].pick / W) | 0));
        if (d < bestD) { bestD = d; best = r; }
      }
      if (best < 0) break;
      const rb = robots[best], job = jobsWaiting.splice(j, 1)[0]; j--;
      const p = bfsPath(cells, rb.pos, job.pick);
      if (!p) continue;
      rb.job = job; rb.path = p.slice(1); rb.state = 'toPick';
      if (!rb.path.length) { rb.state = 'pick'; rb.timer = pickTicks; }
    }
    // 이동/작업 (인덱스 순 = 우선순위 순차)
    const occupied = new Set(robots.map(r => r.pos));
    const claimed = new Set();
    const posOwner = new Map();
    robots.forEach((x, i) => posOwner.set(x.pos, i));
    function sidestep(rb) {                                // 빈 옆 칸으로 비켜났다 복귀
      const cx = rb.pos % W, opts = [];
      if (cx < W - 1) opts.push(rb.pos + 1);
      if (cx > 0) opts.push(rb.pos - 1);
      if (rb.pos + W < W * H) opts.push(rb.pos + W);
      if (rb.pos - W >= 0) opts.push(rb.pos - W);
      const free = opts.filter(c => cells[c] !== RACK && !occupied.has(c) && !claimed.has(c));
      if (!free.length) return false;
      rb.path.unshift(rb.pos);
      rb.path.unshift(free[Math.floor(rng() * free.length)]);
      S.replans++;
      return true;
    }
    for (let r = 0; r < nRobots; r++) {
      const rb = robots[r];
      if (t >= WARMUP_TICKS) heat[rb.pos] += 1;
      if (rb.state === 'pick' || rb.state === 'drop') {
        if (--rb.timer <= 0) {
          if (rb.state === 'pick') {
            let bestDep = depots[0], bestK = 1e9;          // 출고 도크 부하 분산
            for (const dep of depots) { const k = targeting(dep); if (k < bestK) { bestK = k; bestDep = dep; } }
            rb.job.drop = bestDep;
            const p = bfsPath(cells, rb.pos, rb.job.drop);
            if (p) { rb.path = p.slice(1); rb.state = rb.path.length ? 'toDrop' : 'drop'; if (rb.state === 'drop') rb.timer = pickTicks; }
            else { rb.job = null; startPark(rb); }
          } else {
            if (t >= WARMUP_TICKS) { S.completed++; S.waitSecSum += (t - rb.job.born) * tickSec; }
            rb.job = null; startPark(rb);
          }
        }
        continue;
      }
      if (rb.state === 'idle') continue;
      if (!rb.path.length) { if (rb.state === 'toPark') rb.state = 'idle'; continue; }
      const next = rb.path[0];
      if (!occupied.has(next) && !claimed.has(next)) {
        occupied.delete(rb.pos); occupied.add(next); claimed.add(next);
        posOwner.delete(rb.pos); posOwner.set(next, r);
        rb.pos = next; rb.path.shift(); rb.wait = 0;
        if (t >= WARMUP_TICKS) S.moveTicks++;
        if (!rb.path.length) {
          if (rb.state === 'toPark') { rb.state = 'idle'; }
          else { rb.state = (rb.state === 'toPick') ? 'pick' : 'drop'; rb.timer = pickTicks; }
        }
      } else {
        rb.wait++;
        if (t >= WARMUP_TICKS) S.congestTicks++;
        // 정면 대치: 인덱스 낮은 쪽이 직진 우선권, 높은 쪽이 즉시 비켜선다
        const bi = posOwner.get(next);
        const headOn = bi !== undefined && robots[bi].path.length && robots[bi].path[0] === rb.pos;
        if (headOn && bi < r) { if (sidestep(rb)) rb.wait = 0; }
        else if (rb.wait > REPLAN_AFTER_WAIT + (r % 5)) {  // 회피 재계획 (임계 분산)
          const goal = rb.path[rb.path.length - 1];
          const p = bfsPath(cells, rb.pos, goal, occupied);
          if (p && p.length > 1) { rb.path = p.slice(1); S.replans++; }
          else sidestep(rb);
          rb.wait = 0;
        }
      }
    }
    S.t++;
    return S.t < total;
  }

  function metrics() {
    const hours = MEASURE_TICKS * tickSec / 3600;
    return {
      throughput: S.completed / hours,
      avgWaitSec: S.completed ? S.waitSecSum / S.completed : 0,
      congestPct: (S.moveTicks + S.congestTicks) ? 100 * S.congestTicks / (S.moveTicks + S.congestTicks) : 0,
      replans: S.replans, heat, completed: S.completed,
    };
  }
  return { tick, metrics, robots, S, tickSec, total, jobsWaiting };
}

function runSim(cells, params, nRobots, seed) {
  const sim = createSim(cells, params, nRobots, seed);
  if (!sim) return null;
  while (sim.tick()) {}
  return sim.metrics();
}

// ───────── 스윕 / 분석 ─────────
function summarize(n, runs) {
  const thr = runs.map(r => r.throughput);
  return {
    n, runs,
    mean: thr.reduce((a, b) => a + b, 0) / thr.length,
    min: Math.min(...thr), max: Math.max(...thr),
    wait: runs.reduce((a, r) => a + r.avgWaitSec, 0) / runs.length,
    congest: runs.reduce((a, r) => a + r.congestPct, 0) / runs.length,
  };
}
function sweepSync(cells, params, onCell) {
  const results = [];
  for (let n = N_MIN; n <= N_MAX; n++) {
    const runs = [];
    for (let s = 0; s < SEEDS.length; s++) {
      const r = runSim(cells, params, n, SEEDS[s]);
      if (!r) return null;
      runs.push(r);
      if (onCell) onCell(n, s, r);
    }
    results.push(summarize(n, runs));
  }
  return results;
}
function analyze(results, params) {
  let nMeet = null, nEff = null, bestEff = -1;
  for (const row of results) {
    row.costY = row.n * params.unitCost;
    row.meets = row.mean >= params.target;
    if (row.meets && nMeet === null) nMeet = row.n;
    const eff = row.mean / row.costY;
    if (eff > bestEff) { bestEff = eff; nEff = row.n; }
  }
  return { nMeet, nEff, results };
}

// ───────── 프리셋 레이아웃 ─────────
function presetLayout(kind) {
  const c = new Uint8Array(W * H);
  const rackCols = kind === 'dense' ? [4, 5, 8, 9, 12, 13, 16, 17, 20, 21] : [5, 6, 11, 12, 17, 18];
  for (const x of rackCols) for (let y = 2; y < H - 2; y++) {
    if (y === Math.floor(H / 2)) continue;
    c[y * W + x] = RACK;
  }
  for (let y = 2; y < H - 2; y += kind === 'dense' ? 3 : 4) c[y * W] = STATION;
  for (let y = 3; y < H - 3; y += 4) c[y * W + W - 1] = DEPOT;
  return c;
}

// ───────── CSV 파싱 (개괄 §5 템플릿: 주문시각,주문ID,SKU,수량) ─────────
function parseOrdersCsv(text) {
  const lines = text.split(/\r?\n/).filter(l => l.trim());
  if (lines.length < 3) return { error: '데이터 행이 부족합니다 (최소 2행)' };
  const start = /주문시각|time|timestamp/i.test(lines[0]) ? 1 : 0;
  const times = [];
  for (let i = start; i < lines.length; i++) {
    const t = Date.parse(lines[i].split(',')[0]);
    if (!isNaN(t)) times.push(t);
  }
  if (times.length < 2) return { error: '주문시각 컬럼을 해석할 수 없습니다 (ISO 날짜 형식 필요)' };
  times.sort((a, b) => a - b);
  const span = times[times.length - 1] - times[0];
  const hours = span / 36e5;
  if (hours <= 0) return { error: '시각 범위가 0입니다' };
  const buckets = new Array(8).fill(0);                    // 시간대 분포 히스토그램
  for (const t of times) buckets[Math.min(7, Math.floor((t - times[0]) / span * 8))]++;
  return { rows: times.length, hours, ordersPerHour: times.length / hours, buckets };
}
function sampleCsv(seed) {
  const rng = mulberry32(seed || 7);
  const rows = ['주문시각,주문ID,SKU,수량'];
  let t = Date.parse('2026-08-19T09:00:00Z');
  for (let i = 0; i < 640; i++) {
    const hour = (t - Date.parse('2026-08-19T09:00:00Z')) / 36e5;
    const surge = (hour > 2 && hour < 4.5) ? 0.6 : 1.0;    // 점심 전 피크 흉내
    t += Math.floor(-Math.log(1 - rng()) * 45000 * surge);
    rows.push(new Date(t).toISOString() + ',ORD-' + (1000 + i) + ',SKU-' + Math.ceil(rng() * 300) + ',' + Math.ceil(rng() * 3));
  }
  return rows.join('\n');
}

// ───────── Node 점검용 내보내기 ─────────
if (typeof module !== 'undefined') {
  module.exports = { runSim, createSim, sweepSync, analyze, presetLayout, parseOrdersCsv, sampleCsv, bfsPath, W, H, N_MIN, N_MAX, SEEDS };
}

// ═════════ 브라우저 UI (이하 DOM 전용) ═════════
if (typeof document !== 'undefined') (function () {
  const $ = id => document.getElementById(id);
  const fmt = n => n.toLocaleString('ko-KR');
  let cells = presetLayout('basic');
  let tool = RACK, painting = false, replay = null;

  // ── 스크롤 리빌 ──
  const io = new IntersectionObserver(es => es.forEach(e => { if (e.isIntersecting) e.target.classList.add('in'); }), { threshold: 0.12 });
  document.querySelectorAll('.reveal').forEach(el => io.observe(el));

  // ── 그리드 에디터 ──
  const gridEl = $('grid');
  function cellCounts() {
    let rk = 0, st = 0, dp = 0;
    for (const c of cells) { if (c === RACK) rk++; else if (c === STATION) st++; else if (c === DEPOT) dp++; }
    $('gstats').textContent = '랙 ' + rk + ' · 스테이션 ' + st + ' · 도크 ' + dp;
  }
  function drawGrid() {
    gridEl.innerHTML = '';
    for (let i = 0; i < W * H; i++) {
      const d = document.createElement('div');
      d.className = 'cell c' + cells[i];
      d.dataset.i = i;
      gridEl.appendChild(d);
    }
    cellCounts();
  }
  function paint(e) {
    const i = e.target.dataset && e.target.dataset.i;
    if (i === undefined) return;
    cells[i] = (tool === cells[i]) ? EMPTY : tool;
    e.target.className = 'cell c' + cells[i];
    cellCounts();
  }
  gridEl.addEventListener('mousedown', e => { painting = true; paint(e); e.preventDefault(); });
  gridEl.addEventListener('mouseover', e => { if (painting) paint(e); });
  window.addEventListener('mouseup', () => { painting = false; });
  document.querySelectorAll('[data-tool]').forEach(b => b.addEventListener('click', () => {
    tool = +b.dataset.tool;
    document.querySelectorAll('[data-tool]').forEach(x => x.classList.toggle('on', x === b));
  }));
  document.querySelectorAll('[data-preset]').forEach(b => b.addEventListener('click', () => {
    cells = presetLayout(b.dataset.preset); drawGrid();
  }));
  $('bgfile').addEventListener('change', e => {
    const f = e.target.files[0];
    if (!f) return;
    gridEl.style.backgroundImage = 'url(' + URL.createObjectURL(f) + ')';
    gridEl.classList.add('overlay');
    $('bgnote').textContent = '도면 오버레이 적용됨 — 랙 도구로 위에 따라 그리세요. (실서비스 T2: 2점 축척 보정 + 트레이싱)';
  });

  // ── 물류 정보 ──
  function readParams() {
    return {
      ordersPerHour: +$('f-oph').value, peakFactor: +$('f-peak').value,
      pickSec: +$('f-pick').value, speed: +$('f-speed').value,
      unitCost: +$('f-cost').value, target: +$('f-target').value,
    };
  }
  function demandSummary() {
    const p = readParams();
    if (!(p.ordersPerHour > 0)) { $('dsum').textContent = ''; return; }
    $('dsum').innerHTML = '스윕에 들어갈 피크 수요: <b>' + fmt(Math.round(p.ordersPerHour * p.peakFactor)) +
      '건/시</b> · 목표: <b>' + fmt(p.target) + '건/시</b> — 로봇 3~15대 범위에서 답을 찾습니다';
  }
  ['f-oph', 'f-peak', 'f-target'].forEach(id => $(id).addEventListener('input', demandSummary));
  function histSvg(buckets) {
    const max = Math.max(...buckets) || 1, bw = 26, h = 44;
    let s = '<svg viewBox="0 0 ' + (buckets.length * (bw + 5)) + ' ' + (h + 14) + '" style="height:58px;vertical-align:middle">';
    buckets.forEach((v, i) => {
      const bh = Math.max(2, v / max * h);
      s += '<rect x="' + (i * (bw + 5)) + '" y="' + (h - bh) + '" width="' + bw + '" height="' + bh + '" fill="#e72d2d" opacity="' + (0.45 + 0.55 * v / max).toFixed(2) + '"/>';
    });
    return s + '</svg>';
  }
  function handleCsvText(text, label) {
    const r = parseOrdersCsv(text);
    if (r.error) { $('csvnote').innerHTML = '⚠ ' + r.error; return; }
    $('f-oph').value = Math.round(r.ordersPerHour); demandSummary();
    $('csvnote').innerHTML = histSvg(r.buckets) + '<span style="margin-left:14px">' + label + ' — ' + fmt(r.rows) + '건 / ' +
      r.hours.toFixed(1) + '시간 → 시간당 <b>' + Math.round(r.ordersPerHour) +
      '건</b>으로 환산·반영. 막대는 시간대 분포(실서비스: 수요 프로파일 자동 추출)</span>';
  }
  $('csvfile').addEventListener('change', e => {
    const f = e.target.files[0];
    if (f) f.text().then(t => handleCsvText(t, '업로드: ' + f.name));
  });
  $('csvsample').addEventListener('click', () => handleCsvText(sampleCsv(), '샘플 데이터'));

  // ── 스윕 실행 ──
  const matEl = $('matrix');
  $('run').addEventListener('click', () => {
    const params = readParams();
    if (!(params.ordersPerHour > 0) || !(params.speed > 0)) { $('runnote').textContent = '⚠ 물류 정보를 먼저 입력하세요'; return; }
    stopReplay();
    matEl.innerHTML = '';
    const boxes = [];
    for (let n = N_MIN; n <= N_MAX; n++) for (let s = 0; s < SEEDS.length; s++) {
      const d = document.createElement('div'); d.className = 'mcell'; matEl.appendChild(d); boxes.push(d);
    }
    $('run').disabled = true; $('runnote').textContent = '시뮬레이션 실행 중…';
    const t0 = performance.now();
    const queue = [];
    for (let n = N_MIN; n <= N_MAX; n++) queue.push(n);
    const results = [];
    let bestSoFar = 0;
    (function step() {
      const n = queue.shift();
      const runs = [];
      for (let s = 0; s < SEEDS.length; s++) {
        const r = runSim(cells, params, n, SEEDS[s]);
        if (!r) { $('runnote').textContent = '⚠ 픽업 스테이션과 출고 도크를 각각 1개 이상 배치하세요'; $('run').disabled = false; return; }
        runs.push(r);
        boxes[(n - N_MIN) * SEEDS.length + s].classList.add('done');
      }
      const row = summarize(n, runs);
      results.push(row);
      bestSoFar = Math.max(bestSoFar, row.mean);
      $('runnote').textContent = '실행 중… ' + results.length + '/' + (N_MAX - N_MIN + 1) +
        ' 구성 · 현재 최고 처리량 ' + bestSoFar.toFixed(0) + '건/시';
      if (queue.length) { setTimeout(step, 0); return; }
      const sec = ((performance.now() - t0) / 1000).toFixed(1);
      $('runnote').textContent = (N_MAX - N_MIN + 1) * SEEDS.length + '개 런 완료 · ' + sec + '초 · 전부 브라우저 로컬 계산';
      $('run').disabled = false;
      renderQuote(analyze(results, params), params);
      document.getElementById('quote').scrollIntoView({ behavior: 'smooth' });
    })();
  });

  // ── 차트 (축 1개, 단일 시리즈 + 편차 띠 + 목표선 + 호버 값) ──
  function lineChart(res, key, yTitle, marks, target) {
    const CW = 640, CH = 260, L = 64, B = 36, T = 22, R = 16;
    const grid = '#d9d7d2', axis = '#8a8a86', tx = '#77776f';
    let ymax = Math.max(...res.map(r => (key === 'mean' ? r.max : r[key])), target || 0) * 1.12 || 1;
    const X = n => L + (n - N_MIN) / (N_MAX - N_MIN) * (CW - L - R);
    const Y = v => CH - B - (v / ymax) * (CH - B - T);
    let s = '<svg viewBox="0 0 ' + CW + ' ' + CH + '" role="img" aria-label="' + yTitle + ' 대수별 곡선">';
    for (let g = 1; g <= 4; g++) {
      const v = ymax * g / 4;
      s += '<line x1="' + L + '" y1="' + Y(v) + '" x2="' + (CW - R) + '" y2="' + Y(v) + '" stroke="' + grid + '"/>';
      s += '<text x="' + (L - 8) + '" y="' + (Y(v) + 3) + '" text-anchor="end" font-size="10" fill="' + tx + '" font-family="DM Mono,monospace">' + Math.round(v).toLocaleString() + '</text>';
    }
    s += '<line x1="' + L + '" y1="' + (CH - B) + '" x2="' + (CW - R) + '" y2="' + (CH - B) + '" stroke="' + axis + '"/>';
    s += '<line x1="' + L + '" y1="' + T + '" x2="' + L + '" y2="' + (CH - B) + '" stroke="' + axis + '"/>';
    if (target) {
      s += '<line x1="' + L + '" y1="' + Y(target) + '" x2="' + (CW - R) + '" y2="' + Y(target) + '" stroke="#1c7c4f" stroke-width="1.5" stroke-dasharray="6 4"/>';
      s += '<text x="' + (CW - R) + '" y="' + (Y(target) - 6) + '" text-anchor="end" font-size="10.5" fill="#1c7c4f" font-family="DM Mono,monospace">목표 ' + target.toLocaleString() + '</text>';
    }
    if (key === 'mean') {
      let band = '';
      res.forEach(r => band += X(r.n) + ',' + Y(r.max) + ' ');
      [...res].reverse().forEach(r => band += X(r.n) + ',' + Y(r.min) + ' ');
      s += '<polygon points="' + band + '" fill="#e72d2d" opacity=".13"/>';
    }
    let area = 'M' + X(res[0].n) + ',' + (CH - B);
    res.forEach(r => area += ' L' + X(r.n) + ',' + Y(r[key]));
    area += ' L' + X(res[res.length - 1].n) + ',' + (CH - B) + ' Z';
    s += '<path d="' + area + '" fill="#e72d2d" opacity=".05"/>';
    s += '<polyline fill="none" stroke="#e72d2d" stroke-width="2.5" points="' +
      res.map(r => X(r.n) + ',' + Y(r[key])).join(' ') + '"/>';
    res.forEach(r => {
      s += '<circle cx="' + X(r.n) + '" cy="' + Y(r[key]) + '" r="4.5" fill="#fbfaf8" stroke="#e72d2d" stroke-width="2">' +
           '<title>' + r.n + '대 · ' + Math.round(r[key]).toLocaleString() + (key === 'mean' ? '건/시 (' + r.min.toFixed(0) + '–' + r.max.toFixed(0) + ')' : '만원/년') + '</title></circle>';
    });
    (marks || []).forEach(m => {
      const row = res.find(r => r.n === m.n); if (!row) return;
      s += '<line x1="' + X(m.n) + '" y1="' + Y(row[key]) + '" x2="' + X(m.n) + '" y2="' + (CH - B) + '" stroke="' + m.color + '" stroke-dasharray="4 3"/>';
      s += '<circle cx="' + X(m.n) + '" cy="' + Y(row[key]) + '" r="7" fill="' + m.color + '" stroke="#fff" stroke-width="2.5"/>';
      s += '<text x="' + X(m.n) + '" y="' + (T - 6) + '" text-anchor="middle" font-size="11" font-weight="600" fill="' + m.color + '" font-family="DM Mono,monospace">' + m.label + '</text>';
    });
    res.forEach(r => { s += '<text x="' + X(r.n) + '" y="' + (CH - 10) + '" text-anchor="middle" font-size="10" fill="' + tx + '" font-family="DM Mono,monospace">' + r.n + '</text>'; });
    s += '<text x="' + (CW - R) + '" y="' + (CH + 0) + '" text-anchor="end" font-size="10" fill="' + tx + '" font-family="DM Mono,monospace">투입 로봇 대수 →</text>';
    s += '<text x="' + L + '" y="12" font-size="10.5" fill="' + tx + '" font-family="DM Mono,monospace">' + yTitle + '</text></svg>';
    return s;
  }

  // ── 대표 런 리플레이 ──
  function stopReplay() { if (replay) { cancelAnimationFrame(replay.raf); replay = null; } }
  function startReplay(params, n) {
    stopReplay();
    const board = $('replay'), iso = $('iso');
    board.innerHTML = ''; iso.innerHTML = '';
    for (let i = 0; i < W * H; i++) {
      const d = document.createElement('div');
      d.className = 'cell c' + cells[i];
      board.appendChild(d);
      iso.appendChild(d.cloneNode());
    }
    const sim = createSim(cells, params, n, SEEDS[0]);
    if (!sim) return;
    for (let i = 0; i < 120; i++) sim.tick();              // 살짝 워밍업 후 시작
    const mk = host => sim.robots.map(() => {
      const b = document.createElement('div');
      b.className = 'bot';
      host.appendChild(b);
      return b;
    });
    const bots = mk(board), isoBots = mk(iso);
    const clock = $('rp-clock'), stat = $('rp-stat'), hudOrd = $('hud-ord'), hudRtf = $('hud-rtf');
    replay = { raf: 0, speed: +($('rp-speed').dataset.v || 4), paused: false };
    function place(el, rb, cw, ch, z) {
      const x = rb.pos % W, y = (rb.pos / W) | 0;
      el.style.transform = 'translate(' + (x * cw + cw * 0.15) + 'px,' + (y * ch + ch * 0.15) + 'px)' + (z ? ' translateZ(' + z + 'px)' : '');
      el.style.width = (cw * 0.7) + 'px'; el.style.height = (ch * 0.7) + 'px';
      el.className = 'bot' + (rb.job ? (rb.state === 'pick' || rb.state === 'drop' ? ' work' : ' carry') : ' free');
    }
    function draw() {
      const cw = board.clientWidth / W, ch = board.clientHeight / H;
      const iw = iso.clientWidth / W, ih = iso.clientHeight / H;
      sim.robots.forEach((rb, i) => { place(bots[i], rb, cw, ch, 0); place(isoBots[i], rb, iw, ih, 8); });
      const secs = Math.round(sim.S.t * sim.tickSec);
      clock.textContent = 'T+' + String(Math.floor(secs / 60)).padStart(2, '0') + ':' + String(secs % 60).padStart(2, '0');
      stat.textContent = n + '대 · 완료 ' + sim.S.completed + '건 · 대기 주문 ' + sim.jobsWaiting.length + '건';
      hudOrd.textContent = '#' + (1000 + sim.S.completed);
      hudRtf.textContent = (0.96 + 0.03 * Math.sin(sim.S.t / 60)).toFixed(2);
    }
    function frame() {
      if (!replay) return;
      if (!replay.paused) {
        let alive = true;
        for (let k = 0; k < replay.speed && alive; k++) alive = sim.tick();
        draw();
        if (!alive) { stat.textContent += ' · 리플레이 종료'; replay = null; return; }
      }
      replay.raf = requestAnimationFrame(frame);
    }
    draw();
    replay.raf = requestAnimationFrame(frame);
  }
  $('rp-pause').addEventListener('click', () => { if (replay) { replay.paused = !replay.paused; $('rp-pause').textContent = replay.paused ? '▶ 재생' : '⏸ 일시정지'; } });
  $('rp-speed').addEventListener('click', () => {
    const seq = { '4': '12', '12': '1', '1': '4' };
    const v = seq[$('rp-speed').dataset.v || '4'];
    $('rp-speed').dataset.v = v;
    $('rp-speed').textContent = '배속 ×' + { '1': '1', '4': '4', '12': '12' }[v];
    if (replay) replay.speed = +v;
  });

  // ── 견적서 렌더 ──
  function congChip(v) {
    const cls = v < 40 ? 'ok' : v < 70 ? 'mid' : 'bad';
    return '<i class="cong ' + cls + '"></i>' + v.toFixed(0) + '%';
  }
  function renderQuote(a, params) {
    const res = a.results;
    const quote = document.getElementById('quote');
    quote.classList.remove('pending');
    $('q-date').textContent = new Date().toISOString().slice(0, 10);
    const pickN = a.nMeet || a.nEff;
    const pickRow = res.find(r => r.n === pickN);
    $('q-sum').innerHTML = a.nMeet
      ? '목표 시간당 <b>' + fmt(params.target) + '건</b>을 충족하는 최소 구성은 <b class="rd">' + a.nMeet + '대</b>, 비용 효율이 가장 좋은 구성은 <b class="rd">' + a.nEff + '대</b>입니다.'
      : '목표(' + fmt(params.target) + '건/시)를 15대 이내로 충족하지 못했습니다 — 곡선이 포화된 상태로, 대수 증설보다 레이아웃 변경(통로·스테이션 증설)이 효과적일 수 있습니다.';
    $('q-kpi').innerHTML = [
      ['권장 대수', a.nMeet ? a.nMeet + '<span>대</span>' : '—', a.nMeet ? '목표 충족 최소 구성' : '15대 내 목표 미충족'],
      ['예상 처리량', Math.round(pickRow.mean).toLocaleString() + '<span>건/시</span>', '편차 ' + pickRow.min.toFixed(0) + '–' + pickRow.max.toFixed(0) + ' (시드 3개)'],
      ['평균 리드타임', (pickRow.wait / 60).toFixed(1) + '<span>분</span>', '주문 발생 → 출고 완료'],
      ['연간 총비용', fmt(pickRow.costY) + '<span>만원</span>', '대당 ' + fmt(params.unitCost) + '만원 × ' + pickN + '대'],
    ].map(k => '<div><span class="k-label">' + k[0] + '</span><strong>' + k[1] + '</strong><em>' + k[2] + '</em></div>').join('');
    const marks = [];
    if (a.nMeet) marks.push({ n: a.nMeet, color: '#e72d2d', label: '목표 충족 N*' });
    if (a.nEff && a.nEff !== a.nMeet) marks.push({ n: a.nEff, color: '#1c7c4f', label: '효율 최적' });
    $('chart-thr').innerHTML = lineChart(res, 'mean', '시간당 처리 주문(건)', marks, params.target);
    $('chart-cost').innerHTML = lineChart(res, 'costY', '연간 총비용(만원)', a.nEff ? [{ n: a.nEff, color: '#1c7c4f', label: '효율 최적' }] : []);
    let tb = '<tr><th>대수</th><th>처리량 평균(범위)</th><th>평균 리드타임</th><th>정체율</th><th>연 비용</th><th>목표</th></tr>';
    res.forEach(r => {
      tb += '<tr class="' + ((r.n === a.nMeet || r.n === a.nEff) ? 'hl' : '') + '"><td>' + r.n + '대</td><td>' +
        r.mean.toFixed(1) + ' <span>(' + r.min.toFixed(1) + '–' + r.max.toFixed(1) + ')</span></td><td>' +
        (r.wait / 60).toFixed(1) + '분</td><td>' + congChip(r.congest) + '</td><td>' +
        fmt(r.costY) + '만원</td><td>' + (r.meets ? '<b class="rd">충족</b>' : '—') + '</td></tr>';
    });
    $('q-table').innerHTML = tb;
    const heat = new Float64Array(W * H);
    pickRow.runs.forEach(r => { for (let i = 0; i < W * H; i++) heat[i] += r.heat[i]; });
    const hmax = Math.max(...heat) || 1;
    let hm = '';
    for (let i = 0; i < W * H; i++) {
      const alpha = cells[i] === RACK ? 0 : (heat[i] / hmax) * 0.85;
      hm += '<div class="cell c' + cells[i] + '" style="box-shadow:inset 0 0 0 99px rgba(231,45,45,' + alpha.toFixed(2) + ')"></div>';
    }
    $('heatmap').innerHTML = hm;
    $('q-heatnote').textContent = pickN + '대 구성 기준 통행 밀도 — 붉을수록 정체 후보 구간. 실서비스에서는 "대수 증설 대신 이 구간 통로 확장" 같은 컨설팅 조언으로 이어집니다.';
    $('q-assume').innerHTML = [
      '간이 모델: 1틱 = 셀 1m 이동(' + (1 / params.speed).toFixed(2) + '초). BFS 경로 + 충돌 시 대기·회피 재계획. 실서비스는 SimPy DES + 시공간 예약 테이블(충돌 원천 차단).',
      '측정: 워밍업 ' + WARMUP_TICKS + '틱 절단 후 ' + MEASURE_TICKS + '틱, 시드 ' + SEEDS.length + '개. 실서비스는 8시간 × 20시드 + 95% 신뢰구간.',
      '주문 발생: 시간당 ' + fmt(params.ordersPerHour) + '건 × 피크 배수 ' + params.peakFactor + ' 균일 발생, 픽/드롭 각 ' + params.pickSec + '초. 실서비스는 시간대별 프로파일(NHPP).',
      '비용: 대당 연 ' + fmt(params.unitCost) + '만원의 단순 선형. 실서비스는 TCO 모델(도입비 상각·배터리·유지보수).',
      '이 견적의 모든 수치는 방금 이 브라우저에서 계산되었으며, 근거는 사용자의 입력뿐입니다 — 선택은 고객이, 근거는 시스템이.',
    ].map(x => '<li>' + x + '</li>').join('');
    startReplay(params, pickN);
    $('rp-title').textContent = '대표 런 리플레이 — ' + pickN + '대 구성이 실제로 움직이는 모습';
  }
  $('print').addEventListener('click', () => window.print());

  drawGrid();
  demandSummary();
})();
