// ─── AI Decision Lab — Complete Script ──────────────────────
// Grid Pathfinding + 1D Array Search + Optimization Visualizations

// ═══ DOM Elements ═══
const gridCanvas = document.getElementById("gridCanvas");
const ctx = gridCanvas.getContext("2d");
const gradientCanvas = document.getElementById("gradientCanvas");
const gctx = gradientCanvas.getContext("2d");

// ═══ Grid Config ═══
const rows = 20, cols = 26, cellSize = 30;
const EMPTY = 0, WALL = 1, START = 2, GOAL = 3, OPEN = 4, CLOSED = 5, PATH = 6, ROBOT = 7;
const ALLOW_DIAGONALS = true;

const colors = {
  [EMPTY]: "#f8fafc", [WALL]: "#475569", [START]: "#22c55e", [GOAL]: "#ef4444",
  [OPEN]: "#60a5fa", [CLOSED]: "#fbbf24", [PATH]: "#a855f7", [ROBOT]: "#e2e8f0"
};

// ═══ State ═══
let grid = [], weights = [];
let currentTool = "wall";
let startNode = { row: 2, col: 2 }, goalNode = { row: 16, col: 21 };
let isRunning = false, isMouseDown = false, liveMode = false;
let lastPath = [], lastRawPath = [];
let nodesExploredCount = 0, currentAlgorithmName = "None";
let gradientSurface = [], gradientTrace = [], gradientPoint = { x: 60, y: 60 };
let speedMultiplier = 5;

// ═══ Helpers ═══
function $(id) { return document.getElementById(id); }
function setStatus(msg) {
  const el = $("status");
  el.textContent = msg;
  el.className = "stat-value " + (
    msg === "Ready" ? "status-ready" :
    msg.includes("Running") || msg.includes("Animating") || msg.includes("Optimiz") ? "status-running" :
    msg.includes("no path") || msg.includes("first") ? "status-error" :
    "status-done"
  );
}
function delay(base) { return new Promise(r => setTimeout(r, Math.max(1, base / speedMultiplier))); }
function inBounds(r, c) { return r >= 0 && r < rows && c >= 0 && c < cols; }
function key(n) { return n.row + "," + n.col; }
function heuristic(a, b) {
  const dx = Math.abs(a.col - b.col), dy = Math.abs(a.row - b.row);
  return ALLOW_DIAGONALS ? Math.hypot(dx, dy) : dx + dy;
}

function setTool(name, label) {
  currentTool = name;
  $("current-tool").textContent = label;
  const map = { start: "tool-start", goal: "tool-goal", wall: "tool-wall", erase: "tool-erase", weight: "tool-weight" };
  Object.entries(map).forEach(([k, id]) => {
    const b = $(id);
    if (b) { b.classList.toggle("active-tool", k === name); b.setAttribute("aria-checked", String(k === name)); }
  });
}

function updateStats({ explored = 0, pathLength = 0, pathCost = 0, turns = 0, algorithm = "None" } = {}) {
  $("nodes-explored").textContent = explored;
  $("path-length").textContent = pathLength;
  $("path-cost").textContent = typeof pathCost === "number" ? pathCost.toFixed(2) : pathCost;
  $("turn-count").textContent = turns;
  $("algorithm-name").textContent = algorithm;
  const badge = $("grid-algo-badge");
  if (badge) badge.textContent = algorithm === "None" ? "—" : algorithm;
}

function setBusy(busy) {
  isRunning = busy;
  document.querySelectorAll(".algo-group button").forEach(b => {
    if (busy) b.classList.add("disabled"); else b.classList.remove("disabled");
  });
}

// ═══ Grid Management ═══
function createGrid() {
  grid = Array.from({ length: rows }, () => Array(cols).fill(EMPTY));
  weights = Array.from({ length: rows }, () => Array(cols).fill(1));
  grid[startNode.row][startNode.col] = START;
  grid[goalNode.row][goalNode.col] = GOAL;
  lastPath = []; lastRawPath = [];
}

function resetSearchStates() {
  for (let r = 0; r < rows; r++)
    for (let c = 0; c < cols; c++)
      if ([OPEN, CLOSED, PATH, ROBOT].includes(grid[r][c])) grid[r][c] = EMPTY;
  grid[startNode.row][startNode.col] = START;
  grid[goalNode.row][goalNode.col] = GOAL;
}

function clearPathOnly() {
  if (isRunning) return;
  resetSearchStates(); lastPath = []; lastRawPath = [];
  updateStats(); setStatus("Path cleared."); drawGrid();
}

// ═══ Weight + Color ═══
function getWeightColor(w) {
  if (w <= 1) return null;
  if (w === 2) return "rgba(34,197,94,.18)";
  if (w === 3) return "rgba(250,204,21,.26)";
  if (w === 4) return "rgba(249,115,22,.28)";
  return "rgba(239,68,68,.34)";
}

// ═══ Draw Grid ═══
function drawGrid(robotNode) {
  ctx.clearRect(0, 0, gridCanvas.width, gridCanvas.height);
  for (let r = 0; r < rows; r++) {
    for (let c = 0; c < cols; c++) {
      const st = robotNode && robotNode.row === r && robotNode.col === c ? ROBOT : grid[r][c];
      ctx.fillStyle = colors[st];
      ctx.fillRect(c * cellSize, r * cellSize, cellSize, cellSize);
      const ov = getWeightColor(weights[r][c]);
      if (ov && ![WALL, START, GOAL, ROBOT].includes(st)) {
        ctx.fillStyle = ov;
        ctx.fillRect(c * cellSize, r * cellSize, cellSize, cellSize);
      }
      if (weights[r][c] > 1 && st !== WALL) {
        ctx.fillStyle = "#1e293b";
        ctx.font = "bold 11px system-ui";
        ctx.textAlign = "center"; ctx.textBaseline = "middle";
        ctx.fillText(String(weights[r][c]), c * cellSize + cellSize / 2, r * cellSize + cellSize / 2);
      }
      // Start/Goal labels
      if (st === START || st === GOAL) {
        ctx.fillStyle = "#fff";
        ctx.font = "bold 14px system-ui";
        ctx.textAlign = "center"; ctx.textBaseline = "middle";
        ctx.fillText(st === START ? "S" : "G", c * cellSize + cellSize / 2, r * cellSize + cellSize / 2);
      }
      ctx.strokeStyle = "rgba(148,163,184,.2)";
      ctx.strokeRect(c * cellSize, r * cellSize, cellSize, cellSize);
    }
  }
}

// ═══ Mouse / Touch ═══
function getCellFromEvent(e) {
  const r = gridCanvas.getBoundingClientRect();
  const sx = gridCanvas.width / r.width, sy = gridCanvas.height / r.height;
  const cx = e.clientX || (e.touches && e.touches[0] ? e.touches[0].clientX : 0);
  const cy = e.clientY || (e.touches && e.touches[0] ? e.touches[0].clientY : 0);
  return { row: Math.floor((cy - r.top) * sy / cellSize), col: Math.floor((cx - r.left) * sx / cellSize) };
}

function applyToolAtCell(row, col) {
  if (!inBounds(row, col) || isRunning) return;
  const isStart = row === startNode.row && col === startNode.col;
  const isGoal = row === goalNode.row && col === goalNode.col;
  if (currentTool === "start") {
    grid[startNode.row][startNode.col] = EMPTY; weights[startNode.row][startNode.col] = 1;
    startNode = { row, col }; grid[row][col] = START;
  } else if (currentTool === "goal") {
    grid[goalNode.row][goalNode.col] = EMPTY; weights[goalNode.row][goalNode.col] = 1;
    goalNode = { row, col }; grid[row][col] = GOAL;
  } else if (currentTool === "wall") {
    if (isStart || isGoal) return;
    grid[row][col] = WALL; weights[row][col] = 1;
  } else if (currentTool === "erase") {
    if (isStart || isGoal) return;
    grid[row][col] = EMPTY; weights[row][col] = 1;
  } else if (currentTool === "weight") {
    if (isStart || isGoal || grid[row][col] === WALL) return;
    weights[row][col] = weights[row][col] >= 5 ? 1 : weights[row][col] + 1;
  }
  drawGrid();
  if (liveMode && !isRunning) runAStar();
}

// ═══ Neighbors ═══
function getNeighbors(node) {
  const dirs = ALLOW_DIAGONALS
    ? [{dr:-1,dc:0,cost:1},{dr:1,dc:0,cost:1},{dr:0,dc:-1,cost:1},{dr:0,dc:1,cost:1},
       {dr:-1,dc:-1,cost:Math.SQRT2},{dr:-1,dc:1,cost:Math.SQRT2},{dr:1,dc:-1,cost:Math.SQRT2},{dr:1,dc:1,cost:Math.SQRT2}]
    : [{dr:-1,dc:0,cost:1},{dr:1,dc:0,cost:1},{dr:0,dc:-1,cost:1},{dr:0,dc:1,cost:1}];
  const result = [];
  for (const d of dirs) {
    const nr = node.row + d.dr, nc = node.col + d.dc;
    if (!inBounds(nr, nc) || grid[nr][nc] === WALL) continue;
    if (Math.abs(d.dr) + Math.abs(d.dc) === 2) {
      if (grid[node.row + d.dr]?.[node.col] === WALL || grid[node.row]?.[node.col + d.dc] === WALL) continue;
    }
    result.push({ row: nr, col: nc, moveCost: d.cost * weights[nr][nc] });
  }
  return result;
}

// ═══ Path utilities ═══
function rebuildPath(from, end) {
  let cur = end; const p = [cur];
  while (key(cur) in from) { cur = from[key(cur)]; p.push(cur); }
  return p.reverse();
}
function countTurns(p) {
  if (p.length < 3) return 0;
  let t = 0;
  for (let i = 2; i < p.length; i++) {
    if (p[i-1].row - p[i-2].row !== p[i].row - p[i-1].row || p[i-1].col - p[i-2].col !== p[i].col - p[i-1].col) t++;
  }
  return t;
}
function computePathCost(p) {
  let t = 0;
  for (const n of p) t += weights[n.row][n.col];
  return t;
}

async function animateStoredPath(path) {
  resetSearchStates();
  for (const n of path) {
    if ((n.row === startNode.row && n.col === startNode.col) || (n.row === goalNode.row && n.col === goalNode.col)) continue;
    grid[n.row][n.col] = PATH;
    drawGrid(); await delay(25);
  }
}

async function animateFinalPath(cameFrom, current, algoName) {
  const path = rebuildPath(cameFrom, current);
  lastRawPath = [...path]; lastPath = [...path];
  await animateStoredPath(path);
  const turns = countTurns(path);
  updateStats({ explored: nodesExploredCount, pathLength: path.length, pathCost: computePathCost(path) + turns * 1.5, turns, algorithm: algoName });
  setStatus(algoName + " finished: path found.");
  setBusy(false);
}

// ═══════════════════════════════════════════════════════════
// GRID SEARCH ALGORITHMS
// ═══════════════════════════════════════════════════════════

async function runGridSearch(mode) {
  resetSearchStates(); drawGrid();
  setBusy(true); nodesExploredCount = 0;
  currentAlgorithmName = mode;
  setStatus("Running " + mode + "...");

  if (mode === "DFS") {
    const stack = [startNode], visited = new Set(), cameFrom = {};
    while (stack.length) {
      const cur = stack.pop(), ck = key(cur);
      if (visited.has(ck)) continue;
      visited.add(ck); nodesExploredCount++;
      if (cur.row === goalNode.row && cur.col === goalNode.col) { await animateFinalPath(cameFrom, cur, mode); return; }
      if (!(cur.row === startNode.row && cur.col === startNode.col)) grid[cur.row][cur.col] = CLOSED;
      for (const nb of getNeighbors(cur).reverse()) {
        const nk = key(nb);
        if (!visited.has(nk)) { cameFrom[nk] = cur; stack.push({row:nb.row,col:nb.col}); if (grid[nb.row][nb.col] === EMPTY) grid[nb.row][nb.col] = OPEN; }
      }
      drawGrid(); await delay(14);
    }

  } else if (mode === "BFS") {
    const queue = [startNode], visited = new Set([key(startNode)]), cameFrom = {};
    while (queue.length) {
      const cur = queue.shift(); nodesExploredCount++;
      if (cur.row === goalNode.row && cur.col === goalNode.col) { await animateFinalPath(cameFrom, cur, mode); return; }
      if (!(cur.row === startNode.row && cur.col === startNode.col)) grid[cur.row][cur.col] = CLOSED;
      for (const nb of getNeighbors(cur)) {
        const nk = key(nb);
        if (!visited.has(nk)) { visited.add(nk); cameFrom[nk] = cur; queue.push({row:nb.row,col:nb.col}); if (grid[nb.row][nb.col] === EMPTY) grid[nb.row][nb.col] = OPEN; }
      }
      drawGrid(); await delay(14);
    }

  } else if (mode === "UCS") {
    // Uniform-Cost Search — priority queue by cumulative g-cost, no heuristic
    const openSet = [{node: startNode, cost: 0}], visited = new Set(), cameFrom = {}, gScore = {};
    gScore[key(startNode)] = 0;
    while (openSet.length) {
      openSet.sort((a, b) => a.cost - b.cost);
      const {node: cur} = openSet.shift();
      const ck = key(cur);
      if (visited.has(ck)) continue;
      visited.add(ck); nodesExploredCount++;
      if (cur.row === goalNode.row && cur.col === goalNode.col) { await animateFinalPath(cameFrom, cur, mode); return; }
      if (!(cur.row === startNode.row && cur.col === startNode.col)) grid[cur.row][cur.col] = CLOSED;
      for (const nb of getNeighbors(cur)) {
        const nk = key(nb);
        const ng = (gScore[ck] || 0) + nb.moveCost;
        if (!visited.has(nk) && (!(nk in gScore) || ng < gScore[nk])) {
          gScore[nk] = ng; cameFrom[nk] = cur;
          openSet.push({node: {row:nb.row,col:nb.col}, cost: ng});
          if (grid[nb.row][nb.col] === EMPTY) grid[nb.row][nb.col] = OPEN;
        }
      }
      drawGrid(); await delay(12);
    }

  } else if (mode === "Greedy Best-First") {
    // Greedy — priority queue by heuristic only
    const openSet = [{node: startNode, h: 0}], visited = new Set(), cameFrom = {};
    while (openSet.length) {
      openSet.sort((a, b) => a.h - b.h);
      const {node: cur} = openSet.shift();
      const ck = key(cur);
      if (visited.has(ck)) continue;
      visited.add(ck); nodesExploredCount++;
      if (cur.row === goalNode.row && cur.col === goalNode.col) { await animateFinalPath(cameFrom, cur, mode); return; }
      if (!(cur.row === startNode.row && cur.col === startNode.col)) grid[cur.row][cur.col] = CLOSED;
      for (const nb of getNeighbors(cur)) {
        const nk = key(nb);
        if (!visited.has(nk)) {
          cameFrom[nk] = cur;
          openSet.push({node: {row:nb.row,col:nb.col}, h: heuristic(nb, goalNode)});
          if (grid[nb.row][nb.col] === EMPTY) grid[nb.row][nb.col] = OPEN;
        }
      }
      drawGrid(); await delay(12);
    }

  } else {
    // Weighted A*
    const openSet = [startNode], cameFrom = {}, gScore = {}, fScore = {};
    for (let r = 0; r < rows; r++) for (let c = 0; c < cols; c++) { gScore[r+","+c] = Infinity; fScore[r+","+c] = Infinity; }
    gScore[key(startNode)] = 0; fScore[key(startNode)] = heuristic(startNode, goalNode);
    while (openSet.length) {
      openSet.sort((a, b) => fScore[key(a)] - fScore[key(b)]);
      const cur = openSet.shift(); nodesExploredCount++;
      if (cur.row === goalNode.row && cur.col === goalNode.col) { await animateFinalPath(cameFrom, cur, mode); return; }
      if (!(cur.row === startNode.row && cur.col === startNode.col)) grid[cur.row][cur.col] = CLOSED;
      for (const nb of getNeighbors(cur)) {
        const prev = cameFrom[key(cur)];
        let tp = 0;
        if (prev) { if (cur.col - prev.col !== nb.col - cur.col || cur.row - prev.row !== nb.row - cur.row) tp = 0.35; }
        const ng = gScore[key(cur)] + nb.moveCost + tp;
        if (ng < gScore[key(nb)]) {
          cameFrom[key(nb)] = {row:cur.row,col:cur.col};
          gScore[key(nb)] = ng; fScore[key(nb)] = ng + heuristic(nb, goalNode);
          if (!openSet.some(n => n.row === nb.row && n.col === nb.col)) {
            openSet.push({row:nb.row,col:nb.col});
            if (grid[nb.row][nb.col] === EMPTY) grid[nb.row][nb.col] = OPEN;
          }
        }
      }
      drawGrid(); await delay(12);
    }
  }
  setStatus(mode + " finished: no path found.");
  setBusy(false);
}

// ═══ Path Smoothing + Robot ═══
function hasLineOfSight(a, b) {
  let x0 = a.col, y0 = a.row;
  const x1 = b.col, y1 = b.row;
  const dx = Math.abs(x1 - x0), dy = Math.abs(y1 - y0);
  const sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1;
  let err = dx - dy;
  while (true) {
    if (!inBounds(y0, x0)) return false;
    if (grid[y0][x0] === WALL) return false;
    if (x0 === x1 && y0 === y1) return true;
    const e2 = err * 2;
    if (e2 > -dy) { err -= dy; x0 += sx; }
    if (e2 < dx) { err += dx; y0 += sy; }
  }
}
function smoothPath(p) {
  if (p.length < 3) return [...p];
  const s = [p[0]]; let i = 0;
  while (i < p.length - 1) {
    let j = p.length - 1;
    while (j > i + 1) { if (hasLineOfSight(p[i], p[j])) break; j--; }
    s.push(p[j]); i = j;
  }
  return s;
}

async function optimizePath() {
  if (isRunning || !lastPath.length) { setStatus("Run a search first."); return; }
  setBusy(true); setStatus("Optimizing route...");
  let wp = [...lastPath];
  for (let i = 0; i < 4; i++) {
    wp = smoothPath(wp); lastPath = [...wp];
    await animateStoredPath(wp);
    const turns = countTurns(wp);
    updateStats({ explored: nodesExploredCount, pathLength: wp.length, pathCost: computePathCost(wp) + turns * 1.5, turns, algorithm: currentAlgorithmName + " + Smooth" });
    await delay(180);
  }
  setStatus("Optimization complete."); setBusy(false);
}

async function animateRobot(path) {
  path = path || lastPath;
  if (isRunning || !path.length) return;
  setBusy(true); setStatus("Animating robot...");
  for (let i = 0; i < path.length; i++) {
    drawGrid(path[i]);
    if (i > 0) {
      const p = path[i-1], c = path[i];
      ctx.beginPath();
      ctx.moveTo(p.col * cellSize + cellSize/2, p.row * cellSize + cellSize/2);
      ctx.lineTo(c.col * cellSize + cellSize/2, c.row * cellSize + cellSize/2);
      ctx.strokeStyle = "#a855f7"; ctx.lineWidth = 3; ctx.stroke(); ctx.lineWidth = 1;
    }
    await delay(80);
  }
  drawGrid(); setStatus("Robot animation complete."); setBusy(false);
}

// ═══ Maze Generation (recursive backtracker) ═══
function generateMaze() {
  if (isRunning) return;
  clearPathOnly();
  for (let r = 0; r < rows; r++) for (let c = 0; c < cols; c++) { grid[r][c] = WALL; weights[r][c] = 1; }
  const visited = new Set();
  function carve(r, c) {
    visited.add(r + "," + c); grid[r][c] = EMPTY;
    const dirs = [{dr:-2,dc:0},{dr:2,dc:0},{dr:0,dc:-2},{dr:0,dc:2}];
    for (let i = dirs.length - 1; i > 0; i--) { const j = Math.floor(Math.random() * (i + 1)); [dirs[i], dirs[j]] = [dirs[j], dirs[i]]; }
    for (const d of dirs) {
      const nr = r + d.dr, nc = c + d.dc;
      if (inBounds(nr, nc) && !visited.has(nr + "," + nc)) {
        grid[r + d.dr/2][c + d.dc/2] = EMPTY;
        carve(nr, nc);
      }
    }
  }
  const sr = startNode.row % 2 === 0 ? startNode.row + 1 : startNode.row;
  const sc = startNode.col % 2 === 0 ? startNode.col + 1 : startNode.col;
  carve(Math.min(sr, rows - 2), Math.min(sc, cols - 2));
  grid[startNode.row][startNode.col] = START;
  grid[goalNode.row][goalNode.col] = GOAL;
  // Clear around start and goal
  for (const n of [{row:startNode.row,col:startNode.col},{row:goalNode.row,col:goalNode.col}]) {
    for (const d of [{dr:-1,dc:0},{dr:1,dc:0},{dr:0,dc:-1},{dr:0,dc:1}]) {
      const nr = n.row+d.dr, nc = n.col+d.dc;
      if (inBounds(nr,nc) && grid[nr][nc] === WALL) grid[nr][nc] = EMPTY;
    }
  }
  drawGrid(); setStatus("Maze generated.");
}

function randomWalls(density) {
  density = density || 0.22;
  if (isRunning) return;
  clearPathOnly();
  for (let r = 0; r < rows; r++) for (let c = 0; c < cols; c++) {
    if ((r === startNode.row && c === startNode.col) || (r === goalNode.row && c === goalNode.col)) continue;
    if (Math.random() < density) { grid[r][c] = WALL; weights[r][c] = 1; }
    else if (grid[r][c] === WALL) grid[r][c] = EMPTY;
  }
  drawGrid(); setStatus("Random obstacles generated.");
}

async function compareAlgorithms() {
  if (isRunning) return;
  setStatus("Comparing DFS then A*...");
  await runGridSearch("DFS"); await delay(500);
  await runGridSearch("Weighted A*");
}

// Shortcut wrappers
function runAStar() { runGridSearch("Weighted A*"); }
function runBFS() { runGridSearch("BFS"); }
function runDFS() { runGridSearch("DFS"); }
function runUCS() { runGridSearch("UCS"); }
function runGreedy() { runGridSearch("Greedy Best-First"); }

// ═══════════════════════════════════════════════════════════
// 1D ARRAY SEARCH VISUALIZATIONS
// ═══════════════════════════════════════════════════════════

let arrayCanvas, actx, searchArray = [], searchTarget = -1;

function initArrayCanvas() {
  arrayCanvas = $("arrayCanvas");
  if (!arrayCanvas) return;
  actx = arrayCanvas.getContext("2d");
  generateSearchArray();
}

function generateSearchArray() {
  searchArray = Array.from({ length: 32 }, () => Math.floor(Math.random() * 95) + 5);
  searchTarget = searchArray[Math.floor(Math.random() * searchArray.length)];
  $("search-target").textContent = searchTarget;
  drawArray();
}

function drawArray(highlights) {
  if (!actx) return;
  highlights = highlights || {};
  const W = arrayCanvas.width, H = arrayCanvas.height;
  actx.clearRect(0, 0, W, H);
  const n = searchArray.length, barW = (W - 20) / n, maxVal = 100;
  for (let i = 0; i < n; i++) {
    const barH = (searchArray[i] / maxVal) * (H - 50);
    const x = 10 + i * barW, y = H - 20 - barH;
    if (highlights.found === i) actx.fillStyle = "#22c55e";
    else if (highlights.checking === i) actx.fillStyle = "#f59e0b";
    else if (highlights.left !== undefined && highlights.right !== undefined && i >= highlights.left && i <= highlights.right) actx.fillStyle = "#60a5fa";
    else if (highlights.eliminated && highlights.eliminated.has(i)) actx.fillStyle = "#334155";
    else actx.fillStyle = "#64748b";
    actx.fillRect(x + 1, y, barW - 2, barH);
    // Value label
    actx.fillStyle = "#cbd5e1"; actx.font = "10px system-ui"; actx.textAlign = "center";
    actx.fillText(String(searchArray[i]), x + barW / 2, y - 4);
  }
  // Target line
  const targetY = H - 20 - (searchTarget / maxVal) * (H - 50);
  actx.strokeStyle = "#ef4444"; actx.setLineDash([4, 3]); actx.lineWidth = 1;
  actx.beginPath(); actx.moveTo(5, targetY); actx.lineTo(W - 5, targetY); actx.stroke();
  actx.setLineDash([]);
  actx.fillStyle = "#ef4444"; actx.font = "bold 11px system-ui"; actx.textAlign = "left";
  actx.fillText("Target: " + searchTarget, 12, targetY - 6);
}

async function runLinearSearch() {
  if (isRunning) return;
  setBusy(true); setStatus("Running Linear Search...");
  const badge = $("array-algo-badge");
  if (badge) badge.textContent = "Linear Search";
  let comparisons = 0;
  for (let i = 0; i < searchArray.length; i++) {
    comparisons++;
    drawArray({ checking: i });
    $("array-comparisons").textContent = comparisons;
    $("array-status").textContent = "Checking index " + i;
    await delay(60);
    if (searchArray[i] === searchTarget) {
      drawArray({ found: i });
      $("array-status").textContent = "Found at index " + i + "!";
      $("array-comparisons").textContent = comparisons;
      setStatus("Linear Search: found in " + comparisons + " comparisons.");
      setBusy(false); return;
    }
  }
  $("array-status").textContent = "Not found";
  setStatus("Linear Search: not found."); setBusy(false);
}

async function runBinarySearch() {
  if (isRunning) return;
  // Sort array for binary search
  const sorted = [...searchArray].sort((a, b) => a - b);
  searchArray = sorted;
  drawArray();
  await delay(300);
  setBusy(true); setStatus("Running Binary Search...");
  const badge = $("array-algo-badge");
  if (badge) badge.textContent = "Binary Search";
  let left = 0, right = searchArray.length - 1, comparisons = 0;
  const eliminated = new Set();
  while (left <= right) {
    const mid = Math.floor((left + right) / 2);
    comparisons++;
    drawArray({ checking: mid, left, right, eliminated });
    $("array-comparisons").textContent = comparisons;
    $("array-status").textContent = "Checking index " + mid + " (value: " + searchArray[mid] + ")";
    await delay(120);
    if (searchArray[mid] === searchTarget) {
      drawArray({ found: mid, eliminated });
      $("array-status").textContent = "Found at index " + mid + "!";
      setStatus("Binary Search: found in " + comparisons + " comparisons.");
      setBusy(false); return;
    } else if (searchArray[mid] < searchTarget) {
      for (let i = left; i <= mid; i++) eliminated.add(i);
      left = mid + 1;
    } else {
      for (let i = mid; i <= right; i++) eliminated.add(i);
      right = mid - 1;
    }
  }
  $("array-status").textContent = "Not found";
  setStatus("Binary Search: not found."); setBusy(false);
}

// ═══════════════════════════════════════════════════════════
// GRADIENT DESCENT (continuous optimization)
// ═══════════════════════════════════════════════════════════

function generateSurface() {
  gradientSurface = [];
  for (let y = 0; y < gradientCanvas.height; y++) {
    const row = [];
    for (let x = 0; x < gradientCanvas.width; x++) {
      const nx = x / 90, ny = y / 90;
      row.push(Math.sin(nx) * Math.cos(ny) + 0.45 * Math.sin(1.8 * nx) + 0.35 * Math.cos(2.4 * ny));
    }
    gradientSurface.push(row);
  }
}

function drawSurface(extras) {
  extras = extras || {};
  const W = gradientCanvas.width, H = gradientCanvas.height;
  const img = gctx.createImageData(W, H);
  for (let y = 0; y < H; y++) {
    for (let x = 0; x < W; x++) {
      const v = gradientSurface[y][x], m = Math.floor((v + 2) * 55);
      const i = (y * W + x) * 4;
      img.data[i] = Math.min(255, m + 10);
      img.data[i+1] = 80 + Math.floor(m * 0.2);
      img.data[i+2] = Math.min(255, 240 - m);
      img.data[i+3] = 255;
    }
  }
  gctx.putImageData(img, 0, 0);
  // Gradient descent trace
  if (gradientTrace.length > 1) {
    gctx.beginPath(); gctx.moveTo(gradientTrace[0].x, gradientTrace[0].y);
    for (let i = 1; i < gradientTrace.length; i++) gctx.lineTo(gradientTrace[i].x, gradientTrace[i].y);
    gctx.strokeStyle = "rgba(255,255,255,.85)"; gctx.lineWidth = 2; gctx.stroke();
  }
  // SA trace
  if (extras.saTrace && extras.saTrace.length > 1) {
    gctx.beginPath(); gctx.moveTo(extras.saTrace[0].x, extras.saTrace[0].y);
    for (let i = 1; i < extras.saTrace.length; i++) gctx.lineTo(extras.saTrace[i].x, extras.saTrace[i].y);
    gctx.strokeStyle = "rgba(239,68,68,.8)"; gctx.lineWidth = 2; gctx.stroke();
  }
  // GA population
  if (extras.gaPopulation) {
    for (const ind of extras.gaPopulation) {
      gctx.fillStyle = "rgba(168,85,247,.7)";
      gctx.beginPath(); gctx.arc(ind.x, ind.y, 4, 0, Math.PI * 2); gctx.fill();
    }
    if (extras.gaBest) {
      gctx.fillStyle = "#22c55e";
      gctx.beginPath(); gctx.arc(extras.gaBest.x, extras.gaBest.y, 7, 0, Math.PI * 2); gctx.fill();
      gctx.strokeStyle = "#fff"; gctx.lineWidth = 2; gctx.stroke();
    }
  }
  // Current point
  gctx.fillStyle = "#ffffff";
  gctx.beginPath(); gctx.arc(gradientPoint.x, gradientPoint.y, 6, 0, Math.PI * 2); gctx.fill();
  gctx.strokeStyle = "#0f172a"; gctx.lineWidth = 2; gctx.stroke();
}

function surfaceGradientAt(x, y) {
  const eps = 1;
  const x1 = Math.min(Math.floor(x + eps), gradientCanvas.width - 1);
  const x0 = Math.max(Math.floor(x - eps), 0);
  const y1 = Math.min(Math.floor(y + eps), gradientCanvas.height - 1);
  const y0 = Math.max(Math.floor(y - eps), 0);
  const cy = Math.floor(y), cx = Math.floor(x);
  return {
    dx: gradientSurface[cy][x1] - gradientSurface[cy][x0],
    dy: gradientSurface[y1][cx] - gradientSurface[y0][cx]
  };
}

function surfaceValueAt(x, y) {
  const fx = Math.max(0, Math.min(Math.floor(x), gradientCanvas.width - 1));
  const fy = Math.max(0, Math.min(Math.floor(y), gradientCanvas.height - 1));
  return gradientSurface[fy][fx];
}

async function runGradientDescent() {
  if (isRunning) return;
  setBusy(true); setStatus("Running Gradient Descent...");
  const badge = $("gradient-badge"); if (badge) badge.textContent = "Gradient Descent";
  gradientPoint = {
    x: Math.floor(Math.random() * (gradientCanvas.width - 40)) + 20,
    y: Math.floor(Math.random() * (gradientCanvas.height - 40)) + 20
  };
  gradientTrace = [{ ...gradientPoint }];
  for (let i = 0; i < 150; i++) {
    const { dx, dy } = surfaceGradientAt(gradientPoint.x, gradientPoint.y);
    gradientPoint.x -= dx * 5; gradientPoint.y -= dy * 5;
    gradientPoint.x = Math.max(0, Math.min(gradientCanvas.width - 1, gradientPoint.x));
    gradientPoint.y = Math.max(0, Math.min(gradientCanvas.height - 1, gradientPoint.y));
    gradientTrace.push({ ...gradientPoint });
    drawSurface(); await delay(20);
  }
  setStatus("Gradient Descent complete (value: " + surfaceValueAt(gradientPoint.x, gradientPoint.y).toFixed(3) + ").");
  setBusy(false);
}

// ═══════════════════════════════════════════════════════════
// SIMULATED ANNEALING
// ═══════════════════════════════════════════════════════════

async function runSimulatedAnnealing() {
  if (isRunning) return;
  setBusy(true); setStatus("Running Simulated Annealing...");
  const badge = $("gradient-badge"); if (badge) badge.textContent = "Simulated Annealing";
  const W = gradientCanvas.width, H = gradientCanvas.height;
  let curX = Math.floor(Math.random() * (W - 40)) + 20;
  let curY = Math.floor(Math.random() * (H - 40)) + 20;
  let curVal = surfaceValueAt(curX, curY);
  let bestX = curX, bestY = curY, bestVal = curVal;
  const saTrace = [{ x: curX, y: curY }];
  let temp = 2.0;
  gradientPoint = { x: curX, y: curY };
  gradientTrace = [];
  for (let i = 0; i < 200; i++) {
    temp *= 0.985;
    const angle = Math.random() * Math.PI * 2;
    const dist = Math.random() * 30 * temp + 2;
    const nx = Math.max(0, Math.min(W - 1, curX + Math.cos(angle) * dist));
    const ny = Math.max(0, Math.min(H - 1, curY + Math.sin(angle) * dist));
    const nv = surfaceValueAt(nx, ny);
    const delta = nv - curVal;
    if (delta < 0 || Math.random() < Math.exp(-delta / Math.max(temp, 0.01))) {
      curX = nx; curY = ny; curVal = nv;
      if (curVal < bestVal) { bestX = curX; bestY = curY; bestVal = curVal; }
    }
    saTrace.push({ x: curX, y: curY });
    gradientPoint = { x: bestX, y: bestY };
    drawSurface({ saTrace }); await delay(16);
  }
  gradientPoint = { x: bestX, y: bestY };
  drawSurface({ saTrace });
  setStatus("Simulated Annealing done (best: " + bestVal.toFixed(3) + ", temp: " + temp.toFixed(3) + ").");
  setBusy(false);
}

// ═══════════════════════════════════════════════════════════
// GENETIC ALGORITHM
// ═══════════════════════════════════════════════════════════

async function runGeneticAlgorithm() {
  if (isRunning) return;
  setBusy(true); setStatus("Running Genetic Algorithm...");
  const badge = $("gradient-badge"); if (badge) badge.textContent = "Genetic Algorithm";
  const W = gradientCanvas.width, H = gradientCanvas.height;
  const POP = 30, GENS = 80;
  let pop = Array.from({ length: POP }, () => ({
    x: Math.random() * (W - 20) + 10,
    y: Math.random() * (H - 20) + 10
  }));
  pop.forEach(ind => { ind.fitness = -surfaceValueAt(ind.x, ind.y); });
  gradientTrace = [];
  for (let gen = 0; gen < GENS; gen++) {
    pop.sort((a, b) => b.fitness - a.fitness);
    const best = pop[0];
    gradientPoint = { x: best.x, y: best.y };
    drawSurface({ gaPopulation: pop, gaBest: best });
    await delay(30);
    // Selection + crossover + mutation
    const next = [pop[0], pop[1]]; // elitism
    while (next.length < POP) {
      // Tournament selection
      const pick = () => {
        const a = pop[Math.floor(Math.random() * POP)];
        const b = pop[Math.floor(Math.random() * POP)];
        return a.fitness > b.fitness ? a : b;
      };
      const p1 = pick(), p2 = pick();
      const cx = (p1.x + p2.x) / 2 + (Math.random() - 0.5) * 40;
      const cy = (p1.y + p2.y) / 2 + (Math.random() - 0.5) * 40;
      const child = { x: Math.max(0, Math.min(W - 1, cx)), y: Math.max(0, Math.min(H - 1, cy)) };
      // Mutation
      if (Math.random() < 0.15) {
        child.x = Math.max(0, Math.min(W - 1, child.x + (Math.random() - 0.5) * 80));
        child.y = Math.max(0, Math.min(H - 1, child.y + (Math.random() - 0.5) * 80));
      }
      child.fitness = -surfaceValueAt(child.x, child.y);
      next.push(child);
    }
    pop = next;
  }
  pop.sort((a, b) => b.fitness - a.fitness);
  gradientPoint = { x: pop[0].x, y: pop[0].y };
  drawSurface({ gaPopulation: pop, gaBest: pop[0] });
  setStatus("Genetic Algorithm done (best: " + surfaceValueAt(pop[0].x, pop[0].y).toFixed(3) + ", " + GENS + " generations).");
  setBusy(false);
}

// ═══════════════════════════════════════════════════════════
// LIVE MODE
// ═══════════════════════════════════════════════════════════

function setLiveMode(next) {
  liveMode = next;
  const btn = $("live-mode");
  btn.setAttribute("aria-pressed", String(liveMode));
  const lbl = $("live-label");
  if (lbl) lbl.textContent = "Live: " + (liveMode ? "On" : "Off");
  setStatus(liveMode ? "Live replanning on." : "Live replanning off.");
}

// ═══════════════════════════════════════════════════════════
// EVENT LISTENERS
// ═══════════════════════════════════════════════════════════

// Tool buttons
$("tool-start").addEventListener("click", () => setTool("start", "Start"));
$("tool-goal").addEventListener("click", () => setTool("goal", "Goal"));
$("tool-wall").addEventListener("click", () => setTool("wall", "Walls"));
$("tool-erase").addEventListener("click", () => setTool("erase", "Erase"));
$("tool-weight").addEventListener("click", () => setTool("weight", "Weight"));

// Algorithm buttons
$("run-astar").addEventListener("click", () => !isRunning && runAStar());
$("run-bfs").addEventListener("click", () => !isRunning && runBFS());
$("run-dfs").addEventListener("click", () => !isRunning && runDFS());
if ($("run-ucs")) $("run-ucs").addEventListener("click", () => !isRunning && runUCS());
if ($("run-greedy")) $("run-greedy").addEventListener("click", () => !isRunning && runGreedy());
$("optimize-path").addEventListener("click", () => optimizePath());
$("animate-robot").addEventListener("click", () => animateRobot());
$("run-gradient").addEventListener("click", () => runGradientDescent());
if ($("run-sa")) $("run-sa").addEventListener("click", () => runSimulatedAnnealing());
if ($("run-ga")) $("run-ga").addEventListener("click", () => runGeneticAlgorithm());

// Utility buttons
$("generate-maze").addEventListener("click", generateMaze);
$("random-scatter").addEventListener("click", () => randomWalls());
$("clear-path").addEventListener("click", clearPathOnly);
$("clear-grid").addEventListener("click", () => {
  if (isRunning) return;
  createGrid(); drawGrid(); updateStats(); setStatus("Grid cleared.");
});
$("compare-mode").addEventListener("click", compareAlgorithms);
$("live-mode").addEventListener("click", () => setLiveMode(!liveMode));

// 1D search buttons
if ($("run-linear")) $("run-linear").addEventListener("click", runLinearSearch);
if ($("run-binary")) $("run-binary").addEventListener("click", runBinarySearch);
if ($("new-array")) $("new-array").addEventListener("click", generateSearchArray);

// Speed slider
const speedSlider = $("speed-slider");
if (speedSlider) {
  speedSlider.addEventListener("input", () => {
    speedMultiplier = parseInt(speedSlider.value);
    $("speed-label").textContent = speedMultiplier + "\u00d7";
  });
}

// Canvas interaction
gridCanvas.addEventListener("mousedown", (e) => { isMouseDown = true; applyToolAtCell(...Object.values(getCellFromEvent(e))); });
gridCanvas.addEventListener("mousemove", (e) => { if (!isMouseDown) return; if (["wall","erase","weight"].includes(currentTool)) applyToolAtCell(...Object.values(getCellFromEvent(e))); });
window.addEventListener("mouseup", () => { isMouseDown = false; });
gridCanvas.addEventListener("touchstart", (e) => { e.preventDefault(); isMouseDown = true; applyToolAtCell(...Object.values(getCellFromEvent(e))); }, { passive: false });
gridCanvas.addEventListener("touchmove", (e) => { e.preventDefault(); if (!isMouseDown) return; if (["wall","erase","weight"].includes(currentTool)) applyToolAtCell(...Object.values(getCellFromEvent(e))); }, { passive: false });
gridCanvas.addEventListener("touchend", (e) => { e.preventDefault(); isMouseDown = false; }, { passive: false });

// Keyboard shortcuts
window.addEventListener("keydown", (e) => {
  if (e.target.tagName === "INPUT") return;
  const k = e.key.toLowerCase();
  if (k === "1") setTool("start", "Start");
  else if (k === "2") setTool("goal", "Goal");
  else if (k === "3") setTool("wall", "Walls");
  else if (k === "4") setTool("erase", "Erase");
  else if (k === "5") setTool("weight", "Weight");
  else if (k === "a" && !isRunning) runAStar();
  else if (k === "b" && !isRunning) runBFS();
  else if (k === "d" && !isRunning) runDFS();
  else if (k === "u" && !isRunning) runUCS();
  else if (k === "o" && !isRunning) optimizePath();
  else if (k === "r" && !isRunning) animateRobot();
  else if (k === "g" && !isRunning) runGradientDescent();
  else if (k === "m" && !isRunning) generateMaze();
  else if (k === "c" && !isRunning) clearPathOnly();
  else if (k === "escape" && !isRunning) { createGrid(); drawGrid(); updateStats(); setStatus("Grid cleared."); }
});

// ═══ Init ═══
createGrid();
generateSurface();
drawGrid();
drawSurface();
updateStats();
setTool("wall", "Walls");
setLiveMode(false);
setStatus("Ready");
initArrayCanvas();
