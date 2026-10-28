/**
 * ESP32 Indoor Positioning Live Dashboard Application
 * Handles SSE streaming, real-time Canvas 2D floorplan rendering,
 * RSSI & IMU metrics, and historical breadcrumbs playback.
 */

// Global State
const state = {
  connected: false,
  simulated: false,
  showGrid: true,
  device_id: 'tracker_01',
  latestPosition: null,
  latestTelemetry: null,
  trail: [], // array of {x, y, timestamp}
  maxTrailLength: 120,
  messageCount: 0,
  rateCalcTime: Date.now(),
  rateSamples: 0,
  anchors: [
    { id: 'A', name: 'Anchor A', x: 0.0, y: 0.0, ssid: 'INDOOR_ANCHOR_A' },
    { id: 'B', name: 'Anchor B', x: 10.0, y: 0.0, ssid: 'INDOOR_ANCHOR_B' }
  ]
};

// Canvas references
const canvas = document.getElementById('floorplan-canvas');
const ctx = canvas.getContext('2d');

// DOM Elements
const elConnDot = document.getElementById('conn-dot');
const elConnText = document.getElementById('conn-text');
const elRateBadge = document.getElementById('rate-badge');
const elTimeBadge = document.getElementById('time-badge');
const elLogCount = document.getElementById('log-count');

const elValX = document.getElementById('val-x');
const elValY = document.getElementById('val-y');
const elValUncertainty = document.getElementById('val-uncertainty');
const elValHeading = document.getElementById('val-heading');
const elValMethod = document.getElementById('val-method');

const elRssiA = document.getElementById('rssi-val-a');
const elRssiB = document.getElementById('rssi-val-b');
const elBarA = document.getElementById('rssi-bar-a');
const elBarB = document.getElementById('rssi-bar-b');
const elDistA = document.getElementById('dist-val-a');
const elDistB = document.getElementById('dist-val-b');
const elPillA = document.getElementById('pill-anchor-a');
const elPillB = document.getElementById('pill-anchor-b');

const elImuAx = document.getElementById('imu-ax');
const elImuAy = document.getElementById('imu-ay');
const elImuAz = document.getElementById('imu-az');
const elImuGz = document.getElementById('imu-gz');
const elValZupt = document.getElementById('val-zupt');

const elCovXx = document.getElementById('cov-xx');
const elCovYy = document.getElementById('cov-yy');
const elCovVx = document.getElementById('cov-vx');
const elCovVy = document.getElementById('cov-vy');
const elCovMode = document.getElementById('cov-mode');
const elLogContainer = document.getElementById('log-container');

// Resize Canvas to fit container
function resizeCanvas() {
  const rect = canvas.parentElement.getBoundingClientRect();
  canvas.width = rect.width * window.devicePixelRatio;
  canvas.height = rect.height * window.devicePixelRatio;
  ctx.scale(window.devicePixelRatio, window.devicePixelRatio);
}
window.addEventListener('resize', resizeCanvas);

// Convert World (meters) to Canvas Pixels
function worldToCanvas(x_m, y_m) {
  const rect = canvas.parentElement.getBoundingClientRect();
  const w = rect.width;
  const h = rect.height;

  // Viewport: X from -2.0m to 12.0m (14m total), Y from -4.0m to 4.0m (8m total)
  const x_min = -2.0, x_max = 12.0;
  const y_min = -3.5, y_max = 3.5;

  const scaleX = w / (x_max - x_min);
  const scaleY = h / (y_max - y_min);
  const scale = Math.min(scaleX, scaleY) * 0.85;

  const originX = w * 0.15;
  const originY = h * 0.50;

  const cx = originX + (x_m - x_min) * scale;
  const cy = originY - (y_m) * scale;

  return { x: cx, y: cy, scale: scale };
}

// Render Canvas
function render() {
  const rect = canvas.parentElement.getBoundingClientRect();
  const w = rect.width;
  const h = rect.height;

  ctx.clearRect(0, 0, w, h);

  // 1. Grid lines
  if (state.showGrid) {
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.04)';
    ctx.lineWidth = 1;

    for (let xm = -2; xm <= 12; xm += 1) {
      const pt1 = worldToCanvas(xm, -4);
      const pt2 = worldToCanvas(xm, 4);
      ctx.beginPath();
      ctx.moveTo(pt1.x, pt1.y);
      ctx.lineTo(pt2.x, pt2.y);
      ctx.stroke();
    }

    for (let ym = -3; ym <= 3; ym += 1) {
      const pt1 = worldToCanvas(-2, ym);
      const pt2 = worldToCanvas(12, ym);
      ctx.beginPath();
      ctx.moveTo(pt1.x, pt1.y);
      ctx.lineTo(pt2.x, pt2.y);
      ctx.stroke();
    }
  }

  // 2. Baseline Corridor Axis between Anchor A (0,0) and Anchor B (10,0)
  const aPos = worldToCanvas(0, 0);
  const bPos = worldToCanvas(10, 0);

  ctx.strokeStyle = 'rgba(0, 242, 254, 0.25)';
  ctx.lineWidth = 2;
  ctx.setLineDash([6, 6]);
  ctx.beginPath();
  ctx.moveTo(aPos.x, aPos.y);
  ctx.lineTo(bPos.x, bPos.y);
  ctx.stroke();
  ctx.setLineDash([]);

  // Baseline Tick marks every 2m
  for (let m = 0; m <= 10; m += 2) {
    const tick = worldToCanvas(m, 0);
    ctx.fillStyle = 'rgba(148, 163, 184, 0.5)';
    ctx.font = '10px JetBrains Mono';
    ctx.fillText(`${m}m`, tick.x - 8, tick.y + 18);
  }

  // 3. Fading Movement Breadcrumbs Trail
  if (state.trail.length > 1) {
    for (let i = 1; i < state.trail.length; i++) {
      const alpha = (i / state.trail.length) * 0.7;
      const pt0 = worldToCanvas(state.trail[i - 1].x, state.trail[i - 1].y);
      const pt1 = worldToCanvas(state.trail[i].x, state.trail[i].y);

      ctx.strokeStyle = `rgba(79, 172, 254, ${alpha})`;
      ctx.lineWidth = 2.5;
      ctx.beginPath();
      ctx.moveTo(pt0.x, pt0.y);
      ctx.lineTo(pt1.x, pt1.y);
      ctx.stroke();
    }
  }

  // 4. Fixed Anchors
  state.anchors.forEach(a => {
    const pt = worldToCanvas(a.x, a.y);
    const now = Date.now() / 1000;
    const wave = (now * 2) % 3;

    // Sonar pulse rings
    ctx.strokeStyle = `rgba(245, 158, 11, ${0.4 - wave * 0.12})`;
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.arc(pt.x, pt.y, 14 + wave * 18, 0, 2 * Math.PI);
    ctx.stroke();

    // Anchor Icon Box
    ctx.fillStyle = '#f59e0b';
    ctx.shadowColor = '#f59e0b';
    ctx.shadowBlur = 10;
    ctx.beginPath();
    ctx.arc(pt.x, pt.y, 8, 0, 2 * Math.PI);
    ctx.fill();
    ctx.shadowBlur = 0;

    // Label
    ctx.fillStyle = '#ffffff';
    ctx.font = 'bold 11px Inter';
    ctx.fillText(a.name, pt.x - 22, pt.y - 14);
  });

  // 5. Moving Tracker Marker & Uncertainty Boundary
  if (state.latestPosition && state.latestPosition.valid) {
    const x = state.latestPosition.x_m;
    const y = state.latestPosition.y_m;
    const unc = state.latestPosition.uncertainty_m || 0.5;
    const heading = (state.latestPosition.heading_deg || 0) * (Math.PI / 180.0);

    const pt = worldToCanvas(x, y);

    // Uncertainty Radius Circle
    const uncRadius = Math.max(12, unc * pt.scale);
    ctx.fillStyle = 'rgba(0, 242, 254, 0.08)';
    ctx.strokeStyle = 'rgba(0, 242, 254, 0.4)';
    ctx.lineWidth = 1.5;
    ctx.setLineDash([4, 4]);
    ctx.beginPath();
    ctx.arc(pt.x, pt.y, uncRadius, 0, 2 * Math.PI);
    ctx.fill();
    ctx.stroke();
    ctx.setLineDash([]);

    // Tracker Outer Glow
    ctx.fillStyle = '#00f2fe';
    ctx.shadowColor = '#00f2fe';
    ctx.shadowBlur = 16;
    ctx.beginPath();
    ctx.arc(pt.x, pt.y, 9, 0, 2 * Math.PI);
    ctx.fill();
    ctx.shadowBlur = 0;

    // Tracker Inner Core
    ctx.fillStyle = '#0a0d14';
    ctx.beginPath();
    ctx.arc(pt.x, pt.y, 4, 0, 2 * Math.PI);
    ctx.fill();

    // Heading Arrow Pointer
    const arrowLen = 22;
    const tipX = pt.x + arrowLen * Math.cos(heading);
    const tipY = pt.y - arrowLen * Math.sin(heading);

    ctx.strokeStyle = '#00f2fe';
    ctx.lineWidth = 2.5;
    ctx.beginPath();
    ctx.moveTo(pt.x, pt.y);
    ctx.lineTo(tipX, tipY);
    ctx.stroke();

    // Tracker Label
    ctx.fillStyle = '#00f2fe';
    ctx.font = 'bold 11px JetBrains Mono';
    ctx.fillText(`Tracker C (${x.toFixed(2)}, ${y.toFixed(2)})m`, pt.x - 55, pt.y - 18);
  }

  requestAnimationFrame(render);
}

// Update UI metrics
function updateMetrics(posData, telData) {
  if (posData && posData.position) {
    const p = posData.position;
    state.latestPosition = p;

    state.messageCount++;
    if (elLogCount) elLogCount.textContent = state.messageCount.toLocaleString();
    if (elTimeBadge) {
      const d = new Date();
      elTimeBadge.textContent = 'Updated: ' + d.toLocaleTimeString();
    }

    if (p.valid && p.x_m !== null) {
      elValX.textContent = p.x_m.toFixed(2);
      elValY.textContent = p.y_m.toFixed(2);
      elValUncertainty.textContent = `±${p.uncertainty_m.toFixed(2)} m`;
      elValHeading.textContent = p.heading_deg !== null ? `${p.heading_deg.toFixed(1)}°` : '--°';
      elValMethod.textContent = p.method;
      elValMethod.style.color = 'var(--accent-cyan)';

      // Record to breadcrumb trail
      state.trail.push({ x: p.x_m, y: p.y_m, time: Date.now() });
      if (state.trail.length > state.maxTrailLength) {
        state.trail.shift();
      }
    } else {
      elValX.textContent = '--.--';
      elValY.textContent = '--.--';
      elValUncertainty.textContent = '±-- m';
      elValMethod.textContent = 'Degraded / Missing Anchor Lock';
      elValMethod.style.color = 'var(--accent-amber)';
    }

    // Distance estimates
    if (posData.anchor_distances) {
      const da = posData.anchor_distances.A;
      const db = posData.anchor_distances.B;
      elDistA.textContent = da !== null && da !== undefined ? `${da.toFixed(2)} m` : '-- m';
      elDistB.textContent = db !== null && db !== undefined ? `${db.toFixed(2)} m` : '-- m';
    }

    // Covariance
    if (p.covariance) {
      elCovXx.textContent = p.covariance.p_xx?.toFixed(4) || '0.0000';
      elCovYy.textContent = p.covariance.p_yy?.toFixed(4) || '0.0000';
      elCovVx.textContent = p.covariance.p_vx?.toFixed(4) || '0.0000';
      elCovVy.textContent = p.covariance.p_vy?.toFixed(4) || '0.0000';
      elCovMode.textContent = p.method || 'Adaptive Kalman';
    }
  }

  if (telData) {
    state.latestTelemetry = telData;

    // RSSI meters
    let ra = null, rb = null;
    (telData.anchors || []).forEach(a => {
      if (a.anchor_id === 'A') ra = a.rssi_dbm;
      if (a.anchor_id === 'B') rb = a.rssi_dbm;
    });

    if (ra !== null) {
      elRssiA.textContent = Math.round(ra);
      const pctA = Math.max(0, Math.min(100, (ra + 100) * 1.5));
      elBarA.style.width = `${pctA}%`;
      elPillA.textContent = 'Detected';
      elPillA.className = 'indicator-pill pill-success';
    } else {
      elRssiA.textContent = '--';
      elBarA.style.width = '0%';
      elPillA.textContent = 'Missing / Sentinel';
      elPillA.className = 'indicator-pill pill-danger';
    }

    if (rb !== null) {
      elRssiB.textContent = Math.round(rb);
      const pctB = Math.max(0, Math.min(100, (rb + 100) * 1.5));
      elBarB.style.width = `${pctB}%`;
      elPillB.textContent = 'Detected';
      elPillB.className = 'indicator-pill pill-success';
    } else {
      elRssiB.textContent = '--';
      elBarB.style.width = '0%';
      elPillB.textContent = 'Missing / Sentinel';
      elPillB.className = 'indicator-pill pill-danger';
    }

    // IMU
    if (telData.imu && telData.imu.accel_mps2) {
      const ax = telData.imu.accel_mps2.x;
      const ay = telData.imu.accel_mps2.y;
      const az = telData.imu.accel_mps2.z;
      const gz = telData.imu.gyro_rads ? telData.imu.gyro_rads.z : 0.0;

      elImuAx.textContent = ax.toFixed(2);
      elImuAy.textContent = ay.toFixed(2);
      elImuAz.textContent = az.toFixed(2);
      elImuGz.textContent = gz.toFixed(2);

      const motionMag = Math.sqrt(ax * ax + ay * ay);
      if (motionMag < 0.25) {
        elValZupt.textContent = 'Stationary (ZUPT Clamped)';
        elValZupt.style.color = 'var(--accent-emerald)';
      } else {
        elValZupt.textContent = 'Active Motion';
        elValZupt.style.color = 'var(--accent-cyan)';
      }
    }
  }

  // Rate calculation
  state.rateSamples++;
  const elapsed = (Date.now() - state.rateCalcTime) / 1000;
  if (elapsed >= 1.0) {
    const hz = (state.rateSamples / elapsed).toFixed(1);
    elRateBadge.textContent = `${hz} Hz`;
    state.rateSamples = 0;
    state.rateCalcTime = Date.now();
  }
}

// Append log entry
function addLogEntry(text, severity = 'info') {
  const timeStr = new Date().toLocaleTimeString();
  const div = document.createElement('div');
  div.className = `log-entry log-${severity}`;
  div.innerHTML = `<span class="log-time">[${timeStr}]</span> ${text}`;
  elLogContainer.appendChild(div);
  elLogContainer.scrollTop = elLogContainer.scrollHeight;

  // Limit log count
  while (elLogContainer.children.length > 50) {
    elLogContainer.removeChild(elLogContainer.firstChild);
  }
}

// Connect SSE stream
function connectSSE() {
  const sse = new EventSource('/api/live');

  sse.onopen = () => {
    state.connected = true;
    elConnDot.className = 'status-dot online';
    elConnText.textContent = 'Streaming';
    addLogEntry('Connected to live telemetry stream (/api/live)', 'info');
  };

  sse.onmessage = (event) => {
    try {
      const msg = JSON.parse(event.data);
      if (msg.type === 'update') {
        updateMetrics(msg.position, msg.telemetry);
      } else if (msg.type === 'event') {
        const sev = msg.severity === 'ERROR' ? 'error' : (msg.severity === 'WARN' ? 'warn' : 'info');
        addLogEntry(`[${msg.event_type}] ${msg.message}`, sev);
      }
    } catch (e) {
      console.error(e);
    }
  };

  sse.onerror = () => {
    state.connected = false;
    elConnDot.className = 'status-dot';
    elConnText.textContent = 'Disconnected';
    elValMethod.textContent = 'Backend Offline / Reconnecting...';
    elValMethod.style.color = 'var(--accent-danger)';
    sse.close();
    setTimeout(connectSSE, 2000);
  };
}

// Fetch initial historical trail
async function loadHistory() {
  try {
    const res = await fetch('/api/history?limit=100');
    if (res.ok) {
      const data = await res.json();
      state.trail = data
        .filter(d => d.valid && d.x_m !== null)
        .map(d => ({ x: d.x_m, y: d.y_m, time: d.server_time * 1000 }));
    }
  } catch (err) {
    console.warn('Could not load history:', err);
  }
}

// UI Event Handlers
document.getElementById('btn-reset-trail').addEventListener('click', () => {
  state.trail = [];
  addLogEntry('Cleared motion trail history.', 'info');
});

document.getElementById('btn-toggle-grid').addEventListener('click', () => {
  state.showGrid = !state.showGrid;
});

// Initialization
window.addEventListener('DOMContentLoaded', () => {
  resizeCanvas();
  loadHistory();
  connectSSE();
  requestAnimationFrame(render);
});
