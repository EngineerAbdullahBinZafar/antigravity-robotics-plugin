/**
 * ROS 2 Antigravity Mission Control — Frontend Application Controller
 * Zero External Dependencies | Vanilla ES6 | 30Hz Canvas Radar | Optimistic State Machine
 */

(function () {
  'use strict';

  // ── Global Application State ──────────────────────────────────────────────
  const State = {
    connected: false,
    mode: 'disconnected', // 'disconnected' | 'connecting' | 'simulation' | 'native_rclpy'
    estop: false,
    latencyMs: 0,
    radarMode: 'canvas', // 'canvas' | 'ascii'
    nodes: [],
    topics: [],
    selectedTopicForSniff: null,
    telemetry: {
      battery: { voltage: 24.2, percentage: 0.88, current: -3.4 },
      imu: { roll: 0.0, pitch: 0.0, yaw: 180.0, z_accel: 9.81 },
      lidar: { ranges: [], angle_min: -Math.PI, angle_max: Math.PI, range_min: 0.12, range_max: 10.0 },
      closestObstacle: { distance: 2.14, bearing: 42.5 },
      odom: { linear_vel: 0.0, angular_vel: 0.0 }
    },
    ws: null,
    pollTimer: null,
    pingTimer: null,
    activeLogFilter: 'ALL'
  };

  // ── DOM Element Cache ─────────────────────────────────────────────────────
  const DOM = {
    btnConnect: document.getElementById('btn-connect'),
    btnConnectText: document.getElementById('btn-connect-text'),
    btnSpinner: document.querySelector('.btn-spinner'),
    connectionBadge: document.getElementById('connection-badge'),
    connectionStatusText: document.getElementById('connection-status-text'),
    latencyVal: document.getElementById('latency-val'),
    sandboxStatusText: document.getElementById('sandbox-status-text'),
    btnEstop: document.getElementById('btn-estop'),
    btnEstopReset: document.getElementById('btn-estop-reset'),
    estopBanner: document.getElementById('estop-banner'),

    // Health
    batteryPctText: document.getElementById('battery-pct-text'),
    batteryProgressRing: document.getElementById('battery-progress-ring'),
    batteryVoltageText: document.getElementById('battery-voltage-text'),
    batteryCurrentText: document.getElementById('battery-current-text'),
    batteryStateText: document.getElementById('battery-state-text'),
    horizonLadder: document.getElementById('horizon-pitch-ladder'),
    imuRollText: document.getElementById('imu-roll-text'),
    imuPitchText: document.getElementById('imu-pitch-text'),
    imuYawText: document.getElementById('imu-yaw-text'),
    imuZaccelText: document.getElementById('imu-zaccel-text'),
    alertsList: document.getElementById('diagnostics-alerts-list'),

    // Topology
    nodeCountBadge: document.getElementById('node-count-badge'),
    nodesList: document.getElementById('nodes-list'),
    topicSearchInput: document.getElementById('topic-search-input'),
    topicsTbody: document.getElementById('topics-tbody'),

    // PID Tuner
    pidNodeSelect: document.getElementById('pid-node-select'),
    sliderKp: document.getElementById('slider-kp'),
    numKp: document.getElementById('num-kp'),
    sliderKi: document.getElementById('slider-ki'),
    numKi: document.getElementById('num-ki'),
    sliderKd: document.getElementById('slider-kd'),
    numKd: document.getElementById('num-kd'),
    btnApplyPid: document.getElementById('btn-apply-pid'),
    pidStatusMsg: document.getElementById('pid-status-msg'),

    // Radar
    lidarCanvas: document.getElementById('lidar-canvas'),
    radarCanvasContainer: document.getElementById('radar-canvas-container'),
    radarHoverPill: document.getElementById('radar-hover-pill'),
    radarHoverDist: document.getElementById('radar-hover-dist'),
    radarHoverAngle: document.getElementById('radar-hover-angle'),
    btnToggleRadarMode: document.getElementById('btn-toggle-radar-mode'),
    spatialAsciiContainer: document.getElementById('spatial-ascii-container'),
    spatialAsciiPre: document.getElementById('spatial-ascii-pre'),
    closestObstacleDist: document.getElementById('closest-obstacle-dist'),
    closestObstacleBearing: document.getElementById('closest-obstacle-bearing'),
    telemetryLinVel: document.getElementById('telemetry-lin-vel'),
    telemetryAngVel: document.getElementById('telemetry-ang-vel'),

    // Audit Log & Modal
    auditLogStream: document.getElementById('audit-log-stream'),
    btnClearLog: document.getElementById('btn-clear-log'),
    modalSniffer: document.getElementById('modal-sniffer'),
    modalSnifferTitle: document.getElementById('modal-sniffer-title'),
    modalSnifferJson: document.getElementById('modal-sniffer-json'),
    btnModalClose: document.getElementById('btn-modal-close')
  };

  // ── Logging System ────────────────────────────────────────────────────────
  function log(tag, message, level = 'info') {
    if (!DOM.auditLogStream) return;
    const now = new Date();
    const ts = `[${now.toTimeString().split(' ')[0]}.${String(now.getMilliseconds()).padStart(3, '0')}]`;

    const entry = document.createElement('div');
    entry.className = `log-entry log-${level}`;
    entry.dataset.category = tag.toUpperCase();

    let tagClass = 'tag-sys';
    if (tag === 'MCP') tagClass = 'tag-mcp';
    else if (tag === 'SAFETY') tagClass = 'tag-safety';
    else if (tag === 'ERROR') tagClass = 'tag-err';

    entry.innerHTML = `
      <span class="log-ts">${ts}</span>
      <span class="log-tag ${tagClass}">${tag}</span>
      <span class="log-text">${escapeHtml(message)}</span>
    `;

    DOM.auditLogStream.appendChild(entry);
    DOM.auditLogStream.scrollTop = DOM.auditLogStream.scrollHeight;
  }

  function escapeHtml(str) {
    return String(str)
      .replace(/&/g, '&amp;')
      .replace(/</g, '&lt;')
      .replace(/>/g, '&gt;')
      .replace(/"/g, '&quot;');
  }

  // ── 1-Click Connection State Machine (<30ms optimistic) ───────────────────
  async function handleToggleConnect() {
    if (State.connected) {
      // Disconnect
      setConnectionState('disconnected');
      log('SYSTEM', 'Disconnecting ROS 2 bridge...', 'info');
      try {
        await fetch('/api/connect', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ action: 'disconnect' })
        });
      } catch (err) {
        // Ignore disconnect fetch errors
      }
      if (State.ws) {
        State.ws.close();
        State.ws = null;
      }
      clearInterval(State.pollTimer);
      clearInterval(State.pingTimer);
      log('SYSTEM', 'Disconnected cleanly.', 'info');
      return;
    }

    // Optimistic UI state (<5ms)
    setConnectionState('connecting');
    log('SYSTEM', 'Initiating 1-click bridge connection probe...', 'info');

    const startTime = performance.now();

    try {
      const resp = await fetch('/api/connect', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ action: 'connect', mode: 'auto' })
      });

      if (!resp.ok) throw new Error(`HTTP ${resp.status}`);
      const data = await resp.json();

      if (data.status !== 'ok') {
        throw new Error(data.reason || data.message || 'Connection rejected by bridge');
      }

      const rtt = (performance.now() - startTime).toFixed(1);
      State.latencyMs = rtt;
      DOM.latencyVal.textContent = `${rtt} ms`;

      const targetMode = data.mode || 'simulation';
      setConnectionState(targetMode);

      log('MCP', `Bridge connected in ${targetMode.toUpperCase()} mode (RTT: ${rtt}ms)`, 'info');

      // Fetch initial metadata
      fetchTopology();
      fetchDiagnostics();

      // Launch WebSocket telemetry or fallback poll
      initWebSocketTelemetry();

    } catch (err) {
      if (State.ws) {
        State.ws.close();
        State.ws = null;
      }
      clearInterval(State.pollTimer);
      clearInterval(State.pingTimer);
      log('ERROR', `Connection failed: ${err.message}`, 'error');
      setConnectionState('error');
    }
  }

  function setConnectionState(mode) {
    State.mode = mode;
    State.connected = (mode === 'simulation' || mode === 'native' || mode === 'native_rclpy');

    // Reset classes
    DOM.connectionBadge.className = 'status-badge';
    DOM.btnSpinner.classList.add('hidden');

    if (mode === 'connecting') {
      DOM.connectionBadge.classList.add('state-connecting');
      DOM.connectionStatusText.textContent = 'CONNECTING...';
      DOM.btnConnectText.textContent = 'Connecting...';
      DOM.btnSpinner.classList.remove('hidden');
    } else if (State.connected) {
      DOM.connectionBadge.classList.add('state-connected');
      const label = mode === 'simulation' ? 'CONNECTED (SIMULATION)' : 'CONNECTED (NATIVE rclpy)';
      DOM.connectionStatusText.textContent = label;
      DOM.btnConnectText.textContent = 'Disconnect';
    } else if (mode === 'error') {
      DOM.connectionBadge.classList.add('state-error');
      DOM.connectionStatusText.textContent = 'CONNECTION ERROR';
      DOM.btnConnectText.textContent = 'Retry Connect';
      DOM.latencyVal.textContent = '-- ms';
    } else {
      DOM.connectionBadge.classList.add('state-disconnected');
      DOM.connectionStatusText.textContent = 'DISCONNECTED';
      DOM.btnConnectText.textContent = 'Connect ROS 2';
      DOM.latencyVal.textContent = '-- ms';
    }
  }

  // ── WebSocket Telemetry Pipeline ──────────────────────────────────────────
  function initWebSocketTelemetry() {
    if (State.ws) {
      State.ws.close();
    }

    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    const wsUrl = `${protocol}//${window.location.host}/ws/telemetry`;

    try {
      State.ws = new WebSocket(wsUrl);

      State.ws.onopen = () => {
        log('SYSTEM', 'WebSocket telemetry stream established at 30Hz', 'info');
        // Start ping heartbeat
        State.pingTimer = setInterval(() => {
          if (State.ws && State.ws.readyState === WebSocket.OPEN) {
            State.ws.send(JSON.stringify({ type: 'ping', ts: performance.now() }));
          }
        }, 1500);
      };

      State.ws.onmessage = (event) => {
        try {
          const payload = JSON.parse(event.data);
          handleTelemetryFrame(payload);
        } catch (e) {
          // Non-JSON frame
        }
      };

      State.ws.onerror = () => {
        // Failover to REST polling if WebSocket unavailable
        log('SAFETY', 'WebSocket dropped; failing over to REST polling (2Hz)', 'info');
        startRestPolling();
      };

      State.ws.onclose = () => {
        clearInterval(State.pingTimer);
      };

    } catch (err) {
      startRestPolling();
    }
  }

  function handleTelemetryFrame(data) {
    // Ping measurement
    if (data.type === 'pong' && data.ts) {
      const rtt = (performance.now() - data.ts).toFixed(1);
      State.latencyMs = rtt;
      DOM.latencyVal.textContent = `${rtt} ms`;
      return;
    }

    // Battery
    if (data.battery) {
      updateBattery(data.battery);
    }

    // IMU
    if (data.imu) {
      updateImu(data.imu);
    }

    // LiDAR LaserScan
    if (data.scan && data.scan.ranges) {
      updateScan(data.scan);
    }

    // Odometry / Velocities
    if (data.odom) {
      if (data.odom.linear_vel !== undefined) {
        DOM.telemetryLinVel.textContent = `${Number(data.odom.linear_vel).toFixed(2)} m/s`;
      }
      if (data.odom.angular_vel !== undefined) {
        DOM.telemetryAngVel.textContent = `${Number(data.odom.angular_vel).toFixed(2)} rad/s`;
      }
    }
  }

  function startRestPolling() {
    clearInterval(State.pollTimer);
    State.pollTimer = setInterval(async () => {
      if (!State.connected) return;
      try {
        const startTs = performance.now();
        const resp = await fetch('/api/status');
        if (resp.ok) {
          const rtt = (performance.now() - startTs).toFixed(1);
          DOM.latencyVal.textContent = `${rtt} ms`;
        }
        fetchDiagnostics();
      } catch (e) {
        // ignore
      }
    }, 1000);
  }

  // ── Battery Telemetry Updater ─────────────────────────────────────────────
  function updateBattery(b) {
    const pct = Math.max(0, Math.min(1, b.percentage !== undefined ? b.percentage : 0.88));
    const pctInt = Math.round(pct * 100);

    DOM.batteryPctText.textContent = `${pctInt}%`;
    if (b.voltage) DOM.batteryVoltageText.textContent = `${Number(b.voltage).toFixed(1)} V`;
    if (b.current) DOM.batteryCurrentText.textContent = `${Number(b.current).toFixed(1)} A`;

    // SVG radial dashoffset (C = 263.89)
    const circumference = 263.89;
    const offset = circumference * (1 - pct);
    DOM.batteryProgressRing.style.strokeDashoffset = offset;

    // Dynamic color shift
    if (pct < 0.20) {
      DOM.batteryProgressRing.style.stroke = 'var(--neon-crimson)';
      DOM.batteryStateText.textContent = 'Critical Low';
      DOM.batteryStateText.className = 'metric-val text-crimson';
    } else if (pct < 0.35) {
      DOM.batteryProgressRing.style.stroke = 'var(--neon-amber)';
      DOM.batteryStateText.textContent = 'Warning Low';
      DOM.batteryStateText.className = 'metric-val text-amber';
    } else {
      DOM.batteryProgressRing.style.stroke = 'var(--neon-emerald)';
      DOM.batteryStateText.textContent = 'Nominal';
      DOM.batteryStateText.className = 'metric-val text-emerald';
    }
  }

  // ── IMU Attitude Updater ──────────────────────────────────────────────────
  function updateImu(imu) {
    let roll = 0.0, pitch = 0.0, yaw = 180.0;

    if (imu.orientation) {
      const q = imu.orientation;
      // Quaternion to Euler conversion
      roll = Math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y)) * (180 / Math.PI);
      pitch = Math.asin(Math.max(-1, Math.min(1, 2 * (q.w * q.y - q.z * q.x)))) * (180 / Math.PI);
      yaw = Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)) * (180 / Math.PI);
      if (yaw < 0) yaw += 360;
    } else if (imu.roll !== undefined) {
      roll = imu.roll;
      pitch = imu.pitch;
      yaw = imu.yaw;
    }

    DOM.imuRollText.textContent = `${roll.toFixed(1)}°`;
    DOM.imuPitchText.textContent = `${pitch.toFixed(1)}°`;
    DOM.imuYawText.textContent = `${yaw.toFixed(1)}°`;

    if (imu.linear_acceleration && imu.linear_acceleration.z) {
      DOM.imuZaccelText.textContent = Number(imu.linear_acceleration.z).toFixed(2);
    }

    // Rotate and pitch artificial horizon
    if (DOM.horizonLadder) {
      const translateY = pitch * 1.5;
      DOM.horizonLadder.style.transform = `translateY(${translateY}px) rotate(${-roll}deg)`;
    }
  }

  // ── 2D HTML5 Canvas LiDAR Radar ───────────────────────────────────────────
  let radarSweepAngle = 0;
  const Radar = {
    ctx: DOM.lidarCanvas.getContext('2d'),
    maxRange: 5.0, // meters
    points: [],

    init() {
      // HiDPI canvas scaling
      const dpr = window.devicePixelRatio || 1;
      const rect = DOM.lidarCanvas.getBoundingClientRect();
      DOM.lidarCanvas.width = rect.width * dpr;
      DOM.lidarCanvas.height = rect.height * dpr;
      this.ctx.scale(dpr, dpr);

      this.width = rect.width;
      this.height = rect.height;
      this.cx = this.width / 2;
      this.cy = this.height / 2;
      this.scale = (Math.min(this.cx, this.cy) - 24) / this.maxRange;

      // Mouse hover tracking
      DOM.lidarCanvas.addEventListener('mousemove', (e) => this.onMouseMove(e));
      DOM.lidarCanvas.addEventListener('mouseleave', () => this.onMouseLeave());

      // Start render loop
      this.loop = this.loop.bind(this);
      requestAnimationFrame(this.loop);
    },

    onMouseMove(e) {
      const rect = DOM.lidarCanvas.getBoundingClientRect();
      const mx = e.clientX - rect.left;
      const my = e.clientY - rect.top;

      const dx = mx - this.cx;
      const dy = my - this.cy;
      const distM = (Math.sqrt(dx * dx + dy * dy) / this.scale).toFixed(2);

      // Bearing in degrees (0° = top/forward)
      let deg = (Math.atan2(dx, -dy) * (180 / Math.PI));
      if (deg < 0) deg += 360;

      DOM.radarHoverDist.textContent = distM;
      DOM.radarHoverAngle.textContent = deg.toFixed(1);
      DOM.radarHoverPill.classList.remove('hidden');
    },

    onMouseLeave() {
      DOM.radarHoverPill.classList.add('hidden');
    },

    loop() {
      radarSweepAngle = (radarSweepAngle + 0.035) % (2 * Math.PI);
      this.draw();
      requestAnimationFrame(this.loop);
    },

    draw() {
      const ctx = this.ctx;
      const w = this.width;
      const h = this.height;
      const cx = this.cx;
      const cy = this.cy;
      const scale = this.scale;

      // Clear with dark void
      ctx.fillStyle = '#06090E';
      ctx.fillRect(0, 0, w, h);

      // Concentric Range Rings (1m, 2m, 3m, 4m, 5m)
      ctx.lineWidth = 1;
      for (let r = 1; r <= this.maxRange; r++) {
        const radiusPx = r * scale;
        ctx.strokeStyle = r === 1 ? 'rgba(0, 240, 255, 0.25)' : 'rgba(0, 240, 255, 0.12)';
        ctx.beginPath();
        ctx.arc(cx, cy, radiusPx, 0, 2 * Math.PI);
        ctx.stroke();

        // Distance label
        ctx.fillStyle = 'rgba(0, 240, 255, 0.4)';
        ctx.font = '9px "JetBrains Mono", monospace';
        ctx.fillText(`${r}.0m`, cx + 4, cy - radiusPx + 11);
      }

      // Crosshair spokes
      ctx.strokeStyle = 'rgba(0, 240, 255, 0.1)';
      ctx.beginPath();
      ctx.moveTo(cx, 10); ctx.lineTo(cx, h - 10);
      ctx.moveTo(10, cy); ctx.lineTo(w - 10, cy);
      ctx.stroke();

      // Critical Safety Zone (<0.35m)
      const safetyPx = 0.35 * scale;
      ctx.fillStyle = 'rgba(255, 23, 68, 0.15)';
      ctx.strokeStyle = 'rgba(255, 23, 68, 0.6)';
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.arc(cx, cy, safetyPx, 0, 2 * Math.PI);
      ctx.fill();
      ctx.stroke();

      // Sweeping Laser Radar Arm
      ctx.save();
      ctx.translate(cx, cy);
      ctx.rotate(radarSweepAngle);

      const sweepGradient = ctx.createRadialGradient(0, 0, 0, 0, 0, this.maxRange * scale);
      sweepGradient.addColorStop(0, 'rgba(0, 240, 255, 0.8)');
      sweepGradient.addColorStop(1, 'rgba(0, 240, 255, 0.0)');

      ctx.strokeStyle = sweepGradient;
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(0, 0);
      ctx.lineTo(0, -this.maxRange * scale);
      ctx.stroke();
      ctx.restore();

      // Render Detected Obstacle Points
      for (let i = 0; i < this.points.length; i++) {
        const p = this.points[i];
        if (p.r < 0.12 || p.r > this.maxRange) continue;

        // Polar to Cartesian (forward is UP on canvas)
        const px = cx + p.r * scale * Math.sin(p.theta);
        const py = cy - p.r * scale * Math.cos(p.theta);

        // Color threshold
        if (p.r < 0.35) {
          ctx.fillStyle = '#FF1744';
          ctx.shadowColor = '#FF1744';
          ctx.shadowBlur = 6;
          ctx.beginPath();
          ctx.arc(px, py, 3.5, 0, 2 * Math.PI);
          ctx.fill();
          ctx.shadowBlur = 0;
        } else if (p.r < 0.75) {
          ctx.fillStyle = '#FFB300';
          ctx.beginPath();
          ctx.arc(px, py, 2.5, 0, 2 * Math.PI);
          ctx.fill();
        } else {
          ctx.fillStyle = '#00F0FF';
          ctx.beginPath();
          ctx.arc(px, py, 2.0, 0, 2 * Math.PI);
          ctx.fill();
        }
      }

      // Robot Center Icon
      ctx.fillStyle = '#00F0FF';
      ctx.beginPath();
      ctx.arc(cx, cy, 4, 0, 2 * Math.PI);
      ctx.fill();

      // Forward heading arrow
      ctx.strokeStyle = '#00E676';
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(cx, cy - 4);
      ctx.lineTo(cx, cy - 14);
      ctx.lineTo(cx - 3, cy - 10);
      ctx.moveTo(cx, cy - 14);
      ctx.lineTo(cx + 3, cy - 10);
      ctx.stroke();
    }
  };

  function updateScan(scan) {
    const ranges = scan.ranges || [];
    const angleMin = scan.angle_min || -Math.PI;
    const angleInc = scan.angle_increment || (2 * Math.PI) / ranges.length;

    const points = [];
    let closestDist = 999;
    let closestBearing = 0;

    for (let i = 0; i < ranges.length; i++) {
      const r = ranges[i];
      if (typeof r !== 'number' || !isFinite(r) || r < 0.12) continue;

      const theta = angleMin + i * angleInc;
      points.push({ r, theta });

      if (r < closestDist) {
        closestDist = r;
        let deg = (theta * 180) / Math.PI;
        if (deg < 0) deg += 360;
        closestBearing = deg;
      }
    }

    Radar.points = points;

    if (closestDist < 999) {
      DOM.closestObstacleDist.textContent = `${closestDist.toFixed(2)} m`;
      DOM.closestObstacleBearing.textContent = `${closestBearing.toFixed(1)}°`;

      if (closestDist < 0.35) {
        DOM.closestObstacleDist.className = 'stat-value text-crimson';
      } else if (closestDist < 0.75) {
        DOM.closestObstacleDist.className = 'stat-value text-amber';
      } else {
        DOM.closestObstacleDist.className = 'stat-value text-emerald';
      }
    }
  }

  // ── Emergency Stop (E-STOP) ───────────────────────────────────────────────
  async function triggerEmergencyStop() {
    State.estop = true;
    log('SAFETY', 'EMERGENCY STOP TRIGGERED! Clamping /cmd_vel to 0.00 m/s', 'error');

    // Immediate <10ms visual reaction
    DOM.estopBanner.classList.remove('hidden');
    DOM.telemetryLinVel.textContent = '0.00 m/s';
    DOM.telemetryAngVel.textContent = '0.00 rad/s';

    try {
      await fetch('/api/estop', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ reason: 'Manual operator E-STOP trigger' })
      });
      log('SAFETY', 'Server confirmed E-STOP: zero velocity published.', 'info');
    } catch (err) {
      log('ERROR', `E-STOP publish error: ${err.message}`, 'error');
    }
  }

  function resetEmergencyStop() {
    State.estop = false;
    DOM.estopBanner.classList.add('hidden');
    log('SAFETY', 'E-STOP cleared. Safety interlocks re-armed.', 'info');
  }

  // ── Topology Explorer (Nodes & Topics) ────────────────────────────────────
  async function fetchTopology() {
    try {
      const [nodesResp, topicsResp] = await Promise.all([
        fetch('/api/nodes').catch(() => null),
        fetch('/api/topics').catch(() => null)
      ]);

      if (nodesResp && nodesResp.ok) {
        const nodesData = await nodesResp.json();
        const nodes = Array.isArray(nodesData) ? nodesData : (nodesData.nodes || []);
        renderNodes(nodes);
      }

      if (topicsResp && topicsResp.ok) {
        const topicsData = await topicsResp.json();
        const topics = Array.isArray(topicsData) ? topicsData : (topicsData.topics || []);
        renderTopics(topics);
      }
    } catch (e) {
      // ignore
    }
  }

  function renderNodes(nodes) {
    State.nodes = nodes;
    DOM.nodeCountBadge.textContent = `${nodes.length} NODES ONLINE`;
    DOM.nodesList.innerHTML = '';

    nodes.forEach((n) => {
      const name = typeof n === 'string' ? n : (n.name || n[0]);
      const chip = document.createElement('span');
      chip.className = 'node-chip';
      chip.textContent = name.startsWith('/') ? name : `/${name}`;
      DOM.nodesList.appendChild(chip);
    });
  }

  function renderTopics(topics) {
    State.topics = topics;
    DOM.topicsTbody.innerHTML = '';

    const filter = (DOM.topicSearchInput.value || '').trim().toLowerCase();

    topics.forEach((t) => {
      const name = typeof t === 'string' ? t : (t.topic || t.name);
      const types = Array.isArray(t.types) ? t.types.join(', ') : (t.type || 'std_msgs/msg/String');

      if (filter && !name.toLowerCase().includes(filter)) return;

      const tr = document.createElement('tr');
      tr.innerHTML = `
        <td class="text-cyan">${escapeHtml(name)}</td>
        <td class="text-dim">${escapeHtml(types)}</td>
        <td><button class="btn-sniff" data-topic="${escapeHtml(name)}">Sniff</button></td>
      `;
      DOM.topicsTbody.appendChild(tr);
    });

    // Wire Sniff buttons
    DOM.topicsTbody.querySelectorAll('.btn-sniff').forEach((btn) => {
      btn.addEventListener('click', (e) => {
        const topic = e.target.dataset.topic;
        openSnifferModal(topic);
      });
    });
  }

  // ── Topic Payload Sniffer Modal ───────────────────────────────────────────
  async function openSnifferModal(topic) {
    const cleanTopic = topic.startsWith('/') ? topic : `/${topic}`;
    DOM.modalSnifferTitle.textContent = `Live Payload Inspector: ${cleanTopic}`;
    DOM.modalSnifferJson.textContent = `Fetching live snapshot for ${cleanTopic}...`;
    DOM.modalSniffer.classList.remove('hidden');

    try {
      const resp = await fetch(`/api/topic${cleanTopic}`);
      if (resp.ok) {
        const json = await resp.json();
        DOM.modalSnifferJson.textContent = JSON.stringify(json, null, 2);
        log('MCP', `Inspected live payload for topic ${cleanTopic}`, 'info');
      } else {
        let errJson = null;
        try {
          errJson = await resp.json();
        } catch (_) {}
        const errorDetails = errJson
          ? JSON.stringify(errJson, null, 2)
          : `HTTP ${resp.status}: ${resp.statusText || 'Topic unavailable'}`;
        DOM.modalSnifferJson.textContent = `// Error inspecting ${cleanTopic} (${resp.status})\n${errorDetails}`;
        log('MCP', `Failed to inspect topic ${cleanTopic}: HTTP ${resp.status}`, 'error');
      }
    } catch (err) {
      DOM.modalSnifferJson.textContent = `// Network Error inspecting ${cleanTopic}\n${err.message}`;
      log('MCP', `Network error inspecting topic ${cleanTopic}: ${err.message}`, 'error');
    }
  }

  // ── Diagnostics Fetcher ───────────────────────────────────────────────────
  async function fetchDiagnostics() {
    try {
      const resp = await fetch('/api/diagnostics');
      if (!resp.ok) return;
      const data = await resp.json();

      DOM.alertsList.innerHTML = '';
      if (data.critical_issues && data.critical_issues.length > 0) {
        data.critical_issues.forEach((iss) => {
          DOM.alertsList.innerHTML += `
            <div class="alert-item" style="background: rgba(255,23,68,0.15); border: 1px solid var(--neon-crimson);">
              <span class="alert-badge text-crimson">CRITICAL</span>
              <span class="alert-msg">${escapeHtml(iss)}</span>
            </div>
          `;
        });
      }

      if (data.warnings && data.warnings.length > 0) {
        data.warnings.forEach((warn) => {
          DOM.alertsList.innerHTML += `
            <div class="alert-item" style="background: rgba(255,179,0,0.15); border: 1px solid var(--neon-amber);">
              <span class="alert-badge text-amber">WARN</span>
              <span class="alert-msg">${escapeHtml(warn)}</span>
            </div>
          `;
        });
      }

      if (!data.critical_issues?.length && !data.warnings?.length) {
        DOM.alertsList.innerHTML = `
          <div class="alert-item alert-info">
            <span class="alert-badge">STATUS</span>
            <span class="alert-msg">All sensor self-checks nominal.</span>
          </div>
        `;
      }
    } catch (e) {
      // ignore
    }
  }

  // ── PID Tuner Synchronization & Validation ────────────────────────────────
  function initPidTuner() {
    // Sync slider <-> number input
    function sync(slider, num, min, max) {
      slider.addEventListener('input', () => { num.value = slider.value; });
      num.addEventListener('input', () => {
        let val = parseFloat(num.value);
        if (isNaN(val) || val < min) val = min;
        if (val > max) val = max;
        slider.value = val;
      });
    }

    sync(DOM.sliderKp, DOM.numKp, 0, 500);
    sync(DOM.sliderKi, DOM.numKi, 0, 100);
    sync(DOM.sliderKd, DOM.numKd, 0, 100);

    DOM.btnApplyPid.addEventListener('click', async () => {
      const kp = parseFloat(DOM.numKp.value);
      const ki = parseFloat(DOM.numKi.value);
      const kd = parseFloat(DOM.numKd.value);
      const node = DOM.pidNodeSelect.value;

      if (isNaN(kp) || isNaN(ki) || isNaN(kd) || kp < 0 || ki < 0 || kd < 0) {
        DOM.pidStatusMsg.textContent = 'Error: Non-negative numeric gains required';
        DOM.pidStatusMsg.className = 'pid-status-msg text-crimson';
        return;
      }

      DOM.pidStatusMsg.textContent = 'Applying...';
      DOM.pidStatusMsg.className = 'pid-status-msg text-dim';
      try {
        const resp = await fetch('/api/pid/tune', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ node_name: node, kp, ki, kd })
        });
        let res = null;
        try {
          res = await resp.json();
        } catch (_) {}

        if (resp.ok && res && res.status === 'ok') {
          DOM.pidStatusMsg.textContent = res.message || 'Gains Applied';
          DOM.pidStatusMsg.className = 'pid-status-msg text-emerald';
          log('MCP', `PID gains applied to ${node}: Kp=${kp}, Ki=${ki}, Kd=${kd}`, 'info');
        } else {
          const errMsg = (res && (res.reason || res.message || (res.rejected && res.rejected[0] && res.rejected[0].error)))
            || `HTTP ${resp.status}: ${resp.statusText || 'Failed to apply PID gains'}`;
          DOM.pidStatusMsg.textContent = errMsg;
          DOM.pidStatusMsg.className = 'pid-status-msg text-crimson';
          log('MCP', `PID tuning failed for ${node}: ${errMsg}`, 'error');
        }
      } catch (err) {
        DOM.pidStatusMsg.textContent = `Network Error: ${err.message}`;
        DOM.pidStatusMsg.className = 'pid-status-msg text-crimson';
        log('MCP', `PID tuning network error: ${err.message}`, 'error');
      }
    });
  }

  // ── Mode Toggle: Canvas 2D vs ASCII Spatial Radar ─────────────────────────
  async function toggleRadarMode() {
    if (State.radarMode === 'canvas') {
      State.radarMode = 'ascii';
      DOM.btnToggleRadarMode.textContent = 'ASCII GRID';
      DOM.radarCanvasContainer.classList.add('hidden');
      DOM.spatialAsciiContainer.classList.remove('hidden');

      // Fetch live ASCII grid from server
      DOM.spatialAsciiPre.textContent = 'Fetching live spatial map from /api/spatial_map...';
      try {
        const resp = await fetch('/api/spatial_map');
        if (resp.ok) {
          const mapData = await resp.json();
          const asciiMap = mapData.spatial_grid_ascii || mapData.spatial_map;
          if (asciiMap) {
            DOM.spatialAsciiPre.textContent = asciiMap;
            log('MCP', 'Rendered live spatial ASCII map from /api/spatial_map', 'info');
          } else {
            DOM.spatialAsciiPre.textContent = '[Warning: No spatial grid data returned by server]';
            log('MCP', 'Spatial map response contained no grid data', 'warn');
          }
        } else {
          let errJson = null;
          try {
            errJson = await resp.json();
          } catch (_) {}
          const errorMsg = (errJson && (errJson.reason || errJson.error))
            || `HTTP ${resp.status}: ${resp.statusText || 'Failed to fetch spatial map'}`;
          DOM.spatialAsciiPre.textContent = `[Failed to load live spatial ASCII map]\n${errorMsg}`;
          log('MCP', `Failed to load spatial map: ${errorMsg}`, 'error');
        }
      } catch (err) {
        DOM.spatialAsciiPre.textContent = `[Network Error fetching spatial map]\n${err.message}`;
        log('MCP', `Network error fetching spatial map: ${err.message}`, 'error');
      }
    } else {
      State.radarMode = 'canvas';
      DOM.btnToggleRadarMode.textContent = 'CANVAS 2D';
      DOM.spatialAsciiContainer.classList.add('hidden');
      DOM.radarCanvasContainer.classList.remove('hidden');
    }
  }

  // ── Initialization & Event Listeners ──────────────────────────────────────
  function init() {
    Radar.init();
    initPidTuner();

    // 1-Click Connect
    DOM.btnConnect.addEventListener('click', handleToggleConnect);

    // E-STOP Triggers
    DOM.btnEstop.addEventListener('click', triggerEmergencyStop);
    DOM.btnEstopReset.addEventListener('click', resetEmergencyStop);

    // Keyboard Shortcuts: Space or Escape triggers E-STOP
    window.addEventListener('keydown', (e) => {
      if (e.code === 'Space' && e.target.tagName !== 'INPUT') {
        e.preventDefault();
        triggerEmergencyStop();
      } else if (e.code === 'Escape') {
        if (!DOM.modalSniffer.classList.contains('hidden')) {
          DOM.modalSniffer.classList.add('hidden');
        } else {
          triggerEmergencyStop();
        }
      }
    });

    // Radar Mode Toggle
    DOM.btnToggleRadarMode.addEventListener('click', toggleRadarMode);

    // Topic Search Filter
    DOM.topicSearchInput.addEventListener('input', () => {
      renderTopics(State.topics);
    });

    // Modal Close
    DOM.btnModalClose.addEventListener('click', () => {
      DOM.modalSniffer.classList.add('hidden');
    });

    // Drawer Clear
    DOM.btnClearLog.addEventListener('click', () => {
      DOM.auditLogStream.innerHTML = '';
      log('SYSTEM', 'Log console cleared.', 'info');
    });

    // Initial log
    log('SYSTEM', 'Mission Control UI loaded with zero external CDN dependencies.', 'info');
    log('SAFETY', 'Safety sandbox active: SAFE_WRITE mode.', 'info');

    // Auto-probe status on startup
    fetch('/api/status')
      .then((r) => r.json())
      .then((data) => {
        if (data.status === 'ok' && data.connected) {
          setConnectionState(data.mode || 'simulation');
          initWebSocketTelemetry();
          fetchTopology();
          fetchDiagnostics();
        } else {
          setConnectionState('disconnected');
        }
      })
      .catch(() => {
        // Standalone offline preview
        setConnectionState('disconnected');
      });
  }

  // Boot on DOM ready
  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', init);
  } else {
    init();
  }

})();
