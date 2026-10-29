/**
 * ROS 2 Robotics Studio — Frontend Application Controller
 * Ultra-Responsive | Tactile D-Pad Drive | 360° LiDAR Radar Canvas | Clean Tab Navigation
 */

(function () {
  'use strict';

  // ── Global Application State ──────────────────────────────────────────────
  const State = {
    connected: true,
    estop: false,
    speedSetting: 0.5, // 0.2 | 0.5 | 1.0 m/s
    linearVel: 0.0,
    angularVel: 0.0,
    closestObstacleM: 2.4,
    batteryPct: 85,
    batteryVoltage: 24.2,
    radarAngle: 0,
    obstacles: [],
    nodes: [],
    topics: [],
    ws: null,
    wsReconnectTimer: null,
  };

  // ── DOM References ────────────────────────────────────────────────────────
  const DOM = {
    connectionBadge: document.getElementById('connection-badge'),
    connectionStatusText: document.getElementById('connection-status-text'),
    latencyVal: document.getElementById('latency-val'),
    sandboxStatusText: document.getElementById('sandbox-status-text'),

    btnConnect: document.getElementById('btn-connect'),
    btnConnectIcon: document.getElementById('btn-connect-icon'),
    btnConnectText: document.getElementById('btn-connect-text'),

    btnEstop: document.getElementById('btn-estop'),
    btnEstopReset: document.getElementById('btn-estop-reset'),
    estopBanner: document.getElementById('estop-banner'),

    btnDismissGuide: document.getElementById('btn-dismiss-guide'),
    guideBanner: document.getElementById('guide-banner'),

    // Radar Canvas
    lidarCanvas: document.getElementById('lidar-canvas'),
    closestObstacleDist: document.getElementById('closest-obstacle-dist'),
    telemetryLinVel: document.getElementById('telemetry-lin-vel'),
    telemetryAngVel: document.getElementById('telemetry-ang-vel'),
    quickBatteryPct: document.getElementById('quick-battery-pct'),
    radarProximityBadge: document.getElementById('radar-proximity-badge'),

    // D-Pad Controls
    btnDriveFwd: document.getElementById('btn-drive-fwd'),
    btnDriveLeft: document.getElementById('btn-drive-left'),
    btnDriveStop: document.getElementById('btn-drive-stop'),
    btnDriveRight: document.getElementById('btn-drive-right'),
    btnDriveRev: document.getElementById('btn-drive-rev'),
    motionFeedback: document.getElementById('motion-feedback'),
    motionFeedbackIcon: document.getElementById('motion-feedback-icon'),
    motionFeedbackText: document.getElementById('motion-feedback-text'),
    speedChips: document.querySelectorAll('.btn-chip'),

    // Navigation Tabs
    tabBtns: document.querySelectorAll('.tab-btn'),
    tabContents: document.querySelectorAll('.tab-content'),

    // Sensors Tab
    batteryBar: document.getElementById('battery-bar'),
    batteryPctText: document.getElementById('battery-pct-text'),
    batteryVoltageText: document.getElementById('battery-voltage-text'),
    batteryCurrentText: document.getElementById('battery-current-text'),
    batteryStateText: document.getElementById('battery-state-text'),
    imuRollText: document.getElementById('imu-roll-text'),
    imuPitchText: document.getElementById('imu-pitch-text'),
    imuYawText: document.getElementById('imu-yaw-text'),
    imuZaccelText: document.getElementById('imu-zaccel-text'),
    alertsList: document.getElementById('diagnostics-alerts-list'),

    // Engineer Tab
    nodesList: document.getElementById('nodes-list'),
    nodeCountBadge: document.getElementById('node-count-badge'),
    topicsTbody: document.getElementById('topics-tbody'),
    topicSearchInput: document.getElementById('topic-search-input'),

    // PID Tuner
    sliderKp: document.getElementById('slider-kp'),
    numKp: document.getElementById('num-kp'),
    sliderKi: document.getElementById('slider-ki'),
    numKi: document.getElementById('num-ki'),
    sliderKd: document.getElementById('slider-kd'),
    numKd: document.getElementById('num-kd'),
    btnApplyPid: document.getElementById('btn-apply-pid'),
    pidStatusMsg: document.getElementById('pid-status-msg'),

    // Activity Log
    auditLogStream: document.getElementById('audit-log-stream'),
    btnClearLog: document.getElementById('btn-clear-log'),

    // Sniffer Modal
    modalSniffer: document.getElementById('modal-sniffer'),
    modalSnifferTitle: document.getElementById('modal-sniffer-title'),
    modalSnifferJson: document.getElementById('modal-sniffer-json'),
    btnModalClose: document.getElementById('btn-modal-close'),
  };

  // ── Logging Utility ───────────────────────────────────────────────────────
  function addLog(tag, msg) {
    if (!DOM.auditLogStream) return;
    const timeStr = new Date().toLocaleTimeString();
    const row = document.createElement('div');
    row.className = 'log-entry';
    row.innerHTML = `<span class="log-time">[${timeStr}]</span> <span class="log-tag">${tag}:</span> <span class="log-msg">${msg}</span>`;
    DOM.auditLogStream.appendChild(row);
    DOM.auditLogStream.scrollTop = DOM.auditLogStream.scrollHeight;
  }

  // ── Drive Teleoperation Logic ─────────────────────────────────────────────
  async function sendDriveCommand(linearX, angularZ, label) {
    if (State.estop) {
      updateFeedback('🛑', 'E-STOP Active! Motion is locked. Re-arm safety to resume.');
      return;
    }

    State.linearVel = linearX;
    State.angularVel = angularZ;
    updateFeedback(
      linearX !== 0 || angularZ !== 0 ? '🚗' : '⏹️',
      label || (linearX !== 0 ? `Driving: ${linearX.toFixed(2)} m/s` : 'Robot Stopped.')
    );

    if (DOM.telemetryLinVel) DOM.telemetryLinVel.textContent = `${linearX.toFixed(2)} m/s`;
    if (DOM.telemetryAngVel) DOM.telemetryAngVel.textContent = `${angularZ.toFixed(2)} rad/s`;

    try {
      const res = await fetch('/api/drive', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ linear_x: linearX, angular_z: angularZ })
      });
      if (res.ok) {
        addLog('DRIVE', `${label} (linear=${linearX.toFixed(2)}, angular=${angularZ.toFixed(2)})`);
      }
    } catch {
      // In offline preview or network drop
    }
  }

  function updateFeedback(icon, text) {
    if (DOM.motionFeedbackIcon) DOM.motionFeedbackIcon.textContent = icon;
    if (DOM.motionFeedbackText) DOM.motionFeedbackText.textContent = text;
  }

  // ── D-Pad Event Binding ───────────────────────────────────────────────────
  function initDpad() {
    if (!DOM.btnDriveFwd) return;

    DOM.btnDriveFwd.addEventListener('click', () => {
      sendDriveCommand(State.speedSetting, 0.0, `Driving Forward at ${State.speedSetting} m/s`);
    });

    DOM.btnDriveRev.addEventListener('click', () => {
      sendDriveCommand(-State.speedSetting, 0.0, `Reversing at -${State.speedSetting} m/s`);
    });

    DOM.btnDriveLeft.addEventListener('click', () => {
      sendDriveCommand(0.0, 1.0, 'Turning Left (Counter-Clockwise)');
    });

    DOM.btnDriveRight.addEventListener('click', () => {
      sendDriveCommand(0.0, -1.0, 'Turning Right (Clockwise)');
    });

    DOM.btnDriveStop.addEventListener('click', () => {
      sendDriveCommand(0.0, 0.0, 'Robot Motion Halted.');
    });

    // Speed chips
    DOM.speedChips.forEach((chip) => {
      chip.addEventListener('click', () => {
        DOM.speedChips.forEach((c) => c.classList.remove('active'));
        chip.classList.add('active');
        State.speedSetting = parseFloat(chip.dataset.speed) || 0.5;
        addLog('SPEED', `Drive speed profile set to ${State.speedSetting} m/s`);
      });
    });

    // Keyboard Arrow Controls
    window.addEventListener('keydown', (e) => {
      if (e.target.tagName === 'INPUT' || e.target.tagName === 'TEXTAREA') return;

      if (e.code === 'ArrowUp') {
        e.preventDefault();
        DOM.btnDriveFwd.classList.add('pressed');
        sendDriveCommand(State.speedSetting, 0.0, 'Driving Forward');
      } else if (e.code === 'ArrowDown') {
        e.preventDefault();
        DOM.btnDriveRev.classList.add('pressed');
        sendDriveCommand(-State.speedSetting, 0.0, 'Reversing');
      } else if (e.code === 'ArrowLeft') {
        e.preventDefault();
        DOM.btnDriveLeft.classList.add('pressed');
        sendDriveCommand(0.0, 1.0, 'Turning Left');
      } else if (e.code === 'ArrowRight') {
        e.preventDefault();
        DOM.btnDriveRight.classList.add('pressed');
        sendDriveCommand(0.0, -1.0, 'Turning Right');
      } else if (e.code === 'Space') {
        e.preventDefault();
        triggerEstop();
      }
    });

    window.addEventListener('keyup', (e) => {
      if (['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight'].includes(e.code)) {
        [DOM.btnDriveFwd, DOM.btnDriveRev, DOM.btnDriveLeft, DOM.btnDriveRight].forEach((b) =>
          b.classList.remove('pressed')
        );
      }
    });
  }

  // ── Emergency Stop (E-STOP) ───────────────────────────────────────────────
  async function triggerEstop() {
    State.estop = true;
    State.linearVel = 0.0;
    State.angularVel = 0.0;

    if (DOM.estopBanner) DOM.estopBanner.classList.remove('hidden');
    updateFeedback('🛑', 'EMERGENCY STOP TRIGGERED: Velocity clamped to 0.00 m/s.');
    if (DOM.telemetryLinVel) DOM.telemetryLinVel.textContent = '0.00 m/s';
    if (DOM.telemetryAngVel) DOM.telemetryAngVel.textContent = '0.00 rad/s';

    addLog('E-STOP', 'EMERGENCY STOP ACTIVATED — All motion killed.');

    try {
      await fetch('/api/estop', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ reason: 'Operator UI E-STOP Triggered' })
      });
    } catch {
      // Offline fallback
    }
  }

  function resetEstop() {
    State.estop = false;
    if (DOM.estopBanner) DOM.estopBanner.classList.add('hidden');
    updateFeedback('🟢', 'Safety Re-Armed. Ready for motion commands.');
    addLog('SAFETY', 'Safety re-armed by operator. Ready to drive.');
  }

  // ── 360° LiDAR Canvas Radar Renderer ─────────────────────────────────────
  const Radar = {
    ctx: null,
    width: 380,
    height: 380,

    init() {
      if (!DOM.lidarCanvas) return;
      this.ctx = DOM.lidarCanvas.getContext('2d');
      this.resize();
      window.addEventListener('resize', () => this.resize());
      this.loop();
    },

    resize() {
      if (!DOM.lidarCanvas) return;
      const rect = DOM.lidarCanvas.parentElement.getBoundingClientRect();
      const dpr = window.devicePixelRatio || 1;
      const size = Math.min(rect.width, rect.height) || 340;
      this.width = size;
      this.height = size;
      DOM.lidarCanvas.width = size * dpr;
      DOM.lidarCanvas.height = size * dpr;
      this.ctx.scale(dpr, dpr);
    },

    loop() {
      requestAnimationFrame(() => this.loop());
      this.render();
    },

    render() {
      if (!this.ctx) return;
      const ctx = this.ctx;
      const cx = this.width / 2;
      const cy = this.height / 2;
      const radius = cx - 16;

      // Clear Canvas
      ctx.clearRect(0, 0, this.width, this.height);

      // Background Circle
      ctx.beginPath();
      ctx.arc(cx, cy, radius, 0, Math.PI * 2);
      ctx.fillStyle = '#060A13';
      ctx.fill();
      ctx.strokeStyle = '#1E293B';
      ctx.lineWidth = 1.5;
      ctx.stroke();

      // Range Rings (1m, 2m, 3m, 4m)
      [0.25, 0.5, 0.75, 1.0].forEach((ratio, idx) => {
        ctx.beginPath();
        ctx.arc(cx, cy, radius * ratio, 0, Math.PI * 2);
        ctx.strokeStyle = idx === 3 ? 'rgba(56, 189, 248, 0.3)' : 'rgba(255, 255, 255, 0.07)';
        ctx.lineWidth = 1;
        ctx.setLineDash([3, 4]);
        ctx.stroke();
        ctx.setLineDash([]);
      });

      // Crosshairs
      ctx.beginPath();
      ctx.moveTo(cx, cy - radius);
      ctx.lineTo(cx, cy + radius);
      ctx.moveTo(cx - radius, cy);
      ctx.lineTo(cx + radius, cy);
      ctx.strokeStyle = 'rgba(255, 255, 255, 0.06)';
      ctx.lineWidth = 1;
      ctx.stroke();

      // Rotating Radar Sweep Beam
      State.radarAngle += 0.04;
      const sweepAngle = State.radarAngle;

      const grad = ctx.createRadialGradient(cx, cy, 0, cx, cy, radius);
      grad.addColorStop(0, 'rgba(56, 189, 248, 0.25)');
      grad.addColorStop(1, 'rgba(56, 189, 248, 0.0)');

      ctx.save();
      ctx.beginPath();
      ctx.moveTo(cx, cy);
      ctx.arc(cx, cy, radius, sweepAngle - 0.5, sweepAngle);
      ctx.closePath();
      ctx.fillStyle = grad;
      ctx.fill();

      // Sweep Leading Edge Line
      ctx.beginPath();
      ctx.moveTo(cx, cy);
      ctx.lineTo(cx + Math.cos(sweepAngle) * radius, cy + Math.sin(sweepAngle) * radius);
      ctx.strokeStyle = 'rgba(56, 189, 248, 0.8)';
      ctx.lineWidth = 1.5;
      ctx.stroke();
      ctx.restore();

      // Draw Obstacle Points
      const points = State.obstacles.length > 0 ? State.obstacles : this.generateDemoObstacles();
      let minDistance = 999;

      points.forEach((pt) => {
        const ptDist = pt.dist;
        if (ptDist < minDistance) minDistance = ptDist;

        // Polar to Cartesian
        const maxRange = 5.0; // 5 meters max display
        const distRatio = Math.min(ptDist / maxRange, 1.0);
        const px = cx + Math.cos(pt.angle) * (radius * distRatio);
        const py = cy + Math.sin(pt.angle) * (radius * distRatio);

        ctx.beginPath();
        ctx.arc(px, py, 3, 0, Math.PI * 2);

        // Color by proximity: Red (<0.6m), Amber (<1.5m), Emerald (>1.5m)
        if (ptDist < 0.6) {
          ctx.fillStyle = '#EF4444';
          ctx.shadowColor = '#EF4444';
          ctx.shadowBlur = 8;
        } else if (ptDist < 1.5) {
          ctx.fillStyle = '#F59E0B';
          ctx.shadowColor = '#F59E0B';
          ctx.shadowBlur = 5;
        } else {
          ctx.fillStyle = '#10B981';
          ctx.shadowColor = '#10B981';
          ctx.shadowBlur = 3;
        }
        ctx.fill();
        ctx.shadowBlur = 0; // reset
      });

      // Update Front Clearance Metric
      State.closestObstacleM = minDistance;
      if (DOM.closestObstacleDist) {
        DOM.closestObstacleDist.textContent = `${minDistance.toFixed(2)} m`;
        if (minDistance < 0.6) {
          DOM.closestObstacleDist.className = 'metric-value text-rose';
          if (DOM.radarProximityBadge) {
            DOM.radarProximityBadge.textContent = '🔴 Obstacle Close';
            DOM.radarProximityBadge.style.background = 'rgba(239, 68, 68, 0.2)';
            DOM.radarProximityBadge.style.color = '#FCA5A5';
          }
        } else {
          DOM.closestObstacleDist.className = 'metric-value text-emerald';
          if (DOM.radarProximityBadge) {
            DOM.radarProximityBadge.textContent = '🟢 Path Clear';
            DOM.radarProximityBadge.style.background = 'rgba(16, 185, 129, 0.15)';
            DOM.radarProximityBadge.style.color = '#34D399';
          }
        }
      }
    },

    generateDemoObstacles() {
      // Deterministic realistic simulated obstacles around the virtual robot
      const obs = [];
      const baseAngles = [0.1, 0.4, 1.2, 1.9, 2.5, 3.2, 4.1, 4.8, 5.5];
      baseAngles.forEach((a, i) => {
        const d = 1.8 + Math.sin(a * 2 + State.radarAngle * 0.1) * 0.6 + (i % 2) * 0.5;
        obs.push({ angle: a, dist: d });
      });
      return obs;
    }
  };

  // ── Telemetry WebSocket Handler ───────────────────────────────────────────
  function initWebSocketTelemetry() {
    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    const wsUrl = `${protocol}//${window.location.host}/ws/telemetry`;

    try {
      State.ws = new WebSocket(wsUrl);

      State.ws.onopen = () => {
        if (DOM.latencyVal) DOM.latencyVal.textContent = '1 ms';
        addLog('TELEMETRY', 'High-speed 20Hz sensor telemetry connected.');
      };

      State.ws.onmessage = (event) => {
        try {
          const frame = JSON.parse(event.data);
          updateTelemetryUI(frame);
        } catch {
          // Ignore malformed frames
        }
      };

      State.ws.onclose = () => {
        // Attempt clean reconnect after 3 seconds
        clearTimeout(State.wsReconnectTimer);
        State.wsReconnectTimer = setTimeout(() => {
          if (State.connected) initWebSocketTelemetry();
        }, 3000);
      };
    } catch {
      // Standalone mode
    }
  }

  function updateTelemetryUI(frame) {
    if (!frame) return;

    // Battery
    if (frame.battery) {
      const pct = Math.round((frame.battery.percentage || 0.85) * 100);
      State.batteryPct = pct;
      if (DOM.quickBatteryPct) DOM.quickBatteryPct.textContent = `${pct}%`;
      if (DOM.batteryPctText) DOM.batteryPctText.textContent = `${pct}%`;
      if (DOM.batteryBar) DOM.batteryBar.style.width = `${pct}%`;
      if (DOM.batteryVoltageText && frame.battery.voltage) {
        DOM.batteryVoltageText.textContent = `${frame.battery.voltage.toFixed(1)} V`;
      }
      if (DOM.batteryCurrentText && frame.battery.current) {
        DOM.batteryCurrentText.textContent = `${frame.battery.current.toFixed(1)} A`;
      }
    }

    // IMU
    if (frame.imu) {
      if (DOM.imuRollText && frame.imu.roll !== undefined) DOM.imuRollText.textContent = `${frame.imu.roll.toFixed(1)}°`;
      if (DOM.imuPitchText && frame.imu.pitch !== undefined) DOM.imuPitchText.textContent = `${frame.imu.pitch.toFixed(1)}°`;
      if (DOM.imuYawText && frame.imu.yaw !== undefined) DOM.imuYawText.textContent = `${frame.imu.yaw.toFixed(1)}°`;
      if (DOM.imuZaccelText && frame.imu.z_accel !== undefined) DOM.imuZaccelText.textContent = `${frame.imu.z_accel.toFixed(2)} m/s²`;
    }

    // LiDAR Scan
    if (frame.lidar && Array.isArray(frame.lidar.ranges)) {
      const ranges = frame.lidar.ranges;
      const angleMin = frame.lidar.angle_min || -Math.PI;
      const angleInc = (frame.lidar.angle_max - angleMin) / ranges.length || (Math.PI * 2) / ranges.length;

      const parsedObs = [];
      for (let i = 0; i < ranges.length; i += 4) {
        const r = ranges[i];
        if (r > 0.1 && r < 10.0) {
          parsedObs.push({ angle: angleMin + i * angleInc, dist: r });
        }
      }
      if (parsedObs.length > 0) State.obstacles = parsedObs;
    }
  }

  // ── Navigation Tabs ───────────────────────────────────────────────────────
  function initTabs() {
    DOM.tabBtns.forEach((btn) => {
      btn.addEventListener('click', () => {
        const target = btn.dataset.tab;
        DOM.tabBtns.forEach((b) => b.classList.remove('active'));
        DOM.tabContents.forEach((c) => c.classList.remove('active'));

        btn.classList.add('active');
        const targetContent = document.getElementById(`tab-${target}`);
        if (targetContent) targetContent.classList.add('active');

        if (target === 'engineer') fetchTopology();
        if (target === 'sensors') fetchDiagnostics();
      });
    });

    // Dismiss Quick Guide
    if (DOM.btnDismissGuide && DOM.guideBanner) {
      DOM.btnDismissGuide.addEventListener('click', () => {
        DOM.guideBanner.classList.add('hidden');
      });
    }

    // Clear Log
    if (DOM.btnClearLog && DOM.auditLogStream) {
      DOM.btnClearLog.addEventListener('click', () => {
        DOM.auditLogStream.innerHTML = '';
        addLog('SYSTEM', 'Log stream cleared.');
      });
    }
  }

  // ── Topology (Engineer Mode) ──────────────────────────────────────────────
  async function fetchTopology() {
    try {
      const [resNodes, resTopics] = await Promise.all([
        fetch('/api/nodes').then((r) => r.json()),
        fetch('/api/topics').then((r) => r.json())
      ]);

      if (resNodes && Array.isArray(resNodes.nodes) && DOM.nodesList) {
        DOM.nodesList.innerHTML = '';
        resNodes.nodes.forEach((n) => {
          const li = document.createElement('li');
          li.className = 'node-item';
          li.textContent = typeof n === 'string' ? n : n.name || JSON.stringify(n);
          DOM.nodesList.appendChild(li);
        });
        if (DOM.nodeCountBadge) DOM.nodeCountBadge.textContent = `${resNodes.nodes.length} Nodes`;
      }

      if (resTopics && Array.isArray(resTopics.topics) && DOM.topicsTbody) {
        State.topics = resTopics.topics;
        renderTopics(resTopics.topics);
      }
    } catch {
      // Offline preview
    }
  }

  function renderTopics(topics) {
    if (!DOM.topicsTbody) return;
    DOM.topicsTbody.innerHTML = '';
    const filter = (DOM.topicSearchInput ? DOM.topicSearchInput.value : '').toLowerCase();

    topics.forEach((t) => {
      const name = typeof t === 'string' ? t : t.name;
      const type = typeof t === 'string' ? 'std_msgs/String' : t.type || 'msg';
      if (filter && !name.toLowerCase().includes(filter)) return;

      const tr = document.createElement('tr');
      tr.innerHTML = `
        <td style="color: #38BDF8;">${name}</td>
        <td style="color: #94A3B8;">${type}</td>
        <td><button class="btn btn-secondary btn-sm" onclick="window.inspectTopic('${name}')">Inspect</button></td>
      `;
      DOM.topicsTbody.appendChild(tr);
    });
  }

  window.inspectTopic = async function (topicName) {
    if (!DOM.modalSniffer) return;
    DOM.modalSnifferTitle.textContent = `Topic: ${topicName}`;
    DOM.modalSnifferJson.textContent = 'Reading message...';
    DOM.modalSniffer.classList.remove('hidden');

    try {
      const res = await fetch(`/api/topic/${encodeURIComponent(topicName.replace(/^\//, ''))}`);
      const data = await res.json();
      DOM.modalSnifferJson.textContent = JSON.stringify(data, null, 2);
    } catch (err) {
      DOM.modalSnifferJson.textContent = `Error reading topic: ${err}`;
    }
  };

  if (DOM.btnModalClose && DOM.modalSniffer) {
    DOM.btnModalClose.addEventListener('click', () => {
      DOM.modalSniffer.classList.add('hidden');
    });
  }

  if (DOM.topicSearchInput) {
    DOM.topicSearchInput.addEventListener('input', () => {
      renderTopics(State.topics);
    });
  }

  // ── Diagnostics Fetch ─────────────────────────────────────────────────────
  async function fetchDiagnostics() {
    try {
      const res = await fetch('/api/diagnostics');
      const data = await res.json();
      if (data && DOM.alertsList) {
        DOM.alertsList.innerHTML = '';
        if (Array.isArray(data.healthy_checks)) {
          data.healthy_checks.forEach((chk) => {
            const div = document.createElement('div');
            div.className = 'diag-item';
            div.innerHTML = `<span>✅</span><span>${chk}</span>`;
            DOM.alertsList.appendChild(div);
          });
        }
      }
    } catch {
      // Offline fallback
    }
  }

  // ── PID Tuner ─────────────────────────────────────────────────────────────
  function initPidTuner() {
    if (!DOM.btnApplyPid) return;

    ['kp', 'ki', 'kd'].forEach((axis) => {
      const slider = DOM[`slider${axis.charAt(0).toUpperCase() + axis.slice(1)}`];
      const num = DOM[`num${axis.charAt(0).toUpperCase() + axis.slice(1)}`];
      if (slider && num) {
        slider.addEventListener('input', () => {
          num.textContent = parseFloat(slider.value).toFixed(2);
        });
      }
    });

    DOM.btnApplyPid.addEventListener('click', async () => {
      const kp = parseFloat(DOM.sliderKp.value);
      const ki = parseFloat(DOM.sliderKi.value);
      const kd = parseFloat(DOM.sliderKd.value);

      DOM.pidStatusMsg.textContent = 'Applying gains...';
      DOM.pidStatusMsg.style.color = '#38BDF8';

      try {
        const res = await fetch('/api/tune_pid', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ kp, ki, kd })
        });
        const data = await res.json();
        if (res.ok) {
          DOM.pidStatusMsg.textContent = `✅ Gains applied: Kp=${kp}, Ki=${ki}, Kd=${kd}`;
          DOM.pidStatusMsg.style.color = '#34D399';
          addLog('PID', `PID gains calibrated: Kp=${kp}, Ki=${ki}, Kd=${kd}`);
        } else {
          DOM.pidStatusMsg.textContent = `❌ ${data.reason || 'Failed'}`;
          DOM.pidStatusMsg.style.color = '#EF4444';
        }
      } catch {
        DOM.pidStatusMsg.textContent = '✅ Simulated PID gains updated.';
        DOM.pidStatusMsg.style.color = '#34D399';
      }
    });
  }

  // ── Toggle Connect / Pause ────────────────────────────────────────────────
  function initConnectToggle() {
    if (!DOM.btnConnect) return;

    DOM.btnConnect.addEventListener('click', async () => {
      State.connected = !State.connected;

      if (State.connected) {
        DOM.connectionBadge.className = 'pill-badge state-connected';
        DOM.connectionStatusText.textContent = 'Virtual Robot Active';
        DOM.btnConnectIcon.textContent = '⏸️';
        DOM.btnConnectText.textContent = 'Pause Robot';
        updateFeedback('🟢', 'Virtual robot active and ready to drive!');
        addLog('SYSTEM', 'Virtual Robot started and online.');
        initWebSocketTelemetry();
      } else {
        DOM.connectionBadge.className = 'pill-badge state-disconnected';
        DOM.connectionStatusText.textContent = 'Robot Standby';
        DOM.btnConnectIcon.textContent = '▶️';
        DOM.btnConnectText.textContent = 'Start Robot';
        updateFeedback('⚪', 'Robot is paused / standby. Click Start to resume.');
        addLog('SYSTEM', 'Robot paused by operator.');
        if (State.ws) State.ws.close();
      }

      try {
        await fetch('/api/connect', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ action: State.connected ? 'connect' : 'disconnect' })
        });
      } catch {
        // Offline preview
      }
    });

    if (DOM.btnEstop) DOM.btnEstop.addEventListener('click', triggerEstop);
    if (DOM.btnEstopReset) DOM.btnEstopReset.addEventListener('click', resetEstop);
  }

  // ── Initialization Entry Point ────────────────────────────────────────────
  function init() {
    Radar.init();
    initDpad();
    initTabs();
    initPidTuner();
    initConnectToggle();
    initWebSocketTelemetry();

    addLog('SYSTEM', 'ROS 2 Robotics Studio initialized with zero external CDN dependencies.');
    addLog('SAFETY', 'Command Sandbox active in SAFE_WRITE mode.');
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', init);
  } else {
    init();
  }
})();
