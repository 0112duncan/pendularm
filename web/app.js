const $ = selector => document.querySelector(selector);
const $$ = selector => [...document.querySelectorAll(selector)];
const canvas = $('#arm');
const ctx = canvas.getContext('2d');
let socket;
let requestNumber = 0;
let pending = new Map();
let lengths = [1, 1];
let masses = [1, 1];
let gains = { kp: [], ki: [], kd: [] };
let latestState = null;
let simulationPaused = false;
let activeGoalId = null;
let activeGoalTarget = null;

function log(text) {
  const box = $('#messages');
  box.textContent = `${new Date().toLocaleTimeString()}  ${text}\n${box.textContent}`.slice(0, 5000);
}
function report(error) { log(error instanceof Error ? error.message : String(error)); }
function optionalNumber(selector) {
  const raw = $(selector).value.trim();
  return raw === '' ? undefined : Number(raw);
}
function numberValues(selector) { return $$(selector).map(input => Number(input.value)); }
function call(service, args = {}) {
  return new Promise((resolve, reject) => {
    if (!socket || socket.readyState !== WebSocket.OPEN) return reject(Error('Web bridge is disconnected'));
    const id = `web-${++requestNumber}`;
    const timer = setTimeout(() => {
      pending.delete(id);
      reject(Error(`${service} timed out`));
    }, 4500);
    pending.set(id, { service, resolve, reject, timer });
    socket.send(JSON.stringify({ op: 'call_service', service, id, args }));
  });
}
function mountInputs(count) {
  lengths = Array.from({ length: count }, (_, i) => lengths[i] ?? 1);
  masses = Array.from({ length: count }, (_, i) => masses[i] ?? 1);
  $('#param-inputs').innerHTML = Array.from({ length: count }, (_, i) => `
    <div class="link-row"><span class="joint-label">L${i + 1}</span>
      <label>Mass<input class="mass" type="number" min="0.001" step="0.1" value="${masses[i]}"></label>
      <label>Length<input class="length" type="number" min="0.001" step="0.1" value="${lengths[i]}"></label>
    </div>`).join('');
  const gainValue = (key, index) => Number.isFinite(gains[key][index]) ? gains[key][index] : '';
  $('#gain-inputs').innerHTML = Array.from({ length: count }, (_, i) => `
    <div class="gain-row"><span class="joint-label">J${i + 1}</span>
      <label>Kp<input class="kp" type="number" min="0" step="0.1" value="${gainValue('kp', i)}"></label>
      <label>Ki<input class="ki" type="number" min="0" step="0.1" value="${gainValue('ki', i)}"></label>
      <label>Kd<input class="kd" type="number" min="0" step="0.1" value="${gainValue('kd', i)}"></label>
    </div>`).join('');
}
function applyParams(values) {
  if (!values) return;
  const count = Array.isArray(values.lengths) ? values.lengths.length : lengths.length;
  if (count !== lengths.length) mountInputs(count);
  if (Array.isArray(values.lengths)) lengths = values.lengths;
  if (Array.isArray(values.masses)) masses = values.masses;
  if (typeof values.gravity === 'number') $('#gravity').value = values.gravity;
  $$('.length').forEach((input, i) => input.value = lengths[i]);
  $$('.mass').forEach((input, i) => input.value = masses[i]);
  if (latestState) renderArm(latestState.position);
}
function applyGains(values) {
  const keys = ['kp', 'ki', 'kd'];
  if (!keys.every(key => Array.isArray(values?.[key]))) return;

  gains = {
    kp: [...values.kp],
    ki: [...values.ki],
    kd: [...values.kd]
  };
  const count = gains.kp.length;
  if (count !== lengths.length) mountInputs(count);
  $$('.kp').forEach((input, i) => input.value = gains.kp[i]);
  $$('.ki').forEach((input, i) => input.value = gains.ki[i]);
  $$('.kd').forEach((input, i) => input.value = gains.kd[i]);
  $('#send-gains').disabled = false;
}
function readGoalTarget(value) {
  const x = Number(value?.x);
  const y = Number(value?.y);
  const phi = Number(value?.phi);
  if (!Number.isFinite(x) || !Number.isFinite(y)) return null;
  return { x, y, phi: Number.isFinite(phi) ? phi : null };
}
function drawGoalTarget(baseX, baseY, scale) {
  if (!activeGoalTarget) return;

  const x = baseX + activeGoalTarget.x * scale;
  const y = baseY - activeGoalTarget.y * scale;

  ctx.save();
  ctx.shadowColor = '#ffb45f';
  ctx.shadowBlur = 18;
  ctx.fillStyle = '#ffb45f';
  ctx.beginPath();
  ctx.arc(x, y, 10, 0, Math.PI * 2);
  ctx.fill();
  ctx.shadowBlur = 0;
  ctx.strokeStyle = '#fff4df';
  ctx.lineWidth = 3;
  ctx.beginPath();
  ctx.arc(x, y, 18, 0, Math.PI * 2);
  ctx.stroke();

  if (activeGoalTarget.phi !== null) {
    const arrowLength = 58;
    const tipX = x + Math.cos(activeGoalTarget.phi) * arrowLength;
    const tipY = y - Math.sin(activeGoalTarget.phi) * arrowLength;
    ctx.strokeStyle = '#ffb45f';
    ctx.fillStyle = '#ffb45f';
    ctx.lineWidth = 5;
    ctx.beginPath();
    ctx.moveTo(x, y);
    ctx.lineTo(tipX, tipY);
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(tipX, tipY);
    ctx.lineTo(tipX - Math.cos(activeGoalTarget.phi - 0.55) * 16,
      tipY + Math.sin(activeGoalTarget.phi - 0.55) * 16);
    ctx.lineTo(tipX - Math.cos(activeGoalTarget.phi + 0.55) * 16,
      tipY + Math.sin(activeGoalTarget.phi + 0.55) * 16);
    ctx.closePath();
    ctx.fill();
  }

  ctx.fillStyle = '#fff4df';
  ctx.font = 'bold 17px ui-monospace, monospace';
  ctx.fillText('GOAL', x + 25, y - 12);
  ctx.restore();
}
function renderArm(angles) {
  const { width, height } = canvas;
  ctx.fillStyle = '#07110e';
  ctx.fillRect(0, 0, width, height);
  ctx.strokeStyle = '#133027';
  ctx.lineWidth = 1;
  for (let x = 30; x < width; x += 55) { ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, height); ctx.stroke(); }
  for (let y = 30; y < height; y += 55) { ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(width, y); ctx.stroke(); }

  const totalLength = lengths.reduce((sum, value) => sum + value, 0);
  const scale = Math.min(width, height) / (2.65 * Math.max(1, totalLength));
  const baseX = width / 2;
  const baseY = height / 2;
  let x = baseX;
  let y = baseY;
  let orientation = 0;
  ctx.fillStyle = '#e7f0ec';
  ctx.beginPath(); ctx.arc(x, y, 13, 0, Math.PI * 2); ctx.fill();
  ctx.fillStyle = '#9cbab0';
  ctx.font = '20px ui-monospace, monospace';
  ctx.fillText('BASE', x + 20, y + 35);

  angles.forEach((rawAngle, index) => {
    const angle = Number(rawAngle);
    if (!Number.isFinite(angle)) return;
    orientation += angle;
    const nx = x + Math.cos(orientation) * (lengths[index] || 1) * scale;
    const ny = y - Math.sin(orientation) * (lengths[index] || 1) * scale;
    ctx.strokeStyle = index % 2 ? '#6ae2cb' : '#b9f26e';
    ctx.lineWidth = 24;
    ctx.lineCap = 'round';
    ctx.beginPath(); ctx.moveTo(x, y); ctx.lineTo(nx, ny); ctx.stroke();
    ctx.fillStyle = '#f0f8f5';
    ctx.beginPath(); ctx.arc(nx, ny, 12, 0, Math.PI * 2); ctx.fill();
    ctx.fillStyle = '#07110e';
    ctx.font = 'bold 17px ui-monospace, monospace';
    ctx.fillText(String(index + 1), nx - 5, ny + 6);
    x = nx; y = ny;
  });
  drawGoalTarget(baseX, baseY, scale);
}
function updateState(data) {
  if (!Array.isArray(data?.position)) return;
  if (data.position.length !== lengths.length) mountInputs(data.position.length);
  latestState = data;
  const stamp = data.header?.stamp || {};
  const seconds = Number(stamp.sec || 0) + Number(stamp.nanosec || 0) / 1e9;
  $('#sim-time').textContent = `${seconds.toFixed(3)} s`;
  $('#link-count').textContent = `${data.position.length} links`;
  $('#joint-state').innerHTML = data.position.map((position, i) => `
    <tr><td>${data.name?.[i] || `joint${i + 1}`}</td>
      <td>${Number(position).toFixed(4)}</td>
      <td>${Number(data.velocity?.[i] || 0).toFixed(4)}</td>
      <td>${Number(data.effort?.[i] || 0).toFixed(4)}</td></tr>`).join('');
  renderArm(data.position);
}
function formatPidValue(value) {
  const number = Number(value);
  if (!Number.isFinite(number)) return '—';
  const magnitude = Math.abs(number);
  if (magnitude >= 10000 || (magnitude > 0 && magnitude < 0.001)) return number.toExponential(2);
  return number.toFixed(4);
}
function updatePidTelemetry(data) {
  const count = Array.isArray(data?.position_error) ? data.position_error.length : 0;
  $('#pid-state').textContent = data?.enabled ? 'PID running' : 'PID stopped';
  if (!count) {
    $('#pid-live').textContent = 'Waiting for /pid_controller/state';
    return;
  }
  $('#pid-live').innerHTML = Array.from({ length: count }, (_, i) => `
    <div class="pid-live-row">
      <div class="pid-live-head"><strong>J${i + 1}</strong><span>angle error ${formatPidValue(data.position_error?.[i])} rad · output ${formatPidValue(data.output?.[i])}</span></div>
      <div class="pid-live-metrics">
        <span><small>VEL ERR</small>${formatPidValue(data.velocity_error?.[i])}</span>
        <span><small>P</small>${formatPidValue(data.p_term?.[i])}</span>
        <span><small>I</small>${formatPidValue(data.i_term?.[i])}</span>
        <span><small>D</small>${formatPidValue(data.d_term?.[i])}</span>
      </div>
    </div>`).join('');
}
function updateTrial(trial) {
  const state = trial.running ? 'Running' : 'Stopped';
  const elapsed = Number(trial.elapsed || 0).toFixed(1);
  const duration = Number(trial.duration || 0).toFixed(1);
  const reached = Number(trial.targets_reached || 0);
  $('#trial-status').textContent = `${state} · ${elapsed}/${duration}s · ${reached} reached · ${trial.action_status || 'idle'}`;
}
function handle(message) {
  if (message.op === 'bridge_status') {
    $('#connection').textContent = message.connected ? 'Node runtime connected' : (message.message || 'Runtime disconnected');
    $('#connection').className = `connection ${message.connected ? 'on' : 'off'}`;
    return;
  }
  if (message.op === 'bridge_error') return log(message.message);
  if (message.op === 'service_response' && pending.has(message.id)) {
    const job = pending.get(message.id);
    pending.delete(message.id);
    clearTimeout(job.timer);
    if (message.result) {
      log(`${job.service}: success`);
      job.resolve(message.values || {});
    } else {
      const error = message.status || 'rejected';
      log(`${job.service}: ${error}`);
      job.reject(Error(error));
    }
    return;
  }
  if (message.op !== 'publish') return;
  if (message.topic === '/joint_states') return updateState(message.msg);
  if (message.topic === '/arm_sim/energy') {
    const energy = message.msg?.data;
    $('#energy').textContent = Number.isFinite(energy) ? `${Number(energy).toFixed(5)} J` : '—';
    return;
  }
  if (message.topic === '/pid_controller/state') return updatePidTelemetry(message.msg || {});
  if (message.topic === "/ik_action/feedback") {
    activeGoalId = message.msg?.goal_id || activeGoalId;
    activeGoalTarget = readGoalTarget(message.msg?.target) || activeGoalTarget;
    const distance = Number(message.msg?.distance_remaining);
    $("#goal-status").textContent = Number.isFinite(distance) ? "Active · distance " + distance.toFixed(4) : "Goal active";
    if (latestState) renderArm(latestState.position);
    return;
  }
  if (message.topic === "/ik_action/result") {
    const finishedGoalId = message.msg?.goal_id;
    if (finishedGoalId && activeGoalId && finishedGoalId !== activeGoalId) return;
    activeGoalId = null;
    activeGoalTarget = null;
    if (latestState) renderArm(latestState.position);
    const outcome = message.msg?.outcome || "finished";
    const distance = Number(message.msg?.final_distance);
    $("#goal-status").textContent = Number.isFinite(distance) ? outcome + " · final distance " + distance.toFixed(4) : outcome;
    return;
  }
  if (message.topic === '/ik_trial/status') updateTrial(message.msg || {});
}
function loadParams() {
  call('/arm_sim/set_params', {}).then(applyParams).catch(() => setTimeout(loadParams, 600));
}
function loadGains() {
  call('/pid_controller/set_gains', {}).then(applyGains).catch(() => setTimeout(loadGains, 600));
}
function subscribe(topic, type) {
  socket.send(JSON.stringify({ op: 'subscribe', topic, type }));
}
function connect() {
  socket = new WebSocket(`ws://${location.host}/ws`);
  socket.onopen = () => {
    subscribe('/joint_states', 'sensor_msgs/JointState');
    subscribe('/arm_sim/energy', 'std_msgs/Float64');
    subscribe('/pid_controller/state', 'pid_controller/State');
    loadGains();
    subscribe("/ik_action/feedback", "ik_action/feedback");
    subscribe("/ik_action/result", "ik_action/result");
    subscribe('/ik_trial/status', 'ik_trial/status');
    loadParams();
  };
  socket.onmessage = event => {
    try { handle(JSON.parse(event.data)); }
    catch { log('Malformed bridge response'); }
  };
  socket.onclose = () => {
    $('#connection').textContent = 'Reconnecting…';
    $('#connection').className = 'connection off';
    setTimeout(connect, 1200);
  };
}

$("#send-integrator").onclick = () => call("/arm_sim/set_integrator", {
  method: $("#integrator-method").value,
  timestep: Number($("#integrator-timestep").value)
}).then(() => log("Integrator: " + $("#integrator-method").value + ", dt=" + $("#integrator-timestep").value + "s")).catch(report);
$('#send-params').onclick = () => call('/arm_sim/set_params', {
  gravity: Number($('#gravity').value),
  masses: numberValues('.mass'),
  lengths: numberValues('.length')
}).then(applyParams).catch(report);
$('#toggle-sim').onclick = () => {
  const requested = !simulationPaused;
  call('/arm_sim/pause', { data: requested }).then(values => {
    simulationPaused = Boolean(values.data);
    $('#toggle-sim').textContent = simulationPaused ? 'Continue simulation' : 'Pause simulation';
  }).catch(report);
};
$("#reset-sim").onclick = () => call("/arm_sim/reset", {}).then(() => log("Arm reset to home pose")).catch(report);
$('#send-goal').onclick = () => {
  const args = { x: Number($('#goal-x').value), y: Number($('#goal-y').value) };
  for (const [key, selector] of [['phi', '#goal-phi'], ['epsilon', '#goal-epsilon'], ['success_hold', '#goal-hold']]) {
    const value = optionalNumber(selector);
    if (value !== undefined) args[key] = value;
  }
  call('/ik_action/send_goal', args).then(values => {
    if (values.goal_id) {
      activeGoalId = values.goal_id;
      activeGoalTarget = readGoalTarget(args);
      $("#goal-status").textContent = "Active · " + values.goal_id;
      log("Goal active: " + values.goal_id);
      if (latestState) renderArm(latestState.position);
    }
  }).catch(report);
};
$("#cancel-goal").onclick = () => call("/ik_action/cancel_goal", activeGoalId ? { goal_id: activeGoalId } : {})
  .then(() => {
    activeGoalId = null;
    activeGoalTarget = null;
    $("#goal-status").textContent = "Goal cancelled";
    if (latestState) renderArm(latestState.position);
  })
  .catch(report);
$('#send-gains').onclick = () => call('/pid_controller/set_gains', {
  kp: numberValues('.kp'), ki: numberValues('.ki'), kd: numberValues('.kd')
}).then(applyGains).catch(report);
$('#start-pid').onclick = () => call('/pid_controller/enable', { data: true }).then(() => {
  $('#pid-state').textContent = 'PID running';
}).catch(report);
$('#stop-pid').onclick = () => call('/pid_controller/enable', { data: false }).then(() => {
  $('#pid-state').textContent = 'PID stopped';
}).catch(report);
$('#trial-start').onclick = () => {
  const args = {};
  for (const [key, selector] of [['duration', '#trial-duration'], ['epsilon', '#trial-epsilon'], ['success_hold', '#trial-hold']]) {
    const value = optionalNumber(selector);
    if (value !== undefined) args[key] = value;
  }
  call('/ik_trial/start', args).catch(report);
};
$('#trial-skip').onclick = () => call('/ik_trial/skip', {}).catch(report);
$('#trial-stop').onclick = () => call('/ik_trial/stop', {}).catch(report);

mountInputs(2);
connect();
