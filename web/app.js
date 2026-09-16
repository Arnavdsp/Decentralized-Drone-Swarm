import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";

// --------------------------------------------------------------------- setup
const holder = document.getElementById("canvas-holder");
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0b0d13);

const camera = new THREE.PerspectiveCamera(55, holder.clientWidth / holder.clientHeight, 0.05, 500);
camera.position.set(10, 8, 14);

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(window.devicePixelRatio);
renderer.setSize(holder.clientWidth, holder.clientHeight);
holder.appendChild(renderer.domElement);

const controls = new OrbitControls(camera, renderer.domElement);
controls.target.set(0, 2, 0);
controls.enableDamping = true;

scene.add(new THREE.HemisphereLight(0x8fa8ff, 0x0b0d13, 1.1));
const sun = new THREE.DirectionalLight(0xffffff, 0.6);
sun.position.set(8, 20, 6);
scene.add(sun);

const grid = new THREE.GridHelper(40, 40, 0x2a2d3a, 0x1a1d27);
scene.add(grid);
const axes = new THREE.AxesHelper(1.5);
scene.add(axes);

window.addEventListener("resize", () => {
  camera.aspect = holder.clientWidth / holder.clientHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(holder.clientWidth, holder.clientHeight);
});

// ------------------------------------------------------------ scene contents
const COLOR_HEALTHY = 0x4c9be8;
const COLOR_FAILED = 0xe8544c;
const COLOR_SELECTED = 0x4ce87a;

let droneMeshes = [];       // THREE.Mesh (cone), one per drone
let droneLabels = [];       // small sprite-less markers not used; index badges via DOM list instead
let tetherLines = [];       // THREE.Line, one per drone
let commLines = [];         // THREE.Line, one per ring edge
let targetMarkers = [];     // THREE.Mesh (ring), one per drone
let velArrows = [];
let accArrows = [];
let droneTrails = [];       // { points: [], line }
let payloadTrail = { points: [], line: null };
let payloadMesh = null;
let centerMesh = null;
let commLinks = [];         // [[i,j], ...] from meta
let nDrones = 6;

function disposeObject(obj) {
  if (!obj) return;
  scene.remove(obj);
  if (obj.geometry) obj.geometry.dispose();
  if (obj.material) obj.material.dispose();
}

function clearSceneObjects() {
  droneMeshes.forEach(disposeObject);
  tetherLines.forEach(disposeObject);
  commLines.forEach(disposeObject);
  targetMarkers.forEach(disposeObject);
  velArrows.forEach((a) => scene.remove(a));
  accArrows.forEach((a) => scene.remove(a));
  droneTrails.forEach((t) => disposeObject(t.line));
  disposeObject(payloadTrail.line);
  disposeObject(payloadMesh);
  disposeObject(centerMesh);
  droneMeshes = []; tetherLines = []; commLines = []; targetMarkers = [];
  velArrows = []; accArrows = []; droneTrails = [];
  payloadTrail = { points: [], line: null };
}

function buildSceneForSwarm(n, links) {
  clearSceneObjects();
  nDrones = n;
  commLinks = links;

  const coneGeo = new THREE.ConeGeometry(0.18, 0.5, 10);
  for (let i = 0; i < n; i++) {
    const mat = new THREE.MeshStandardMaterial({ color: COLOR_HEALTHY, emissive: 0x0a1a2a });
    const mesh = new THREE.Mesh(coneGeo, mat);
    scene.add(mesh);
    droneMeshes.push(mesh);

    const tetherMat = new THREE.LineBasicMaterial({ color: 0x666b80 });
    const tetherGeo = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]);
    const tether = new THREE.Line(tetherGeo, tetherMat);
    scene.add(tether);
    tetherLines.push(tether);

    const ringMat = new THREE.MeshBasicMaterial({ color: 0xe8c44c, side: THREE.DoubleSide, transparent: true, opacity: 0.6 });
    const ring = new THREE.Mesh(new THREE.RingGeometry(0.18, 0.24, 20), ringMat);
    ring.rotation.x = -Math.PI / 2;
    scene.add(ring);
    targetMarkers.push(ring);

    velArrows.push(new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(), 1, 0x4ce87a, 0.15, 0.08));
    accArrows.push(new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(), 1, 0xe8734c, 0.15, 0.08));
    velArrows[i].visible = false; accArrows[i].visible = false;
    scene.add(velArrows[i]); scene.add(accArrows[i]);

    const trailMat = new THREE.LineBasicMaterial({ color: 0x4c9be8, transparent: true, opacity: 0.5 });
    const trail = new THREE.Line(new THREE.BufferGeometry(), trailMat);
    trail.visible = false;
    scene.add(trail);
    droneTrails.push({ points: [], line: trail });
  }

  commLines = links.map(() => {
    const geo = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]);
    const mat = new THREE.LineBasicMaterial({ color: 0x3d4a66, transparent: true, opacity: 0.7 });
    const line = new THREE.Line(geo, mat);
    scene.add(line);
    return line;
  });

  payloadMesh = new THREE.Mesh(
    new THREE.IcosahedronGeometry(0.32, 0),
    new THREE.MeshStandardMaterial({ color: 0xe8c44c, emissive: 0x332a05 })
  );
  scene.add(payloadMesh);

  const payloadTrailMat = new THREE.LineBasicMaterial({ color: 0xe8c44c, transparent: true, opacity: 0.6 });
  payloadTrail.line = new THREE.Line(new THREE.BufferGeometry(), payloadTrailMat);
  payloadTrail.line.visible = false;
  scene.add(payloadTrail.line);

  centerMesh = new THREE.Mesh(
    new THREE.SphereGeometry(0.1, 12, 12),
    new THREE.MeshBasicMaterial({ color: 0xffffff })
  );
  scene.add(centerMesh);
}

// ---------------------------------------------------------------- animation
function animate() {
  requestAnimationFrame(animate);
  controls.update();
  renderer.render(scene, camera);
}
animate();

// -------------------------------------------------------------- overlay refs
const ov = {
  comm: document.getElementById("ov-comm"),
  tethers: document.getElementById("ov-tethers"),
  targets: document.getElementById("ov-targets"),
  center: document.getElementById("ov-center"),
  accel: document.getElementById("ov-accel"),
  vel: document.getElementById("ov-vel"),
  trailDrone: document.getElementById("ov-trail-drone"),
  trailLoad: document.getElementById("ov-trail-load"),
};

// ----------------------------------------------------------- frame rendering
let selectedDrone = 0;
const MAX_TRAIL = 1500;

function applyFrame(frame) {
  document.getElementById("phase-value").textContent = frame.phase;
  document.getElementById("t-value").textContent = `t = ${frame.t.toFixed(1)}s`;

  document.getElementById("m-quality").textContent = frame.shape_quality.toFixed(3);
  document.getElementById("m-rmse").textContent = `${frame.formation_rmse_m.toFixed(2)} m`;
  document.getElementById("m-minsep").textContent = `${frame.min_separation_m.toFixed(2)} m`;
  document.getElementById("m-altsp").textContent = `${frame.alt_setpoint.toFixed(2)} m`;
  document.getElementById("m-loadmass").textContent = `${frame.load_mass_carried_kg.toFixed(2)} kg`;
  document.getElementById("m-wind").textContent =
    `[${frame.wind.map((v) => v.toFixed(2)).join(", ")}]`;
  document.getElementById("m-abort").textContent = frame.abort_reason || "–";

  if (frame.event) {
    const b = document.getElementById("event-banner");
    b.textContent = `⚡ ${frame.event.replaceAll("_", " ")}`;
    b.style.display = "block";
    clearTimeout(applyFrame._t);
    applyFrame._t = setTimeout(() => (b.style.display = "none"), 2500);
  }

  const targets = frame.drones.map((d) => d.target);
  const centre = targets.reduce((acc, t) => [acc[0] + t[0] / targets.length, acc[1] + t[1] / targets.length, acc[2] + t[2] / targets.length], [0, 0, 0]);
  if (centerMesh) {
    centerMesh.visible = ov.center.checked;
    centerMesh.position.set(centre[0], centre[2], -centre[1]);
  }

  frame.drones.forEach((d, i) => {
    const mesh = droneMeshes[i];
    if (!mesh) return;
    const [x, y, z] = d.position; // world (x east, y north, z up) -> three (x, z_up->y, -north->z)
    const pos = new THREE.Vector3(x, z, -y);
    mesh.position.copy(pos);
    mesh.material.color.set(i === selectedDrone ? COLOR_SELECTED : (d.healthy ? COLOR_HEALTHY : COLOR_FAILED));
    mesh.material.emissive.set(d.healthy ? 0x0a1a2a : 0x330806);

    const vel = new THREE.Vector3(d.velocity[0], d.velocity[2], -d.velocity[1]);
    const acc = new THREE.Vector3(d.accel_cmd[0], d.accel_cmd[2], -d.accel_cmd[1]);
    if (vel.length() > 1e-3) mesh.lookAt(pos.clone().add(vel));

    // tether: drone -> payload attach point (load_position + relative attach offset)
    const attach = window.__attachPoints ? window.__attachPoints[i] : [0, 0, 0];
    const load = frame.load_position;
    const attachWorld = new THREE.Vector3(load[0] + attach[0], load[2] + attach[2], -(load[1] + attach[1]));
    const tether = tetherLines[i];
    tether.visible = ov.tethers.checked && frame.load_mass_carried_kg > 1e-6;
    tether.geometry.setFromPoints([pos, attachWorld]);

    const tMarker = targetMarkers[i];
    tMarker.visible = ov.targets.checked;
    tMarker.position.set(d.target[0], d.target[2], -d.target[1]);

    const vArrow = velArrows[i];
    vArrow.visible = ov.vel.checked;
    if (vel.length() > 1e-3) { vArrow.position.copy(pos); vArrow.setDirection(vel.clone().normalize()); vArrow.setLength(Math.min(vel.length(), 3), 0.15, 0.08); }

    const aArrow = accArrows[i];
    aArrow.visible = ov.accel.checked;
    if (acc.length() > 1e-3) { aArrow.position.copy(pos); aArrow.setDirection(acc.clone().normalize()); aArrow.setLength(Math.min(acc.length(), 3), 0.15, 0.08); }

    const trail = droneTrails[i];
    trail.line.visible = ov.trailDrone.checked;
    trail.points.push(pos.clone());
    if (trail.points.length > MAX_TRAIL) trail.points.shift();
    if (ov.trailDrone.checked) trail.line.geometry.setFromPoints(trail.points);
  });

  commLines.forEach((line, k) => {
    const [i, j] = commLinks[k];
    line.visible = ov.comm.checked;
    if (!ov.comm.checked) return;
    const a = frame.drones[i].position, b = frame.drones[j].position;
    line.geometry.setFromPoints([
      new THREE.Vector3(a[0], a[2], -a[1]),
      new THREE.Vector3(b[0], b[2], -b[1]),
    ]);
  });

  if (payloadMesh) {
    const [lx, ly, lz] = frame.load_position;
    payloadMesh.position.set(lx, lz, -ly);
    payloadMesh.visible = frame.load_mass_carried_kg > 1e-6 || lz > 0.01;
    payloadTrail.line.visible = ov.trailLoad.checked;
    payloadTrail.points.push(new THREE.Vector3(lx, lz, -ly));
    if (payloadTrail.points.length > MAX_TRAIL) payloadTrail.points.shift();
    if (ov.trailLoad.checked) payloadTrail.line.geometry.setFromPoints(payloadTrail.points);
  }

  renderDroneList(frame);
}

function renderDroneList(frame) {
  const list = document.getElementById("drone-list");
  list.innerHTML = "";
  frame.drones.forEach((d, i) => {
    const card = document.createElement("div");
    card.className = "drone-card" + (i === selectedDrone ? " selected" : "") + (d.healthy ? "" : " unhealthy");
    card.innerHTML = `
      <div class="dtitle"><span><span class="dot ${d.healthy ? "ok" : "bad"}"></span>Drone ${d.id} (slot ${d.slot})</span>
      <span>${d.battery_frac >= 0 ? (d.battery_frac * 100).toFixed(0) + "%" : ""}</span></div>
      <div class="metric-row"><span>Formation error</span><span>${d.formation_error_m.toFixed(2)} m</span></div>
      <div class="metric-row"><span>Load share</span><span>${d.load_share_n.toFixed(1)} N</span></div>
      <div class="metric-row"><span>Tether tension</span><span>${d.tether_tension_n.toFixed(1)} N</span></div>
      <div class="metric-row"><span>Velocity</span><span>${d.velocity.map((v) => v.toFixed(1)).join(",")}</span></div>
      <div class="metric-row"><span>Accel cmd</span><span>${d.accel_cmd.map((v) => v.toFixed(1)).join(",")}</span></div>
      ${d.fault ? `<div class="metric-row"><span>Fault</span><span>${d.fault}</span></div>` : ""}
    `;
    card.addEventListener("click", () => { selectedDrone = i; });
    list.appendChild(card);
  });
}

// -------------------------------------------------------------- scenario UI
const SCENARIO_FIELDS = {
  A: [],
  B: [
    { id: "b-speed", label: "Wind speed (m/s^2)", param: "wind_speed", type: "number", value: 0.8, step: 0.1 },
    { id: "b-dir", label: "Wind direction (deg)", param: "wind_dir_deg", type: "number", value: 0, step: 5 },
    { id: "b-start", label: "Wind start (s)", param: "wind_start_s", type: "number", value: 0, step: 1 },
  ],
  C: [
    { id: "c-drone", label: "Drone to fail (0-5)", param: "fail_drone", type: "number", value: 3, step: 1 },
    { id: "c-at", label: "Failure time (s)", param: "fail_at_s", type: "number", value: 20, step: 1 },
  ],
  D: [],
  E: [
    { id: "e-drone", label: "Drone to push (0-5)", param: "push_drone", type: "number", value: 0, step: 1 },
    { id: "e-at", label: "Push time (s)", param: "push_at_s", type: "number", value: 5, step: 1 },
    { id: "e-off", label: "Push distance (m)", param: "push_dist", type: "number", value: 1.5, step: 0.5 },
  ],
};

const SCENARIO_DESC = {
  A: "Full mission: arm, take off, form hexagon, descend, tension tethers, lift, cruise, lower, release, land.",
  B: "Steady horizontal wind hits the loaded swarm; watch the integral + damping terms pull formation error back down.",
  C: "One drone fails mid-lift. The coordinator does not reassign the payload — it aborts and lands, per the real controller.",
  D: "Payload mass exceeds the safety-margined lift capacity. The mission is refused before arming — nothing leaves the ground.",
  E: "One drone is shoved off its slot before the tethers load. Formation + consensus + avoidance pull it back in.",
};

function buildScenarioParamUI(key) {
  const container = document.getElementById("scenario-params");
  container.innerHTML = "";
  document.getElementById("scenario-desc").textContent = SCENARIO_DESC[key];
  (SCENARIO_FIELDS[key] || []).forEach((f) => {
    const label = document.createElement("label");
    label.textContent = f.label;
    const input = document.createElement("input");
    input.type = f.type;
    input.step = f.step ?? "any";
    input.value = f.value;
    input.id = f.id;
    container.appendChild(label);
    container.appendChild(input);
  });
}

document.getElementById("scenario-select").addEventListener("change", (e) => {
  buildScenarioParamUI(e.target.value);
});
buildScenarioParamUI("A");

document.getElementById("speed-slider").addEventListener("input", (e) => {
  document.getElementById("speed-label").textContent = parseFloat(e.target.value).toFixed(2);
});

function readOverrides() {
  return {
    radius_m: parseFloat(document.getElementById("p-radius").value),
    payload_mass_kg: parseFloat(document.getElementById("p-payload").value),
    tether_len_m: parseFloat(document.getElementById("p-tether").value),
    tether_stiffness_n_per_m: parseFloat(document.getElementById("p-stiffness").value),
    k_formation: parseFloat(document.getElementById("p-kform").value),
    k_consensus: parseFloat(document.getElementById("p-kcons").value),
    k_integral: parseFloat(document.getElementById("p-kint").value),
    k_damping: parseFloat(document.getElementById("p-kdamp").value),
  };
}

function readScenarioParams(key) {
  const params = {};
  (SCENARIO_FIELDS[key] || []).forEach((f) => {
    const el = document.getElementById(f.id);
    params[f.param] = parseFloat(el.value);
  });
  if (key === "E") {
    // collapse the single "push distance" field into the (x, y, z) offset the
    // scenario builder actually takes.
    params.push_offset = [params.push_dist, 0.0, 0.0];
    delete params.push_dist;
  }
  if (key === "D") {
    params.payload_mass_kg = parseFloat(document.getElementById("p-payload").value);
  }
  return params;
}

// ------------------------------------------------------------------- socket
const urlParams = new URLSearchParams(window.location.search);
const wsUrl = urlParams.get("ws") || `ws://${window.location.hostname}:8765`;
let socket = null;
let running = false;

function connect() {
  socket = new WebSocket(wsUrl);
  socket.addEventListener("open", () => setStatus("Connected. Choose a scenario and press Run."));
  socket.addEventListener("close", () => { setStatus("Disconnected — retrying in 2s…"); setTimeout(connect, 2000); });
  socket.addEventListener("error", () => setStatus("Connection error."));
  socket.addEventListener("message", (ev) => {
    const msg = JSON.parse(ev.data);
    if (msg.type === "meta") {
      buildSceneForSwarm(msg.n_drones, msg.comm_links);
      window.__attachPoints = msg.attach_points;
      setStatus(`Running: ${msg.name}`);
      document.getElementById("summary-box").style.display = "none";
    } else if (msg.type === "frame") {
      applyFrame(msg.data);
    } else if (msg.type === "rejected") {
      setStatus("Mission rejected before liftoff — see feasibility below.");
      showSummary({ rejected_before_liftoff: true, feasibility: msg.feasibility });
      setRunning(false);
    } else if (msg.type === "done") {
      setStatus("Scenario complete.");
      showSummary(msg.summary);
      setRunning(false);
    } else if (msg.type === "error") {
      setStatus(`Error: ${msg.message}`);
      setRunning(false);
    }
  });
}
connect();

function setStatus(text) { document.getElementById("status-line").textContent = text; }
function showSummary(summary) {
  const box = document.getElementById("summary-box");
  box.style.display = "block";
  box.textContent = JSON.stringify(summary, null, 2);
}
function setRunning(v) {
  running = v;
  document.getElementById("run-btn").disabled = v;
  document.getElementById("stop-btn").disabled = !v;
}

document.getElementById("run-btn").addEventListener("click", () => {
  if (!socket || socket.readyState !== WebSocket.OPEN) return;
  const key = document.getElementById("scenario-select").value;
  const speed = parseFloat(document.getElementById("speed-slider").value);
  socket.send(JSON.stringify({
    cmd: "start",
    scenario: key,
    overrides: readOverrides(),
    params: readScenarioParams(key),
    speed,
  }));
  setRunning(true);
  document.getElementById("event-banner").style.display = "none";
});

document.getElementById("stop-btn").addEventListener("click", () => {
  if (socket && socket.readyState === WebSocket.OPEN) socket.send(JSON.stringify({ cmd: "stop" }));
  setRunning(false);
});
