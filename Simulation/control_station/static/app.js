"use strict";

const app = {
  config: null,
  state: null,
  environmentMode: "fixed",
  mapControl: null,
  mapMission: null,
  droneMarkerControl: null,
  droneMarkerMission: null,
  trailLineControl: null,
  trailLineMission: null,
  trail: [],
  waypointMarkers: [],
  routeLine: null,
  tileLayerControl: null,
  tileLayerMission: null,
  tileErrors: 0,
  waypoints: [],
  followDrone: true,
  lastLogId: 0,
  rcTimers: new Map(),
  pendingActions: new Set(),
  pollBusy: false,
  pollFailures: 0,
  transport: { status: 'online', since: Date.now(), error: null },
  followControl: true,
  followMission: true,
};

const experimentState = {
  selectedProfile: null,
  queue: [],
  activeRun: null,
  runs: [],
  comparison: null,
  filters: { status: 'ALL' },
};

const $ = (selector) => document.querySelector(selector);
const $$ = (selector) => [...document.querySelectorAll(selector)];
// Keep a small margin below the backend's 4 s MAVLink safety timeout. This
// avoids presenting a false STALE state during a short remote-desktop stall.
const MAVLINK_STALE_AFTER_S = 3.5;

async function api(path, options = {}) {
  const headers = { "Content-Type": "application/json", ...(options.headers || {}) };
  if (app.config && app.config.csrfToken && options.method && options.method.toUpperCase() !== "GET") {
    headers["X-P4-Transwing-CSRF"] = app.config.csrfToken;
  }
  const response = await fetch(path, {
    headers,
    ...options,
  });
  const payload = await response.json();
  if (!response.ok || payload.ok === false) {
    throw new Error(payload.error || `HTTP ${response.status}`);
  }
  return payload;
}

function toast(message, type = "info", timeout = 4200) {
  const element = document.createElement("div");
  element.className = `toast ${type}`;
  element.textContent = message;
  $("#toast-region").append(element);
  window.setTimeout(() => element.remove(), timeout);
}

function format(value, digits = 1, fallback = "--") {
  const numeric = Number(value);
  return Number.isFinite(numeric) ? numeric.toFixed(digits) : fallback;
}

function clamp(value, minimum, maximum) {
  return Math.min(maximum, Math.max(minimum, Number(value)));
}

function escapeHtml(value) {
  return String(value)
    .replaceAll("&", "&amp;")
    .replaceAll("<", "&lt;")
    .replaceAll(">", "&gt;")
    .replaceAll('"', "&quot;")
    .replaceAll("'", "&#039;");
}

const FLIGHT_ACTIONS = {
  arm: { endpoint: "/api/vehicle/arm", pendingText: "ĐANG ARM...", confirm: null },
  disarm: { endpoint: "/api/vehicle/disarm", pendingText: "ĐANG DISARM...", confirm: "Bạn có chắc chắn muốn DISARM? (Có thể gây rơi máy bay nếu đang bay)" },
  takeoff: { endpoint: "/api/vehicle/takeoff", pendingText: "ĐANG CẤT CÁNH...", confirm: "Bắt đầu trình tự cất cánh tự động?" },
  qland: { endpoint: "/api/vehicle/qland", pendingText: "ĐANG QLAND...", confirm: "Kích hoạt hạ cánh QLAND?" },
  rtl: { endpoint: "/api/vehicle/mode", payload: { mode: "RTL" }, pendingText: "ĐANG RTL...", confirm: "Quay về Home (RTL)?" },
  abort: { endpoint: "/api/vehicle/abort", pendingText: "ĐANG ABORT...", confirm: "Hủy tác vụ hiện tại (ABORT)?" },
  climb: { endpoint: "/api/vehicle/climb", pendingText: "ĐANG LEO/HẠ...", confirm: null },
  transitionFw: {
    endpoint: "/api/vehicle/transition-fw",
    payload: { throttle: 1750, minimumAltitude: 25 },
    pendingText: "ĐANG CHUYỂN...",
    confirm: "Chuyển từ QLOITER sang FBWB? RC3 chỉ tăng lên 1750 trong pha chuyển tiếp, sau đó tự về cruise 1470 (~34 m/s). Hãy bảo đảm vùng bay phía trước thông thoáng."
  }
};

function deriveFlightPhase(snapshot) {
  if (app.transport.status === 'offline') return 'OFFLINE';
  if (snapshot.heartbeatAge > MAVLINK_STALE_AFTER_S) return 'STALE';
  const mode = String(snapshot.mode || '').toUpperCase();
  if (mode === 'UNKNOWN' || mode === 'INITIALISING') return 'FAILSAFE';
  if (!snapshot.armed) return 'READY_DISARMED';
  if (snapshot.operation?.status === 'running') {
    if (snapshot.operation.name === 'takeoff') return 'TAKING_OFF';
    if (snapshot.operation.name === 'land' || mode === 'QLAND' || mode === 'LAND') return 'LANDING';
  }
  if ((snapshot.position?.relativeAlt > 3) || (snapshot.hud?.groundspeed > 4)) {
    return (mode.startsWith('Q') || (mode === 'AUTO' && snapshot.position?.relativeAlt < 10)) ? 'AIRBORNE_VTOL' : 'AIRBORNE_FW';
  }
  return 'ARMED_GROUND';
}

function deriveCapabilities(state, missionState) {
  const phase = deriveFlightPhase(state.vehicle);
  const isFlightReady = phase !== 'OFFLINE'
    && phase !== 'STALE'
    && phase !== 'FAILSAFE'
    && Number(state.vehicle.gps?.fixType || 0) >= 3
    && state.vehicle.homeSet === true;
  const isArmed = state.vehicle.armed;
  const opRunning = state.vehicle.operation?.status === 'running';
  const connected = phase !== 'OFFLINE' && phase !== 'STALE';
  const mode = String(state.vehicle.mode || '').toUpperCase();
  const altitude = Number(state.vehicle.position?.relativeAlt || 0);
  const roll = Math.abs(Number(state.vehicle.attitude?.roll || 0));
  const pitch = Math.abs(Number(state.vehicle.attitude?.pitch || 0));

  const fcCount = state.vehicle.mission?.fcCount || 0;

  return {
    // ArduPilot remains the authority for real pre-arm checks. Do not block
    // commands merely because optional SITL battery telemetry is absent.
    arm: isFlightReady && !isArmed && !opRunning,
    disarm: connected && isArmed,
    takeoff: isFlightReady && !opRunning,
    qland: connected,
    rtl: connected,
    abort: connected,
    // Missions remain on the FC across dashboard restarts, while the
    // session-local uploadedMissionId does not. fcCount is authoritative.
    startAuto: isFlightReady && !opRunning && (fcCount >= 2),
    climb: isFlightReady && !opRunning && String(state.vehicle.mode).toUpperCase() === 'FBWB',
    transitionFw: isFlightReady
      && isArmed
      && !opRunning
      && mode === 'QLOITER'
      && altitude >= 25
      && roll <= 10
      && pitch <= 10,
    uploadMission: connected && !isArmed,
    mode: connected,
    rc: connected,
    phase
  };
}

function setButtonAvailability(vehicle) {
  const capabilities = deriveCapabilities(app.state || { vehicle }, null);
  
  const bindState = (btnId, canDo, isPending, pendingText, defaultText) => {
    const el = $(btnId);
    if (!el) return;
    el.disabled = !canDo || isPending;
    if (pendingText && defaultText) {
      el.textContent = isPending ? pendingText : defaultText;
    }
  };

  bindState("#arm-button", capabilities.arm, app.pendingActions.has("/api/vehicle/arm"), FLIGHT_ACTIONS.arm.pendingText, "ARM");
  bindState("#disarm-button", capabilities.disarm, app.pendingActions.has("/api/vehicle/disarm"), FLIGHT_ACTIONS.disarm.pendingText, "DISARM");
  bindState("#takeoff-button", capabilities.takeoff, app.pendingActions.has("/api/vehicle/takeoff"), FLIGHT_ACTIONS.takeoff.pendingText, "CẤT CÁNH");
  bindState("#qland-button", capabilities.qland, app.pendingActions.has("/api/vehicle/qland"), FLIGHT_ACTIONS.qland.pendingText, "QLAND");
  bindState("#rtl-button", capabilities.rtl, app.pendingActions.has("/api/vehicle/mode"), FLIGHT_ACTIONS.rtl.pendingText, "RTL");
  bindState("#abort-button", capabilities.abort, app.pendingActions.has("/api/vehicle/abort"), FLIGHT_ACTIONS.abort.pendingText, "ABORT");
  bindState("#climb-button", capabilities.climb, app.pendingActions.has("/api/vehicle/climb"), FLIGHT_ACTIONS.climb.pendingText, "TỰ ĐỘNG LEO/HẠ");
  bindState(
    "#transition-fw-button",
    capabilities.transitionFw,
    app.pendingActions.has("/api/vehicle/transition-fw"),
    FLIGHT_ACTIONS.transitionFw.pendingText,
    "CHUYỂN CÁNH BẰNG",
  );

  ["#center-rc-button", "#release-rc-button", "#clear-vehicle-mission"].forEach(selector => {
    const el = $(selector);
    if (el) el.disabled = !capabilities.rc;
  });

  const uploadBtn = $("#upload-mission");
  if (uploadBtn) uploadBtn.disabled = !capabilities.uploadMission;

  const startMissionBtn = $("#start-mission");
  if (startMissionBtn) startMissionBtn.disabled = !capabilities.startAuto;

  $$("#mode-grid button").forEach((btn) => {
    btn.disabled = btn.dataset.mode === "FBWB"
      ? !capabilities.transitionFw
      : !capabilities.mode;
  });
  
  const rcOwner = app.state?.vehicle?.rcOwner || 'NONE';
  const externalEnabled = Boolean(app.state?.vehicle?.externalController?.enabled);
  const isRcFree = (rcOwner === 'NONE' || rcOwner === 'USER') && !externalEnabled;
  
  if (!isRcFree) {
    // If rcOwner is TAKEOFF, ALTITUDE, LANDING, etc., sliders are disabled
    $$(".rc-panel input[type=range]").forEach(slider => slider.disabled = true);
  } else {
    $$(".rc-panel input[type=range]").forEach(slider => slider.disabled = !capabilities.rc);
  }

  // Guided FAB mirror the primary controls' availability
  const fTakeoff = $("[data-fab='takeoff']"); if(fTakeoff) fTakeoff.disabled = !capabilities.takeoff || app.pendingActions.has("/api/vehicle/takeoff");
  const fDisarm = $("[data-fab='disarm']"); if(fDisarm) fDisarm.disabled = !capabilities.disarm || app.pendingActions.has("/api/vehicle/disarm");
  ["rtl", "qland", "abort"].forEach((k) => {
    const el = $(`[data-fab='${k}']`);
    if (el) el.disabled = !capabilities[k];
  });
}

function askConfirm(title, message) {
  return new Promise((resolve) => {
    const dialog = $("#confirm-dialog");
    $("#confirm-dialog-title").textContent = title;
    $("#confirm-dialog-message").textContent = message;
    
    const onOk = () => { cleanup(); resolve(true); };
    const onCancel = () => { cleanup(); resolve(false); };
    
    const cleanup = () => {
      $("#confirm-dialog-ok").removeEventListener("click", onOk);
      $("#confirm-dialog-cancel").removeEventListener("click", onCancel);
      dialog.close();
    };
    
    $("#confirm-dialog-ok").addEventListener("click", onOk);
    $("#confirm-dialog-cancel").addEventListener("click", onCancel);
    dialog.showModal();
  });
}

async function dispatchFlightAction(actionName, extraPayload = {}) {
  const vehicle = app.state?.vehicle;
  if (!vehicle) return;
  const capabilities = deriveCapabilities(app.state, null);
  if (!capabilities[actionName] && actionName !== 'disarm') { // Disarm might bypass some capability checks in emergencies
    return;
  }
  
  const actionDef = FLIGHT_ACTIONS[actionName];
  if (!actionDef) return;
  
  if (actionDef.confirm) {
    const confirmed = await askConfirm("Xác nhận", actionDef.confirm);
    if (!confirmed) return;
  }
  
  const payload = { ...(actionDef.payload || {}), ...extraPayload };
  await vehicleAction(actionDef.endpoint, payload, `Đã thực hiện: ${actionName}`);
}

function enterDisconnectedUi(error) {
  app.transport = { status: 'offline', since: Date.now(), error };
  const banner = $("#disconnect-banner");
  if (banner) {
    banner.textContent = `Mất kết nối backend — mọi điều khiển bay đã bị khóa (${error.message || 'Lỗi mạng'})`;
    banner.classList.remove("hidden");
  }
  
  if (app.state && app.state.vehicle) {
    app.state.vehicle.mode = "UNKNOWN";
    app.state.vehicle.armed = false;
    app.state.vehicle.connected = false;
    app.state.vehicle.hud = { throttle: 0, groundspeed: 0, airspeed: 0, climb: 0 };
    setButtonAvailability(app.state.vehicle);
  }
}


function initialiseEnvironmentSelector() {
  $$(".segment").forEach((button) => {
    button.addEventListener("click", () => {
      app.environmentMode = button.dataset.environment;
      $$(".segment").forEach((item) => item.classList.toggle("active", item === button));
      $("#fixed-options").classList.toggle("hidden", app.environmentMode !== "fixed");
      $("#live-options").classList.toggle("hidden", app.environmentMode !== "live");
    });
  });
}

function populateConfig(config) {
  app.config = config;
  const select = $("#case-select");
  select.innerHTML = "";
  config.cases.forEach((testCase) => {
    const option = document.createElement("option");
    option.value = testCase.name;
    option.textContent = `${testCase.name} · ${Number(testCase.payloadKg).toFixed(2)} kg tải`;
    option.dataset.description = testCase.description || "";
    select.append(option);
  });
  const preferred = [...select.options].find((option) => option.value === "baseline");
  if (preferred) select.value = preferred.value;
  updateCaseDescription();
  select.addEventListener("change", updateCaseDescription);
  if (!config.missionPlannerFound) {
    $("#open-mission-planner").checked = false;
    toast(`Không tìm thấy Mission Planner tại ${config.missionPlannerPath}`, "error", 7000);
  }
}

function updateCaseDescription() {
  const option = $("#case-select").selectedOptions[0];
  $("#case-description").textContent = option?.dataset.description || "Không có mô tả.";
}

async function startSimulation() {
  const payload = {
    environmentMode: app.environmentMode,
    case: $("#case-select").value,
    payloadKg: Number($("#payload-input").value),
    windMode: $("#wind-mode").value,
    openMissionPlanner: $("#open-mission-planner").checked,
  };
  $("#start-button").disabled = true;
  try {
    await api("/api/simulation/start", {
      method: "POST",
      body: JSON.stringify(payload),
    });
    toast("Đang khởi động Gazebo, SITL và Mission Planner…", "success");
  } catch (error) {
    toast(error.message, "error");
    $("#start-button").disabled = false;
  }
}

async function stopSimulation() {
  if (!window.confirm("Dừng toàn bộ tiến trình do dashboard đã khởi động?")) return;
  $("#stop-button").disabled = true;
  try {
    await api("/api/simulation/stop", { method: "POST", body: "{}" });
    toast("Đã dừng stack mô phỏng.", "success");
  } catch (error) {
    toast(error.message, "error");
  }
}

async function vehicleAction(path, body = {}, successMessage = "") {
  app.pendingActions.add(path);
  if (app.state?.vehicle) setButtonAvailability(app.state.vehicle);
  try {
    const result = await api(path, {
      method: "POST",
      body: JSON.stringify(body),
    });
    if (successMessage) toast(successMessage, "success");
    return result;
  } catch (error) {
    toast(error.message, "error");
    return null;
  } finally {
    app.pendingActions.delete(path);
    if (app.state?.vehicle) setButtonAvailability(app.state.vehicle);
  }
}


function formatBytes(bytes) {
  const numeric = Number(bytes);
  if (!Number.isFinite(numeric)) return "--";
  if (numeric >= 1024 * 1024) return `${(numeric / (1024 * 1024)).toFixed(1)} MB`;
  if (numeric >= 1024) return `${(numeric / 1024).toFixed(0)} KB`;
  return `${numeric} B`;
}

async function refreshFlightLogs() {
  const list = $("#flight-log-list");
  if (!list) return;
  try {
    const payload = await api("/api/logs/flight");
    const entries = payload.entries || [];
    list.innerHTML = entries.length
      ? entries
          .map(
            (entry) => `
              <div class="flight-log-item">
                <div class="flight-log-meta">
                  <span class="flight-log-name">${escapeHtml(entry.name)} ${Date.now() - new Date(entry.modifiedAt).getTime() < 60000 ? '<span class="mini-badge">đang ghi</span>' : ''}</span>
                  <span class="flight-log-sub">${new Date(entry.modifiedAt).toLocaleString("vi-VN")} · ${formatBytes(entry.sizeBytes)}</span>
                </div>
                ${Date.now() - new Date(entry.modifiedAt).getTime() < 60000 ? '<span class="text-button" style="color:var(--text-faint);cursor:default">Đang ghi</span>' : `<a class="text-button" href="/api/logs/flight/${encodeURIComponent(entry.name)}" download>Tải xuống</a>`}
              </div>`,
          )
          .join("")
      : `<div class="empty-message">Chưa tìm thấy log SITL (kiểm tra thư mục ~/ardupilot/ArduPlane/logs trong WSL).</div>`;
  } catch (error) {
    list.innerHTML = `<div class="empty-message">Không tải được danh sách log: ${escapeHtml(error.message)}</div>`;
  }
}

function setupGuidedFab() {
  $$("[data-fab]").forEach((btn) => {
    btn.addEventListener("click", () => {
      const action = btn.dataset.fab;
      let extra = {};
      if (action === "takeoff") extra = { altitude: Number($("#takeoff-altitude").value) };
      dispatchFlightAction(action, extra);
    });
  });
}


function buildCompassTape() {
  const track = $("#pfd-compass-track");
  if (!track) return;
  const marks = [];
  for (let loop = 0; loop < 2; loop++) {
    for (let deg = 0; deg < 360; deg += 10) {
      const card = { 0: "N", 90: "E", 180: "S", 270: "W" }[deg];
      marks.push(`<span class="${card ? "cardinal" : ""}">${card || deg}</span>`);
    }
  }
  track.innerHTML = marks.join("");
}

function setupVehicleControls() {
  $("#arm-button").addEventListener("click", () => dispatchFlightAction("arm"));
  $("#disarm-button").addEventListener("click", () => dispatchFlightAction("disarm"));
  $("#takeoff-button").addEventListener("click", () => {
    const altitude = Number($("#takeoff-altitude").value);
    dispatchFlightAction("takeoff", { altitude });
  });
  $("#qland-button").addEventListener("click", () => dispatchFlightAction("qland"));
  $("#rtl-button").addEventListener("click", () => dispatchFlightAction("rtl"));
  $("#abort-button").addEventListener("click", () => dispatchFlightAction("abort"));
  $("#climb-button")?.addEventListener("click", () => {
    const altitude = Number($("#climb-altitude").value);
    dispatchFlightAction("climb", { altitude });
  });
  $("#transition-fw-button")?.addEventListener("click", () =>
    dispatchFlightAction("transitionFw")
  );
  $$("#mode-grid button").forEach((button) => {
    button.addEventListener("click", () => {
      if (button.dataset.mode === "FBWB") {
        dispatchFlightAction("transitionFw");
        return;
      }
      vehicleAction(
        "/api/vehicle/mode",
        { mode: button.dataset.mode },
        `Đã yêu cầu mode ${button.dataset.mode}.`,
      );
    });
  });
  $("#center-rc-button").addEventListener("click", async () => {
    await vehicleAction("/api/vehicle/center", { throttle: 1500 }, "RC đã cân giữa.");
    setLocalRc([1500, 1500, 1500, 1500]);
  });
  $("#release-rc-button").addEventListener("click", () =>
    vehicleAction("/api/vehicle/release-rc", {}, "Đã trả quyền RC cho autopilot.")
  );
  $("#external-rc-enable-button")?.addEventListener("click", () =>
    vehicleAction(
      "/api/controller/external/enable",
      {},
      "Đã trao RC1–4 cho radio ngoài. Giữ vùng bay an toàn.",
    )
  );
  $("#external-rc-disable-button")?.addEventListener("click", () =>
    vehicleAction(
      "/api/controller/external/disable",
      {},
      "Đã ngắt radio ngoài; RC được giữ trung tính.",
    )
  );

  const sliderMap = {
    "roll-slider": "roll-output",
    "pitch-slider": "pitch-output",
    "throttle-slider": "throttle-output",
    "yaw-slider": "yaw-output",
  };
  Object.entries(sliderMap).forEach(([sliderId, outputId]) => {
    const slider = $(`#${sliderId}`);
    slider.addEventListener("input", () => {
      $(`#${outputId}`).value = slider.value;
      const channel = Number(slider.dataset.channel);
      window.clearTimeout(app.rcTimers.get(channel));
      app.rcTimers.set(
        channel,
        window.setTimeout(() => sendRc(channel, Number(slider.value)), 120),
      );
    });
  });
}

async function sendRc(channel, value) {
  try {
    await api("/api/vehicle/rc", {
      method: "POST",
      body: JSON.stringify({ channel, value }),
    });
  } catch (error) {
    toast(error.message, "error");
  }
}

function setLocalRc(values) {
  const ids = ["roll", "pitch", "throttle", "yaw"];
  ids.forEach((name, index) => {
    $(`#${name}-slider`).value = values[index];
    $(`#${name}-output`).value = values[index];
  });
}

function initialiseMap() {
  const home = app.config.home;
  const tileUrl = "https://mt1.google.com/vt/lyrs=s&x={x}&y={y}&z={z}";
  
  // 1. Control Map
  app.mapControl = L.map("map-control", {
    center: [home.lat, home.lon],
    zoom: 15,
    zoomControl: true,
    attributionControl: false,
  });
  app.tileLayerControl = L.tileLayer(tileUrl, { maxZoom: 22, crossOrigin: true }).addTo(app.mapControl);
  
  const mapControlStatus = $("#map-control-status");
  app.tileLayerControl.on("loading", () => { mapControlStatus.textContent = "Map loading"; mapControlStatus.className = "map-status loading"; });
  app.tileLayerControl.on("load", () => { mapControlStatus.textContent = `Map online · Z${app.mapControl.getZoom()}`; mapControlStatus.className = "map-status online"; });

  const droneIcon = L.divIcon({
    className: "",
    html: `<div class="drone-marker" style="--heading:270deg">
        <svg viewBox="0 0 64 64" aria-hidden="true">
          <path d="M32 3c2.2 0 3.8 2 4.2 5.1l2.2 16.2 20.3 10.8c1.6.9 2.3 2.6 1.7 4.3l-1 2.7-21.2-6.2-.6 11.6 8.2 7.4-1.5 4.2L32 54.7 19.7 59l-1.5-4.2 8.2-7.4-.6-11.6-21.2 6.2-1-2.7c-.6-1.7.1-3.4 1.7-4.3l20.3-10.8 2.2-16.2C28.2 5 29.8 3 32 3Z"/>
          <path class="plane-detail" d="M32 10v39"/>
        </svg>
      </div>`,
    iconSize: [42, 42],
    iconAnchor: [21, 21],
  });
  
  app.droneMarkerControl = L.marker([home.lat, home.lon], { icon: droneIcon, zIndexOffset: 1000 }).addTo(app.mapControl);
  app.trailLineControl = L.polyline([], { color: "#54e6e7", weight: 2, opacity: 0.72 }).addTo(app.mapControl);
  
  app.lineHeading = L.polyline([], { color: "red", weight: 2 }).addTo(app.mapControl);
  app.lineTarget = L.polyline([], { color: "green", weight: 2 }).addTo(app.mapControl);
  app.lineTrack = L.polyline([], { color: "black", weight: 2 }).addTo(app.mapControl);
  app.lineWp = L.polyline([], { color: "orange", weight: 2 }).addTo(app.mapControl);

  // 2. Mission Map
  app.mapMission = L.map("map-mission", {
    center: [home.lat, home.lon],
    zoom: 15,
    zoomControl: true,
    attributionControl: false,
  });
  app.tileLayerMission = L.tileLayer(tileUrl, { maxZoom: 22, crossOrigin: true }).addTo(app.mapMission);
  
  const mapMissionStatus = $("#map-mission-status");
  app.tileLayerMission.on("loading", () => { mapMissionStatus.textContent = "Map loading"; mapMissionStatus.className = "map-status loading"; });
  app.tileLayerMission.on("load", () => { mapMissionStatus.textContent = `Map online · Z${app.mapMission.getZoom()}`; mapMissionStatus.className = "map-status online"; });
  
  app.droneMarkerMission = L.marker([home.lat, home.lon], { icon: droneIcon, zIndexOffset: 1000 }).addTo(app.mapMission);
  app.trailLineMission = L.polyline([], { color: "#54e6e7", weight: 2, opacity: 0.72 }).addTo(app.mapMission);
  app.routeLine = L.polyline([], { color: "#ffbf5a", weight: 2, opacity: 0.86, dashArray: "8 6" }).addTo(app.mapMission);

  const radius = app.config?.missionPolicy?.operatingRadiusM ?? 425;
  // Geofence overlay
  L.circle([home.lat, home.lon], {
    radius: radius,
    color: "#f5a623",
    fillColor: "#f5a623",
    fillOpacity: 0.12,
    weight: 1.5,
    dashArray: "5 6"
  }).addTo(app.mapControl);
  
  L.circle([home.lat, home.lon], {
    radius: radius,
    color: "#f5a623",
    fillColor: "#f5a623",
    fillOpacity: 0.12,
    weight: 1.5,
    dashArray: "5 6"
  }).addTo(app.mapMission);

  app.mapControl.on("dragstart", () => { app.followDrone = false; });
  app.mapMission.on("dragstart", () => { app.followDrone = false; });

  app.mapMission.on("click", (event) => {
    addWaypoint({
      command: "WAYPOINT",
      lat: event.latlng.lat,
      lon: event.latlng.lng,
      alt: 50,
      acceptance: 20,
      hold: 0,
    });
  });
  app.mapMission.on("mousemove", (event) => {
    $("#map-coordinates").textContent = `${event.latlng.lat.toFixed(6)}, ${event.latlng.lng.toFixed(6)}`;
  });
  
  app.mapControl.on("zoomstart", () => { mapControlStatus.textContent = `Map loading · Z${app.mapControl.getZoom()}`; mapControlStatus.className = "map-status loading"; });
  app.mapMission.on("zoomstart", () => { mapMissionStatus.textContent = `Map loading · Z${app.mapMission.getZoom()}`; mapMissionStatus.className = "map-status loading"; });

  [app.tileLayerControl, app.tileLayerMission].forEach(layer => {
    layer.on("tileerror", () => {
      app.tileErrors++;
      if (app.tileErrors >= 3) {
        mapControlStatus.textContent = "MAP DEGRADED"; mapControlStatus.className = "map-status error";
        mapMissionStatus.textContent = "MAP DEGRADED"; mapMissionStatus.className = "map-status error";
      }
    });
  });

  const ro = new ResizeObserver(() => {
    if (app.mapControl) app.mapControl.invalidateSize({ pan: false });
    if (app.mapMission) app.mapMission.invalidateSize({ pan: false });
  });
  ro.observe($("#map-control"));
  ro.observe($("#map-mission"));

  window.setTimeout(() => {
    if (app.mapControl) app.mapControl.invalidateSize({ pan: false });
    if (app.mapMission) app.mapMission.invalidateSize({ pan: false });
  }, 150);
}

function addWaypoint(data) {
  const maxItems = app.config?.missionPolicy?.maxItems ?? 100;
  if (app.waypoints.length >= maxItems) {
    toast(`Không thể thêm waypoint: vượt quá số lượng tối đa (${maxItems})`, "error");
    return;
  }
  const home = app.config.home;
  app.waypoints.push({
    command: data.command || "WAYPOINT",
    lat: Number.isFinite(Number(data.lat)) ? Number(data.lat) : home.lat,
    lon: Number.isFinite(Number(data.lon)) ? Number(data.lon) : home.lon,
    alt: Number.isFinite(Number(data.alt)) ? Number(data.alt) : 50,
    acceptance: Number.isFinite(Number(data.acceptance)) ? Number(data.acceptance) : 20,
    hold: Number.isFinite(Number(data.hold)) ? Number(data.hold) : 0,
  });
  renderWaypoints();
}

function renderWaypoints() {
  const body = $("#waypoint-body");
  body.innerHTML = "";
  if (!app.waypoints.length) {
    body.innerHTML = `
      <tr class="empty-row"><td colspan="7">
        Chưa có waypoint. Nhấp bản đồ hoặc dùng nút phía trên.
      </td></tr>`;
  } else {
    const maxAlt = app.config?.missionPolicy?.maxRelativeAltitudeM ?? 500;
    app.waypoints.forEach((waypoint, index) => {
      const positional = !waypoint.command.startsWith("TRANSITION_");
      const row = document.createElement("tr");
      row.draggable = true;
      row.dataset.index = index;
      row.innerHTML = `
        <td class="row-index">${index + 1}</td>
        <td>
          <select data-field="command">
            ${["WAYPOINT", "VTOL_TAKEOFF", "TRANSITION_FW", "LOITER_TIME", "TRANSITION_VTOL", "VTOL_LAND"]
              .map((command) => `<option value="${command}" ${waypoint.command === command ? "selected" : ""}>${command}</option>`)
              .join("")}
          </select>
        </td>
        <td><input data-field="lat" type="number" step="0.000001" value="${waypoint.lat.toFixed(7)}" ${positional ? "" : "disabled"}></td>
        <td><input data-field="lon" type="number" step="0.000001" value="${waypoint.lon.toFixed(7)}" ${positional ? "" : "disabled"}></td>
        <td><input data-field="alt" type="number" min="0" max="${maxAlt}" step="1" value="${waypoint.alt}" ${positional ? "" : "disabled"}></td>
        <td><input data-field="acceptance" type="number" min="0" max="200" step="1" value="${waypoint.acceptance}" ${positional ? "" : "disabled"}></td>
        <td><button class="delete-waypoint" title="Xóa waypoint">×</button></td>`;
      row.querySelectorAll("[data-field]").forEach((input) => {
        input.addEventListener("change", () => {
          const field = input.dataset.field;
          waypoint[field] = field === "command" ? input.value : Number(input.value);
          renderWaypoints();
        });
      });
      row.querySelector(".delete-waypoint").addEventListener("click", () => {
        app.waypoints.splice(index, 1);
        renderWaypoints();
      });
      addDragHandlers(row);
      body.append(row);
    });
  }
  $("#mission-count").textContent = `${app.waypoints.length} ITEM`;
  updateMissionMap();
}

function addDragHandlers(row) {
  row.addEventListener("dragstart", () => row.classList.add("dragging"));
  row.addEventListener("dragend", () => {
    row.classList.remove("dragging");
    $$("#waypoint-body tr").forEach((item) => item.classList.remove("drag-over"));
  });
  row.addEventListener("dragover", (event) => {
    event.preventDefault();
    row.classList.add("drag-over");
  });
  row.addEventListener("dragleave", () => row.classList.remove("drag-over"));
  row.addEventListener("drop", (event) => {
    event.preventDefault();
    const source = $("#waypoint-body tr.dragging");
    if (!source || source === row) return;
    const from = Number(source.dataset.index);
    const to = Number(row.dataset.index);
    const [item] = app.waypoints.splice(from, 1);
    app.waypoints.splice(to, 0, item);
    renderWaypoints();
  });
}

function updateMissionMap() {
  if (!app.mapMission) return;
  app.waypointMarkers.forEach((marker) => marker.remove());
  app.waypointMarkers = [];
  const positions = [];
  app.waypoints.forEach((waypoint, index) => {
    if (waypoint.command.startsWith("TRANSITION_")) return;
    const icon = L.divIcon({
      className: "",
      html: `<div class="waypoint-marker"><span>${index + 1}</span></div>`,
      iconSize: [26, 26],
      iconAnchor: [13, 24],
    });
    const marker = L.marker([waypoint.lat, waypoint.lon], {
      icon,
      draggable: true,
    })
      .addTo(app.mapMission)
      .bindTooltip(`${index + 1} · ${waypoint.command}<br>${waypoint.alt} m`, {
        direction: "top",
      });
    marker.on("dragend", (event) => {
      const point = event.target.getLatLng();
      waypoint.lat = point.lat;
      waypoint.lon = point.lng;
      renderWaypoints();
    });
    app.waypointMarkers.push(marker);
    positions.push([waypoint.lat, waypoint.lon]);
  });
  app.routeLine.setLatLngs(positions);
}

function setupMissionControls() {
  $$("[data-add-command]").forEach((button) => {
    button.addEventListener("click", () => {
      const home = app.config.home;
      const command = button.dataset.addCommand;
      addWaypoint({
        command,
        lat: home.lat,
        lon: home.lon,
        alt: command === "VTOL_TAKEOFF" ? 50 : 0,
        acceptance: 20,
        hold: 0,
      });
    });
  });
  $("#load-sample-mission").addEventListener("change", (e) => {
    if (!e.target.value) return;
    loadSampleMission(e.target.value);
    e.target.value = "";
  });
  $("#clear-editor").addEventListener("click", () => {
    app.waypoints = [];
    renderWaypoints();
  });
  $("#upload-mission").addEventListener("click", async () => {
    if (!app.waypoints.length) {
      toast("Hãy thêm ít nhất một mission item.", "error");
      return;
    }
    await vehicleAction(
      "/api/mission/upload",
      { waypoints: app.waypoints },
      `Đã upload ${app.waypoints.length + 1} item (bao gồm Home).`,
    );
  });
  $("#start-mission").addEventListener("click", async () => {
    if (!window.confirm("ARM máy bay và chuyển sang AUTO từ waypoint 1?")) return;
    const uploadedMissionId = app.state?.vehicle?.mission?.uploadedMissionId;
    await vehicleAction(
      "/api/mission/start",
      {
        expected_mission_id: uploadedMissionId == null
          ? null
          : Number(uploadedMissionId),
      },
      "Mission AUTO đã bắt đầu.",
    );
  });
  $("#clear-vehicle-mission").addEventListener("click", () =>
    vehicleAction("/api/mission/clear", {}, "Đã yêu cầu xóa mission trên FC.")
  );
  
  $("#center-map-control-button").addEventListener("click", () => {
    app.followControl = true;
    const gps = app.state?.vehicle?.gps;
    if (gps && app.mapControl) app.mapControl.setView([gps.lat, gps.lon], Math.max(app.mapControl.getZoom(), 17));
  });
  $("#center-map-mission-button").addEventListener("click", () => {
    app.followMission = true;
    const gps = app.state?.vehicle?.gps;
    if (gps && app.mapMission) app.mapMission.setView([gps.lat, gps.lon], Math.max(app.mapMission.getZoom(), 17));
  });

}

function loadSampleMission(type) {
  const home = app.config.home;
  if (type === "rectangle") {
    app.waypoints = [
      { command: "VTOL_TAKEOFF", lat: home.lat, lon: home.lon, alt: 50, acceptance: 0, hold: 0 },
      { command: "TRANSITION_FW", lat: 0, lon: 0, alt: 0, acceptance: 0, hold: 0 },
      { command: "WAYPOINT", lat: 10.772100, lon: 106.655155604, alt: 50, acceptance: 25, hold: 0 },
      { command: "WAYPOINT", lat: 10.774794934, lon: 106.655155604, alt: 50, acceptance: 25, hold: 0 },
      { command: "WAYPOINT", lat: 10.774794934, lon: 106.657900, alt: 50, acceptance: 25, hold: 0 },
      { command: "WAYPOINT", lat: 10.773447967, lon: 106.657900, alt: 40, acceptance: 20, hold: 0 },
      { command: "VTOL_LAND", lat: home.lat, lon: home.lon, alt: 0, acceptance: 0, hold: 0 },
    ];
  } else if (type === "bk-logo") {
    const waypoints = [
      { command: "VTOL_TAKEOFF", lat: home.lat, lon: home.lon, alt: 50, acceptance: 0, hold: 0 },
      { command: "TRANSITION_FW", lat: 0, lon: 0, alt: 0, acceptance: 0, hold: 0 }
    ];
    
    const r = 250;
    const sin60 = Math.sin(Math.PI / 3);
    const cos60 = Math.cos(Math.PI / 3);
    
    const w0 = [0, r];
    const w1 = [r * sin60, r * cos60];
    const w2 = [r * sin60, -r * cos60];
    const w3 = [0, -r];
    const w4 = [-r * sin60, -r * cos60];
    const w5 = [-r * sin60, r * cos60];
    
    const UP = w0, DR = w2, DL = w4;
    const addV = (a, b) => [a[0] + b[0], a[1] + b[1]];
    
    const seq = [
      w1, addV(w1, UP), addV(w0, UP), w0, w1, w0, w5, addV(w5, UP), addV(w0, UP), addV(w1, UP), w1,
      addV(w1, DR), addV(w2, DR), w2, w1, w2, w3, addV(w3, DR), addV(w2, DR), addV(w3, DR), w3,
      addV(w3, DL), addV(w4, DL), w4, w3, w4, w5, addV(w5, DL), addV(w4, DL), addV(w5, DL), w5
    ];
    
    seq.forEach(pt => {
      const dLat = pt[1] / 111111;
      const dLon = pt[0] / (111111 * Math.cos(home.lat * Math.PI / 180));
      waypoints.push({ command: "WAYPOINT", lat: home.lat + dLat, lon: home.lon + dLon, alt: 50, acceptance: 15, hold: 0 });
    });
    
    waypoints.push({ command: "VTOL_LAND", lat: home.lat, lon: home.lon, alt: 0, acceptance: 0, hold: 0 });
    app.waypoints = waypoints;
  }
  renderWaypoints();
  if (app.mapMission) {
    const points = app.waypoints
      .filter((item) => !item.command.startsWith("TRANSITION_"))
      .map((item) => [item.lat, item.lon]);
    app.mapMission.fitBounds(points, { padding: [40, 40] });
  }
  toast(`Đã nạp lộ trình mẫu: ${type === 'bk-logo' ? 'Logo Bách Khoa' : 'Hình chữ nhật'}`, "success");
}

function updateSimulationState(simulation) {
  const statusMap = {
    stopped: "Đã dừng",
    starting: "Đang khởi động",
    running: "Đang chạy",
    degraded: "Tiến trình lỗi",
    error: "Khởi động lỗi",
    stopping: "Đang dừng",
  };
  $("#sim-status").textContent = statusMap[simulation.status] || simulation.status;
  $("#stack-badge").textContent = simulation.status.toUpperCase();
  const dot = $("#sim-dot");
  dot.className = "status-dot";
  if (simulation.status === "running") dot.classList.add("online");
  if (simulation.status === "starting" || simulation.status === "stopping") dot.classList.add("starting");
  $("#start-button").disabled = ["starting", "running", "degraded", "stopping"].includes(simulation.status);
  $("#stop-button").disabled = simulation.status === "stopped";
  if (simulation.error) {
    $("#stack-badge").textContent = "ERROR";
  }

  const processes = Object.values(simulation.processes || {});
  $("#process-list").innerHTML = processes.length
    ? processes
        .map(
          (process) => `
            <div class="process-item ${process.running ? "running" : "failed"}">
              <i></i><span>${escapeHtml(process.name)}</span>
              <code>${process.running ? `PID ${process.pid}` : `EXIT ${process.exitCode}`}</code>
            </div>`,
        )
        .join("")
    : `<div class="process-item"><i></i><span>Chưa khởi động tiến trình</span></div>`;
  if (simulation.missionPlannerPid) {
    $("#process-list").insertAdjacentHTML(
      "beforeend",
      `<div class="process-item running"><i></i><span>mission-planner</span><code>WIN ${simulation.missionPlannerPid}</code></div>`,
    );
  }
}

function updateExternalRcState(vehicle) {
  const external = vehicle.externalController || {};
  const fresh = Boolean(external.fresh);
  const enabled = Boolean(external.enabled);
  const failsafe = Boolean(external.failsafeLatched);
  const hoverRecovery = Boolean(external.hoverRecoveryActive);
  const age = external.ageSeconds == null ? null : Number(external.ageSeconds);
  const channels = Array.isArray(external.channels) ? external.channels : [];

  const dot = $("#external-rc-dot");
  const indicator = $("#external-rc-indicator");
  const panel = $(".external-rc-panel");
  dot.className = `status-dot ${fresh ? "online" : external.received ? "starting" : ""}`;
  indicator.classList.toggle("caution", Boolean(external.received) && !fresh);
  panel.classList.toggle("is-fresh", fresh);
  panel.classList.toggle("is-enabled", enabled);
  panel.classList.toggle("is-failsafe", failsafe);

  if (hoverRecovery) {
    $("#external-rc-status").textContent = "Hover recovery";
  } else if (failsafe) {
    $("#external-rc-status").textContent = "FAILSAFE";
  } else if (enabled && fresh) {
    $("#external-rc-status").textContent = "Radio active";
  } else if (fresh) {
    $("#external-rc-status").textContent = "MKR ready";
  } else if (external.received) {
    $("#external-rc-status").textContent = "MKR stale";
  } else {
    $("#external-rc-status").textContent = "Waiting for MKR";
  }

  $("#external-rc-chip").textContent = hoverRecovery
    ? "MKR: HOVER RECOVERY"
    : enabled
    ? "MKR: đang điều khiển"
    : failsafe
      ? "MKR: failsafe khóa"
      : fresh
        ? "MKR: frame hợp lệ"
        : "MKR: chờ frame";
  $("#external-rc-protocol").textContent = external.protocol || "P4RC/1";
  $("#external-rc-sequence").textContent = external.sequence ?? "--";
  $("#external-rc-age").textContent = Number.isFinite(age)
    ? `${Math.round(age * 1000)} ms`
    : "--";
  $("#external-rc-arm-switch").textContent = external.armSwitch || "--";
  $("#external-rc-qrtl-switch").textContent = external.qrtlSwitch || "--";
  $("#external-rc-mode-switch").textContent = external.modeSwitch
    ? `${external.modeSwitch} · ${external.modeRequest || "--"}`
    : "--";
  for (let index = 0; index < 10; index += 1) {
    const output = $(`#external-ch${index + 1}`);
    if (output) output.value = Number.isFinite(Number(channels[index])) ? channels[index] : "--";
  }

  const details = external.lastError
    || external.lastAction
    || (fresh
      ? (enabled
        ? "RC1–4 đang được gửi từ radio. SWA=ARM, SWB=QRTL, SWC=QLOITER/TRANS/FBWB."
        : "Frame hợp lệ. Kiểm tra stick rồi nhấn TRAO QUYỀN RADIO.")
      : "Chưa có USB bridge/MKR hợp lệ; radio không thể điều khiển máy bay.");
  $("#external-rc-message").textContent = details;

  const connected = Boolean(vehicle.connected);
  const enableButton = $("#external-rc-enable-button");
  const disableButton = $("#external-rc-disable-button");
  enableButton.disabled = !connected || !fresh || enabled;
  disableButton.disabled = !connected || !enabled;
}

function updateVehicleState(vehicle) {
  const connected = Boolean(vehicle.connected);
  $("#link-dot").classList.toggle("online", connected);
  const isStale = vehicle.heartbeatAge > MAVLINK_STALE_AFTER_S;
  
  if (!connected) {
    $("#link-status").textContent = "Chưa có MAVLink";
  } else if (isStale) {
    $("#link-status").textContent = `MAVLink (Stale) · ${format(vehicle.heartbeatAge, 1)} s`;
  } else {
    $("#link-status").textContent = `MAVLink · ${format(vehicle.heartbeatAge, 1)} s`;
  }
  
  $("#arm-state").textContent = vehicle.armed ? "ARMED" : "DISARMED";
  $("#arm-state").classList.toggle("disarmed", !vehicle.armed);

  const rcOwner = vehicle.rcOwner || 'NONE';
  const rcChip = $("#rc-owner-chip");
  if (rcChip) rcChip.textContent = `RC: ${rcOwner}`;
  updateExternalRcState(vehicle);

  // GPS chip
  const fix = Number(vehicle.gps?.fixType || 0);
  $("#ind-gps-value").textContent = isStale ? "--" : `${fix >= 3 ? "3D" : fix} · ${vehicle.gps?.satellites ?? 0}`;
  $("#gps-dot").className = `status-dot ${fix >= 3 && !isStale ? "online" : fix > 0 && !isStale ? "starting" : ""}`;
  
  // Battery chip
  const volt = vehicle.battery?.voltage;
  $("#ind-battery-value").textContent = (volt == null || isStale) ? "--" : `${format(volt, 1)}V`;
  $("#batt-dot").className = `status-dot ${volt == null || isStale ? "" : "online"}`;
  $("#vehicle-mode").textContent = isStale ? "UNKNOWN" : (vehicle.mode || "UNKNOWN");
  setButtonAvailability(vehicle);
  $$("#mode-grid button").forEach((button) => {
    const isActive = button.dataset.mode === vehicle.mode;
    button.classList.toggle("active", isActive);
    button.setAttribute("aria-pressed", isActive ? "true" : "false");
  });

  const attitude = vehicle.attitude || { roll: 0, pitch: 0, yaw: 0 };
  $("#horizon-world").style.setProperty("--roll", `${-attitude.roll}deg`);
  $("#horizon-world").style.setProperty("--pitch", `${clamp(attitude.pitch * 2.2, -75, 75)}px`);
  $("#roll-value").textContent = isStale ? "--" : `${format(attitude.roll, 1)}°`;
  $("#pitch-value").textContent = isStale ? "--" : `${format(attitude.pitch, 1)}°`;
  $("#yaw-value").textContent = isStale ? "--" : `${format(attitude.yaw, 1)}°`;
  const hdg = vehicle.position?.heading ?? attitude.yaw;
  $("#heading-value").textContent = isStale ? "--" : `${Math.round(hdg)}°`;
  const compassTrack = $("#pfd-compass-track");
  if (compassTrack && !isStale) compassTrack.style.transform = `translateX(${-hdg * 4.2}px)`;
  const pfdSpeed = $("#pfd-speed");
  if (pfdSpeed) pfdSpeed.textContent = isStale ? "--" : format(vehicle.hud?.groundspeed, 1);
  const pfdAlt = $("#pfd-alt");
  if (pfdAlt) pfdAlt.textContent = isStale ? "--" : format(vehicle.position?.relativeAlt, 1);

  $("#altitude-value").textContent = isStale ? "--" : format(vehicle.position?.relativeAlt, 1);
  $("#climb-value").textContent = isStale ? "--" : `VS ${format(vehicle.hud?.climb, 2)} m/s`;
  $("#groundspeed-value").textContent = isStale ? "--" : format(vehicle.hud?.groundspeed, 1);
  $("#airspeed-value").textContent = isStale ? "--" : `AS ${format(vehicle.hud?.airspeed, 1)} m/s`;
  $("#hud-throttle-value").textContent = isStale ? "--" : String(vehicle.hud?.throttle ?? 0);
  $("#throttle-meter").style.width = isStale ? "0%" : `${clamp(vehicle.hud?.throttle ?? 0, 0, 100)}%`;

  renderPrearmChecklist(vehicle.prearm);
  $("#battery-value").textContent = (vehicle.battery?.voltage == null || isStale)
    ? "UNKNOWN"
    : format(vehicle.battery.voltage, 1);
  $("#battery-remaining").textContent = vehicle.battery?.remaining == null
    ? "Chưa mô hình"
    : `${vehicle.battery.remaining}% còn lại`;
  $("#latitude-value").textContent = isStale ? "--" : format(vehicle.gps?.lat, 7);
  $("#longitude-value").textContent = isStale ? "--" : format(vehicle.gps?.lon, 7);
  $("#alt-msl-value").textContent = isStale ? "--" : `${format(vehicle.position?.altMsl, 1)} m`;
  $("#gps-value").textContent = isStale ? "--" : `${vehicle.gps?.fixType} / ${vehicle.gps?.satellites}`;
  $("#hdop-value").textContent = isStale ? "--" : format(vehicle.gps?.hdop, 2);
  $("#heartbeat-value").textContent = vehicle.heartbeatAge == null
    ? "--"
    : `${format(vehicle.heartbeatAge, 1)} s`;
  
  if (vehicle.nav && !vehicle.nav.valid) {
    $("#dist-home-value").parentElement.classList.add("stale-value");
  } else {
    $("#dist-home-value").parentElement.classList.remove("stale-value");
  }
  if (vehicle.gps?.lat != null && app.config?.home) {
    const dist = distanceMeters([vehicle.gps.lat, vehicle.gps.lon], [app.config.home.lat, app.config.home.lon]);
    $("#dist-home-value").textContent = dist > 1000 ? `${(dist / 1000).toFixed(2)} km` : `${dist.toFixed(1)} m`;
  } else {
    const el = $("#dist-home-value");
    if (el) el.textContent = "--";
  }
  $("#current-waypoint").textContent =
    `${vehicle.mission.current || 0} / ${vehicle.mission.total || 0}`;
  const operation = vehicle.operation || {
    status: "idle",
    message: "Chưa có tác vụ bay đang chạy",
  };
  const operationElement = $("#flight-action-status");
  operationElement.className = `flight-action-status ${operation.status || "idle"}`;
  operationElement.querySelector("span").textContent =
    operation.message || "Chưa có tác vụ bay đang chạy";
  $("#arm-button").textContent =
    app.pendingActions.has("/api/vehicle/arm") ? "ĐANG ARM…" : "ARM";
  $("#disarm-button").textContent =
    app.pendingActions.has("/api/vehicle/disarm") ? "ĐANG DISARM…" : "DISARM";
  $("#takeoff-button").textContent =
    (operation.status === "running" && operation.name === "takeoff")
    || app.pendingActions.has("/api/vehicle/takeoff")
      ? "ĐANG CẤT CÁNH…"
      : "CẤT CÁNH";
  if ($("#climb-button")) {
    $("#climb-button").textContent =
      (operation.status === "running" && operation.name === "altitude")
      || app.pendingActions.has("/api/vehicle/climb")
        ? "ĐANG CHỈNH ĐỘ CAO…"
        : "TỰ ĐỘNG LEO/HẠ";
  }
  if ($("#transition-fw-button")) {
    $("#transition-fw-button").textContent =
      (operation.status === "running" && operation.name === "transition_fw")
      || app.pendingActions.has("/api/vehicle/transition-fw")
        ? "ĐANG CHUYỂN…"
        : "CHUYỂN CÁNH BẰNG";
  }
  if (!["roll-slider", "pitch-slider", "throttle-slider", "yaw-slider"].includes(
    document.activeElement?.id,
  )) {
    setLocalRc([
      vehicle.rc?.roll ?? 1500,
      vehicle.rc?.pitch ?? 1500,
      vehicle.rc?.throttle ?? 1000,
      vehicle.rc?.yaw ?? 1500,
    ]);
  }
  renderStatusMessages(vehicle.statusMessages || []);
  updateDroneMap(vehicle);
}

function updateDroneMap(vehicle) {
  if (!vehicle.connected) return;
  const lat = Number(vehicle.gps.lat);
  const lon = Number(vehicle.gps.lon);
  if (!Number.isFinite(lat) || !Number.isFinite(lon)) return;
  
  [app.droneMarkerControl, app.droneMarkerMission].forEach(marker => {
    if(marker) {
      marker.setLatLng([lat, lon]);
      const element = marker.getElement()?.querySelector(".drone-marker");
      if (element) element.style.setProperty("--heading", `${vehicle.position.heading || 0}deg`);
    }
  });

  const last = app.trail.at(-1);
  if (!last || distanceMeters(last, [lat, lon]) > 500) {
    app.trail = [[lat, lon]];
  } else if (distanceMeters(last, [lat, lon]) > 1.5) {
    app.trail.push([lat, lon]);
    if (app.trail.length > 700) app.trail.shift();
  }
  if(app.trailLineControl) app.trailLineControl.setLatLngs(app.trail);
  if(app.trailLineMission) app.trailLineMission.setLatLngs(app.trail);

  if (vehicle.attitude?.yaw != null) app.lineHeading.setLatLngs([[lat, lon], getDest(lat, lon, vehicle.attitude.yaw, 300)]); else app.lineHeading.setLatLngs([]);
  if (vehicle.nav?.navBearing != null) app.lineTarget.setLatLngs([[lat, lon], getDest(lat, lon, vehicle.nav.navBearing, 250)]); else app.lineTarget.setLatLngs([]);
  if (vehicle.position?.heading != null) app.lineTrack.setLatLngs([[lat, lon], getDest(lat, lon, vehicle.position.heading, 400)]); else app.lineTrack.setLatLngs([]);
  if (vehicle.nav?.targetBearing != null) app.lineWp.setLatLngs([[lat, lon], getDest(lat, lon, vehicle.nav.targetBearing, 500)]); else app.lineWp.setLatLngs([]);
  
  if (app.followControl) {
    if (app.mapControl && app.mapControl.getBounds().pad(-0.18).contains([lat, lon]) === false) {
      app.mapControl.panTo([lat, lon], { animate: true, duration: 0.4 });
    }
  }
  if (app.followMission) {
    if (app.mapMission && app.mapMission.getBounds().pad(-0.18).contains([lat, lon]) === false) {
      app.mapMission.panTo([lat, lon], { animate: true, duration: 0.4 });
    }
  }
}

function getDest(lat, lon, brng, d) {
  const R = 6371e3;
  const rad = Math.PI / 180;
  const lat1 = lat * rad, lon1 = lon * rad, brngRad = brng * rad;
  const lat2 = Math.asin(Math.sin(lat1) * Math.cos(d/R) + Math.cos(lat1) * Math.sin(d/R) * Math.cos(brngRad));
  const lon2 = lon1 + Math.atan2(Math.sin(brngRad) * Math.sin(d/R) * Math.cos(lat1), Math.cos(d/R) - Math.sin(lat1) * Math.sin(lat2));
  return [lat2 / rad, lon2 / rad];
}

function distanceMeters(a, b) {
  const latScale = 111320;
  const lonScale = Math.cos((b[0] * Math.PI) / 180) * 111320;
  const dy = (b[0] - a[0]) * latScale;
  const dx = (b[1] - a[1]) * lonScale;
  return Math.hypot(dx, dy);
}

function renderStatusMessages(messages) {
  const container = $("#status-messages");
  if (!messages.length) {
    container.innerHTML = `<div class="empty-message">Chưa có STATUSTEXT.</div>`;
    return;
  }
  container.innerHTML = [...messages]
    .reverse()
    .slice(0, 18)
    .map(
      (message) => `
        <div class="status-message ${message.severity <= 3 ? "critical" : ""}">
          <time>${new Date(message.timestamp).toLocaleTimeString("vi-VN")}</time>
          ${escapeHtml(message.text)}
        </div>`,
    )
    .join("");
}

async function pollState() {
  if (app.pollBusy) return;
  app.pollBusy = true;
  const controller = new AbortController();
  const timeoutId = window.setTimeout(() => controller.abort(), 1500);
  try {
    const state = await api("/api/state", { signal: controller.signal });
    app.pollFailures = 0;
    app.state = state;
    
    if (app.transport.status === 'offline') {
      const banner = $("#disconnect-banner");
      if (banner) banner.classList.add("hidden");
      app.transport = { status: 'online', since: Date.now(), error: null };
    }
    
    updateSimulationState(state.simulation);
    updateVehicleState(state.vehicle);
  } catch (error) {
    // One missed poll is common on a busy remote desktop; require three
    // consecutive failures before presenting a real disconnect. A refused
    // connection is therefore detected in about 1.2 s; the 1.5 s request
    // timeout also prevents a hung request from blocking the poller forever.
    app.pollFailures += 1;
    if (app.pollFailures >= 3 && app.transport.status !== 'offline') {
      enterDisconnectedUi(error);
    }
    if (app.pollFailures >= 3) {
      $("#link-status").textContent = "Backend mất kết nối";
      $("#link-dot").classList.remove("online");
    }
  } finally {
    window.clearTimeout(timeoutId);
    app.pollBusy = false;
  }
}

async function pollLogs() {
  try {
    const payload = await api(`/api/logs?after=${app.lastLogId}&limit=250`);
    const container = $("#system-log");
    payload.entries.forEach((entry) => {
      app.lastLogId = Math.max(app.lastLogId, entry.id);
      const row = document.createElement("div");
      row.className = `log-line ${entry.level}`;
      row.innerHTML = `
        <time>${new Date(entry.timestamp).toLocaleTimeString("vi-VN")}</time>
        <span>${escapeHtml(entry.source)}</span>
        <p>${escapeHtml(entry.message)}</p>`;
      container.append(row);
    });
    while (container.children.length > 700) container.firstElementChild.remove();
    if ($("#auto-scroll-log").checked && payload.entries.length) {
      container.scrollTop = container.scrollHeight;
    }
  } catch {
    // State polling presents the connection status; avoid repeated log toasts.
  }
}

function setupGeneralUi() {
  $("#start-button").addEventListener("click", startSimulation);
  $("#stop-button").addEventListener("click", stopSimulation);
  $("#clear-log").addEventListener("click", () => {
    $("#system-log").innerHTML = "";
  });
  $("#refresh-flight-logs")?.addEventListener("click", refreshFlightLogs);

  $$(".tab-btn").forEach(btn => {
    btn.addEventListener("click", () => {
      $$(".tab-btn").forEach(b => b.classList.remove("active"));
      $$(".tab-content").forEach(c => c.classList.remove("active"));
      btn.classList.add("active");
      const targetId = btn.dataset.target;
      document.getElementById(targetId).classList.add("active");
      
      if (app.mapControl) window.setTimeout(() => app.mapControl.invalidateSize(), 50);
      if (app.mapMission) window.setTimeout(() => app.mapMission.invalidateSize(), 50);
    });
  });

  window.setInterval(() => {
    $("#clock").textContent = new Date().toLocaleTimeString("vi-VN", { hour12: false });
  }, 500);
}

async function initialise() {
  setupGeneralUi();
  initialiseEnvironmentSelector();
  setupVehicleControls();
  setupGuidedFab();
  buildCompassTape();
  setupMissionControls();
  setButtonAvailability(false);
  try {
    const config = await api("/api/config");
    populateConfig(config);
    initialiseMap();
    renderWaypoints();
    refreshFlightLogs();
    initExperimentTab();
    await pollState();
    await pollLogs();
    window.setInterval(pollState, 400);
    window.setInterval(pollLogs, 700);
  } catch (error) {
    toast(`Không thể khởi tạo dashboard: ${error.message}`, "error", 9000);
  }
}

window.addEventListener("DOMContentLoaded", initialise);


function renderPrearmChecklist(prearmData) {
  const list = $("#prearm-checklist");
  if (!prearmData) {
    if (list) list.innerHTML = `<div class="checklist-item watch"><span class="mark">–</span><span>Chưa có dữ liệu</span></div>`;
    return;
  }
  
  if (!prearmData.checks) {
    if (list) {
      const msg = prearmData.message || "Chưa có dữ liệu prearm";
      const mk = prearmData.ready ? "✓" : "✗";
      const st = prearmData.ready ? "pass" : "watch";
      list.innerHTML = `<div class="checklist-item ${st}"><span class="mark">${mk}</span><span>${escapeHtml(msg)}</span></div>`;
    }
    const cntBadge = $("#checklist-count");
    if (cntBadge) cntBadge.textContent = prearmData.message ? `1 mục` : `0 mục`;
    return;
  }
  
  let html = "";
  let count = 0;
  
  prearmData.checks.forEach((check) => {
    count++;
    let st = "watch";
    let mk = "–";
    if (check.status === "pass") { st = "pass"; mk = "✓"; }
    else if (check.status === "fail") { st = "watch"; mk = "✗"; }
    else if (check.status === "warn") { st = "watch"; mk = "⚠"; }
    
    html += `<div class="checklist-item ${st}"><span class="mark">${mk}</span><span>${escapeHtml(check.label)}</span></div>`;
  });
  
  if (list) list.innerHTML = html || `<div class="checklist-item watch"><span class="mark">–</span><span>Chưa có dữ liệu prearm</span></div>`;
  const cntBadge = $("#checklist-count");
  if(cntBadge) cntBadge.textContent = `${count} mục`;
}

// EXPERIMENTS TAB LOGIC

function initExperimentTab() {
  const tabBtn = document.querySelector('[data-target="tab-experiments"]');
  if (tabBtn) {
    tabBtn.addEventListener('click', () => {
      loadExperimentRuns();
      compareExperiments();
    });
  }

  const startBtn = document.getElementById('exp-start-button');
  if (startBtn) {
    startBtn.addEventListener('click', startExperiment);
  }

  const abortBtn = document.getElementById('exp-abort-button');
  if (abortBtn) {
    abortBtn.addEventListener('click', () => {
      if (experimentState.activeRun) {
        abortExperiment(experimentState.activeRun.id);
      }
    });
  }

  const filterStatus = document.getElementById('exp-filter-status');
  if (filterStatus) {
    filterStatus.addEventListener('change', (e) => {
      experimentState.filters.status = e.target.value;
      renderExperimentRuns(experimentState.runs);
    });
  }

  const compareBtn = document.getElementById('exp-compare-button');
  if (compareBtn) {
    compareBtn.addEventListener('click', compareExperiments);
  }

  window.setInterval(() => {
    if (document.getElementById('tab-experiments')?.classList.contains('active')) {
      loadExperimentRuns();
    }
  }, 2000);
}

async function loadExperimentRuns() {
  try {
    const data = await api('/api/v1/experiments');
    experimentState.runs = data.runs || [];
    if (data.activeRun) {
      experimentState.activeRun = data.activeRun;
      updateActiveRunStatus(data.activeRun);
    } else {
      experimentState.activeRun = null;
      updateActiveRunStatus(null);
    }
    renderExperimentRuns(experimentState.runs);
  } catch (err) {
    // Silent fail on polling to avoid spam
  }
}

async function startExperiment() {
  const profile = document.getElementById('exp-profile-select').value;
  const reps = parseInt(document.getElementById('exp-reps-input').value, 10);
  const seedInput = document.getElementById('exp-seed-input').value;
  const seed = seedInput ? parseInt(seedInput, 10) : null;
  
  const confirmed = await askConfirm('Xác nhận', `Bạn có chắc chắn muốn bắt đầu chiến dịch ${profile} với ${reps} lần lặp?`);
  if (!confirmed) return;
  
  try {
    document.getElementById('exp-start-button').disabled = true;
    await api('/api/v1/experiments', {
      method: 'POST',
      body: JSON.stringify({ profile, repetitions: reps, seed })
    });
    toast('Đã gửi yêu cầu tạo chiến dịch thí nghiệm', 'success');
    loadExperimentRuns();
  } catch (err) {
    toast(`Lỗi bắt đầu thí nghiệm: ${err.message}`, 'error');
  } finally {
    document.getElementById('exp-start-button').disabled = false;
  }
}

async function abortExperiment(runId) {
  const confirmed = await askConfirm('Xác nhận', `Hủy run hiện tại (${runId})?`);
  if (!confirmed) return;
  
  try {
    await api(`/api/v1/experiments/${runId}/abort`, {
      method: 'POST',
      body: JSON.stringify({})
    });
    toast('Đã gửi lệnh ABORT run.', 'success');
    loadExperimentRuns();
  } catch (err) {
    toast(`Lỗi hủy run: ${err.message}`, 'error');
  }
}

window.viewRunDetail = async function(runId) {
  try {
    const run = await api(`/api/v1/experiments/${runId}`);
    toast(`Tải chi tiết run ${runId} thành công.`, 'success');
  } catch (err) {
    toast(`Lỗi tải chi tiết: ${err.message}`, 'error');
  }
};

async function compareExperiments() {
  try {
    const data = await api('/api/v1/experiments/compare');
    renderComparisonTable(data.comparison || []);
  } catch (err) {
    toast(`Lỗi tải dữ liệu so sánh: ${err.message}`, 'error');
  }
}

function renderExperimentRuns(runs) {
  const tbody = document.getElementById('exp-runs-body');
  if (!tbody) return;
  
  let filtered = runs;
  if (experimentState.filters.status && experimentState.filters.status !== 'ALL') {
    filtered = runs.filter(r => r.verdict === experimentState.filters.status);
  }
  
  if (filtered.length === 0) {
    tbody.innerHTML = '<tr class="empty-row"><td colspan="6">Không có dữ liệu phù hợp</td></tr>';
    return;
  }
  
  tbody.innerHTML = filtered.map(run => {
    let verdictBadge = '';
    if (run.verdict === 'PASS') verdictBadge = '<span class="mini-badge" style="background:var(--success);color:#fff">PASS</span>';
    else if (run.verdict === 'FAIL') verdictBadge = '<span class="mini-badge" style="background:var(--danger);color:#fff">FAIL</span>';
    else verdictBadge = `<span class="mini-badge">${escapeHtml(run.verdict || 'RUNNING')}</span>`;
    
    return `
      <tr>
        <td>${escapeHtml(run.id)}</td>
        <td>${escapeHtml(run.case || '')}</td>
        <td>${escapeHtml(run.phase || '')}</td>
        <td>${verdictBadge}</td>
        <td>${escapeHtml(run.timestamp || '')}</td>
        <td>
          <button class="text-button" onclick="viewRunDetail('${escapeHtml(run.id)}')">Chi tiết</button>
        </td>
      </tr>
    `;
  }).join('');
}

function renderComparisonTable(data) {
  const tbody = document.getElementById('exp-compare-body');
  if (!tbody) return;
  
  if (!data || data.length === 0) {
    tbody.innerHTML = '<tr class="empty-row"><td colspan="6">Chưa có dữ liệu so sánh</td></tr>';
    return;
  }
  
  tbody.innerHTML = data.map(row => {
    let verdictBadge = '';
    if (row.verdict === 'PASS') verdictBadge = '<span class="mini-badge" style="background:var(--success);color:#fff">PASS</span>';
    else if (row.verdict === 'FAIL') verdictBadge = '<span class="mini-badge" style="background:var(--danger);color:#fff">FAIL</span>';
    else verdictBadge = `<span class="mini-badge">${escapeHtml(row.verdict || '')}</span>`;
    
    return `
      <tr>
        <td>${escapeHtml(row.case || '')}</td>
        <td>${escapeHtml(row.rep || '')}</td>
        <td>${escapeHtml(row.hover_rmse || '')}</td>
        <td>${escapeHtml(row.transition_loss || '')}</td>
        <td>${escapeHtml(row.landing_error || '')}</td>
        <td>${verdictBadge}</td>
      </tr>
    `;
  }).join('');
}

function updateActiveRunStatus(run) {
  const badge = document.getElementById('exp-active-badge');
  const text = document.getElementById('exp-active-text');
  const abortBtn = document.getElementById('exp-abort-button');
  
  if (!badge || !text || !abortBtn) return;
  
  if (!run) {
    badge.textContent = 'NONE';
    badge.className = 'mini-badge';
    text.textContent = 'Chưa có thí nghiệm đang chạy';
    abortBtn.disabled = true;
  } else {
    badge.textContent = escapeHtml(run.phase || 'RUNNING');
    badge.className = 'mini-badge neutral';
    text.textContent = `Run ${escapeHtml(run.id)} đang chạy...`;
    abortBtn.disabled = false;
  }
}
