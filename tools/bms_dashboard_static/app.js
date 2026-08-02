"use strict";

const DEVICE_LABELS = { daly: "Daly", seplos: "Seplos", jk: "JK BMS" };
const state = { data: null, selected: "daly", toastTimer: null };

const $ = (selector) => document.querySelector(selector);
const esc = (value) => String(value ?? "—").replace(/[&<>'"]/g, (char) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", "'": "&#39;", '"': "&quot;" })[char]);
const num = (value, digits = 1) => Number.isFinite(Number(value)) ? Number(value).toFixed(digits) : "—";
const clamp = (value, min, max) => Math.min(max, Math.max(min, value));

function toast(message, error = false) {
  const element = $("#toast");
  element.textContent = message;
  element.className = `toast show${error ? " error" : ""}`;
  clearTimeout(state.toastTimer);
  state.toastTimer = setTimeout(() => { element.className = "toast"; }, 4500);
}

async function api(path, options = {}) {
  const response = await fetch(path, { cache: "no-store", ...options });
  if (!response.ok) {
    const message = await response.text();
    throw new Error(message || `${response.status} ${response.statusText}`);
  }
  return response.json();
}

function deviceMode(telemetry) {
  const current = Number(telemetry.current);
  if (!Number.isFinite(current) || Math.abs(current) < 0.1) return "Idle";
  return telemetry.battery_charging ? "Charging" : "Discharging";
}

function temperatureValues(telemetry) {
  return (telemetry.temp_values || []).map((item) => Number(item?.value)).filter(Number.isFinite);
}

function cellStrip(cells) {
  if (!Array.isArray(cells) || !cells.length) return "";
  const min = Math.min(...cells);
  const max = Math.max(...cells);
  const span = Math.max(max - min, .001);
  return `<div class="cell-strip" title="Cells ${num(min, 3)}–${num(max, 3)} V">${cells.map((cell) => {
    const position = (cell - min) / span;
    const hue = 28 + position * 122;
    return `<i style="background:hsla(${hue},72%,56%,.28);border-color:hsla(${hue},72%,56%,.32)"></i>`;
  }).join("")}</div>`;
}

function deviceCard(alias, device) {
  const t = device.telemetry || {};
  const cells = t.cell_voltages || [];
  const temperatures = temperatureValues(t);
  const maxTemp = temperatures.length ? Math.max(...temperatures) : t.temperature;
  const soc = clamp(Number(t.battery_level) || 0, 0, 100);
  const name = device.info?.model || device.info?.hw_version || device.advertised_name;
  const status = device.online ? "Online" : (device.updating ? "Connecting" : "Offline");
  return `
    <article class="device-card ${state.selected === alias ? "selected" : ""}" data-device="${alias}" tabindex="0" role="button" aria-label="${esc(DEVICE_LABELS[alias])} details">
      <div class="device-head">
        <div><span class="eyebrow">${esc(device.address)}</span><h3>${esc(DEVICE_LABELS[alias])}</h3><span class="device-subtitle">${esc(name)}</span></div>
        <span class="device-status ${device.online ? "online" : ""}"><span class="dot"></span>${status}</span>
      </div>
      <div class="primary-metrics">
        <div class="primary-metric"><span>Voltage</span><strong>${num(t.voltage, 2)}</strong> <small>V</small></div>
        <div class="primary-metric"><span>Current</span><strong>${num(t.current, 2)}</strong> <small>A</small></div>
      </div>
      <div class="mini-metrics">
        <div class="mini-metric"><span>Power</span><strong>${num(t.power, 0)} W</strong></div>
        <div class="mini-metric"><span>Temperature</span><strong>${num(maxTemp, 1)} °C</strong></div>
        <div class="mini-metric"><span>Mode</span><strong>${deviceMode(t)}</strong></div>
      </div>
      <div class="soc-row"><span>State of charge</span><strong>${num(t.battery_level, 1)}%</strong></div>
      <div class="progress"><i style="width:${soc}%"></i></div>
      ${cellStrip(cells)}
      ${device.error ? `<div class="device-error">${esc(device.error)}</div>` : ""}
    </article>`;
}

function renderHeader(data) {
  const devices = Object.values(data.devices || {});
  const online = devices.filter((device) => device.online).length;
  const totalPower = devices.reduce((sum, device) => sum + (Number(device.telemetry?.power) || 0), 0);
  $("#online-count").textContent = online;
  $("#total-power").textContent = `${num(totalPower, 0)} W`;
  $("#control-state").textContent = data.control_enabled ? "Protected" : "Disabled";
  $("#poll-interval").textContent = `${data.poll_interval_seconds}s`;
  $("#last-update").textContent = data.last_poll_finished ? `Last cycle: ${new Date(data.last_poll_finished).toLocaleString("en-GB")}` : "Initializing Bluetooth connections…";
  const poll = $("#poll-state");
  poll.className = `status-pill ${data.polling ? "busy" : online ? "live" : ""}`;
  poll.innerHTML = `<span class="dot"></span>${data.polling ? "Reading Bluetooth" : `${online}/3 online`}`;
}

function renderCards(data) {
  $("#device-grid").innerHTML = Object.entries(data.devices).map(([alias, device]) => deviceCard(alias, device)).join("");
  document.querySelectorAll(".device-card").forEach((card) => {
    const select = () => { state.selected = card.dataset.device; renderAll(); };
    card.addEventListener("click", select);
    card.addEventListener("keydown", (event) => { if (event.key === "Enter" || event.key === " ") select(); });
  });
}

function dl(items) {
  return items.map(([key, value]) => `<div><dt>${esc(key)}</dt><dd>${esc(value)}</dd></div>`).join("");
}

function renderChart(history) {
  const svg = $("#history-chart");
  const samples = (history || []).filter((item) => Number.isFinite(Number(item.current)));
  if (samples.length < 2) {
    svg.innerHTML = `<text x="400" y="125" class="chart-empty">History appears after the first two readings</text>`;
    $("#chart-value").textContent = samples.length ? `${num(samples.at(-1).current, 2)} A` : "—";
    return;
  }
  const width = 800, height = 240, pad = 22;
  const values = samples.map((item) => Number(item.current));
  let min = Math.min(...values), max = Math.max(...values);
  if (Math.abs(max - min) < .2) { min -= .1; max += .1; }
  const x = (index) => pad + index * (width - pad * 2) / (values.length - 1);
  const y = (value) => height - pad - (value - min) * (height - pad * 2) / (max - min);
  const points = values.map((value, index) => `${x(index)},${y(value)}`).join(" ");
  const area = `${pad},${height - pad} ${points} ${width - pad},${height - pad}`;
  svg.innerHTML = `
    <defs><linearGradient id="areaGradient" x1="0" y1="0" x2="0" y2="1"><stop offset="0" stop-color="#4de0a4" stop-opacity=".22"/><stop offset="1" stop-color="#4de0a4" stop-opacity="0"/></linearGradient></defs>
    ${[.25,.5,.75].map((p) => `<line class="chart-grid" x1="${pad}" y1="${height*p}" x2="${width-pad}" y2="${height*p}"/>`).join("")}
    <polygon class="chart-area" points="${area}"/><polyline class="chart-line" points="${points}"/>`;
  $("#chart-value").textContent = `${num(values.at(-1), 2)} A`;
  $("#chart-range").textContent = `${num(min, 2)}…${num(max, 2)} A`;
}

function renderDetails(data) {
  const device = data.devices[state.selected] || Object.values(data.devices)[0];
  if (!device) return;
  const t = device.telemetry || {};
  const cells = t.cell_voltages || [];
  const label = DEVICE_LABELS[state.selected];
  $("#detail-title").textContent = `${label} · ${device.info?.model || device.advertised_name}`;
  $("#device-selector").innerHTML = Object.keys(data.devices).map((alias) => `<button class="${state.selected === alias ? "active" : ""}" data-select="${alias}" type="button">${esc(DEVICE_LABELS[alias])}</button>`).join("");
  document.querySelectorAll("[data-select]").forEach((button) => button.addEventListener("click", () => { state.selected = button.dataset.select; renderAll(); }));
  const deltaMv = cells.length ? (Math.max(...cells) - Math.min(...cells)) * 1000 : null;
  $("#identity-list").innerHTML = dl([
    ["Status", device.online ? "Online" : "Offline"],
    ["BLE address", device.address],
    ["Model", device.info?.model || device.info?.hw_version || "—"],
    ["Firmware", device.info?.sw_version || device.info?.software_version || "—"],
    ["Cell count", t.cell_count ?? cells.length ?? "—"],
    ["Cell delta", deltaMv === null ? "—" : `${num(deltaMv, 1)} mV`],
    ["Cycles", t.cycles ?? "—"],
    ["Last reading", device.last_success ? new Date(device.last_success).toLocaleString("en-GB") : "—"],
  ]);
  renderChart(data.history[state.selected]);
  renderCellVoltages(cells);
}

function renderCellVoltages(cells) {
  const values = (Array.isArray(cells) ? cells : [])
    .map((value, index) => ({ index, value: Number(value) }))
    .filter((cell) => Number.isFinite(cell.value));
  if (!values.length) {
    $("#cell-summary").textContent = "No cell telemetry";
    $("#cell-voltage-grid").innerHTML = `<p class="cell-empty">Individual cell voltages are not available for this BMS.</p>`;
    return;
  }
  const minimum = Math.min(...values.map((cell) => cell.value));
  const maximum = Math.max(...values.map((cell) => cell.value));
  $("#cell-summary").textContent = `${values.length} cells · ${num(minimum, 3)}–${num(maximum, 3)} V · Δ ${num((maximum - minimum) * 1000, 1)} mV`;
  $("#cell-voltage-grid").innerHTML = values.map((cell) => {
    const classes = ["cell-voltage"];
    if (cell.value === minimum) classes.push("minimum");
    if (cell.value === maximum) classes.push("maximum");
    return `<div class="${classes.join(" ")}"><span>Cell ${String(cell.index + 1).padStart(2, "0")}</span><strong>${num(cell.value, 3)} <small>V</small></strong></div>`;
  }).join("");
}

function jkData(data) { return data.protocols?.jk?.data || null; }
function seplosData(data) { return data.protocols?.seplos?.data || null; }

function updateJkOptions() {
  if (!state.data) return;
  const jk = jkData(state.data);
  const interfaceName = $("#jk-interface").value;
  const info = jk?.interfaces?.[interfaceName];
  $("#jk-protocol").innerHTML = (info?.enabled_protocols || []).map((item) => `<option value="${item.index}" ${item.index === info.selected.index ? "selected" : ""}>${item.index} · ${esc(item.name)}</option>`).join("");
}

function renderProtocols(data) {
  const jk = jkData(data);
  const interfaces = jk?.interfaces || {};
  $("#jk-current-protocols").innerHTML = Object.entries(interfaces).map(([name, item]) => `<div><span>${esc(name)}</span><strong>${esc(item.selected.index)} · ${esc(item.selected.name)}</strong></div>`).join("") || `<p class="note">JK protocol data is not available yet.</p>`;
  const previousInterface = $("#jk-interface").value;
  $("#jk-interface").innerHTML = Object.keys(interfaces).map((name) => `<option value="${name}" ${name === previousInterface ? "selected" : ""}>${name.toUpperCase()}</option>`).join("");
  updateJkOptions();
  const seplosDevice = seplosData(data);
  const seplos = seplosDevice?.inverter_protocol;
  $("#seplos-protocol").innerHTML = seplos ? dl([
    ["Profile", seplos.selector_profile || "unknown"], ["Protocol", seplos.protocol_name], ["Version", seplos.protocol_version], ["Baud rate", `${seplos.baud_rate} bps`], ["Inverter", seplos.inverter_name], ["Selector", seplos.selector_index]
  ]) : dl([["Status", data.protocols?.seplos?.error || "Waiting for data"]]);
  const previousProfile = $("#seplos-profile").value;
  $("#seplos-profile").innerHTML = (data.seplos_protocol_profiles || []).map((profile) => `<option value="${esc(profile.name)}" ${profile.name === (previousProfile || seplos?.selector_profile) ? "selected" : ""}>${profile.index} · ${esc(profile.protocol_name)}</option>`).join("");
  $("#jk-protocol-form button[type=submit]").disabled = !data.control_enabled || !jk;
  $("#seplos-protocol-form button[type=submit]").disabled = !data.control_enabled || !seplosDevice;
}

function renderAll() {
  if (!state.data) return;
  renderHeader(state.data);
  renderCards(state.data);
  renderDetails(state.data);
  renderProtocols(state.data);
}

async function loadStatus(showError = false) {
  try {
    state.data = await api("/api/status");
    renderAll();
  } catch (error) {
    if (showError) toast(`Cannot read the dashboard: ${error.message}`, true);
  }
}

$("#refresh-button").addEventListener("click", async () => {
  try {
    const result = await api("/api/refresh", { method: "POST" });
    toast(result.accepted ? "Bluetooth refresh scheduled." : "A recent refresh is already pending.");
    await loadStatus();
  } catch (error) { toast(error.message, true); }
});

document.querySelectorAll(".tab").forEach((button) => button.addEventListener("click", () => {
  document.querySelectorAll(".tab").forEach((item) => item.classList.toggle("active", item === button));
  document.querySelectorAll(".tab-panel").forEach((panel) => panel.classList.toggle("active", panel.id === `${button.dataset.tab}-panel`));
}));

$("#jk-interface").addEventListener("change", updateJkOptions);
$("#jk-protocol-form").addEventListener("submit", async (event) => {
  event.preventDefault();
  const jk = jkData(state.data);
  if (!jk) return;
  if ($("#jk-confirmation").value.trim() !== jk.model) {
    toast(`For confirmation, type exactly ${jk.model}`, true);
    return;
  }
  const button = event.submitter;
  button.disabled = true;
  button.textContent = "Applying…";
  try {
    const result = await api("/api/jk/protocol", {
      method: "POST",
      headers: { "Content-Type": "application/json", "X-Control-Token": $("#control-token").value },
      body: JSON.stringify({ interface: $("#jk-interface").value, protocol: $("#jk-protocol").value, confirmation: jk.address }),
    });
    toast(result.result.changed ? "The JK protocol was changed and verified by read-back." : "The selected protocol was already active.");
    $("#jk-confirmation").value = "";
    await loadStatus(true);
  } catch (error) { toast(`Command rejected: ${error.message}`, true); }
  finally { button.disabled = false; button.textContent = "Apply JK protocol"; }
});

$("#seplos-protocol-form").addEventListener("submit", async (event) => {
  event.preventDefault();
  const seplos = seplosData(state.data);
  if (!seplos) return;
  const serial = seplos.identity?.bms_serial_number;
  if (!serial || $("#seplos-confirmation").value.trim() !== serial) {
    toast(`For confirmation, type exactly ${serial || "the BMS serial number"}`, true);
    return;
  }
  const button = event.submitter;
  button.disabled = true;
  button.textContent = "Applying…";
  try {
    const result = await api("/api/seplos/protocol", {
      method: "POST",
      headers: { "Content-Type": "application/json", "X-Control-Token": $("#seplos-control-token").value },
      body: JSON.stringify({ profile: $("#seplos-profile").value, confirmation: serial }),
    });
    toast(result.result.changed ? "The Seplos protocol was changed and verified by read-back." : "The selected protocol was already active.");
    $("#seplos-confirmation").value = "";
    await loadStatus(true);
  } catch (error) { toast(`Command rejected: ${error.message}`, true); }
  finally { button.disabled = false; button.textContent = "Apply Seplos protocol"; }
});

loadStatus(true);
setInterval(() => loadStatus(false), 3000);
