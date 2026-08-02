"use strict";

const DEVICE_LABELS = { daly: "Daly", seplos: "Seplos", jk: "JK BMS" };
const INVERTER_CONFIG_REFRESH_MS = 30_000;
const BMS_CONFIG_REFRESH_MS = 30_000;
const state = {
  data: null,
  selected: "daly",
  selectedInverter: "inverter-anenji",
  inverterConfig: null,
  inverterConfigLoading: false,
  inverterConfigWriting: false,
  inverterConfigTimer: null,
  selectedBms: "daly",
  bmsConfig: null,
  bmsConfigLoading: false,
  bmsConfigWriting: false,
  bmsConfigTimer: null,
  toastTimer: null,
};

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

function inverterMeasurement(value, digits, unit) {
  if (value === null || value === undefined || !Number.isFinite(Number(value))) return "—";
  return `${Number(value).toFixed(digits)}${unit ? ` ${unit}` : ""}`;
}

function inverterMetric(label, value, digits = 1, unit = "") {
  return `<div class="inverter-metric"><span>${esc(label)}</span><strong>${esc(inverterMeasurement(value, digits, unit))}</strong></div>`;
}

function inverterCard(device) {
  const t = device.telemetry || {};
  const soc = clamp(Number(t.battery_soc_pct) || 0, 0, 100);
  const mode = t.working_mode || "Unknown";
  const protocol = t.protocol || device.configured_protocol;
  const age = Number.isFinite(Number(device.age_seconds)) ? `${Math.round(Number(device.age_seconds))}s ago` : "No sample";
  const lastSample = device.last_sample ? new Date(device.last_sample).toLocaleString("en-GB") : "—";
  return `
    <article class="inverter-card ${device.online ? "online" : "offline"}" data-inverter="${esc(device.id)}">
      <div class="device-head">
        <div>
          <span class="eyebrow">${esc(device.source_ip || device.configured_ip)}</span>
          <h3>${esc(device.name)}</h3>
          <span class="device-subtitle">${esc(protocol)}</span>
        </div>
        <span class="device-status ${device.online ? "online" : ""}"><span class="dot"></span>${device.online ? "Online" : device.stale ? "Stale" : "Offline"}</span>
      </div>
      <div class="inverter-primary">
        <div><span>Output power</span><strong>${esc(inverterMeasurement(t.output_power_w, 0, "W"))}</strong></div>
        <div><span>Operating mode</span><strong>${esc(mode)}</strong></div>
      </div>
      <div class="inverter-section">
        <h4>AC output and grid</h4>
        <div class="inverter-metrics">
          ${inverterMetric("Output voltage", t.output_voltage_v, 1, "V")}
          ${inverterMetric("Output current", t.output_current_a, 1, "A")}
          ${inverterMetric("Output frequency", t.output_frequency_hz, 2, "Hz")}
          ${inverterMetric("Apparent power", t.output_apparent_power_va, 0, "VA")}
          ${inverterMetric("Load", t.load_pct, 0, "%")}
          ${inverterMetric("Grid voltage", t.grid_voltage_v, 1, "V")}
          ${inverterMetric("Grid frequency", t.grid_frequency_hz, 2, "Hz")}
          ${inverterMetric("Grid power", t.grid_power_w, 0, "W")}
        </div>
      </div>
      <div class="inverter-section">
        <div class="section-title"><h4>Battery</h4><span>${esc(inverterMeasurement(t.battery_soc_pct, 0, "%"))}</span></div>
        <div class="progress"><i style="width:${soc}%"></i></div>
        <div class="inverter-metrics compact">
          ${inverterMetric("Voltage", t.battery_voltage_v, 2, "V")}
          ${inverterMetric("Current", t.battery_current_a, 2, "A")}
          ${inverterMetric("Power", t.battery_power_w, 0, "W")}
          ${inverterMetric("Charge current", t.battery_charge_current_a, 1, "A")}
          ${inverterMetric("Discharge current", t.battery_discharge_current_a, 1, "A")}
        </div>
      </div>
      <div class="inverter-section">
        <h4>Solar and temperatures</h4>
        <div class="inverter-metrics compact">
          ${inverterMetric("PV voltage", t.pv_voltage_v, 1, "V")}
          ${inverterMetric("PV current", t.pv_current_a, 1, "A")}
          ${inverterMetric("PV power", t.pv_power_w, 0, "W")}
          ${inverterMetric("PV 2 power", t.pv2_power_w, 0, "W")}
          ${inverterMetric("Inverter temp.", t.inverter_temperature_c, 1, "°C")}
          ${inverterMetric("DCDC temp.", t.dcdc_temperature_c, 1, "°C")}
          ${inverterMetric("PV temp.", t.pv_temperature_c, 1, "°C")}
        </div>
      </div>
      <dl class="inverter-meta info-list">
        ${dl([
          ["Last sample", lastSample],
          ["Sample age", age],
          ["Wi-Fi MAC", device.mac],
          ["Linked ESP32", device.linked_board_id || "—"],
          ["Warning bits", t.warning_bits || "None reported"],
        ])}
      </dl>
      ${device.error ? `<div class="device-error">${esc(device.error)}</div>` : ""}
    </article>`;
}

function renderInverters(data) {
  const monitor = data.inverters || { available: false, devices: {}, error: "Waiting for inverter data" };
  const devices = Object.values(monitor.devices || {});
  const databaseState = $("#inverter-database-state");
  const online = devices.filter((device) => device.online).length;
  databaseState.className = `status-pill ${monitor.available ? (online ? "live" : "") : ""}`;
  databaseState.innerHTML = `<span class="dot"></span>${monitor.available ? `SQLite v${monitor.schema_version} · ${online}/${devices.length} online` : "SQLite unavailable"}`;
  if (!monitor.available) {
    $("#inverter-grid").innerHTML = `<article class="inverter-empty"><strong>Inverter telemetry is unavailable</strong><p>${esc(monitor.error || "The local database could not be read.")}</p></article>`;
    return;
  }
  if (!devices.length) {
    $("#inverter-grid").innerHTML = `<article class="inverter-empty"><strong>No inverters configured</strong><p>The collector database does not contain inverter inventory yet.</p></article>`;
    return;
  }
  $("#inverter-grid").innerHTML = devices.map(inverterCard).join("");
}

function setInverterConfigStatus(message, mode = "") {
  const element = $("#inverter-config-status");
  element.className = `status-pill ${mode}`.trim();
  element.innerHTML = `<span class="dot"></span>${esc(message)}`;
}

function renderInverterControlShell(data) {
  const devices = Object.values(data.inverters?.devices || {});
  if (devices.length && !devices.some((device) => device.id === state.selectedInverter)) {
    state.selectedInverter = devices[0].id;
  }
  const selector = $("#inverter-control-selector");
  const options = devices.map((device) => `<option value="${esc(device.id)}" ${device.id === state.selectedInverter ? "selected" : ""}>${esc(device.name)} · ${esc(device.configured_ip)}</option>`).join("");
  if (selector.innerHTML !== options) selector.innerHTML = options;
  selector.disabled = !devices.length || state.inverterConfigLoading || state.inverterConfigWriting;
}

function configurationValueLabel(item) {
  const option = (item.options || []).find((candidate) => String(candidate.value) === String(item.value));
  if (option) return option.label;
  if (item.display_value !== undefined) return item.display_value;
  return `${item.value ?? "—"}${item.unit ? ` ${item.unit}` : ""}`;
}

function configurationInput(item) {
  const id = `inverter-setting-${item.key}`;
  if (!item.writable) {
    return `<div class="setting-readonly">${esc(configurationValueLabel(item))}</div>`;
  }
  if (item.type === "action") {
    return `<button class="button danger setting-action" type="button" data-apply-setting="${esc(item.key)}">Execute action</button>`;
  }
  if (item.type === "select") {
    return `<select id="${esc(id)}" data-setting-input="${esc(item.key)}">${(item.options || []).map((option) => `<option value="${esc(option.value)}" ${String(option.value) === String(item.value) ? "selected" : ""}>${esc(option.label)}</option>`).join("")}</select>`;
  }
  const attributes = [
    `id="${esc(id)}"`, `data-setting-input="${esc(item.key)}"`,
    `type="${item.type === "text" ? "text" : "number"}"`, `value="${esc(item.value)}"`,
    item.minimum !== undefined ? `min="${esc(item.minimum)}"` : "",
    item.maximum !== undefined ? `max="${esc(item.maximum)}"` : "",
    item.step !== undefined ? `step="${esc(item.step)}"` : "",
  ].filter(Boolean).join(" ");
  return `<div class="setting-input-wrap"><input ${attributes}>${item.unit ? `<span>${esc(item.unit)}</span>` : ""}</div>`;
}

function renderInverterConfiguration(configuration) {
  const identity = configuration.identity || {};
  const ratings = configuration.ratings || {};
  const inconsistencyCount = Object.keys(configuration.raw?.inconsistencies || {}).length;
  $("#inverter-config-identity").innerHTML = `
    <article><span>Inverter</span><strong>${esc(configuration.inverter?.name || configuration.protocol)}</strong></article>
    <article><span>Serial number</span><strong>${esc(identity.serial)}</strong></article>
    <article><span>Firmware</span><strong>${esc(identity.firmware || "—")}</strong></article>
    <article><span>Rated power</span><strong>${esc(ratings.rated_power_w !== undefined ? `${ratings.rated_power_w} W` : ratings.output_power_w !== undefined ? `${ratings.output_power_w} W` : "—")}</strong></article>
    <article><span>Protocol map</span><strong>${esc(configuration.protocol)}</strong></article>
    <article><span>Read-only mismatches</span><strong>${inconsistencyCount}</strong></article>`;
  $("#inverter-config-content").innerHTML = (configuration.groups || []).map((group) => `
    <article class="inverter-config-group">
      <div class="card-title"><div><span>Live settings</span><strong>${esc(group.title)}</strong></div></div>
      <div class="inverter-settings-list">${(group.settings || []).map((item) => `
        <div class="inverter-setting ${item.writable ? "" : "readonly"} ${item.critical ? "critical" : ""}">
          <div class="setting-copy">
            <div><strong>${esc(item.label)}</strong>${item.critical ? `<span class="setting-badge">critical</span>` : ""}</div>
            <span>Current: ${esc(configurationValueLabel(item))}</span>
            ${item.description ? `<p>${esc(item.description)}</p>` : ""}
          </div>
          <div class="setting-control">
            ${configurationInput(item)}
            ${item.writable && item.type !== "action" ? `<button class="button secondary" type="button" data-apply-setting="${esc(item.key)}">Apply</button>` : ""}
          </div>
        </div>`).join("")}</div>
    </article>`).join("");
  $("#inverter-config-backup").disabled = false;
  setInverterConfigStatus(`Updated ${new Date().toLocaleTimeString("en-GB")} · every 30s`, "live");
}

function findConfigurationItem(key) {
  for (const group of state.inverterConfig?.groups || []) {
    const item = (group.settings || []).find((candidate) => candidate.key === key);
    if (item) return item;
  }
  return null;
}

async function loadInverterConfiguration() {
  if (!state.data || !state.selectedInverter || state.inverterConfigLoading || state.inverterConfigWriting) return;
  state.inverterConfigLoading = true;
  renderInverterControlShell(state.data);
  setInverterConfigStatus("Reading live configuration…", "busy");
  try {
    const result = await api(`/api/inverters/${encodeURIComponent(state.selectedInverter)}/configuration`);
    state.inverterConfig = result.configuration;
    renderInverterConfiguration(state.inverterConfig);
  } catch (error) {
    setInverterConfigStatus("Configuration read failed");
    if (!state.inverterConfig) {
      $("#inverter-config-content").innerHTML = `<article class="inverter-empty"><strong>Cannot read live configuration</strong><p>${esc(error.message)}. The dashboard will retry automatically.</p></article>`;
    }
  } finally {
    state.inverterConfigLoading = false;
    renderInverterControlShell(state.data);
  }
}

function inverterControlIsActive() {
  return $("#inverter-control-panel").classList.contains("active") && document.visibilityState === "visible";
}

function stopInverterConfigPolling() {
  clearInterval(state.inverterConfigTimer);
  state.inverterConfigTimer = null;
}

function startInverterConfigPolling() {
  stopInverterConfigPolling();
  if (!inverterControlIsActive()) return;
  loadInverterConfiguration();
  state.inverterConfigTimer = setInterval(() => {
    const active = document.activeElement;
    const editing = $("#inverter-config-content").contains(active)
      && (active.matches("input") || active.matches("select"));
    if (inverterControlIsActive() && !editing) loadInverterConfiguration();
  }, INVERTER_CONFIG_REFRESH_MS);
}

async function applyInverterSetting(key, button) {
  const item = findConfigurationItem(key);
  if (!item || !item.writable) return;
  const serial = state.inverterConfig?.identity?.serial;
  if (!serial) {
    toast("The inverter serial number is unavailable; reload before writing.", true);
    return;
  }
  const input = document.querySelector(`[data-setting-input="${CSS.escape(key)}"]`);
  const requested = item.type === "action" ? 1 : input?.value;
  const current = item.type === "action" ? "one-shot action" : configurationValueLabel(item);
  const warning = item.critical ? "\n\nThis is a critical setting and can affect loads or battery safety." : "";
  if (!window.confirm(`Apply “${item.label}” to ${state.inverterConfig.inverter?.name || "the inverter"}?\n\nCurrent: ${current}\nRequested: ${requested}${warning}`)) return;
  const originalText = button.textContent;
  state.inverterConfigWriting = true;
  renderInverterControlShell(state.data);
  button.disabled = true;
  button.textContent = "Applying…";
  setInverterConfigStatus(`Writing ${item.label}…`, "busy");
  try {
    const response = await api(`/api/inverters/${encodeURIComponent(state.selectedInverter)}/setting`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ setting: key, value: requested, confirmation: serial }),
    });
    state.inverterConfig = response.result.after_configuration;
    renderInverterConfiguration(state.inverterConfig);
    toast(response.result.written
      ? (response.result.verified ? "Setting written and verified by read-back." : "Action acknowledged by the inverter.")
      : "The requested value was already active; no write was needed.");
  } catch (error) {
    setInverterConfigStatus("Write failed; reload before retrying");
    toast(`Command rejected: ${error.message}`, true);
  } finally {
    state.inverterConfigWriting = false;
    renderInverterControlShell(state.data);
    button.disabled = false;
    button.textContent = originalText;
  }
}

function setBmsConfigStatus(message, mode = "") {
  const element = $("#bms-config-status");
  element.className = `status-pill ${mode}`.trim();
  element.innerHTML = `<span class="dot"></span>${esc(message)}`;
}

function renderBmsControlShell() {
  const selector = $("#bms-control-selector");
  selector.value = state.selectedBms;
  selector.disabled = state.bmsConfigLoading || state.bmsConfigWriting;
}

function bmsConfigurationInput(item) {
  const id = `bms-setting-${item.key}`;
  if (!item.writable) return `<div class="setting-readonly">${esc(configurationValueLabel(item))}</div>`;
  if (item.type === "select") {
    return `<select id="${esc(id)}" data-bms-setting-input="${esc(item.key)}">${(item.options || []).map((option) => `<option value="${esc(option.value)}" ${String(option.value) === String(item.value) ? "selected" : ""}>${esc(option.label)}</option>`).join("")}</select>`;
  }
  if (item.type === "bool") {
    return `<select id="${esc(id)}" data-bms-setting-input="${esc(item.key)}"><option value="true" ${item.value ? "selected" : ""}>Enabled</option><option value="false" ${item.value ? "" : "selected"}>Disabled</option></select>`;
  }
  const attributes = [
    `id="${esc(id)}"`, `data-bms-setting-input="${esc(item.key)}"`, `type="number"`,
    `value="${esc(item.value)}"`,
    item.minimum !== undefined ? `min="${esc(item.minimum)}"` : "",
    item.maximum !== undefined ? `max="${esc(item.maximum)}"` : "",
    item.step !== undefined ? `step="${esc(item.step)}"` : "",
  ].filter(Boolean).join(" ");
  return `<div class="setting-input-wrap"><input ${attributes}>${item.unit ? `<span>${esc(item.unit)}</span>` : ""}</div>`;
}

function renderBmsConfiguration(configuration) {
  const identity = configuration.identity || {};
  const settingCount = (configuration.groups || []).reduce((total, group) => total + (group.settings || []).length, 0);
  $("#bms-config-identity").innerHTML = `
    <article><span>BMS</span><strong>${esc(DEVICE_LABELS[configuration.device] || configuration.device)}</strong></article>
    <article><span>Model</span><strong>${esc(identity.model)}</strong></article>
    <article><span>Firmware</span><strong>${esc(identity.firmware || "—")}</strong></article>
    <article><span>Hardware</span><strong>${esc(identity.hardware || "—")}</strong></article>
    <article><span>BLE address</span><strong>${esc(identity.address)}</strong></article>
    <article><span>Writable parameters</span><strong>${settingCount}</strong></article>`;
  $("#bms-config-content").innerHTML = (configuration.groups || []).map((group) => `
    <article class="inverter-config-group">
      <div class="card-title"><div><span>Live BMS settings</span><strong>${esc(group.title)}</strong></div></div>
      <div class="inverter-settings-list">${(group.settings || []).map((item) => `
        <div class="inverter-setting ${item.writable ? "" : "readonly"} ${item.critical ? "critical" : ""}">
          <div class="setting-copy">
            <div><strong>${esc(item.label)}</strong>${item.critical ? `<span class="setting-badge">critical</span>` : ""}</div>
            <span>Current: ${esc(configurationValueLabel(item))}</span>
            ${item.description ? `<p>${esc(item.description)}</p>` : ""}
          </div>
          <div class="setting-control">
            ${bmsConfigurationInput(item)}
            ${item.writable ? `<button class="button secondary" type="button" data-apply-bms-setting="${esc(item.key)}">Apply</button>` : ""}
          </div>
        </div>`).join("")}</div>
    </article>`).join("");
  $("#bms-config-backup").disabled = false;
  setBmsConfigStatus(`Updated ${new Date().toLocaleTimeString("en-GB")} · every 30s`, "live");
}

function findBmsConfigurationItem(key) {
  for (const group of state.bmsConfig?.groups || []) {
    const item = (group.settings || []).find((candidate) => candidate.key === key);
    if (item) return item;
  }
  return null;
}

async function loadBmsConfiguration() {
  if (!state.selectedBms || state.bmsConfigLoading || state.bmsConfigWriting) return;
  state.bmsConfigLoading = true;
  renderBmsControlShell();
  setBmsConfigStatus("Reading live configuration…", "busy");
  try {
    const result = await api(`/api/bms/${encodeURIComponent(state.selectedBms)}/configuration`);
    state.bmsConfig = result.configuration;
    renderBmsConfiguration(state.bmsConfig);
  } catch (error) {
    setBmsConfigStatus(error.message.includes("another Bluetooth operation") ? "Waiting for Bluetooth…" : "Configuration read failed");
    if (!state.bmsConfig) {
      $("#bms-config-content").innerHTML = `<article class="inverter-empty"><strong>Cannot read live configuration</strong><p>${esc(error.message)}. The dashboard will retry automatically.</p></article>`;
    }
  } finally {
    state.bmsConfigLoading = false;
    renderBmsControlShell();
  }
}

function bmsControlIsActive() {
  return $("#bms-control-panel").classList.contains("active") && document.visibilityState === "visible";
}

function stopBmsConfigPolling() {
  clearInterval(state.bmsConfigTimer);
  state.bmsConfigTimer = null;
}

function startBmsConfigPolling() {
  stopBmsConfigPolling();
  if (!bmsControlIsActive()) return;
  loadBmsConfiguration();
  state.bmsConfigTimer = setInterval(() => {
    const active = document.activeElement;
    const editing = $("#bms-config-content").contains(active)
      && (active.matches("input") || active.matches("select"));
    if (bmsControlIsActive() && !editing) loadBmsConfiguration();
  }, BMS_CONFIG_REFRESH_MS);
}

async function applyBmsSetting(key, button) {
  const item = findBmsConfigurationItem(key);
  const confirmation = state.bmsConfig?.identity?.confirmation;
  if (!item || !item.writable || !confirmation) {
    toast("The BMS identity is unavailable; reload before writing.", true);
    return;
  }
  const input = document.querySelector(`[data-bms-setting-input="${CSS.escape(key)}"]`);
  const requested = input?.value;
  if (requested === undefined || requested === "") return;
  if (!window.confirm(`Apply “${item.label}” to ${DEVICE_LABELS[state.selectedBms]}?\n\nCurrent: ${configurationValueLabel(item)}\nRequested: ${requested}\n\nThe command will be read back immediately. This setting can affect battery safety or wired communication.`)) return;
  const originalText = button.textContent;
  state.bmsConfigWriting = true;
  renderBmsControlShell();
  button.disabled = true;
  button.textContent = "Applying…";
  setBmsConfigStatus(`Writing ${item.label}…`, "busy");
  try {
    const response = await api(`/api/bms/${encodeURIComponent(state.selectedBms)}/setting`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ setting: key, value: requested, confirmation }),
    });
    state.bmsConfig = response.result.after_configuration;
    renderBmsConfiguration(state.bmsConfig);
    toast(response.result.written
      ? "BMS setting written and verified by read-back."
      : "The requested value was already active; no write was needed.");
  } catch (error) {
    setBmsConfigStatus("Write failed; reload before retrying");
    toast(`BMS command rejected: ${error.message}`, true);
  } finally {
    state.bmsConfigWriting = false;
    renderBmsControlShell();
    button.disabled = false;
    button.textContent = originalText;
  }
}

function renderHeader(data) {
  const devices = Object.values(data.devices || {});
  const online = devices.filter((device) => device.online).length;
  const totalPower = devices.reduce((sum, device) => sum + (Number(device.telemetry?.power) || 0), 0);
  const inverters = Object.values(data.inverters?.devices || {});
  const onlineInverters = inverters.filter((device) => device.online).length;
  const inverterOutput = inverters.reduce((sum, device) => sum + (Number(device.telemetry?.output_power_w) || 0), 0);
  $("#online-count").textContent = online;
  $("#total-power").textContent = `${num(totalPower, 0)} W`;
  $("#inverter-output-total").textContent = `${num(inverterOutput, 0)} W`;
  $("#inverter-online-summary").textContent = `${onlineInverters}/${inverters.length || 2} online`;
  $("#control-state").textContent = data.control_auth_required ? "Protected" : "Trusted LAN";
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

function renderAll() {
  if (!state.data) return;
  renderHeader(state.data);
  renderCards(state.data);
  renderDetails(state.data);
  renderInverters(state.data);
  renderInverterControlShell(state.data);
  renderBmsControlShell();
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

$("#inverter-control-selector").addEventListener("change", (event) => {
  state.selectedInverter = event.target.value;
  state.inverterConfig = null;
  $("#inverter-config-identity").innerHTML = "";
  $("#inverter-config-content").innerHTML = `<article class="inverter-empty"><strong>Reading selected inverter</strong><p>Waiting for its live configuration.</p></article>`;
  $("#inverter-config-backup").disabled = true;
  setInverterConfigStatus("Waiting for automatic read");
  startInverterConfigPolling();
});
$("#inverter-config-content").addEventListener("click", (event) => {
  const button = event.target.closest("[data-apply-setting]");
  if (button) applyInverterSetting(button.dataset.applySetting, button);
});
$("#inverter-config-backup").addEventListener("click", () => {
  if (!state.inverterConfig) return;
  const blob = new Blob([JSON.stringify(state.inverterConfig, null, 2)], { type: "application/json" });
  const url = URL.createObjectURL(blob);
  const link = document.createElement("a");
  link.href = url;
  link.download = `${state.selectedInverter}-configuration-${new Date().toISOString().replace(/[:.]/g, "-")}.json`;
  link.click();
  URL.revokeObjectURL(url);
});

$("#bms-control-selector").addEventListener("change", (event) => {
  state.selectedBms = event.target.value;
  state.bmsConfig = null;
  $("#bms-config-identity").innerHTML = "";
  $("#bms-config-content").innerHTML = `<article class="inverter-empty"><strong>Reading selected BMS</strong><p>Waiting for its live Bluetooth configuration.</p></article>`;
  $("#bms-config-backup").disabled = true;
  setBmsConfigStatus("Waiting for automatic read");
  startBmsConfigPolling();
});
$("#bms-config-content").addEventListener("click", (event) => {
  const button = event.target.closest("[data-apply-bms-setting]");
  if (button) applyBmsSetting(button.dataset.applyBmsSetting, button);
});
$("#bms-config-backup").addEventListener("click", () => {
  if (!state.bmsConfig) return;
  const blob = new Blob([JSON.stringify(state.bmsConfig, null, 2)], { type: "application/json" });
  const url = URL.createObjectURL(blob);
  const link = document.createElement("a");
  link.href = url;
  link.download = `${state.selectedBms}-bms-configuration-${new Date().toISOString().replace(/[:.]/g, "-")}.json`;
  link.click();
  URL.revokeObjectURL(url);
});

document.querySelectorAll(".tab").forEach((button) => button.addEventListener("click", () => {
  document.querySelectorAll(".tab").forEach((item) => item.classList.toggle("active", item === button));
  document.querySelectorAll(".tab-panel").forEach((panel) => panel.classList.toggle("active", panel.id === `${button.dataset.tab}-panel`));
  if (button.dataset.tab === "inverter-control") startInverterConfigPolling();
  else stopInverterConfigPolling();
  if (button.dataset.tab === "bms-control") startBmsConfigPolling();
  else stopBmsConfigPolling();
}));

document.addEventListener("visibilitychange", () => {
  if (document.visibilityState === "visible") {
    startInverterConfigPolling();
    startBmsConfigPolling();
  } else {
    stopInverterConfigPolling();
    stopBmsConfigPolling();
  }
});

loadStatus(true);
setInterval(() => loadStatus(false), 3000);
