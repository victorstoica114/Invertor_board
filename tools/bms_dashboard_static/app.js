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
  iotWriting: new Set(),
  iotChartMetric: "power",
  toastTimer: null,
};

const IOT_CHART_METRICS = {
  power: {
    key: "electric_power_w", title: "Power history", unit: "W", digits: 1,
    note: "Power is measured by each smart plug.", startsAtZero: true, minimumMaximum: 1,
  },
  current: {
    key: "estimated_current_a", title: "Calculated current history", unit: "A", digits: 3,
    note: "Calculated from plug power and the latest valid inverter AC voltage.", startsAtZero: true, minimumMaximum: .05,
  },
  voltage: {
    key: "voltage_v", title: "Inverter voltage history", unit: "V", digits: 1,
    note: "Voltage is sourced from the freshest valid inverter AC-output sample.", startsAtZero: false, minimumMaximum: 0,
  },
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
  const inverterState = $("#inverter-database-state");
  const online = devices.filter((device) => device.online).length;
  inverterState.className = `status-pill ${monitor.available ? (online ? "live" : "") : ""}`;
  inverterState.innerHTML = `<span class="dot"></span>${monitor.available ? `Direct LAN · ${online}/${devices.length} online` : "Direct telemetry unavailable"}`;
  if (!monitor.available) {
    $("#inverter-grid").innerHTML = `<article class="inverter-empty"><strong>Inverter telemetry is unavailable</strong><p>${esc(monitor.error || "The direct inverter inventory could not be loaded.")}</p></article>`;
    return;
  }
  if (!devices.length) {
    $("#inverter-grid").innerHTML = `<article class="inverter-empty"><strong>No inverters configured</strong><p>The direct inverter inventory is empty.</p></article>`;
    return;
  }
  $("#inverter-grid").innerHTML = devices.map(inverterCard).join("");
}

function iotReading(value, digits, unit = "") {
  if (value === null || value === undefined || !Number.isFinite(Number(value))) return "—";
  return `${Number(value).toFixed(digits)}${unit ? ` ${unit}` : ""}`;
}

function powerSwitch(id, label, enabled, checked, kind = "plug") {
  return `<label class="power-switch" title="Power ${esc(label)}">
    <input type="checkbox" data-iot-power="${esc(id)}" data-iot-kind="${kind}" ${checked ? "checked" : ""} ${enabled ? "" : "disabled"} aria-label="Power ${esc(label)}">
    <span aria-hidden="true"></span>
  </label>`;
}

function plugCard(device) {
  const telemetry = device.telemetry || {};
  const canControl = device.online && typeof telemetry.on === "boolean" && !state.iotWriting.has(device.id);
  const status = device.updating ? "Reading" : device.online ? "Online" : "Offline";
  return `<article class="iot-device-card ${device.online ? "online" : "offline"}">
    <div class="device-head">
      <div><span class="eyebrow">${esc(device.ip)}</span><h3>${esc(device.name)}</h3><span class="device-subtitle">Xiaomi smart plug</span></div>
      <span class="device-status ${device.online ? "online" : ""}"><span class="dot"></span>${status}</span>
    </div>
    <div class="iot-power-row">
      <div><span>Live power</span><strong>${esc(iotReading(telemetry.electric_power_w, 1, "W"))}</strong></div>
      ${powerSwitch(device.id, device.name, canControl, telemetry.on === true)}
    </div>
    <div class="iot-metrics">
      <div><span>Power state</span><strong>${typeof telemetry.on === "boolean" ? (telemetry.on ? "On" : "Off") : "—"}</strong></div>
      <div><span>Energy counter</span><strong>${esc(iotReading(telemetry.energy_counter, 3, "kWh"))}</strong></div>
      <div><span>Calculated current</span><strong>${esc(iotReading(telemetry.estimated_current_a, 3, "A"))}</strong></div>
      <div><span>${String(telemetry.voltage_source).startsWith("inverter_") ? "Inverter voltage" : "Fallback voltage"}</span><strong>${esc(iotReading(telemetry.voltage_v, 1, "V"))}</strong></div>
      <div><span>Protection</span><strong>${esc(telemetry.fault_label || "—")}</strong></div>
      <div><span>Last reading</span><strong>${device.last_success ? esc(new Date(device.last_success).toLocaleTimeString("en-GB")) : "—"}</strong></div>
    </div>
    ${device.error ? `<div class="device-error">${esc(device.error)}</div>` : ""}
  </article>`;
}

function controlLabel(value) {
  const labels = {
    auto: "Auto", cold: "Cool", hot: "Heat", wet: "Dry", wind: "Fan",
    low: "Low", mid_low: "Medium low", mid: "Medium", mid_high: "Medium high",
    high: "High", strong: "Turbo", mute: "Quiet", off: "Off",
  };
  return labels[value] || String(value).replaceAll("_", " ");
}

function acSegment(setting, values, selected, enabled) {
  return `<div class="ac-segmented">${values.map((value) =>
    `<button class="${String(selected) === String(value) ? "active" : ""}" type="button" data-ac-setting="${esc(setting)}" data-ac-value="${esc(value)}" ${enabled ? "" : "disabled"}>${esc(controlLabel(value))}</button>`
  ).join("")}</div>`;
}

function acOptions(values) {
  return (values || []).map((option) => typeof option === "object"
    ? { value: String(option.value), label: String(option.label) }
    : { value: String(option), label: controlLabel(option) });
}

function acSelect(setting, label, values, selected, enabled) {
  const options = acOptions(values);
  const selectedValue = selected === null || selected === undefined ? "" : String(selected);
  return `<label class="ac-select-control">
    <span>${esc(label)}</span>
    <select data-ac-select="${esc(setting)}" ${enabled && options.length ? "" : "disabled"}>
      ${selectedValue === "" ? '<option value="" selected>Unavailable</option>' : ""}
      ${options.map((option) => `<option value="${esc(option.value)}" ${option.value === selectedValue ? "selected" : ""}>${esc(option.label)}</option>`).join("")}
    </select>
  </label>`;
}

function acToggle(setting, label, checked, enabled) {
  return `<div class="ac-toggle-row">
    <div><strong>${esc(label)}</strong><small>${checked === true ? "On" : checked === false ? "Off" : "Unavailable"}</small></div>
    <label class="power-switch" title="${esc(label)}">
      <input type="checkbox" data-ac-toggle="${esc(setting)}" ${checked ? "checked" : ""} ${enabled && typeof checked === "boolean" ? "" : "disabled"} aria-label="${esc(label)}">
      <span aria-hidden="true"></span>
    </label>
  </div>`;
}

function acDiagnostic(label, value) {
  const display = value === null || value === undefined || value === "" ? "—" : String(value);
  return `<div><span>${esc(label)}</span><strong title="${esc(display)}">${esc(display)}</strong></div>`;
}

function acScheduleRow(schedule) {
  const dayNames = ["Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Sun"];
  const action = schedule.on ? "Turn on" : "Turn off";
  const timing = schedule.kind === "timer"
    ? new Date(schedule.run_at).toLocaleString("en-GB")
    : `${schedule.time} · ${(schedule.days || []).map((day) => dayNames[day]).join(" ")}`;
  const stateLabel = schedule.enabled ? "Active" : schedule.last_error ? "Failed" : "Completed";
  return `<div class="ac-schedule-row ${schedule.enabled ? "active" : ""}">
    <div><strong>${esc(action)}</strong><span>${esc(timing)}</span>${schedule.last_error ? `<small>${esc(schedule.last_error)}</small>` : ""}</div>
    <span class="ac-schedule-state">${esc(stateLabel)}</span>
    <button type="button" data-ac-delete-schedule="${esc(schedule.id)}" title="Delete schedule" aria-label="Delete schedule">×</button>
  </div>`;
}

function renderAirConditioner(device) {
  const telemetry = device.telemetry || {};
  const canControl = device.configured && device.online && typeof telemetry.on === "boolean" && !state.iotWriting.has(device.id);
  const statusText = !device.configured ? "Local key required" : device.updating ? "Reading" : device.online ? "Online" : "Offline";
  const status = $("#ac-state");
  status.className = `status-pill ${device.updating ? "busy" : device.online ? "live" : ""}`;
  status.innerHTML = `<span class="dot"></span>${esc(statusText)}`;
  const controls = device.controls || {};
  const temperature = controls.temperature || { minimum_c: 16, maximum_c: 30, step_c: 1 };
  const currentTarget = Number(telemetry.target_temperature_c);
  const target = Number.isFinite(currentTarget) ? currentTarget : Number(temperature.minimum_c);
  const writable = canControl && !device.updating;
  const temperatureWritable = writable && Number.isFinite(currentTarget);
  const available = telemetry.available_controls || {};
  const canWrite = (setting) => writable && available[setting] !== false;
  const activeCapabilities = Object.entries(telemetry.capabilities || {})
    .filter(([, enabled]) => enabled)
    .map(([name]) => controlLabel(name))
    .join(", ");
  const advancedSwitches = controls.advanced_switches || [];
  const ecoTemperatures = Array.from({ length: 6 }, (_, index) => ({
    value: 26 + index,
    label: `${26 + index} °C`,
  }));
  const statistics = telemetry.statistics && typeof telemetry.statistics === "object"
    ? JSON.stringify(telemetry.statistics)
    : telemetry.statistics;
  const schedules = [...(device.schedules || [])].reverse();
  const reservationTime = new Date(Date.now() + 60 * 60 * 1000)
    .toLocaleTimeString("en-GB", { hour: "2-digit", minute: "2-digit" });
  $("#ac-control-content").innerHTML = `
    <section class="ac-overview">
      <div>
        <span class="eyebrow">${esc(device.ip || "192.168.1.200")}</span>
        <h4>${esc(device.name)}</h4>
        <p>${device.last_success ? `Last reading ${esc(new Date(device.last_success).toLocaleString("en-GB"))}` : "Waiting for local telemetry"}</p>
      </div>
      <div class="ac-overview-readings">
        <div class="ac-room-reading"><span>Room</span><strong>${esc(iotReading(telemetry.current_temperature_c, 1, "°C"))}</strong></div>
        <div class="ac-overview-metric"><span>Humidity</span><strong>${esc(iotReading(telemetry.current_humidity_pct, 0, "%"))}</strong></div>
        <div class="ac-overview-metric"><span>Air quality</span><strong>${esc(telemetry.air_quality_label || "—")}</strong></div>
      </div>
      <div class="ac-power-control"><span>${telemetry.on === true ? "On" : telemetry.on === false ? "Off" : "Power"}</span>${powerSwitch(device.id, device.name, canControl, telemetry.on === true, "ac")}</div>
    </section>
    <section class="ac-controls-panel">
      <div class="ac-control-section">
        <div class="ac-control-title"><span>Target temperature</span><strong>${esc(iotReading(target, 1, "°C"))}</strong></div>
        <div class="temperature-stepper">
          <button type="button" data-ac-temperature="decrease" title="Decrease target temperature" ${temperatureWritable && target > Number(temperature.minimum_c) ? "" : "disabled"}>−</button>
          <output>${esc(iotReading(target, 1, "°C"))}</output>
          <button type="button" data-ac-temperature="increase" title="Increase target temperature" ${temperatureWritable && target < Number(temperature.maximum_c) ? "" : "disabled"}>+</button>
        </div>
      </div>
      <div class="ac-control-section">
        <div class="ac-control-title"><span>Operating mode</span><strong>${esc(controlLabel(telemetry.mode ?? "—"))}</strong></div>
        ${acSegment("mode", controls.modes || [], telemetry.mode, writable)}
      </div>
      <div class="ac-control-section">
        <div class="ac-control-title"><span>Fan speed</span><strong>${esc(controlLabel(telemetry.fan ?? "—"))}</strong></div>
        ${acSegment("fan", controls.fan_speeds || [], telemetry.fan, writable)}
      </div>
    </section>
    <section class="ac-feature-section">
      <div class="ac-feature-heading"><span>Precision airflow</span><strong>Vertical and horizontal</strong></div>
      <div class="ac-select-grid">
        ${acSelect("vertical_swing", "Vertical sweep", controls.vertical_swing, telemetry.vertical_swing, canWrite("vertical_swing"))}
        ${acSelect("vertical_position", "Vertical position", controls.vertical_position, telemetry.vertical_position, canWrite("vertical_position"))}
        ${acSelect("horizontal_swing", "Horizontal sweep", controls.horizontal_swing, telemetry.horizontal_swing, canWrite("horizontal_swing"))}
        ${acSelect("horizontal_position", "Horizontal position", controls.horizontal_position, telemetry.horizontal_position, canWrite("horizontal_position"))}
      </div>
    </section>
    <section class="ac-feature-section">
      <div class="ac-feature-heading"><span>Comfort and energy</span><strong>${esc(telemetry.sleep_label || "Sleep off")}</strong></div>
      <div class="ac-select-grid">
        ${acSelect("sleep", "Sleep profile", controls.sleep, telemetry.sleep, canWrite("sleep"))}
        ${acSelect("generator_mode", "GEN mode", controls.generator_mode, telemetry.generator_mode, canWrite("generator_mode"))}
        ${acSelect("energy_saving", "Electricity management", controls.energy_saving, telemetry.energy_saving, canWrite("energy_saving"))}
        ${acSelect("eco_temperature_c", "ECO temperature", ecoTemperatures, telemetry.eco_temperature_c, canWrite("eco_temperature_c"))}
      </div>
    </section>
    <section class="ac-feature-section">
      <div class="ac-feature-heading"><span>Timer and reservations</span><strong>${schedules.filter((schedule) => schedule.enabled).length} active</strong></div>
      <div class="ac-scheduler-grid">
        <div class="ac-scheduler-control">
          <strong>Relative timer</strong>
          <div class="ac-scheduler-fields">
            <select data-ac-timer-action aria-label="Timer action"><option value="off">Turn off</option><option value="on">Turn on</option></select>
            <input type="number" data-ac-timer-minutes min="1" max="10080" step="1" value="60" aria-label="Timer minutes">
            <span>min</span>
            <button type="button" data-ac-create-timer ${device.configured ? "" : "disabled"}>Set timer</button>
          </div>
        </div>
        <div class="ac-scheduler-control">
          <strong>Weekly reservation</strong>
          <div class="ac-scheduler-fields reservation">
            <select data-ac-reservation-action aria-label="Reservation action"><option value="on">Turn on</option><option value="off">Turn off</option></select>
            <input type="time" data-ac-reservation-time value="${esc(reservationTime)}" aria-label="Reservation time">
            <button type="button" data-ac-create-reservation ${device.configured ? "" : "disabled"}>Add</button>
          </div>
          <div class="ac-day-selector">
            ${["M", "T", "W", "T", "F", "S", "S"].map((day, index) => `<label title="${["Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday", "Sunday"][index]}"><input type="checkbox" data-ac-reservation-day="${index}" checked><span>${day}</span></label>`).join("")}
          </div>
        </div>
      </div>
      <div class="ac-schedule-list">${schedules.length ? schedules.map(acScheduleRow).join("") : '<span class="ac-schedule-empty">No timers or reservations</span>'}</div>
    </section>
    <section class="ac-feature-section">
      <div class="ac-feature-heading"><span>Advanced functions</span><strong>${advancedSwitches.filter((item) => telemetry[item.setting] === true).length} active</strong></div>
      <div class="ac-toggle-grid">
        ${advancedSwitches.map((item) => acToggle(
          item.setting,
          item.label,
          telemetry[item.setting],
          canWrite(item.setting) && !(item.setting === "self_clean" && telemetry.on === true),
        )).join("")}
        ${acToggle("hot_cold_air", "Hot/cold airflow", telemetry.hot_cold_air, canWrite("hot_cold_air"))}
      </div>
    </section>
    <section class="ac-feature-section ac-diagnostics">
      <div class="ac-feature-heading"><span>Self diagnostics and service</span><strong>${esc(telemetry.fault_label || "Unavailable")}</strong></div>
      <div class="ac-diagnostic-grid">
        ${acDiagnostic("Fault DP20", telemetry.fault_code)}
        ${acDiagnostic("Secondary fault DP122", telemetry.fault_secondary_code)}
        ${acDiagnostic("Filter", telemetry.filter_dirty === true ? "Service required" : telemetry.filter_dirty === false ? "Clean" : null)}
        ${acDiagnostic("PM2.5", telemetry.pm25_ug_m3 === null || telemetry.pm25_ug_m3 === undefined ? null : `${telemetry.pm25_ug_m3} µg/m³`)}
        ${acDiagnostic("Capability flags DP110", telemetry.capability_flags_raw)}
        ${acDiagnostic("Advanced flags DP123", telemetry.advanced_flags_raw)}
        ${acDiagnostic("Model code DP128", telemetry.model_code)}
        ${acDiagnostic("Energy selector DP129", telemetry.energy_quota_raw)}
        ${acDiagnostic("Swing action DP133", telemetry.swing_action_raw)}
        ${acDiagnostic("Statistics DP134", statistics)}
        ${acDiagnostic("Runtime DP135", telemetry.running_time_raw)}
        ${acDiagnostic("Unmapped service DP136", telemetry.service_value_136_raw)}
      </div>
      <div class="ac-capabilities"><span>Reported capabilities</span><strong>${esc(activeCapabilities || "None reported")}</strong></div>
      <details class="ac-raw-dps"><summary>Raw service datapoints</summary><pre>${esc(JSON.stringify(telemetry.raw_dps || {}, null, 2))}</pre></details>
    </section>
    ${!device.configured ? `<div class="ac-unavailable"><strong>Local control is not authenticated</strong><span>The panel is ready and will unlock after the Tuya local key is installed.</span></div>` : ""}
    ${device.configured && device.error ? `<div class="device-error">${esc(device.error)}</div>` : ""}`;
}

function renderIotPowerChart(iot) {
  const svg = $("#iot-power-chart");
  const devices = Object.values(iot.plugs?.devices || {});
  const history = iot.plugs?.history || {};
  const metric = IOT_CHART_METRICS[state.iotChartMetric] || IOT_CHART_METRICS.power;
  const voltageReference = iot.inverter_voltage_reference || {};
  const colors = ["#4de0a4", "#65d6df", "#f4c76d", "#ff7b75"];
  const series = devices.map((device, index) => ({
    device,
    color: colors[index % colors.length],
    values: (history[device.id] || []).filter((sample) =>
      sample[metric.key] !== null && sample[metric.key] !== undefined
      && Number.isFinite(Number(sample[metric.key]))
    ),
  }));
  $("#iot-chart-title").textContent = metric.title;
  $("#iot-chart-note").textContent = state.iotChartMetric === "power"
    ? metric.note
    : voltageReference.available
      ? `${metric.note} Source: ${voltageReference.source_name}, sampled ${new Date(voltageReference.sampled_at).toLocaleString("en-GB")}.`
      : `${metric.note} No fresh inverter sample is available, so the configured 230 V fallback is marked in the data.`;
  svg.setAttribute("aria-label", `${metric.title} chart`);
  document.querySelectorAll("[data-iot-chart]").forEach((button) =>
    button.classList.toggle("active", button.dataset.iotChart === state.iotChartMetric)
  );
  $("#iot-chart-legend").innerHTML = series.map(({ device, color }) =>
    `<span><i style="background:${color}"></i>${esc(device.name)}</span>`
  ).join("");
  const allValues = series.flatMap((item) => item.values.map((sample) => Number(sample[metric.key])));
  const latestTotal = series.reduce((total, item) =>
    total + (item.values.length ? Number(item.values.at(-1)[metric.key]) : 0), 0
  );
  const latestDisplay = state.iotChartMetric === "voltage" && series.length
    ? latestTotal / series.filter((item) => item.values.length).length
    : latestTotal;
  $("#iot-chart-value").textContent = allValues.length
    ? `${num(latestDisplay, metric.digits)} ${metric.unit}${state.iotChartMetric === "voltage" ? " reference" : " total"}`
    : "—";
  if (series.every((item) => item.values.length < 2)) {
    svg.innerHTML = `<text x="400" y="125" class="chart-empty">History appears after the first two readings</text>`;
    $("#iot-chart-range").textContent = "latest readings";
    return;
  }
  const width = 800, height = 240, pad = 24;
  const observedMinimum = Math.min(...allValues);
  const observedMaximum = Math.max(...allValues);
  let minimum = metric.startsAtZero ? 0 : observedMinimum;
  let maximum = Math.max(metric.minimumMaximum, observedMaximum);
  if (Math.abs(maximum - minimum) < .01) {
    const padding = Math.max(1, Math.abs(maximum) * .015);
    minimum = metric.startsAtZero ? 0 : minimum - padding;
    maximum += padding;
  } else {
    maximum += (maximum - minimum) * .08;
  }
  const longest = Math.max(...series.map((item) => item.values.length), 2);
  const x = (index) => pad + index * (width - pad * 2) / (longest - 1);
  const y = (value) => height - pad - (value - minimum) * (height - pad * 2) / (maximum - minimum);
  const paths = series.map(({ values, color }) => {
    if (values.length < 2) return "";
    const points = values.map((sample, index) => `${x(index)},${y(Number(sample[metric.key]))}`).join(" ");
    return `<polyline class="iot-chart-line" style="stroke:${color}" points="${points}"/>`;
  }).join("");
  svg.innerHTML = `${[.25, .5, .75].map((part) => `<line class="chart-grid" x1="${pad}" y1="${height * part}" x2="${width - pad}" y2="${height * part}"/>`).join("")}${paths}`;
  const observedRange = Math.abs(observedMaximum - observedMinimum) < 1e-9
    ? num(observedMinimum, metric.digits)
    : `${num(observedMinimum, metric.digits)}…${num(observedMaximum, metric.digits)}`;
  $("#iot-chart-range").textContent = `${observedRange} ${metric.unit} · ${longest} samples`;
}

function renderIot(data) {
  const iot = data.iot || {};
  const plugs = Object.values(iot.plugs?.devices || {});
  const ac = iot.air_conditioner || { configured: false, id: "air-conditioner", name: "Air conditioner", telemetry: {} };
  const online = plugs.filter((device) => device.online).length;
  const status = $("#iot-state");
  status.className = `status-pill ${iot.polling ? "busy" : online ? "live" : ""}`;
  status.innerHTML = `<span class="dot"></span>${iot.polling ? "Reading smart plugs" : `${online}/${plugs.length} online`}`;
  $("#iot-grid").innerHTML = plugs.map(plugCard).join("");
  renderIotPowerChart(iot);
  renderAirConditioner(ac);
}

async function applyIotPower(input) {
  const id = input.dataset.iotPower;
  const isAc = input.dataset.iotKind === "ac";
  const device = isAc
    ? state.data?.iot?.air_conditioner
    : state.data?.iot?.plugs?.devices?.[id];
  if (!device || state.iotWriting.has(id)) return;
  const enabled = input.checked;
  state.iotWriting.add(id);
  renderIot(state.data);
  try {
    const path = isAc
      ? "/api/iot/air-conditioner/power"
      : `/api/iot/plugs/${encodeURIComponent(id)}/power`;
    const response = await api(path, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ on: enabled }),
    });
    toast(response.result.written
      ? `${device.name} power changed and verified.`
      : `${device.name} was already ${enabled ? "on" : "off"}.`);
    await loadStatus();
  } catch (error) {
    toast(`Power command failed: ${error.message}`, true);
  } finally {
    state.iotWriting.delete(id);
    renderIot(state.data);
  }
}

async function applyAcSetting(setting, value) {
  const device = state.data?.iot?.air_conditioner;
  if (!device?.configured || !device.online || state.iotWriting.has(device.id)) return;
  state.iotWriting.add(device.id);
  renderIot(state.data);
  try {
    const response = await api("/api/iot/air-conditioner/setting", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ setting, value }),
    });
    toast(response.result.written
      ? `Air-conditioner ${setting.replaceAll("_", " ")} changed and verified.`
      : "The requested air-conditioner setting was already active.");
    await loadStatus();
  } catch (error) {
    toast(`Air-conditioner command failed: ${error.message}`, true);
  } finally {
    state.iotWriting.delete(device.id);
    renderIot(state.data);
  }
}

async function createAcTimer(button) {
  const root = $("#ac-control-content");
  const minutes = Number(root.querySelector("[data-ac-timer-minutes]")?.value);
  const on = root.querySelector("[data-ac-timer-action]")?.value === "on";
  button.disabled = true;
  try {
    await api("/api/iot/air-conditioner/timers", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ minutes, on }),
    });
    toast(`AC ${on ? "on" : "off"} timer saved.`);
    await loadStatus();
  } catch (error) {
    toast(`Cannot save timer: ${error.message}`, true);
    button.disabled = false;
  }
}

async function createAcReservation(button) {
  const root = $("#ac-control-content");
  const time = root.querySelector("[data-ac-reservation-time]")?.value;
  const on = root.querySelector("[data-ac-reservation-action]")?.value === "on";
  const days = [...root.querySelectorAll("[data-ac-reservation-day]:checked")]
    .map((input) => Number(input.dataset.acReservationDay));
  button.disabled = true;
  try {
    await api("/api/iot/air-conditioner/reservations", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ time, days, on }),
    });
    toast(`Weekly AC ${on ? "on" : "off"} reservation saved.`);
    await loadStatus();
  } catch (error) {
    toast(`Cannot save reservation: ${error.message}`, true);
    button.disabled = false;
  }
}

async function deleteAcSchedule(button) {
  button.disabled = true;
  try {
    await api(`/api/iot/air-conditioner/schedules/${encodeURIComponent(button.dataset.acDeleteSchedule)}`, {
      method: "DELETE",
    });
    toast("AC schedule deleted.");
    await loadStatus();
  } catch (error) {
    toast(`Cannot delete schedule: ${error.message}`, true);
    button.disabled = false;
  }
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
  const actionsDisabled = !state.inverterConfig || state.inverterConfigLoading || state.inverterConfigWriting;
  $("#inverter-config-backup").disabled = actionsDisabled;
  $("#inverter-config-restore").disabled = actionsDisabled;
}

function configurationValueLabel(item) {
  const option = (item.options || []).find((candidate) => String(candidate.value) === String(item.value));
  if (option) return option.label;
  if (item.display_value !== undefined) return item.display_value;
  return `${item.value ?? "—"}${item.unit ? ` ${item.unit}` : ""}`;
}

function configurationItems(configuration) {
  if (!configuration || !Array.isArray(configuration.groups)) return [];
  return configuration.groups.flatMap((group) => Array.isArray(group.settings) ? group.settings : []);
}

function sameConfigurationValue(first, second) {
  const firstBoolean = typeof first === "boolean" || /^(true|false)$/i.test(String(first));
  const secondBoolean = typeof second === "boolean" || /^(true|false)$/i.test(String(second));
  if (firstBoolean && secondBoolean) return String(first).toLowerCase() === String(second).toLowerCase();
  const firstNumber = Number(first);
  const secondNumber = Number(second);
  if (String(first).trim() !== "" && String(second).trim() !== ""
      && Number.isFinite(firstNumber) && Number.isFinite(secondNumber)) {
    return firstNumber === secondNumber;
  }
  return String(first) === String(second);
}

function configurationRestorePlan(liveConfiguration, backupConfiguration) {
  const backupValues = new Map(configurationItems(backupConfiguration)
    .filter((item) => item && typeof item.key === "string"
      && Object.prototype.hasOwnProperty.call(item, "value"))
    .map((item) => [item.key, item.value]));
  const compatible = configurationItems(liveConfiguration)
    .filter((item) => item?.writable && item.type !== "action" && backupValues.has(item.key));
  return {
    compatibleCount: compatible.length,
    changes: compatible
      .filter((item) => !sameConfigurationValue(item.value, backupValues.get(item.key)))
      .map((item) => ({ item, value: backupValues.get(item.key) })),
  };
}

function requestedConfigurationValueLabel(item, value) {
  const option = (item.options || []).find((candidate) => sameConfigurationValue(candidate.value, value));
  if (option) return option.label;
  return `${value}${item.unit ? ` ${item.unit}` : ""}`;
}

function configurationRestorePrompt(title, file, changes, identityWarning = "") {
  const previewLimit = 10;
  const preview = changes.slice(0, previewLimit).map(({ item, value }) =>
    `• ${item.label}: ${configurationValueLabel(item)} → ${requestedConfigurationValueLabel(item, value)}`
  );
  if (changes.length > previewLimit) preview.push(`• …and ${changes.length - previewLimit} more`);
  const criticalCount = changes.filter(({ item }) => item.critical).length;
  const criticalWarning = criticalCount
    ? `\n\n${criticalCount} critical setting${criticalCount === 1 ? "" : "s"} will be changed.`
    : "";
  return `${title}\n\nFile: ${file.name}\nChanges: ${changes.length}\n\n${preview.join("\n")}${identityWarning}${criticalWarning}\n\nEach setting will be written and verified by read-back. Continue?`;
}

async function readConfigurationBackup(file) {
  if (!file) throw new Error("No JSON backup was selected.");
  if (file.size > 5 * 1024 * 1024) throw new Error("The JSON backup is larger than 5 MB.");
  let configuration;
  try {
    configuration = JSON.parse(await file.text());
  } catch (error) {
    throw new Error("The selected file is not valid JSON.", { cause: error });
  }
  if (!configuration || Array.isArray(configuration) || typeof configuration !== "object"
      || !Array.isArray(configuration.groups)) {
    throw new Error("The selected JSON does not contain a configuration backup.");
  }
  return configuration;
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

async function restoreInverterConfiguration(file, button) {
  const originalText = button.textContent;
  let restoreStarted = false;
  let completed = 0;
  try {
    const backup = await readConfigurationBackup(file);
    const live = state.inverterConfig;
    if (!live) throw new Error("Load the live inverter configuration before uploading a backup.");
    if (!backup.protocol || backup.protocol !== live.protocol) {
      throw new Error(`This backup uses ${backup.protocol || "an unknown protocol"}; the selected inverter uses ${live.protocol}.`);
    }
    const confirmation = live.identity?.serial;
    if (!confirmation) throw new Error("The live inverter serial number is unavailable.");
    const plan = configurationRestorePlan(live, backup);
    if (!plan.compatibleCount) throw new Error("The backup contains no writable settings compatible with this inverter.");
    if (!plan.changes.length) {
      toast("All compatible backup settings are already active.");
      return;
    }

    const identityNotes = [];
    if (backup.identity?.serial && backup.identity.serial !== confirmation) {
      identityNotes.push(`backup serial ${backup.identity.serial} differs from live serial ${confirmation}`);
    }
    if (backup.inverter?.id && backup.inverter.id !== state.selectedInverter) {
      identityNotes.push(`backup inverter ${backup.inverter.id} differs from selected inverter ${state.selectedInverter}`);
    }
    const identityWarning = identityNotes.length ? `\n\nIdentity warning: ${identityNotes.join("; ")}.` : "";
    const target = live.inverter?.name || state.selectedInverter;
    if (!window.confirm(configurationRestorePrompt(
      `Restore the JSON backup to ${target}?`, file, plan.changes, identityWarning
    ))) return;

    restoreStarted = true;
    state.inverterConfigWriting = true;
    renderInverterControlShell(state.data);
    button.textContent = "Uploading…";
    let pending = plan.changes;
    const maximumWrites = Math.max(plan.compatibleCount * 2, pending.length);
    while (pending.length && completed < maximumWrites) {
      const { item, value } = pending[0];
      setInverterConfigStatus(`Restoring backup · write ${completed + 1}: ${item.label}…`, "busy");
      const response = await api(`/api/inverters/${encodeURIComponent(state.selectedInverter)}/setting`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ setting: item.key, value, confirmation }),
      });
      state.inverterConfig = response.result.after_configuration;
      completed += 1;
      pending = configurationRestorePlan(state.inverterConfig, backup).changes;
    }
    if (pending.length) throw new Error(`${pending.length} settings still differ after ${completed} verified writes.`);
    renderInverterConfiguration(state.inverterConfig);
    toast(`JSON backup restored: ${completed} verified write${completed === 1 ? "" : "s"}.`);
  } catch (error) {
    if (restoreStarted && state.inverterConfig) {
      renderInverterConfiguration(state.inverterConfig);
      setInverterConfigStatus(`Restore stopped after ${completed} setting${completed === 1 ? "" : "s"}`);
    }
    toast(`Cannot restore inverter backup: ${error.message}`, true);
  } finally {
    if (restoreStarted) state.inverterConfigWriting = false;
    button.textContent = originalText;
    renderInverterControlShell(state.data);
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
  const actionsDisabled = !state.bmsConfig || state.bmsConfigLoading || state.bmsConfigWriting;
  $("#bms-config-backup").disabled = actionsDisabled;
  $("#bms-config-restore").disabled = actionsDisabled;
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

async function restoreBmsConfiguration(file, button) {
  const originalText = button.textContent;
  let restoreStarted = false;
  let completed = 0;
  try {
    const backup = await readConfigurationBackup(file);
    const live = state.bmsConfig;
    if (!live) throw new Error("Load the live BMS configuration before uploading a backup.");
    if (!backup.device || backup.device !== state.selectedBms || backup.device !== live.device) {
      throw new Error(`This is a ${DEVICE_LABELS[backup.device] || backup.device || "different device"} backup, not a ${DEVICE_LABELS[state.selectedBms]} backup.`);
    }
    if (backup.protocol && live.protocol && backup.protocol !== live.protocol) {
      throw new Error(`This backup uses ${backup.protocol}; the selected BMS uses ${live.protocol}.`);
    }
    const confirmation = live.identity?.confirmation;
    if (!confirmation) throw new Error("The live BMS identity is unavailable.");
    const plan = configurationRestorePlan(live, backup);
    if (!plan.compatibleCount) throw new Error("The backup contains no writable settings compatible with this BMS.");
    if (!plan.changes.length) {
      toast("All compatible backup settings are already active.");
      return;
    }

    const backupIdentity = backup.identity?.confirmation;
    const identityWarning = backupIdentity && backupIdentity !== confirmation
      ? `\n\nIdentity warning: backup identity ${backupIdentity} differs from live identity ${confirmation}.`
      : "";
    if (!window.confirm(configurationRestorePrompt(
      `Restore the JSON backup to ${DEVICE_LABELS[state.selectedBms]}?`, file, plan.changes, identityWarning
    ))) return;

    restoreStarted = true;
    state.bmsConfigWriting = true;
    renderBmsControlShell();
    button.textContent = "Uploading…";
    let pending = plan.changes;
    const maximumWrites = Math.max(plan.compatibleCount * 2, pending.length);
    while (pending.length && completed < maximumWrites) {
      const { item, value } = pending[0];
      setBmsConfigStatus(`Restoring backup · write ${completed + 1}: ${item.label}…`, "busy");
      const response = await api(`/api/bms/${encodeURIComponent(state.selectedBms)}/setting`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ setting: item.key, value, confirmation }),
      });
      state.bmsConfig = response.result.after_configuration;
      completed += 1;
      pending = configurationRestorePlan(state.bmsConfig, backup).changes;
    }
    if (pending.length) throw new Error(`${pending.length} settings still differ after ${completed} verified writes.`);
    renderBmsConfiguration(state.bmsConfig);
    toast(`JSON backup restored: ${completed} verified write${completed === 1 ? "" : "s"}.`);
  } catch (error) {
    if (restoreStarted && state.bmsConfig) {
      renderBmsConfiguration(state.bmsConfig);
      setBmsConfigStatus(`Restore stopped after ${completed} setting${completed === 1 ? "" : "s"}`);
    }
    toast(`Cannot restore BMS backup: ${error.message}`, true);
  } finally {
    if (restoreStarted) state.bmsConfigWriting = false;
    button.textContent = originalText;
    renderBmsControlShell();
  }
}

function renderHeader(data) {
  const devices = Object.values(data.devices || {});
  const online = devices.filter((device) => device.online).length;
  const totalPower = devices.reduce((sum, device) => sum + (Number(device.telemetry?.power) || 0), 0);
  const inverters = Object.values(data.inverters?.devices || {});
  const onlineInverters = inverters.filter((device) => device.online).length;
  const inverterOutput = inverters.reduce((sum, device) => sum + (Number(device.telemetry?.output_power_w) || 0), 0);
  const plugs = Object.values(data.iot?.plugs?.devices || {});
  const iotPower = plugs.reduce((sum, device) => sum + (Number(device.telemetry?.electric_power_w) || 0), 0);
  $("#online-count").textContent = online;
  $("#total-power").textContent = `${num(totalPower, 0)} W`;
  $("#inverter-output-total").textContent = `${num(inverterOutput, 0)} W`;
  $("#inverter-online-summary").textContent = `${onlineInverters}/${inverters.length || 2} online`;
  $("#iot-power-summary").textContent = plugs.length ? `${num(iotPower, 1)} W` : "—";
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
  renderIot(state.data);
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
    toast(result.accepted ? "Device refresh scheduled." : "A recent refresh is already pending.");
    await loadStatus();
  } catch (error) { toast(error.message, true); }
});

$("#iot-grid").addEventListener("change", (event) => {
  const input = event.target.closest("[data-iot-power]");
  if (input) applyIotPower(input);
});
$("#ac-control-content").addEventListener("change", (event) => {
  const powerInput = event.target.closest('[data-iot-power][data-iot-kind="ac"]');
  if (powerInput) {
    applyIotPower(powerInput);
    return;
  }
  const toggle = event.target.closest("[data-ac-toggle]");
  if (toggle) {
    applyAcSetting(toggle.dataset.acToggle, toggle.checked);
    return;
  }
  const select = event.target.closest("[data-ac-select]");
  if (select) applyAcSetting(select.dataset.acSelect, select.value);
});
$("#ac-control-content").addEventListener("click", (event) => {
  const timerButton = event.target.closest("[data-ac-create-timer]");
  if (timerButton) {
    createAcTimer(timerButton);
    return;
  }
  const reservationButton = event.target.closest("[data-ac-create-reservation]");
  if (reservationButton) {
    createAcReservation(reservationButton);
    return;
  }
  const deleteScheduleButton = event.target.closest("[data-ac-delete-schedule]");
  if (deleteScheduleButton) {
    deleteAcSchedule(deleteScheduleButton);
    return;
  }
  const settingButton = event.target.closest("[data-ac-setting]");
  if (settingButton) {
    applyAcSetting(settingButton.dataset.acSetting, settingButton.dataset.acValue);
    return;
  }
  const temperatureButton = event.target.closest("[data-ac-temperature]");
  if (!temperatureButton) return;
  const device = state.data?.iot?.air_conditioner;
  const current = Number(device?.telemetry?.target_temperature_c);
  const step = Number(device?.controls?.temperature?.step_c || 1);
  if (!Number.isFinite(current) || !Number.isFinite(step)) return;
  const direction = temperatureButton.dataset.acTemperature === "increase" ? 1 : -1;
  applyAcSetting("target_temperature_c", current + direction * step);
});
document.querySelectorAll("[data-iot-chart]").forEach((button) => button.addEventListener("click", () => {
  state.iotChartMetric = button.dataset.iotChart;
  renderIotPowerChart(state.data?.iot || {});
}));

$("#inverter-control-selector").addEventListener("change", (event) => {
  state.selectedInverter = event.target.value;
  state.inverterConfig = null;
  $("#inverter-config-identity").innerHTML = "";
  $("#inverter-config-content").innerHTML = `<article class="inverter-empty"><strong>Reading selected inverter</strong><p>Waiting for its live configuration.</p></article>`;
  $("#inverter-config-backup").disabled = true;
  $("#inverter-config-restore").disabled = true;
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
$("#inverter-config-restore").addEventListener("click", () => {
  const input = $("#inverter-config-file");
  input.value = "";
  input.click();
});
$("#inverter-config-file").addEventListener("change", async (event) => {
  const file = event.target.files?.[0];
  if (file) await restoreInverterConfiguration(file, $("#inverter-config-restore"));
  event.target.value = "";
});

$("#bms-control-selector").addEventListener("change", (event) => {
  state.selectedBms = event.target.value;
  state.bmsConfig = null;
  $("#bms-config-identity").innerHTML = "";
  $("#bms-config-content").innerHTML = `<article class="inverter-empty"><strong>Reading selected BMS</strong><p>Waiting for its live Bluetooth configuration.</p></article>`;
  $("#bms-config-backup").disabled = true;
  $("#bms-config-restore").disabled = true;
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
$("#bms-config-restore").addEventListener("click", () => {
  const input = $("#bms-config-file");
  input.value = "";
  input.click();
});
$("#bms-config-file").addEventListener("change", async (event) => {
  const file = event.target.files?.[0];
  if (file) await restoreBmsConfiguration(file, $("#bms-config-restore"));
  event.target.value = "";
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
