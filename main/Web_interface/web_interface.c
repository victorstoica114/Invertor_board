#include "web_interface.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "bridge.h"
#include "config.h"
#include "runtime_settings.h"

#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

#define WEB_TAG "WEB_INTERFACE"
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static EventGroupHandle_t s_wifiEventGroup;
static int s_wifiRetryCount = 0;
static httpd_handle_t s_httpd = NULL;
static esp_netif_t *s_wifiStaNetif = NULL;
static TaskHandle_t s_settingsApplyTask = NULL;
static char s_logsResponse[2048];

typedef struct {
    bool restartWeb;
} settingsApplyCtx_t;

static void startHttpServer(void);

static void setNoCacheHeaders(httpd_req_t *req)
{
    if (req == NULL) {
        return;
    }

    httpd_resp_set_hdr(req, "Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
    httpd_resp_set_hdr(req, "Pragma", "no-cache");
    httpd_resp_set_hdr(req, "Expires", "0");
}

static void configureWebLogLevels(void)
{
    esp_log_level_set("httpd", ESP_LOG_WARN);
    esp_log_level_set("httpd_txrx", ESP_LOG_WARN);
    esp_log_level_set("httpd_parse", ESP_LOG_WARN);
    esp_log_level_set("httpd_uri", ESP_LOG_WARN);
    esp_log_level_set("httpd_sess", ESP_LOG_WARN);
    esp_log_level_set("event", ESP_LOG_WARN);
}

static void wifiCopyField(uint8_t *dst, size_t dstSize, const char *src)
{
    size_t copyLen = 0;

    if (dst == NULL || dstSize == 0) {
        return;
    }

    memset(dst, 0, dstSize);
    if (src == NULL) {
        return;
    }

    copyLen = strlen(src);
    if (copyLen >= dstSize) {
        copyLen = dstSize - 1;
    }
    memcpy(dst, src, copyLen);
}

static const char *lineToStr(int line)
{
    switch (line) {
        case LINE_CAN: return "CAN";
        case LINE_RS485: return "RS485";
        default: return "UNKNOWN";
    }
}

static const char *protocolToStr(int protocol)
{
    switch (protocol) {
        case PROTOCOL_CAN_GROWATT: return "CAN_GROWATT";
        case PROTOCOL_RS485_GROWATT: return "RS485_GROWATT";
        case PROTOCOL_RS485_PYLON: return "RS485_PYLON";
        case PROTOCOL_CAN_PYLON: return "CAN_PYLON";
        case PROTOCOL_CAN_DEYE: return "CAN_DEYE";
        default: return "UNKNOWN";
    }
}

static const char *modeToStr(int mode)
{
    switch (mode) {
        case MODE_SNIFFER: return "sniffer";
        case MODE_FORWARD: return "forward";
        case MODE_BRIDGE: return "bridge";
        default: return "unknown";
    }
}

static bool extractJsonInt(const char *json, const char *key, int *out)
{
    char pattern[32];
    const char *p = NULL;

    if (json == NULL || key == NULL || out == NULL) {
        return false;
    }

    snprintf(pattern, sizeof(pattern), "\"%s\":", key);
    p = strstr(json, pattern);
    if (p == NULL) {
        return false;
    }
    p += strlen(pattern);
    while (*p == ' ' || *p == '\t') {
        p++;
    }
    *out = atoi(p);
    return true;
}

static bool extractJsonString(const char *json, const char *key, char *out, size_t outSize)
{
    char pattern[32];
    const char *p = NULL;
    const char *end = NULL;
    size_t len = 0;

    if (json == NULL || key == NULL || out == NULL || outSize == 0) {
        return false;
    }

    snprintf(pattern, sizeof(pattern), "\"%s\":\"", key);
    p = strstr(json, pattern);
    if (p == NULL) {
        return false;
    }
    p += strlen(pattern);
    end = strchr(p, '"');
    if (end == NULL) {
        return false;
    }

    len = (size_t)(end - p);
    if (len >= outSize) {
        len = outSize - 1;
    }
    memcpy(out, p, len);
    out[len] = '\0';
    return true;
}

static void wifiApplyRuntimeSettings(void)
{
    bridge_runtime_settings_t settings = runtimeSettingsGet();
    wifi_config_t wifiConfig = { 0 };

    wifiCopyField(wifiConfig.sta.ssid, sizeof(wifiConfig.sta.ssid), settings.wifi_ssid);
    wifiCopyField(wifiConfig.sta.password, sizeof(wifiConfig.sta.password), settings.wifi_password);
    wifiConfig.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    if (s_wifiStaNetif != NULL) {
        esp_netif_set_hostname(s_wifiStaNetif, WIFI_STA_HOSTNAME);
    }

    esp_wifi_disconnect();
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifiConfig));
    esp_wifi_connect();
}

static void stopHttpServer(void)
{
    if (s_httpd != NULL) {
        httpd_stop(s_httpd);
        s_httpd = NULL;
    }
}

static void settingsApplyTask(void *pv)
{
    settingsApplyCtx_t ctx = *(settingsApplyCtx_t *)pv;

    vTaskDelay(pdMS_TO_TICKS(300));
    bridgeReloadFromRuntimeSettings();
    wifiApplyRuntimeSettings();

    if (ctx.restartWeb) {
        stopHttpServer();
        startHttpServer();
    }

    s_settingsApplyTask = NULL;
    vTaskDelete(NULL);
}

static void wifiEventHandler(void *arg,
                             esp_event_base_t event_base,
                             int32_t event_id,
                             void *event_data)
{
    (void)arg;

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
        return;
    }

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_wifiRetryCount < WIFI_STA_MAX_RETRY) {
            s_wifiRetryCount++;
            esp_wifi_connect();
            return;
        }
        xEventGroupSetBits(s_wifiEventGroup, WIFI_FAIL_BIT);
        return;
    }

    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        s_wifiRetryCount = 0;
        xEventGroupSetBits(s_wifiEventGroup, WIFI_CONNECTED_BIT);
        ESP_LOGI(WEB_TAG,
                 "Wi-Fi connected: IP=" IPSTR " hostname=%s",
                 IP2STR(&event->ip_info.ip),
                 WIFI_STA_HOSTNAME);
        ESP_LOGI(WEB_TAG,
                 "Open: http://" IPSTR "/  (hostname=%s if your router/DNS resolves it)",
                 IP2STR(&event->ip_info.ip),
                 WIFI_STA_HOSTNAME);
        ESP_LOGI(WEB_TAG,
                 "Hostname hint: http://%s/",
                 WIFI_STA_HOSTNAME);
    }
}

static esp_err_t rootHandler(httpd_req_t *req)
{
    static const char html[] =
        "<!DOCTYPE html><html><head><meta charset='utf-8'>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<title>ESP32 Bridge</title>"
        "<style>"
        "body{font-family:Arial,sans-serif;background:#101820;color:#f4f4f4;margin:0;padding:0}"
        ".tabs{display:flex;background:#172533}"
        ".tab{flex:1;padding:14px 16px;text-align:center;cursor:pointer;border:none;background:#172533;color:#d6e2ee}"
        ".tab.active{background:#1f8a70;color:#fff}"
        ".panel{display:none;padding:16px}"
        ".panel.active{display:block}"
        ".card{background:#16212b;border-radius:12px;padding:16px;margin-bottom:16px}"
        "table{width:100%;border-collapse:collapse}"
        "td{padding:8px;border-bottom:1px solid #2b3c4f;vertical-align:top}"
        "td:first-child{color:#8aa0b7;width:42%}"
        ".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(260px,1fr));gap:16px}"
        ".cell-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(120px,1fr));gap:10px}"
        ".cell-item{background:#0f1a24;border:1px solid #2b3c4f;border-radius:10px;padding:10px 12px}"
        ".cell-item.max{border-color:#1f8a70}"
        ".cell-item.min{border-color:#d17a22}"
        ".cell-item .label{display:block;color:#8aa0b7;margin-bottom:4px}"
        ".cell-item .value{display:block;font-family:Consolas,monospace}"
        ".alert-list{display:flex;flex-wrap:wrap;gap:8px}"
        ".badge{display:inline-block;background:#0f1a24;border:1px solid #2b3c4f;border-radius:999px;padding:6px 10px;font-family:Consolas,monospace;font-size:12px}"
        ".badge.prot{border-color:#d17a22;color:#ffd8a8}"
        ".badge.alarm{border-color:#c94f4f;color:#ffc6c6}"
        ".badge.warn{border-color:#c6a03d;color:#ffe9a8}"
        ".mono{font-family:Consolas,monospace}"
        "select,button{background:#0f1a24;color:#fff;border:1px solid #2b3c4f;border-radius:8px;padding:8px 10px}"
        "button{cursor:pointer;background:#1f8a70}"
        ".actions{display:flex;gap:12px;align-items:center;margin-top:16px}"
        "</style></head><body>"
        "<div class='tabs'>"
        "<button class='tab active' onclick='showTab(\"telemetry\",this)'>Telemetry</button>"
        "<button class='tab' onclick='showTab(\"settings\",this)'>Settings</button>"
        "<button class='tab' onclick='showTab(\"logs\",this)'>Logs</button>"
        "</div>"
        "<div id='telemetry' class='panel active'><div id='telemetryCards' class='grid'></div></div>"
        "<div id='settings' class='panel'><div class='card'><div id='settingsForm'>Loading...</div></div></div>"
        "<div id='logs' class='panel'><div class='card'><pre id='logsContent' class='mono' style='white-space:pre-wrap;margin:0'>Loading...</pre></div></div>"
        "<script>"
        "let currentTab='telemetry';"
        "function showTab(id,btn){document.querySelectorAll('.tab').forEach(t=>t.classList.remove('active'));"
        "document.querySelectorAll('.panel').forEach(p=>p.classList.remove('active'));"
        "if(btn){btn.classList.add('active');}"
        "currentTab=id;document.getElementById(id).classList.add('active');if(id==='settings'){loadSettings();}if(id==='logs'){refreshLogs();}}"
        "function row(k,v){return '<tr><td>'+k+'</td><td class=\"mono\">'+v+'</td></tr>';}"
        "function card(title,rows){return '<div class=\"card\"><h3>'+title+'</h3><table>'+rows.join('')+'</table></div>';}"
        "function cellGridCard(t){"
        "const count=(t.cell_count&&t.cell_count>0)?t.cell_count:(Array.isArray(t.cells_v)?t.cells_v.length:0);"
        "if(!count){return '<div class=\"card\"><h3>All Cells</h3><div class=\"mono\">No per-cell voltages available.</div></div>';}"
        "const items=[];"
        "for(let i=0;i<count;i++){"
        "const v=Array.isArray(t.cells_v)?t.cells_v[i]:0;"
        "const idx=i+1;"
        "let cls='cell-item';"
        "if(idx===t.cell_max_idx){cls+=' max';}"
        "if(idx===t.cell_min_idx){cls+=' min';}"
        "items.push('<div class=\"'+cls+'\"><span class=\"label\">Cell '+String(idx).padStart(2,'0')+'</span><span class=\"value\">'+(v>0?v.toFixed(3)+' V':'-')+'</span></div>');"
        "}"
        "return '<div class=\"card\"><h3>All Cells</h3><div class=\"cell-grid\">'+items.join('')+'</div></div>';"
        "}"
        "function alertBadges(title,kind,text){"
        "const items=(text&&text.trim())?text.split(/,\\s*/).filter(Boolean):[];"
        "if(!items.length){return '<div class=\"card\"><h3>'+title+'</h3><div class=\"mono\">None</div></div>';}"
        "return '<div class=\"card\"><h3>'+title+'</h3><div class=\"alert-list\">'+items.map(x=>'<span class=\"badge '+kind+'\">'+x+'</span>').join('')+'</div></div>';"
        "}"
        "function miscCard(t){"
        "if(t.protocol!=='CAN_DEYE'){return '';}"
        "return card('Miscellaneous',["
        "row('0x35C Raw','0x'+t.deye_status_35c.toString(16).toUpperCase().padStart(2,'0')),"
        "row('State Flags',t.state_flags&&t.state_flags.trim()?t.state_flags:'-'),"
        "row('0x371 Temp Max Sensor','#'+t.deye_temp_max_sensor),"
        "row('0x371 Temp Min Sensor','#'+t.deye_temp_min_sensor)"
        "]);"
        "}"
        "function renderTelemetry(t){"
        "const cards=["
        "card('Runtime',[row('Valid',t.valid?'YES':'NO'),row('Source',t.source),row('Protocol',t.protocol),row('Status 0x63',t.status_63)]),"
        "card('Pack',[row('Current',t.current_a.toFixed(2)+' A'),row('SOC',t.soc_pct+' %'),row('SOH',t.soh_pct+' %'),row('Cycles',t.cycles)]),"
        "card('Cells',[row('Cell Max',t.cell_max_v.toFixed(3)+' V @ #'+t.cell_max_idx),row('Cell Min',t.cell_min_v.toFixed(3)+' V @ #'+t.cell_min_idx),row('Delta',t.delta_v.toFixed(3)+' V')]),"
        "card('Temperatures',[row('MOS',t.temp_mos_c.toFixed(1)+' C'),row('T1',t.temp_t1_c.toFixed(1)+' C'),row('T2',t.temp_t2_c.toFixed(1)+' C'),row('T4',t.temp_t4_c.toFixed(1)+' C'),row('T5',t.temp_t5_c.toFixed(1)+' C')]),"
        "miscCard(t),"
        "cellGridCard(t),"
        "alertBadges('Protections','prot',t.protections),"
        "alertBadges('Alarms','alarm',t.alarms),"
        "alertBadges('Warnings','warn',t.warnings)"
        "];"
        "document.getElementById('telemetryCards').innerHTML=cards.filter(Boolean).join('');"
        "}"
        "function renderSettings(s){"
        "function sel(id,val,opts){return '<select id=\"'+id+'\">'+opts.map(o=>'<option value=\"'+o.value+'\"'+(String(o.value)===String(val)?' selected':'')+'>'+o.label+'</option>').join('')+'</select>';}"
        "const modeOpts=[{value:1,label:'sniffer'},{value:2,label:'forward'},{value:3,label:'bridge'}];"
        "const lineOpts=[{value:1,label:'CAN'},{value:2,label:'RS485'}];"
        "const protoOpts=[{value:1,label:'CAN_GROWATT'},{value:2,label:'RS485_GROWATT'},{value:3,label:'RS485_PYLON'},{value:4,label:'CAN_PYLON'},{value:5,label:'CAN_DEYE'}];"
        "const portOpts=[{value:1,label:'1'},{value:2,label:'2'}];"
        "const rows=["
        "row('Mode',sel('mode',s.mode_id,modeOpts)),"
        "row('BMS line',sel('bms_line',s.bms_line_id,lineOpts)),"
        "row('Inverter line',sel('inverter_line',s.inverter_line_id,lineOpts)),"
        "row('BMS protocol',sel('bms_protocol',s.bms_protocol_id,protoOpts)),"
        "row('Inverter protocol',sel('inverter_protocol',s.inverter_protocol_id,protoOpts)),"
        "row('BMS port',sel('bms_port',s.bms_port,portOpts)),"
        "row('Inverter port',sel('inverter_port',s.inverter_port,portOpts)),"
        "row('Wi-Fi SSID','<input id=\"wifi_ssid\" value=\"'+s.wifi_ssid+'\" />'),"
        "row('Wi-Fi PASS','<input id=\"wifi_password\" type=\"password\" value=\"'+s.wifi_password+'\" />'),"
        "row('Web port','<input id=\"web_port\" type=\"number\" min=\"1\" max=\"65535\" value=\"'+s.web_port+'\" />')"
        "];"
        "const actions='<div class=\"actions\"><button onclick=\"saveSettings()\">Save</button><span id=\"settingsStatus\" class=\"mono\"></span></div>';"
        "document.getElementById('settingsForm').innerHTML='<table>'+rows.join('')+'</table>'+actions;"
        "}"
        "async function saveSettings(){"
        "const payload={"
        "mode:parseInt(document.getElementById('mode').value,10),"
        "bms_line:parseInt(document.getElementById('bms_line').value,10),"
        "inverter_line:parseInt(document.getElementById('inverter_line').value,10),"
        "bms_protocol:parseInt(document.getElementById('bms_protocol').value,10),"
        "inverter_protocol:parseInt(document.getElementById('inverter_protocol').value,10),"
        "bms_port:parseInt(document.getElementById('bms_port').value,10),"
        "inverter_port:parseInt(document.getElementById('inverter_port').value,10),"
        "wifi_ssid:document.getElementById('wifi_ssid').value,"
        "wifi_password:document.getElementById('wifi_password').value,"
        "web_port:parseInt(document.getElementById('web_port').value,10)"
        "};"
        "document.getElementById('settingsStatus').textContent='Saving...';"
        "const res=await fetch('/api/settings',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(payload)});"
        "const body=await res.json();"
        "document.getElementById('settingsStatus').textContent=body.message||'done';"
        "if(body.ok){setTimeout(loadSettings,300);}"
        "}"
        "async function refreshTelemetry(){"
        "let t=await fetch('/api/telemetry?ts='+Date.now(),{cache:'no-store'}).then(r=>r.json());"
        "renderTelemetry(t);"
        "}"
        "async function loadSettings(){"
        "let s=await fetch('/api/settings?ts='+Date.now(),{cache:'no-store'}).then(r=>r.json());"
        "renderSettings(s);"
        "}"
        "async function refreshLogs(){"
        "const el=document.getElementById('logsContent');"
        "if(!el){return;}"
        "try{"
        "const res=await fetch('/api/logs?ts='+Date.now(),{cache:'no-store'});"
        "if(!res.ok){el.textContent='Log fetch failed: HTTP '+res.status;return;}"
        "const t=await res.text();"
        "el.textContent=t&&t.trim()?t:'No decoded BMS logs yet.';"
        "}catch(e){"
        "el.textContent='Log fetch failed: '+(e&&e.message?e.message:String(e));"
        "}"
        "}"
        "refreshTelemetry();refreshLogs();setInterval(refreshTelemetry,2000);"
        "setInterval(function(){if(currentTab==='logs'){refreshLogs();}},5000);"
        "</script></body></html>";

    setNoCacheHeaders(req);
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    return httpd_resp_send(req, html, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t telemetryHandler(httpd_req_t *req)
{
    bridgeTelemetrySnapshot_t snap = {0};
    char json[2048];
    int pos = 0;

    bridgeGetTelemetrySnapshot(&snap);

    pos += snprintf(json + pos,
                    sizeof(json) - pos,
                    "{"
                    "\"valid\":%s,"
                    "\"source\":\"%s\","
                    "\"protocol\":\"%s\","
                    "\"current_a\":%.2f,"
                    "\"soc_pct\":%u,"
                    "\"soh_pct\":%u,"
                    "\"cycles\":%u,"
                    "\"cell_max_v\":%.3f,"
                    "\"cell_min_v\":%.3f,"
                    "\"cell_max_idx\":%u,"
                    "\"cell_min_idx\":%u,"
                    "\"delta_v\":%.3f,"
                    "\"temp_mos_c\":%.1f,"
                    "\"temp_t1_c\":%.1f,"
                    "\"temp_t2_c\":%.1f,"
                    "\"temp_t4_c\":%.1f,"
                    "\"temp_t5_c\":%.1f,"
                    "\"status_63\":%u,"
                    "\"deye_status_35c\":%u,"
                    "\"deye_temp_max_sensor\":%u,"
                    "\"deye_temp_min_sensor\":%u,"
                    "\"state_flags\":\"%s\","
                    "\"cell_count\":%u,"
                    "\"protections\":\"%s\","
                    "\"alarms\":\"%s\","
                    "\"warnings\":\"%s\","
                    "\"cells_v\":[",
                    snap.valid ? "true" : "false",
                    snap.source,
                    snap.protocol,
                    (double)snap.currentA,
                    (unsigned)snap.socPct,
                    (unsigned)snap.sohPct,
                    (unsigned)snap.cycles,
                    (double)snap.cellMaxV,
                    (double)snap.cellMinV,
                    (unsigned)snap.cellMaxIdx,
                    (unsigned)snap.cellMinIdx,
                    (double)snap.deltaV,
                    (double)snap.tempMosC,
                    (double)snap.tempT1C,
                    (double)snap.tempT2C,
                    (double)snap.tempT4C,
                    (double)snap.tempT5C,
                    (unsigned)snap.pylonStatus63,
                    (unsigned)snap.deyeStatus35C,
                    (unsigned)snap.deyeTempMaxSensor,
                    (unsigned)snap.deyeTempMinSensor,
                    snap.stateFlags,
                    (unsigned)snap.cellCount,
                    snap.protections,
                    snap.alarms,
                    snap.warnings);

    for (uint8_t i = 0; i < snap.cellCount && pos < (int)sizeof(json); i++) {
        pos += snprintf(json + pos,
                        sizeof(json) - pos,
                        "%s%.3f",
                        (i == 0u) ? "" : ",",
                        (double)snap.cellVoltagesV[i]);
    }

    snprintf(json + pos, sizeof(json) - pos, "]}");

    setNoCacheHeaders(req);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t logsHandler(httpd_req_t *req)
{
    bridgeGetDecodedLogSnapshot(s_logsResponse, sizeof(s_logsResponse));
    ESP_LOGI(WEB_TAG, "/api/logs requested (len=%u)", (unsigned)strlen(s_logsResponse));
    setNoCacheHeaders(req);
    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    return httpd_resp_send(req,
                           s_logsResponse[0] != '\0' ? s_logsResponse : "No decoded BMS logs yet.",
                           HTTPD_RESP_USE_STRLEN);
}

static esp_err_t settingsHandler(httpd_req_t *req)
{
    bridge_runtime_settings_t settings = runtimeSettingsGet();
    char json[768];

    snprintf(json,
             sizeof(json),
             "{"
             "\"mode\":\"%s\","
             "\"mode_id\":%u,"
             "\"bms_line\":\"%s\","
             "\"bms_line_id\":%u,"
             "\"inverter_line\":\"%s\","
             "\"inverter_line_id\":%u,"
             "\"bms_protocol\":\"%s\","
             "\"bms_protocol_id\":%u,"
             "\"inverter_protocol\":\"%s\","
             "\"inverter_protocol_id\":%u,"
             "\"bms_port\":%d,"
             "\"inverter_port\":%d,"
             "\"wifi_ssid\":\"%s\","
             "\"wifi_password\":\"%s\","
             "\"web_port\":%d"
             "}",
             modeToStr(settings.mode),
             (unsigned)settings.mode,
             lineToStr(settings.bms_line),
             (unsigned)settings.bms_line,
             lineToStr(settings.inverter_line),
             (unsigned)settings.inverter_line,
             protocolToStr(settings.bms_protocol),
             (unsigned)settings.bms_protocol,
             protocolToStr(settings.inverter_protocol),
             (unsigned)settings.inverter_protocol,
             settings.bms_port,
             settings.inverter_port,
             settings.wifi_ssid,
             settings.wifi_password,
             settings.web_port);

    setNoCacheHeaders(req);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t settingsPostHandler(httpd_req_t *req)
{
    char buf[256];
    int received = httpd_req_recv(req, buf, sizeof(buf) - 1);
    bridge_runtime_settings_t settings = runtimeSettingsGet();
    bridge_runtime_settings_t oldSettings = settings;
    int v = 0;
    const char *okResp = "{\"ok\":true,\"message\":\"Saved and applied\"}";
    const char *errResp = "{\"ok\":false,\"message\":\"Invalid settings\"}";

    if (received <= 0) {
        httpd_resp_set_status(req, "400 Bad Request");
        setNoCacheHeaders(req);
        httpd_resp_set_type(req, "application/json");
        return httpd_resp_send(req, errResp, HTTPD_RESP_USE_STRLEN);
    }

    buf[received] = '\0';

    if (extractJsonInt(buf, "mode", &v)) settings.mode = (uint8_t)v;
    if (extractJsonInt(buf, "bms_line", &v)) settings.bms_line = (uint8_t)v;
    if (extractJsonInt(buf, "inverter_line", &v)) settings.inverter_line = (uint8_t)v;
    if (extractJsonInt(buf, "bms_protocol", &v)) settings.bms_protocol = (uint8_t)v;
    if (extractJsonInt(buf, "inverter_protocol", &v)) settings.inverter_protocol = (uint8_t)v;
    if (extractJsonInt(buf, "bms_port", &v)) settings.bms_port = (uint8_t)v;
    if (extractJsonInt(buf, "inverter_port", &v)) settings.inverter_port = (uint8_t)v;
    (void)extractJsonString(buf, "wifi_ssid", settings.wifi_ssid, sizeof(settings.wifi_ssid));
    (void)extractJsonString(buf, "wifi_password", settings.wifi_password, sizeof(settings.wifi_password));
    if (extractJsonInt(buf, "web_port", &v)) settings.web_port = (uint16_t)v;

    if (!runtimeSettingsSave(&settings)) {
        httpd_resp_set_status(req, "400 Bad Request");
        setNoCacheHeaders(req);
        httpd_resp_set_type(req, "application/json");
        return httpd_resp_send(req, errResp, HTTPD_RESP_USE_STRLEN);
    }

    setNoCacheHeaders(req);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, okResp, HTTPD_RESP_USE_STRLEN);

    if (s_settingsApplyTask != NULL) {
        vTaskDelete(s_settingsApplyTask);
        s_settingsApplyTask = NULL;
    }

    static settingsApplyCtx_t applyCtx;
    applyCtx.restartWeb = (settings.web_port != oldSettings.web_port);

    xTaskCreate(
        settingsApplyTask,
        "settings_apply",
        4096,
        &applyCtx,
        5,
        &s_settingsApplyTask);

    return ESP_OK;
}

static void startHttpServer(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = runtimeSettingsGet().web_port;
    config.stack_size = 8192;

    if (httpd_start(&s_httpd, &config) != ESP_OK) {
        ESP_LOGE(WEB_TAG, "Failed to start HTTP server");
        return;
    }

    httpd_uri_t rootUri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = rootHandler,
        .user_ctx = NULL
    };
    httpd_uri_t telemetryUri = {
        .uri = "/api/telemetry",
        .method = HTTP_GET,
        .handler = telemetryHandler,
        .user_ctx = NULL
    };
    httpd_uri_t logsUri = {
        .uri = "/api/logs",
        .method = HTTP_GET,
        .handler = logsHandler,
        .user_ctx = NULL
    };
    httpd_uri_t settingsUri = {
        .uri = "/api/settings",
        .method = HTTP_GET,
        .handler = settingsHandler,
        .user_ctx = NULL
    };
    httpd_uri_t settingsPostUri = {
        .uri = "/api/settings",
        .method = HTTP_POST,
        .handler = settingsPostHandler,
        .user_ctx = NULL
    };

    httpd_register_uri_handler(s_httpd, &rootUri);
    httpd_register_uri_handler(s_httpd, &telemetryUri);
    httpd_register_uri_handler(s_httpd, &logsUri);
    httpd_register_uri_handler(s_httpd, &settingsUri);
    httpd_register_uri_handler(s_httpd, &settingsPostUri);
}

static void initWifiSta(void)
{
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    wifi_config_t wifiConfig = { 0 };
    bridge_runtime_settings_t settings = runtimeSettingsGet();

    wifiCopyField(wifiConfig.sta.ssid, sizeof(wifiConfig.sta.ssid), settings.wifi_ssid);
    wifiCopyField(wifiConfig.sta.password, sizeof(wifiConfig.sta.password), settings.wifi_password);
    wifiConfig.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    s_wifiStaNetif = esp_netif_create_default_wifi_sta();
    if (s_wifiStaNetif != NULL) {
        esp_netif_set_hostname(s_wifiStaNetif, WIFI_STA_HOSTNAME);
    }

    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifiEventHandler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifiEventHandler, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifiConfig));
    ESP_ERROR_CHECK(esp_wifi_start());
}

static void webInterfaceTask(void *pv)
{
    (void)pv;

    configureWebLogLevels();
    s_wifiEventGroup = xEventGroupCreate();
    initWifiSta();
    startHttpServer();

    ESP_LOGI(WEB_TAG, "Web interface task started (STA ssid=%s port=%d)", runtimeSettingsGet().wifi_ssid, runtimeSettingsGet().web_port);

    while (1) {
        EventBits_t bits = xEventGroupGetBits(s_wifiEventGroup);
        if (bits & WIFI_CONNECTED_BIT) {
            vTaskDelay(pdMS_TO_TICKS(5000));
        } else if (bits & WIFI_FAIL_BIT) {
            ESP_LOGW(WEB_TAG, "Wi-Fi connection failed after %d retries", WIFI_STA_MAX_RETRY);
            vTaskDelay(pdMS_TO_TICKS(10000));
        } else {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}

void webInterfaceStartTask(void)
{
#if WEB_INTERFACE_ENABLE
    xTaskCreate(webInterfaceTask,
                "web_interface",
                WEB_INTERFACE_TASK_STACK,
                NULL,
                WEB_INTERFACE_TASK_PRIO,
                NULL);
#endif
}
