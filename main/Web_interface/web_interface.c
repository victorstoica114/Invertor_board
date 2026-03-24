#include "Web_interface/web_interface.h"

#include <stdio.h>
#include <string.h>

#include "Working_modes.h"
#include "config.h"
#include "orchestrator/protocol_types.h"
#include "protocols/growatt/growatt_bms_task.h"

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
static int s_wifiRetryCount;
static httpd_handle_t s_httpd;
static esp_netif_t *s_wifiStaNetif;

static void setNoCacheHeaders(httpd_req_t *req)
{
    if (req == NULL) {
        return;
    }

    httpd_resp_set_hdr(req, "Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
    httpd_resp_set_hdr(req, "Pragma", "no-cache");
    httpd_resp_set_hdr(req, "Expires", "0");
}

static const char *protocolFromPacket(const bms_decoded_packet_t *packet)
{
    if (packet == NULL) {
        return "UNKNOWN";
    }
    return protocolIdToStr(packet->sourceProtocol);
}

static esp_err_t rootHandler(httpd_req_t *req)
{
    static const char html[] =
        "<!DOCTYPE html><html><head><meta charset='utf-8'>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<title>ESP32 Bridge</title>"
        "<style>"
        "body{font-family:Arial,sans-serif;background:#101820;color:#f4f4f4;margin:0;padding:0}"
        ".wrap{max-width:980px;margin:0 auto;padding:18px}"
        ".card{background:#16212b;border-radius:12px;padding:16px;margin-bottom:16px}"
        "table{width:100%;border-collapse:collapse}"
        "td{padding:8px;border-bottom:1px solid #2b3c4f;vertical-align:top}"
        "td:first-child{color:#8aa0b7;width:42%}"
        ".mono{font-family:Consolas,monospace}"
        "h1{font-size:24px;margin:0 0 12px 0}"
        "h3{margin:0 0 10px 0}"
        ".muted{color:#8aa0b7}"
        "</style></head><body><div class='wrap'>"
        "<h1>ESP32 Bridge Web</h1>"
        "<div class='card'><h3>Runtime</h3><table id='runtimeTbl'></table></div>"
        "<div class='card'><h3>BMS Packet</h3><table id='pktTbl'></table></div>"
        "<div class='muted'>Auto refresh: 2s</div>"
        "</div><script>"
        "function row(k,v){return '<tr><td>'+k+'</td><td class=\"mono\">'+v+'</td></tr>';}"
        "function asNum(v,d){return (typeof v==='number')?v:d;}"
        "function asBool(v){return v?'YES':'NO';}"
        "async function refresh(){"
        "const t=await fetch('/api/telemetry?ts='+Date.now(),{cache:'no-store'}).then(r=>r.json());"
        "document.getElementById('runtimeTbl').innerHTML=["
        "row('Working mode',t.mode),"
        "row('BMS protocol',t.active_bms_protocol),"
        "row('Inverter protocol',t.active_inverter_protocol),"
        "row('Has packet',asBool(t.has_packet))"
        "].join('');"
        "document.getElementById('pktTbl').innerHTML=["
        "row('Source protocol',t.source_protocol),"
        "row('Sequence',asNum(t.sequence,0)),"
        "row('Timestamp (us)',asNum(t.timestamp_us,0)),"
        "row('SOC',t.has_soc?asNum(t.soc_pct,0)+' %':'-'),"
        "row('Temperature',t.has_temperature?asNum(t.temperature_c,0)+' C':'-'),"
        "row('Pack voltage',t.has_pack_voltage?(asNum(t.pack_voltage_cv,0)/100.0).toFixed(2)+' V':'-'),"
        "row('Cell min',t.has_cell_extremes?(asNum(t.min_cell_mv,0)/1000.0).toFixed(3)+' V (#'+asNum(t.min_cell_index,0)+')':'-'),"
        "row('Cell max',t.has_cell_extremes?(asNum(t.max_cell_mv,0)/1000.0).toFixed(3)+' V (#'+asNum(t.max_cell_index,0)+')':'-')"
        "].join('');"
        "}"
        "refresh();setInterval(refresh,2000);"
        "</script></body></html>";

    setNoCacheHeaders(req);
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    return httpd_resp_send(req, html, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t telemetryHandler(httpd_req_t *req)
{
    bms_decoded_packet_t packet = {0};
    const bool hasPacket = growattBmsTaskGetLatestPacket(&packet);
    char json[1024];

    snprintf(json,
             sizeof(json),
             "{"
             "\"mode\":\"%s\","
             "\"active_bms_protocol\":\"%s\","
             "\"active_inverter_protocol\":\"%s\","
             "\"has_packet\":%s,"
             "\"source_protocol\":\"%s\","
             "\"sequence\":%lu,"
             "\"timestamp_us\":%lld,"
             "\"has_soc\":%s,"
             "\"soc_pct\":%u,"
             "\"has_temperature\":%s,"
             "\"temperature_c\":%d,"
             "\"has_pack_voltage\":%s,"
             "\"pack_voltage_cv\":%u,"
             "\"has_cell_extremes\":%s,"
             "\"min_cell_mv\":%u,"
             "\"max_cell_mv\":%u,"
             "\"min_cell_index\":%u,"
             "\"max_cell_index\":%u"
             "}",
             workingModeToStr((working_mode_t)ACTIVE_WORKING_MODE),
             protocolIdToStr((protocol_id_t)ACTIVE_BMS_PROTOCOL),
             protocolIdToStr((protocol_id_t)ACTIVE_INVERTER_PROTOCOL),
             hasPacket ? "true" : "false",
             protocolFromPacket(&packet),
             (unsigned long)packet.sequence,
             (long long)packet.timestampUs,
             packet.hasSoc ? "true" : "false",
             (unsigned)packet.socPct,
             packet.hasTemperatureC ? "true" : "false",
             (int)packet.temperatureC,
             packet.hasPackVoltageCv ? "true" : "false",
             (unsigned)packet.packVoltageCv,
             packet.hasCellExtremes ? "true" : "false",
             (unsigned)packet.minCellMv,
             (unsigned)packet.maxCellMv,
             (unsigned)packet.minCellIndex,
             (unsigned)packet.maxCellIndex);

    setNoCacheHeaders(req);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
}

static void startHttpServer(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = WEB_INTERFACE_PORT;
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

    httpd_register_uri_handler(s_httpd, &rootUri);
    httpd_register_uri_handler(s_httpd, &telemetryUri);
}

static void wifiEventHandler(void *arg,
                             esp_event_base_t eventBase,
                             int32_t eventId,
                             void *eventData)
{
    (void)arg;

    if (eventBase == WIFI_EVENT && eventId == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
        return;
    }

    if (eventBase == WIFI_EVENT && eventId == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_wifiRetryCount < WIFI_STA_MAX_RETRY) {
            s_wifiRetryCount++;
            esp_wifi_connect();
            return;
        }
        xEventGroupSetBits(s_wifiEventGroup, WIFI_FAIL_BIT);
        return;
    }

    if (eventBase == IP_EVENT && eventId == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)eventData;
        s_wifiRetryCount = 0;
        xEventGroupSetBits(s_wifiEventGroup, WIFI_CONNECTED_BIT);
        ESP_LOGI(WEB_TAG,
                 "Wi-Fi connected: IP=" IPSTR " hostname=%s",
                 IP2STR(&event->ip_info.ip),
                 WIFI_STA_HOSTNAME);
        ESP_LOGI(WEB_TAG, "Open: http://" IPSTR ":%u/", IP2STR(&event->ip_info.ip), WEB_INTERFACE_PORT);
    }
}

static void initWifiSta(void)
{
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    wifi_config_t wifiConfig = { 0 };
    size_t ssidLen = strlen(WIFI_STA_SSID);
    size_t passLen = strlen(WIFI_STA_PASSWORD);

    if (ssidLen >= sizeof(wifiConfig.sta.ssid)) {
        ssidLen = sizeof(wifiConfig.sta.ssid) - 1u;
    }
    if (passLen >= sizeof(wifiConfig.sta.password)) {
        passLen = sizeof(wifiConfig.sta.password) - 1u;
    }

    memcpy(wifiConfig.sta.ssid, WIFI_STA_SSID, ssidLen);
    memcpy(wifiConfig.sta.password, WIFI_STA_PASSWORD, passLen);
    wifiConfig.sta.threshold.authmode = (passLen > 0u) ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;

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

    s_wifiEventGroup = xEventGroupCreate();
    initWifiSta();
    startHttpServer();

    ESP_LOGI(WEB_TAG,
             "Web interface started (ssid=%s, port=%u)",
             WIFI_STA_SSID,
             WEB_INTERFACE_PORT);

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
