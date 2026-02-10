#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <stdarg.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_eth.h"
#include "nvs_flash.h"
#include "esp_timer.h"
#include "esp_system.h"

/* OTA + HTTP server */
#include "esp_http_server.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_app_format.h"

/* Project components */
#include "ethernet_init.h"
#include "mqtt.h"
#include "cc1101.h"

/* Auto-generated build info (main/generated/build_info.h) */
#include "build_info.h"

static const char *TAG = "apollo_p4";

/* ================= MQTT SETTINGS ================= */
#define MQTT_HOST     "192.168.1.10"
#define MQTT_PORT     1883
#define MQTT_USERNAME "rflogger"
#define MQTT_PASSWORD "rflogger"

/* ================= OTA SETTINGS ================= */
#define OTA_PORT   3232
#define OTA_PATH   "/update"
#define OTA_TOKEN  "ota-esp32-p4-update"

/* Mark image VALID after N seconds of “healthy runtime” (after IP+MQTT are up) */
#define OTA_MARK_VALID_AFTER_SEC  20

/* Enforce content type for OTA uploads */
#define OTA_REQUIRED_CONTENT_TYPE "application/octet-stream"

/* ================= Home Assistant ================= */
#define HA_DISCOVERY_PREFIX "homeassistant"

/* ================= Device identity ================= */
#define DEVICE_ID    "rf-logger-node"
#define DEVICE_NAME  "Apollo P4 Ethernet"

/* ================= Topics ================= */
#define BASE_TOPIC          "apollo_p4/" DEVICE_ID
#define AVAIL_TOPIC         BASE_TOPIC "/status"
#define IP_STATE_TOPIC      BASE_TOPIC "/ip"
#define UPTIME_STATE_TOPIC  BASE_TOPIC "/uptime"

#define RF_RAW_TOPIC        BASE_TOPIC "/rf_raw"
#define RF_LAST_TOPIC       BASE_TOPIC "/rf_last"
#define APOLLO_PKT_TOPIC    BASE_TOPIC "/apollo_pkt"
#define OIL_DEPTH_TOPIC     BASE_TOPIC "/oil_depth_raw"
#define OIL_CRC_OK_TOPIC    BASE_TOPIC "/oil_crc_ok"
#define LAST_SEEN_TOPIC     BASE_TOPIC "/last_seen"

#define LOG_TOPIC           BASE_TOPIC "/log"

/* HA discovery topics */
#define IP_DISC_TOPIC         HA_DISCOVERY_PREFIX "/sensor/" DEVICE_ID "/ip/config"
#define UPTIME_DISC_TOPIC     HA_DISCOVERY_PREFIX "/sensor/" DEVICE_ID "/uptime/config"
#define RF_LAST_DISC_TOPIC    HA_DISCOVERY_PREFIX "/sensor/" DEVICE_ID "/rf_last/config"
#define OIL_DEPTH_DISC_TOPIC  HA_DISCOVERY_PREFIX "/sensor/" DEVICE_ID "/oil_depth_raw/config"
#define LAST_SEEN_DISC_TOPIC  HA_DISCOVERY_PREFIX "/sensor/" DEVICE_ID "/last_seen/config"

/* Publish uptime every N seconds */
#define UPTIME_PUBLISH_PERIOD_SEC  10

static esp_netif_t *s_eth_netif = NULL;
static esp_eth_handle_t *s_eth_handles = NULL;
static uint8_t s_eth_count = 0;

static char s_ip_str[32] = "0.0.0.0";
static TimerHandle_t s_uptime_timer = NULL;

/* OTA HTTP server handle */
static httpd_handle_t s_ota_httpd = NULL;

/* Health gating for OTA VALID */
static EventGroupHandle_t s_health_eg = NULL;
#define BIT_GOT_IP        (1 << 0)
#define BIT_MQTT_UP       (1 << 1)

/* OTA in-progress guard */
static bool s_ota_in_progress = false;

/* ============== helpers ============== */

static void mqtt_log(const char *fmt, ...)
{
    if (!mqtt_is_connected()) return;

    char buf[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    mqtt_publish(LOG_TOPIC, buf, 0, false);
}

static void publish_availability(bool online)
{
    if (!mqtt_is_connected()) return;
    mqtt_publish(AVAIL_TOPIC, online ? "online" : "offline", 1, true);
}

static void publish_ip_state(void)
{
    if (!mqtt_is_connected()) return;
    mqtt_publish(IP_STATE_TOPIC, s_ip_str, 0, true);
}

static void publish_uptime_state(void)
{
    if (!mqtt_is_connected()) return;

    int64_t sec = esp_timer_get_time() / 1000000LL;
    char buf[32];
    snprintf(buf, sizeof(buf), "%" PRId64, sec);
    mqtt_publish(UPTIME_STATE_TOPIC, buf, 0, false);
}

static void bytes_to_hex(const uint8_t *in, size_t len, char *out, size_t out_len)
{
    if (!out || out_len == 0) return;
    out[0] = '\0';

    size_t used = 0;
    for (size_t i = 0; i < len; i++) {
        if (used + 2 + 1 > out_len) break;
        snprintf(out + used, out_len - used, "%02X", in[i]);
        used += 2;
    }
}

/* Dallas / 1-Wire CRC8 (poly 0x8C, reflected) */
static uint8_t crc8_1wire(const uint8_t *data, size_t len)
{
    uint8_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        uint8_t inbyte = data[i];
        for (uint8_t j = 0; j < 8; j++) {
            uint8_t mix = (crc ^ inbyte) & 0x01;
            crc >>= 1;
            if (mix) crc ^= 0x8C;
            inbyte >>= 1;
        }
    }
    return crc;
}

static bool is_trivial_false_positive(const uint8_t *p8)
{
    int zeros = 0;
    int same  = 0;
    for (int i = 0; i < 8; i++) {
        if (p8[i] == 0x00) zeros++;
        if (p8[i] == p8[0]) same++;
    }
    if (zeros >= 7) return true;
    if (same  >= 7) return true;
    return false;
}

static bool find_apollo_packet(const uint8_t *buf, size_t len, uint8_t out_pkt[8], size_t *out_offset)
{
    if (len < 8) return false;

    for (size_t off = 0; off + 8 <= len; off++) {
        const uint8_t *p = &buf[off];
        uint8_t calc = crc8_1wire(p, 7);
        if (calc != p[7]) continue;
        if (is_trivial_false_positive(p)) continue;

        memcpy(out_pkt, p, 8);
        if (out_offset) *out_offset = off;
        return true;
    }

    return false;
}

/* ============== OTA (HTTP push) ============== */

static bool ota_check_token(httpd_req_t *req)
{
    char token[128] = {0};
    if (httpd_req_get_hdr_value_str(req, "X-OTA-Token", token, sizeof(token)) != ESP_OK) {
        return false;
    }
    return (strcmp(token, OTA_TOKEN) == 0);
}

static bool ota_check_content_type(httpd_req_t *req)
{
    char ct[96] = {0};
    if (httpd_req_get_hdr_value_str(req, "Content-Type", ct, sizeof(ct)) != ESP_OK) {
        return false;
    }
    /* Some clients may append charset; keep this strict for now */
    return (strcmp(ct, OTA_REQUIRED_CONTENT_TYPE) == 0);
}

static const char *ota_state_str(esp_ota_img_states_t s)
{
    switch (s) {
        case ESP_OTA_IMG_NEW:             return "NEW";
        case ESP_OTA_IMG_PENDING_VERIFY:  return "PENDING_VERIFY";
        case ESP_OTA_IMG_VALID:           return "VALID";
        case ESP_OTA_IMG_INVALID:         return "INVALID";
        case ESP_OTA_IMG_ABORTED:         return "ABORTED";
        case ESP_OTA_IMG_UNDEFINED:       return "UNDEFINED";
        default:                          return "UNKNOWN";
    }
}

static esp_err_t ota_version_get_handler(httpd_req_t *req)
{
    const esp_app_desc_t *desc = esp_app_get_description();

    char out[768];
    snprintf(out, sizeof(out),
             "project=%s\n"
             "version=%s\n"
             "build=%d\n"
             "utc=%s\n"
             "idf=%s\n",
             desc->project_name,
             APP_VERSION_STRING,
             (int)APP_BUILD_NUMBER,
             APP_BUILD_UTC,
             desc->idf_ver);

    httpd_resp_set_type(req, "text/plain");
    httpd_resp_sendstr(req, out);
    return ESP_OK;
}

static esp_err_t ota_status_get_handler(httpd_req_t *req)
{
    const esp_partition_t *run = esp_ota_get_running_partition();
    const esp_partition_t *upd = esp_ota_get_next_update_partition(NULL);

    esp_ota_img_states_t run_state = ESP_OTA_IMG_UNDEFINED;
    esp_ota_img_states_t upd_state = ESP_OTA_IMG_UNDEFINED;

    if (run) esp_ota_get_state_partition(run, &run_state);
    if (upd) esp_ota_get_state_partition(upd, &upd_state);

    char out[512];
    snprintf(out, sizeof(out),
             "running=%s addr=0x%08" PRIx32 " size=0x%08" PRIx32 " state=%s\n"
             "update_target=%s addr=0x%08" PRIx32 " size=0x%08" PRIx32 " state=%s\n"
             "ota_in_progress=%s\n",
             run ? run->label : "NULL",
             run ? run->address : 0,
             run ? run->size : 0,
             ota_state_str(run_state),
             upd ? upd->label : "NULL",
             upd ? upd->address : 0,
             upd ? upd->size : 0,
             ota_state_str(upd_state),
             s_ota_in_progress ? "true" : "false");

    httpd_resp_set_type(req, "text/plain");
    httpd_resp_sendstr(req, out);
    return ESP_OK;
}

static esp_err_t ota_reboot_post_handler(httpd_req_t *req)
{
    if (!ota_check_token(req)) {
        httpd_resp_set_status(req, "401 Unauthorized");
        httpd_resp_sendstr(req, "Missing/invalid X-OTA-Token\n");
        return ESP_OK;
    }

    httpd_resp_set_type(req, "text/plain");
    httpd_resp_sendstr(req, "OK, rebooting...\n");
    mqtt_log("HTTP: reboot requested");

    vTaskDelay(pdMS_TO_TICKS(800));
    esp_restart();
    return ESP_OK;
}

static esp_err_t ota_update_post_handler(httpd_req_t *req)
{
    if (!ota_check_token(req)) {
        httpd_resp_set_status(req, "401 Unauthorized");
        httpd_resp_sendstr(req, "Missing/invalid X-OTA-Token\n");
        return ESP_OK;
    }

    if (!ota_check_content_type(req)) {
        httpd_resp_set_status(req, "415 Unsupported Media Type");
        httpd_resp_sendstr(req, "Content-Type must be application/octet-stream\n");
        return ESP_OK;
    }

    if (s_ota_in_progress) {
        httpd_resp_set_status(req, "409 Conflict");
        httpd_resp_sendstr(req, "OTA already in progress\n");
        return ESP_OK;
    }
    s_ota_in_progress = true;

    const esp_partition_t *update_part = esp_ota_get_next_update_partition(NULL);
    if (!update_part) {
        s_ota_in_progress = false;
        httpd_resp_set_status(req, "500 Internal Server Error");
        httpd_resp_sendstr(req, "No OTA partition available\n");
        return ESP_OK;
    }

    if (req->content_len <= 0) {
        s_ota_in_progress = false;
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_sendstr(req, "Empty body\n");
        return ESP_OK;
    }

    if ((size_t)req->content_len > update_part->size) {
        s_ota_in_progress = false;
        httpd_resp_set_status(req, "413 Payload Too Large");
        httpd_resp_sendstr(req, "Image too large for OTA partition\n");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "OTA: writing to '%s' at 0x%08" PRIx32 ", size=0x%08" PRIx32 " (content_len=%d)",
             update_part->label, update_part->address, update_part->size, req->content_len);
    mqtt_log("OTA: writing to '%s' addr=0x%08" PRIx32 " size=0x%08" PRIx32 " len=%d",
             update_part->label, update_part->address, update_part->size, req->content_len);

    esp_ota_handle_t ota_handle = 0;
    esp_err_t err = esp_ota_begin(update_part, OTA_SIZE_UNKNOWN, &ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin failed: %s (0x%x)", esp_err_to_name(err), err);
        mqtt_log("esp_ota_begin failed: %s (0x%x)", esp_err_to_name(err), err);

        s_ota_in_progress = false;
        httpd_resp_set_status(req, "500 Internal Server Error");
        char msg[128];
        snprintf(msg, sizeof(msg), "esp_ota_begin failed: %s (0x%x)\n", esp_err_to_name(err), err);
        httpd_resp_sendstr(req, msg);
        return ESP_OK;
    }

    char buf[2048];
    int remaining = req->content_len;

    while (remaining > 0) {
        int want = remaining > (int)sizeof(buf) ? (int)sizeof(buf) : remaining;
        int recv_len = httpd_req_recv(req, buf, want);
        if (recv_len <= 0) {
            esp_ota_abort(ota_handle);
            s_ota_in_progress = false;
            httpd_resp_set_status(req, "500 Internal Server Error");
            httpd_resp_sendstr(req, "Receive failed\n");
            return ESP_OK;
        }

        err = esp_ota_write(ota_handle, buf, recv_len);
        if (err != ESP_OK) {
            esp_ota_abort(ota_handle);
            s_ota_in_progress = false;
            httpd_resp_set_status(req, "500 Internal Server Error");
            httpd_resp_sendstr(req, "esp_ota_write failed\n");
            return ESP_OK;
        }

        remaining -= recv_len;
    }

    err = esp_ota_end(ota_handle);
    if (err != ESP_OK) {
        s_ota_in_progress = false;
        httpd_resp_set_status(req, "500 Internal Server Error");
        httpd_resp_sendstr(req, "esp_ota_end failed\n");
        return ESP_OK;
    }

    err = esp_ota_set_boot_partition(update_part);
    if (err != ESP_OK) {
        s_ota_in_progress = false;
        httpd_resp_set_status(req, "500 Internal Server Error");
        httpd_resp_sendstr(req, "esp_ota_set_boot_partition failed\n");
        return ESP_OK;
    }

    httpd_resp_set_type(req, "text/plain");
    httpd_resp_sendstr(req, "OK, rebooting...\n");
    mqtt_log("OTA: OK, rebooting");

    /* Give curl time to read the response before we reset the socket */
    vTaskDelay(pdMS_TO_TICKS(900));
    esp_restart();
    return ESP_OK;
}

static void ota_http_server_start(void)
{
    if (s_ota_httpd) return;

    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port = OTA_PORT;
    cfg.stack_size = 8192;
    cfg.recv_wait_timeout = 15;
    cfg.send_wait_timeout = 15;
    cfg.lru_purge_enable = true;

    esp_err_t err = httpd_start(&s_ota_httpd, &cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "OTA httpd start failed: %s", esp_err_to_name(err));
        mqtt_log("OTA httpd start failed: %s", esp_err_to_name(err));
        s_ota_httpd = NULL;
        return;
    }

    httpd_uri_t uri_update = {
        .uri = OTA_PATH,
        .method = HTTP_POST,
        .handler = ota_update_post_handler,
        .user_ctx = NULL
    };

    httpd_uri_t uri_version = {
        .uri = "/version",
        .method = HTTP_GET,
        .handler = ota_version_get_handler,
        .user_ctx = NULL
    };

    httpd_uri_t uri_ota = {
        .uri = "/ota",
        .method = HTTP_GET,
        .handler = ota_status_get_handler,
        .user_ctx = NULL
    };

    httpd_uri_t uri_reboot = {
        .uri = "/reboot",
        .method = HTTP_POST,
        .handler = ota_reboot_post_handler,
        .user_ctx = NULL
    };

    httpd_register_uri_handler(s_ota_httpd, &uri_update);
    httpd_register_uri_handler(s_ota_httpd, &uri_version);
    httpd_register_uri_handler(s_ota_httpd, &uri_ota);
    httpd_register_uri_handler(s_ota_httpd, &uri_reboot);

    ESP_LOGI(TAG, "OTA ready: http://%s:%d%s  (GET /version, /ota; POST /reboot)",
             s_ip_str, OTA_PORT, OTA_PATH);
    mqtt_log("OTA ready: http://%s:%d%s", s_ip_str, OTA_PORT, OTA_PATH);
}

/* Mark image VALID after some “healthy runtime” */
static void ota_mark_valid_task(void *arg)
{
    (void)arg;

    /* Wait until we have both IP + MQTT, then wait N seconds */
    xEventGroupWaitBits(s_health_eg, BIT_GOT_IP | BIT_MQTT_UP, pdFALSE, pdTRUE, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(OTA_MARK_VALID_AFTER_SEC * 1000));

    const esp_partition_t *run = esp_ota_get_running_partition();
    esp_ota_img_states_t st = ESP_OTA_IMG_UNDEFINED;

    if (run && esp_ota_get_state_partition(run, &st) == ESP_OK) {
        if (st == ESP_OTA_IMG_PENDING_VERIFY) {
            ESP_LOGW(TAG, "OTA: running image is PENDING_VERIFY -> marking VALID");
            mqtt_log("OTA: marking running image VALID");
            esp_err_t e = esp_ota_mark_app_valid_cancel_rollback();
            if (e == ESP_OK) {
                ESP_LOGI(TAG, "OTA: marked VALID");
                mqtt_log("OTA: marked VALID");
            } else {
                ESP_LOGE(TAG, "OTA: mark VALID failed: %s", esp_err_to_name(e));
                mqtt_log("OTA: mark VALID failed: %s", esp_err_to_name(e));
            }
        } else {
            ESP_LOGI(TAG, "OTA: running image state=%s (no action)", ota_state_str(st));
        }
    }

    vTaskDelete(NULL);
}

/* ============== HA discovery ============== */

static void publish_discovery(void)
{
    if (!mqtt_is_connected()) return;

    char ip_cfg[512];
    snprintf(ip_cfg, sizeof(ip_cfg),
        "{"
          "\"name\":\"%s IP\","
          "\"unique_id\":\"%s_ip\","
          "\"state_topic\":\"%s\","
          "\"availability_topic\":\"%s\","
          "\"payload_available\":\"online\","
          "\"payload_not_available\":\"offline\","
          "\"device\":{"
            "\"identifiers\":[\"%s\"],"
            "\"name\":\"%s\","
            "\"manufacturer\":\"DIY\","
            "\"model\":\"ESP32-P4 Ethernet\","
            "\"sw_version\":\"%s\""
          "}"
        "}",
        DEVICE_NAME, DEVICE_ID, IP_STATE_TOPIC, AVAIL_TOPIC,
        DEVICE_ID, DEVICE_NAME, APP_VERSION_STRING
    );

    char up_cfg[512];
    snprintf(up_cfg, sizeof(up_cfg),
        "{"
          "\"name\":\"%s Uptime\","
          "\"unique_id\":\"%s_uptime\","
          "\"state_topic\":\"%s\","
          "\"unit_of_measurement\":\"s\","
          "\"device_class\":\"duration\","
          "\"availability_topic\":\"%s\","
          "\"payload_available\":\"online\","
          "\"payload_not_available\":\"offline\","
          "\"device\":{"
            "\"identifiers\":[\"%s\"],"
            "\"name\":\"%s\","
            "\"manufacturer\":\"DIY\","
            "\"model\":\"ESP32-P4 Ethernet\","
            "\"sw_version\":\"%s\""
          "}"
        "}",
        DEVICE_NAME, DEVICE_ID, UPTIME_STATE_TOPIC, AVAIL_TOPIC,
        DEVICE_ID, DEVICE_NAME, APP_VERSION_STRING
    );

    char rf_last_cfg[512];
    snprintf(rf_last_cfg, sizeof(rf_last_cfg),
        "{"
          "\"name\":\"%s RF Last\","
          "\"unique_id\":\"%s_rf_last\","
          "\"state_topic\":\"%s\","
          "\"icon\":\"mdi:radio\","
          "\"availability_topic\":\"%s\","
          "\"payload_available\":\"online\","
          "\"payload_not_available\":\"offline\","
          "\"device\":{"
            "\"identifiers\":[\"%s\"],"
            "\"name\":\"%s\","
            "\"manufacturer\":\"DIY\","
            "\"model\":\"ESP32-P4 + CC1101 (Apollo RX)\","
            "\"sw_version\":\"%s\""
          "}"
        "}",
        DEVICE_NAME, DEVICE_ID, RF_LAST_TOPIC, AVAIL_TOPIC,
        DEVICE_ID, DEVICE_NAME, APP_VERSION_STRING
    );

    char oil_cfg[512];
    snprintf(oil_cfg, sizeof(oil_cfg),
        "{"
          "\"name\":\"%s Oil Depth Raw\","
          "\"unique_id\":\"%s_oil_depth_raw\","
          "\"state_topic\":\"%s\","
          "\"icon\":\"mdi:cup-water\","
          "\"availability_topic\":\"%s\","
          "\"payload_available\":\"online\","
          "\"payload_not_available\":\"offline\","
          "\"device\":{"
            "\"identifiers\":[\"%s\"],"
            "\"name\":\"%s\","
            "\"manufacturer\":\"DIY\","
            "\"model\":\"ESP32-P4 + CC1101 (Apollo RX)\","
            "\"sw_version\":\"%s\""
          "}"
        "}",
        DEVICE_NAME, DEVICE_ID, OIL_DEPTH_TOPIC, AVAIL_TOPIC,
        DEVICE_ID, DEVICE_NAME, APP_VERSION_STRING
    );

    char seen_cfg[512];
    snprintf(seen_cfg, sizeof(seen_cfg),
        "{"
          "\"name\":\"%s Last Seen\","
          "\"unique_id\":\"%s_last_seen\","
          "\"state_topic\":\"%s\","
          "\"icon\":\"mdi:clock-outline\","
          "\"availability_topic\":\"%s\","
          "\"payload_available\":\"online\","
          "\"payload_not_available\":\"offline\","
          "\"device\":{"
            "\"identifiers\":[\"%s\"],"
            "\"name\":\"%s\","
            "\"manufacturer\":\"DIY\","
            "\"model\":\"ESP32-P4 + CC1101 (Apollo RX)\","
            "\"sw_version\":\"%s\""
          "}"
        "}",
        DEVICE_NAME, DEVICE_ID, LAST_SEEN_TOPIC, AVAIL_TOPIC,
        DEVICE_ID, DEVICE_NAME, APP_VERSION_STRING
    );

    mqtt_publish(IP_DISC_TOPIC, ip_cfg, 1, true);
    mqtt_publish(UPTIME_DISC_TOPIC, up_cfg, 1, true);
    mqtt_publish(RF_LAST_DISC_TOPIC, rf_last_cfg, 1, true);
    mqtt_publish(OIL_DEPTH_DISC_TOPIC, oil_cfg, 1, true);
    mqtt_publish(LAST_SEEN_DISC_TOPIC, seen_cfg, 1, true);

    ESP_LOGI(TAG, "HA discovery published");
    mqtt_log("HA discovery published");
}

/* ============== CC1101 RX callback ============== */

static void rf_rx_cb(const cc1101_rx_frame_t *f, void *user_ctx)
{
    (void)user_ctx;
    if (!mqtt_is_connected()) return;

    char raw_hex[2 * 64 + 1];
    size_t cap_len = f->len;
    if (cap_len > 64) cap_len = 64;

    bytes_to_hex(f->data, cap_len, raw_hex, sizeof(raw_hex));
    mqtt_publish(RF_RAW_TOPIC, raw_hex, 0, false);

    int64_t now_s = esp_timer_get_time() / 1000000LL;
    char seen[32];
    snprintf(seen, sizeof(seen), "%" PRId64, now_s);
    mqtt_publish(LAST_SEEN_TOPIC, seen, 0, false);

    uint8_t pkt[8];
    size_t off = 0;
    bool ok = find_apollo_packet(f->data, cap_len, pkt, &off);

    mqtt_publish(OIL_CRC_OK_TOPIC, ok ? "1" : "0", 0, false);

    if (!ok) {
        char msg[256];
        snprintf(msg, sizeof(msg),
                 "CAPTURE len=%u rssi=%d lqi=%u no_valid_crc raw=%s",
                 (unsigned)f->len, (int)f->rssi_dbm, (unsigned)f->lqi, raw_hex);
        mqtt_publish(RF_LAST_TOPIC, msg, 0, false);
        mqtt_log("%s", msg);
        return;
    }

    char pkt_hex[2 * 8 + 1];
    bytes_to_hex(pkt, 8, pkt_hex, sizeof(pkt_hex));
    mqtt_publish(APOLLO_PKT_TOPIC, pkt_hex, 0, false);

    uint8_t depth = pkt[6];
    char depth_str[8];
    snprintf(depth_str, sizeof(depth_str), "%u", (unsigned)depth);
    mqtt_publish(OIL_DEPTH_TOPIC, depth_str, 0, false);

    char msg[256];
    snprintf(msg, sizeof(msg),
             "APOLLO OK off=%u rssi=%d lqi=%u pkt=%s depth_raw=%u",
             (unsigned)off, (int)f->rssi_dbm, (unsigned)f->lqi, pkt_hex, (unsigned)depth);
    mqtt_publish(RF_LAST_TOPIC, msg, 0, false);
    mqtt_log("%s", msg);
}

/* ============== Ethernet + MQTT events ============== */

static void on_mqtt_connected(void)
{
    xEventGroupSetBits(s_health_eg, BIT_MQTT_UP);

    publish_availability(true);
    publish_discovery();
    publish_ip_state();
    publish_uptime_state();
    mqtt_log("MQTT connected");
}

static void uptime_timer_cb(TimerHandle_t xTimer)
{
    (void)xTimer;
    publish_uptime_state();
}

static void eth_event_handler(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data)
{
    (void)arg; (void)event_base; (void)event_data;

    switch (event_id) {
    case ETHERNET_EVENT_CONNECTED:
        ESP_LOGI(TAG, "Ethernet Link Up");
        break;
    case ETHERNET_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "Ethernet Link Down");

        publish_availability(false);
        mqtt_log("Ethernet Link Down");

        /* Gate “healthy” state */
        xEventGroupClearBits(s_health_eg, BIT_GOT_IP | BIT_MQTT_UP);

        mqtt_stop();
        break;
    case ETHERNET_EVENT_START:
        ESP_LOGI(TAG, "Ethernet Started");
        break;
    case ETHERNET_EVENT_STOP:
        ESP_LOGI(TAG, "Ethernet Stopped");
        xEventGroupClearBits(s_health_eg, BIT_GOT_IP | BIT_MQTT_UP);
        break;
    default:
        break;
    }
}

static void got_ip_event_handler(void *arg, esp_event_base_t event_base,
                                 int32_t event_id, void *event_data)
{
    (void)arg; (void)event_base; (void)event_id;

    const ip_event_got_ip_t *event = (const ip_event_got_ip_t *)event_data;
    const esp_netif_ip_info_t *ip_info = &event->ip_info;

    snprintf(s_ip_str, sizeof(s_ip_str), IPSTR, IP2STR(&ip_info->ip));
    ESP_LOGI(TAG, "Ethernet Got IP: %s", s_ip_str);

    xEventGroupSetBits(s_health_eg, BIT_GOT_IP);

    /* Start OTA server now we have an IP */
    ota_http_server_start();

    mqtt_set_connected_callback(on_mqtt_connected);
    ESP_ERROR_CHECK(mqtt_start(MQTT_HOST, MQTT_PORT, MQTT_USERNAME, MQTT_PASSWORD));
}

/* ============== app_main ============== */

void app_main(void)
{
    ESP_LOGI(TAG, "*** Booting Apollo P4 (Ethernet + MQTT + HA + CC1101 RX) ***");
    ESP_LOGI(TAG, "Firmware: %s build=%d utc=%s", APP_VERSION_STRING, (int)APP_BUILD_NUMBER, APP_BUILD_UTC);

    s_health_eg = xEventGroupCreate();
    if (!s_health_eg) {
        ESP_LOGE(TAG, "Failed to create health event group");
        abort();
    }

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    } else {
        ESP_ERROR_CHECK(ret);
    }

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* Start the “mark valid” task (so rollback won’t happen once stable) */
    xTaskCreate(ota_mark_valid_task, "ota_mark_valid", 4096, NULL, 5, NULL);

    /* ---------- CC1101 wiring (your pins) ---------- */
    cc1101_pins_t pins = {
        .pin_cs   = 5,
        .pin_sck  = 15,
        .pin_mosi = 16,
        .pin_miso = 17,
        .pin_gdo0 = 14,
        .pin_gdo2 = 27,
    };

    ESP_ERROR_CHECK(cc1101_init(&pins));

    cc1101_id_t id;
    ESP_ERROR_CHECK(cc1101_read_id(&id));
    ESP_LOGI(TAG, "CC1101 ID: PARTNUM=0x%02X VERSION=0x%02X", id.partnum, id.version);

    ESP_ERROR_CHECK(cc1101_start_rx_apollo(rf_rx_cb, NULL));
    ESP_LOGI(TAG, "CC1101 RX started (Apollo profile)");

    /* ---------- Ethernet ---------- */
    esp_netif_config_t cfg = ESP_NETIF_DEFAULT_ETH();
    s_eth_netif = esp_netif_new(&cfg);
    assert(s_eth_netif);

    ESP_LOGI(TAG, "Calling example_eth_init()...");
    ESP_ERROR_CHECK(example_eth_init(&s_eth_handles, &s_eth_count));
    ESP_LOGI(TAG, "example_eth_init OK, eth_count=%u", s_eth_count);

    ESP_ERROR_CHECK(esp_netif_attach(s_eth_netif, esp_eth_new_netif_glue(s_eth_handles[0])));

    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &got_ip_event_handler, NULL));

    /* ---------- uptime publish ---------- */
    s_uptime_timer = xTimerCreate("uptime_pub",
                                  pdMS_TO_TICKS(UPTIME_PUBLISH_PERIOD_SEC * 1000),
                                  pdTRUE,
                                  NULL,
                                  uptime_timer_cb);
    if (s_uptime_timer) xTimerStart(s_uptime_timer, 0);

    ESP_LOGI(TAG, "Starting Ethernet driver...");
    ESP_ERROR_CHECK(esp_eth_start(s_eth_handles[0]));
    ESP_LOGI(TAG, "Ethernet started, waiting for DHCP...");
}
