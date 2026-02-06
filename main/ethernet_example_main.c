#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_eth.h"
#include "nvs_flash.h"
#include "esp_timer.h"

// Your component
#include "ethernet_init.h"

// Our MQTT wrapper
#include "mqtt.h"

static const char *TAG = "apollo_p4";

// ===== MQTT SETTINGS (yours) =====
#define MQTT_HOST     "192.168.1.81"
#define MQTT_PORT     1883
#define MQTT_USERNAME "rflogger"
#define MQTT_PASSWORD "rflogger"

// ===== Home Assistant discovery prefix =====
#define HA_DISCOVERY_PREFIX "homeassistant"

// ===== Device identity =====
#define DEVICE_ID    "rf-logger-node"
#define DEVICE_NAME  "Apollo P4 Ethernet"

// ===== Base topics =====
#define BASE_TOPIC          "apollo_p4/" DEVICE_ID
#define AVAIL_TOPIC         BASE_TOPIC "/status"
#define IP_STATE_TOPIC      BASE_TOPIC "/ip"
#define UPTIME_STATE_TOPIC  BASE_TOPIC "/uptime"

// Discovery topics
#define IP_DISC_TOPIC       HA_DISCOVERY_PREFIX "/sensor/" DEVICE_ID "/ip/config"
#define UPTIME_DISC_TOPIC   HA_DISCOVERY_PREFIX "/sensor/" DEVICE_ID "/uptime/config"

// Publish uptime every N seconds
#define UPTIME_PUBLISH_PERIOD_SEC  10

static esp_netif_t *s_eth_netif = NULL;

static esp_eth_handle_t *s_eth_handles = NULL;
static uint8_t s_eth_count = 0;

static char s_ip_str[32] = "0.0.0.0";
static TimerHandle_t s_uptime_timer = NULL;

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
            "\"sw_version\":\"esp-idf\""
          "}"
        "}",
        DEVICE_NAME, DEVICE_ID, IP_STATE_TOPIC, AVAIL_TOPIC,
        DEVICE_ID, DEVICE_NAME
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
            "\"sw_version\":\"esp-idf\""
          "}"
        "}",
        DEVICE_NAME, DEVICE_ID, UPTIME_STATE_TOPIC, AVAIL_TOPIC,
        DEVICE_ID, DEVICE_NAME
    );

    mqtt_publish(IP_DISC_TOPIC, ip_cfg, 1, true);
    mqtt_publish(UPTIME_DISC_TOPIC, up_cfg, 1, true);

    ESP_LOGI(TAG, "HA discovery published");
}

static void on_mqtt_connected(void)
{
    publish_availability(true);
    publish_discovery();
    publish_ip_state();
    publish_uptime_state();

    // Optional test message (you can remove)
    mqtt_publish("apollo/test", "hello from ESP32-P4 (ethernet)", 0, false);
}

static void uptime_timer_cb(TimerHandle_t xTimer)
{
    (void)xTimer;
    publish_uptime_state();
}

static void eth_event_handler(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data)
{
    (void)arg;
    (void)event_base;
    (void)event_data;

    switch (event_id) {
    case ETHERNET_EVENT_CONNECTED:
        ESP_LOGI(TAG, "Ethernet Link Up");
        break;
    case ETHERNET_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "Ethernet Link Down");
        publish_availability(false);
        mqtt_stop();
        break;
    case ETHERNET_EVENT_START:
        ESP_LOGI(TAG, "Ethernet Started");
        break;
    case ETHERNET_EVENT_STOP:
        ESP_LOGI(TAG, "Ethernet Stopped");
        break;
    default:
        break;
    }
}

static void got_ip_event_handler(void *arg, esp_event_base_t event_base,
                                 int32_t event_id, void *event_data)
{
    (void)arg;
    (void)event_base;
    (void)event_id;

    const ip_event_got_ip_t *event = (const ip_event_got_ip_t *)event_data;
    const esp_netif_ip_info_t *ip_info = &event->ip_info;

    snprintf(s_ip_str, sizeof(s_ip_str), IPSTR, IP2STR(&ip_info->ip));

    ESP_LOGI(TAG, "Ethernet Got IP Address");
    ESP_LOGI(TAG, "IP      : %s", s_ip_str);
    ESP_LOGI(TAG, "Netmask : " IPSTR, IP2STR(&ip_info->netmask));
    ESP_LOGI(TAG, "Gateway : " IPSTR, IP2STR(&ip_info->gw));

    mqtt_set_connected_callback(on_mqtt_connected);

    ESP_ERROR_CHECK(mqtt_start(
        MQTT_HOST,
        MQTT_PORT,
        MQTT_USERNAME,
        MQTT_PASSWORD
    ));
}

void app_main(void)
{
    ESP_LOGI(TAG, "*** Booting Apollo P4 (Ethernet + MQTT + HA) ***");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    } else {
        ESP_ERROR_CHECK(ret);
    }

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_config_t cfg = ESP_NETIF_DEFAULT_ETH();
    s_eth_netif = esp_netif_new(&cfg);
    assert(s_eth_netif);

    ESP_LOGI(TAG, "Calling example_eth_init()...");
    ESP_ERROR_CHECK(example_eth_init(&s_eth_handles, &s_eth_count));
    ESP_LOGI(TAG, "example_eth_init OK, eth_count=%u", s_eth_count);

    if (s_eth_count < 1 || !s_eth_handles || !s_eth_handles[0]) {
        ESP_LOGE(TAG, "No valid Ethernet handle returned!");
        return;
    }

    ESP_ERROR_CHECK(esp_netif_attach(s_eth_netif, esp_eth_new_netif_glue(s_eth_handles[0])));

    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &got_ip_event_handler, NULL));

    s_uptime_timer = xTimerCreate("uptime_pub",
                                  pdMS_TO_TICKS(UPTIME_PUBLISH_PERIOD_SEC * 1000),
                                  pdTRUE,
                                  NULL,
                                  uptime_timer_cb);
    if (s_uptime_timer) {
        xTimerStart(s_uptime_timer, 0);
    }

    ESP_LOGI(TAG, "Starting Ethernet driver...");
    ESP_ERROR_CHECK(esp_eth_start(s_eth_handles[0]));
    ESP_LOGI(TAG, "Ethernet started, waiting for DHCP...");
}
