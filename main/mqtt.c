#include "mqtt.h"

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "mqtt_client.h"

static const char *TAG = "mqtt";

static esp_mqtt_client_handle_t s_client = NULL;
static bool s_connected = false;
static mqtt_connected_cb_t s_on_connected = NULL;

static void mqtt_event_handler(void *handler_args,
                               esp_event_base_t base,
                               int32_t event_id,
                               void *event_data)
{
    (void)handler_args;
    (void)base;

    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
    (void)event;

    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        s_connected = true;
        ESP_LOGI(TAG, "MQTT connected");
        if (s_on_connected) {
            s_on_connected();
        }
        break;

    case MQTT_EVENT_DISCONNECTED:
        s_connected = false;
        ESP_LOGW(TAG, "MQTT disconnected");
        break;

    case MQTT_EVENT_ERROR:
        ESP_LOGE(TAG, "MQTT error");
        break;

    default:
        break;
    }
}

void mqtt_set_connected_callback(mqtt_connected_cb_t cb)
{
    s_on_connected = cb;
}

bool mqtt_is_connected(void)
{
    return s_connected;
}

int mqtt_publish(const char *topic, const char *payload, int qos, bool retain)
{
    if (!s_client) return -1;
    return esp_mqtt_client_publish(s_client, topic, payload, 0, qos, retain ? 1 : 0);
}

esp_err_t mqtt_start(const char *host,
                     int port,
                     const char *username,
                     const char *password)
{
    if (s_client) {
        ESP_LOGW(TAG, "MQTT already started");
        return ESP_OK;
    }

    char uri[128];
    snprintf(uri, sizeof(uri), "mqtt://%s:%d", host, port);

    esp_mqtt_client_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));

    // These fields are known-good on your build (you used broker.address.uri earlier)
    cfg.broker.address.uri = uri;
    cfg.credentials.username = username;
    cfg.credentials.authentication.password = password;

    ESP_LOGI(TAG, "Starting MQTT: %s", uri);

    s_client = esp_mqtt_client_init(&cfg);
    if (!s_client) {
        ESP_LOGE(TAG, "esp_mqtt_client_init failed");
        return ESP_FAIL;
    }

    ESP_ERROR_CHECK(esp_mqtt_client_register_event(s_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL));
    ESP_ERROR_CHECK(esp_mqtt_client_start(s_client));

    return ESP_OK;
}

void mqtt_stop(void)
{
    if (!s_client) return;

    esp_mqtt_client_stop(s_client);
    esp_mqtt_client_destroy(s_client);
    s_client = NULL;
    s_connected = false;
}
