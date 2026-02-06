#include <stdio.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_event.h"
#include "nvs_flash.h"

#include "esp_netif.h"
#include "esp_eth.h"
#include "esp_eth_driver.h"

#include "mqtt_client.h"

// From your custom component: components/ethernet_init/ethernet_init.h
#include "ethernet_init.h"

static const char *TAG = "eth_example";

#define MQTT_BROKER_URI  "mqtt://192.168.1.81:1883"
#define MQTT_USERNAME    "rflogger"
#define MQTT_PASSWORD    "rflogger"
#define MQTT_PUB_TOPIC   "apollo/test"
#define MQTT_PUB_PAYLOAD "hello from ESP32-P4 (ethernet)"

static EventGroupHandle_t s_eth_event_group;
static esp_mqtt_client_handle_t s_mqtt_client = NULL;

#define ETH_CONNECTED_BIT BIT0
#define ETH_GOT_IP_BIT    BIT1

static void mqtt_event_handler(void *handler_args,
                               esp_event_base_t base,
                               int32_t event_id,
                               void *event_data)
{
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;

    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT connected");
        esp_mqtt_client_publish(event->client, MQTT_PUB_TOPIC, MQTT_PUB_PAYLOAD, 0, 1, 0);
        ESP_LOGI(TAG, "MQTT published to %s", MQTT_PUB_TOPIC);
        break;

    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "MQTT disconnected");
        break;

    case MQTT_EVENT_ERROR:
        ESP_LOGE(TAG, "MQTT error");
        break;

    default:
        break;
    }
}

static void start_mqtt(void)
{
    if (s_mqtt_client) {
        ESP_LOGW(TAG, "MQTT already started");
        return;
    }

    esp_mqtt_client_config_t cfg = {
        .broker.address.uri = MQTT_BROKER_URI,
        .credentials.username = MQTT_USERNAME,
        .credentials.authentication.password = MQTT_PASSWORD,
    };

    ESP_LOGI(TAG, "Starting MQTT: %s", MQTT_BROKER_URI);
    s_mqtt_client = esp_mqtt_client_init(&cfg);
    ESP_ERROR_CHECK(esp_mqtt_client_register_event(s_mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL));
    ESP_ERROR_CHECK(esp_mqtt_client_start(s_mqtt_client));
}

static void eth_event_handler(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data)
{
    if (event_id == ETHERNET_EVENT_CONNECTED) {
        ESP_LOGI(TAG, "Ethernet Link Up");
        xEventGroupSetBits(s_eth_event_group, ETH_CONNECTED_BIT);
    } else if (event_id == ETHERNET_EVENT_DISCONNECTED) {
        ESP_LOGW(TAG, "Ethernet Link Down");
        xEventGroupClearBits(s_eth_event_group, ETH_CONNECTED_BIT | ETH_GOT_IP_BIT);

        if (s_mqtt_client) {
            esp_mqtt_client_stop(s_mqtt_client);
            esp_mqtt_client_destroy(s_mqtt_client);
            s_mqtt_client = NULL;
        }
    } else if (event_id == ETHERNET_EVENT_START) {
        ESP_LOGI(TAG, "Ethernet Started");
    } else if (event_id == ETHERNET_EVENT_STOP) {
        ESP_LOGI(TAG, "Ethernet Stopped");
    }
}

static void got_ip_event_handler(void *arg, esp_event_base_t event_base,
                                 int32_t event_id, void *event_data)
{
    const ip_event_got_ip_t *event = (const ip_event_got_ip_t *)event_data;
    const esp_netif_ip_info_t *ip_info = &event->ip_info;

    ESP_LOGI(TAG, "Ethernet Got IP Address");
    ESP_LOGI(TAG, "IP      : " IPSTR, IP2STR(&ip_info->ip));
    ESP_LOGI(TAG, "Netmask : " IPSTR, IP2STR(&ip_info->netmask));
    ESP_LOGI(TAG, "Gateway : " IPSTR, IP2STR(&ip_info->gw));

    xEventGroupSetBits(s_eth_event_group, ETH_GOT_IP_BIT);

    // Start MQTT once we have an IP
    start_mqtt();
}

void app_main(void)
{
    ESP_LOGI(TAG, "*** APP_MAIN STARTED (Ethernet + MQTT) ***");

    // NVS required by lots of IDF components
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    } else {
        ESP_ERROR_CHECK(ret);
    }

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    s_eth_event_group = xEventGroupCreate();

    // Create default Ethernet netif
    esp_netif_config_t cfg = ESP_NETIF_DEFAULT_ETH();
    esp_netif_t *eth_netif = esp_netif_new(&cfg);
    assert(eth_netif);

    // Init Ethernet driver(s) using your ethernet_init component
    esp_eth_handle_t *eth_handles = NULL;
    uint8_t eth_count = 0;

    ESP_LOGI(TAG, "Calling example_eth_init()...");
    ESP_ERROR_CHECK(example_eth_init(&eth_handles, &eth_count));
    ESP_LOGI(TAG, "example_eth_init OK, eth_count=%u", eth_count);

    if (eth_count < 1 || eth_handles == NULL || eth_handles[0] == NULL) {
        ESP_LOGE(TAG, "No valid Ethernet handle returned!");
        return;
    }

    // Attach driver to netif
    ESP_ERROR_CHECK(esp_netif_attach(eth_netif, esp_eth_new_netif_glue(eth_handles[0])));

    // Register event handlers
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &got_ip_event_handler, NULL));

    ESP_LOGI(TAG, "Starting Ethernet driver...");
    ESP_ERROR_CHECK(esp_eth_start(eth_handles[0]));

    ESP_LOGI(TAG, "Ethernet started, waiting for DHCP...");
    // app_main returns; events keep running
}
