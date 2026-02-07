#include "cc1101.h"

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_rom_sys.h"

static const char *TAG = "cc1101";

static spi_device_handle_t s_spi = NULL;
static spi_host_device_t   s_host = SPI2_HOST;

static int s_pin_cs = -1;
static int s_pin_miso = -1;

// CC1101 strobes
#define CC1101_SRES      0x30

// CC1101 registers
#define CC1101_PARTNUM   0x30
#define CC1101_VERSION   0x31

// Access bits
#define CC1101_READ_BIT   0x80
#define CC1101_BURST_BIT  0x40
#define CC1101_RW_STATUS  (CC1101_READ_BIT | CC1101_BURST_BIT)   // 0xC0

static inline void dly_us(uint32_t us) { esp_rom_delay_us(us); }
static inline void cs_low(void)        { gpio_set_level(s_pin_cs, 0); }
static inline void cs_high(void)       { gpio_set_level(s_pin_cs, 1); }

static esp_err_t wait_so_low(uint32_t timeout_us)
{
    if (s_pin_miso < 0) return ESP_ERR_INVALID_STATE;

    const uint32_t step_us = 5;
    uint32_t waited = 0;

    while (gpio_get_level(s_pin_miso) != 0) {
        dly_us(step_us);
        waited += step_us;
        if (waited >= timeout_us) return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

static esp_err_t spi_txrx(const uint8_t *tx, uint8_t *rx, size_t len_bytes)
{
    if (!s_spi) return ESP_ERR_INVALID_STATE;

    spi_transaction_t t = {0};
    t.length = len_bytes * 8;
    t.tx_buffer = tx;
    t.rx_buffer = rx;
    return spi_device_polling_transmit(s_spi, &t);
}

static esp_err_t cc1101_reset_sequence(void)
{
    cs_high();
    dly_us(50);

    cs_low();
    esp_err_t err = wait_so_low(5000);
    cs_high();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Reset: SO did not go low (%s)", esp_err_to_name(err));
        return err;
    }

    dly_us(50);

    cs_low();
    err = wait_so_low(5000);
    if (err != ESP_OK) {
        cs_high();
        ESP_LOGE(TAG, "SRES: SO did not go low (%s)", esp_err_to_name(err));
        return err;
    }

    uint8_t tx[1] = { CC1101_SRES };
    uint8_t rx[1] = { 0 };
    err = spi_txrx(tx, rx, 1);
    cs_high();

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SRES SPI failed: %s", esp_err_to_name(err));
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(5));
    return ESP_OK;
}

static esp_err_t cc1101_read_reg(uint8_t addr, uint8_t *out)
{
    if (!out) return ESP_ERR_INVALID_ARG;

    // IMPORTANT:
    // - Config regs (0x00–0x2E): use READ (0x80)
    // - Status regs (0x30–0x3D): must use READ|BURST (0xC0)
    uint8_t cmd = (addr >= 0x30) ? (uint8_t)(addr | CC1101_RW_STATUS)
                                 : (uint8_t)(addr | CC1101_READ_BIT);

    uint8_t tx[2] = { cmd, 0x00 };
    uint8_t rx[2] = { 0, 0 };

    cs_low();
    esp_err_t err = wait_so_low(5000);
    if (err != ESP_OK) {
        cs_high();
        return err;
    }

    err = spi_txrx(tx, rx, 2);
    cs_high();

    if (err != ESP_OK) return err;

    *out = rx[1];
    return ESP_OK;
}

esp_err_t cc1101_init(const cc1101_pins_t *pins)
{
    if (!pins) return ESP_ERR_INVALID_ARG;

    s_pin_cs = pins->pin_cs;
    s_pin_miso = pins->pin_miso;

    // CS output, default high
    gpio_config_t io = {0};
    io.mode = GPIO_MODE_OUTPUT;
    io.pin_bit_mask = 1ULL << s_pin_cs;
    io.pull_up_en = 0;
    io.pull_down_en = 0;
    ESP_ERROR_CHECK(gpio_config(&io));
    cs_high();

    // MISO input (SO)
    io.mode = GPIO_MODE_INPUT;
    io.pin_bit_mask = 1ULL << s_pin_miso;
    io.pull_up_en = 1;
    io.pull_down_en = 0;
    ESP_ERROR_CHECK(gpio_config(&io));

    // Optional GDO pins
    if (pins->pin_gdo0 >= 0) {
        io.mode = GPIO_MODE_INPUT;
        io.pin_bit_mask = 1ULL << pins->pin_gdo0;
        io.pull_up_en = 0;
        io.pull_down_en = 0;
        ESP_ERROR_CHECK(gpio_config(&io));
    }
    if (pins->pin_gdo2 >= 0) {
        io.mode = GPIO_MODE_INPUT;
        io.pin_bit_mask = 1ULL << pins->pin_gdo2;
        io.pull_up_en = 0;
        io.pull_down_en = 0;
        ESP_ERROR_CHECK(gpio_config(&io));
    }

    // Dedicated SPI bus for CC1101
    spi_bus_config_t buscfg = {
        .miso_io_num = pins->pin_miso,
        .mosi_io_num = pins->pin_mosi,
        .sclk_io_num = pins->pin_sck,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0
    };

    esp_err_t err = spi_bus_initialize(s_host, &buscfg, SPI_DMA_CH_AUTO);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "spi_bus_initialize failed: %s", esp_err_to_name(err));
        return err;
    }

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 250 * 1000,  // safe slow clock
        .mode = 0,
        .spics_io_num = -1,            // manual CS
        .queue_size = 1,
    };

    err = spi_bus_add_device(s_host, &devcfg, &s_spi);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_add_device failed: %s", esp_err_to_name(err));
        return err;
    }

    err = cc1101_reset_sequence();
    if (err != ESP_OK) return err;

    ESP_LOGI(TAG, "CC1101 init OK");
    return ESP_OK;
}

esp_err_t cc1101_read_id(cc1101_id_t *out)
{
    if (!out) return ESP_ERR_INVALID_ARG;

    for (int i = 0; i < 5; i++) {
        uint8_t p = 0, v = 0;

        esp_err_t e1 = cc1101_read_reg(CC1101_PARTNUM, &p);
        esp_err_t e2 = cc1101_read_reg(CC1101_VERSION, &v);

        if (e1 == ESP_OK && e2 == ESP_OK) {
            out->partnum = p;
            out->version = v;

            ESP_LOGI(TAG, "ID read attempt %d: PARTNUM=0x%02X VERSION=0x%02X", i + 1, p, v);

            // Accept any non-0xFF PARTNUM immediately
            if (p != 0xFF) return ESP_OK;

            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        ESP_LOGW(TAG, "ID read attempt %d failed: %s / %s",
                 i + 1, esp_err_to_name(e1), esp_err_to_name(e2));
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    return ESP_ERR_TIMEOUT;
}
