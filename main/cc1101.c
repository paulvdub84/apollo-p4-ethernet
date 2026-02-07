#include "cc1101.h"

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_rom_sys.h"   // esp_rom_delay_us()

static const char *TAG = "cc1101";

static spi_device_handle_t s_spi = NULL;
static int s_pin_cs = -1;
static int s_pin_miso = -1;

// CC1101 strobe
#define CC1101_SRES      0x30

// CC1101 registers
#define CC1101_PARTNUM   0x30
#define CC1101_VERSION   0x31

#define CC1101_READ_BIT  0x80

static inline void cc_delay_us(uint32_t us)
{
    esp_rom_delay_us(us);
}

static inline void cs_select(void)   { gpio_set_level(s_pin_cs, 0); }
static inline void cs_deselect(void) { gpio_set_level(s_pin_cs, 1); }

/**
 * CC1101 requires: after CS low, wait until SO(MISO) goes low before clocking.
 */
static esp_err_t wait_so_low(uint32_t timeout_us)
{
    if (s_pin_miso < 0) return ESP_ERR_INVALID_STATE;

    const uint32_t step_us = 5;
    uint32_t waited = 0;

    while (gpio_get_level(s_pin_miso) != 0) {
        cc_delay_us(step_us);
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

static esp_err_t cc1101_strobe(uint8_t strobe)
{
    cs_select();
    esp_err_t err = wait_so_low(2000);
    if (err != ESP_OK) {
        cs_deselect();
        ESP_LOGE(TAG, "SO(MISO) never went low (strobe 0x%02X)", strobe);
        return err;
    }

    uint8_t tx[1] = { strobe };
    uint8_t rx[1] = { 0 };

    err = spi_txrx(tx, rx, 1);
    cs_deselect();

    if (err != ESP_OK) return err;

    if (strobe == CC1101_SRES) {
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    return ESP_OK;
}

static esp_err_t cc1101_read_reg(uint8_t addr, uint8_t *out)
{
    if (!out) return ESP_ERR_INVALID_ARG;

    uint8_t tx[2] = { (uint8_t)(addr | CC1101_READ_BIT), 0x00 };
    uint8_t rx[2] = { 0, 0 };

    cs_select();
    esp_err_t err = wait_so_low(2000);
    if (err != ESP_OK) {
        cs_deselect();
        ESP_LOGE(TAG, "SO(MISO) never went low (read reg 0x%02X)", addr);
        return err;
    }

    err = spi_txrx(tx, rx, 2);
    cs_deselect();

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
    ESP_ERROR_CHECK(gpio_config(&io));
    cs_deselect();

    // MISO input for SO-low wait
    io.mode = GPIO_MODE_INPUT;
    io.pin_bit_mask = 1ULL << s_pin_miso;
    io.pull_up_en = 1;
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

    // Dedicated SPI bus (separate from DM9051 to avoid conflicts)
    spi_bus_config_t buscfg = {
        .miso_io_num = pins->pin_miso,
        .mosi_io_num = pins->pin_mosi,
        .sclk_io_num = pins->pin_sck,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0
    };

    esp_err_t err = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "spi_bus_initialize failed: %s", esp_err_to_name(err));
        return err;
    }

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = -1,     // manual CS
        .queue_size = 1,
    };

    err = spi_bus_add_device(SPI2_HOST, &devcfg, &s_spi);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_add_device failed: %s", esp_err_to_name(err));
        return err;
    }

    // Reset sequence
    cs_deselect();
    vTaskDelay(pdMS_TO_TICKS(1));
    cs_select();
    cc_delay_us(10);
    cs_deselect();
    vTaskDelay(pdMS_TO_TICKS(1));

    err = cc1101_strobe(CC1101_SRES);
    if (err != ESP_OK) return err;

    ESP_LOGI(TAG, "CC1101 init OK");
    return ESP_OK;
}

esp_err_t cc1101_read_id(cc1101_id_t *out)
{
    if (!out) return ESP_ERR_INVALID_ARG;

    esp_err_t err = cc1101_read_reg(CC1101_PARTNUM, &out->partnum);
    if (err != ESP_OK) return err;

    err = cc1101_read_reg(CC1101_VERSION, &out->version);
    if (err != ESP_OK) return err;

    return ESP_OK;
}
