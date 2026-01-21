#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ssd1306_simple.h"
#include <stdio.h>
#include <string.h>

static const char* TAG = "oled_demo";

#define I2C_PORT 0
#define SDA_GPIO 41
#define SCL_GPIO 42
#define I2C_FREQ_HZ 100000

#define SHT45_ADDR 0x44
#define SHT45_CMD_MEAS_HIGH 0xFD
#define SHT45_MEAS_DELAY_MS 10

static uint8_t sht4x_crc8(const uint8_t* data, size_t len) {
    uint8_t crc = 0xFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) {
            if (crc & 0x80) {
                crc = (uint8_t)((crc << 1) ^ 0x31);
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

static esp_err_t sht45_read(i2c_master_dev_handle_t dev, float* temp_c, float* rh) {
    uint8_t cmd = SHT45_CMD_MEAS_HIGH;
    esp_err_t ret = i2c_master_transmit(dev, &cmd, 1, 100);
    if (ret != ESP_OK) {
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(SHT45_MEAS_DELAY_MS));

    uint8_t rx[6] = {0};
    ret = i2c_master_receive(dev, rx, sizeof(rx), 100);
    if (ret != ESP_OK) {
        return ret;
    }

    if (sht4x_crc8(rx, 2) != rx[2] || sht4x_crc8(&rx[3], 2) != rx[5]) {
        return ESP_ERR_INVALID_CRC;
    }

    uint16_t t_raw = (uint16_t)((rx[0] << 8) | rx[1]);
    uint16_t rh_raw = (uint16_t)((rx[3] << 8) | rx[4]);

    *temp_c = -45.0f + 175.0f * ((float)t_raw / 65535.0f);
    *rh = 100.0f * ((float)rh_raw / 65535.0f);
    return ESP_OK;
}

static void oled_write_line(ssd1306_t* oled, uint8_t page, const char* text) {
    char line[22];
    size_t len = strlen(text);
    if (len > 21) {
        len = 21;
    }
    memcpy(line, text, len);
    for (size_t i = len; i < 21; ++i) {
        line[i] = ' ';
    }
    line[21] = '\0';

    ESP_ERROR_CHECK(ssd1306_set_cursor(oled, page, 0));
    ESP_ERROR_CHECK(ssd1306_write_text_5x7(oled, line));
}

void app_main(void) {
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_PORT,
        .sda_io_num = SDA_GPIO,
        .scl_io_num = SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    i2c_master_bus_handle_t bus;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus));

    ssd1306_t oled = {
        .bus = bus,
        .addr = 0x3C,
        .width = 128,
        .height = 64,
    };

    i2c_device_config_t sht_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SHT45_ADDR,
        .scl_speed_hz = I2C_FREQ_HZ,
    };
    i2c_master_dev_handle_t sht_dev;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &sht_cfg, &sht_dev));

    ESP_ERROR_CHECK(ssd1306_init(&oled));
    ESP_ERROR_CHECK(ssd1306_clear(&oled));

    // Page 0 = top row (8px tall). Column 0 = left.
    ESP_ERROR_CHECK(ssd1306_set_cursor(&oled, 0, 0));
    ESP_ERROR_CHECK(ssd1306_write_text_5x7(&oled, "HELLO OLED"));

    ESP_ERROR_CHECK(ssd1306_set_cursor(&oled, 1, 0));
    ESP_ERROR_CHECK(ssd1306_write_text_5x7(&oled, "SHT45 @ 0X44"));

    ESP_LOGI(TAG, "Displayed text on OLED (0x3C).");

    while (true) {
        float temp_c = 0.0f;
        float rh = 0.0f;
        esp_err_t ret = sht45_read(sht_dev, &temp_c, &rh);
        if (ret == ESP_OK) {
            char temp_line[22];
            char hum_line[22];
            snprintf(temp_line, sizeof(temp_line), "TEMP: %5.1fC", temp_c);
            snprintf(hum_line, sizeof(hum_line), "HUM : %5.1f%%", rh);
            oled_write_line(&oled, 3, temp_line);
            oled_write_line(&oled, 4, hum_line);
        } else {
            oled_write_line(&oled, 3, "SHT45 READ ERR");
            oled_write_line(&oled, 4, "CHECK WIRING");
            ESP_LOGW(TAG, "SHT45 read failed: %s", esp_err_to_name(ret));
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
