#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "scd41_simple.h"
#include "sgp30_simple.h"
#include "ssd1306_simple.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

static const char* TAG = "oled_demo";

#define I2C_PORT 0
#define SDA_GPIO 41
#define SCL_GPIO 42
#define I2C_FREQ_HZ 100000

#define ENABLE_SHT45 1
#define SHT45_ADDR 0x44
#define SHT45_CMD_MEAS_HIGH 0xFD
#define SHT45_MEAS_DELAY_MS 10
#define SCD41_ADDR 0x62
#define SCD41_WARMUP_MS 5000
#define SGP30_ADDR 0x58
#define SGP30_WARMUP_MS 15000

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

// Calculate absolute humidity in g/m^3 from temperature and relative humidity.
static float abs_humidity_gm3(float temp_c, float rh_percent) {
    float rh = rh_percent * 0.01f;
    float svp = 6.112f * expf((17.62f * temp_c) / (243.12f + temp_c));
    float vapor_pressure = rh * svp;
    return 216.7f * (vapor_pressure / (273.15f + temp_c));
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

#if ENABLE_SHT45
    i2c_device_config_t sht_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SHT45_ADDR,
        .scl_speed_hz = I2C_FREQ_HZ,
    };
    i2c_master_dev_handle_t sht_dev;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &sht_cfg, &sht_dev));
#endif

    esp_err_t probe_ret = i2c_master_probe(bus, SCD41_ADDR, 100);
    if (probe_ret == ESP_OK) {
        ESP_LOGI(TAG, "SCD41 detected at 0x%02X", SCD41_ADDR);
    } else {
        ESP_LOGW(TAG, "SCD41 probe failed: %s", esp_err_to_name(probe_ret));
    }

    scd41_t scd = {
        .bus = bus,
        .dev = NULL,
        .addr = SCD41_ADDR,
    };

    sgp30_t sgp = {
        .bus = bus,
        .dev = NULL,
        .addr = SGP30_ADDR,
    };

    ESP_ERROR_CHECK(ssd1306_init(&oled));
    ESP_ERROR_CHECK(ssd1306_clear(&oled));

    ESP_ERROR_CHECK(scd41_init(&scd));
    vTaskDelay(pdMS_TO_TICKS(1000)); // Allow SCD41 power-up time before first command.

    ESP_ERROR_CHECK(scd41_wakeup(&scd));
    vTaskDelay(pdMS_TO_TICKS(100)); // Give the sensor more time after wake-up.

    esp_err_t scd_stop_ret = scd41_stop_periodic_measurement(&scd);
    if (scd_stop_ret != ESP_OK) {
        ESP_LOGW(TAG, "SCD41 stop failed (ignored): %s", esp_err_to_name(scd_stop_ret));
    }
    vTaskDelay(pdMS_TO_TICKS(200));

    bool scd_started = false;
    esp_err_t scd_start_ret = ESP_FAIL;
    TickType_t scd_start_tick = 0;
    for (int attempt = 0; attempt < 3 && !scd_started; ++attempt) {
        scd_start_ret = scd41_start_periodic_measurement(&scd);
        if (scd_start_ret == ESP_OK) {
            scd_started = true;
            scd_start_tick = xTaskGetTickCount();
            ESP_LOGI(TAG, "Started SCD41 periodic measurement.");
        } else {
            ESP_LOGW(TAG, "SCD41 start attempt %d failed: %s", attempt + 1, esp_err_to_name(scd_start_ret));
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }
    if (!scd_started) {
        ESP_LOGW(TAG, "SCD41 start failed after retries: %s", esp_err_to_name(scd_start_ret));
    }

    ESP_ERROR_CHECK(sgp30_init(&sgp));
    vTaskDelay(pdMS_TO_TICKS(20)); // Allow SGP30 power-up time before IAQ init.
    ESP_ERROR_CHECK(sgp30_iaq_init(&sgp));
    TickType_t sgp_start_tick = xTaskGetTickCount();

#if ENABLE_SHT45
    oled_write_line(&oled, 0, "SHT45 TEMP: ----");
    oled_write_line(&oled, 1, "SHT45 HUM : ----");
#else
    oled_write_line(&oled, 0, "SHT45 DISABLED");
    oled_write_line(&oled, 1, " ");
#endif
    oled_write_line(&oled, 2, "---------------------");
    oled_write_line(&oled, 3, "SCD41 CO2 : ----");
    oled_write_line(&oled, 4, "SCD41 TEMP: ----");
    oled_write_line(&oled, 5, "SCD41 HUM : ----");
    oled_write_line(&oled, 6, "---------------------");
    oled_write_line(&oled, 7, "eCO2:----- TVOC:----");

    while (true) {
        static uint16_t scd_co2 = 0;
        static float scd_temp_c = 0.0f;
        static float scd_rh = 0.0f;
        static bool scd_valid = false;

        static uint16_t sgp_eco2 = 0;
        static uint16_t sgp_tvoc = 0;
        static bool sgp_valid = false;

        bool sht_ok = false;
        float temp_c = 0.0f;
        float rh = 0.0f;

#if ENABLE_SHT45
        esp_err_t ret = sht45_read(sht_dev, &temp_c, &rh);
        if (ret == ESP_OK) {
            sht_ok = true;
            char temp_line[22];
            char hum_line[22];
            snprintf(temp_line, sizeof(temp_line), "TEMP: %5.1fC", temp_c);
            snprintf(hum_line, sizeof(hum_line), "HUM : %5.1f%%", rh);
            char sht_temp_line[22];
            char sht_hum_line[22];
            snprintf(sht_temp_line, sizeof(sht_temp_line), "SHT45 TEMP:%5.1fC", temp_c);
            snprintf(sht_hum_line, sizeof(sht_hum_line), "SHT45 HUM :%5.1f%%", rh);
            oled_write_line(&oled, 0, sht_temp_line);
            oled_write_line(&oled, 1, sht_hum_line);
        } else {
            oled_write_line(&oled, 0, "SHT45 READ ERR");
            oled_write_line(&oled, 1, "CHECK WIRING");
            ESP_LOGW(TAG, "SHT45 read failed: %s", esp_err_to_name(ret));
        }
#endif

        if (!scd_started) {
            oled_write_line(&oled, 3, "SCD41 START ERR");
            oled_write_line(&oled, 4, "CHECK SENSOR");
            oled_write_line(&oled, 5, " ");
        } else {
            bool scd_ready = false;
            bool scd_warm = (xTaskGetTickCount() - scd_start_tick) >= pdMS_TO_TICKS(SCD41_WARMUP_MS);
            esp_err_t scd_ret = scd41_data_ready(&scd, &scd_ready);
            if (scd_ret == ESP_OK && scd_ready && scd_warm) {
                scd_ret = scd41_read_measurement(&scd, &scd_co2, &scd_temp_c, &scd_rh);
                if (scd_ret == ESP_OK) {
                    scd_valid = true;
                    ESP_LOGI(TAG, "SCD41 OK: %u ppm, %.1f C, %.1f %%RH", (unsigned)scd_co2, scd_temp_c, scd_rh);
                }
            }

            if (scd_ret == ESP_OK) {
                if (!scd_warm) {
                    oled_write_line(&oled, 3, "SCD41 WARMUP...");
                    oled_write_line(&oled, 4, "WAIT 5 SECONDS");
                    oled_write_line(&oled, 5, " ");
                } else if (scd_valid) {
                    char co2_line[22];
                    char scd_temp_line[22];
                    char scd_hum_line[22];
                    snprintf(co2_line, sizeof(co2_line), "SCD41 CO2 :%4uppm", scd_co2);
                    snprintf(scd_temp_line, sizeof(scd_temp_line), "SCD41 TEMP:%5.1fC", scd_temp_c);
                    snprintf(scd_hum_line, sizeof(scd_hum_line), "SCD41 HUM :%5.1f%%", scd_rh);
                    oled_write_line(&oled, 3, co2_line);
                    oled_write_line(&oled, 4, scd_temp_line);
                    oled_write_line(&oled, 5, scd_hum_line);
                }
            } else {
                oled_write_line(&oled, 3, "SCD41 READ ERR");
                oled_write_line(&oled, 4, "CHECK WIRING");
                oled_write_line(&oled, 5, " ");
                ESP_LOGW(TAG, "SCD41 read failed: %s", esp_err_to_name(scd_ret));
            }
        }

        if (sht_ok) {
            float ah = abs_humidity_gm3(temp_c, rh);
            esp_err_t ah_ret = sgp30_set_absolute_humidity(&sgp, ah);
            if (ah_ret != ESP_OK) {
                ESP_LOGW(TAG, "SGP30 humidity set failed: %s", esp_err_to_name(ah_ret));
            }
        }

        bool sgp_warm = (xTaskGetTickCount() - sgp_start_tick) >= pdMS_TO_TICKS(SGP30_WARMUP_MS);
        esp_err_t sgp_ret = sgp30_measure_iaq(&sgp, &sgp_eco2, &sgp_tvoc);
        if (sgp_ret == ESP_OK) {
            sgp_valid = true;
        }

        if (sgp_ret == ESP_OK) {
            if (!sgp_warm) {
                oled_write_line(&oled, 7, "SGP30 WARMUP...");
            } else if (sgp_valid) {
                char sgp_line[22];
                snprintf(sgp_line, sizeof(sgp_line), "eCO2:%5u TVOC:%4u", sgp_eco2, sgp_tvoc);
                oled_write_line(&oled, 7, sgp_line);
            }
        } else {
            oled_write_line(&oled, 7, "SGP30 READ ERR");
            ESP_LOGW(TAG, "SGP30 read failed: %s", esp_err_to_name(sgp_ret));
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
