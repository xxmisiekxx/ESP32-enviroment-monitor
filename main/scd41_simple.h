#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_err.h"

typedef struct {
    i2c_master_bus_handle_t bus;
    i2c_master_dev_handle_t dev; // Created once and reused for all transfers.
    uint8_t addr;                // 0x62
} scd41_t;                       // structure to comunicate with sensor

esp_err_t scd41_init(scd41_t* s);
esp_err_t scd41_wakeup(scd41_t* s);
esp_err_t scd41_start_periodic_measurement(scd41_t* s);
esp_err_t scd41_stop_periodic_measurement(scd41_t* s);
esp_err_t scd41_data_ready(scd41_t* s, bool* ready);
esp_err_t scd41_read_measurement(scd41_t* s, uint16_t* co2_ppm, float* temp_c, float* rh);
