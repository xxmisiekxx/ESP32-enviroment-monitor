#pragma once

#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_err.h"

typedef struct {
    i2c_master_bus_handle_t bus;
    i2c_master_dev_handle_t dev; // Created once and reused for all transfers.
    uint8_t addr; // 0x58
} sgp30_t;

esp_err_t sgp30_init(sgp30_t* s);
esp_err_t sgp30_iaq_init(sgp30_t* s);
esp_err_t sgp30_measure_iaq(sgp30_t* s, uint16_t* eco2_ppm, uint16_t* tvoc_ppb);
