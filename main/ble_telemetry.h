#pragma once

#include "stdint.h"

typedef struct {
    uint32_t encoder_counts[4];
    float rpms[4];
    float drive_power[4];
    float raw_imu[6];
} telemetry_data_t;

#define SERVICE_UUID    0x1234
#define TELEM_CHAR_UUID 0x8765

