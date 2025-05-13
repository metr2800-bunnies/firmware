#pragma once

#include "stdint.h"

#define SERVICE_UUID    0x1234
#define TELEM_CHAR_UUID 0x8765

typedef struct {
    uint32_t packet_num;
    int32_t encoder_counts[4];
    float rpms[4];
    float drive_power[4];
    //float raw_imu[6];
    float ff, p, i, d;
} telemetry_data_t;

extern telemetry_data_t telemetry;

void ble_telemetry_init(void);
