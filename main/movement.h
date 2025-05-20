#pragma once

#define LENGTH_MM           320
#define WIDTH_MM            220
#define WHEEL_RADIUS_MM     40


void movement_set(float vx, float vy);
void movement_pid_update(int frequency_hz);
