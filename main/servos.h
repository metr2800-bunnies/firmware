#pragma once

#define SERVO1_GPIO     GPIO_NUM_18
#define SERVO2_GPIO     GPIO_NUM_8
#define SERVO3_GPIO     GPIO_NUM_46

void servo_init(void);
void servo_pinion(float value);
void servo_winch(float value);
