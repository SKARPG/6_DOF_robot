#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "AX_servo.h"
#include "mks_servo.h"
#include "emm42_servo.h"


void motor_init(AX_servo_conf_t AX_conf, emm42_conf_t emm42_conf, mks_conf_t mks_conf);

void single_DOF_move(AX_servo_conf_t AX_conf, emm42_conf_t emm42_conf, mks_conf_t mks_conf, uint8_t DOF, uint32_t position, int16_t speed);