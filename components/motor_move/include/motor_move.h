/**
 * @file motor_move.h
 * @author JanG175
 * @brief 6-DOF ROBOT MOTOR SERVO API
 * 
 * @copyright All rigths reserved (R) 2023
 */

#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "AX_servo.h"
#include "mks_servo.h"
#include "emm42_servo.h"

#define MOTORS_NUM 6 // number of motors

#define EMM42_POS_TRESHOLD 1.8f // emm42 servo position treshold
#define MKS_POS_TRESHOLD 1.8f // MKS servo position treshold
#define AX_POS_TRESHOLD 1.0f // AX servo position treshold

#define FULL_ROT 200 // full rotation of motor in steps
#define GEAR_RATIO 1 // gearbox ratio

#define EMM42_ACCEL 255 // emm42 servo acceleration
#define MKS_ACCEL 0 // MKS servo acceleration

#define UART_WAIT (10 / portTICK_PERIOD_MS) // UART wait time


float get_motor_pos(AX_servo_conf_t AX_conf, emm42_conf_t emm42_conf, mks_conf_t mks_conf, uint8_t DOF);

void wait_for_motors_stop(AX_servo_conf_t AX_conf, emm42_conf_t emm42_conf, mks_conf_t mks_conf, float* motor_goal);

void single_DOF_move(AX_servo_conf_t AX_conf, emm42_conf_t emm42_conf, mks_conf_t mks_conf, uint8_t DOF, float position, uint8_t speed_percent, float* motor_goal);

void motor_init(AX_servo_conf_t AX_conf, emm42_conf_t emm42_conf, mks_conf_t mks_conf, float* motor_pos);