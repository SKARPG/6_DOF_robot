/**
 * @file motor_move.h
 * @author JanG175
 * @brief 6-DOF ROBOT MOTOR SERVO API
 * 
 * @copyright All rigths reserved (R) 2023
 */

#pragma once

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
#include "inv_kin.h"
#include "nvs.h"
#include "nvs_flash.h"

#define MOTORS_NUM 6 // number of motors

#define EMM42_POS_TRESHOLD 0.05f // emm42 servo position treshold
#define MKS_POS_TRESHOLD 0.05f // MKS servo position treshold
#define AX_POS_TRESHOLD 5.0f // AX servo position treshold

#define FULL_ROT 200 // full rotation of motor in steps
#define GEAR_RATIO 38.4f // gearbox ratio

#define EMM42_MAX_RPM (2200.0f / GEAR_RATIO) // emm42 servo maximum RPM
#define MKS_MAX_RPM (((1279.0f * 6000.0f) / (float)FULL_ROT) / GEAR_RATIO) // MKS servo maximum RPM
#define AX_MAX_RPM 114.0f // AX servo maximum RPM

#define EMM42_ACCEL 255 // emm42 servo acceleration
#define MKS_ACCEL 0 // MKS servo acceleration

#define UART_WAIT (10 / portTICK_PERIOD_MS) // UART wait time

#define FLOAT_PRECISION 100000.0f // float precision
#define NVS_DATA_KEY_SIZE 12 // NVS key size


float get_motor_pos(uint8_t DOF);

void wait_for_motors_stop();

void single_DOF_move(uint8_t DOF, float position, float rpm);

void robot_move_to_pos(double* desired_pos, float rpm);

void motor_init(AX_conf_t* AX_config, emm42_conf_t* emm42_config, mks_conf_t* mks_config, rpi_i2c_conf_t* rpi_i2c_config);

void motor_deinit();

void motor_set_zero_pos(uint8_t DOF);

void motor_save_enc_states(uint8_t DOF);