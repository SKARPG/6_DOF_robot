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
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "AX_servo.h"
#include "mks_servo.h"
#include "emm42_servo.h"
#include "inv_kin.h"
#include "nvs.h"
#include "nvs_flash.h"

#define TEST_MODE                1 // uncomment to enable test mode
// #define DEBUG_MODE               1 // uncomment to enable debug mode
#define STEP_MODE_ENABLE         1 // uncomment to enable step mode

#define MOTORS_NUM               6 // number of motors

#define EMM42_POS_TRESHOLD       0.1f // emm42 servo position treshold
#define MKS_POS_TRESHOLD         0.1f // MKS servo position treshold
#define AX_POS_TRESHOLD          1.0f // AX servo position treshold

#define FULL_ROT                 200 // full rotation of motor in steps
#define GEAR_RATIO               38.4f // gearbox ratio

#define EMM42_MIN_RPM            (2.5f / GEAR_RATIO) // emm42 servo minimum RPM
#define EMM42_MAX_RPM            (2900.0f / GEAR_RATIO) // emm42 servo maximum RPM
#define MKS_MIN_RPM              (30.0f / GEAR_RATIO) // MKS servo minimum RPM
#define MKS_MAX_RPM              (24500.0f / GEAR_RATIO) // MKS servo maximum RPM
#define AX_MIN_RPM               (5.0f * 0.111f) // AX servo minimum RPM
#define AX_MAX_RPM               105.0f // AX servo maximum RPM

#define EMM42_ACCEL              255 // emm42 servo acceleration
#define MKS_ACCEL                0 // MKS servo acceleration
#define STEP_ACCEL               0.1f // step acceleration

#define UART_WAIT                (10 / portTICK_PERIOD_MS) // UART wait time

#define FLOAT_PRECISION          100000.0f // float precision
#define NVS_DATA_KEY_SIZE        12 // NVS key size

#define LINTERPOLATION_STEP_MM   3.0f // linear interpolation mm

#define QUEUE_SIZE               100 // queue for linear interpolation size
#define MAX_POS_NUM              20 // max number of positions to learn

typedef struct lin_int_task_arg_t
{
    float max_speed;
    float* desired_pos;
    float* joint_pos;
    float* joint_start_pos;
} lin_int_task_arg_t;

typedef struct lin_int_queue_arg_t
{
    float joint_pos[6];
    float joint_rpm[6];
    bool end_move;
} lin_int_queue_arg_t;


void robot_check_constrains(float* joint_pos);

void axes_interpolation(float max_rpm, float* joint_pos, float* joint_start_pos, float* ax_rpm);

float get_motor_pos(uint8_t DOF);

void wait_for_motors_stop();

void single_DOF_move(uint8_t DOF, float position, float rpm, float accel_phase);

void robot_move_to_pos(float* desired_pos, float speed, uint8_t interpolation);

void motor_init(AX_conf_t* AX_config, emm42_conf_t* emm42_config, mks_conf_t* mks_config, linux_conf_t* linux_config);

void motor_deinit();

void motor_set_zero_pos(uint8_t DOF);

void motors_save_enc_states();

void robot_learn_pos(float max_speed, uint32_t delay_ms, uint8_t interpolation);

void robot_reset_learned_pos();

void robot_move_to_learned_pos();