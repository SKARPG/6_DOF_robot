/**
 * @file console_api.h
 * @author JanG175
 * @brief 6-DOF ROBOT MOTOR CONSOLE API
 * 
 * @copyright All rigths reserved (R) 2023
 */

#pragma once

#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <inttypes.h>
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_console.h"
#include "linenoise/linenoise.h"
#include "argtable3/argtable3.h"
#include "esp_vfs_dev.h"
#include "esp_vfs_fat.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "motor_move.h"

#define PROMPT_STR "[esp32]"

#define CONSOLE_UART_NUM UART_NUM_0
#define CONSOLE_UART_BAUDRATE 115200

#define CONSOLE_MOUNT_PATH "/data"
#define CONSOLE_HISTORY_PATH CONSOLE_MOUNT_PATH "/history.txt"


void console_api_start();