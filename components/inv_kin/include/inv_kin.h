#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"

#define DATA_ACCURACY   10000.0f
#define INIT_KEY        0x05

#define UART_BAUD       115200
#define UART_TIMEOUT    (100 / portTICK_PERIOD_MS)

// robot dimensions in mm
#define DELTA0          77.5
#define DELTA1          96.35
#define L0              226.231
#define DELTA2          83.594
#define L1              192.487
#define DELTA3          62.423
#define DELTA4          62.427
#define DELTA5          25.94

// zero position in mm
#define X_ZERO          0.0f
#define Y_ZERO          (DELTA1 - DELTA2 + DELTA3 - DELTA5)
#define Z_ZERO          (DELTA0 + L0 + L1 + DELTA4)

// zero orientation in radians
#define PHI_ZERO        90.0f
#define PSI_ZERO        0.0f
#define THETA_ZERO      0.0f

typedef struct linux_conf_t
{
    uart_port_t uart_port;
    gpio_num_t tx_pin;
    gpio_num_t rx_pin;
} linux_conf_t;


void init_linux_pc(linux_conf_t* linux_config);

void deinit_linux_pc();

void calc_inv_kin(double* desired_pos, double* joint_pos);

void calc_forw_kin(double* cur_pos, double* joint_pos);