#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"

#define I2C_SLAVE_ADDR 0x12
#define I2C_FREQ_HZ 100000
#define I2C_TIMEOUT_MS (100 / portTICK_PERIOD_MS)

#define DATA_ACCURACY 10000.0f
#define INIT_KEY 0x05

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

typedef struct rpi_i2c_conf_t
{
    i2c_port_t i2c_port;
    gpio_num_t sda_pin;
    gpio_num_t scl_pin;
    gpio_num_t isr_pin;
} rpi_i2c_conf_t;


void init_rpi_i2c(rpi_i2c_conf_t* rpi_i2c_config);

void deinit_rpi_i2c();

void calc_inv_kin(double* desired_pos, double* joint_pos);