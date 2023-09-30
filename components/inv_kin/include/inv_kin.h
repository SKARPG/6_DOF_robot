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

// zero position in mm
#define X_ZERO          0.0f
#define Y_ZERO          37.0f
#define Z_ZERO          593.0f

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

void calc_inv_kin(double* desired_pos, double* joint_pos);