#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "motor_move.h"
#include "console_api.h"

#define UART_NUM     UART_NUM_2
#define UART_BAUD    115200
#define TX_PIN       17
#define RX_PIN       16
#define RTS_PIN      15

#define RPI_INTR_PIN 19

#define I2C_NUM      I2C_NUM_0
#define I2C_SDA      21
#define I2C_SCL      22

static const char *TAG = "main";


void app_main(void)
{
    AX_conf_t AX_config = {
        .uart = UART_NUM,
        .tx_pin = TX_PIN,
        .rx_pin = RX_PIN,
        .rts_pin = RTS_PIN,
        .baudrate = UART_BAUD
    };

    emm42_conf_t emm42_config = {
        .uart = UART_NUM,
        .baudrate = UART_BAUD,
        .tx_pin = TX_PIN,
        .rx_pin = RX_PIN
    };

    mks_conf_t mks_config = {
        .uart = UART_NUM,
        .baudrate = UART_BAUD,
        .tx_pin = TX_PIN,
        .rx_pin = RX_PIN
    };

    rpi_i2c_conf_t rpi_i2c_config = {
        .i2c_port = I2C_NUM,
        .sda_pin = I2C_SDA,
        .scl_pin = I2C_SCL,
        .isr_pin = RPI_INTR_PIN
    };

    motor_init(&AX_config, &emm42_config, &mks_config, &rpi_i2c_config);

    // desired end effector position for inverse kinematics
    double desired_pos[6] = {X_ZERO, Y_ZERO, Z_ZERO, PHI_ZERO, PSI_ZERO, THETA_ZERO};

    int16_t speed = 10;

    for (uint8_t i = 0; i < MOTORS_NUM; i++)
        single_DOF_move(i, 0.0f, speed);
    wait_for_motors_stop();

    // start console
    console_api_start();

/*
    while (1)
    {
        // mm
        desired_pos[0] = 90.0f;
        desired_pos[1] = 101.0f;
        desired_pos[2] = 574.0f;
        // deg
        desired_pos[3] = 45.0f;
        desired_pos[4] = -68.0f;
        desired_pos[5] = 49.0f;

        ESP_LOGI(TAG, "moving to position 0");
        robot_move_to_pos(desired_pos, speed);
        ESP_LOGI(TAG, "moved to position 0");

        // mm
        desired_pos[0] = 105.0f;
        desired_pos[1] = 80.0f;
        desired_pos[2] = 554.0f;
        // deg
        desired_pos[3] = 45.0f;
        desired_pos[4] = -68.0f;
        desired_pos[5] = 52.0f;

        ESP_LOGI(TAG, "moving to position 1");
        robot_move_to_pos(desired_pos, speed);
        ESP_LOGI(TAG, "moved to position 1");
    }
*/

    // go back to zero position before power off

    for (uint8_t i = 0; i < MOTORS_NUM; i++)
        single_DOF_move(i, 0.0f, speed);
    wait_for_motors_stop();

    for (uint8_t i = 0; i < MOTORS_NUM; i++)
        motor_reset_zero_pos(i);

    motor_deinit();
}