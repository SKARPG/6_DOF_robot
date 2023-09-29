#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "motor_move.h"
#include "console_api.h"
#include "inv_kin.h"

#define UART_NUM    UART_NUM_2
#define UART_BAUD   115200
#define TX_PIN      17
#define RX_PIN      16
#define RTS_PIN     15

#define RPI_INTR_PIN 18

#define I2C_NUM I2C_NUM_0
#define I2C_SDA 23
#define I2C_SCL 19

static const char *TAG = "main";


void app_main(void)
{
    rpi_i2c_conf_t rpi_i2c_config = {
        .i2c_port = I2C_NUM,
        .sda_pin = I2C_SDA,
        .scl_pin = I2C_SCL,
        .isr_pin = RPI_INTR_PIN
    };
    init_rpi_i2c(&rpi_i2c_config);

    // calculate inverse kinematics
    double desired_pos[6] = {X_ZERO, Y_ZERO, Z_ZERO, PHI_ZERO, PSI_ZERO, THETA_ZERO};
    double joint_pos[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

    desired_pos[0] = 90.0f;
    desired_pos[1] = 101.0f;
    desired_pos[2] = 574.0f;
    desired_pos[3] = 45.0f;
    desired_pos[4] = -68.0f;
    desired_pos[5] = 49.0f;

    joint_pos[0] = 10.0f;
    joint_pos[1] = 50.0f;
    joint_pos[2] = 10.0f;
    joint_pos[3] = 20.0f;
    joint_pos[4] = -20.0f;
    joint_pos[5] = 90.0f;

    calc_inv_kin(desired_pos, joint_pos);
    vTaskDelay(30000/portTICK_PERIOD_MS);


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

    motor_init(&AX_config, &emm42_config, &mks_config);

    int16_t speed = 25;
    float pos = 0.0f;

    int16_t dir = -1;

    for (uint8_t i = 0; i < MOTORS_NUM; i++)
        single_DOF_move(i, 0.0f, speed);

    // start console
    console_api_start();

    while (1)
    {
        // if (speed >= 100 || speed <= 0)
        //     dir = -dir;

        // speed = speed + 5 * dir;
        // ESP_LOGI(TAG, "speed: %d\n", speed);

        if (pos >= 120.0f || pos <= 0.0f)
            dir = -dir;
        pos += (float)dir;
        ESP_LOGI(TAG, "pos: %f\n", pos);

        // ======================================================================

        for (uint8_t i = 0; i < 3; i++)
            single_DOF_move(i, pos, speed);
        wait_for_motors_stop();

        for (uint8_t i = 0; i < MOTORS_NUM; i++)
            ESP_LOGI(TAG, "motor %d pos: %f\n", i, get_motor_pos(i));
        ESP_LOGI(TAG, "==================================================");

        vTaskDelay(100 / portTICK_PERIOD_MS);

        // ======================================================================

        for (uint8_t i = 0; i < 3; i++)
            single_DOF_move(i, -pos, speed);
        wait_for_motors_stop();

        for (uint8_t i = 0; i < MOTORS_NUM; i++)
            ESP_LOGI(TAG, "motor %d pos: %f\n", i, get_motor_pos(i));
        ESP_LOGI(TAG, "==================================================");

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}