#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "motor_move.h"
#include "console_api.h"

#define UART_NUM    UART_NUM_2
#define UART_BAUD   115200
#define TX_PIN      17
#define RX_PIN      16
#define RTS_PIN     15

static const char *TAG = "main";


void app_main(void)
{
    AX_servo_conf_t AX_config = {
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