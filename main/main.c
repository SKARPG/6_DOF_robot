#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "motor_move.h"

#define UART_NUM    UART_NUM_2
#define UART_BAUD   115200
#define TX_PIN      17
#define RX_PIN      16
#define RTS_PIN     15

static const char *TAG = "main";


void app_main(void)
{
    AX_servo_conf_t AX_conf = {
        .uart = UART_NUM,
        .tx_pin = TX_PIN,
        .rx_pin = RX_PIN,
        .rts_pin = RTS_PIN,
        .baudrate = UART_BAUD
    };

    emm42_conf_t emm42_conf = {
        .uart = UART_NUM,
        .baudrate = UART_BAUD,
        .tx_pin = TX_PIN,
        .rx_pin = RX_PIN
    };

    mks_conf_t mks_conf = {
        .uart = UART_NUM,
        .baudrate = UART_BAUD,
        .tx_pin = TX_PIN,
        .rx_pin = RX_PIN
    };

    float motor_pos[MOTORS_NUM];
    for (uint8_t i = 0; i < MOTORS_NUM; i++)
        motor_pos[i] = 0.0f;

    motor_init(AX_conf, emm42_conf, mks_conf, motor_pos);

    int16_t speed = 25;
    float pos = 0.0f;

    int16_t dir = -1;

    single_DOF_move(AX_conf, emm42_conf, mks_conf, 3, 0.0f, speed, motor_pos);
    single_DOF_move(AX_conf, emm42_conf, mks_conf, 4, 0.0f, speed, motor_pos);
    single_DOF_move(AX_conf, emm42_conf, mks_conf, 5, 0.0f, speed, motor_pos);

    while (1)
    {
        // if (speed >= 100 || speed <= 0)
        //     dir = -dir;

        // speed = speed + 5 * dir;
        // ESP_LOGI(TAG, "speed: %d\n", speed);

        if (pos >= 180.0f || pos <= 0.0f)
            dir = -dir;
        pos += (float)dir;
        ESP_LOGI(TAG, "pos: %f\n", pos);

        // ======================================================================

        single_DOF_move(AX_conf, emm42_conf, mks_conf, 0, pos, speed, motor_pos);
        single_DOF_move(AX_conf, emm42_conf, mks_conf, 1, pos, speed, motor_pos);
        wait_for_motors_stop(AX_conf, emm42_conf, mks_conf, motor_pos);

        for (uint8_t i = 0; i < MOTORS_NUM; i++)
            ESP_LOGI(TAG, "motor %d pos: %f\n", i, motor_pos[i]);
        ESP_LOGI(TAG, "==================================================");

        vTaskDelay(100 / portTICK_PERIOD_MS);

        // ======================================================================

        single_DOF_move(AX_conf, emm42_conf, mks_conf, 0, -pos, speed, motor_pos);
        single_DOF_move(AX_conf, emm42_conf, mks_conf, 1, -pos, speed, motor_pos);
        wait_for_motors_stop(AX_conf, emm42_conf, mks_conf, motor_pos);

        for (uint8_t i = 0; i < MOTORS_NUM; i++)
            ESP_LOGI(TAG, "motor %d pos: %f\n", i, motor_pos[i]);
        ESP_LOGI(TAG, "==================================================");

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}