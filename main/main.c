#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "motor_move.h"

#define FULL_ROT 200

static const char *TAG = "main";


void app_main(void)
{
    AX_servo_conf_t AX_conf = {
        .uart = UART_NUM_0,
        .tx_pin = 1,
        .rx_pin = 3,
        .rts_pin = 5,
        .baudrate = 115200
    };

    gpio_num_t emm42_step_pin[] = {23, 14};
    gpio_num_t emm42_dir_pin[] = {22, 13};
    gpio_num_t emm42_en_pin[] = {21, 12};

    emm42_conf_t emm42_conf = {
        .uart = UART_NUM_2,
        .baudrate = 115200,
        .tx_pin = 17,
        .rx_pin = 16,
        .step_pin = emm42_step_pin,
        .dir_pin = emm42_dir_pin,
        .en_pin = emm42_en_pin
    };

    gpio_num_t mks_step_pin[] = {27};
    gpio_num_t mks_dir_pin[] = {26};
    gpio_num_t mks_en_pin[] = {25};

    mks_conf_t mks_conf = {
        .uart = UART_NUM_2,
        .baudrate = 115200,
        .tx_pin = 17,
        .rx_pin = 16,
        .step_pin = mks_step_pin,
        .dir_pin = mks_dir_pin,
        .en_pin = mks_en_pin
    };

    float motor_pos[MOTORS_NUM];
    for (uint8_t i = 0; i < MOTORS_NUM; i++)
        motor_pos[i] = 0.0f;

    motor_init(AX_conf, emm42_conf, mks_conf, motor_pos);

    int16_t max_speed = 100;
    int16_t speed = 25;
    float pos = 0.0f;

    int16_t dir = -1;

    while (1)
    {
        // if (speed >= max_speed || speed <= 0)
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