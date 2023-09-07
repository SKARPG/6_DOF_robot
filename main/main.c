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

    motor_init(AX_conf, emm42_conf, mks_conf);

    int16_t speed = 0;
    uint8_t accel = 255;
    uint32_t pulses = 10 * FULL_ROT;

    int16_t dir = 1;

    while (1)
    {
        if (speed >= 250 || speed <= -250)
            dir = -dir;

        speed = speed + 50 * dir;
        ESP_LOGI(TAG, "speed: %d\n", speed);

        accel = 255;
        emm42_servo_uart_move(emm42_conf, 1, speed, accel, pulses);
        vTaskDelay(10 / portTICK_PERIOD_MS);

        accel = 32;
        mks_servo_uart_cr_set_pos(mks_conf, 2, speed, accel, pulses);
        vTaskDelay(10 / portTICK_PERIOD_MS);

        vTaskDelay(5000 / portTICK_PERIOD_MS);

        // ESP_LOGI(TAG, "emm42 l_enc: %f\n", emm42_servo_uart_read_encoder(emm42_conf, 1));
        ESP_LOGI(TAG, "emm42 g_enc: %f\n", emm42_servo_uart_read_motor_pos(emm42_conf, 1));
        vTaskDelay(10 / portTICK_PERIOD_MS);

        ESP_LOGI(TAG, "mks g_enc: %f\n", mks_servo_uart_read_encoder(mks_conf, 2));
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}