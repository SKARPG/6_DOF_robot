#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "AX_servo.h"
#include "mks_servo.h"
#include "emm42_servo.h"


void app_main(void)
{
    AX_servo_conf_t AX_conf = {
        .uart = UART_NUM_2,
        .tx_pin = 17,
        .rx_pin = 16,
        .rts_pin = 5,
        .baudrate = 115200
    };

    AX_servo_init(AX_conf);

    gpio_num_t emm42_step_pin[] = {18, 19};
    gpio_num_t emm42_dir_pin[] = {12, 14};
    gpio_num_t emm42_en_pin[] = {27, 26};

    emm42_conf_t emm42_conf = {
        .uart = UART_NUM_0,
        .baudrate = 115200,
        .tx_pin = 1,
        .rx_pin = 3,
        .step_pin = emm42_step_pin,
        .dir_pin = emm42_dir_pin,
        .en_pin = emm42_en_pin
    };

    emm42_servo_init(emm42_conf); 

    gpio_num_t mks_step_pin[] = {13};
    gpio_num_t mks_dir_pin[] = {15};
    gpio_num_t mks_en_pin[] = {25};

    mks_conf_t mks_conf = {
        .uart = UART_NUM_0,
        .baudrate = 115200,
        .tx_pin = 1,
        .rx_pin = 3,
        .step_pin = mks_step_pin,
        .dir_pin = mks_dir_pin,
        .en_pin = mks_en_pin
    };

    mks_servo_init(mks_conf);

    while (1)
    {
        printf("loop\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}