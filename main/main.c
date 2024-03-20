#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "motor_move.h"
#include "console_api.h"

#define UART_NUM       UART_NUM_2
#define UART_BAUD      115200
#define TX_PIN         17
#define RX_PIN         16
#define RTS_PIN        4

#define LINUX_UART_NUM UART_NUM_1
#define LINUX_TX       21
#define LINUX_RX       22

static const char *TAG = "main";


void presentation_move(float* des_joint_pos, float* cur_joint_pos, float max_rpm) 
{
    float ax_rpm[MOTORS_NUM];
    axes_interpolation(max_rpm, des_joint_pos, cur_joint_pos, ax_rpm);

    for (uint8_t i = 0; i < MOTORS_NUM; i++)
    {
        if (ax_rpm[i] <= 0.0f)
            ax_rpm[i] = 5.0f;

        single_DOF_move(i, des_joint_pos[i], ax_rpm[i], STEP_ACCEL);
    }
    wait_for_motors_stop();

    for (uint8_t i = 0; i < MOTORS_NUM; i++)
        cur_joint_pos[i] = des_joint_pos[i];
}


void app_main(void)
{
    AX_conf_t AX_config = {
        .uart = UART_NUM,
        .tx_pin = TX_PIN,
        .rx_pin = RX_PIN,
        .rts_pin = RTS_PIN,
        .baudrate = UART_BAUD
    };

    gpio_num_t emm42_step_pin[] = {26, 14};
    gpio_num_t emm42_dir_pin[] = {5, 19};
    gpio_num_t emm42_en_pin[] = {13, 2};

    emm42_conf_t emm42_config = {
        .uart = UART_NUM,
        .baudrate = UART_BAUD,
        .tx_pin = TX_PIN,
        .rx_pin = RX_PIN,
        .motor_num = 2,
        .step_pin = emm42_step_pin,
        .dir_pin = emm42_dir_pin,
        .en_pin = emm42_en_pin
    };

    gpio_num_t mks_step_pin[] = {27};
    gpio_num_t mks_dir_pin[] = {18};
    gpio_num_t mks_en_pin[] = {12};

    mks_conf_t mks_config = {
        .uart = UART_NUM,
        .baudrate = UART_BAUD,
        .tx_pin = TX_PIN,
        .rx_pin = RX_PIN,
        .motor_num = 1,
        .step_pin = mks_step_pin,
        .dir_pin = mks_dir_pin,
        .en_pin = mks_en_pin
    };

    linux_conf_t linux_config = {
        .uart_port = LINUX_UART_NUM,
        .tx_pin = LINUX_TX,
        .rx_pin = LINUX_RX
    };

    motor_init(&AX_config, &emm42_config, &mks_config, &linux_config);

    float rpm = 2.0f;

    ESP_LOGI(TAG, "Motor test");

    // go to zero position
    for (uint8_t i = 0; i < MOTORS_NUM; i++)
        single_DOF_move(i, 0.0f, rpm, STEP_ACCEL);
    wait_for_motors_stop();

    // start console
    // console_api_start();

    float max_rpm = 5.0f;
    float cur_joint_pos[MOTORS_NUM] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    float des_joint_pos[MOTORS_NUM];

    while (1)
    {
        des_joint_pos[0] = 30.0f;
        des_joint_pos[1] = -30.0f;
        des_joint_pos[2] = -90.0f;
        des_joint_pos[3] = -45.0f;
        des_joint_pos[4] = 30.0f;
        des_joint_pos[5] = 30.0f;
        presentation_move(des_joint_pos, cur_joint_pos, max_rpm);
        vTaskDelay(20000 / portTICK_PERIOD_MS);

        des_joint_pos[0] = 0.0f;
        des_joint_pos[1] = 0.0f;
        des_joint_pos[2] = 0.0f;
        des_joint_pos[3] = 0.0f;
        des_joint_pos[4] = 0.0f;
        des_joint_pos[5] = 0.0f;
        presentation_move(des_joint_pos, cur_joint_pos, max_rpm);
        vTaskDelay(10000 / portTICK_PERIOD_MS);

        des_joint_pos[0] = -45.0f;
        des_joint_pos[1] = 50.0f;
        des_joint_pos[2] = 110.0f;
        des_joint_pos[3] = -80.0f;
        des_joint_pos[4] = -40.0f;
        des_joint_pos[5] = -40.0f;
        presentation_move(des_joint_pos, cur_joint_pos, max_rpm);
        vTaskDelay(20000 / portTICK_PERIOD_MS);

        des_joint_pos[0] = 0.0f;
        des_joint_pos[1] = 0.0f;
        des_joint_pos[2] = 0.0f;
        des_joint_pos[3] = 0.0f;
        des_joint_pos[4] = 0.0f;
        des_joint_pos[5] = 0.0f;
        presentation_move(des_joint_pos, cur_joint_pos, max_rpm);
        vTaskDelay(10000 / portTICK_PERIOD_MS);

        des_joint_pos[0] = -30.0f;
        des_joint_pos[1] = -40.0f;
        des_joint_pos[2] = -90.0f;
        des_joint_pos[3] = -45.0f;
        des_joint_pos[4] = -30.0f;
        des_joint_pos[5] = -30.0f;
        presentation_move(des_joint_pos, cur_joint_pos, max_rpm);
        vTaskDelay(20000 / portTICK_PERIOD_MS);

        des_joint_pos[0] = 0.0f;
        des_joint_pos[1] = 0.0f;
        des_joint_pos[2] = 0.0f;
        des_joint_pos[3] = 0.0f;
        des_joint_pos[4] = 0.0f;
        des_joint_pos[5] = 0.0f;
        presentation_move(des_joint_pos, cur_joint_pos, max_rpm);
        vTaskDelay(10000 / portTICK_PERIOD_MS);

        des_joint_pos[0] = 45.0f;
        des_joint_pos[1] = 50.0f;
        des_joint_pos[2] = 100.0f;
        des_joint_pos[3] = -30.0f;
        des_joint_pos[4] = 60.0f;
        des_joint_pos[5] = 40.0f;
        presentation_move(des_joint_pos, cur_joint_pos, max_rpm);
        vTaskDelay(20000 / portTICK_PERIOD_MS);

        des_joint_pos[0] = 0.0f;
        des_joint_pos[1] = 0.0f;
        des_joint_pos[2] = 0.0f;
        des_joint_pos[3] = 0.0f;
        des_joint_pos[4] = 0.0f;
        des_joint_pos[5] = 0.0f;
        presentation_move(des_joint_pos, cur_joint_pos, max_rpm);
        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }

    // save encoders position before power off
    motors_save_enc_states();

    motor_deinit();

    ESP_LOGI(TAG, "safe to power off");
}