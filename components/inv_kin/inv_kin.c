#include <stdio.h>
#include "inv_kin.h"

static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
static linux_conf_t linux_conf;

static const char* TAG = "linux_conf_inv_kin";


/**
 * @brief convert radians to degrees
 * 
 * @param rad angle in radians
*/
static double rad2deg(double rad)
{
    return (rad * 180.0f / M_PI);
}


/**
 * @brief convert degrees to radians
 * 
 * @param deg angle in degrees
*/
static double deg2rad(double deg)
{
    return (deg * M_PI / 180.0f);
}


/**
 * @brief initialize uart connection with linux pc
 * 
 * @param linux_conf_config struct with uart parameters
 */
void init_linux_pc(linux_conf_t* linux_config)
{
    portENTER_CRITICAL(&mux);
    linux_conf = *linux_config;
    portEXIT_CRITICAL(&mux);

    // configure UART
    uart_config_t uart_config = {
            .baud_rate = UART_BAUD,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .rx_flow_ctrl_thresh = 0,
            .source_clk = UART_SCLK_DEFAULT
        };

    if (uart_is_driver_installed(linux_conf.uart_port) == true)
        ESP_ERROR_CHECK(uart_driver_delete(linux_conf.uart_port));

    ESP_ERROR_CHECK(uart_driver_install(linux_conf.uart_port, 2048, 2048, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(linux_conf.uart_port, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(linux_conf.uart_port, linux_conf.tx_pin, linux_conf.rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // init connection
    uint8_t init = 0x00;

    while (init != INIT_KEY)
    {
        uart_read_bytes(linux_conf.uart_port, &init, sizeof(init), portMAX_DELAY);

        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    init = INIT_KEY;
    uart_write_bytes(linux_conf.uart_port, (const char*)&init, sizeof(init));
    ESP_ERROR_CHECK(uart_wait_tx_done(linux_conf.uart_port, UART_TIMEOUT));
}


/**
 * @brief deinit uart connection with linux pc
 * 
 */
void deinit_linux_pc()
{
    ESP_ERROR_CHECK(uart_driver_delete(linux_conf.uart_port));
}


/**
 * @brief send desired positions to rpi and receive solved inverse kinematics joint positions
 * 
 * @param desired_pos pointer to array of desired positions
 * @param joint_pos pointer to array of joint positions
 */
void calc_inv_kin(float* desired_pos, float* joint_pos)
{
    // prepare data
    uint8_t w_desired_pos[6][8];
    for (uint8_t i = 0; i < 6; i++)
    {
        desired_pos[i] *= DATA_ACCURACY;

        w_desired_pos[i][0] = ((int64_t)((double)desired_pos[i]) >> 56) & 0xFF;
        w_desired_pos[i][1] = ((int64_t)((double)desired_pos[i]) >> 48) & 0xFF;
        w_desired_pos[i][2] = ((int64_t)((double)desired_pos[i]) >> 40) & 0xFF;
        w_desired_pos[i][3] = ((int64_t)((double)desired_pos[i]) >> 32) & 0xFF;
        w_desired_pos[i][4] = ((int64_t)((double)desired_pos[i]) >> 24) & 0xFF;
        w_desired_pos[i][5] = ((int64_t)((double)desired_pos[i]) >> 16) & 0xFF;
        w_desired_pos[i][6] = ((int64_t)((double)desired_pos[i]) >> 8) & 0xFF;
        w_desired_pos[i][7] = ((int64_t)((double)desired_pos[i]) >> 0) & 0xFF;

        desired_pos[i] /= DATA_ACCURACY;
    }

    uint8_t w_joint_pos[6][8];
    for (uint8_t i = 0; i < 6; i++)
    {
        joint_pos[i] *= DATA_ACCURACY;

        w_joint_pos[i][0] = ((int64_t)((double)joint_pos[i]) >> 56) & 0xFF;
        w_joint_pos[i][1] = ((int64_t)((double)joint_pos[i]) >> 48) & 0xFF;
        w_joint_pos[i][2] = ((int64_t)((double)joint_pos[i]) >> 40) & 0xFF;
        w_joint_pos[i][3] = ((int64_t)((double)joint_pos[i]) >> 32) & 0xFF;
        w_joint_pos[i][4] = ((int64_t)((double)joint_pos[i]) >> 24) & 0xFF;
        w_joint_pos[i][5] = ((int64_t)((double)joint_pos[i]) >> 16) & 0xFF;
        w_joint_pos[i][6] = ((int64_t)((double)joint_pos[i]) >> 8) & 0xFF;
        w_joint_pos[i][7] = ((int64_t)((double)joint_pos[i]) >> 0) & 0xFF;

        joint_pos[i] /= DATA_ACCURACY;
    }

    uint8_t r_joint_pos[6][8];

    // send data
    for (uint8_t i = 0; i < 6; i++)
    {
        uart_write_bytes(linux_conf.uart_port, (const char*)w_desired_pos[i], sizeof(w_desired_pos[i]));
        ESP_ERROR_CHECK(uart_wait_tx_done(linux_conf.uart_port, UART_TIMEOUT));
    }

    for (uint8_t i = 0; i < 6; i++)
    {
        uart_write_bytes(linux_conf.uart_port, (const char*)w_joint_pos[i], sizeof(w_joint_pos[i]));
        ESP_ERROR_CHECK(uart_wait_tx_done(linux_conf.uart_port, UART_TIMEOUT));
    }

    // receive data
    for (uint8_t i = 0; i < 6; i++)
    {
        uint32_t rd_len = uart_read_bytes(linux_conf.uart_port, r_joint_pos[i], sizeof(r_joint_pos[i]), portMAX_DELAY);

        if (rd_len != sizeof(r_joint_pos[i]))
            ESP_LOGE(TAG, "uart_read_bytes() error");
    }

    for (uint8_t i = 0; i < 6; i++)
        joint_pos[i] = (float)((double)((int64_t)r_joint_pos[i][0] << 56 | (int64_t)r_joint_pos[i][1] << 48 |
                                (int64_t)r_joint_pos[i][2] << 40 | (int64_t)r_joint_pos[i][3] << 32 |
                                (int64_t)r_joint_pos[i][4] << 24 | (int64_t)r_joint_pos[i][5] << 16 |
                                (int64_t)r_joint_pos[i][6] << 8 | (int64_t)r_joint_pos[i][7] << 0
                               ) / DATA_ACCURACY);

    // // for debug
    // for (uint8_t i = 0; i < 6; i++)
    //     ESP_LOGI(TAG, "received joint_pos[%u]: %f", i, joint_pos[i]);
}


/**
 * @brief calculate current position of robot
 * 
 * @param cur_pos pointer to the array with current position in mm and degrees
 * @param joint_pos pointer to the array with current joint positions in degrees
*/
void calc_forw_kin(float* cur_pos, float* joint_pos)
{
    double angle[6];
    for (uint8_t i = 0; i < 6; i++)
        angle[i] = (double)deg2rad(joint_pos[i]);

    cur_pos[0] = (float)(DELTA2*sin(angle[0]) - DELTA1*sin(angle[0]) - DELTA3*sin(angle[0]) + L0*cos(angle[0])*sin(angle[1]) - DELTA5*cos(angle[4])*sin(angle[0]) - L1*cos(angle[0])*cos(angle[1])*sin(angle[2]) + L1*cos(angle[0])*cos(angle[2])*sin(angle[1]) + DELTA4*cos(angle[0])*cos(angle[1])*cos(angle[2])*sin(angle[3]) - DELTA4*cos(angle[0])*cos(angle[1])*cos(angle[3])*sin(angle[2]) + DELTA4*cos(angle[0])*cos(angle[2])*cos(angle[3])*sin(angle[1]) + DELTA4*cos(angle[0])*sin(angle[1])*sin(angle[2])*sin(angle[3]) - DELTA5*cos(angle[0])*cos(angle[1])*sin(angle[2])*sin(angle[3])*sin(angle[4]) + DELTA5*cos(angle[0])*cos(angle[2])*sin(angle[1])*sin(angle[3])*sin(angle[4]) - DELTA5*cos(angle[0])*cos(angle[3])*sin(angle[1])*sin(angle[2])*sin(angle[4]) - DELTA5*cos(angle[0])*cos(angle[1])*cos(angle[2])*cos(angle[3])*sin(angle[4]));

    cur_pos[1] = (float)(DELTA1*cos(angle[0]) - DELTA2*cos(angle[0]) + DELTA3*cos(angle[0]) + DELTA5*cos(angle[0])*cos(angle[4]) + L0*sin(angle[0])*sin(angle[1]) - L1*cos(angle[1])*sin(angle[0])*sin(angle[2]) + L1*cos(angle[2])*sin(angle[0])*sin(angle[1]) + DELTA4*cos(angle[1])*cos(angle[2])*sin(angle[0])*sin(angle[3]) - DELTA4*cos(angle[1])*cos(angle[3])*sin(angle[0])*sin(angle[2]) + DELTA4*cos(angle[2])*cos(angle[3])*sin(angle[0])*sin(angle[1]) + DELTA4*sin(angle[0])*sin(angle[1])*sin(angle[2])*sin(angle[3]) - DELTA5*cos(angle[1])*cos(angle[2])*cos(angle[3])*sin(angle[0])*sin(angle[4]) - DELTA5*cos(angle[1])*sin(angle[0])*sin(angle[2])*sin(angle[3])*sin(angle[4]) + DELTA5*cos(angle[2])*sin(angle[0])*sin(angle[1])*sin(angle[3])*sin(angle[4]) - DELTA5*cos(angle[3])*sin(angle[0])*sin(angle[1])*sin(angle[2])*sin(angle[4]));

    cur_pos[2] = (float)(DELTA0 + L0*cos(angle[1]) + L1*cos(angle[1])*cos(angle[2]) + L1*sin(angle[1])*sin(angle[2]) + DELTA4*cos(angle[1])*cos(angle[2])*cos(angle[3]) + DELTA4*cos(angle[1])*sin(angle[2])*sin(angle[3]) - DELTA4*cos(angle[2])*sin(angle[1])*sin(angle[3]) + DELTA4*cos(angle[3])*sin(angle[1])*sin(angle[2]) + DELTA5*cos(angle[1])*cos(angle[2])*sin(angle[3])*sin(angle[4]) - DELTA5*cos(angle[1])*cos(angle[3])*sin(angle[2])*sin(angle[4]) + DELTA5*cos(angle[2])*cos(angle[3])*sin(angle[1])*sin(angle[4]) + DELTA5*sin(angle[1])*sin(angle[2])*sin(angle[3])*sin(angle[4]));

    double sin_phi = cos(angle[1])*cos(angle[2])*cos(angle[3])*cos(angle[5]) + cos(angle[1])*cos(angle[5])*sin(angle[2])*sin(angle[3]) - cos(angle[2])*cos(angle[5])*sin(angle[1])*sin(angle[3]) + cos(angle[3])*cos(angle[5])*sin(angle[1])*sin(angle[2]) - cos(angle[1])*cos(angle[2])*cos(angle[4])*sin(angle[3])*sin(angle[5]) + cos(angle[1])*cos(angle[3])*cos(angle[4])*sin(angle[2])*sin(angle[5]) - cos(angle[2])*cos(angle[3])*cos(angle[4])*sin(angle[1])*sin(angle[5]) - cos(angle[4])*sin(angle[1])*sin(angle[2])*sin(angle[3])*sin(angle[5]);
    double cos_phi = cos(angle[1])*cos(angle[2])*sin(angle[3])*sin(angle[4]) - cos(angle[1])*cos(angle[3])*sin(angle[2])*sin(angle[4]) + cos(angle[2])*cos(angle[3])*sin(angle[1])*sin(angle[4]) + sin(angle[1])*sin(angle[2])*sin(angle[3])*sin(angle[4]);

    cur_pos[3] = (float)atan2(sin_phi, cos_phi);

    double sin_theta = cos(angle[1])*cos(angle[2])*sin(angle[0])*sin(angle[3])*sin(angle[5]) - cos(angle[0])*cos(angle[5])*sin(angle[4]) - cos(angle[1])*cos(angle[3])*sin(angle[0])*sin(angle[2])*sin(angle[5]) + cos(angle[2])*cos(angle[3])*sin(angle[0])*sin(angle[1])*sin(angle[5]) + sin(angle[0])*sin(angle[1])*sin(angle[2])*sin(angle[3])*sin(angle[5]) - cos(angle[1])*cos(angle[2])*cos(angle[3])*cos(angle[4])*cos(angle[5])*sin(angle[0]) - cos(angle[1])*cos(angle[4])*cos(angle[5])*sin(angle[0])*sin(angle[2])*sin(angle[3]) + cos(angle[2])*cos(angle[4])*cos(angle[5])*sin(angle[0])*sin(angle[1])*sin(angle[3]) - cos(angle[3])*cos(angle[4])*cos(angle[5])*sin(angle[0])*sin(angle[1])*sin(angle[2]);
    double cos_theta = cos(angle[5])*sin(angle[0])*sin(angle[4]) + cos(angle[0])*cos(angle[1])*cos(angle[2])*sin(angle[3])*sin(angle[5]) - cos(angle[0])*cos(angle[1])*cos(angle[3])*sin(angle[2])*sin(angle[5]) + cos(angle[0])*cos(angle[2])*cos(angle[3])*sin(angle[1])*sin(angle[5]) + cos(angle[0])*sin(angle[1])*sin(angle[2])*sin(angle[3])*sin(angle[5]) - cos(angle[0])*cos(angle[1])*cos(angle[2])*cos(angle[3])*cos(angle[4])*cos(angle[5]) - cos(angle[0])*cos(angle[1])*cos(angle[4])*cos(angle[5])*sin(angle[2])*sin(angle[3]) + cos(angle[0])*cos(angle[2])*cos(angle[4])*cos(angle[5])*sin(angle[1])*sin(angle[3]) - cos(angle[0])*cos(angle[3])*cos(angle[4])*cos(angle[5])*sin(angle[1])*sin(angle[2]);

    cur_pos[5] = (float)atan2(sin_theta, cos_theta);

    double sin_psi = -(cos(angle[1])*cos(angle[2])*cos(angle[3])*sin(angle[5]) + cos(angle[1])*sin(angle[2])*sin(angle[3])*sin(angle[5]) - cos(angle[2])*sin(angle[1])*sin(angle[3])*sin(angle[5]) + cos(angle[3])*sin(angle[1])*sin(angle[2])*sin(angle[5]) + cos(angle[1])*cos(angle[2])*cos(angle[4])*cos(angle[5])*sin(angle[3]) - cos(angle[1])*cos(angle[3])*cos(angle[4])*cos(angle[5])*sin(angle[2]) + cos(angle[2])*cos(angle[3])*cos(angle[4])*cos(angle[5])*sin(angle[1]) + cos(angle[4])*cos(angle[5])*sin(angle[1])*sin(angle[2])*sin(angle[3]));
    double cos_psi = (cos(angle[5])*sin(angle[0])*sin(angle[4]) + cos(angle[0])*cos(angle[1])*cos(angle[2])*sin(angle[3])*sin(angle[5]) - cos(angle[0])*cos(angle[1])*cos(angle[3])*sin(angle[2])*sin(angle[5]) + cos(angle[0])*cos(angle[2])*cos(angle[3])*sin(angle[1])*sin(angle[5]) + cos(angle[0])*sin(angle[1])*sin(angle[2])*sin(angle[3])*sin(angle[5]) - cos(angle[0])*cos(angle[1])*cos(angle[2])*cos(angle[3])*cos(angle[4])*cos(angle[5]) - cos(angle[0])*cos(angle[1])*cos(angle[4])*cos(angle[5])*sin(angle[2])*sin(angle[3]) + cos(angle[0])*cos(angle[2])*cos(angle[4])*cos(angle[5])*sin(angle[1])*sin(angle[3]) - cos(angle[0])*cos(angle[3])*cos(angle[4])*cos(angle[5])*sin(angle[1])*sin(angle[2])) / cos(cur_pos[5]);

    cur_pos[4] = (float)atan2(sin_psi, cos_psi);

    // convert radians to degrees
    for (uint8_t i = 3; i < 6; i++)
        cur_pos[i] = rad2deg(cur_pos[i]);
}