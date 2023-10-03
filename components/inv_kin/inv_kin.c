#include <stdio.h>
#include "inv_kin.h"

static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
static rpi_i2c_conf_t rpi_i2c_conf;

// static const char* TAG = "rpi_i2c_inv_kin";


/**
 * @brief initialize i2c connection with rpi
 * 
 * @param rpi_i2c_config struct with i2c parameters
 */
void init_rpi_i2c(rpi_i2c_conf_t* rpi_i2c_config)
{
    portENTER_CRITICAL(&mux);
    rpi_i2c_conf = *rpi_i2c_config;
    portEXIT_CRITICAL(&mux);

    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pin_bit_mask = (1 << rpi_i2c_conf.isr_pin);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = 0;
    io_conf.pull_down_en = 0;
    gpio_config(&io_conf);
    gpio_set_level(rpi_i2c_conf.isr_pin, 0);

    i2c_config_t conf_slave = {
        .sda_io_num = rpi_i2c_conf.sda_pin,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = rpi_i2c_conf.scl_pin,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .mode = I2C_MODE_SLAVE,
        .slave.addr_10bit_en = 0,
        .slave.slave_addr = I2C_SLAVE_ADDR,
        .slave.maximum_speed = I2C_FREQ_HZ,
        .clk_flags = 0
    };
    i2c_param_config(rpi_i2c_conf.i2c_port, &conf_slave);
    ESP_ERROR_CHECK(i2c_driver_install(rpi_i2c_conf.i2c_port, conf_slave.mode, 2048, 2048, 0));

    // init connection
    uint8_t init = 0x00;

    while (init != INIT_KEY)
    {
        gpio_set_level(rpi_i2c_conf.isr_pin, 1);
        i2c_slave_read_buffer(rpi_i2c_conf.i2c_port, &init, sizeof(init), I2C_TIMEOUT_MS);
        gpio_set_level(rpi_i2c_conf.isr_pin, 0);
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}


/**
 * @brief deinit i2c connection with rpi
 * 
 */
void deinit_rpi_i2c()
{
    ESP_ERROR_CHECK(i2c_driver_delete(rpi_i2c_conf.i2c_port));
}


/**
 * @brief send desired positions to rpi and receive solved inverse kinematics joint positions
 * 
 * @param desired_pos pointer to array of desired positions
 * @param joint_pos pointer to array of joint positions
 */
void calc_inv_kin(double* desired_pos, double* joint_pos)
{
    // prepare data
    uint8_t w_desired_pos[6][8];
    for (uint8_t i = 0; i < 6; i++)
    {
        desired_pos[i] *= DATA_ACCURACY;

        w_desired_pos[i][0] = ((int64_t)desired_pos[i] >> 56) & 0xFF;
        w_desired_pos[i][1] = ((int64_t)desired_pos[i] >> 48) & 0xFF;
        w_desired_pos[i][2] = ((int64_t)desired_pos[i] >> 40) & 0xFF;
        w_desired_pos[i][3] = ((int64_t)desired_pos[i] >> 32) & 0xFF;
        w_desired_pos[i][4] = ((int64_t)desired_pos[i] >> 24) & 0xFF;
        w_desired_pos[i][5] = ((int64_t)desired_pos[i] >> 16) & 0xFF;
        w_desired_pos[i][6] = ((int64_t)desired_pos[i] >> 8) & 0xFF;
        w_desired_pos[i][7] = ((int64_t)desired_pos[i] >> 0) & 0xFF;

        desired_pos[i] /= DATA_ACCURACY;
    }

    uint8_t w_joint_pos[6][8];
    for (uint8_t i = 0; i < 6; i++)
    {
        joint_pos[i] *= DATA_ACCURACY;

        w_joint_pos[i][0] = ((int64_t)joint_pos[i] >> 56) & 0xFF;
        w_joint_pos[i][1] = ((int64_t)joint_pos[i] >> 48) & 0xFF;
        w_joint_pos[i][2] = ((int64_t)joint_pos[i] >> 40) & 0xFF;
        w_joint_pos[i][3] = ((int64_t)joint_pos[i] >> 32) & 0xFF;
        w_joint_pos[i][4] = ((int64_t)joint_pos[i] >> 24) & 0xFF;
        w_joint_pos[i][5] = ((int64_t)joint_pos[i] >> 16) & 0xFF;
        w_joint_pos[i][6] = ((int64_t)joint_pos[i] >> 8) & 0xFF;
        w_joint_pos[i][7] = ((int64_t)joint_pos[i] >> 0) & 0xFF;

        joint_pos[i] /= DATA_ACCURACY;
    }

    uint8_t r_joint_pos[6][8];

    // notify rpi that esp wants to send data
    gpio_set_level(rpi_i2c_conf.isr_pin, 1);

    // send data
    for (uint8_t i = 0; i < 6; i++)
        i2c_slave_write_buffer(rpi_i2c_conf.i2c_port, w_desired_pos[i], sizeof(w_desired_pos[i]), I2C_TIMEOUT_MS);

    for (uint8_t i = 0; i < 6; i++)
        i2c_slave_write_buffer(rpi_i2c_conf.i2c_port, w_joint_pos[i], sizeof(w_joint_pos[i]), I2C_TIMEOUT_MS);

    // receive data
    for (uint8_t i = 0; i < 6; i++)
        i2c_slave_read_buffer(rpi_i2c_conf.i2c_port, r_joint_pos[i], sizeof(r_joint_pos[i]), portMAX_DELAY);

    for (uint8_t i = 0; i < 6; i++)
        joint_pos[i] = (double)((int64_t)r_joint_pos[i][0] << 56 | (int64_t)r_joint_pos[i][1] << 48 |
                                (int64_t)r_joint_pos[i][2] << 40 | (int64_t)r_joint_pos[i][3] << 32 |
                                (int64_t)r_joint_pos[i][4] << 24 | (int64_t)r_joint_pos[i][5] << 16 |
                                (int64_t)r_joint_pos[i][6] << 8 | (int64_t)r_joint_pos[i][7] << 0
                               ) / DATA_ACCURACY;

    gpio_set_level(rpi_i2c_conf.isr_pin, 0);

    // // for debug
    // for (uint8_t i = 0; i < 6; i++)
    //     ESP_LOGI(TAG, "received joint_pos[%u]: %f", i, joint_pos[i]);
}