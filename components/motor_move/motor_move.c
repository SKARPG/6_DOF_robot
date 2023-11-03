/**
 * @file motor_move.c
 * @author JanG175
 * @brief 6-DOF ROBOT MOTOR SERVO API
 * 
 * @copyright All rigths reserved (R) 2023
 */

#include <stdio.h>
#include "motor_move.h"

static const char *TAG = "motor_move";

static portMUX_TYPE motor_spinlock = portMUX_INITIALIZER_UNLOCKED; // spinlock for critical sections

// globals
static AX_conf_t AX_conf; // struct with AX servo parameters
static emm42_conf_t emm42_conf; // struct with emm42 servo parameters
static mks_conf_t mks_conf; // struct with mks servo parameters
static float motor_pos[MOTORS_NUM]; // array with current positions for each motor in degrees
static float motor_speed[MOTORS_NUM]; // array with last speeds of each motor in rpm
static float motor_pos_offset[MOTORS_NUM]; // array with saved motor position offsets in degrees

static nvs_handle_t zero_pos_handle; // handle for nvs storage
static const char nvs_offset_key[][NVS_DATA_KEY_SIZE] = {
    "nvs_offset0", "nvs_offset1", "nvs_offset2", "nvs_offset3", "nvs_offset4", "nvs_offset5"
}; // array with nvs keys for positions offsets



#ifdef STEP_MODE_ENABLE
/**
 * @brief calculate period in microseconds for desired motor in STEP mode
 * 
 * @param rpm desired speed in rpm
 * @return period_us calculated period in microseconds
 */
static uint64_t calc_period_us(float rpm)
{
    uint64_t period_us = 0;

    if (rpm > 0.0f)
        period_us = (uint64_t)((60.0f * 1000000.0f) / (rpm * (float)FULL_ROT * GEAR_RATIO));
    else
        ESP_LOGW(TAG, "calc_period_us: invalid RPM!");

    return period_us;
}
#endif // STEP_MODE_ENABLE


/**
 * @brief calculate normalized speed for desired motor
 * 
 * @param DOF number of DOF
 * @param rpm desired speed in rpm
 * @return calculated speed
 */
static int16_t calc_speed(uint8_t DOF, float rpm)
{
    int16_t speed = 0;

    if (rpm > 0.0f)
    {
        if (DOF == 0 || DOF == 2)
        {
            speed = (int16_t)(rpm / EMM42_MAX_RPM * 1279.0f);
            if (speed > 1279)
            {
                ESP_LOGW(TAG, "emm42 servo: too high speed!");
                speed = 1279;
            }
        }
        else if (DOF == 1)
        {
            speed = (int16_t)(rpm / MKS_MAX_RPM * 1279.0f);
            if (speed > 1279)
            {
                ESP_LOGW(TAG, "mks servo: too high speed!");
                speed = 1279;
            }
        }
        else if (DOF == 3 || DOF == 4 || DOF == 5)
        {
            speed = (int16_t)(rpm / AX_MAX_RPM * 1023.0f);
            if (speed > 1023)
            {
                ESP_LOGW(TAG, "AX servo: too high speed!");
                speed = 1023;
            }
        }
        else
        {
            ESP_LOGW(TAG, "calc_speed: invalid DOF!");
        }

        // prevent locking engines at too low speed
        if (speed == 0)
            speed = 1;
    }

    return speed;
}


/**
 * @brief get motors position
 * 
 * @param DOF number of DOF
 * @return position in degrees
 */
float get_motor_pos(uint8_t DOF)
{
    float position = 0.0f;

    switch (DOF)
    {
        case 0:
            position = -emm42_servo_uart_read_motor_pos(emm42_conf, 1); // CCW orientation
            break;
        case 1:
            position = mks_servo_uart_read_encoder(mks_conf, 2);
            break;
        case 2:
            position = -emm42_servo_uart_read_motor_pos(emm42_conf, 3); // CCW orientation
            break;
        case 3:
            position = (float)AX_servo_get_pos(AX_conf, 4);
            break;
        case 4:
            position = (float)AX_servo_get_pos(AX_conf, 5);
            break;
        case 5:
            position = (float)AX_servo_get_pos(AX_conf, 6);
            break;
        default:
            ESP_LOGW(TAG, "invalid DOF");
            break;
    }

    vTaskDelay(UART_WAIT);

    // convert servo position to degrees
    if (DOF == 0 || DOF == 1 || DOF == 2)
        position = position / GEAR_RATIO;
    else if (DOF == 3 || DOF == 4 || DOF == 5)
        position = position / 1023.0f * 300.0f - 150.0f;

    // position from zero point
    portENTER_CRITICAL(&motor_spinlock);
    position -= motor_pos_offset[DOF];
    portEXIT_CRITICAL(&motor_spinlock);

    return position;
}


/**
 * @brief wait for all motors to stop
 * 
 */
void wait_for_motors_stop()
{
    bool motor_stop[MOTORS_NUM];
    for (uint32_t i = 0; i < MOTORS_NUM; i++)
        motor_stop[i] = false;

    uint8_t stop = 1;
    float pos_tresh = EMM42_POS_TRESHOLD;
    float motor_goal = 0.0f;

    while (stop)
    {
        for (uint32_t i = 0; i < MOTORS_NUM; i++)
        {
            // set acceptable position error
            if (i == 0 || i == 2)
                pos_tresh = EMM42_POS_TRESHOLD;
            else if (i == 1)
                pos_tresh = MKS_POS_TRESHOLD;
            else if (i == 3 || i == 4 || i == 5)
                pos_tresh = AX_POS_TRESHOLD;

            portENTER_CRITICAL(&motor_spinlock);
            motor_goal = motor_pos[i];
            portEXIT_CRITICAL(&motor_spinlock);

            if (fabs(get_motor_pos(i) - motor_goal) <= pos_tresh)
                motor_stop[i] = true;
            else
                motor_stop[i] = false;

            // printf("%lu\t%f\n", i, get_motor_pos(i) - motor_goal); // for debug
        }

        // printf("\n"); // for debug

        stop = 0;

        for (uint32_t i = 0; i < MOTORS_NUM; i++)
        {
            if (motor_stop[i] == false)
            {
                stop = 1;
                break;
            }
        }

        // resend move command to AX servos
        if (MOTORS_NUM > 3)
        {
            for (uint8_t i = 3; i < MOTORS_NUM; i++)
            {
                portENTER_CRITICAL(&motor_spinlock);
                float speed = motor_speed[i];
                portEXIT_CRITICAL(&motor_spinlock);

                single_DOF_move(i, motor_pos[i], speed);
            }
        }
    }

    // prevent position error accumulation
    vTaskDelay(25 / portTICK_PERIOD_MS);

    float cur_pos = 0.0f;

    for (uint32_t i = 0; i < MOTORS_NUM; i++)
    {
        cur_pos = get_motor_pos(i);

        portENTER_CRITICAL(&motor_spinlock);
        motor_pos[i] = cur_pos;
        portEXIT_CRITICAL(&motor_spinlock);
    }
}


/**
 * @brief move each DOF to position with speed
 * 
 * @param DOF number of DOF
 * @param position desired position in degrees
 * @param rpm desired speed in rpm
 */
void single_DOF_move(uint8_t DOF, float position, float rpm)
{
    portENTER_CRITICAL(&motor_spinlock);
    motor_speed[DOF] = rpm;
    portEXIT_CRITICAL(&motor_spinlock);

    uint32_t pulses = 0;
    uint16_t AX_pos = 0;

    int16_t speed = 0;

#ifndef STEP_MODE_ENABLE

    if (rpm > 0.0f)
    {
        speed = calc_speed(DOF, rpm);
    }
    else if (rpm < 0.0f)
    {
        speed = calc_speed(DOF, -rpm);
        position = -position;
    }

    if (DOF == 0 || DOF == 1 || DOF == 2)
    {

        float cur_motor_pos = 0.0f;

        portENTER_CRITICAL(&motor_spinlock);
        cur_motor_pos = motor_pos[DOF];
        portEXIT_CRITICAL(&motor_spinlock);

        // convert position in degrees to pulses
        pulses = (uint32_t)(fabs(position - cur_motor_pos) / 360.0f * (float)FULL_ROT * GEAR_RATIO);

        // CCW orientation
        if (DOF == 0 || DOF == 2)
        {
            if (position < cur_motor_pos)
                speed = -speed;
        }
        else
        {
            if (position > cur_motor_pos)
                speed = -speed;
        }
    }
    else if (DOF == 3 || DOF == 4 || DOF == 5)
    {
        // convert position in degrees to AX servo position
        AX_pos = (uint16_t)((position + 150.0f) / 300.0f * 1023.0f);

        if (AX_pos > 1023)
        {
            ESP_LOGW(TAG, "AX servo: too high position!");
            AX_pos = 1023;
        }

        if (speed > 1023)
        {
            ESP_LOGW(TAG, "AX servo: too high speed!");
            speed = 1023;
        }
    }

    switch (DOF)
    {
        case 0:
            emm42_servo_uart_move(emm42_conf, 1, speed, EMM42_ACCEL, pulses);
            break;
        case 1:
            mks_servo_uart_cr_set_pos(mks_conf, 2, speed, MKS_ACCEL, pulses);
            break;
        case 2:
            emm42_servo_uart_move(emm42_conf, 3, speed, EMM42_ACCEL, pulses);
            break;
        case 3:
            if (speed != 0)
                AX_servo_set_pos_w_spd(AX_conf, 4, AX_pos, speed);
            break;
        case 4:
            if (speed != 0)
                AX_servo_set_pos_w_spd(AX_conf, 5, AX_pos, speed);
            break;
        case 5:
            if (speed != 0)
                AX_servo_set_pos_w_spd(AX_conf, 6, AX_pos, speed);
            break;
        default:
            ESP_LOGW(TAG, "invalid DOF!");
            break;
    }

#else

    int64_t period_us = 0;

    if (DOF == 0 || DOF == 1 || DOF == 2)
    {
        if (rpm > 0.0f)
            period_us = calc_period_us(rpm);
        else
        {
            period_us = calc_period_us(-rpm);
            position = -position;
        }

        float cur_motor_pos = 0.0f;

        portENTER_CRITICAL(&motor_spinlock);
        cur_motor_pos = motor_pos[DOF];
        portEXIT_CRITICAL(&motor_spinlock);

        // convert position in degrees to pulses
        pulses = (uint32_t)(fabs(position - cur_motor_pos) / 360.0f * (float)FULL_ROT * GEAR_RATIO);

        if (position < cur_motor_pos)
            period_us = -period_us;
    }
    else if (DOF == 3 || DOF == 4 || DOF == 5)
    {
        if (rpm > 0.0f)
        {
            speed = calc_speed(DOF, rpm);
        }
        else if (rpm < 0.0f)
        {
            speed = calc_speed(DOF, -rpm);
            position = -position;
        }

        // convert position in degrees to AX servo position
        AX_pos = (uint16_t)((position + 150.0f) / 300.0f * 1023.0f);

        if (AX_pos > 1023)
        {
            ESP_LOGW(TAG, "AX servo: too high position!");
            AX_pos = 1023;
        }

        if (speed > 1023)
        {
            ESP_LOGW(TAG, "AX servo: too high speed!");
            speed = 1023;
        }
    }

    switch (DOF)
    {
        case 0:
            if (period_us != 0)
                emm42_servo_step_move(emm42_conf, 0, pulses, period_us);
            break;
        case 1:
            if (period_us != 0)
                mks_servo_step_move(mks_conf, 0, pulses, period_us);
            break;
        case 2:
            if (period_us != 0)
                emm42_servo_step_move(emm42_conf, 1, pulses, period_us);
            break;
        case 3:
            if (speed != 0)
                AX_servo_set_pos_w_spd(AX_conf, 4, AX_pos, speed);
            break;
        case 4:
            if (speed != 0)
                AX_servo_set_pos_w_spd(AX_conf, 5, AX_pos, speed);
            break;
        case 5:
            if (speed != 0)
                AX_servo_set_pos_w_spd(AX_conf, 6, AX_pos, speed);
            break;
        default:
            ESP_LOGW(TAG, "invalid DOF!");
            break;
    }

#endif // STEP_MODE_ENABLE

    vTaskDelay(UART_WAIT);

    // update goal position
    if (rpm != 0)
    {
        float update_pos = 0.0f;

        if (DOF == 0 || DOF == 1 || DOF == 2)
        {

#ifndef STEP_MODE_ENABLE
            update_pos = (float)(speed/abs(speed)) * (float)pulses / (float)FULL_ROT * 360.0f / GEAR_RATIO;

            if (DOF == 0 || DOF == 2) // CCW orientation
                update_pos = -update_pos;
#else
            update_pos = -(float)(period_us/llabs(period_us)) * (float)pulses / (float)FULL_ROT * 360.0f / GEAR_RATIO;
#endif // STEP_MODE_ENABLE

            portENTER_CRITICAL(&motor_spinlock);
            motor_pos[DOF] -= update_pos;
            portEXIT_CRITICAL(&motor_spinlock);
        }
        else if (DOF == 3 || DOF == 4 || DOF == 5)
        {
            update_pos = (float)AX_pos / 1023.0f * 300.0f - 150.0f;

            portENTER_CRITICAL(&motor_spinlock);
            motor_pos[DOF] = update_pos;
            portEXIT_CRITICAL(&motor_spinlock);
        }
    }
}


/**
 * @brief move end effector of robot to desired position with desired speed
 * 
 * @param desired_pos pointer to array with desired position in mm and degrees
 * @param rpm desired speed in rpm
 */
void robot_move_to_pos(double* desired_pos, float rpm)
{
    double joint_pos[MOTORS_NUM];

    for (uint8_t i = 0; i < MOTORS_NUM; i++)
        joint_pos[i] = (double)motor_pos[i];

    calc_inv_kin(desired_pos, joint_pos);

    // printf("joint pos:\n");
    // for (uint8_t i = 0; i < MOTORS_NUM; i++)
    //     printf("%f\t", joint_pos[i]);
    // printf("\n");

    for (uint8_t i = 0; i < MOTORS_NUM; i++)
        single_DOF_move(i, (float)joint_pos[i], rpm);
    wait_for_motors_stop();
}


/**
 * @brief initialize all motors
 * 
 * @param AX_config pointer to a struct with AX servo parameters
 * @param emm42_config pointer to a struct with emm42 servo parameters
 * @param mks_config pointer to a struct with mks servo parameters
 * @param rpi_i2c_config pointer to a struct with rpi i2c parameters
 */
void motor_init(AX_conf_t* AX_config, emm42_conf_t* emm42_config, mks_conf_t* mks_config, rpi_i2c_conf_t* rpi_i2c_config)
{
    portENTER_CRITICAL(&motor_spinlock);
    AX_conf = *AX_config;
    emm42_conf = *emm42_config;
    mks_conf = *mks_config;
    portEXIT_CRITICAL(&motor_spinlock);

    emm42_servo_init(emm42_conf);
    mks_servo_init(mks_conf);
    AX_servo_init(AX_conf);

    init_rpi_i2c(rpi_i2c_config);

    vTaskDelay(UART_WAIT);

    if (MOTORS_NUM > 3)
    {
        for (uint32_t i = 4; i < MOTORS_NUM + 1; i++)
        {
            AX_servo_write_register(AX_conf, i, AX_RETURN_LEVEL, AX_RETURN_READ);
            vTaskDelay(UART_WAIT);

            AX_servo_write_register(AX_conf, i, AX_RETURN_DELAY_TIME, 250);
            vTaskDelay(UART_WAIT);

            AX_servo_set_shutdown_alarm(AX_conf, i, 5);
            vTaskDelay(UART_WAIT);

            AX_servo_set_angle_limit(AX_conf, i, 0, 1023);
            vTaskDelay(UART_WAIT);

            AX_servo_set_max_torque(AX_conf, i, 1022);
            vTaskDelay(UART_WAIT);
        }
    }

    for (uint8_t i = 0; i < MOTORS_NUM; i++)
    {
        portENTER_CRITICAL(&motor_spinlock);
        motor_speed[i] = 0.0f;
        portEXIT_CRITICAL(&motor_spinlock);
    }

    // initialize NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    // open Non-Volatile Storage (NVS) handle
    err = nvs_open("zero_pos", NVS_READWRITE, &zero_pos_handle);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));

    // read zero position from NVS
    for (uint32_t i = 0; i < MOTORS_NUM; i++)
    {
        // set default value to 0, if not set yet in NVS
        int32_t data = 0;

        err = nvs_get_i32(zero_pos_handle, nvs_offset_key[i], &data);
        switch (err)
        {
            case ESP_OK:
                portENTER_CRITICAL(&motor_spinlock);
                motor_pos_offset[i] = (float)data / FLOAT_PRECISION;
                portEXIT_CRITICAL(&motor_spinlock);
                break;
            case ESP_ERR_NVS_NOT_FOUND:
                portENTER_CRITICAL(&motor_spinlock);
                motor_pos_offset[i] = 0.0f;
                portEXIT_CRITICAL(&motor_spinlock);

                ESP_LOGW(TAG, "The value is not initialized yet!");
                break;
            default :
                ESP_LOGE(TAG, "Error (%s) reading!", esp_err_to_name(err));
        }
    }

    vTaskDelay(100 / portTICK_PERIOD_MS);

    // get all motors positions
    for (uint32_t i = 0; i < MOTORS_NUM; i++)
    {
        float cur_pos = get_motor_pos(i);

        portENTER_CRITICAL(&motor_spinlock);
        motor_pos[i] = cur_pos;
        portEXIT_CRITICAL(&motor_spinlock);
    }
}


/**
 * @brief deinitialize all motors
 * 
 */
void motor_deinit()
{
    AX_servo_deinit(AX_conf);
    emm42_servo_deinit(emm42_conf);
    mks_servo_deinit(mks_conf);

    deinit_rpi_i2c();

    vTaskDelay(UART_WAIT);

    nvs_close(zero_pos_handle);
}


/**
 * @brief set current motor position as zero
 * 
 * @param DOF motor address
 */
void motor_set_zero_pos(uint8_t DOF)
{
    // zero offset
    portENTER_CRITICAL(&motor_spinlock);
    motor_pos_offset[DOF] = 0.0f;
    portEXIT_CRITICAL(&motor_spinlock);

    // get new offset from global zero position
    float offset = get_motor_pos(DOF);

    portENTER_CRITICAL(&motor_spinlock);
    motor_pos_offset[DOF] = offset;
    motor_pos[DOF] = 0.0f;
    portEXIT_CRITICAL(&motor_spinlock);

    // write zero position to nvs
    int32_t data = (int32_t)(offset * FLOAT_PRECISION);
    esp_err_t err = nvs_set_i32(zero_pos_handle, nvs_offset_key[DOF], data);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "Error (%s) setting NVS value!", esp_err_to_name(err));

    // commit written value
    err = nvs_commit(zero_pos_handle);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "Error (%s) committing NVS!", esp_err_to_name(err));

    vTaskDelay(100 / portTICK_PERIOD_MS);
}


/**
 * @brief save encoder position to nvs before power off
 * 
 * @param DOF motor address
 */
void motor_save_enc_states(uint8_t DOF)
{
    portENTER_CRITICAL(&motor_spinlock);
    float offset = motor_pos_offset[DOF];
    portEXIT_CRITICAL(&motor_spinlock);

    if (DOF == 0 || DOF == 1 || DOF == 2)
    {
        // get_motor_pos(DOF) = pos - offset
        float enc = get_motor_pos(DOF) + offset;
        offset -= enc;

        // mks servo encoder does not zero itself after reset
        if (DOF == 1)
        {
            while (offset > 180.0f)
                offset -= 360.0f;
            while (offset < -180.0f)
                offset += 360.0f;
        }
    }

    portENTER_CRITICAL(&motor_spinlock);
    motor_pos_offset[DOF] = offset;
    portEXIT_CRITICAL(&motor_spinlock);

    // write zero position to nvs
    int32_t data = (int32_t)(offset * FLOAT_PRECISION);
    esp_err_t err = nvs_set_i32(zero_pos_handle, nvs_offset_key[DOF], data);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "Error (%s) setting NVS value!", esp_err_to_name(err));

    // commit written value
    err = nvs_commit(zero_pos_handle);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "Error (%s) committing NVS!", esp_err_to_name(err));

    vTaskDelay(100 / portTICK_PERIOD_MS);
}