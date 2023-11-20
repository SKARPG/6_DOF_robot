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
static QueueHandle_t rpm_queue = NULL;

// globals
static AX_conf_t AX_conf; // struct with AX servo parameters
static emm42_conf_t emm42_conf; // struct with emm42 servo parameters
static mks_conf_t mks_conf; // struct with mks servo parameters
static float motor_pos[MOTORS_NUM]; // array with current positions for each motor in degrees
static float motor_speed[MOTORS_NUM]; // array with last speeds of each motor in rpm
static float motor_pos_offset[MOTORS_NUM]; // array with saved motor position offsets in degrees

static nvs_handle_t zero_pos_handle; // handle for nvs storage
static const char nvs_offset_key[][NVS_DATA_KEY_SIZE] = {
    "nvs_offset0", "nvs_offset1", "nvs_offset2", "nvs_offset3", "nvs_offset4", "nvs_offset5", "is_cal"
}; // array with nvs keys for positions offsets


/**
 * @brief calculate speeds for each motor for linear interpolation
 * 
 * @param max_speed maximal desired speed in mm/s
 * @param desired_pos pointer to the array with desired position in mm and degrees
 * @param cur_pos pointer to the array with current position in mm and degrees
 * @param joint_pos pointer to the array with goal positions in degrees
 * @param joint_start_pos pointer to the array with start positions in degrees
 * @param lin_rpm pointer to the array with calculated speeds in rpm
*/
static void linear_interpolation(float max_speed, float* desired_pos, float* cur_pos, float* joint_pos, float* joint_start_pos, float* lin_rpm)
{
    // calculate path length
    float s = sqrt(pow((desired_pos[0] - cur_pos[0]), 2.0f) + pow((desired_pos[1] - cur_pos[1]), 2.0f) + pow((desired_pos[2] - cur_pos[2]), 2.0f)); // mm

    // TODO: move with only degrees

    float t = s / max_speed; // mm/(mm/s) = s

    // find the longest path
    float max_s = 0.0f;

    for (uint8_t i = 0; i < 6; i++)
    {
        if (fabs(joint_pos[i] - joint_start_pos[i]) > max_s)
            max_s = fabs(joint_pos[i] - joint_start_pos[i]); // deg
    }

    float max_rpm = (max_s / 360.0f) / (t / 60.0f); // (deg/360)/(s/60) = rev/min = rpm

    // check for NaN
    if (isnan(max_rpm))
        max_rpm = 2.0f;

    axes_interpolation(max_rpm, joint_pos, joint_start_pos, lin_rpm);
}


/**
 * @brief task for calculating linear interpolation speeds
 * 
 * @param pvParameters pointer to the task arguments
*/
static void lin_int_task(void* pvParameters)
{
    lin_int_task_arg_t* arg = (lin_int_task_arg_t*)pvParameters;
    float joint_rpm[6];
    lin_int_queue_arg_t queue_arg;

    queue_arg.end_move = false;

    float cur_pos[6];
    // calculate current position
    calc_forw_kin(cur_pos, arg->joint_start_pos);

    // find the largest position difference
    uint8_t max_i_mm = 0;
    for (uint8_t i = 0; i < 3; i++)
    {
        if (fabs(arg->desired_pos[max_i_mm] - cur_pos[max_i_mm]) < fabs(arg->desired_pos[i] - cur_pos[i]))
            max_i_mm = i;
    }

    // calculate position step
    uint32_t steps_num_mm = (uint32_t)(fabs((arg->desired_pos[max_i_mm] - cur_pos[max_i_mm]) / LINTERPOLATION_STEP_MM));

    float step_len_mm[3] = {0.0f, 0.0f, 0.0f};
    float step_len_deg[3] = {0.0f, 0.0f, 0.0f};

    for (uint8_t i = 0; i < 3; i++)
    {
        if (steps_num_mm != 0)
            step_len_mm[i] = (arg->desired_pos[i] - cur_pos[i]) / (float)steps_num_mm;
        else
            step_len_mm[i] = arg->desired_pos[i] - cur_pos[i];

        if (steps_num_mm != 0)
            step_len_deg[i] = (arg->desired_pos[i + 3] - cur_pos[i + 3]) / (float)steps_num_mm;
        else
            step_len_deg[i] = arg->desired_pos[i + 3] - cur_pos[i + 3];
    }

    // linear interpolation
    do
    {
        // calculate new desired position
        for (uint8_t i = 0; i < 3; i++)
        {
            arg->desired_pos[i] = cur_pos[i] + step_len_mm[i];
            arg->desired_pos[i + 3] = cur_pos[i + 3] + step_len_deg[i];
        }
    
        // calculate inverse kinematics
        calc_inv_kin(arg->desired_pos, arg->joint_pos);
        // constraint joint positions
        robot_check_constrains(arg->joint_pos);
        // calculate speeds for each motor
        linear_interpolation(arg->max_speed, arg->desired_pos, cur_pos, arg->joint_pos, arg->joint_start_pos, joint_rpm);

        // move to new position
        for (uint8_t i = 0; i < 6; i++)
        {
            queue_arg.joint_pos[i] = arg->joint_pos[i];
            queue_arg.joint_rpm[i] = joint_rpm[i];
        }
        if (steps_num_mm == 1)
            queue_arg.end_move = true;

        xQueueSend(rpm_queue, &queue_arg, portMAX_DELAY);

        // set new current position
        for (uint8_t i = 0; i < 6; i++)
        {
            cur_pos[i] = arg->desired_pos[i];
            arg->joint_start_pos[i] = arg->joint_pos[i];
        }

        if (steps_num_mm > 0)
            steps_num_mm--;
    }
    while (steps_num_mm > 0);

    vTaskDelete(NULL);
}


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

    // set max speed
    if (period_us < (uint64_t)(1.0f / (AX_MAX_RPM * (float)FULL_ROT * GEAR_RATIO / (60.0f * 1000000.0f))))
    {
        period_us = (uint64_t)(1.0f / (AX_MAX_RPM * (float)FULL_ROT * GEAR_RATIO / (60.0f * 1000000.0f)));
        ESP_LOGW(TAG, "calc_period_us: too high speed!");
    }

    // prevent locking engines at too low speeds
    if (period_us > (uint64_t)(1.0f / (AX_MIN_RPM * (float)FULL_ROT * GEAR_RATIO / (60.0f * 1000000.0f))))
    {
        period_us = (uint64_t)(1.0f / (AX_MIN_RPM * (float)FULL_ROT * GEAR_RATIO / (60.0f * 1000000.0f)));
        ESP_LOGW(TAG, "calc_period_us: too too low speed!");
    }

    return period_us;
}
#endif // STEP_MODE_ENABLE


/**
 * @brief calculate normalized speed for desired motor
 * 
 * @param DOF number of DOF
 * @param rpm desired speed in rpm
 * @param accel_phase acceleration percentage (inactive in UART mode)
 * @return calculated speed
 */
static int16_t calc_speed(uint8_t DOF, float rpm, float accel_phase)
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
#ifdef STEP_MODE_ENABLE
            // accel move takes 120 % of move without acceleration time
            rpm = rpm / (1.0f + 2.0f * accel_phase);
#endif // STEP_MODE_ENABLE

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

        // prevent locking engines at too low speeds
        if (speed < 5)
            speed = 5;
    }

    return speed;
}


/**
 * @brief check if desired position is in constraints
 * 
 * @param joint_pos pointer to array with joint positions in degrees
*/
void robot_check_constrains(float* joint_pos)
{
    if (joint_pos[1] > 90.0f)
        joint_pos[1] = 90.0f;
    else if (joint_pos[1] < -90.0f)
        joint_pos[1] = -90.0f;
    
    if (joint_pos[2] > 135.0f)
        joint_pos[2] = 135.0f;
    else if (joint_pos[2] < -135.0f)
        joint_pos[2] = -135.0f;

    for (uint8_t i = 3; i < 6; i++)
    {
        if (joint_pos[i] > 150.0f)
            joint_pos[i] = 150.0f;
        else if (joint_pos[i] < -150.0f)
            joint_pos[i] = -150.0f;
    }

    if (joint_pos[1] > 90.0f || joint_pos[1] < -90.0f || joint_pos[2] > 135.0f || joint_pos[2] < -135.0f ||
            joint_pos[3] > 150.0f || joint_pos[3] < -150.0f || joint_pos[4] > 150.0f || joint_pos[4] < -150.0f ||
            joint_pos[5] > 150.0f || joint_pos[5] < -150.0f)
        ESP_LOGW(TAG, "out of constraints alert!");
}


/**
 * @brief calculate speeds for each motor for linear interpolation
 * 
 * @param max_rpm maximal desired speed in rpm
 * @param joint_pos array with goal positions in degrees
 * @param joint_start_pos array with start positions in degrees
 * @param ax_rpm array with calculated speeds in rpm
 */
void axes_interpolation(float max_rpm, float* joint_pos, float* joint_start_pos, float* ax_rpm)
{
    uint8_t max_i = 0;
    float max_s = 0.0f;

    // find the longest path
    for (uint8_t i = 0; i < MOTORS_NUM; i++)
    {
        if (fabs(joint_pos[i] - joint_start_pos[i]) > max_s)
        {
            max_i = i;
            max_s = fabs(joint_pos[i] - joint_start_pos[i]); // deg
        }
    }

    ax_rpm[max_i] = max_rpm;

    // v=s/t => t=s/v
    float t = max_s / max_rpm; // deg/(rev/min) = min*deg/rev

    for (uint8_t i = 0; i < MOTORS_NUM; i++)
    {
        if (i != max_i)
            ax_rpm[i] = fabs(joint_pos[i] - joint_start_pos[i]) / t; // deg/(min*deg/rev) = rev/min = rpm

        // check for NaN
        if (isnan(ax_rpm[i]))
            ax_rpm[i] = 2.0f;
    }
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
    }

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
 * @param accel_phase acceleration percentage (inactive in UART mode)
 */
void single_DOF_move(uint8_t DOF, float position, float rpm, float accel_phase)
{
    portENTER_CRITICAL(&motor_spinlock);
    motor_speed[DOF] = rpm;
    portEXIT_CRITICAL(&motor_spinlock);

    uint32_t pulses = 0;
    uint16_t AX_pos = 0;

    int16_t speed = 0;

#ifndef STEP_MODE_ENABLE
    accel_phase = 0.0f;

    if (rpm > 0.0f)
    {
        speed = calc_speed(DOF, rpm, accel_phase);
    }
    else if (rpm < 0.0f)
    {
        speed = calc_speed(DOF, -rpm, accel_phase);
        position = -position;
    }

    if (DOF == 0 || DOF == 1 || DOF == 2)
    {
        float cur_motor_pos = get_motor_pos(DOF);

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

        float cur_motor_pos = get_motor_pos(DOF);

        // convert position in degrees to pulses
        pulses = (uint32_t)(fabs(position - cur_motor_pos) / 360.0f * (float)FULL_ROT * GEAR_RATIO);

        if (position < cur_motor_pos)
            period_us = -period_us;
    }
    else if (DOF == 3 || DOF == 4 || DOF == 5)
    {
        if (rpm > 0.0f)
        {
            speed = calc_speed(DOF, rpm, accel_phase);
        }
        else if (rpm < 0.0f)
        {
            speed = calc_speed(DOF, -rpm, accel_phase);
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
            if (period_us != 0 && pulses != 0)
                emm42_servo_step_move(emm42_conf, 0, pulses, period_us, accel_phase);
            break;
        case 1:
            if (period_us != 0 && pulses != 0)
                mks_servo_step_move(mks_conf, 0, pulses, period_us, accel_phase);
            break;
        case 2:
            if (period_us != 0 && pulses != 0)
                emm42_servo_step_move(emm42_conf, 1, pulses, period_us, accel_phase);
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
 * @param speed desired speed
 * @param interpolation 0 - no interpolation [rpm], 1 - axes interpolation [rpm], 2 - linear interpolation [mm/s]
 */
void robot_move_to_pos(float* desired_pos, float speed, uint8_t interpolation)
{
    float joint_start_pos[MOTORS_NUM];
    float joint_pos[MOTORS_NUM];
    float joint_rpm[MOTORS_NUM];

    for (uint8_t i = 0; i < MOTORS_NUM; i++)
    {
        joint_pos[i] = get_motor_pos(i);
        joint_start_pos[i] = joint_pos[i];
    }

    if (interpolation == 0) // no interpolation
    {
        // calculate inverse kinematics
        calc_inv_kin(desired_pos, joint_pos);
        // constraint joint positions
        robot_check_constrains(joint_pos);
        // move to new position
        for (uint8_t i = 0; i < MOTORS_NUM; i++)
            single_DOF_move(i, joint_pos[i], speed, STEP_ACCEL);
        wait_for_motors_stop();
    }
    else if (interpolation == 1) // axes interpolation
    {
        // calculate inverse kinematics
        calc_inv_kin(desired_pos, joint_pos);
        // constraint joint positions
        robot_check_constrains(joint_pos);
        // calculate speeds for each motor
        axes_interpolation(speed, joint_pos, joint_start_pos, joint_rpm);
        // move to new position
        for (uint8_t i = 0; i < MOTORS_NUM; i++)
            single_DOF_move(i, joint_pos[i], joint_rpm[i], STEP_ACCEL);
        wait_for_motors_stop();
    }
    else if (interpolation == 2) // linear interpolation
    {
        lin_int_task_arg_t task_arg = {
            .max_speed = speed,
            .desired_pos = desired_pos,
            .joint_pos = joint_pos,
            .joint_start_pos = joint_start_pos
        };
        xTaskCreate(lin_int_task, "linear interpolation task", 4096, (void*)&task_arg, 3, NULL);

        lin_int_queue_arg_t queue_arg;

        // linear interpolation
        do
        {
            if (xQueueReceive(rpm_queue, &queue_arg, portMAX_DELAY) == pdTRUE)
            {
                // move to new position
                for (uint8_t i = 0; i < MOTORS_NUM; i++)
                    single_DOF_move(i, queue_arg.joint_pos[i], queue_arg.joint_rpm[i], 0.0f);

                wait_for_motors_stop();
            }
        }
        while (queue_arg.end_move == false);
    }
    else
        ESP_LOGW(TAG, "invalid interpolation mode!");
}


/**
 * @brief initialize all motors
 * 
 * @param AX_config pointer to a struct with AX servo parameters
 * @param emm42_config pointer to a struct with emm42 servo parameters
 * @param mks_config pointer to a struct with mks servo parameters
 * @param rpi_i2c_config pointer to a struct with rpi i2c parameters
 */
void motor_init(AX_conf_t* AX_config, emm42_conf_t* emm42_config, mks_conf_t* mks_config, linux_conf_t* linux_config)
{
    rpm_queue = xQueueCreate(QUEUE_SIZE, sizeof(lin_int_queue_arg_t));
    if (rpm_queue == NULL)
        ESP_LOGE(TAG, "failed to create rpm queue!");

    portENTER_CRITICAL(&motor_spinlock);
    AX_conf = *AX_config;
    emm42_conf = *emm42_config;
    mks_conf = *mks_config;
    portEXIT_CRITICAL(&motor_spinlock);

    emm42_servo_init(emm42_conf);
    mks_servo_init(mks_conf);
    AX_servo_init(AX_conf);

    init_linux_pc(linux_config);

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

            AX_servo_set_C_margin(AX_conf, i, 0, 0);
            vTaskDelay(UART_WAIT);

            AX_servo_set_C_slope(AX_conf, i, 2, 2);
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

    // get all motors positions
    for (uint32_t i = 0; i < MOTORS_NUM; i++)
    {
        float cur_pos = get_motor_pos(i);

        portENTER_CRITICAL(&motor_spinlock);
        motor_pos[i] = cur_pos;
        portEXIT_CRITICAL(&motor_spinlock);
    }

    // read calibration flag
    int8_t is_calibrated;
    err = nvs_get_i8(zero_pos_handle, nvs_offset_key[6], &is_calibrated);
    switch (err)
    {
        case ESP_OK:
            if (is_calibrated == 0)
            {
                ESP_LOGW(TAG, "Robot is not calibrated - set robot to zero position!");
                for (uint8_t i = 0; i < MOTORS_NUM; i++)
                    motor_set_zero_pos(i);
            }
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            is_calibrated = 0;
            ESP_LOGW(TAG, "The value is not initialized yet!");
            break;
        default :
            ESP_LOGE(TAG, "Error (%s) reading!", esp_err_to_name(err));
    }

    // set new calibration flag
    is_calibrated = 0;
    err = nvs_set_i8(zero_pos_handle, nvs_offset_key[6], is_calibrated);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "Error (%s) setting NVS value!", esp_err_to_name(err));

    // commit written value
    err = nvs_commit(zero_pos_handle);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "Error (%s) committing NVS!", esp_err_to_name(err));
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

    deinit_linux_pc();

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
 */
void motors_save_enc_states()
{
    esp_err_t err;

    for (uint8_t DOF = 0; DOF < MOTORS_NUM; DOF++)
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
        err = nvs_set_i32(zero_pos_handle, nvs_offset_key[DOF], data);
        if (err != ESP_OK)
            ESP_LOGE(TAG, "Error (%s) setting NVS value!", esp_err_to_name(err));

        // commit written value
        err = nvs_commit(zero_pos_handle);
        if (err != ESP_OK)
            ESP_LOGE(TAG, "Error (%s) committing NVS!", esp_err_to_name(err));
    }

    // save calibration flag
    int8_t is_calibrated = 1;
    err = nvs_set_i8(zero_pos_handle, nvs_offset_key[6], is_calibrated);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "Error (%s) setting NVS value!", esp_err_to_name(err));

    // commit written value
    err = nvs_commit(zero_pos_handle);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "Error (%s) committing NVS!", esp_err_to_name(err));
}