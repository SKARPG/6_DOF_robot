# 6-DOF ROBOT MOTOR SERVO API for ESP32 (ESP-IDF) 

## Notes
* Declare how many motors you want to use by changing `MOTORS_NUM` in `motor_move.h`.
* Uncomment `#define STEP_MODE_ENABLE 1` in `motor_move.h` to enable step mode.
* Define position error tresholds in `motor_move.h`:
    * EMM42_POS_TRESHOLD - emm42 servo position treshold,
    * MKS_POS_TRESHOLD - MKS servo position treshold,
    * AX_POS_TRESHOLD - AX servo position treshold.
* Acceleration can be changed in `motor_move.h` (see `EMM42_ACCEL` and `MKS_ACCEL`). For now it is disabled.
* Gearbox ratio can be changed in `motor_move.h` (see `GEAR_RATIO`).
* Sometimes when multiple motors are running at the same time, there is an UART error (it is fixed by resending the command).
* All motors got CW orientation (positive angle is clockwise).
* AX servos can go from -150.0 to 150.0 degrees.

## To do list:
* debug and test
* test linear intepolation

## Sources
https://github.com/JanG175/6_DOF_robot/tree/450f79c7eb65cb54c5a698a6c0e79beca74c9bbc/components/motor_move