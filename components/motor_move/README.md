# 6-DOF ROBOT MOTOR SERVO API for ESP32 (ESP-IDF) 

## Notes
* Declare how many motors you want to use by changing `MOTORS_NUM` in `motor_move.h`.
* Define position error tresholds in `motor_move.h`:
    * EMM42_POS_TRESHOLD - emm42 servo position treshold
    * MKS_POS_TRESHOLD - MKS servo position treshold
    * AX_POS_TRESHOLD - AX servo position treshold

## To do list:
* debug and test
* normilize speeds and positions among all motors
* add position handling for AX servos (`single_DOF_move` in `motor_move.c`)

## Sources
https://github.com/JanG175/6_DOF_robot/tree/450f79c7eb65cb54c5a698a6c0e79beca74c9bbc/components/motor_move