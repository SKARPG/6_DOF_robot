# 6-DOF ROBOT MOTOR CONSOLE API for ESP32 (ESP-IDF) 

## Notes
* Console works the best on Putty.
* Implemented commands:
    * `help` - prints the list of available commands,
    * `servo_move <1|2|3|4|5|6> <float> <0-100>` - moves servo to a given position with a given speed (negative positions have to be parsed, for example, like that: `servo_move -- 5 -90.0 25`),
    * `servo_get_pos <1|2|3|4|5|6>` - gets servo position,
    * `robot_move_to_pos <double> <double> <double> <double> <double> <double> <0 - 100>` - moves robot end effector to a desired position,
    * `motor_zero_pos <1|2|3|4|5|6>` - sets current position of a motor as zero.

## To do list:
* debug and test
* check if parameters are being parsed correctly
* register rest of the commands
* get rid of artifacts in console

## Sources
https://github.com/JanG175/6_DOF_robot/tree/763b2879e24a50b3380067f1b9e4b25f2b77ebfa/components/console
