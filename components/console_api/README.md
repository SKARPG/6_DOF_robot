# 6-DOF ROBOT MOTOR CONSOLE API for ESP32 (ESP-IDF) 

## Notes
* Console works the best on Putty.
* Implemented commands:
    * `help` - prints the list of available commands,
    * `servo_move <DOF> <pos> <rpm>` - moves servo to a given position with a given speed in RPM (negative positions have to be parsed, for example, like that: `servo_move -- 5 -90.0 25`); negative speed is not acceptable,
    * `servo_get_pos <DOF>` - gets servo position,
    * `robot_move_to_pos <x> <y> <z> <phi> <psi> <theta> <speed> <inter>` - moves robot end effector to a given position with a given speed and with a given interpolation type (n - none [RPM], a - axes interpolation [RPM], l - linear interpolation [mm/s]),
    * `servo_set_zero_pos <DOF>` - sets current position of a servo as zero,
    * `robot_go_to_zero_pos <rpm>` - moves robot to zero position with a given speed in RPM,
    * `robot_get_pos` - gets robot's end effector position.
* `ctrl + c` - exits the console.

## To do list:
* debug and test
* register rest of the commands

## Sources
https://github.com/JanG175/6_DOF_robot/tree/763b2879e24a50b3380067f1b9e4b25f2b77ebfa/components/console
