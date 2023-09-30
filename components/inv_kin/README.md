# 6-DOF ROBOT RPI I2C COMMUNICTION API FOR CALCULATING INVERSE KINEMATICS (ESP-IDF) 

## Notes
* Change I2C connection parameters in `inv_kin.h` file (see `#define I2C_SLAVE_ADDR` and `#define I2C_FREQ_HZ`).
* In order to change data accuracy, change `#define DATA_ACCURACY` in `inv_kin.h` file.
* `INIT_KEY` and `I2C_SLAVE_ADDR` must be the same on ESP and RPi (see `inv_kin.h` file).

## To do list:
* debug and test


## Sources
https://github.com/JanG175/6_DOF_robot/tree/04bca78ab2b91207ac70fc0138bdd17fa5a336c4/components/inv_kin

https://github.com/JanG175/inv_kin_solv.git