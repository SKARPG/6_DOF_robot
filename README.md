# 6-DOF robot main program

## Add custom components
In order to add additional components to project open new `Bash` terminal in the project directory and run following commands:

```bash
cd components
git clone https://github.com/JanG175/mks_servo.git
git clone https://github.com/JanG175/emm42_servo.git
git clone https://github.com/JanG175/AX_servo.git
```

## Configure SDK
It is necessary to change partition table to custom one (`partitions.csv`) and set flash size to 4 MB. In order to do that go to `menuconfig` and search for:
* `Partition Table` settings

![Partition table settings](README_images/image.png)

* `Serial flasher config` settings

![Flash size settings](README_images/image-1.png)

## RPi program
RPi works as an inverse kinematic solver. It receives data via I2C protocol from ESP32 and calculates angles for each servo. Then it sends calculated angles back to ESP32.

In order to get RPi inverse kinematics solver program follow these commands in `Bash` terminal:

```bash
cd ~/Desktop
git clone https://github.com/JanG175/inv_kin_solv.git
mkdir build
cd build
cmake --build .
./inv_kin_solv
```

Then setup this program as a startup program following these instructions:
https://learn.sparkfun.com/tutorials/how-to-run-a-raspberry-pi-program-on-startup#method-3-systemd