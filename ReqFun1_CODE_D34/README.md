# ELEC3848 Required Function Code

## List of programs
- `jetson_code.zip`: python program run on the jetson nano
- `main_mcu_program.zip`: PlatformIO program run on the first Arduino to control chassis, read sensor data, display, etc.
- `gyro_mcu_code.ino`: Arduino program run on the second Arduino to acquire gyroscope data and send to Jetson

Why use a seperated Arduino to read gyroscope data:
We use Hardware Interrupt to measure motor encoder output, and on Arduino it will interfere with I2C communication.
Occasionally it makes the program blocked at `Wire.requestFrom()` function
since the receiving is interrupted and cannot receive specified number of bytes. 
After tesing we found the `update()` in MPU6050 lib acquires 14 bytes on I2C and hence are more easily to be interfered.

The MCU code (PlatformIO project) will be continuously developed for our self-proposed functions. 
So we coded some modules that are not very necessary to fulfill required functions but will be useful later.

## References and related works
The author and copyright of some source files are indicated at the beginning of each file. 
If not indicated, the code is written by us (Zou Chunyu and / or Jin Yushang).

The hierarchy and implementation of the main MCU program partially refered to [https://github.com/roboMaster/roboRTS-Firmware/](https://github.com/roboMaster/roboRTS-Firmware/), 
a robot control firmware on STM32F4, and [https://github.com/EricEricEricJin/ELEC3442-Group-Project-Plane](https://github.com/EricEricEricJin/ELEC3442-Group-Project-Plane), 
a fixed-wing plane control system on RaspberryPi. Some modules are also reused with the authers indicated.

