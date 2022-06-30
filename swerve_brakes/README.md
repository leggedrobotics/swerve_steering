# Swerve Brakes
The brake relays and encoders are controller by an Arduino.

## Arduino Code
The arduino code can be compiled and flashed with the Arduino IDE. The following libraries are needed on the Microcontroller:

### Encoders:
We use [Seed Studio AS5600 encoders](https://wiki.seeedstudio.com/Grove-12-bit-Magnetic-Rotary-Position-Sensor-AS5600/).
It requires the following library:
[https://github.com/Seeed-Studio/Seeed_Arduino_AS5600](https://github.com/Seeed-Studio/Seeed_Arduino_AS5600).

### I2C Multiplexer:
Since the encoders have a fixed I2C address, we need to use a multiplexer to connect to all four encoders.
We use the [Grove - 8 Channel I2C Multiplexer/I2C Hub (TCA9548A](https://wiki.seeedstudio.com/Grove-8-Channel-I2C-Multiplexer-I2C-Hub-TCA9548A). It requires the following library: [https://files.seeedstudio.com/products/103020293/document/Grove_8Channel_I2C_Hub_test_library.zip](https://files.seeedstudio.com/products/103020293/document/Grove_8Channel_I2C_Hub_test_library.zip)


### Relay Board:
We use a [Grove - 4-Channel SPDT Relay](https://wiki.seeedstudio.com/Grove-4-Channel_SPDT_Relay/) Relay board to control the brakes. It requires the following library: [https://github.com/Seeed-Studio/Multi_Channel_Relay_Arduino_Library](https://github.com/Seeed-Studio/Multi_Channel_Relay_Arduino_Library)

### ROS Interface
The ROS Interface library for Arduino can be generated with `ros-serial-arduino` by calling the following command:
```
rosrun rosserial_arduino make_libraries.py /home/[USER_HOME]/Arduino/libraries
```
Please note that the custom messages from `swerve_msgs` need to be build before exporting the ROS interface library.

### System Service
The ROS Interface can be run as a system service. This allows to autostart the driver when the computer boots.

The file `service/swerve-brakes-arduino.service` contains the service currently running on ANYMAL-C CHAP APC.
#### Installation
   1. Create a workspace with the package `swerve_msgs` and build the package.
   2. Add the correct path to the workspace to `service/swerve-brakes-arduino.service`.
   3. Edit the `service/ros.conf` file and enter the correct IP-addresses of your ros network.
   4. Copy `service/ros.conf` to `/etc/robot/` and `service/swerve-brakes-arduino.service` to `/etc/systemd/system/`.
   5. Enable the service by calling `sudo systemctl enable swerve-brakes-arduino.service`.