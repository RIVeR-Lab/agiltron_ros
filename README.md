# Agiltron ROS

## A simple ROS Wrapper for the Agiltron Fiber Switch

### Instructions for Use

#### Installation
```
pip install pyserial
cd catkin_ws/src
git clone <<AGILTRON_ROS_REPO>>
cd ..
catkin build # or catkin_make
source devel/setup.bash
rosrun agiltron_ros switcher.py
```
### Testing
Publish a message to the channel. Replace "8" with any number [1,16]. You should hear the motor move in the device and appropriate message in the ROS log file.
```
rostopic pub /agiltron/read_fiber std_msgs/UInt8 -- 8
```

### Development Progress (TODO)
 - [X] Initial implementation
 - [ ] Make serial port a rosparam
 - [ ] Make read rate a rosparam
 - [ ] Support for status messages
 - [ ] Integration with Spectrometer libraries

### Base Commands (from Documentation)
```
Command/Echo  /Comments

CMD:   0x01 0x12 0x00 x(HEX)  /Switch to port, x: 0, 1, 2, 3.
Echo:  0x01 0x12 a(HEX) b(HEX)/Success, a: Target Step High Byte, b: Target Step Low Byte.
       0x01 0x12 0xFF 0xFF    /Failed
CMD:   0x01 0x20 0x00 0x00    /Homing.
Echo:  0x01 0x20 0x00 0x00    /Success.
```