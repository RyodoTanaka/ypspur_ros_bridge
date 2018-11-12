# ypspur_ros_bridge
## About
This package provides the bridge to use ypspur with ROS.  
With this package, you can send `cmd_vel` message and get `odom` message.

## Installation
### 1. git clone
```bash
$ cd <catkin_ws>/src
$ git clone https://github.com/RyodoTanaka/ypspur_ros_bridge.git
```

### 2. rosdep
```bash
$ cd <catkin_ws>
$ rosdep install -i -y -r --from-paths src --ignore-src
```

### 3. build
```bash
$ cd <catkin_ws>
$ catkin_make
$ source devel/setup.bash
```

## Usage
### 1. start the ypspur
First, you need to launch `ypspur-coordinator`.
After connecting your PC and ypspur by mini-USB code, type follwing command.
```bash
$ cd <ypspur_ros_bridge>/script
$ sudo sh ypspur_starter.sh
```

And you should see follwing messages, if it's succeeded.
```bash
++++++++++++++++++++++++++++++++++++++++++++++++++
YamabicoProject-Spur
 Ver. 1.14.0
++++++++++++++++++++++++++++++++++++++++++++++++++
Device Information
 Port    : /dev/ttyACM0 
Warn: Baudrate setting is not supported on this device.
Applying parameters.
YP-Spur coordinator started.
Command analyzer started.
Trajectory control loop started.
```

If you couldn't get such messages, insert your USB plug again.  
After 10 seconds, try to start `ypspur-coordinator`.


### 2. launch ypspur_ros_bridge
```bash
$ roslaunch ypspur_ros_bridge ypspur_ros_bridge.launch
```

See `ypspur_ros_bridge/launch/ypspur_ros_bridge.launch` file, for more detail.  
And the parameter file is inside of `ypspur_ros_bridge/config/` directory.
