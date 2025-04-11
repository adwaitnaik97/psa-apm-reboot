# HGuide Robotic Operating System (ROS) Drivers

The ROS drivers support both IMU and INS devices:

- **IMU devices** - hg_node
- **INS devices** - hg_nav_node

Learn more details about the driver architecture in the [IMU](./AN0004E_RosDriver.pptx) and [INS](AN0006E_Navigation_RosDriver.pptx) specific application notes.

The ROS Drivers are utilizing the **HGuideAPI** available in the [HGuideSDK repository](https://gitlab.com/HoneywellIMU/HgDataParser).

# How to Get?

1) **Clone repository**

To get the source code, simply clone the repository via:
```cpp
git clone https://gitlab.com/HoneywellIMU/RosDriver.git
```
Or download it as a .zip / .tar via Download Button

2) **Quick Start**

Make sure to be able to write to USB serial port without super user privileges 

```cpp
$ sudo chmod 666 /dev/ttyUSB0
```

Source catkin workspace

```cpp
$ source ./devel/setup.bash
```

Build the package

```cpp
$ catkin_make
```

Start the launch file - example for *hg_nav_node*

```cpp
$ roslaunch hg_nav_node TurtleExample.launch
```

3) **Enjoy!**

Feel free to create issues and upload any updates or custom code back to the repository. We welcome the feedback!


