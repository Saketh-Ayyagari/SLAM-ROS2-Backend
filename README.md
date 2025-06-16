# About this project
Semester 2 High School Senior Project where I developed and utilized ROS2 packages for controlling an Andino differential drive robot for performing Simultaneous Localization and Mapping.

## Physical hardware required
Will update w/ links soon

## How can I use this ROS2 package?
1. Make sure you download Ubuntu 22.04 or 24.04 onto your Raspberry Pi 4/5 (respectively).
2. Download ROS2 [Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) if you are running Ubuntu 22.04, or ROS2 [Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) if you are running Ubuntu 24.04.
3. Clone this repository using the following command
``` bash
git clone https://github.com/Saketh-Ayyagari/SLAM-ROS2-Backend
```
4. Use the following command to go into the src/ directory of the main repository
```bash
cd SLAM-ROS2-Backend/src/
```
5. Clone the following two github repositories. These are the drivers for the Adafruit BNO055 IMU and the LD20 360 degree LiDAR
```bash
git clone https://github.com/the-hive-lab/bno055_driver
```
```bash
git clone https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2
```

## NOTE: Will soon put up a _requirements.txt_ for required packages

