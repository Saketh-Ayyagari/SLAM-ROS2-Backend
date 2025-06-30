# About this project
Semester 2 High School Senior Project where I developed and utilized ROS2 packages for controlling an Andino differential drive robot for performing Simultaneous Localization and Mapping.

## Physical hardware required
Will update w/ links soon

## How can I use this ROS2 package?
1. Make sure you download Ubuntu 22.04 or 24.04 onto your Raspberry Pi 4/5 (respectively).
2. Download ROS2 [Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) if you are running Ubuntu 22.04, or ROS2 [Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) if you are running Ubuntu 24.04.
3. Clone this repository onto BOTH your Raspberry Pi and development machine using the following command
``` 
git clone https://github.com/Saketh-Ayyagari/SLAM-ROS2-Backend
```
4. Use the following command to go into the src/ directory of the main repository
```
cd SLAM-ROS2-Backend/src/
```
5. Clone the following two github repositories. These are the drivers for the Adafruit BNO055 IMU and the LD20 360 degree LiDAR
```
git clone https://github.com/Saketh-Ayyagari/bno055_driver
```
git clone https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2
```
6. On either your Raspberry Pi or development machine, run the following command to download the [slam-toolbox](https://github.com/SteveMacenski/slam_toolbox) package
```
sudo apt-get install ros-[humble/jammy]-slam-toolbox
```
## NOTE: Will soon put up a _requirements.txt_ for required packages

