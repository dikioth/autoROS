<aside class="notice">
Ongoing project. ETA: 16 March.
</aside>

# ROScar
UU project for autonomous robot using EKF for positioning and PID for controlling.

# Components are:
 - RC car Monster Truck “Cyclone” 4WD RtR (https://www.amazon.de/MONSTERTRUCK-Cyclone-4WD-100-RTR/dp/B07F1355XY)
 - DWM10001 module (https://www.decawave.com/product/dwm1001-development-board/) used for real time positioning of the RC car.
 - Adafruit 9DOF IMU sensor (https://www.adafruit.com/product/1714) build on:
    - L3GD20H 3-axis gyroscope: ±250, ±500, or ±2000 degree-per-second scale
    - LSM303 3-axis compass: ±1.3 to ±8.1 gauss magnetic field scale
    - LSM303 3-axis accelerometer: ±2g/±4g/±8g/±16g selectable scale
 - Raspberry Pi model 3B

# The project is build on:

| Package | Description   | ROS link  | Git Link 
|---|---|---|---|
|Rosbrideg suite| For websocket server | http://wiki.ros.org/rosbridge_suite| https://github.com/RobotWebTools/rosbridge_suite.git
| localization_dwm1001  | 4 Anchors 1 Tag   | http://wiki.ros.org/localizer_dwm1001 | https://github.com/20chix/dwm1001_ros.git | 
| adafruit IMU | Reading 9DOF IMU | - |- https://github.com/rolling-robot/adafruit_imu |
|imu_filter_madgwick | Orientation estimation using IMU| http://wiki.ros.org/imu_filter_madgwick | https://github.com/ccny-ros-pkg/imu_tools.git
