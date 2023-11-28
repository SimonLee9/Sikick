# Sikick

## Function

### Camera

sh cam

sh cam-obj

sh yolo-custom


### VSLAM

roslaunch zed_rtabmap_example zed_rtabmap.launch


### 2D Lidar

sh rplidar


### GPS

sh gps [roslaunch ublox_gps ublox_device.launch]

sh gps-rtk [roslaunch ntrip_ros ntrip_ros.launch]

sh gps-utm [roslaunch utm_lla coordinate_convertion.launch]


### GPS+Odometry
sh vins-ekf [roslaunch visual_ins vins-ekf.launch]


### Target position
sh target [roslaunch goto_targetpositions traget_positions.launch]

### USB
sh usb [sudo chmod 777/dev/tty*]

### steering_straight

rosrun steering_straight teleop_key.py

rosrun steering_striaght move.py
