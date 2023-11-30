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

### target path
sh target_path
x1: 351762.345724
y1: 4025746.35403
351762.345724, 4025746.35403

x2?: 351775.80848
y2?: 4025747.96584
351775.80848, 4025747.96584

X3: 351782.703744
y3: 4025739.72508
351782.703744, 4025739.72508

### USB
sh usb [sudo chmod 777/dev/tty*]

### steering_straight

rosrun steering_straight teleop_key.py

rosrun steering_striaght move.py

## Test

### 1.Tracking

### 1-1.position tracking
sh vins-ekf 

roslaunch visual_ins vins-ekf.launch

### 1-2.target_path [이동할 경로]
sh target_path

roslaunch target_path target_path.launch

### 1-3.target_positions ??
roslaunch goto_targetpositions target_positions.launch

### 1-4.path_tracking
sh path_tracking

roslaunch path_tracking path_tracking.launch

### 1-6 gotoposition
sh gotoposition

roslaunch goto_targetpositions goto_targetpositions.launch
