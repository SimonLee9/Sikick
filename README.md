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
X1: 351763.901
Y1: 4025749.144
351763.901, 4025749.144

X2: 351783.3108
Y2: 4025748.981
351783.3108, 4025748.981

X3: 351782.6192
Y3: 4025739.505
351782.6192, 4025739.505

### USB
sh usb [sudo chmod 777/dev/tty*]

### steering_straight

rosrun steering_straight teleop_key.py

rosrun steering_striaght move.py

## Test

### 1.Tracking

### 1-1.position tracking
sh vins-ekf 

roslaunch visual_ins vins-ekf.launch # integrate_EKF.py

### 1-2.target_path [이동할 경로]
sh target_path

roslaunch target_path target_path.launch

### 1-3.path_tracking
sh path_tracking

roslaunch path_tracking path_tracking.launch # purepursuit.py

### 1-4 gotoposition
sh gotoposition

roslaunch path_tracking path_tracking.launch

### 1-3_2.target_positions ??
roslaunch goto_targetpositions target_positions.launch
roslaunch goto_targetpositions goto_targetpositions.launch
