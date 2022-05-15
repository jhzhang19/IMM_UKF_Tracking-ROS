#IMM_UKF_Tracking-ROS
### Introduction

This project combine IMM(CV,CTRV,CTRA) and UKF to achieve a fast object tracking in ROS program, define the object message.

### Requirements

1. ROS

2. pcl

3. boost

4. Eigen

### Usage
1. Download the package
2. Prepare the data KITTI MOT data 
![2022-05-15 10-28-20屏幕截图](https://user-images.githubusercontent.com/55379665/168454605-ed51f608-b0cf-4a3e-9c50-50f2c2c79252.png)

3. edit the data path in launch file
4. catkin_make
5. roslaunch detect_track_node.launch


### Result view
![GIF3](https://user-images.githubusercontent.com/55379665/168454565-896ccada-8a8a-4f42-9a3e-722dd2a68a8a.gif)


### Reference

[1] https://github.com/wangx1996/Multi-Object-Tracking.git

[2] [JPDA](https://github.com/apennisi/jpdaf_tracking)

[3] [UKF](https://github.com/mithi/fusion-ukf)


