# wrp_workspace #
Navigation support of nus-wheelchair. 

## System Requirements ##
Ubuntu 22.04 with ROS2 Humble installed

## Installation ##
clone this repository 
```
git clone https://github.com/maple5717/wrp_workspace.git
cd wrp_workspace -r
```
### Nav2 & cartographer ###
<!-- Please refer to the [installation guide](https://docs.nav2.org/getting_started/index.html) -->
```
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
sudo apt install ros-humble-cartographer
sudo apt install ros-humble-cartographer-ros
```
install Cartographer using [this link](https://ros2-industrial-workshop.readthedocs.io/en/latest/_source/navigation/ROS2-Cartographer.html)


### RealSense ###
Please make sure the package ```realsense-ros``` is cloned to ```src/camera```
...

### ORB-SLAM3 ###
Please make sure the forked repository ```orbslam3_ros2```  [[link](github.com/maple5717/orbslam3_ros2)] is cloned to ```src/slam```

Install g20
```
sudo apt-get install ros-humble-libg2o
```
Install orbslam following the instructions from the original [ORB_SLAM3_ROS2 repository](github.com/zang09/ORB_SLAM3_ROS2). **Do not forget to correctly set ORB_SLAM3_ROOT_DIR**

Finally, build the project 
```
colcon build --symlink-install
source install/setup.bash
```


## Usage ##
### Camera intrinsics setup ###
Please set up the camera intrinsics in ```src/slam/wheelchair_slam/config/RealSense_D435i.yaml```

In one terminal, launch realsense
```
ros2 launch realsense2_camera rs_launch.py   unite_imu_method:=2 align_depth.enable:=true enable_accel:=true enable_gyro:=true 
```
In another terminal, launch orb-slam3
```
ros2 launch wheelchair_slam orbslam_start.launch.py 
```

If you meet the QOS problem when subscribing to the imu data, please refer to the solution in [this link](https://github.com/IntelRealSense/realsense-ros/issues/3033#issuecomment-1983139591)



```
ros2 run orbslam3 rgbd src/slam/orbslam3_ros2/vocabulary/ORBvoc.txt src/slam/orbslam3_ros2/config/rgb-d/RealSense_D435i.yaml 
```




# Notes #
stl scale: 1/ 25.4 / 8
add moveit_config.move_group_capabilities = {"capabilities": "move_group/ExecuteTaskSolutionCapability"} to config files
todo: 
1. load urdf and robot_state_publisher
2. check tracked pose
3. wheelchair control
