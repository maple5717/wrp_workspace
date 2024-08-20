# wrp_workspace #
Navigation support of nus-wheelchair. 

## System Requirements ##
ROS2 installed (tested with humble and foxy)

ROS1 installed (tested with noetic)

ros1_bridge installed (see [this link](https://github.com/ros2/ros1_bridge))

## Installation ##
clone this repository 
```
git clone --recursive https://github.com/maple5717/wrp_workspace.git
cd wrp_workspace 
```
### Nav2 & cartographer (ROS2) ###
<!-- Please refer to the [installation guide](https://docs.nav2.org/getting_started/index.html) -->
```
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
sudo apt install ros-humble-cartographer
sudo apt install ros-humble-cartographer-ros
```
install Cartographer using [this link](https://ros2-industrial-workshop.readthedocs.io/en/latest/_source/navigation/ROS2-Cartographer.html)


### RealSense (ROS2) ###
Please make sure the [librealsense](https://github.com/IntelRealSense/librealsense) is installed and the package ```realsense-ros``` is cloned to ```src/camera```
...

### gmapping (ROS1) ###
```
sudo apt install ros-noetic-gmapping
```

### VIO (ROS1) ###
In this project, we use the HKUST VINS-Mono VIO. Please follow the guidance in this [forked repository](https://github.com/maple5717/VINS-Mono) to build up the dependencies. 

<!-- ### ORB-SLAM3 (ROS2) ###
 Install epoxy
```
sudo apt-get install libepoxy-dev
```
Clone and build [Pangolin](https://github.com/stevenlovegrove/Pangolin)


Please make sure the forked repository ```orbslam3_ros2```  [[link](github.com/maple5717/orbslam3_ros2)] is cloned to ```src/slam```

Install g20
```
sudo apt-get install ros-humble-libg2o
```
Install orbslam following the instructions from the original [ORB_SLAM3_ROS2 repository](github.com/zang09/ORB_SLAM3_ROS2). **Do not forget to correctly set ORB_SLAM3_ROOT_DIR** -->



Finally, build the ROS2 project 
```
colcon build --symlink-install
source install/setup.bash # optional
```

Build the ROS1 project 
```
cd ros1_ws
catkin build
source devel/setup.bash # optional
```


## Usage ##
### Camera intrinsics setup ###
<!-- Please set up the camera intrinsics in ```src/slam/wheelchair_slam/config/RealSense_D435i.yaml``` -->
Set up the intrinsics file in ```ros1_ws/src/VINS-Mono/config/realsense```


### ROS2 nodes ###
In one terminal, launch realsense
<!-- ```
ros2 launch realsense2_camera rs_launch.py   unite_imu_method:=2 align_depth.enable:=true enable_accel:=true enable_gyro:=true 
``` -->
```
ros2 launch wheelchair_slam realsense_start.launch.py
```
In another terminal, launch the slam node
```
ros2 launch wheelchair_slam create_map.launch.py
```
<!-- ```
ros2 run orbslam3 rgbd src/slam/orbslam3_ros2/vocabulary/ORBvoc.txt src/slam/orbslam3_ros2/config/rgb-d/RealSense_D435i.yaml 
``` -->

### ROS1 nodes ###
In one terminal, start the ros1 bridge 
```
ros2 run ros1_bridge dynamic_bridge 
```

THen go to the ros1_ws
```
cd ros1_ws
```
Open another two terminals, run the VIO and gmapping node: 
```
roslaunch vins_estimator realsense_color.launch # make sure the realsense configuration file is set up correctly before running thie node! 
roslaunch mapping gmapping.launch
```

## Potential Problems ##
If you meet the QOS problem when subscribing to the imu data, please refer to the solution in [this link](https://github.com/IntelRealSense/realsense-ros/issues/3033#issuecomment-1983139591)







# Notes #
stl scale: 1/ 25.4 / 8
add moveit_config.move_group_capabilities = {"capabilities": "move_group/ExecuteTaskSolutionCapability"} to config files
todo: 
1. try to run mapping and filtering in ros1
