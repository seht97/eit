# Repo for Experts in Dream Team

## Install dependencies

```console
sudo apt-get install ros-$ROS_DISTRO-realsense2-camera
```

## Instructions for simulating AED UAV in Gazebo
1. Ensure that ROS, PX4 and eit_playground have been installed
2. Run `source ~/eit_ws/src/eit/setup_posix.bash`
3. Run `beit-ws`
4. Launch simulation using `roslaunch eit posix.launch` or `roslaunch eit mavros_posix_aed.launch`
5. To fix rviz being dumb, run `rosrun tf static_transform_publisher 0 0 0 0 0 0 1 map camera_link 10` in a new terminal 

## Setup of landing detector
Install PCL library (system wide)
```shell script
sudo add-apt-repository ppa:sweptlaser/python3-pcl
sudo apt update
sudo apt install python3-pcl
```
To view pcd files use:
```shell script
pcl_viewer <filename>
```
Then install ROS-numpy:
```shell script
sudo apt-get install ros-${ROS_DISTRO}-ros-numpy
```