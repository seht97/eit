# Repo for Experts in Dream Team

## Instructions for simulating AED UAV in Gazebo
1. Ensure that ROS, PX4 and eit_playground have been installed
2. Make sure you are in `uav-aed-gazebo` branch
3. Run `source ~/eit_ws/src/eit/setup_posix.bash`
4. Run `beit-ws`
5. Launch simulation using `roslaunch eit posix.launch vehicle:=sdu_aed`
6. To fix rviz being dumb run `rosrun tf static_transform_publisher 0 0 0 0 0 0 1 map camera_link 10` in a new terminal 

## Install PCL library (system wide)
```shell script
sudo add-apt-repository ppa:sweptlaser/python3-pcl
sudo apt update
sudo apt install python3-pcl
```
To view pcd files use:
```shell script
pcl_viewer <filename>
```