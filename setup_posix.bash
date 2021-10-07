#!/bin/bash

echo "Symlinking.."
ln -s /home/$USER/eit_ws/src/eit/init.d-posix/* /home/$USER/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/
ln -s /home/$USER/eit_ws/src/eit/models/* /home/$USER/PX4-Autopilot/Tools/sitl_gazebo/models/
