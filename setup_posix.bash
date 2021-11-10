#!/bin/bash
echo "Cleaning up.."

if [ $1 = "M" ]
then
	: #Argument was given
	echo "Running Magnus paths"
	find ~/EiT/PX4-Autopilot/ -name "*_aed*" -delete
	echo "Symlinking.."
	ln -s /home/$USER/EiT/catkin_ws/src/eit/init.d-posix/* /home/$USER/EiT/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/
	ln -s /home/$USER/EiT/catkin_ws/src/eit/models/* /home/$USER/EiT/PX4-Autopilot/Tools/sitl_gazebo/models/
elif [ $1 = "S" ]
then
	echo "Running Snørens paths"
	find $PX4DIR -name "*_aed*" -delete
	echo "Symlinking.."
	ln -s $EITWSDIR/src/eit/init.d-posix/* $PX4DIR/ROMFS/px4fmu_common/init.d-posix/airframes/
	ln -s $EITWSDIR/src/eit/models/* $PX4DIR/Tools/sitl_gazebo/models/
else
	: # Argument was not given
	find ~/PX4-Autopilot/ -name "*_aed*" -delete
	echo "Symlinking.."
	ln -s /home/$USER/eit_ws/src/eit/init.d-posix/* /home/$USER/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/
	ln -s /home/$USER/eit_ws/src/eit/models/* /home/$USER/PX4-Autopilot/Tools/sitl_gazebo/models/
fi
