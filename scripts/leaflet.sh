#!/bin/bash

# If no arguments passed
if [ "$1" ]
then
  echo "Using RPi's rosmaster at $1"

  # Use RPi's rosmaster
  export ROS_MASTER_URI=http://$1:11311
  export ROS_HOSTNAME=$(hostname -I | cut -d " " -f1)
  echo ROS_MASTER_URI=$ROS_MASTER_URI
  echo ROS_HOSTNAME=$ROS_HOSTNAME
fi

firefox $(rospack find eit)/leaflet.html
roslaunch rosbridge_server rosbridge_websocket.launch port:=9090 --wait

exit 0

# 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