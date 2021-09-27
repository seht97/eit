#ifndef OFFB_NODE_H
#define OFFB_NODE_H

#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>

// Modes
#define MODE_OFFBOARD   "OFFBOARD"
#define MODE_LAND       "AUTO.LAND"
#define MODE_LOITER     "AUTO.LOITER"

class OffboardNode {
public:
    // Constructors
    explicit OffboardNode(ros::NodeHandle nh);

    void run();

private:
    // Callbacks
    void _stateCb(const mavros_msgs::State::ConstPtr &msg) {_currentState = *msg;}
    void _poseCb(const geometry_msgs::PoseStamped::ConstPtr &msg) {_currentPose = *msg;}

    // ROS node
    ros::NodeHandle _nh;
    ros::Subscriber _stateSub, _posSub;
    ros::Publisher _localPosPub;

    // Messages
    ros::ServiceClient _armingClient, _setModeClient;
    mavros_msgs::State _currentState;
    geometry_msgs::PoseStamped _currentPose;
};

#endif
