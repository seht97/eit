#include <eit/offboard_node.h>


namespace {
double eucDist(const geometry_msgs::Point &p0, const geometry_msgs::Point &p1) {
    return sqrt(pow(p0.x - p1.x, 2) + pow(p0.y - p1.y, 2) + pow(p0.z - p1.z, 2));
}
}

/* Constructors */
OffboardNode::OffboardNode(ros::NodeHandle nh)
    : _nh{nh},
      _stateSub{_nh.subscribe<mavros_msgs::State>("mavros/state", 10, &OffboardNode::_stateCb, this)},
      _posSub{_nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, &OffboardNode::_poseCb, this)},
      _localPosPub{_nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10)},
      _armingClient{_nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming")},
      _setModeClient{_nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode")} {}


/* Functions */
void OffboardNode::run() {
    // Set options
    ros::Rate rate(20);
    const double dist2point_thresh = 0.1;

    // Wait for FCU connection
    while (ros::ok() && !_currentState.connected) {
        ros::spinOnce();
        rate.sleep();
    }

    // Send a few setpoints before starting
    for (uint8_t i = 0; ros::ok() && i < 100; ++i){
        _localPosPub.publish(_currentPose);
        ros::spinOnce();
        rate.sleep();
    }

    // Custom modes (http://wiki.ros.org/mavros/CustomModes#PX4_native_flight_stack)
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = MODE_OFFBOARD;
    mavros_msgs::SetMode land_set_mode;
    land_set_mode.request.custom_mode = MODE_LAND;
    mavros_msgs::SetMode loiter_set_mode;
    loiter_set_mode.request.custom_mode = MODE_LOITER;

    // ARM command
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    // Variable definitions
    ros::Time last_request = ros::Time::now();
    const geometry_msgs::PoseStamped orig_pose = _currentPose;
    geometry_msgs::PoseStamped new_pose;
    geometry_msgs::Point& new_pos = new_pose.pose.position;

    // Waypoints setup
    uint8_t round_no = 0;
    uint8_t num_rounds = 2;
    uint8_t wp_idx = 0;
    std::vector<geometry_msgs::Point> waypoints;
    auto add_waypoint = [&waypoints, &orig_pose](const int8_t &x, const int8_t &y, const int8_t &z, const bool& relative = false){
        waypoints.emplace_back(relative ? orig_pose.pose.position : geometry_msgs::Point());
        geometry_msgs::Point& wp = waypoints.back();
        wp.x += x;
        wp.y += y;
        wp.z += z;
    };
    add_waypoint(0, 0, 2);
    add_waypoint(1, 0, 2);
    add_waypoint(1, 1, 2);
    add_waypoint(0, 1, 2);

    // States
    enum STATE {
        EN_OFFBOARD,
        WAIT_FOR_OFFBOARD,
        ARM,
        WAIT_FOR_ARMED,
        EXECUTE,
        WAIT_FOR_LAND,
        LANDING,
        FINISH
    };
    STATE state = EN_OFFBOARD;

    // State transitions
    ROS_INFO(">> Starting the offb_node");
    while (ros::ok()) {
        switch (state) {
        case EN_OFFBOARD:
            _localPosPub.publish(_currentPose);
            if (_currentState.mode != MODE_OFFBOARD && (ros::Time::now() - last_request > ros::Duration(5.0))) {
                ROS_INFO("Waiting for offboard");
//                if (_setModeClient.call(offb_set_mode) && offb_set_mode.response.mode_sent)
                state = WAIT_FOR_OFFBOARD;
                last_request = ros::Time::now();
            }
            break;

        case WAIT_FOR_OFFBOARD:
            if (_currentState.mode == MODE_OFFBOARD) {
                ROS_INFO(">> Offboard enabled");
                state = ARM;
            }
            break;

        case ARM:
            _localPosPub.publish(_currentPose);
            if (!_currentState.armed && (ros::Time::now() - last_request > ros::Duration(5.0))) {
                ROS_INFO("Arming");
                if (_armingClient.call(arm_cmd) && arm_cmd.response.success)
                    state = WAIT_FOR_ARMED;
                last_request = ros::Time::now();
            }
            break;

        case WAIT_FOR_ARMED:
            if (_currentState.armed) {
                ROS_INFO(">> Vehicle armed, executing");
                state = EXECUTE;
            }
            break;

        case EXECUTE:
            if (_currentState.armed && round_no < num_rounds) {
                if (eucDist(_currentPose.pose.position, new_pos) < dist2point_thresh) {
                    ROS_INFO("Moving to xyz (%.2f, %.2f, %.2f)", new_pos.x, new_pos.y, new_pos.z);
                    wp_idx = (wp_idx + 1) % waypoints.size();
                    if (wp_idx == 0)
                        ++round_no;
                } else
                    std::cout << "Distance to setpoint: " << eucDist(_currentPose.pose.position, new_pos) << '\r' << std::flush;
                new_pos = waypoints[wp_idx];

                _localPosPub.publish(new_pose);
            } else if (_setModeClient.call(land_set_mode) && land_set_mode.response.mode_sent) {
                ROS_INFO("Enabling %s", MODE_LAND);
                state = WAIT_FOR_LAND;
            }
            break;

        case WAIT_FOR_LAND:
            if (_currentState.mode == MODE_LAND) {
                ROS_INFO(">> AUTO.LAND enabled, waiting until disarmed");
                state = LANDING;
            }
            break;

        case LANDING:
            if (!_currentState.armed) {
                ROS_INFO("Disarmed");
                if (_setModeClient.call(loiter_set_mode) && loiter_set_mode.response.mode_sent) {
                    ROS_INFO("Enabling %s", MODE_LOITER);
                }
                state = FINISH;
            }
            break;

        default:
            break;
        }

        ros::spinOnce();
        rate.sleep();

        // Mode + armed mode print
        ROS_DEBUG("mode: %s, armed: %u", _currentState.mode.c_str(), _currentState.armed);

        if (state == FINISH)
            break;
    }

    ROS_INFO(">> Done");
}
