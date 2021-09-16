#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>


/* Defines */
// Modes
#define MODE_OFFBOARD   "OFFBOARD"
#define MODE_LAND       "AUTO.LAND"
#define MODE_LOITER     "AUTO.LOITER"
// Other
#define RATE            20              // Must be greater than 2 Hz


/* Callbacks */
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}
geometry_msgs::PoseStamped current_pose;
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    current_pose = *msg;
}


/* Main */
int main(int argc, char **argv) {
    // Initialize node
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    // Initialize subscribers, publishers and service clients
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Subscriber pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, pose_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    // Setpoint publishing rate
    ros::Rate rate(RATE);

    // Wait for FCU connection
    while (ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }

    // Send a few setpoints before starting
    for (uint8_t i = 0; ros::ok() && i < 100; ++i){
        local_pos_pub.publish(current_pose);
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
    const ros::Time ts_start = ros::Time::now();
    geometry_msgs::PoseStamped new_pose;
    geometry_msgs::Point* new_pos;

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
            local_pos_pub.publish(current_pose);
            if (current_state.mode != MODE_OFFBOARD && (ros::Time::now() - last_request > ros::Duration(5.0))) {
                ROS_INFO("Enabling offboard");
                if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
                    state = WAIT_FOR_OFFBOARD;
                last_request = ros::Time::now();
            }
            break;

        case WAIT_FOR_OFFBOARD:
            if (current_state.mode == MODE_OFFBOARD) {
                ROS_INFO(">> Offboard enabled");
                state = ARM;
            }
            break;

        case ARM:
            local_pos_pub.publish(current_pose);
            if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))) {
                ROS_INFO("Arming");
                if (arming_client.call(arm_cmd) && arm_cmd.response.success)
                    state = WAIT_FOR_ARMED;
                last_request = ros::Time::now();
            }
            break;

        case WAIT_FOR_ARMED:
            if (current_state.armed) {
                ROS_INFO(">> Vehicle armed, executing");
                state = EXECUTE;
            }
            break;

        case EXECUTE:
            if (current_state.armed && (ros::Time::now() - ts_start < ros::Duration(30.0))) {
                new_pos = &new_pose.pose.position;
                new_pos->z = 2;

                ROS_INFO("Moving to xyz (%.2f, %.2f, %.2f)", new_pos->x, new_pos->y, new_pos->z);
                local_pos_pub.publish(new_pose);
            } else if (set_mode_client.call(land_set_mode) && land_set_mode.response.mode_sent) {
                ROS_INFO("Enabling %s", MODE_LAND);
                state = WAIT_FOR_LAND;
            }
            break;

        case WAIT_FOR_LAND:
            if (current_state.mode == MODE_LAND) {
                ROS_INFO(">> AUTO.LAND enabled, waiting until disarmed");
                state = LANDING;
            }
            break;

        case LANDING:
            if (!current_state.armed) {
                ROS_INFO("Disarmed");
                if (set_mode_client.call(loiter_set_mode) && loiter_set_mode.response.mode_sent) {
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
        ROS_DEBUG("mode: %s, armed: %u", current_state.mode.c_str(), current_state.armed);

        if (state == FINISH)
            break;
    }

    ROS_INFO(">> Done");

    return 0;
}
