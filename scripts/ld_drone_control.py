#!/usr/bin/env python3
# Drone control script for testing landing detector algorithm in Gazebo

import rospy
import mavros
from std_msgs.msg import Header, Bool
from geometry_msgs.msg import PoseStamped, Point32
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from math import sqrt


class Drone():
    def __init__(self):
        self.hz = 25
        self.rate = rospy.Rate(self.hz)
        self.state = State()
        self.received_position = False
        self.received_data_from_landing_detector = False
        self.landing_position = None
        self.safe_to_land = False
        self.allowed_to_land = False
        self.home_position = PoseStamped()
        self.current_position = PoseStamped()
        self.altitude = 5   # [m]
        self.altitude_inc = 0.5 # amount to increase altitude when first objects are detected [m]
        self.land_velocity = 1  # [m/s]
        self.land_velocity_slow = 0.5   # velocity to decrease to when first objects are detected [m/s]
        # Setup stuff
        self.__init_publishers()
        self.__init_subscribers()
        self.__init_services()
        self.setup_drone()

    def __init_publishers(self):
        # Setup publishers
        self.target_pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=1)
        self.toggle_detection_pub = rospy.Publisher("/landing_detector/toggle_detection", Bool, queue_size=1)

    def __init_subscribers(self):
        # Setup subscribers
        self.state_sub = rospy.Subscriber('/mavros/state', State, self.__state_cb)
        self.pos_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.__position_cb)
        self.landing_detector_sub = rospy.Subscriber('landing_detector/position', Point32, self.__landing_detector_cb)
        self.allowed_to_land_sub = rospy.Subscriber('landing_detector/allow_land', Bool, self.__allow_land_cb)

    def __init_services(self):
        # Setup services
        rospy.wait_for_service("/mavros/cmd/arming")
        rospy.loginfo("/mavros/cmd/arming service ready!")
        rospy.wait_for_service("mavros/set_mode")
        rospy.loginfo("/mavros/set_mode service ready!")
        rospy.wait_for_service("mavros/cmd/land")
        rospy.loginfo("/mavros/cmd/land service ready!")
        self.arming_client = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self.set_mode_client = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        self.land_client = rospy.ServiceProxy("/mavros/cmd/land", CommandTOL)
        self.takeoff_client = rospy.ServiceProxy("/mavros/cmd/takeoff", CommandTOL)

    # Callbacks
    def __state_cb(self, state):
        self.state = state

    def __position_cb(self, position):
        self.current_position = position
        # Save home position
        if(not self.received_position):
            self.home_position = position
        self.received_position = True

    def __landing_detector_cb(self, pos_msg):
        # Save position detected as current position + offset
        # Only do so if offset is not 0, else we need to descend and not set the altitude to self.altitude
        if not self.safe_to_land:
            self.landing_position = self.current_position
            self.landing_position.pose.position.x = self.current_position.pose.position.x + pos_msg.x
            self.landing_position.pose.position.y = self.current_position.pose.position.y + pos_msg.y
        # If we recieve data, increase altitude by 0.5m and decrease landing velocity (but only once)
        # z-value from msg is only used to indicate wether the landing detector is running the detection
        # (meaning that it recieves point cloud values)
        if pos_msg.z != 0 and not self.received_data_from_landing_detector:
            rospy.loginfo('Data recieved, increasing altitude by {}m and decreasing landing velocity to {}m/s'.format(self.altitude_inc, self.land_velocity_slow))
            self.land_velocity = 0.5
            self.landing_position.pose.position.z = self.current_position.pose.position.z + 0.5
            self.received_data_from_landing_detector = True
        # If no obstacles underneath, descend slowly
        if pos_msg.x == 0 and pos_msg.y == 0:
            self.safe_to_land = True
        else:
            self.safe_to_land = False

    def __allow_land_cb(self, msg):
        # True if we are allowed to land
        self.allowed_to_land = msg.data

    def setup_drone(self):
        rospy.loginfo("Waiting for FCU connection...")
        while not self.state.connected:
            self.rate.sleep()
        rospy.loginfo("FCU connected")

        rospy.loginfo("Waiting on position...")
        while not self.received_position:
            self.rate.sleep()
        rospy.loginfo("Position received")

        self.take_off_position = PoseStamped()
        self.take_off_position.pose.position.x = self.home_position.pose.position.x
        self.take_off_position.pose.position.y = self.home_position.pose.position.y
        self.take_off_position.pose.position.z = self.altitude

        # Send a few takeoff commands before starting
        for i in range(20):
            self.target_pos_pub.publish(self.take_off_position)
            self.rate.sleep()

        rospy.loginfo("Waiting for change mode to offboard & arming rotorcraft...")
        while not self.state.mode == "OFFBOARD":
            #if not self.state.mode == "OFFBOARD":
            self.set_mode_client(base_mode=0, custom_mode="OFFBOARD")
            self.target_pos_pub.publish(self.take_off_position)
            self.rate.sleep()
        while not self.state.armed:
            self.arming_client(True)
            self.target_pos_pub.publish(self.take_off_position)
            self.rate.sleep()
        rospy.loginfo("Offboard mode enabled and rotorcraft armed")

        # Reached takeoff position
        header = Header()
        dist_to_takeoff_pos = self.altitude
        while(0.5 < dist_to_takeoff_pos):
            x = self.take_off_position.pose.position.x - self.current_position.pose.position.x
            y = self.take_off_position.pose.position.y - self.current_position.pose.position.y
            z = self.take_off_position.pose.position.z - self.current_position.pose.position.z
            dist_to_takeoff_pos = sqrt(x*x + y*y + z*z)
            header.stamp = rospy.Time.now()
            self.take_off_position.header = header
            self.target_pos_pub.publish(self.take_off_position)
            self.rate.sleep()

    def distance_to_target(self, target_position):
        x = target_position.pose.position.x - self.current_position.pose.position.x
        y = target_position.pose.position.y - self.current_position.pose.position.y
        return sqrt(x*x + y*y)

    def make_waypoints(self):
        waypoints = []
        tp1 = PoseStamped()
        # On top of middle cylinder
        #tp1.pose.position.x = 4.01
        #tp1.pose.position.y = 24.87
        #tp1.pose.position.z = self.altitude
        tp1.pose.position.x = 0.0
        tp1.pose.position.y = 0.0
        tp1.pose.position.z = self.altitude
        #tp1.pose.orientation.x = 0
        #tp1.pose.orientation.y = 0
        #tp1.pose.orientation.z = 0.383
        #tp1.pose.orientation.w = 0.924
        waypoints.append(tp1)
        return waypoints

    def shutdown_drone(self):
        rospy.loginfo("Landing..")
        while not self.state.mode == "AUTO.LAND":
            self.land_client()
        rospy.signal_shutdown("Done")

    def fly_route(self):
        rospy.loginfo("Flying Route")
        waypoint = 0
        header = Header()
        waypoints = self.make_waypoints()
        while (waypoint < len(waypoints) and self.state.armed):
            target_position = waypoints[waypoint]
            header.stamp = rospy.Time.now()
            target_position.header = header
            self.target_pos_pub.publish(target_position)
            self.rate.sleep()
            if(self.distance_to_target(target_position) < 0.50):
                rospy.loginfo('Next waypoint')
                waypoint += 1
        # Hold last position until landing detector is activated
        # and landing position is recieved
        rospy.loginfo('Holding position, activating landing detector')
        #while not self.recievedLandingPosition and self.landing_position is None:
        while self.landing_position is None:
            self.toggle_detection_pub.publish(Bool(data=True))
            self.target_pos_pub.publish(waypoints[-1])
            self.rate.sleep()
        # Landing position recieved, initate landing
        rospy.loginfo('Landing position detected, initating landing..')
        # Go to landing position and land slowly
        # when .1m from ground, initiate auto land as it is assumed that no obstacles
        # will show up underneath the drone at this point
        while not self.allowed_to_land:
            # While not allowed to land, check if no obstacles underneath
            if self.safe_to_land and self.landing_position.pose.position.z > 0.1:
                # Descend slowly, keep checking for obstacles
                self.landing_position.pose.position.z -= self.land_velocity/self.hz
            self.target_pos_pub.publish(self.landing_position)
            self.rate.sleep()
        # Should have landed if we autodetect and disarm
        rospy.loginfo('Landing done')
        self.shutdown_drone()



if __name__ == '__main__':
    rospy.init_node('ld_drone_control', anonymous=True)
    drone = Drone()
    drone.fly_route()
    #drone.shutdown_drone()
    rospy.spin()
