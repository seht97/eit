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
        self.receivedPosition = False
        self.recievedLandingPosition = False
        self.landing_position = None
        self.safe_to_land = False
        self.allowed_to_land = False
        self.home_position = PoseStamped()
        self.current_position = PoseStamped()
        self.altitude = 15   # [m]
        self.land_velocity = 1  # [m/s]
        # Setup stuff
        self._init_publishers()
        self._init_subscribers()
        self._init_services()
        self.setup_drone()

    def _init_publishers(self):
        # Setup publishers
        self.target_pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=1)
        self.toggle_detection_pub = rospy.Publisher("/landing_detector/toggle_detection", Bool, queue_size=1)

    def _init_subscribers(self):
        # Setup subscribers
        self.state_sub = rospy.Subscriber('/mavros/state', State, self.state_cb)
        self.pos_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.position_cb)
        self.landing_detector_sub = rospy.Subscriber('landing_detector/position', Point32, self.landing_detector_cb)
        self.allowed_to_land_sub = rospy.Subscriber('landing_detector/allow_land', Bool, self.allow_land_cb)

    def _init_services(self):
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
    def state_cb(self, state):
        self.state = state

    def position_cb(self, position):
        self.current_position = position
        # Save home position
        if(not self.receivedPosition):
            self.home_position = position
        self.receivedPosition = True

    def landing_detector_cb(self, pos_msg):
        # Save position detected as current position + offset
        # Only do so if offset is not 0, else we need to descend and not set the altitude to self.altitude
        if not self.safe_to_land:
            self.landing_position = self.current_position
            self.landing_position.pose.position.x = self.current_position.pose.position.x + pos_msg.x
            self.landing_position.pose.position.y = self.current_position.pose.position.y + pos_msg.y
        self.recievedLandingPosition = True
        if pos_msg.x == 0 and pos_msg.y == 0:
            self.safe_to_land = True
        else:
            self.safe_to_land = False

    def allow_land_cb(self, msg):
        # True if we are allowed to land
        self.allowed_to_land = msg.data
        print(self.allowed_to_land)
        if self.allowed_to_land:
            print("Working")

    def setup_drone(self):
        rospy.loginfo("Waiting for FCU connection...")
        while not self.state.connected:
            self.rate.sleep()
        rospy.loginfo("FCU connected")

        rospy.loginfo("Waiting on position...")
        while not self.receivedPosition:
            self.rate.sleep()
        rospy.loginfo("Position received")

        self.takeOffPosition = PoseStamped()
        self.takeOffPosition.pose.position.x = self.home_position.pose.position.x
        self.takeOffPosition.pose.position.y = self.home_position.pose.position.y
        self.takeOffPosition.pose.position.z = self.altitude

        # Send a few takeoff commands before starting
        for i in range(20):
            self.target_pos_pub.publish(self.takeOffPosition)
            self.rate.sleep()

        rospy.loginfo("Waiting for change mode to offboard & arming rotorcraft...")
        while not self.state.mode == "OFFBOARD" or not self.state.armed:
            if not self.state.mode == "OFFBOARD":
                self.set_mode_client(base_mode=0, custom_mode="OFFBOARD")
            if not self.state.armed:
                self.arming_client(True)
            self.target_pos_pub.publish(self.takeOffPosition)
            self.rate.sleep()
        rospy.loginfo("Offboard mode enabled and rotorcraft armed")

        # Reached takeoff position
        header = Header()
        dist_to_takeoff_pos = self.altitude
        while(0.5 < dist_to_takeoff_pos):
            x = self.takeOffPosition.pose.position.x - self.current_position.pose.position.x
            y = self.takeOffPosition.pose.position.y - self.current_position.pose.position.y
            z = self.takeOffPosition.pose.position.z - self.current_position.pose.position.z
            dist_to_takeoff_pos = sqrt(x*x + y*y + z*z)
            header.stamp = rospy.Time.now()
            self.takeOffPosition.header = header
            self.target_pos_pub.publish(self.takeOffPosition)
            self.rate.sleep()

    def distanceToTarget(self,targetPosition):
        x = targetPosition.pose.position.x - self.current_position.pose.position.x
        y = targetPosition.pose.position.y - self.current_position.pose.position.y
        return sqrt(x*x + y*y)

    def makeWaypoints(self):
        waypoints = []
        tp1 = PoseStamped()
        # On top of middle cylinder
        tp1.pose.position.x = 4.01
        tp1.pose.position.y = 24.87
        tp1.pose.position.z = self.altitude
        #tp1.pose.orientation.x = 0
        #tp1.pose.orientation.y = 0
        #tp1.pose.orientation.z = 0.383
        #tp1.pose.orientation.w = 0.924
        waypoints.append(tp1)
        return waypoints

    def shutdownDrone(self):
        rospy.loginfo("Landing..")
        while not self.state.mode == "AUTO.LAND":
            self.land_client()
        rospy.signal_shutdown("Done")

    def flyRoute(self):
        rospy.loginfo("Flying Route")
        waypoint = 0
        header = Header()
        waypoints = self.makeWaypoints()
        while (waypoint < len(waypoints) and self.state.armed):
            targetPosition = waypoints[waypoint]
            header.stamp = rospy.Time.now()
            targetPosition.header = header
            self.target_pos_pub.publish(targetPosition)
            self.rate.sleep()
            if(self.distanceToTarget(targetPosition) < 0.50):
                rospy.loginfo('Next waypoint')
                waypoint += 1
        # Hold last position
        rospy.loginfo('Holding position, activating landing detector')
        while not self.recievedLandingPosition and self.landing_position is None:
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
        self.shutdownDrone()



if __name__ == '__main__':
    rospy.init_node('ld_drone_control', anonymous=True)
    drone = Drone()
    drone.flyRoute()
    #drone.shutdownDrone()
    rospy.spin()
