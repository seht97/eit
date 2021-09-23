#!/usr/bin/env python3

import rospy
import mavros
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from math import sqrt


class Drone():
    def __init__(self):
        self.hz = 25
        self.rate = rospy.Rate(self.hz)
        self.state = State()
        self.receivedPosition = False
        self.home_position = PoseStamped()
        self.current_position = PoseStamped()
        self.altitude = 2

        self.setup_topics()
        self.setup()

    def setup_topics(self):
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

        ## Subscribers:
        self.state_sub = rospy.Subscriber('/mavros/state', State, self.state_cb)
        self.pos_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.position_cb)

        ## Publishers:
        self.target_pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=1)

    ## Callbacks
    def state_cb(self, state):
        self.state = state

    def position_cb(self, position):
        self.current_position = position
        if(not self.receivedPosition):
            self.home_position = position
        self.receivedPosition = True

    def setup(self):
        print("Waiting for FCU connection...")
        while not self.state.connected:
            self.rate.sleep()
        print("FCU connected")

        print("Waiting on position...")
        while not self.receivedPosition:
            self.rate.sleep()
        print("Position received")

        self.takeOffPosition = PoseStamped()
        self.takeOffPosition.pose.position.x = self.home_position.pose.position.x
        self.takeOffPosition.pose.position.y = self.home_position.pose.position.y
        self.takeOffPosition.pose.position.z = self.altitude

        # send a few takeoff commands before starting
        for i in range(20):
            self.target_pos_pub.publish(self.takeOffPosition)
            self.rate.sleep()

        print("Waiting for change mode to offboard & arming rotorcraft...")
        while not self.state.mode == "OFFBOARD" and not self.state.armed:
            if not self.state.mode == "OFFBOARD":
                self.set_mode_client(base_mode=0, custom_mode="OFFBOARD")
                print("OFFBOARD enalbe")

            if not self.state.armed:
                self.arming_client(True)
                print("Rotorcraft armed")

            # reached takeoff position
            header = Header()
            dist_to_takeoff_pos = 2
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
        #z = targetPosition.pose.position.z - self.current_position.pose.position.z
        return sqrt(x*x + y*y) #+ z*z)

    def makeWaypoints(self):
        waypoints = []
        targetPosition1 = PoseStamped()
        targetPosition1.pose.position.x = 2
        targetPosition1.pose.position.y = 2
        targetPosition1.pose.position.z = self.altitude
        targetPosition1.pose.orientation.x = 0
        targetPosition1.pose.orientation.y = 0
        targetPosition1.pose.orientation.z = 0.383
        targetPosition1.pose.orientation.w = 0.924
        waypoints.append(targetPosition1)
        targetPosition2 = PoseStamped()
        targetPosition2.pose.position.x = 2
        targetPosition2.pose.position.y = -2
        targetPosition2.pose.position.z = self.altitude
        targetPosition2.pose.orientation.x = 0
        targetPosition2.pose.orientation.y = 0
        targetPosition2.pose.orientation.z = -0.383
        targetPosition2.pose.orientation.w = 0.924
        waypoints.append(targetPosition2)
        targetPosition3 = PoseStamped()
        targetPosition3.pose.position.x = -2
        targetPosition3.pose.position.y = -2
        targetPosition3.pose.position.z = self.altitude
        targetPosition3.pose.orientation.x = 0
        targetPosition3.pose.orientation.y = 0
        targetPosition3.pose.orientation.z = -0.924
        targetPosition3.pose.orientation.w = 0.383
        waypoints.append(targetPosition3)
        targetPosition4 = PoseStamped()
        targetPosition4.pose.position.x = -2
        targetPosition4.pose.position.y = 2
        targetPosition4.pose.position.z = self.altitude
        targetPosition4.pose.orientation.x = 0
        targetPosition4.pose.orientation.y = 0
        targetPosition4.pose.orientation.z = 0.924
        targetPosition4.pose.orientation.w = 0.383
        waypoints.append(targetPosition4)
        targetPosition5 = PoseStamped()
        targetPosition5.pose.position.x = 0
        targetPosition5.pose.position.y = 0
        targetPosition5.pose.position.z = self.altitude
        targetPosition5.pose.orientation.x = 0
        targetPosition5.pose.orientation.y = 0
        targetPosition5.pose.orientation.z = 0
        targetPosition5.pose.orientation.w = 0
        waypoints.append(targetPosition5)
        targetPosition6 = PoseStamped()
        targetPosition6.pose.position.x = 0
        targetPosition6.pose.position.y = 0
        targetPosition6.pose.position.z = -1
        waypoints.append(targetPosition5)
        return waypoints

    def shutdownDrone(self):
        while not self.state.mode == "AUTO.LAND":
            self.land_client()
            print("Landing..")
        rospy.signal_shutdown("Done")

    def flyRoute(self):
        print("Flying Route")
        waypoint = 0
        header = Header()
        waypoints = self.makeWaypoints()
        while (waypoint < len(waypoints) and self.state.armed):
            targetPosition = waypoints[waypoint]
            header.stamp = rospy.Time.now()
            targetPosition.header = header
            self.target_pos_pub.publish(targetPosition)
            self.rate.sleep()
            if(self.distanceToTarget(targetPosition) < 0.20):
                print("next waypoint:")
                #print(targetPosition)
                waypoint += 1


if __name__ == '__main__':
    rospy.init_node('drone_control', anonymous=True)
    drone = Drone()
    drone.flyRoute()
    drone.shutdownDrone()
    rospy.spin()
