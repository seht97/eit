#!/usr/bin/env python3

import rospy
import mavros
from utm import utmconv
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from geographic_msgs.msg import GeoPoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from math import sqrt
from sensor_msgs.msg import NavSatFix


class Drone():
    def __init__(self):
        self.hz = 25
        self.rate = rospy.Rate(self.hz)
        self.state = State()
        self.receivedPosition = False
        self.home_position = GeoPoseStamped()
        self.current_position = PoseStamped()
        self.current_position_geo = GeoPoseStamped()
        #self.home_position = NavSatFix()
        #self.current_position = NavSatFix()
        self.altitude = 30
        self.targetGPS = NavSatFix()
        self.receivedGPS = False
        self.uc = utmconv()
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
        self.pos_sub = rospy.Subscriber("/mavros/global_position/raw/fix", NavSatFix, self.gps_pos_cb)
        self.pos_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.position_cb)
        self.target_pos_sub = rospy.Subscriber("/mytopic/GPS_target", NavSatFix, self.target_cb)

        ## Publishers:
        self.target_pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=1)
        #self.target_pos_pub = rospy.Publisher("/mavros/setpoint_position/global", GeoPoseStamped, queue_size=1)

    ## Callbacks
    def state_cb(self, state):
        self.state = state

    def gps_pos_cb(self, position):
        self.current_position_geo = position
        if(not self.receivedPosition):
            self.home_position = position
            (self.hemisphere, self.zone, _, self.home_e, self.home_n) = self.uc.geodetic_to_utm (self.home_position.latitude,self.home_position.longitude)
            self.home_e = self.home_e - self.current_position.pose.position.x
            self.home_n = self.home_n - self.current_position.pose.position.y
        self.receivedPosition = True

    def position_cb(self, position):
        self.current_position = position

    def target_cb(self, data):
        self.targetGPS = data
        print("longitude %f" % self.targetGPS.longitude)
        self.receivedGPS = True


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
        self.takeOffPosition.pose.position.x = 0
        self.takeOffPosition.pose.position.y= 0
        self.takeOffPosition.pose.position.z = 0

        # send a few takeoff commands before starting
        for i in range(20):
            self.target_pos_pub.publish(self.takeOffPosition)
            self.rate.sleep()

        print("Waiting for target position")
        while not self.receivedGPS and not self.valid_gps(self.set_target()):
            self.target_pos_pub.publish(self.takeOffPosition)
            self.rate.sleep()

        print("Target position received")

        #print("Waiting for change mode to offboard & arming rotorcraft...")
        #print("Enabling OFFBOARD")
        while not self.state.mode == "OFFBOARD": #FJERN DETTE I REAL LIFE
            self.rate.sleep()
        #    if not self.state.mode == "OFFBOARD":
        #        self.set_mode_client(base_mode=0, custom_mode="OFFBOARD")
        #        print("OFFBOARD enable")

        print("Rotorcraft arming")
        while not self.state.armed:
            if not self.state.armed:
                self.arming_client(True)

            # reached takeoff position
        header = Header()
        dist_to_takeoff_pos = 2
        self.takeOffPosition.pose.position.z = 30
        while(0.5 < dist_to_takeoff_pos):
            x = self.takeOffPosition.pose.position.x - self.current_position.pose.position.x
            y = self.takeOffPosition.pose.position.y - self.current_position.pose.position.y
            z = self.takeOffPosition.pose.position.z - self.current_position.pose.position.z
            dist_to_takeoff_pos = sqrt(x*x + y*y + z*z)
            header.stamp = rospy.Time.now()
            #print(self.takeOffPosition.pose.position.altitude)
            #print(self.current_position.altitude)
            self.takeOffPosition.header = header
            self.target_pos_pub.publish(self.takeOffPosition)
            self.rate.sleep()

    def distanceToTarget(self,targetPosition):
        x = targetPosition.pose.position.x - self.current_position.pose.position.x
        y = targetPosition.pose.position.y - self.current_position.pose.position.y
        z = targetPosition.pose.position.z - self.current_position.pose.position.z
        return sqrt(x*x + y*y + z*z)

    def shutdownDrone(self):
        while not self.state.mode == "AUTO.LAND":
            self.land_client()
            print("Landing..")
        rospy.signal_shutdown("Done")

    def valid_gps(self, targetPosition):
        x = targetPosition.pose.position.x
        y = targetPosition.pose.position.y
        dist = sqrt(x*x + y*y)
        if dist < 200:
            return True
        else:
            return False

    def set_target(self):
        (self.hemisphere, self.zone, _, e1, n1) = self.uc.geodetic_to_utm (self.targetGPS.latitude,self.targetGPS.longitude)
        targetPosition = PoseStamped()
        targetPosition.pose.position.x = e1-self.home_e
        targetPosition.pose.position.y = n1-self.home_n
        targetPosition.pose.position.z = self.altitude
        targetPosition.pose.orientation.x = 0
        targetPosition.pose.orientation.y = 0
        targetPosition.pose.orientation.z = 0
        targetPosition.pose.orientation.w = 0
        return targetPosition

    def flyRoute(self):
        print("Flying Route")
        waypoint = 0
        header = Header()
        while (self.state.armed):
            if self.valid_gps(self.set_target()):
                targetPosition = self.set_target()
            else:
                print("Coordinates too far away")
            header.stamp = rospy.Time.now()
            targetPosition.header = header
            self.target_pos_pub.publish(targetPosition)
            self.rate.sleep()
            #print(self.distanceToTarget(targetPosition))
            #print(targetPosition)
            if(self.distanceToTarget(targetPosition) < 0.20):
                break


if __name__ == '__main__':
    rospy.init_node('drone_control', anonymous=True)
    drone = Drone()
    drone.flyRoute()
    drone.shutdownDrone()
    rospy.spin()
