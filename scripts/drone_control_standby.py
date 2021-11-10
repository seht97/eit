#!/usr/bin/env python3

import rospy
import mavros
from std_msgs.msg import Header
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from geographic_msgs.msg import GeoPoseStamped
from mavros_msgs.msg import State, Altitude
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from math import sqrt


class Drone():
    def __init__(self):
        self.hz = 25
        self.lat2m = 111178.605412924
        self.long2m = 75486.48933635012 # http://www.csgnetwork.com/degreelenllavcalc.html
        self.altitude = 30
        self.geofenceRange = 200 #meters
        self.distanceThreshold = 2

        self.rate = rospy.Rate(self.hz)
        self.state = State()
        self.receivedPosition = False
        self.received_GPS_target = False
        self.received_GPS_position = False
        self.received_MAVROS_altitude = False

        self.simulation = False # ------------------------------------- IMPORTANT to set prior flight -----------------------------------
        
        
        # self.home_position = PoseStamped()
        # self.current_position = PoseStamped()

        self.GPS_target = GeoPoseStamped()        
        
        self.current_GPS_position = NavSatFix()
        self.home_GPS_position = NavSatFix() 


        self.current_MAVROS_altitude= Altitude()
        self.home_MAVROS_altitude= Altitude()


       # self.altitude = 1

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
        #self.pos_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.position_cb)
        self.GPS_pos_sub = rospy.Subscriber("/mavros/global_position/raw/fix", NavSatFix, self.GPS_pos_cb)
        self.MAVROS_altitude_sub = rospy.Subscriber("/mavros/altitude", Altitude, self.mavros_altitude_cb)

        self.tar_pos_sub = rospy.Subscriber("/gui/target", GeoPoseStamped, self.GPS_target_cb) 

        ## Publishers:
        #self.target_pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=1)
        self.target_pos_pub = rospy.Publisher("/mavros/setpoint_position/global", GeoPoseStamped, queue_size=1)
        
    ## Callbacks
    def state_cb(self, state):
        self.state = state

    def mavros_altitude_cb(self, data): #  ------------------
        self.current_MAVROS_altitude = data #Altitude
        if(not self.received_MAVROS_altitude):
            self.home_MAVROS_altitude = data
        self.received_MAVROS_altitude = True
        


    # def position_cb(self, position):
    #     self.current_position = position
    #     if(not self.receivedPosition):
    #         self.home_position = position
    #     self.receivedPosition = True

    def GPS_pos_cb(self, data):
        self.current_GPS_position = data #NavSatFix
        if(not self.received_GPS_position):
            self.home_GPS_position = data #NavSatFix
            rospy.loginfo("Home GPS position is set")
            
        self.received_GPS_position = True


    def GPS_target_cb(self, data):
        self.GPS_target = data #GeoPoseStamped
        self.received_GPS_target = True
        
        rospy.loginfo("GPS target: Long lat %f og %f" % (data.pose.position.longitude, data.pose.position.latitude))
        self.GPS_target.pose.position.altitude = self.current_MAVROS_altitude.amsl
        #if(not self.receivedPosition):
        #    self.home_position = position
        #self.receivedPosition = True

    def setup(self):
        print("Waiting for FCU connection...")
        while not self.state.connected:
            self.rate.sleep()
        print("FCU connected")

        self.GPS_To_Publish = GeoPoseStamped()
        #self.GPS_To_Publish = self.GPS_target

        print("Waiting on GPS target")



        header = Header()
        while not self.received_GPS_target:
            self.GPS_To_Publish.pose.position.latitude = self.home_GPS_position.latitude   #
            self.GPS_To_Publish.pose.position.longitude = self.home_GPS_position.longitude
            self.GPS_To_Publish.pose.position.altitude = self.home_GPS_position.altitude - 1000 # Stay at the home position 

            self.rate.sleep()
            header.stamp = rospy.Time.now()
            self.GPS_To_Publish.header = header
            self.target_pos_pub.publish(self.GPS_To_Publish) # Publishing GPS target at homeposition to keep in offboard mode 
            self.rate.sleep()
        print("GPS target received")
      


        # send a few takeoff commands before starting
        for i in range(20):
            self.target_pos_pub.publish(self.GPS_To_Publish)
            self.rate.sleep()


        print("Waiting for change mode to offboard rotorcraft...")
        while not self.state.mode == "OFFBOARD":
            self.rate.sleep()
            if not self.state.mode == "OFFBOARD" and self.simulation:
                 self.set_mode_client(base_mode=0, custom_mode="OFFBOARD")
                 rospy.loginfo("OFFBOARD enable")
        
        print("Waiting for change mode to arming rotorcraft...")
        while not self.state.armed: 
            self.rate.sleep()
            if not self.state.armed:
                self.arming_client(True)
                rospy.loginfo("Rotorcraft armed")

          
            


    




    def distanceToTarget(self,targetPosition): #
        #targetPostion is Geoposestamped


        latitude = targetPosition.pose.position.latitude - self.current_GPS_position.latitude 
        longitude = targetPosition.pose.position.longitude - self.current_GPS_position.longitude 
        altitude = targetPosition.pose.position.altitude - self.current_MAVROS_altitude.amsl
        latitudeINMeters = latitude*self.lat2m
        longitudeINMeters = longitude*self.long2m
        #print("altitude error", altitude, "Target Altitude ", targetPosition.pose.position.altitude, "Current alt", self.current_MAVROS_altitude.amsl)
       
        return sqrt(latitudeINMeters*latitudeINMeters + longitudeINMeters*longitudeINMeters + altitude*altitude) #+ z*z)

    def shutdownDrone(self):
        while not self.state.mode == "AUTO.LAND":
            self.land_client()
            rospy.loginfo("Landing..")
        rospy.signal_shutdown("Done")

    def geoFencing(self):
        if(self.distanceToTarget(self.GPS_target)>self.geofenceRange):
            print("Geofence violation detected")
            return True
        else:
            return False
                

    def flyRoute(self):
        header = Header()
        while self.geoFencing():
            self.rate.sleep()
            header.stamp = rospy.Time.now()
            self.target_pos_pub.publish(self.GPS_To_Publish)
            print("Distance to target " + str(self.distanceToTarget(self.GPS_target)) ) 
        rospy.loginfo("Flying Route")

        self.geoFencing()
        self.GPS_To_Publish.pose.position.latitude = self.home_GPS_position.latitude   #
        self.GPS_To_Publish.pose.position.longitude = self.home_GPS_position.longitude
        self.GPS_To_Publish.pose.position.altitude = self.home_MAVROS_altitude.amsl + self.altitude

     
        

        print("1 Publish takeoff Position:  Latitude" , self.GPS_To_Publish.pose.position.latitude, "Longitude ", self.GPS_To_Publish.pose.position.longitude, " Altitude ", self.GPS_To_Publish.pose.position.altitude)
        print("1 Home GPS Position: Latitude", self.home_GPS_position.latitude, "Long", self.home_GPS_position.longitude, "Altitude ",self.home_GPS_position.altitude )
        print("1 Distance to target ", self.distanceToTarget(self.GPS_To_Publish))
        print(" ")
           # print(self.GPS_target)
        while(self.distanceToTarget(self.GPS_To_Publish)>self.distanceThreshold):
            if(self.geoFencing()):
                break

            header.stamp = rospy.Time.now()
            self.GPS_To_Publish.header = header
            print("Distance to GPS takeoff pos  " + str(self.distanceToTarget(self.GPS_To_Publish)) )
            print("Distance to GPS take " + str(self.distanceToTarget(self.GPS_target)) )

            self.target_pos_pub.publish(self.GPS_To_Publish) 
            self.rate.sleep()


        self.land_client = rospy.ServiceProxy("/mavros/cmd/land", CommandTOL)
        self.takeoff_client = rospy.ServiceProxy("/mavros/cmd/takeoff", CommandTOL) # Gør disse noGET????

        self.GPS_To_Publish = self.GPS_target   #
        self.GPS_To_Publish.pose.position.altitude = self.home_MAVROS_altitude.amsl + self.altitude
           
        print("2 GPS_target:  Latitude" , self.GPS_target.pose.position.latitude, "Longitude ", self.GPS_target.pose.position.longitude, " Altitude ", self.GPS_target.pose.position.altitude)
        print("2 Publish takeoff Position:  Latitude" , self.GPS_To_Publish.pose.position.latitude, "Longitude ", self.GPS_To_Publish.pose.position.longitude, " Altitude ", self.GPS_To_Publish.pose.position.altitude)
        print("2 Home GPS Position: Latitude", self.home_GPS_position.latitude, "Long", self.home_GPS_position.longitude, "Altitude ",self.home_GPS_position.altitude )
        print("2 Distance to target ", self.distanceToTarget(self.GPS_To_Publish))
        print(" ")

        while ((self.distanceToTarget(self.GPS_To_Publish)>self.distanceThreshold)):
            if(self.geoFencing()):
                break
            #print("Distance to GPS targat   " + str(self.distanceToTarget(self.GPS_To_Publish)) )
            
            
            header.stamp = rospy.Time.now()
            self.GPS_To_Publish.header = header
            self.target_pos_pub.publish(self.GPS_To_Publish) 
            self.rate.sleep()
            


if __name__ == '__main__':
    rospy.init_node('drone_control', anonymous=True)
    drone = Drone()
    drone.flyRoute()
    drone.shutdownDrone()
    rospy.spin()

