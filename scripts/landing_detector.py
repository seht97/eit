#!/usr/bin/python3
import sys
import rospy
from ros_numpy import numpify
from ros_numpy.point_cloud2 import pointcloud2_to_xyz_array, array_to_pointcloud2, pointcloud2_to_array, get_xyz_points
from sensor_msgs.point_cloud2 import PointCloud2
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point32
from std_msgs.msg import Bool
# Visulalization
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA, Header

import numpy as np
import pcl
from time import time

class LandingDetector:
    def __init__(self):
        # Init
        self.__get_drone_params()
        self.__init_publishers()
        self.__init_subscribers()
        self.run_detection = True
        self.pc = None
        #self.start_time = rospy.Time.now()
        #self.has_saved = False
        rospy.loginfo('Landing detector ready.')

    def __get_drone_params(self):
        # Get the drone parameters
        self.DRONE_WIDTH = rospy.get_param('~drone_width','')
        self.DRONE_HEIGHT = rospy.get_param('~drone_height','')
        if not (self.DRONE_WIDTH and self.DRONE_HEIGHT):
            rospy.logerr('Please specify the drone width and height as parameters (in launch file).')
            sys.exit(-1)
        self.visualize = rospy.get_param('~visualize', False)

    def __init_publishers(self):
        # Setup publishers
        self.point_pub = rospy.Publisher('~position', Point32, queue_size=1)
        self.allow_land_pub = rospy.Publisher('~allow_land', Bool, queue_size=1)
        if self.visualize:
            self.vis_pub = rospy.Publisher('~visualisation', Marker, queue_size=1)

    def __init_subscribers(self):
        # Setup subscribers
        pc_topic = rospy.get_param('~pointcloud_topic','')
        rgb_topic = rospy.get_param('~rgb_topic','')
        toggle_detection_topic = rospy.get_param('~toggle_detection_topic','')
        if not pc_topic:
            rospy.logerr('Parameter \'pointcloud_topic\' is not provided.')
            sys.exit(-1)
        if not toggle_detection_topic:
            rospy.logerr('Parameter \'toggle_detection_topic\' is not provided.')
            sys.exit(-1)
        
        rospy.Subscriber(pc_topic, PointCloud2, self.__pc_callback, queue_size=1)
        rospy.Subscriber(toggle_detection_topic, Bool, self.__toggle_detection_callback, queue_size=1)

    def __pc_callback(self, msg):
        # Callback for point cloud topic
        # Transform PointCloud2 to xyz-numpy array shape (N,3)
        pc = pointcloud2_to_xyz_array(msg).astype('float32')
        pcl_pc = pcl.PointCloud(pc).make_passthrough_filter()
        pcl_pc.set_filter_field_name("z")
        pcl_pc.set_filter_limits(0.1, 9.0)
        pc = pcl_pc.filter()
        
        # Only run detection if we recieve points and detection is initiated
        if pc.size > 1000:
            self.pc = pc
        if self.run_detection and self.pc is not None:
            if self.pc.size > 0:
                # New PC obtained, run normal detection
                self.detect()
            else:
                # if not, descend slowly until new PC is obtained
                point = Point32(x=0, y=0, z=0)
                self.point_pub.publish(point)

    def __toggle_detection_callback(self, msg):
        # Activate/deactivate detection
        self.run_detection = msg.data

    def update_box_filter(self):
        # Create box areas for the drone to search
        # Areas are specied as: [minx, maxx, miny, maxy]
        w = self.DRONE_WIDTH
        h = self.DRONE_HEIGHT
        # Step size for boxes
        dx = w/4
        dy = h/4
        # Save filter sizes
        self.box_filter = []
        # Num steps are based on area of current PC (which is dependent on altitude)
        # As y is smallest FOV we base it on that
        #num_steps = int(np.max(np.asarray(self.pc)[:,1]) / dy) - 1
        # Now we fix number of steps to 5, meaning 5 in each direction
        num_steps = 5
        #if num_steps <= 0:
        #    num_steps = 1
        #if num_steps > 5:
        #    num_steps = 5
        #print(int(np.asarray(self.pc)[-1,1] / dy), num_steps)
        #print('Num steps: {}, min_x: {}, min_y: {}, min_z: {}'.format(num_steps, np.min(np.asarray(self.pc)[:,0]), np.min(np.asarray(self.pc)[:,1]), np.min(np.asarray(self.pc)[:,2])))
        #print('Num steps: {}, max_x: {}, max_y: {}, max_z: {}'.format(num_steps, np.max(np.asarray(self.pc)[:,0]), np.max(np.asarray(self.pc)[:,1]), np.max(np.asarray(self.pc)[:,2])))
        
        for i in range(num_steps):
            for yi in range(-i,i+1):
                for xi in range(-i,i+1):
                    box = [-w/2+xi*dx, w/2+xi*dx, -h/2+yi*dy, h/2+yi*dy]
                    if box not in self.box_filter:
                        self.box_filter.append(box)

    def detect(self):
        # Run detection
        start_time = time()
        # Segment plane once from full PC
        # RANSAC plane estimation using normals (induces extra constraint)
        #seg = self.pc.make_segmenter_normals(ksearch=20)
        #seg.set_optimize_coefficients(True)
        #seg.set_model_type(pcl.SACMODEL_NORMAL_PLANE)
        #seg.set_normal_distance_weight(0.1)
        #seg.set_method_type(pcl.SAC_RANSAC)
        #seg.set_max_iterations(50)
        # Standard RANSAC plane segmentation
        seg = self.pc.make_segmenter()
        seg.set_optimize_coefficients(True)
        seg.set_model_type(pcl.SACMODEL_PLANE)
        seg.set_method_type(pcl.SAC_RANSAC)
        # Change this distance threshold to maximum distance between points on the plane (z-distance)
        seg.set_distance_threshold(0.50)
        indices, model = seg.segment()
        # Extract the plane estimated underneath the drone
        cloud_plane = self.pc.extract(indices, negative=False)

        # Take mean of 100 biggest distances
        z_array = np.asarray(cloud_plane)[:,2]
        n_max = 200
        max_sorted_z_array = z_array[np.argsort(z_array)][-n_max:]
        mean_max_dist_to_plane = np.mean(max_sorted_z_array)
        #max_dist_to_plane = np.max(np.asarray(cloud_plane)[:,2])
        #print("Mean of {} max distances to plane: {}".format(n_max, mean_max_dist_to_plane))

        # Default parameters
        found_landing_spot = False
        landing_loc = [0,0]
        # Update box filters
        self.update_box_filter()

        # Visualization
        if self.visualize:
            now = rospy.Time.now()
            header = Header(stamp=now, frame_id='camera_depth_optical_frame')
            COLOR = ColorRGBA(r=1.0, g=0, b=0, a=1)
            marker = Marker(header=header)
            marker.id = 0
            # Points
            marker.type = Marker.POINTS
            # Add/modify
            marker.action = Marker.ADD
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            pt_array = []
            color_array = []
            for pt in self.box_filter:
                pt_array.append(Point(x=(pt[0]+pt[1])/2, y=(pt[2]+pt[3])/2, z=mean_max_dist_to_plane))
                color_array.append(COLOR)
            marker.points = pt_array

        i = 0
        while not found_landing_spot and i < len(self.box_filter):
            # Filter box area around drone on plane segmentation PC
            # Get box min and max values
            xmin = self.box_filter[i][0]
            xmax = self.box_filter[i][1]
            ymin = self.box_filter[i][2]
            ymax = self.box_filter[i][3]
            # Filter PC according to box on plane segmented PC
            fil = cloud_plane.make_passthrough_filter()
            fil.set_filter_field_name("x")
            fil.set_filter_limits(xmin, xmax)
            cloud_plane_filtered = fil.filter()
            fil = cloud_plane_filtered.make_passthrough_filter()
            fil.set_filter_field_name("y")
            fil.set_filter_limits(ymin, ymax)
            cloud_plane_filtered = fil.filter()
            # Filter full PC according to box
            fil = self.pc.make_passthrough_filter()
            fil.set_filter_field_name("x")
            fil.set_filter_limits(xmin, xmax)
            full_cloud_filtered = fil.filter()
            fil = full_cloud_filtered.make_passthrough_filter()
            fil.set_filter_field_name("y")
            fil.set_filter_limits(ymin, ymax)
            full_cloud_filtered = fil.filter()
            #if (rospy.Time.now() - self.start_time) > rospy.Duration(20) and (rospy.Time.now() - self.start_time) < rospy.Duration(22) and not self.has_saved:
                
            # Decide wether there is an obstacle or not
            if cloud_plane_filtered.size >= full_cloud_filtered.size*0.9:
                landing_loc[0] = (xmin + xmax)/2
                landing_loc[1] = (ymin + ymax)/2
                if self.visualize:
                    color_array[i] = ColorRGBA(r=0, g=1.0, b=0, a=1)
                found_landing_spot = True
            else:
                i += 1
                #if not self.has_saved:
                #    self.has_saved = True
                #    rospy.logwarn('Saving')
                #    pcl.save(full_cloud_filtered, '/home/magnus/full_cloud_segment.pcd')
                #    pcl.save(cloud_plane_filtered, '/home/magnus/cloud_plane_segment.pcd')

        end_time = time()
        if self.visualize:
            marker.colors = color_array
            self.vis_pub.publish(marker)
        print('Elapsed time: {}'.format(end_time-start_time))
        if found_landing_spot:
            print("Landing spot found at: ({},{})".format(landing_loc[0], landing_loc[1]))
            #print(cloud_plane_filtered.size, full_cloud_filtered.size, cloud_plane.size)
            # Publish landing spot offset point to drone control
            point = Point32(x=landing_loc[0]*0.5, y=landing_loc[1]*0.5, z=1.0)
            self.point_pub.publish(point)
            # If we are under 2m from plane/ground, initiate auto land
            if mean_max_dist_to_plane < 2.0:
                self.allow_land_pub.publish(Bool(data=True))
        else:
            rospy.loginfo("Unable to detect sutiable landing spot.")


def main():
    rospy.init_node('landing_detector', log_level=rospy.INFO)
    ld = LandingDetector()
    rospy.spin()

if __name__ == "__main__":
    main()