#!/usr/bin/python3
import sys
import rospy
from ros_numpy.point_cloud2 import pointcloud2_to_xyz_array, array_to_pointcloud2, pointcloud2_to_array, get_xyz_points

from sensor_msgs.point_cloud2 import PointCloud2

import numpy as np
import pcl

class LandingDetector:
    def __init__(self):
        # Init
        self._get_drone_params()
        self._init_publishers()
        self._init_subscribers()

    def _get_drone_params(self):
        # Get the drone parameters
        self.DRONE_WIDTH = rospy.get_param('~drone_width','')
        self.DRONE_HEIGHT = rospy.get_param('~drone_height','')
        if not (self.DRONE_WIDTH and self.DRONE_HEIGHT):
            rospy.logerr('Please specify the drone width and height as parameters (in launch file).')
            sys.exit(-1)

    def _init_publishers(self):
        # Setup publishers
        pass
        #self.filtered_pc_pub = rospy.Publisher('~filtered_pc', PointCloud2, queue_size=1)

    def _init_subscribers(self):
        # Setup subscribers
        pc_topic = rospy.get_param('~pointcloud_topic','')
        if not pc_topic:
            rospy.logerr('Parameter \'pointcloud_topic\' is not provided.')
            sys.exit(-1)
        rospy.Subscriber(pc_topic, PointCloud2, self._callback, queue_size=1)

    def _callback(self, msg):
        # Callback for point cloud topic
        # Transform PointCloud2 to xyz-numpy array shape (N,3)
        pc = pointcloud2_to_xyz_array(msg).astype('float32')
        
        # Only run detection if we recieve points
        if pc.size > 0:
            self.pc = pcl.PointCloud(pc)
            self.detect()

        #self.filtered_pc_pub.publish(array_to_pointcloud2(pc_rec_array, frame_id='camera_link'))

    def detect(self):
        # Run detection
        # Filter box area around drone
        fil = self.pc.make_passthrough_filter()
        fil.set_filter_field_name("x")
        fil.set_filter_limits(-self.DRONE_WIDTH/2, self.DRONE_WIDTH/2)
        cloud_filtered = fil.filter()
        fil = cloud_filtered.make_passthrough_filter()
        fil.set_filter_field_name("y")
        fil.set_filter_limits(-self.DRONE_HEIGHT/2, self.DRONE_HEIGHT/2)
        cloud_filtered = fil.filter()
        # Publish filtered point cloud
        #self.filtered_pc_pub.publish(array_to_pointcloud2(np.asarray(cloud_filtered)))

        # Smoothing
        #fil = self.pc.make_statistical_outlier_filter()
        #fil.set_mean_k(50)
        #fil.set_std_dev_mul_thresh(1.0)
        #cloud_filtered = fil.filter()
        #pcl.save(cloud_filtered, '/home/magnus/EiT/filtered_pcd.pcd')

        seg = cloud_filtered.make_segmenter_normals(ksearch=50)
        seg.set_optimize_coefficients(True)
        seg.set_model_type(pcl.SACMODEL_NORMAL_PLANE)
        seg.set_normal_distance_weight(0.1)
        seg.set_method_type(pcl.SAC_RANSAC)
        seg.set_max_iterations(100)
        # Change this distance threshold to maximum distance between points on the plane (z-distance)
        seg.set_distance_threshold(0.1)
        indices, model = seg.segment()
        # Extract the plane estimated underneath the drone
        cloud_plane = cloud_filtered.extract(indices, negative=False)

        if cloud_plane.size == cloud_filtered.size:
            print("True")
        else:
            print("False")


def main():
    rospy.init_node('mir_interfacer', log_level=rospy.INFO)
    ld = LandingDetector()
    rospy.spin()

if __name__ == "__main__":
    main()