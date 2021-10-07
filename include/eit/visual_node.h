#ifndef VISUAL_NODE_H
#define VISUAL_NODE_H

#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

class VisualNode {
public:
    // Constructors
    explicit VisualNode(ros::NodeHandle nh);

private:
    // Callbacks
    void _pclCb(const sensor_msgs::PointCloud2::ConstPtr &msg) {_pcl = *msg;}

    // ROS node
    ros::NodeHandle &_nh;
    const ros::Subscriber _pclSub;

    // Messages
    sensor_msgs::PointCloud2 _pcl;
};

#endif
