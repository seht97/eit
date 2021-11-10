#ifndef VISUAL_NODE_H
#define VISUAL_NODE_H

#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

class VisualNode {
public:
    // Constructors
    explicit VisualNode(ros::NodeHandle nh);

    // Properties
    const sensor_msgs::PointCloud2 &pcl() const {return _pcl;}
    const sensor_msgs::CameraInfo &depthInfo() const {return _depthInfo;}
    const sensor_msgs::Image &depthData() const {return _depthData;}
    const sensor_msgs::CameraInfo &rgbInfo() const {return _rgbInfo;}
    const sensor_msgs::Image &rgbData() const {return _rgbData;}

private:
    // Callbacks
    void _pclCb(const sensor_msgs::PointCloud2::ConstPtr &msg) {_pcl = *msg;}
    void _depthInfoCb(const sensor_msgs::CameraInfo::ConstPtr &msg) {_depthInfo = *msg;}
    void _depthDataCb(const sensor_msgs::Image::ConstPtr &msg) {_depthData = *msg;}
    void _rgbInfoCb(const sensor_msgs::CameraInfo::ConstPtr &msg) {_rgbInfo = *msg;}
    void _rgbDataCb(const sensor_msgs::Image::ConstPtr &msg) {_rgbData = *msg;}

    // ROS node
    ros::NodeHandle &_nh;
    const ros::Subscriber _pclSub, _depthInfoSub, _depthDataSub, _rgbInfoSub, _rgbDataSub;

    // Messages
    sensor_msgs::PointCloud2 _pcl;
    sensor_msgs::CameraInfo _depthInfo, _rgbInfo;
    sensor_msgs::Image _depthData, _rgbData;
};

#endif
