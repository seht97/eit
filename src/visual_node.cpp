#include <eit/visual_node.h>


/* Constructors */
VisualNode::VisualNode(ros::NodeHandle nh)
    : _nh{nh},
      _pclSub{_nh.subscribe<sensor_msgs::PointCloud2>("/pcl", 10, &VisualNode::_pclCb, this)},
      _depthInfoSub{_nh.subscribe<sensor_msgs::CameraInfo>("/depth/info", 10, &VisualNode::_depthInfoCb, this)},
      _depthDataSub{_nh.subscribe<sensor_msgs::Image>("/depth/data", 10, &VisualNode::_depthDataCb, this)},
      _rgbInfoSub{_nh.subscribe<sensor_msgs::CameraInfo>("/rgb/info", 10, &VisualNode::_rgbInfoCb, this)},
      _rgbDataSub{_nh.subscribe<sensor_msgs::Image>("/rgb/data", 10, &VisualNode::_rgbDataCb, this)}
{}

/* Functions */
