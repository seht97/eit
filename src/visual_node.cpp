#include <eit/visual_node.h>


/* Constructors */
VisualNode::VisualNode(ros::NodeHandle nh)
    : _nh{nh}, _pclSub{_nh.subscribe<sensor_msgs::PointCloud2>("/pointcloud", 10, &VisualNode::_pclCb, this)} {}

/* Functions */
