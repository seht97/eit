#include <eit/offboard_node.h>
#include <eit/visual_node.h>


/* Main */
int main(int argc, char **argv) {
    // Initialize node
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    VisualNode vis_node(nh);

    OffboardNode ob_node(nh);
    ob_node.run();

    return 0;
}
