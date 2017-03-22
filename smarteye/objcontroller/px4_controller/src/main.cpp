#include <Px4Controller.h>

using namespace smarteye;
int main(int argc, char **argv)
{
    ROS_INFO("start objcontroller_node process");
    ros::Time::init();
    Px4Controller this_px4Controller(argc,argv,"objcontroller_node");
    ros::spin();
    return 0;
}
