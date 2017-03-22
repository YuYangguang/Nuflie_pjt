#include <control.h>

using namespace smarteye;
int main(int argc, char **argv)
{
    ROS_INFO("start control_node process");
    ros::Time::init();
    control this_controll(argc,argv,"control_node");
    ros::spin();
    return 0;
}
