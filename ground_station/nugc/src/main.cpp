
#include "nugc.h"

using namespace smarteye;
int main(int argc, char **argv)
{
    ROS_INFO("start nugc process");
    ros::Time::init();
    nugc this_nugc(argc,argv,"world_model_node");
    ros::spin();
    return 0;
}
