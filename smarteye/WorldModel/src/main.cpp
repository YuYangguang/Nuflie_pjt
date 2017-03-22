#include "smarteye/world_model/WorldModel.h"

using namespace smarteye;
int main(int argc, char **argv)
{
    ROS_INFO("start world_model process");
    ros::Time::init();
    WorldModel this_world_model(argc,argv,"world_model_node");
    ros::spin();
    return 0;
}
