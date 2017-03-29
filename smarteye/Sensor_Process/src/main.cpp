#include <sensor_process/sensor_process.h>

using namespace smarteye;
int main(int argc, char **argv)
{
    ROS_INFO("start sensors_node process");
    ros::Time::init();
    SensorProcess this_sensor_process(argc,argv,"sensor_process_node");
    ros::spin();
    return 0;
}

