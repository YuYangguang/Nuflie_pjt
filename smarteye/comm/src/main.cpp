//#include "smarteye/world_model/WorldModel.h"
#include <stdio.h>
#include <comm.h>
#include <QCoreApplication>
using namespace smarteye;
int main(int argc, char **argv)
{
    int i=0;
    ROS_INFO("start comm_node process");
    ros::Time::init();
   // QCoreApplication app(argc, argv);
    comm this_comm(argc,argv,"comm_node");
   // app.exec();
    ros::spin();
    return 0;

    //this_comm.connect(this_comm,&this_comm::receiveBytes,this,&this_comm::_handleHeartbeat);
    //ros::spin();


}
