#ifndef _CONTROL_H_
#define _CONTROL_H_

#include "plan.h"
#include "smarteye_common/flightStateMsg.h"
#include "smarteye_common/controlMsg.h"
#include <smarteye_common/px4controllerMsg.h>
#include <smarteye_common/WorldModelMsg.h>
#include <smarteye_common/sensorMsg.h>
#include <smarteye_common/planMsg.h>
#include <smarteye_common/strategyMsg.h>
#include <smarteye_common/controlMsg.h>
#include <geometry_msgs/TwistStamped.h>
#include <stdio.h>
#include <ros/ros.h>


namespace smarteye{

class control
{
public:
    control(int argc,char** argv,const char * name);
    ~control();

public:
    ros::Publisher  controlInfoPub;
    ros::Timer      controlUpdateTimer;
    ros::Publisher  velPub;
    boost::shared_ptr<ros::NodeHandle> nh;
    void update(const ros::TimerEvent& event);
    ros::Subscriber flightSateSub;
    void receiveFlightSate(smarteye_common::flightStateMsg flightMsg);
    Plan controlPlan;
    DirectGuidInf directGuide;
    CtrlStruct controlResult;
    smarteye_common::controlMsg  controlResMSg;
    ros::Publisher controlResultPub;
    geometry_msgs::TwistStamped velCommand;


};


}

#endif


