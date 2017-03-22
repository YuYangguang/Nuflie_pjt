#ifndef _CONTROL_H_
#define _CONTROL_H_
#include <smarteye_common/px4controllerMsg.h>
#include <smarteye_common/WorldModelMsg.h>
#include <smarteye_common/sensorMsg.h>
#include <smarteye_common/planMsg.h>
#include <smarteye_common/strategyMsg.h>
#include <smarteye_common/behaviorMsg.h>
#include <smarteye_common/controlMsg.h>

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
    boost::shared_ptr<ros::NodeHandle> nh;
    void update(const ros::TimerEvent& event);
    smarteye_common::controlMsg  controlInfo;
    smarteye_common::behaviorMsg behaviorInfo;
    smarteye_common::planMsg planInfo;
    smarteye_common::strategyMsg strategyInfo;

    /********************interface for other nodes********************/
public:
    ros::Subscriber sensorInfoSubscriber;   //receive sensors informaton from sensor_process_node
    void ReceiveSensorInfo(const smarteye_common::sensorMsgConstPtr& msg);

    ros::Subscriber objcontrollerSubscriber;  //receive low-level control information from objcontroller_node
    void ReceiveObjcontrolInfo(const smarteye_common::px4controllerMsgConstPtr& msg);

    ros::Subscriber worldModelSubscriber;   //receive sensors informaton from sensor_process_node
    void ReceiveWorldMdlInfo(const smarteye_common::WorldModelMsgConstPtr& msg);
};


}

#endif


