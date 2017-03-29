#include <control.h>

using namespace smarteye;
control::control(int argc, char **argv, const char *name)
{

    ros::init(argc,argv,name);
    nh = boost::make_shared<ros::NodeHandle>();
    sensorInfoSubscriber=nh->subscribe("sensorProcessTopic", 10, &control::ReceiveSensorInfo,this);
    objcontrollerSubscriber=nh->subscribe("objcontrollerTopic", 10, &control::ReceiveObjcontrolInfo,this);
    worldModelSubscriber=nh->subscribe("worldModelTopic", 10, &control::ReceiveWorldMdlInfo,this);
    controlInfoPub =  nh->advertise<smarteye_common::controlMsg>("controlTopic",10);
    /** 30ms触发一次的定时器 */
    controlUpdateTimer = nh->createTimer(ros::Duration(0.015),&control::update,this);
}

control::~control()
{

}

void control::update(const ros::TimerEvent &event)
{
    controlInfoPub.publish(controlInfo);
}

void control::ReceiveSensorInfo(const smarteye_common::sensorMsgConstPtr &msg)
{

}

void control::ReceiveObjcontrolInfo(const smarteye_common::px4controllerMsgConstPtr &msg)
{

}

void control::ReceiveWorldMdlInfo(const smarteye_common::WorldModelMsgConstPtr &msg)
{

}

