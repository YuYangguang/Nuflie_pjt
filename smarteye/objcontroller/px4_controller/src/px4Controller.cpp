#include <Px4Controller.h>
using namespace smarteye;
Px4Controller::Px4Controller(int argc,char** argv,const char * name)
{
    ros::init(argc,argv,name);
    nh = boost::make_shared<ros::NodeHandle>();
    controlSubscriber=nh->subscribe("controlTopic", 10, &Px4Controller::ReceiveControlInfo,this);
    sensorInfoSubscriber=nh->subscribe("sensorInfoTopic", 10, &Px4Controller::ReceiveSensorInfo,this);
    worldModelSubscriber=nh->subscribe("worldModelTopic", 10, &Px4Controller::ReceiveWorldMdlInfo,this);
    px4controllerInfoPub =  nh->advertise<smarteye_common::px4controllerMsg>("objcontrollerTopic",10);
    /** 30ms触发一次的定时器 */
    px4controllerUpdateTimer = nh->createTimer(ros::Duration(0.015),& Px4Controller::update,this);
    setPointPublisher = nh->advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local",10);\
    setVelocityPublisher = nh->advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);
}

Px4Controller::~Px4Controller()
{
}

void Px4Controller::update(const ros::TimerEvent &event)
{

}

void Px4Controller::ReceiveSensorInfo(const smarteye_common::sensorMsgConstPtr &msg)
{

}

void Px4Controller::ReceiveControlInfo(const smarteye_common::controlMsgConstPtr &msg)
{

}

void Px4Controller::ReceiveWorldMdlInfo(const smarteye_common::WorldModelMsgConstPtr &msg)
{

}

