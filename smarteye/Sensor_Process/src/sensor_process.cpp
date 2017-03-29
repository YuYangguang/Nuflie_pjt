#include <sensor_process/sensor_process.h>

using namespace smarteye;
SensorProcess::SensorProcess(int argc,char** argv,const char * name)
{
    ros::init(argc,argv,name);
    nh = boost::make_shared<ros::NodeHandle>();
    controlSubscriber=nh->subscribe("controlTopic", 10, &SensorProcess::ReceiveControlInfo,this);
    objcontrollerSubscriber=nh->subscribe("objcontrollerTopic", 10, &SensorProcess::ReceiveObjcontrolInfo,this);
    worldModelSubscriber=nh->subscribe("worldModelTopic", 10, &SensorProcess::ReceiveWorldMdlInfo,this);
    sensorInfo_pub_ =  nh->advertise<smarteye_common::sensorMsg>("sensorProcessTopic",10);
    /** 30ms触发一次的定时器 */
    sensor_update_timer_ = nh->createTimer(ros::Duration(0.015),&SensorProcess::update,this);
}

SensorProcess::~SensorProcess()
{
}

void SensorProcess::FuseSensorInfo()
{

}

void SensorProcess::ReceiveGPS(const std_msgs::Int32ConstPtr &data)
{

}

void SensorProcess::ReceiveImage(const std_msgs::Int32ConstPtr &data)
{

}

void SensorProcess::ReceiveIMU(const std_msgs::Int32ConstPtr &data)
{

}

void SensorProcess::ReceiveSonar(const std_msgs::Int32ConstPtr &data)
{

}

void SensorProcess::ReceiveControlInfo(const smarteye_common::controlMsgConstPtr &msg)
{

}

void SensorProcess::ReceiveObjcontrolInfo(const smarteye_common::px4controllerMsgConstPtr &msg)
{

}

void SensorProcess::ReceiveWorldMdlInfo(const smarteye_common::WorldModelMsgConstPtr &msg)
{

}

void SensorProcess::update(const ros::TimerEvent &event)
{
    FuseSensorInfo();
    sensorInfo_pub_.publish(sensorInfo);
}
