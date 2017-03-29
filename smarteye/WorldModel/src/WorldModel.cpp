#include <smarteye/world_model/WorldModel.h>

using namespace smarteye;

WorldModel::WorldModel(int argc,char** argv,const char * name)
{
    ros::init(argc,argv,name);
    int i;
    const char * environment;
    // 读取机器人标号，并赋值.
    if((environment = getenv("AGENT"))==NULL)
    {
        ROS_ERROR("this agent number is not read by uav");
        return;
    }
    AgentID_ = atoi(environment);
    if((environment = getenv("AGENT"))==NULL)
    {
        ROS_ERROR("this agent number is not read by uav");
        return ;
    }
    nh = boost::make_shared<ros::NodeHandle>();
    controlSubscriber=nh->subscribe("controlTopic", 10, &WorldModel::ReceiveControlInfo,this);
    objcontrollerSubscriber=nh->subscribe("objcontrollerTopic", 10, &WorldModel::ReceiveObjcontrolInfo,this);
    sensorInfoSubscriber=nh->subscribe("sensorProcessTopic", 10, &WorldModel::ReceiveSensorInfo,this);
    dataTransSubscriber=nh->subscribe("dataTransReceiveTopic", 10, &WorldModel::ReceiveDataTrans,this);
    imgTransSubscriber=nh->subscribe("imgTransReceiveTopic", 10, &WorldModel::ReceiveImgTrans,this);

    worldmodelinfo_pub_ =  nh->advertise<smarteye_common::WorldModelMsg>("worldModleTopic",10);
    dataTransPub= nh->advertise<smarteye_common::dataTransMsg>("dataTransPubTopic",10);
    imgTransPub= nh->advertise<smarteye_common::imgTransMsg>("imgTransPubTopic",10);
    /** 30ms触发一次的定时器 */
    worldMdlUpdateTimer = nh->createTimer(ros::Duration(0.015),&WorldModel::update,this);

    worldMdlInfo.consensusinfo.resize(UAV_NUM);
    worldMdlInfo.UAVinfo.resize(UAV_NUM);
}


WorldModel::~WorldModel()
{



}


void WorldModel::update(const ros::TimerEvent& event)
{


    /*******************************update the instruction information***************/
    updateInstruction();


    /******************************update the feedback information******************/
    updateFeedback();

    updateConsensus();

    worldmodelinfo_pub_.publish(worldMdlInfo);   //publish
    dataTransPub.publish(dTranSentInfo);
    imgTransPub.publish(imgTransSentInfo);

}



void WorldModel::updateInstruction(void)   //receive the instructions from ground station
{

}



void WorldModel::updateConsensus(void)
{

}

void WorldModel::updateFeedback(void)     //send the UAV state to ground station
{



}

void WorldModel::ReceiveDataTrans(std_msgs::Int32 msg)
{

}

void WorldModel::ReceiveImgTrans(std_msgs::Int32 msg)
{

}

void WorldModel::ReceiveControlInfo(const smarteye_common::controlMsgConstPtr &msg)
{

}

void WorldModel::ReceiveObjcontrolInfo(const smarteye_common::px4controllerMsgConstPtr &msg)
{

}

void WorldModel::ReceiveSensorInfo(const smarteye_common::sensorMsgConstPtr &msg)
{

}
