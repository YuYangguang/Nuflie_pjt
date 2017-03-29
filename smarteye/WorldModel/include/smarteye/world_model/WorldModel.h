#ifndef _WORLD_MODEL_H_
#define _WORLD_MODEL_H_

#include <smarteye_common/WorldModelMsg.h>
#include <smarteye_common/UAV2UAVMsg.h>
#include <smarteye_common/UAV2UAVType.h>
#include <smarteye_common/UAV2stationMsg.h>
#include <smarteye_common/UAV2UAVMsg.h>
#include <smarteye_common/dataTransMsg.h>
#include <smarteye_common/station2UAVMsg.h>
#include <smarteye_common/imgTransMsg.h>
#include <smarteye_common/sensorMsg.h>
#include <smarteye_common/controlMsg.h>
#include <smarteye_common/px4controllerMsg.h>
#include <smarteye/core/core.hpp>

#include <stdio.h>
#include <string>
#include <iostream>
#include <signal.h>
#include <boost/thread.hpp>
#include <ros/ros.h>
#include <std_msgs/Int32.h>

namespace smarteye {
class WorldModel
{

public:
    WorldModel(int argc,char** argv,const char * name);
    ~WorldModel();
    void update(const ros::TimerEvent& event);
    void updateInstruction(void);
    void updateConsensus(void);
    void updateFeedback(void);

public:
    int  AgentID_;
    ros::Publisher  worldmodelinfo_pub_;
    ros::Timer      worldMdlUpdateTimer;
    boost::shared_ptr<ros::NodeHandle> nh;
    smarteye_common::WorldModelMsg worldMdlInfo;
    smarteye_common::station2UAVMsg instructionInfo;
    smarteye_common::UAV2stationMsg feedbackInfo;
    smarteye_common::UAV2UAVMsg consensusInfo;

    /*********************data interaction interface********************/
    /*******************here is for data transmitter**************/
    smarteye_common::dataTransMsg dTransReceivedInfo;   //data received from data transmitter
    ros::Subscriber dataTransSubscriber;
    void ReceiveDataTrans(std_msgs::Int32 msg);        //receive data from data transmitter driver node
    smarteye_common::dataTransMsg dTranSentInfo;   //data sent to data transmitter
    ros::Publisher dataTransPub;

    /******************here is for image transmitter************/
    smarteye_common::imgTransMsg  imgTransReceivedInfo;   //data received from image transmitter
    ros::Subscriber imgTransSubscriber;
    void ReceiveImgTrans(std_msgs::Int32 msg);        //receive image from image transmitter driver node
    smarteye_common::imgTransMsg  imgTransSentInfo;   //data sent to image transmitter
    ros::Publisher imgTransPub;
    /****************************************************************/

    /********************interface for other nodes********************/
public:
    ros::Subscriber controlSubscriber;  //receive high-level control information from control_node
    void ReceiveControlInfo(const smarteye_common::controlMsgConstPtr& msg);

    ros::Subscriber objcontrollerSubscriber;  //receive low-level control information from objcontroller_node
    void ReceiveObjcontrolInfo(const smarteye_common::px4controllerMsgConstPtr& msg);

    ros::Subscriber sensorInfoSubscriber;   //receive sensors informaton from sensor_process_node
    void ReceiveSensorInfo(const smarteye_common::sensorMsgConstPtr& msg);

};
}


#endif

