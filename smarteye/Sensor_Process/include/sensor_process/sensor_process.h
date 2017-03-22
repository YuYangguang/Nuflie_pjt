#ifndef _SENSOR_PROCESS_H_
#define _SENSOR_PROCESS_H_

#include <sensor_process/camera.h>
#include <sensor_process/GPS.h>
#include <sensor_process/IMU.h>
#include <sensor_process/sonar.h>
#include <smarteye_common/sensorMsg.h>
#include <smarteye_common/controlMsg.h>
#include <smarteye_common/px4controllerMsg.h>
#include <smarteye_common/WorldModelMsg.h>
#include <stdio.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
namespace smarteye {
class SensorProcess
{
public:
    SensorProcess(int argc,char** argv,const char * name);
    ~SensorProcess();

public:
    ros::Publisher  sensorInfo_pub_;
    ros::Timer      sensor_update_timer_;
    boost::shared_ptr<ros::NodeHandle> nh;
    smarteye_common::sensorMsg  sensorInfo;
    void FuseSensorInfo();
    void update(const ros::TimerEvent& event);

/*****************here is for GPS********************/
    GPSInfo GPS;
    ros::Subscriber GPSSubscriber;
    void ReceiveGPS(const std_msgs::Int32ConstPtr& data);   //receive data from driver package

/********************here is for camera**************/
    cameraInfo camera;
    ros::Subscriber imageSubscriber;
    void ReceiveImage(const std_msgs::Int32ConstPtr& data);  //receive data from driver package

/********************here is for IMU**************/
    IMUInfo IMU;
    ros::Subscriber IMUSubscriber;
    void ReceiveIMU(const std_msgs::Int32ConstPtr& data);    //receive data from driver package

/********************here is for sonar**************/
    SonarInfo sonar;
    ros::Subscriber sonarSubscriber;
    void ReceiveSonar(const std_msgs::Int32ConstPtr& data);   //receive data from driver package

    /********************interface for other nodes********************/
public:
    ros::Subscriber controlSubscriber;  //receive high-level control information from control_node
    void ReceiveControlInfo(const smarteye_common::controlMsgConstPtr& msg);

    ros::Subscriber objcontrollerSubscriber;  //receive low-level control information from objcontroller_node
    void ReceiveObjcontrolInfo(const smarteye_common::px4controllerMsgConstPtr& msg);

    ros::Subscriber worldModelSubscriber;   //receive sensors informaton from sensor_process_node
    void ReceiveWorldMdlInfo(const smarteye_common::WorldModelMsgConstPtr& msg);
};


}

#endif
