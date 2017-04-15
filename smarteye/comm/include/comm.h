#ifndef _COMM_H_
#define _COMM_H_


#include <smarteye/core/core.hpp>
#include "smarteye_common/flightStateMsg.h"
#include "smarteye_common/controlMsg.h"
#include <stdio.h>
#include <string>
#include <iostream>
#include <boost/thread.hpp>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>
#include <keyboard/Key.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/Waypoint.h>
#include <mavros_msgs/WaypointList.h>
#include <serial/serial.h>
#include <standard/mavlink.h>
#include <queue>
#include <pthread.h>
#include <tf/tf.h>
using namespace std;


extern serial::Serial px4Serial;
extern serial::Serial P900Serial;
namespace smarteye {
class comm
{

public:
    comm();
    ~comm();
    void init(int argc,char** argv,const char * name);
public:
    int  AgentID_;
    smarteye_common::flightStateMsg flightState;
    ros::Publisher  commandPub;
    ros::Publisher  flightStatePub;
    smarteye_common::controlMsg controlResult;
    ros::Subscriber controlResultSub;
    ros::Timer      commUpdateTimer;
    keyboard::Key   command;
    boost::shared_ptr<ros::NodeHandle> nh;
    ros::ServiceClient arming_client;
    ros::ServiceClient set_mode_client;
    ros::Subscriber commandSubscriber;
    mavros_msgs::SetMode offb_set_mode;
    mavros_msgs::State current_state;
    mavros_msgs::CommandBool arm_cmd;
    void sendCommand(const keyboard::Key &key);
    void handleMissionItem(mavlink_message_t* message);
    void handleHILActuatorControls(mavlink_message_t *message);
    void handleAttitudeItem(mavlink_message_t *message);
    void handleLocalPosNed(mavlink_message_t *message);
    void receiveControlRes(smarteye_common::controlMsg result);
    pthread_t threadPx4R;

    //send command to mavros

    double commandTime;
private:

    pthread_t threadPx4W;
    static pthread_mutex_t px4mutex;
    static pthread_mutex_t P900mutex;
    static pthread_mutex_t DPx42QGC_mutex;
    static pthread_mutex_t DQGC2Px4_mutex;
    static queue<uint8_t> DPx42QGC;
    static queue<uint8_t> DQGC2Px4;
    void update(const ros::TimerEvent& event);
    void *writePx4(void *ptr);
    void writeSetAttitude();

};


}


#endif


