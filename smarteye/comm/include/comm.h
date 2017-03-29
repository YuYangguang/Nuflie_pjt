#ifndef _COMM_H_
#define _COMM_H_



//#include <smarteye/core/core.hpp>

#include "SerialLink.h"
#include "QGCSerialPortInfo.h"
#include <QGeoCoordinate>
#include <stdio.h>
#include <string>
#include <iostream>
#include <signal.h>
#include <boost/thread.hpp>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>
#include <keyboard/Key.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/Waypoint.h>
#include <mavros_msgs/WaypointList.h>
namespace smarteye {
class comm: public QObject
{
    Q_OBJECT
public:
    comm(int argc,char** argv,const char * name);
    ~comm();
    void update(const ros::TimerEvent& event);


public:
    int  AgentID_;
    ros::Publisher  commInfo_pub_;
    ros::Publisher  heartbeatPub;
    ros::Publisher  vfrHudPub;
    ros::Publisher  rawIMUPub;
    ros::Timer      commUpdateTimer;
    boost::shared_ptr<ros::NodeHandle> nh;
    SerialLink* m_pSerialLinkLink;


    ros::ServiceClient arming_client;
    ros::ServiceClient set_mode_client;
    ros::ServiceClient waypoint_push_client;
    ros::Subscriber commandSubscriber;
    ros::Subscriber waypointsSubscriber;
    mavros_msgs::SetMode offb_set_mode;
    mavros_msgs::State current_state;
    mavros_msgs::CommandBool arm_cmd;
    mavros_msgs::WaypointPush waypoint_push;
    //send command to mavros
    void sendCommand(const keyboard::Key &key);
    void change_wp(const mavros_msgs::WaypointList & wp_list);

    //串口
    void ConnectToSerialPort(QString port);

    //消息解析
    void _handleHeartbeat(mavlink_message_t message);
    void _handleVfrHud(mavlink_message_t message);
    void _handleRawImu(mavlink_message_t message);

    void writeBytes(const char* buffer, int len);
public slots:
    void receiveBytes(LinkInterface* link, QByteArray b);


};
}


#endif


