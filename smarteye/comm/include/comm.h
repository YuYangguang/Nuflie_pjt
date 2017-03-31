#ifndef _COMM_H_
#define _COMM_H_


#include <smarteye/core/core.hpp>
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
namespace smarteye {
class comm
{

public:
    comm(int argc,char** argv,const char * name);
    ~comm();
    void update(const ros::TimerEvent& event);


public:
    int  AgentID_;
    ros::Publisher  commInfo_pub_;
    ros::Publisher  rawIMUPub;
    ros::Publisher  commandPub;
    ros::Timer      commUpdateTimer;
    keyboard::Key   command;
    boost::shared_ptr<ros::NodeHandle> nh;
    serial::Serial ser; //声明串口对象

    ros::ServiceClient arming_client;
    ros::ServiceClient set_mode_client;
    ros::ServiceClient waypoint_push_client;
    ros::Subscriber commandSubscriber;
    ros::Subscriber waypointsSubscriber;

    ros::Subscriber localposSubscriber;
    ros::Subscriber localvelSubscriber;
    mavlink_local_position_ned_t localpose;
    mavros_msgs::SetMode offb_set_mode;
    mavros_msgs::State current_state;
    mavros_msgs::CommandBool arm_cmd;
    mavros_msgs::Waypoint temp_waypoints[3];
    mavros_msgs::WaypointPush waypoint_push;
    //send command to mavros
    void sendCommand(const keyboard::Key &key);
    void change_wp(const mavros_msgs::WaypointList & wp_list);
    void handleMissionItem(mavlink_message_t* message);
    void handleHILActuatorControls(mavlink_message_t* message);
    double commandTime;
    void receiveLocalpos(geometry_msgs::PoseStamped pose);
    void receiveLocalvel(geometry_msgs::TwistStamped vel);
    void writeLocalPose();
};
}


#endif


