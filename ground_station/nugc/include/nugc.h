#include <iostream>
#include <fstream>
#include <sstream>
#include <signal.h>
#include <ros/ros.h>
#include <boost/thread.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <stdio.h>
#include <std_msgs/Float64.h>
#include <keyboard.h>
#include <smarteye/core/core.hpp>
#include <std_msgs/Int32.h>
#include <smarteye_common/station2UAVMsg.h>
#include <smarteye_common/UAV2stationMsg.h>
#include <smarteye_common/dataTransMsg.h>
#include <smarteye_common/imgTransMsg.h>
#include <standard/mavlink.h>
#include <serial/serial.h>
namespace smarteye {
class nugc
{

public:
    nugc(int argc,char** argv,const char * name);
    ~nugc();
    void update(const ros::TimerEvent& event);
    ros::Timer      nugc_update_timer_;
    boost::shared_ptr<ros::NodeHandle> nh;
    serial::Serial ser; //声明串口对象

public:



    /*********************data interaction interface********************/
    /*******************here is for data transmitter**************/
    void writeWaypoint(uint8_t frame, float x, float y, float z);
    void writeCommand();
    void handleLocalpose(mavlink_message_t* message);
    ros::Publisher localPosPub;

    /******************here is for image transmitter************/
    smarteye_common::imgTransMsg  imgTransReceive_info_;   //data received from image transmitter
    ros::Subscriber imgTransSubscriber;
    void imgTransReceived(std_msgs::Int32 msg);        //receive image from image transmitter driver node

    /*********************here is for command input********************/
    ros::Subscriber inputSubscriber;
    keyboard::Key command;
    void commandinputReceived(keyboard::KeyConstPtr msg);


    /****************************************************************/






};
}



