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
namespace smarteye {
class nugc
{

public:
    nugc(int argc,char** argv,const char * name);
    ~nugc();
    void update(const ros::TimerEvent& event);
    ros::Timer      nugc_update_timer_;
    boost::shared_ptr<ros::NodeHandle> nh;


public:
    smarteye_common::station2UAVMsg instruction_info_;
    smarteye_common::UAV2stationMsg feedback_info_;

    /*********************data interaction interface********************/
    /*******************here is for data transmitter**************/
    smarteye_common::dataTransMsg dTransReceive_info_;   //data received from data transmitter
    ros::Subscriber dataTransSubscriber;
    void dataTransReceived(std_msgs::Int32 msg);        //receive data from data transmitter driver node
    smarteye_common::dataTransMsg dTranSent_info_;   //data sent to data transmitter
    ros::Publisher dataTrans_pub_;

    /******************here is for image transmitter************/
    smarteye_common::imgTransMsg  imgTransReceive_info_;   //data received from image transmitter
    ros::Subscriber imgTransSubscriber;
    void imgTransReceived(std_msgs::Int32 msg);        //receive image from image transmitter driver node

    /*********************here is for command input********************/
     keyboard::Key commandinput;
     ros::Subscriber inputSubscriber;
     void commandinputReceived(keyboard::KeyConstPtr msg);


    /****************************************************************/




   

};
}



