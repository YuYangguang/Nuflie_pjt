#include <nugc.h>


using namespace smarteye;



nugc::nugc(int argc,char** argv,const char * name)
{

    ros::init(argc,argv,"nugc");

    const char * environment;
    nh= boost::make_shared<ros::NodeHandle>("nugc");
    nugc_update_timer_=nh->createTimer(ros::Duration(0.015),&nugc::update,this);
    dataTransSubscriber=nh->subscribe("dataTransReceiveTopic", 10, &nugc::dataTransReceived,this);
    imgTransSubscriber=nh->subscribe("imgTransReceiveTopic", 10, &nugc::imgTransReceived,this);
    inputSubscriber=nh->subscribe("commandInputTopic", 10, &nugc::commandinputReceived,this);
    dataTrans_pub_=nh->advertise<smarteye_common::dataTransMsg>("dataTransPubTopic",10);

}

nugc::~nugc()
{



}

void nugc::update(const ros::TimerEvent& event)
{
    dataTrans_pub_.publish(dTranSent_info_);

}

void nugc::dataTransReceived(std_msgs::Int32 msg)
{

}

void nugc::imgTransReceived(std_msgs::Int32 msg)
{

}

void nugc::commandinputReceived(keyboard::KeyConstPtr msg)
{

}


