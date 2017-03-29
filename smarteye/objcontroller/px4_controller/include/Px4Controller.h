#ifndef PX4CONTROLLER_H_
#define PX4CONTROLLER_H_

#include <smarteye_common/WorldModelMsg.h>
#include <smarteye_common/px4controllerMsg.h>
#include <smarteye_common/sensorMsg.h>
#include <smarteye_common/controlMsg.h>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
namespace smarteye{
class Px4Controller
{
public:
    Px4Controller(int argc,char** argv,const char* name);
    ~Px4Controller();

public:
    ros::Publisher  px4controllerInfoPub;
    ros::Timer      px4controllerUpdateTimer;
    boost::shared_ptr<ros::NodeHandle> nh;
    void update(const ros::TimerEvent& event);

    smarteye_common::controlMsg  controlInfo;
    geometry_msgs::PoseStamped targetPoint;
    geometry_msgs::TwistStamped targetVelocity;
    ros::Publisher setPointPublisher;
    ros::Publisher setVelocityPublisher;

/********************interface for other nodes********************/
public:
    ros::Subscriber sensorInfoSubscriber;   //receive sensors informaton from sensor_process_node
    void ReceiveSensorInfo(const smarteye_common::sensorMsgConstPtr& msg);

    ros::Subscriber controlSubscriber;  //receive low-level control information from objcontroller_node
    void ReceiveControlInfo(const smarteye_common::controlMsgConstPtr& msg);

    ros::Subscriber worldModelSubscriber;   //receive sensors informaton from sensor_process_node
    void ReceiveWorldMdlInfo(const smarteye_common::WorldModelMsgConstPtr& msg);

};
}


#endif
