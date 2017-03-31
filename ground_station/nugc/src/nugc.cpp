#include <nugc.h>


using namespace smarteye;



nugc::nugc(int argc,char** argv,const char * name)
{

    ros::init(argc,argv,"nugc");

    const char * environment;
    nh= boost::make_shared<ros::NodeHandle>("nugc");
    nugc_update_timer_=nh->createTimer(ros::Duration(0.1),&nugc::update,this);
    imgTransSubscriber=nh->subscribe("imgTransReceiveTopic", 10, &nugc::imgTransReceived,this);
    inputSubscriber=nh->subscribe("/keyboard/keydown", 10, &nugc::commandinputReceived,this);
    localPosPub=nh->advertise<geometry_msgs::PoseStamped>("/nugc/local_pose",10);


    try
    {
        //设置串口属性，并打开串口
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(57600);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port /dev/ttyUSB0");
    }

    //检测串口是否已经打开，并给出提示信息
    if(ser.isOpen())
    {
        ROS_INFO_STREAM("Serial Port initialized");
    }
    else
    {
        ROS_ERROR_STREAM(" port /dev/ttyUSB0 is not open");
    }


}

nugc::~nugc()
{



}

void nugc::update(const ros::TimerEvent& event)
{

    float x=1;
    float y=2;
    float z=3;
    uint8_t frame=4;
    writeWaypoint(frame,x,y,z);
    writeCommand();
}

void nugc::writeWaypoint(uint8_t frame, float x, float y, float z)
{
    mavlink_system_t mavlink_system;
    mavlink_system.sysid = 20;                   ///< ID 20 for this airplane
    mavlink_system.compid = MAV_COMP_ID_IMU;
    uint8_t target_system=18;
    uint8_t target_component=MAV_COMP_ID_IMU;
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t seq=0;
    uint16_t command=16;
    uint8_t current=true;
    uint8_t autocontinue=true;
    float param1=0;
    float param2=0;
    float param3=0;
    float param4=0;
    mavlink_msg_mission_item_pack( mavlink_system.sysid , mavlink_system.compid, &msg,
                                   target_system, target_component, seq, frame, command,  current, autocontinue,  param1,  param2,  param3,  param4,  x,  y,  z);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    ser.write(buf,len);

}

void nugc::writeCommand()
{
    mavlink_system_t mavlink_system;
    mavlink_system.sysid = 20;                   ///< ID 20 for this airplane
    mavlink_system.compid = MAV_COMP_ID_IMU;
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint64_t time_usec=command.header.stamp.toNSec();
    float controls=0;
    uint8_t mode=command.code;
    uint64_t flags=0;
    mavlink_msg_hil_actuator_controls_pack(mavlink_system.sysid, mavlink_system.compid, &msg,
                                           time_usec, &controls, mode, flags);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    ser.write(buf,len);
    ROS_INFO("I write");
}

void nugc::handleLocalpose(mavlink_message_t *message)
{
    mavlink_local_position_ned_t mavlink_pose;
    geometry_msgs::PoseStamped ros_pose;
    mavlink_msg_local_position_ned_decode(message,&mavlink_pose);
    ros_pose.pose.position.x=mavlink_pose.x;
    ros_pose.pose.position.y=mavlink_pose.y;
    ros_pose.pose.position.z=mavlink_pose.z;
    localPosPub.publish(ros_pose);
}



void nugc::imgTransReceived(std_msgs::Int32 msg)
{

}

void nugc::commandinputReceived(keyboard::KeyConstPtr msg)
{
   command.code=msg->code;
   command.header.stamp=msg->header.stamp;

}


