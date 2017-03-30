#include <ros/ros.h> 
#include <serial/serial.h>  //ROS已经内置了的串口包 
#include <std_msgs/String.h> 
#include <std_msgs/Empty.h> 
#include <mavlink.h>
#include <mavlink_types.h>
serial::Serial ser; //声明串口对象 
static int packet_drops = 0;


//回调函数 
void write_callback(const std_msgs::String::ConstPtr& msg) 
{ 
    ROS_INFO_STREAM("Writing to serial port" <<msg->data);
    ser.write(msg->data);   //发送串口数据
} 

int main (int argc, char** argv) 
{ 
    //初始化节点
    ros::init(argc, argv, "serial_example_node");
    //声明节点句柄
    ros::NodeHandle nh;
    mavlink_message_t msg;
    mavlink_status_t status;

    //订阅主题，并配置回调函数
    ros::Subscriber write_sub = nh.subscribe("write", 1000, write_callback);
    //发布主题
    ros::Publisher read_pub = nh.advertise<std_msgs::String>("read", 1000);

    try
    {
        //设置串口属性，并打开串口
        ser.setPort("/dev/ttyACM0");
        ser.setBaudrate(57600);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    //检测串口是否已经打开，并给出提示信息
    if(ser.isOpen())
    {
        ROS_INFO_STREAM("Serial Port initialized");
    }
    else
    {
        return -1;
    }

    //指定循环的频率
    ros::Rate loop_rate(50);
    while(ros::ok())
    {




            while(ser.available())
            {
                uint8_t c;
                ser.read(&c,1);
                // Try to get a new message
                if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status))
                {
                    // Handle message

                    switch(msg.msgid)
                    {
                    case MAVLINK_MSG_ID_HOME_POSITION:
                        ROS_INFO("This is home");
                        break;
                    case MAVLINK_MSG_ID_HEARTBEAT:
                        ROS_INFO("This is 1");
                        break;
                    case MAVLINK_MSG_ID_RC_CHANNELS:
                        ROS_INFO("This is 2");
                        break;
                    case MAVLINK_MSG_ID_RC_CHANNELS_RAW:
                        ROS_INFO("This is 3");
                        break;
                    case MAVLINK_MSG_ID_BATTERY_STATUS:
                        ROS_INFO("This is 4");
                        break;
                    case MAVLINK_MSG_ID_SYS_STATUS:
                        ROS_INFO("This is 5");
                        break;
                    case MAVLINK_MSG_ID_RAW_IMU:
                        ROS_INFO("This is 6");
                        break;
                    case MAVLINK_MSG_ID_SCALED_IMU:
                        ROS_INFO("This is 7");
                        break;
                    case MAVLINK_MSG_ID_SCALED_IMU2:
                        ROS_INFO("This is 8");
                        break;
                    case MAVLINK_MSG_ID_SCALED_IMU3:
                        ROS_INFO("This is 9");
                        break;
                    case MAVLINK_MSG_ID_VIBRATION:
                        ROS_INFO("This is 10");
                        break;
                    case MAVLINK_MSG_ID_EXTENDED_SYS_STATE:
                        ROS_INFO("This is 11");
                        break;
                    case MAVLINK_MSG_ID_COMMAND_ACK:
                        ROS_INFO("This is 12");
                        break;
                    case MAVLINK_MSG_ID_AUTOPILOT_VERSION:
                        break;
                    case MAVLINK_MSG_ID_WIND_COV:
                        ROS_INFO("This is 13");
                        break;
                    case MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS:
                        ROS_INFO("This is 14");
                        break;
                    case MAVLINK_MSG_ID_GPS_RAW_INT:
                        ROS_INFO("This is 15");
                        break;
                    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
                        ROS_INFO("This is 16");
                        break;
                    case MAVLINK_MSG_ID_ALTITUDE:
                        ROS_INFO("This is 17");
                        break;
                    case MAVLINK_MSG_ID_ATTITUDE:
                        ROS_INFO("This is 18");
                        break;
                    case MAVLINK_MSG_ID_VFR_HUD:
                        ROS_INFO("This is 19");
                        break;
                    case MAVLINK_MSG_ID_MISSION_REQUEST:
                        ROS_INFO("This is home");
                        break;
                    case MAVLINK_MSG_ID_MISSION_ACK:
                        ROS_INFO("This is home");
                        break;
                    case MAVLINK_MSG_ID_MISSION_COUNT:
                        ROS_INFO("This is home");
                        break;
                    case MAVLINK_MSG_ID_MISSION_ITEM:
                        ROS_INFO("This is home");
                        break;
                    case MAVLINK_MSG_ID_PARAM_VALUE:
                        ROS_INFO("This is home");
                        break;
                    }
                }

            }

            //处理ROS的信息，比如订阅消息,并调用回调函数
            ros::spinOnce();
            loop_rate.sleep();


    }
}
