//#include "smarteye/world_model/WorldModel.h"
#include <stdio.h>
#include <comm.h>


using namespace smarteye;
int main(int argc, char **argv)
{
    mavlink_message_t msg;
    mavlink_status_t status;
    ROS_INFO("start comm_node process");
    ros::Time::init();
    comm this_comm(argc,argv,"comm_node");
    ros::Rate loop_rate(50);
    while(ros::ok())
    {
        while(this_comm.ser.available())
        {
            uint8_t buffer;
            this_comm.ser.read(&buffer,1);
            // Try to get a new message
            if(mavlink_parse_char(MAVLINK_COMM_0, buffer, &msg, &status))
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
                    ROS_INFO("control received");
                    this_comm.handleHILActuatorControls(&msg);
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
                    ROS_INFO("points received");
                    this_comm.handleMissionItem(&msg);
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
    return 0;
}
