//#include "smarteye/world_model/WorldModel.h"
#include <stdio.h>
#include <comm.h>


using namespace smarteye;
comm this_comm;
static void *readPx4(void *ptr)
{    mavlink_message_t msg;
     mavlink_status_t status;
      uint8_t buffer;
       while(1)
       {
           while(px4Serial.available()!=0) // there exsits data in the serial buffer
           {
               px4Serial.read(&buffer,1);
               if(mavlink_parse_char(MAVLINK_COMM_0, buffer, &msg, &status))
               {
                   //uint8_t buf[MAVLINK_MAX_PACKET_LEN];
                   //uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
                   //   P900Serial.write(buf,len);
                   switch(msg.msgid)
                   {

                   case MAVLINK_MSG_ID_HEARTBEAT:
                   {
                       ROS_INFO("heartbeat received");
//                       mavlink_heartbeat_t heartbeat;
//                       mavlink_msg_heartbeat_decode(ms&g,&heartbeat);
                       break;
                   }
                   case MAVLINK_MSG_ID_SYS_STATUS:
                   {
                       uint8_t buf[MAVLINK_MAX_PACKET_LEN];
                       uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
                       break;
                   }
                   case MAVLINK_MSG_ID_ATTITUDE:
                   {
                       this_comm.handleAttitudeItem(&msg);
                       break;

                   }
                   case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
                   {
                       this_comm.handleLocalPosNed(&msg);
                       break;

                   }
                   }

               }
           }
       }

}
int main(int argc, char **argv)
{

    ROS_INFO("start comm_node process");
    ros::Time::init();
    this_comm.init(argc,argv,"comm_code");
    int ret = pthread_create(&this_comm.threadPx4R, NULL, readPx4, NULL);
    if(ret)
    {
        cout << "Create pthread error!" << endl;
        return 0;
    }

    ros::Rate loop_rate(50);
    while(ros::ok())
    {

        ros::spinOnce();
        loop_rate.sleep();


    }
    return 0;
}
