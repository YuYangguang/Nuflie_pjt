//#include "smarteye/world_model/WorldModel.h"
#include <stdio.h>
#include <comm.h>
#include <pthread.h>
#include <queue>
using namespace smarteye;
using namespace std;

static pthread_mutex_t px4mutex;
static pthread_mutex_t P900mutex;
static pthread_mutex_t DPx42QGC_mutex;
static pthread_mutex_t DQGC2Px4_mutex;
static queue<uint8_t> DPx42QGC;
static queue<uint8_t> DQGC2Px4;
void *downRead(void *ptr)   //Read the pixhawk Port
{
    mavlink_message_t msg;
    mavlink_status_t status;
    uint8_t buffer;
    while(1)
    {
        while(px4Serial.available()!=0) // there exsits data in the serial buffer
        {
            px4Serial.read(&buffer,1);
            if(mavlink_parse_char(MAVLINK_COMM_0, buffer, &msg, &status))
            {
                uint8_t buf[MAVLINK_MAX_PACKET_LEN];
                uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
                P900Serial.write(buf,len);

//                switch(msg.msgid)
//                {

//                case MAVLINK_MSG_ID_HEARTBEAT:
//                {
//                    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
//                    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
//                    P900Serial.write(buf,len);
//                    break;
//                }
//                case MAVLINK_MSG_ID_SYS_STATUS:
//                {
//                    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
//                    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
//                    P900Serial.write(buf,len);
//                    break;
//                }
//                case MAVLINK_MSG_ID_ATTITUDE:
//                {
//                    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
//                    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
//                    P900Serial.write(buf,len);
//                    break;

//                }
//                }


            }
        }
        //sleep(0.1);

    }




}



void *upRead(void *ptr)   //Read the P900 port
{
    mavlink_message_t msg;
    mavlink_status_t status;
    uint8_t buffer;
    while(1)
    {
        while(P900Serial.available()!=0) // there exsits data in the serial buffer
        {
            P900Serial.read(&buffer,1);
            if(mavlink_parse_char(MAVLINK_COMM_0, buffer, &msg, &status))
            {
                uint8_t buf[MAVLINK_MAX_PACKET_LEN];
                uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
                ROS_INFO("px4 write");
                px4Serial.write(buf,len);
            }
        }
        //sleep(0.01);


    }


}

int main(int argc, char **argv)
{

    mavlink_message_t msg;
    mavlink_status_t status;

    ros::Time::init();
    ROS_INFO("start bridge_node process");
    ros::Time::init();
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    pthread_mutex_init(&px4mutex,NULL);
    pthread_mutex_init(&P900mutex,NULL);
    pthread_mutex_init(&DPx42QGC_mutex,NULL);
    pthread_mutex_init(&DQGC2Px4_mutex,NULL);

    try
    {
        //设置串口属性，并打开串口
        px4Serial.setPort("/dev/ttyACM0");
        px4Serial.setBaudrate(57600);

        px4Serial.setTimeout(to);
        px4Serial.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port /dev/ACM0");
    }

    //检测串口是否已经打开，并给出提示信息
    if( px4Serial.isOpen())
    {
        ROS_INFO_STREAM("Serial ACM0 initialized");
    }
    else
    {
        ROS_ERROR_STREAM(" port /dev/ACM0 is not open");
    }

    try
    {
        //设置串口属性，并打开串口
        P900Serial.setPort("/dev/ttyUSB0");
        P900Serial.setBaudrate(57600);
        P900Serial.setTimeout(to);
        P900Serial.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port /dev/ttyUSB0");
    }

    //检测串口是否已经打开，并给出提示信息
    if(P900Serial.isOpen())
    {
        ROS_INFO_STREAM("Serial Port initialized");
    }
    else
    {
        ROS_ERROR_STREAM(" port /dev/ttyUSB0 is not open");
    }
    int ret;
    //    pthread_t uWrite;
    //    ret = pthread_create(&uWrite, NULL, upWrite, NULL);
    //    if(ret) {
    //        cout << "Create pthread error!" << endl;
    //        return 1;
    //    }

    pthread_t uRead;
    ret = pthread_create(&uRead, NULL, upRead, NULL);
    if(ret) {
        cout << "Create pthread error!" << endl;
        return 1;
    }

    //    pthread_t dWrite;
    //    ret = pthread_create(&dWrite, NULL, downWrite, NULL);
    //    if(ret) {
    //        cout << "Create pthread error!" << endl;
    //        return 1;
    //    }

    pthread_t dRead;
    ret = pthread_create(&dRead, NULL, downRead, NULL);
    if(ret) {
        cout << "Create pthread error!" << endl;
        return 1;
    }
    //pthread_join(uWrite, NULL);
    pthread_join(uRead, NULL);
    //pthread_join(dWrite, NULL);
    pthread_join(dRead, NULL);
    px4Serial.close();
    P900Serial.close();

    return 0;
}
