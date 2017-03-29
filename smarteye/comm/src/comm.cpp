#include <comm.h>
//mavlink_status_t m_mavlink_status[MAVLINK_COMM_NUM_BUFFERS];

smarteye::comm::comm(int argc, char **argv, const char *name):
    QObject()
{
    ros::init(argc,argv,name);
    nh = boost::make_shared<ros::NodeHandle>();
    //ConnectToSerialPort("ttyUSB0");
    heartbeatPub=nh->advertise<std_msgs::Int32>("/comm/heartbeat",10);
    vfrHudPub=nh->advertise<std_msgs::Float32>("/comm/vfrHud",10);
    rawIMUPub=nh->advertise<sensor_msgs::Imu>("/comm/rawImu",10);
    commandSubscriber = nh->subscribe("/keyboard/keydown",1,&comm::sendCommand,this);
    waypointsSubscriber = nh->subscribe("/mavros/mission/waypoints",10,&comm::change_wp,this);
    commUpdateTimer= nh->createTimer(ros::Duration(0.015),&comm::update,this);
    arming_client = nh->serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    set_mode_client = nh->serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

}

smarteye::comm::~comm()
{

}

void smarteye::comm::update(const ros::TimerEvent &event)
{

}

void smarteye::comm::sendCommand(const keyboard::Key &key)
{
    switch(key.code)
    {

    case 't':
    {

        mavros_msgs::Waypoint temp_waypoints[3];
        temp_waypoints[0].frame = 3;
        temp_waypoints[0].command = 16;
        temp_waypoints[0].is_current = true;
        temp_waypoints[0].autocontinue = true;
        temp_waypoints[0].param1 = 0.0;
        temp_waypoints[0].param2 = 0.0;
        temp_waypoints[0].param3 = 0.0;
        temp_waypoints[0].param4 = 0.0;
        temp_waypoints[0].x_lat = 28.2210254669;
        temp_waypoints[0].y_long = 112.991294861;
        temp_waypoints[0].z_alt = 8.0;


        temp_waypoints[1].frame = 3;
        temp_waypoints[1].command = 16;
        temp_waypoints[1].is_current = false;
        temp_waypoints[1].autocontinue = true;
        temp_waypoints[1].param1 = 10.0;
        temp_waypoints[1].param2 = 0.0;
        temp_waypoints[1].param3 = 0.0;
        temp_waypoints[1].param4 = 0.0;
        temp_waypoints[1].x_lat = 28.2215254669;
        temp_waypoints[1].y_long = 112.991694861;
        temp_waypoints[1].z_alt = 8.0;

        temp_waypoints[2].frame = 3;
        temp_waypoints[2].command = 16;
        temp_waypoints[2].is_current = false;
        temp_waypoints[2].autocontinue = true;
        temp_waypoints[2].param1 = 0.0;
        temp_waypoints[2].param2 = 0.0;
        temp_waypoints[2].param3 = 0.0;
        temp_waypoints[2].param4 = 0.0;
        temp_waypoints[2].x_lat = 28.2218254669;
        temp_waypoints[2].y_long = 112.991694861;
        temp_waypoints[2].z_alt = 8.0;
        waypoint_push.request.waypoints.clear();
        waypoint_push.request.waypoints.push_back(temp_waypoints[0]);
        waypoint_push.request.waypoints.push_back(temp_waypoints[1]);
        waypoint_push.request.waypoints.push_back(temp_waypoints[2]);
        waypoint_push_client.call(waypoint_push);
        if (waypoint_push.response.success)
            ROS_WARN_STREAM("Vehicle received: " << waypoint_push.response.wp_transfered );
        break;

    }

    case 's':
    {

        mavros_msgs::Waypoint temp_waypoints[3];
        temp_waypoints[0].frame = 3;
        temp_waypoints[0].command = 16;
        temp_waypoints[0].is_current = true;
        temp_waypoints[0].autocontinue = true;
        temp_waypoints[0].param1 = 0.0;
        temp_waypoints[0].param2 = 0.0;
        temp_waypoints[0].param3 = 0.0;
        temp_waypoints[0].param4 = 0.0;
        temp_waypoints[0].x_lat = 28.2210254669;
        temp_waypoints[0].y_long = 112.991294861;
        temp_waypoints[0].z_alt = 8.0;


        temp_waypoints[1].frame = 3;
        temp_waypoints[1].command = 16;
        temp_waypoints[1].is_current = false;
        temp_waypoints[1].autocontinue = true;
        temp_waypoints[1].param1 = 10.0;
        temp_waypoints[1].param2 = 0.0;
        temp_waypoints[1].param3 = 0.0;
        temp_waypoints[1].param4 = 0.0;
        temp_waypoints[1].x_lat = 28.2215254669;
        temp_waypoints[1].y_long = 112.991694861;
        temp_waypoints[1].z_alt = 8.0;

        temp_waypoints[2].frame = 3;
        temp_waypoints[2].command = 16;
        temp_waypoints[2].is_current = false;
        temp_waypoints[2].autocontinue = false;
        temp_waypoints[2].param1 = 0.0;
        temp_waypoints[2].param2 = 0.0;
        temp_waypoints[2].param3 = 0.0;
        temp_waypoints[2].param4 = 0.0;
        temp_waypoints[2].x_lat = 28.2218254669;
        temp_waypoints[2].y_long = 112.991694861;
        temp_waypoints[2].z_alt = 8.0;
        waypoint_push.request.waypoints.clear();
        waypoint_push.request.waypoints.push_back(temp_waypoints[0]);
        waypoint_push.request.waypoints.push_back(temp_waypoints[1]);
        waypoint_push.request.waypoints.push_back(temp_waypoints[2]);
        waypoint_push_client.call(waypoint_push);
        if (waypoint_push.response.success)
            ROS_WARN_STREAM("Vehicle received: " << waypoint_push.response.wp_transfered );
        break;

    }

    case 'z':    //armed
    {
        arm_cmd.request.value = true;
        arming_client.call(arm_cmd);
        if (arm_cmd.response.success)
          ROS_WARN_STREAM("Vehicle armed");
        break;

    }


    case 'x':   //disarmedd
    {
      arm_cmd.request.value = false;
      arming_client.call(arm_cmd);
      if (arm_cmd.response.success)
        ROS_WARN_STREAM("Vehicle disarmed");
      break;
    }
    default:
    {

        break;
    }
    }

}

void smarteye::comm::change_wp(const mavros_msgs::WaypointList &wp_list)
{
    ROS_INFO("got waypointd data");
    //delete [4] when reached [2] (that means [3].is_current == true)
    if(wp_list.waypoints.size() > 6)
    {
        if(wp_list.waypoints[3].is_current == true)
        {
            ROS_INFO("reached[2]");

            waypoint_push.request.waypoints.clear();
            waypoint_push.request.waypoints = wp_list.waypoints;
            waypoint_push.request.waypoints.erase(waypoint_push.request.waypoints.begin()+4);
            waypoint_push.request.waypoints.erase(waypoint_push.request.waypoints.begin(), waypoint_push.request.waypoints.begin()+2);
            //before upload new waypoints, check if the first waypoint is too far!!!
            waypoint_push_client.call(waypoint_push);
        }
    }
    else
        ROS_INFO("number of waypoints are %d", wp_list.waypoints.size());

}

void smarteye::comm::_handleHeartbeat(mavlink_message_t message)
{

    mavlink_heartbeat_t heartbeat;

    mavlink_msg_heartbeat_decode(&message, &heartbeat);
    static int c=0;
    c++;
    std_msgs::Int32 heartbeatCount;
    heartbeatCount.data=c;
    heartbeatPub.publish(heartbeatCount);


}

void smarteye::comm::_handleVfrHud(mavlink_message_t message)
{
    mavlink_vfr_hud_t vfrHud;
    mavlink_msg_vfr_hud_decode(&message, &vfrHud);
    std_msgs::Float32 vfrHudMsg;
    vfrHudMsg.data=vfrHud.heading;
    vfrHudPub.publish(vfrHudMsg);

}

void smarteye::comm::_handleRawImu(mavlink_message_t message)
{
    ROS_INFO("This is IMU");
    mavlink_raw_imu_t rawImu;
    mavlink_msg_raw_imu_decode(&message, &rawImu);
    sensor_msgs::Imu ImuMsg;
    ImuMsg.linear_acceleration.x=rawImu.xacc;
    ImuMsg.linear_acceleration.y=rawImu.yacc;
    ImuMsg.linear_acceleration.z=rawImu.zacc;
    ImuMsg.angular_velocity.x=rawImu.xgyro;
    ImuMsg.angular_velocity.y=rawImu.ygyro;
    ImuMsg.angular_velocity.z=rawImu.zgyro;
    rawIMUPub.publish(ImuMsg);
}



void smarteye::comm::receiveBytes(LinkInterface *link, QByteArray b)
{

    mavlink_message_t message;
    mavlink_status_t status;

    for (int position = 0; position < b.size(); position++)
    {
        if (mavlink_parse_char(link->mavlinkChannel(), (uint8_t)(b[position]), &message, &status) == 1)
        {
            if (!link->decodedFirstMavlinkPacket())
            {
                link->setDecodedFirstMavlinkPacket(true);
            }

            switch (message.msgid)
            {
            case MAVLINK_MSG_ID_HOME_POSITION:
                ROS_INFO("This is home");
                break;
            case MAVLINK_MSG_ID_HEARTBEAT:
                _handleHeartbeat(message);
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
                _handleVfrHud(message);
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
}

void smarteye::comm::ConnectToSerialPort(QString port)
{
    //m_pSerialLinkLink->_port->clearError();

    ROS_INFO("I have connected");
    SerialConfiguration* pSerialConfig=new SerialConfiguration("Serial Link");
    pSerialConfig->setBaud(921600);
    pSerialConfig->setPortName(port);

    SharedLinkConfigurationPointer * pconfig = new SharedLinkConfigurationPointer(pSerialConfig);
    m_pSerialLinkLink = new SerialLink(*pconfig);
    m_pSerialLinkLink->_setMavlinkChannel(1);

    bool ret = m_pSerialLinkLink->_connect();

    if (!ret) return ;

    connect(m_pSerialLinkLink, &LinkInterface::bytesReceived,  this,  &comm::receiveBytes);

}
