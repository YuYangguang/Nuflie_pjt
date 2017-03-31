#include <comm.h>
//mavlink_status_t m_mavlink_status[MAVLINK_COMM_NUM_BUFFERS];

smarteye::comm::comm(int argc, char **argv, const char *name)
{
    ros::init(argc,argv,name);
    nh = boost::make_shared<ros::NodeHandle>("~");
    commandTime=0;
    rawIMUPub=nh->advertise<sensor_msgs::Imu>("comm/rawImu",10);
    commandSubscriber = nh->subscribe("/keyboard/keydown",1,&comm::sendCommand,this);
    waypointsSubscriber = nh->subscribe("/mavros/mission/waypoints",10,&comm::change_wp,this);
    localposSubscriber=nh->subscribe("/mavros/local_position/pose",10,&comm::receiveLocalpos,this);
    localvelSubscriber=nh->subscribe("/mavros/local_position/velocity",10,&comm::receiveLocalvel,this);
    commUpdateTimer= nh->createTimer(ros::Duration(0.015),&comm::update,this);
    arming_client = nh->serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    set_mode_client = nh->serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    commandPub=nh->advertise<keyboard::Key>("/comm/keyboard",10);

    temp_waypoints[0].frame = 4;
    temp_waypoints[0].command = 16;
    temp_waypoints[0].is_current = true;
    temp_waypoints[0].autocontinue = true;
    temp_waypoints[0].param1 = 0.0;
    temp_waypoints[0].param2 = 0.0;
    temp_waypoints[0].param3 = 0.0;
    temp_waypoints[0].param4 = 0.0;
    temp_waypoints[0].x_lat = 0;
    temp_waypoints[0].y_long = 0;
    temp_waypoints[0].z_alt = 0;

    try
    {
        //设置串口属性，并打开串口
        ser.setPort("/dev/ttyUSB1");
        ser.setBaudrate(57600);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port /dev/ttyUSB1");
    }

    //检测串口是否已经打开，并给出提示信息
    if(ser.isOpen())
    {
        ROS_INFO_STREAM("Serial Port initialized");
    }
    else
    {
        ROS_ERROR_STREAM(" port /dev/ttyUSB1 is not open");
    }

}

smarteye::comm::~comm()
{

}

void smarteye::comm::update(const ros::TimerEvent &event)
{
    writeLocalPose();


}

void smarteye::comm::sendCommand(const keyboard::Key &key)
{
    switch(key.code)
    {

    case 'u':
    {


        temp_waypoints[0].frame = 4;
        temp_waypoints[0].command = 16;
        temp_waypoints[0].is_current = true;
        temp_waypoints[0].autocontinue = true;
        temp_waypoints[0].param1 = 0.0;
        temp_waypoints[0].param2 = 0.0;
        temp_waypoints[0].param3 = 0.0;
        temp_waypoints[0].param4 = 0.0;
        temp_waypoints[0].x_lat = 0;
        temp_waypoints[0].y_long = 0;
        temp_waypoints[0].z_alt = 0.1;


        waypoint_push.request.waypoints.clear();
        waypoint_push.request.waypoints.push_back(temp_waypoints[0]);

        waypoint_push_client.call(waypoint_push);
        if (waypoint_push.response.success)
            ROS_WARN_STREAM("Vehicle received: " << waypoint_push.response.wp_transfered );
        break;

    }

    case 'o':
    {

        mavros_msgs::Waypoint temp_waypoints[1];
        temp_waypoints[0].frame = 4;
        temp_waypoints[0].command = 16;
        temp_waypoints[0].is_current = true;
        temp_waypoints[0].autocontinue = true;
        temp_waypoints[0].param1 = 0.0;
        temp_waypoints[0].param2 = 0.0;
        temp_waypoints[0].param3 = 0.0;
        temp_waypoints[0].param4 = 0.0;
        temp_waypoints[0].x_lat = 0;
        temp_waypoints[0].y_long = 0;
        temp_waypoints[0].z_alt = 2;


        waypoint_push.request.waypoints.clear();
        waypoint_push.request.waypoints.push_back(temp_waypoints[0]);

        waypoint_push_client.call(waypoint_push);
        if (waypoint_push.response.success)
            ROS_WARN_STREAM("Vehicle received: " << waypoint_push.response.wp_transfered );
        break;

    }

    case '9':    //armed
    {
        arm_cmd.request.value = true;
        arming_client.call(arm_cmd);
        if (arm_cmd.response.success)
            ROS_WARN_STREAM("Vehicle armed");
        break;

    }


    case '0':   //disarmedd
    {
        arm_cmd.request.value = false;
        arming_client.call(arm_cmd);
        if (!arm_cmd.response.success)
        {
            ROS_WARN_STREAM("Disarm unsuccessful");
        }
        if (arm_cmd.response.success)
            ROS_WARN_STREAM("Vehicle disarmed");
        break;
    }
    case '8':   //offboard
    {

        offb_set_mode.request.custom_mode = "OFFBOARD";
        set_mode_client.call(offb_set_mode);

        if (offb_set_mode.response.success)
            ROS_WARN_STREAM("Offboard enabled");
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

void smarteye::comm::handleMissionItem(mavlink_message_t* message)
{
    mavlink_mission_item_t mavlink_mission_item;
    mavlink_msg_mission_item_decode(message,&mavlink_mission_item);
    int seq=mavlink_mission_item.seq;
    temp_waypoints[seq].frame = mavlink_mission_item.frame;
    temp_waypoints[seq].command = mavlink_mission_item.command;
    temp_waypoints[seq].is_current = mavlink_mission_item.current;
    temp_waypoints[seq].autocontinue = mavlink_mission_item.autocontinue;
    temp_waypoints[seq].param1 = mavlink_mission_item.param1;
    temp_waypoints[seq].param2 = mavlink_mission_item.param2;
    temp_waypoints[seq].param3 = mavlink_mission_item.param3;
    temp_waypoints[seq].param4 = mavlink_mission_item.param4;
    temp_waypoints[seq].x_lat = mavlink_mission_item.x;
    temp_waypoints[seq].y_long = mavlink_mission_item.y;
    temp_waypoints[seq].z_alt = mavlink_mission_item.z;
}

void smarteye::comm::handleHILActuatorControls(mavlink_message_t *message)
{
    mavlink_hil_actuator_controls_t control;
    mavlink_msg_hil_actuator_controls_decode(message,&control);
    if (control.time_usec!=commandTime)
    {
        commandTime=control.time_usec;
        command.code=control.mode;
        commandPub.publish(command);
    }

}

void smarteye::comm::receiveLocalpos(geometry_msgs::PoseStamped pose)
{
    localpose.x=pose.pose.position.x;
    localpose.y=pose.pose.position.y;
    localpose.z=pose.pose.position.z;

}

void smarteye::comm::receiveLocalvel(geometry_msgs::TwistStamped vel)
{
    localpose.vx=vel.twist.linear.x;
    localpose.vy=vel.twist.linear.y;
    localpose.vz=vel.twist.linear.z;

}

void smarteye::comm::writeLocalPose()
{
    mavlink_system_t mavlink_system;
    mavlink_system.sysid = 20;                   ///< ID 20 for this airplane
    mavlink_system.compid = MAV_COMP_ID_IMU;
    mavlink_message_t msg;
    uint32_t time_boot_ms=ros::Time::now().toNSec();
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_local_position_ned_pack(mavlink_system.sysid, mavlink_system.compid, &msg,
                                        time_boot_ms, localpose.x, localpose.y, localpose.z, localpose.vx, localpose.vy, localpose.vz);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    ser.write(buf,len);
}

