#include <comm.h>
//mavlink_status_t m_mavlink_status[MAVLINK_COMM_NUM_BUFFERS];

smarteye::comm::comm(int argc, char **argv, const char *name)
{
    ros::init(argc,argv,name);
    nh = boost::make_shared<ros::NodeHandle>("~");

    rawIMUPub=nh->advertise<sensor_msgs::Imu>("comm/rawImu",10);
    commandSubscriber = nh->subscribe("/keyboard/keydown",1,&comm::sendCommand,this);
    waypointsSubscriber = nh->subscribe("/mavros/mission/waypoints",10,&comm::change_wp,this);
    commUpdateTimer= nh->createTimer(ros::Duration(0.015),&comm::update,this);
    arming_client = nh->serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    set_mode_client = nh->serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

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
        ROS_ERROR_STREAM("Unable to open port /dev/ttyACM0");
    }

    //检测串口是否已经打开，并给出提示信息
    if(ser.isOpen())
    {
        ROS_INFO_STREAM("Serial Port initialized");
    }
    else
    {
        ROS_ERROR_STREAM(" port /dev/ttyACM0 is not open");
    }

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

    case '1':
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
        temp_waypoints[0].z_alt = 0.1;


        waypoint_push.request.waypoints.clear();
        waypoint_push.request.waypoints.push_back(temp_waypoints[0]);

        waypoint_push_client.call(waypoint_push);
        if (waypoint_push.response.success)
            ROS_WARN_STREAM("Vehicle received: " << waypoint_push.response.wp_transfered );
        break;

    }

    case '2':
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

