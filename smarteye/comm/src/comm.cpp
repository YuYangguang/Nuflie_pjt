#include <comm.h>
serial::Serial px4Serial;
serial::Serial P900Serial;

smarteye::comm::comm()
{

}



smarteye::comm::~comm()
{
    px4Serial.close();

}

void smarteye::comm::init(int argc, char **argv, const char *name)
{
    ros::init(argc,argv,name);
    nh = boost::make_shared<ros::NodeHandle>("~");
    std::string Px4PortName;
    std::string P00PortName;

    bool IsParamGet=nh->getParam("Px4PortName",Px4PortName);
    if(IsParamGet==false)
    {
        printf("get param success \n");
    }

    //printf("PortName is %s",Px4PortName);
    commandTime=0;
    commandSubscriber = nh->subscribe("/keyboard/keydown",1,&comm::sendCommand,this);
    commUpdateTimer= nh->createTimer(ros::Duration(0.05),&comm::update,this);
    arming_client = nh->serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    set_mode_client = nh->serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    commandPub=nh->advertise<keyboard::Key>("/comm/keyboard",10);
    flightStatePub=nh->advertise<smarteye_common::flightStateMsg>("/comm/flight_state",10);
    controlResultSub=nh->subscribe("/control/control_result",10,&comm::receiveControlRes,this);

    if(Px4PortName!="")
    {
        try
        {
            //设置串口属性，并打开串口
            px4Serial.setPort(Px4PortName);
            px4Serial.setBaudrate(57600);
            serial::Timeout to = serial::Timeout::simpleTimeout(1000);
            px4Serial.setTimeout(to);
            px4Serial.open();
        }
        catch (serial::IOException& e)
        {
            ROS_ERROR_STREAM("Unable to open Px4Port");
        }

        //检测串口是否已经打开，并给出提示信息
        if(px4Serial.isOpen())
        {
            ROS_INFO_STREAM("Px4Portinitialized");
        }
        else
        {
            ROS_ERROR_STREAM("Px4Port is not open");
        }

    }
    else
    {
        ROS_WARN("NO Px4Port is set");
    }
    nh = boost::make_shared<ros::NodeHandle>("~");

}

void smarteye::comm::update(const ros::TimerEvent &event)
{
    flightStatePub.publish(flightState);
    writeSetAttitude();
}

void *smarteye::comm::writePx4(void *ptr)
{

}

void smarteye::comm::writeSetAttitude()
{
    mavlink_system_t mavlink_system;
    mavlink_system.sysid = 1;                   ///< ID 20 for this airplane
    mavlink_system.compid = 1;
    mavlink_message_t msg;
    uint32_t time_boot_ms=ros::Time::now().toNSec();
    uint8_t target_system=0;
    uint8_t target_component=0;
    uint8_t type_mask=0;
    geometry_msgs::Quaternion quater;
    float thrust=controlResult.thrust;
    float roll=controlResult.roll;
    float pitch=controlResult.pitch;
    float yaw=controlResult.yaw;
    //quater=tf::createQuaternionMsgFromRollPitchYaw(txtdata[3],txtdata[4],txtdata[5]);
    quater=tf::createQuaternionMsgFromRollPitchYaw(roll,pitch,yaw);
    float q[4];
    q[0]=quater.w;
    q[1]=quater.x;
    q[2]=quater.y;
    q[3]=quater.z;
    //    printf("the q[0] is %f \n",q[0]);
    float body_roll_rate=roll;
    float body_pitch_rate=pitch;
    float body_yaw_rate=yaw;
    mavlink_msg_set_attitude_target_pack(mavlink_system.sysid, mavlink_system.compid, &msg,
        time_boot_ms,target_system, target_component, type_mask, q,  body_roll_rate, body_pitch_rate,  body_yaw_rate, thrust);
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    px4Serial.write(buf,len);

}

void smarteye::comm::sendCommand(const keyboard::Key &key)
{
//    switch(key.code)
//    {



//    case '9':    //armed
//    {
//        arm_cmd.request.value = true;
//        arming_client.call(arm_cmd);
//        if (arm_cmd.response.success)
//            ROS_WARN_STREAM("Vehicle armed");
//        break;

//    }


//    case '0':   //disarmedd
//    {
//        arm_cmd.request.value = false;
//        arming_client.call(arm_cmd);
//        if (!arm_cmd.response.success)
//        {
//            ROS_WARN_STREAM("Disarm unsuccessful");
//        }
//        if (arm_cmd.response.success)
//            ROS_WARN_STREAM("Vehicle disarmed");
//        break;
//    }
//    case '8':   //offboard
//    {

//        offb_set_mode.request.custom_mode = "OFFBOARD";
//        set_mode_client.call(offb_set_mode);

//        if (offb_set_mode.response.success)
//            ROS_WARN_STREAM("Offboard enabled");
//        break;
//    }
//    default:
//    {

//        break;
//    }
//    }

}



void smarteye::comm::handleMissionItem(mavlink_message_t* message)
{
    mavlink_mission_item_t mavlink_mission_item;
    mavlink_msg_mission_item_decode(message,&mavlink_mission_item);
    int seq=mavlink_mission_item.seq;

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

void smarteye::comm::handleAttitudeItem(mavlink_message_t *message)
{
    mavlink_attitude_t attitude;
    mavlink_msg_attitude_decode(message,&attitude);
    flightState.roll=attitude.roll;
    flightState.pitch=attitude.pitch;
    flightState.yaw=attitude.yaw;
    flightState.rollspeed=attitude.rollspeed;
    flightState.pitchspeed=attitude.pitchspeed;
    flightState.yawspeed=attitude.yawspeed;
}

void smarteye::comm::handleLocalPosNed(mavlink_message_t *message)
{
    mavlink_local_position_ned_t localposi;
    mavlink_msg_local_position_ned_decode(message,&localposi);
    flightState.x=localposi.x;
    flightState.y=localposi.y;
    flightState.z=localposi.z;

}

void smarteye::comm::receiveControlRes(smarteye_common::controlMsg result)
{
    controlResult.roll=result.roll;
    controlResult.pitch=result.pitch;
    controlResult.yaw=result.yaw;
    controlResult.thrust=result.thrust;
}











