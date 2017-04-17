#include <control.h>

using namespace smarteye;
control::control(int argc, char **argv, const char *name)
{

    ros::init(argc,argv,name);
    nh = boost::make_shared<ros::NodeHandle>();
    velPub=nh->advertise<geometry_msgs::TwistStamped>("/mavros/local_position/velocity",10);
    flightSateSub=nh->subscribe("/comm/flight_state",10,&control::receiveFlightSate,this);
    controlResultPub = nh->advertise<smarteye_common::controlMsg>("/control/control_result",10);
    /** 30ms触发一次的定时器 */
    controlUpdateTimer = nh->createTimer(ros::Duration(0.015),&control::update,this);
    commandSubscriber = nh->subscribe("/keyboard/keydown",1,&control::receiveCommand,this);
    directGuide.Gd_Ang=0;
    directGuide.Gd_Height=60;
    directGuide.Gd_V=5;
}

control::~control()
{

}

void control::update(const ros::TimerEvent &event)
{

    velPub.publish(velCommand);
    controlPlan.DirectGuide(&directGuide,&controlResult);
    controlResMSg.pitch=controlResult.Pitch;
    controlResMSg.roll=controlResult.Roll;
    controlResMSg.yaw=controlResult.Yaw;
    controlResMSg.thrust=controlResult.Throttle;
    controlResultPub.publish(controlResMSg);
}

void control::receiveFlightSate(smarteye_common::flightStateMsg flightMsg)
{
    controlPlan.FlightState.Height=-flightMsg.z;
    controlPlan.FlightState.Roll=flightMsg.roll;
    controlPlan.FlightState.Pitch=flightMsg.pitch;
    controlPlan.FlightState.Yaw=flightMsg.yaw;
    controlPlan.FlightState.dYaw=flightMsg.yawspeed;
    controlPlan.FlightState.VelN=flightMsg.vx;
    controlPlan.FlightState.VelE=flightMsg.vy;
    controlPlan.FlightState.VelD=flightMsg.vz;
}

void control::receiveCommand(keyboard::Key key)
{
    switch(key.code)
    {



    case '1':    //increase height
    {
        directGuide.Gd_Height++;
        ROS_INFO("height is %f",directGuide.Gd_Height);

    }
    case '2':    //increase height
    {
        directGuide.Gd_Height--;
        ROS_INFO("height is %f",directGuide.Gd_Height);

    }
    case '3':    //increase angle
    {
        directGuide.Gd_Ang++;
        ROS_INFO("angle is %f",directGuide.Gd_Ang);

    }
    case '4':    //increase angle
    {
        directGuide.Gd_Ang--;
        ROS_INFO("angle is %f",directGuide.Gd_Ang);

    }
    case '5':    //increase speed
    {
        directGuide.Gd_V++;
        ROS_INFO("speed is %f",directGuide.Gd_V);

    }
    case '6':    //increase speed
    {
        directGuide.Gd_V--;
        ROS_INFO("speed is %f",directGuide.Gd_V);

    }
    default:
    {

        break;
    }
    }
}



