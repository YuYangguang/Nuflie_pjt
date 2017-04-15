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



