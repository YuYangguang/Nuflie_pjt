#include <control.h>
//#include "plan.h"

using namespace smarteye;

int main(int argc, char **argv)
{
    //Plan controlPlan;
    ROS_INFO("start control_node process");
    ros::Time::init();
    control this_controll(argc,argv,"control_node");
    ros::spin();
    return 0;
}


//#include <ros/ros.h>
//#include <geometry_msgs/Point.h>
//#include <geometry_msgs/PoseStamped.h>
//#include <geometry_msgs/TwistStamped.h>
//#include <geometry_msgs/TransformStamped.h>
//#include <array>
//#include <angles/angles.h>
//#include <eigen_conversions/eigen_msg.h>
//#include <std_msgs/Float64.h>
//#include <std_msgs/Header.h>
//#include <keyboard/Key.h>
//#include <math.h>
//#include <mavros_msgs/State.h>
//#include <tf/tf.h>
//#include <tf/transform_datatypes.h>
//#include <std_msgs/Float64.h>

//geometry_msgs::PoseStamped oriPos,ps,currentPos,nextPos,initCirclePos;
//geometry_msgs::PoseStamped uavCurrentLocalPose,uavCurrentViconPose;
//geometry_msgs::TransformStamped vicon_currentPos;
//geometry_msgs::PoseStamped vicon_mocaptf;
//ros::Publisher setPositionPublisher;
//ros::Publisher currentViconPositionPublisher;
//ros::Publisher angleCountPublisher;
////ros::Publisher mocaptfPublisher;
//float value;
//bool hasSet;
//Eigen::Vector3d current;
//int angle;
//double angleStep;
//bool isFlyCircle,hasInitFlyCircle;
//bool isFlyInitPos, hasFlyInitPos;
//double radius;

//bool flag_offboard_mode = false;
//bool isSetNextPose;

//double tmpRoll,tmpPitch,tmpYaw;
//double angleCount[3]={0,0,0};
//double initAngle;

//Eigen::Vector3d circle_shape(int angle){
//    double r = 5.0f;  // 5 meters radius

//    return Eigen::Vector3d(r * cos(angles::from_degrees(angle)),
//        r * sin(angles::from_degrees(angle)),
//        1.0f);
//  }

//// 判断飞行模式
//mavros_msgs::State current_state;
//void stateReceived(const mavros_msgs::State::ConstPtr& msg)
//{
//    static int flag = 0;
//    current_state = *msg;

//    if (current_state.mode == "OFFBOARD")
//    {
//        if(flag == 0)
//        {
//            ROS_INFO_STREAM("Offboard Mode");
//            flag = 1;
//        }

//        flag_offboard_mode = true;
//    }
//    else
//    {
//        flag_offboard_mode = false;
//        flag = 0;
//    }
//}

//void angleCountReceived(std_msgs::Float64 count)
//{
//    angleCount[1]=count.data;
//}

//void angleCountReceived2(std_msgs::Float64 count)
//{
//    angleCount[2]=count.data;
//}

//// vicon位姿数据转换为mavros位姿数据
//void PosetopicTransport(geometry_msgs::TransformStamped& viconPos, geometry_msgs::PoseStamped& rosPos)
//{
//    rosPos.pose.position.x = viconPos.transform.translation.x;
//    rosPos.pose.position.y = viconPos.transform.translation.y;
//    rosPos.pose.position.z = viconPos.transform.translation.z;
//    rosPos.pose.orientation.x = viconPos.transform.rotation.x;
//    rosPos.pose.orientation.y = viconPos.transform.rotation.y;
//    rosPos.pose.orientation.z = viconPos.transform.rotation.z;
//    rosPos.pose.orientation.w = viconPos.transform.rotation.w;
//}
//void flyInitPos(double r)
//{

//  currentPos=uavCurrentLocalPose;

//  if(!hasFlyInitPos){
//    nextPos=uavCurrentLocalPose;
//    nextPos.pose.position.y = -r*cos(angles::from_degrees(initAngle));
//    nextPos.pose.position.x = r*sin(angles::from_degrees(initAngle));
//    nextPos.pose.position.z=1.2;
//    ps.pose = nextPos.pose;
//    ps.header.stamp = ros::Time::now();
//    setPositionPublisher.publish(ps);
//  }

//  // judge whether is reached
//  bool isReached = false;
//  double distance = sqrt((currentPos.pose.position.x - nextPos.pose.position.x)*(currentPos.pose.position.x - nextPos.pose.position.x)  +
//                       (currentPos.pose.position.y - nextPos.pose.position.y)*(currentPos.pose.position.y - nextPos.pose.position.y));
//  double threshold = 0.2;
//  if (distance < threshold)
//  {
//    isReached = true;
//  }

//  if (isReached)
//  {
//     hasFlyInitPos=true;
//     ROS_INFO("Position has been inited");
//     ps.header.stamp=ros::Time::now();
//     setPositionPublisher.publish(ps);
//  } else {
//    ps.header.stamp = ros::Time::now();
//    setPositionPublisher.publish(ps);
//   // ROS_INFO_STREAM("next angle:" << angle);
//  }
//}
//void flyCircleWithRadius(double r)
//{
//  std_msgs::Float64 angleCountSend;
//  currentPos=uavCurrentLocalPose;
//  initCirclePos=uavCurrentLocalPose;
//  initCirclePos.pose.position.y =-r;
//  initCirclePos.pose.position.x =0;
//  if(!hasInitFlyCircle){

//    angle = initAngle;
//    // nextPos.pose.position.x = r * cos(angles::from_degrees(angle)) - r +
//    //                           initCirclePos.pose.position.x;
//    // nextPos.pose.position.y = r * sin(angles::from_degrees(angle)) + initCirclePos.pose.position.y;
//    nextPos = uavCurrentLocalPose;
//    nextPos.pose.position.y = -r*cos(angles::from_degrees(initAngle));
//    nextPos.pose.position.x = r*sin(angles::from_degrees(initAngle));
//    nextPos.pose.position.z=1.2;
//    ps.pose = nextPos.pose;
//    ps.header.stamp = ros::Time::now();
//    setPositionPublisher.publish(ps);
//    //ROS_INFO_STREAM("next angle:" << angle);
//    hasInitFlyCircle = true;
//  }

//  // judge whether is reached
//  bool isReached = false;
//  double distance = sqrt((currentPos.pose.position.x - nextPos.pose.position.x)*(currentPos.pose.position.x - nextPos.pose.position.x)  +
//                       (currentPos.pose.position.y - nextPos.pose.position.y)*(currentPos.pose.position.y - nextPos.pose.position.y));
//  double threshold = 0.2;
//  if (distance < threshold)
//  {
//    isReached = true;
//  }
//  double deltaAngleStepCount=0;
//  if (isReached)
//  {
//   angleCountSend.data= angleCount[0];
//   angleCountPublisher.publish(angleCountSend);
//   ROS_INFO("there is reached");
////    if((1+angleCount[1]+angleCount[2]-2*angleCount[0])<1)
////    {
////      deltaAngleStepCount=1;
////    }
////    else
////    {
////      deltaAngleStepCount=1+angleCount[1]+angleCount[2]-2*angleCount[0];
////    }
////    angle = angle + angleStep*deltaAngleStepCount;   //consensus protocol
////   angleCount[0]=angleCount[0]+deltaAngleStepCount;
//angleCount[0]=angleCount[0]+1;

//    if(angle > 360) angle = angle - 360;
//    nextPos = initCirclePos;
//    // nextPos.pose.position.x = r * cos(angles::from_degrees(angle)) - r +
//    //                           initCirclePos.pose.position.x;
//    // nextPos.pose.position.y = r * sin(angles::from_degrees(angle)) + initCirclePos.pose.position.y;
//    nextPos.pose.position.y = r - r * cos(angles::from_degrees(angle)) +
//                              initCirclePos.pose.position.y;
//    nextPos.pose.position.x = r * sin(angles::from_degrees(angle)) + initCirclePos.pose.position.x;
//    nextPos.pose.position.z=1.2;
//    ps.pose = nextPos.pose;
//    ps.header.stamp = ros::Time::now();
//    setPositionPublisher.publish(ps);

//  } else {
//    ps.header.stamp = ros::Time::now();
//    setPositionPublisher.publish(ps);
//   // ROS_INFO_STREAM("next angle:" << angle);
//  }
//}

//void flyHeartWithRadius(double r)
//{
//  if(!hasInitFlyCircle){
//    initCirclePos = currentPos;
//    nextPos = initCirclePos;
//    angle = angle + angleStep;
//    nextPos.pose.position.x = r *(2*cos(angles::from_degrees(angle)) - cos(angles::from_degrees(2*angle)))  - r +
//                              initCirclePos.pose.position.x;
//    nextPos.pose.position.y = r *(2*sin(angles::from_degrees(angle)) - sin(angles::from_degrees(2*angle)))  + initCirclePos.pose.position.y;
//    ps.pose = nextPos.pose;
//    ps.header.stamp = ros::Time::now();
//    setPositionPublisher.publish(ps);
//    //ROS_INFO_STREAM("next angle:" << angle);
//    hasInitFlyCircle = true;
//  }

//  bool isReached = false;
//  double distance = sqrt((currentPos.pose.position.x - nextPos.pose.position.x)*(currentPos.pose.position.x - nextPos.pose.position.x)  +
//                       (currentPos.pose.position.y - nextPos.pose.position.y)*(currentPos.pose.position.y - nextPos.pose.position.y));
//  double threshold = 0.2;
//  if (distance < threshold)
//  {
//    isReached = true;
//  }

//  if (isReached)
//  {
//    // send next pos
//    angle = angle + angleStep;
//    if(angle > 360) angle = angle - 360;
//    nextPos = initCirclePos;
//    nextPos.pose.position.x = r *(2*cos(angles::from_degrees(angle)) - cos(angles::from_degrees(2*angle)))  - r +
//                              initCirclePos.pose.position.x;
//    nextPos.pose.position.y = r *(2*sin(angles::from_degrees(angle)) - sin(angles::from_degrees(2*angle)))  + initCirclePos.pose.position.y;
//    ps.pose = nextPos.pose;
//    ps.header.stamp = ros::Time::now();
//    setPositionPublisher.publish(ps);

//  } else {
//    ps.header.stamp = ros::Time::now();
//    setPositionPublisher.publish(ps);
//    ROS_INFO_STREAM("next angle:" << angle);
//  }
//}

//void flyPeachHeartWithRadius(double r)
//{
//  if(!hasInitFlyCircle){
//    initCirclePos = currentPos;
//    nextPos = initCirclePos;
//    angle = angle + angleStep;
//    nextPos.pose.position.x = 16 * sin(angles::from_degrees(angle))*
//                                 sin(angles::from_degrees(angle))*
//                                 sin(angles::from_degrees(angle))* - r +
//                              initCirclePos.pose.position.x;
//    nextPos.pose.position.y = 13*cos(angles::from_degrees(angle))-
//                               5*cos(angles::from_degrees(2*angle))-
//                               2*cos(angles::from_degrees(3*angle)) -
//                               cos(angles::from_degrees(4*angle))+ initCirclePos.pose.position.y;
//    ps.pose = nextPos.pose;
//    ps.header.stamp = ros::Time::now();
//    setPositionPublisher.publish(ps);
//    //ROS_INFO_STREAM("next angle:" << angle);
//    hasInitFlyCircle = true;
//  }

//  bool isReached = false;
//  double distance = sqrt((currentPos.pose.position.x - nextPos.pose.position.x)*(currentPos.pose.position.x - nextPos.pose.position.x)  +
//                       (currentPos.pose.position.y - nextPos.pose.position.y)*(currentPos.pose.position.y - nextPos.pose.position.y));
//  double threshold = 0.1;
//  if (distance < threshold)
//  {
//    isReached = true;
//  }

//  if (isReached)
//  {
//    // send next pos
//    angle = angle + angleStep;
//    if(angle > 360) angle = angle - 360;
//    nextPos = initCirclePos;
//    nextPos.pose.position.x = 16 * sin(angles::from_degrees(angle))*
//                                 sin(angles::from_degrees(angle))*
//                                 sin(angles::from_degrees(angle))* - r +
//                              initCirclePos.pose.position.x;
//    nextPos.pose.position.y = 13*cos(angles::from_degrees(angle))-
//                               5*cos(angles::from_degrees(2*angle))-
//                               2*cos(angles::from_degrees(3*angle)) -
//                               cos(angles::from_degrees(4*angle))+ initCirclePos.pose.position.y;
//    ps.pose = nextPos.pose;
//    ps.header.stamp = ros::Time::now();
//    setPositionPublisher.publish(ps);

//  } else {
//    ps.header.stamp = ros::Time::now();
//    setPositionPublisher.publish(ps);
//    ROS_INFO_STREAM("next angle:" << angle);
//  }
//}

//void localPositionReceived(const geometry_msgs::PoseStampedConstPtr& msg)
//{
//      //uavPose = *msg;
//    uavCurrentLocalPose.pose.position.x = msg->pose.position.x;
//    uavCurrentLocalPose.pose.position.y = msg->pose.position.y;
//    uavCurrentLocalPose.pose.position.z = msg->pose.position.z;
//    uavCurrentLocalPose.pose.orientation.x = msg->pose.orientation.x;
//    uavCurrentLocalPose.pose.orientation.y = msg->pose.orientation.y;
//    uavCurrentLocalPose.pose.orientation.z = msg->pose.orientation.z;
//    uavCurrentLocalPose.pose.orientation.w = msg->pose.orientation.w;

//    tf::Quaternion quat;
//    tf::quaternionMsgToTF(uavCurrentLocalPose.pose.orientation, quat);
//    tf::Matrix3x3(quat).getRPY(tmpRoll, tmpPitch, tmpYaw);

//   // ROS_INFO("Roll,Pitch,Yaw:[%0.3f,%0.3f,%0.3f]",tmpRoll/3.14*180,tmpPitch/3.14*180,tmpYaw/3.14*180);

//    if (!isSetNextPose)
//    {
//        ps = uavCurrentLocalPose;
//        //ps.pose.position.z = 0.14;
//        hasSet = true;
//    }

//}

//// vicon 数据接受处理
//void viconPositionReceived(const geometry_msgs::TransformStampedConstPtr& vicon_msg)
//{
//    vicon_currentPos = *vicon_msg;
//    PosetopicTransport(vicon_currentPos, uavCurrentViconPose);
//}

//void sendCommand(const keyboard::Key &key)
//{

//  switch(key.code)
//  {
//      case 'i':
//      {
//        // Forward
//        //ps.pose.position.y -= value;
//        ps.pose.position.x = ps.pose.position.x + value * cos(tmpYaw);
//        ps.pose.position.y = ps.pose.position.y + value * sin(tmpYaw);
//        ROS_INFO_STREAM("Forward: " << value);
//        break;
//      }
//      case 'k':
//      {
//        // Backward
//       // ps.pose.position.y += value;
//        ps.pose.position.x = ps.pose.position.x - value * cos(tmpYaw);
//        ps.pose.position.y = ps.pose.position.y - value * sin(tmpYaw);
//        ROS_INFO_STREAM("Backward: " << value);
//        break;
//      }
//      case 'j':
//      {
//        // left
//        //ps.pose.position.x += value;
//        ps.pose.position.x = ps.pose.position.x - value * sin(tmpYaw);
//        ps.pose.position.y = ps.pose.position.y + value * cos(tmpYaw);
//        ROS_INFO_STREAM("Left: " << value);
//        break;
//      }
//      case 'l':
//      {
//        // right
//        //ps.pose.position.x -= value;
//        ps.pose.position.x = ps.pose.position.x + value * sin(tmpYaw);
//        ps.pose.position.y = ps.pose.position.y - value * cos(tmpYaw);
//        ROS_INFO_STREAM("Right: " << value);
//        break;
//      }

//      case '2':
//      {
//        // Up
//        ps.pose.position.z += value;
//        ROS_INFO_STREAM("Up: " << value);
//        break;
//      }
//      case '6':
//      {
//        // Down
//        ps.pose.position.z -= value;
//        ROS_INFO_STREAM("Down: "<< value);
//        break;
//      }
//      case 'a':
//      {
//        // turn left
//        tmpYaw += 0.05;
//        if (tmpYaw > 3.14159)
//        {
//          tmpYaw = tmpYaw - 2*3.14159;
//        }
//        ps.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,tmpYaw);
//ROS_INFO("Yaw:[%0.3f]",tmpYaw/3.14159*180);
//        break;
//      }
//      case 'd':
//      {
//        // turn right
//        tmpYaw -= 0.05;
//        if (tmpYaw < -3.14159)
//        {
//          tmpYaw = tmpYaw + 2*3.14159;
//        }
//        ps.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,tmpYaw);
//ROS_INFO("Yaw:[%0.3f]",tmpYaw/3.14159*180);
//        break;
//      }
//      case 'x':
//      {
//        // turn to origin position
//        ps = oriPos;
//        ROS_INFO_STREAM("Turn to original position");
//        break;
//      }
//      case 'y':
//      {
//        // fly circle
//        isFlyCircle = true;
//        isFlyInitPos=false;
//        hasFlyInitPos=false;
//        ROS_INFO_STREAM("Fly Circle Mode");
//        break;
//      }
//      case 'h':
//      {
//        // turn to manual mode
//        isFlyCircle = false;
//        hasInitFlyCircle=false;
//        hasInitFlyCircle = false;
//        isFlyInitPos=false;
//        angleCount[0]=0;
//        angleCount[1]=0;
//        angleCount[2]=0;
//        ROS_INFO_STREAM("Manual Mode");
//        break;
//      }

//      case 'b':
//      {
//        //angleStep++;
//        //ROS_INFO_STREAM("angle step:" << angleStep);

//        isSetNextPose = false;
//        break;
//      }
//      case 'n':
//      {
//        // angleStep--;
//        // if (angleStep < 1)
//        // {
//        //   angleStep = 1;
//        // }
//        // ROS_INFO_STREAM("angle step:" << angleStep);

//        isSetNextPose = true;
//        break;
//      }

//       case 'z':         //Init position
//      {
//         isFlyCircle = false;
//         hasInitFlyCircle = false;
//         isFlyInitPos=true;

//         ROS_INFO_STREAM("Manual Mode");
//      break;
//      }

//      default:
//      {

//      }
//  }
//}


//int main(int argc, char **argv)
//{
//    ros::init(argc, argv, "px4_offboard_position_control_node");
//  ros::NodeHandle nodeHandle("~");
//  ROS_INFO_STREAM("start");
//   std::string viconName;
//   nodeHandle.getParam("viconName", viconName);
//   nodeHandle.getParam("initAngle",initAngle);
//  setPositionPublisher = nodeHandle.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local",10);
//  currentViconPositionPublisher = nodeHandle.advertise<geometry_msgs::PoseStamped>("/mavros/mocap/pose",10);
//  angleCountPublisher=nodeHandle.advertise<std_msgs::Float64>("angleCount",10);
//  //mocaptfPublisher = nodeHandle.advertise<geometry_msgs::TransformStamped>("/mavros/mocap/tf",10);
//  ros::Subscriber localPositionSubsciber = nodeHandle.subscribe("/mavros/local_position/pose", 10, localPositionReceived);
//  ros::Subscriber viconPositionSubsciber = nodeHandle.subscribe(viconName, 10, viconPositionReceived);
//  ros::Subscriber commandSubscriber = nodeHandle.subscribe("/comm/keyboard",1,sendCommand);
//  ros::Subscriber stateSubscriber = nodeHandle.subscribe("/mavros/state", 10, stateReceived);
//  ros::Subscriber angleCountSubscirber=nodeHandle.subscribe("/nuflie01/angleCount",10,angleCountReceived);
//  ros::Subscriber angleCountSubscirber2=nodeHandle.subscribe("/nuflie04/angleCount",10,angleCountReceived2);
//  value = 0.2f;
//  hasSet = false;

//  // fly circle parameters
//  isFlyCircle = false;
//  hasInitFlyCircle = false;
//  isFlyInitPos=false;
//  hasFlyInitPos=false;
//  angle = 0.0;
//  radius = 2;
//  angleStep = 5.1;

//  isSetNextPose = false;

//  tmpRoll = 0.0;
//  tmpPitch = 0.0;
//  tmpYaw= 0.0;

//  ros::Rate loopRate(50.0);

//  while(ros::ok())
//  {
//    if(hasSet)
//    {
//        ps.header.seq++;

//        if(isFlyCircle)
//        {
//          flyCircleWithRadius(radius);

//          //vicon_mocaptf.header.stamp = ros::Time::now();
//          //vicon_mocaptf.header.frame_id = "/world";
//          //mocaptfPublisher.publish(vicon_mocaptf);
//        }
//        else if(isFlyInitPos)
//        {
//            flyInitPos(radius);
//        }
//        else
//        {

//          ps.header.stamp = ros::Time::now();
//          setPositionPublisher.publish(ps);

//          //flyHeartWithRadius(radius);
//          //flyPeachHeartWithRadius(radius);
//        }

//        uavCurrentViconPose.header.stamp = ros::Time::now();
//        uavCurrentViconPose.header.frame_id = "/world";
//        currentViconPositionPublisher.publish(uavCurrentViconPose);
//    }

//    ros::spinOnce();
//    loopRate.sleep();
//  }
//}
