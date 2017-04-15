#ifndef _FLIGHTCTRL_H
#define _FLIGHTCTRL_H


#include <stdio.h>
#include <smarteye/core/core.hpp>



typedef struct _TDFilterStruct
{
    F32 V1;
    F32 V2;
    F32 Aim;
    F32 Tr;
    F32 Cycle;
}TDFilterStruct;

typedef struct _HeiToSpeCtrlParaStruct
{
    //耦合到油门的控制参数
    F32 ErrTh;
    F32 PitchOpenTh; //为负值
    F32 KP;
    F32 Limit;//限位
    U8 IsOpen;//是否耦合到油门
}HeiToSpeCtrlParaStruct;

typedef struct _HorGuiCtrlParaStruct
{
    F32 ErrTh[3][3];
    F32 KP[3][3];
    F32 KPDiv[3][3];//增益分段
    F32 KP1[3][3];
    F32 KP2[3][3];
    F32 KD[3][3];
    F32 KI[3][3];
    F32 KITh[3][3];//KITh>ErrTh，积分能够开启，否则不能开启，见算法
    F32 ILimit[3][3];
    F32 Limit[3][3];

    U8 IsSpeDiv;
    F32 SpeLowTh;		//低速门限     // low velocity
    F32 SpeHiTh;			//高速门限  //high velocity
    U8 IsAileronTurn;
    U8 IsUseAirspeed;	//使用空速或者地速
    U8 RudderOpenTh;
}HorGuiCtrlParaStruct;

typedef struct _HeiPIDCtrlParaStruct
{

    F32 ErrTh[3];
    F32 KP[3];
    F32 KPDiv[3];//增益分段
    F32 KP1[3];
    F32 KP2[3];
    F32 KD[3];
    F32 KI[3];
    F32 KITh[3];//KITh>ErrTh，积分能够开启，否则不能开启，见算法
    F32 ILimit[3];
    F32 Limit[3];
    TDFilterStruct TD;
}HeiPIDCtrlParaStruct;

typedef struct _ThrCtrlParaStruct
{
    F32 ErrTh;
    F32 KP;
    F32 KPDiv;//增益分段
    F32 KP1;
    F32 KP2;
    F32 KD;
    F32 KI;
    F32 KITh;//KITh>ErrTh，积分能够开启，否则不能开启，见算法
    F32 ILimit;
    F32 Limit;
    F32 Ref;
}ThrCtrlParaStruct;


typedef struct _RollToPitchParaStruct
{
    F32 OpenTh[3];
    F32 KP[3];
    F32 Limit[3];
}RollToPitchParaStruct;

typedef struct 
{
    U8 ID;				//航点号

    U8 Property;			//航点属性

    U8 EnterActClass;	//动作组
    U8 ExitActClass;	//动作组
    U8 JumpNo;			//跳转航线号
    U8 JumpID;			//跳转航点号

    F32 Heading;		//目标航向（固定翼无用）
    F32 Delay;			//绕该点盘旋的时间
    F32 Speed;			//目标速度,单位为m/s
    F32 Radius;			//切换半径/盘旋半径
    F64 Lon;			//经度
    F64 Lat;			//纬度
    F32 Height;			//海拔
}WayPointStruct;

//航线飞行采用两个点，只不过这两个点是滑动更新的，从航线存储结构中不断读出航点放入到该数据结构中
typedef struct 
{
    WayPointStruct WayPoint[3];	//总是由WayPoint[0]-->WayPoint[1]，同时引入WayPoint[2]，方便以后算法的升级
    U8 No;						//航线号
    U8 Num;						//航点总数

    U16 Cnt;						//计数器，更新航线内容后加1
}AirlineStruct;

///////////////////////////////////////////////////////////////////位置，速度，姿态遥控
typedef struct 
{
    F32 Lon,Lat,Height;
    U16 Cnt;						//计数器，更新内容后加1//?是否有用
}PosStruct;

typedef struct 
{
    F32 VX,VY,VZ;
    U16 Cnt;						//计数器，更新内容后加1//?是否有用
}VelStruct;

typedef struct 
{
    F32 Pitch,Roll,Yaw;
    U16 Cnt;						//计数器，更新内容后加1//?是否有用
}AttStruct;

typedef struct 
{
    F32 AX,AY,AZ;
    U16 Cnt;						//计数器，更新内容后加1//?是否有用
}AccStruct;

typedef struct 
{
    F32 P,Q,R;
    U16 Cnt;						//计数器，更新内容后加1//?是否有用
}AngVelStruct;

typedef struct 
{
    F32 PA,QA,RA;
    U16 Cnt;						//计数器，更新内容后加1//?是否有用
}AngAccStruct;


//制导控制量结构体
typedef struct
{
    PosStruct Pos;
    VelStruct Vel;
    AttStruct Att;
    AccStruct Acc;
    AngVelStruct AngVel;
    AngAccStruct AngAcc;
    U8 Overshoot;
}GuiCtrlStruct;

///////////////////////////////////////////////////////////////////手动控制舵量
//控制量结构体,共4*7=28字节,不进行字节对齐
typedef struct 
{
    F32 Pitch;
    F32 Roll;
    F32 Yaw;
    F32 Throttle;
}__attribute__((packed)) CtrlStruct;

///////////////////////////////////////////////////////////////////制导环节的状态，侧偏距，航线角等
//制导状态结构体
typedef struct 
{
    F32 TargetCourse;	//目标航线角:LastPoint--->NextPoint线段与正北的夹角
    F32 CurrCourse;		//当前航线角:当前位置--->NextPoint线段与正北的夹角
    F32 LineLength;		//航线长度
    F32 Distance;    	//当前点到目标航点距离,以米为单位.
    F32 LineErr;		//侧偏距
    F32 dLineErr;		//侧偏距微分
    F32 LineDistance;	//沿着航线方向到目标点的距离(投影)
    F32 HeightErr;		//高度误差
    U8 SpeNum;			//当前速度段
    U8 Msg;				//信息
}GuiStateStruct;


////////////////////////////////////////////////////////////////////飞控算法输出
//飞控核心输出结构体
typedef struct
{
    GuiStateStruct GuiState;//制导状态输出
    GuiCtrlStruct GuiCtrl;//制导数据输出
    CtrlStruct FCACtrl;//飞控算法输出

    U8 WayPointSwitch;//指示是否进行航点切换
}FlightCtrlResultStruct;



typedef struct 
{
    //以下数据存在内存对齐关系请勿随意更改
    F32 VelN,VelE,VelD;	 //无变换，单位为m/s
    F32 Pitch,Roll,Yaw;  //姿态数据，单位为°
    F32 VelX,VelY,VelZ;	   //本体速度，x为机头方向
    F32 AirHeight,Airspeed,CaliAirspeed;
    F32 dYaw;
    F64 Height;	//直接可使用的经纬度值

}FlightStateStruct;

//直接航向，速度和高度控制模式的设定量
//添加：zhangjy
//日期：2012.7.25
//时间：15:11
//2013.5.17 加入 __packed 
typedef struct
{
    F32 Gd_Ang;	 //导航方向，正北坐标系
    F32 Gd_Height;//导航高度
    F32 Gd_V; 	   //导航速度，如果超出安全门限，则以上下限飞行
}__attribute__((packed)) DirectGuidInf;

#endif
