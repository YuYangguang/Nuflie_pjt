#ifndef _PLAN_H_
#define _PLAN_H_
#include "smarteye/core/core.hpp"
#include "FlightCtrl.h"
#include "smarteye/core/MathCal.h"

#define CRU_SPE   	1     //巡航速度序号
#define YAW2ROLL 	1    //航向到滚转


namespace smarteye{

class Plan
{
public:
    Plan();
    ~Plan();
    FlightStateStruct FlightState;
    void DirectGuide(DirectGuidInf *dGInf, CtrlStruct *FlightCtrlResult);
    void getFlightState(void);

private:
    U8  Vn; //速度段序号，PID参数分段用
    F32 HeiToPitchEi;
    F32 TargetHeight;
    U8 IsHeiUpdate;
    HeiToSpeCtrlParaStruct HeiToSpeCtrlPara;
    HorGuiCtrlParaStruct HorGuiCtrlPara;
    HeiPIDCtrlParaStruct HeiPIDCtrlPara;
    ThrCtrlParaStruct ThrCtrlPara;
    RollToPitchParaStruct RollToPitchPara;
    void TDFilter(TDFilterStruct *Obj);
    void HorGuiCtrlParaInit(void);
    F32 HeadingToRoll(F32 aYaw, F32 cYaw,F32 dYaw); //目标航向－》目标滚转－回路4
    F32 HeightToPitch(F32 aHei,F32 cHei,F32 cdHei,U8 Mode, U8 IsHeiUpdate);
    F32 RollToPitch(F32 aRoll);
    F32 SpeedToThrottle(F32 aSpe,F32 cSpe);
    F32 HeightToSpeed(F32 aHei,F32 cHei,F32 cPitch);

};


}

#endif

