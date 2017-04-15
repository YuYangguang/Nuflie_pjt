#include <plan.h>

using namespace smarteye;


Plan::Plan()
{
    Vn = CRU_SPE;
    TargetHeight=0;
    IsHeiUpdate = FALSE;
    HeiToPitchEi=0;
    HorGuiCtrlParaInit();



}

Plan::~Plan()
{

}

void Plan::TDFilter(TDFilterStruct *Obj)
{
    F32 fh;
    fh = Fhan((Obj->V1 - Obj->Aim), Obj->V2, Obj->Tr, Obj->Cycle);
    Obj->V1 = Obj->V1 + Obj->Cycle * Obj->V2;
    Obj->V2 = Obj->V2 + Obj->Cycle * fh;

}

void Plan::HorGuiCtrlParaInit()
{
    U8 i,j;
    ///////////////////////////////////////////////侧偏距到航向
    i=0;
    for(j=0;j<3;j++)
    {
        HorGuiCtrlPara.KP[i][j] = 0.5;
        HorGuiCtrlPara.KP1[i][j] = 0.8;
        HorGuiCtrlPara.KP2[i][j] = 1.5;
        HorGuiCtrlPara.KD[i][j] = 0.2;
        HorGuiCtrlPara.KI[i][j] = 0.01;
        HorGuiCtrlPara.KITh[i][j] = 5;
        HorGuiCtrlPara.Limit[i][j] = 30;
        HorGuiCtrlPara.ErrTh[i][j] = 2;
        HorGuiCtrlPara.KPDiv[i][j] = 20;
        HorGuiCtrlPara.ILimit[i][j] = 5;
    }
    ///////////////////////////////////////////////航向到滚转
    i=1;
    for(j=0;j<3;j++)
    {
        HorGuiCtrlPara.KP[i][j] = 1;
        HorGuiCtrlPara.KP1[i][j] = 1;
        HorGuiCtrlPara.KP2[i][j] = 1.1;
        HorGuiCtrlPara.KD[i][j] = 0.1;
        HorGuiCtrlPara.KI[i][j] = 0.01;
        HorGuiCtrlPara.KITh[i][j] = 5;
        HorGuiCtrlPara.Limit[i][j] = 30;
        HorGuiCtrlPara.ErrTh[i][j] = 1;
        HorGuiCtrlPara.KPDiv[i][j] = 20;
        HorGuiCtrlPara.ILimit[i][j] = 5;
    }
    ///////////////////////////////////////////////盘旋到航向
    i=2;
    for(j=0;j<3;j++)
    {
        HorGuiCtrlPara.KP[i][j] = 1;
        HorGuiCtrlPara.KP1[i][j] = 1;
        HorGuiCtrlPara.KP2[i][j] = 1;
        HorGuiCtrlPara.KD[i][j] = 0.1;
        HorGuiCtrlPara.KI[i][j] = 0.01;
        HorGuiCtrlPara.KITh[i][j] = 10;
        HorGuiCtrlPara.Limit[i][j] = 30;
        HorGuiCtrlPara.ErrTh[i][j] = 3;
        HorGuiCtrlPara.KPDiv[i][j] = 20;
        HorGuiCtrlPara.ILimit[i][j] = 5;
    }
    HorGuiCtrlPara.IsUseAirspeed = FALSE;
    HorGuiCtrlPara.RudderOpenTh = 5;
    ///////////////////////////////////////////////速度分段
    HorGuiCtrlPara.IsSpeDiv = FALSE;
    HorGuiCtrlPara.SpeHiTh = 50;
    HorGuiCtrlPara.SpeLowTh = 20;
    ///////////////////////////////////////////////副翼转弯
    HorGuiCtrlPara.IsAileronTurn = TRUE;
    ///////////////////////////////////////////////速度到油门
    ThrCtrlPara.KP = 1;
    ThrCtrlPara.KD = 0.005;
    ThrCtrlPara.KI = 0.001;
    ThrCtrlPara.KITh = 5;
    ThrCtrlPara.ErrTh = 3;
    ThrCtrlPara.KPDiv = 8;
    ThrCtrlPara.KP1 = 0.04;
    ThrCtrlPara.KP2 = 0.05;
    ThrCtrlPara.Limit = -0.8;
    ThrCtrlPara.ILimit  = 0.3;
    ThrCtrlPara.Ref = -0.2;
    ////////////////////////////////////////////////高度到速度
    HeiToSpeCtrlPara.ErrTh = 10;
    HeiToSpeCtrlPara.IsOpen = TRUE;
    HeiToSpeCtrlPara.KP  = 0.5;
    HeiToSpeCtrlPara.Limit = 10;
    HeiToSpeCtrlPara.PitchOpenTh = 5;
    ////////////////////////////////////////////////高度到俯仰

    for(i=0;i<3;i++)
    {
        HeiPIDCtrlPara.KP[i] = 1.4;
        HeiPIDCtrlPara.KP1[i] = 0.25;
        HeiPIDCtrlPara.KP2[i] = 0.3;
        HeiPIDCtrlPara.KD[i] = 0.03;
        HeiPIDCtrlPara.KI[i] = 0.002;
        HeiPIDCtrlPara.KITh[i] = 10;
        HeiPIDCtrlPara.Limit[i] = 10;
        HeiPIDCtrlPara.ErrTh[i] = 2;
        HeiPIDCtrlPara.KPDiv[i] = 20;
        HeiPIDCtrlPara.ILimit[i] = 5;

        RollToPitchPara.KP[i] = 0.1;
        RollToPitchPara.OpenTh[i] = 10;
        RollToPitchPara.Limit[i] = 2;
    }
    HeiPIDCtrlPara.TD.Tr = 200.0;
    HeiPIDCtrlPara.TD.Cycle = 0.02;
}


F32 Plan::HeadingToRoll(F32 aYaw, F32 cYaw, F32 dYaw)
{

    F32 error, temp;
    //S32 tempS32;
    static F32 ei;
    static F32 ULast;
    F32 KPDiv;
    error = aYaw - cYaw;
    if(error > 180.0)
        error =-(360.0-error);
    if(error<-180.0)
        error=360.0+error;
    error = ValueLimit2(error);
    dYaw = ValueLimit2(dYaw);
    if( fabs(error) + fabs(dYaw) < HorGuiCtrlPara.ErrTh[YAW2ROLL][Vn])
    {
        return ULast;
    }
    //	if(fabs(error) < HorGuiCtrlPara.ErrTh[YAW2ROLL][Vn])
    //		return 0;
    //计算比例的值
    KPDiv = HorGuiCtrlPara.KPDiv[YAW2ROLL][Vn];
    if (fabs(error)>KPDiv)
    {
        //如果在第二段内
        if (error>0)
            temp = HorGuiCtrlPara.KP1[YAW2ROLL][Vn]*KPDiv + HorGuiCtrlPara.KP2[YAW2ROLL][Vn]*(error-KPDiv);
        else
            temp = -HorGuiCtrlPara.KP1[YAW2ROLL][Vn]*KPDiv + HorGuiCtrlPara.KP2[YAW2ROLL][Vn]*(error+KPDiv);
    }
    else
    {
        //如果在第一段内
        temp = HorGuiCtrlPara.KP1[YAW2ROLL][Vn]*error;
    }
    //计算微分的值
    temp = temp - dYaw*HorGuiCtrlPara.KD[YAW2ROLL][Vn];
    //计算积分的值
    if (fabs(error) < HorGuiCtrlPara.KITh[YAW2ROLL][Vn])
    {
        ei = ei + error* HorGuiCtrlPara.KI[YAW2ROLL][Vn];
        ei = ValueLimit(ei, HorGuiCtrlPara.ILimit[YAW2ROLL][1], -HorGuiCtrlPara.ILimit[YAW2ROLL][1]);
        temp = temp + ei ;
    }
    else//超出积分开启的范围，积分无效，并设置误差的积分为0
    {
        ei = 0;
    }
    //总比例控制
    temp = temp * HorGuiCtrlPara.KP[YAW2ROLL][Vn];
    //输出限位
    temp = ValueLimit(temp, HorGuiCtrlPara.Limit[YAW2ROLL][Vn], -HorGuiCtrlPara.Limit[YAW2ROLL][Vn]);
    temp = ValueLimit2(temp);
    //输出
    ULast = temp;
    return temp;

}

F32 Plan::HeightToPitch(F32 aHei, F32 cHei, F32 cdHei, U8 Mode, U8 IsHeiUpdate)
{
    //F32 aPitch;
    F32 error, temp;
    F32 dError;
    //static F32 ei;
    static F32 ULast;
    F32 KPDiv;
    if(Mode == TRUE)
    {
        //这里启用了微分跟踪器，但是即使目标高度没有变化，垂直方向的速度也可能发生变化？
        //不知道为什么不更新跟踪器的数值
        //注释：zhangjy 2013.9.24
        if(IsHeiUpdate == TRUE)
        {
            HeiPIDCtrlPara.TD.Aim= aHei;
            HeiPIDCtrlPara.TD.V1 = cHei;
            HeiPIDCtrlPara.TD.V2 = cdHei;
        }
        TDFilter(&HeiPIDCtrlPara.TD);
        error = HeiPIDCtrlPara.TD.V1  - cHei;
        dError = HeiPIDCtrlPara.TD.V2 - cdHei;
    }
    else
    {
        error = aHei- cHei;
        dError = 0 - cdHei;
    }
    error = ValueLimit2(error);
    dError = ValueLimit2(dError);

    if (fabs(error) + fabs(dError) < HeiPIDCtrlPara.ErrTh[Vn])
    {
        return ULast;
    }

    //计算比例的值
    KPDiv = HeiPIDCtrlPara.KPDiv[Vn];
    if (fabs(error)>KPDiv)
    {
        //如果在第二段内
        if (error>0)
            temp = HeiPIDCtrlPara.KP1[Vn]*KPDiv + HeiPIDCtrlPara.KP2[Vn]*(error-KPDiv);
        else
            temp = -HeiPIDCtrlPara.KP1[Vn]*KPDiv + HeiPIDCtrlPara.KP2[Vn]*(error+KPDiv);
    }
    else
    {
        //如果在第一段内
        temp = HeiPIDCtrlPara.KP1[Vn]*error;
    }
    //计算微分的值
    temp = temp + dError*HeiPIDCtrlPara.KD[Vn];
    //计算积分的值
    if (fabs(error) < HeiPIDCtrlPara.KITh[Vn])
    {
        HeiToPitchEi = HeiToPitchEi + error * HeiPIDCtrlPara.KI[Vn];
        HeiToPitchEi = ValueLimit(HeiToPitchEi,HeiPIDCtrlPara.ILimit[Vn],-HeiPIDCtrlPara.ILimit[Vn]);
        temp = temp + HeiToPitchEi ;
    }
    else//超出积分开启的范围，积分无效，并设置误差的积分为0
    {
        HeiToPitchEi = 0;
    }
    //总比例控制
    temp = temp * HeiPIDCtrlPara.KP[Vn];
    //输出限位
    temp = ValueLimit(temp,HeiPIDCtrlPara.Limit[Vn],-HeiPIDCtrlPara.Limit[Vn]);
    temp = ValueLimit2(temp);
    //输出
    ULast = temp;
    return temp;
    //return aPitch;

}

F32 Plan::RollToPitch(F32 aRoll)
{
    F32 aPitch;
    if(fabs(aRoll) < RollToPitchPara.OpenTh[Vn])
        return 0;

    aPitch = RollToPitchPara.KP[Vn] * (fabs(aRoll) - RollToPitchPara.OpenTh[Vn]);
    if(aPitch > RollToPitchPara.Limit[Vn])
        aPitch = RollToPitchPara.Limit[Vn];

    return aPitch;

}

F32 Plan::SpeedToThrottle(F32 aSpe, F32 cSpe)
{
    F32 error=0;
    F32 temp=0;

    //F32  tempF32;
    static F32 ei = 0;
    static F32 ULast = 0;//zhangjy 修改，2013.1.22，该赋值只有编译时才有效，并不是每次都置0
    error = aSpe -cSpe;
    //如果没有超过门限值，则油门舵角保持不变
    if (fabs(error) < ThrCtrlPara.ErrTh )
    {
        //  error = 0;//使其保持积分值
        return ULast;
    }
    //计算比例的值
    if (fabs(error)>ThrCtrlPara.KPDiv)
    {
        if (error>0)
            temp = ThrCtrlPara.KP1 * ThrCtrlPara.KPDiv + ThrCtrlPara.KP2 * (error - ThrCtrlPara.KPDiv);
        else
            temp = -ThrCtrlPara.KP1 * ThrCtrlPara.KPDiv + ThrCtrlPara.KP2 * (error + ThrCtrlPara.KPDiv);
    }
    else
    {
        temp = ThrCtrlPara.KP1*error;
    }
    //没有计算微分的值
    /////////////////////////////////////////////////////////////////////////
    //计算积分的值
    if (fabs(error) < ThrCtrlPara.KITh)
    {
        //计算误差的积分
        ei = ei + error * ThrCtrlPara.KI;
        ei = ValueLimit(ei,ThrCtrlPara.ILimit,-ThrCtrlPara.ILimit);
        temp = temp + ei ;
    }
    else
    {
        ei=0;
    }

    //总比例控制
    temp = temp * ThrCtrlPara.KP + ThrCtrlPara.Ref;
    //输出限位
    temp = ValueLimit(temp,1,ThrCtrlPara.Limit);
    ULast = temp;
    return temp;


}

F32 Plan::HeightToSpeed(F32 aHei, F32 cHei, F32 cPitch)
{
    F32 error=0;
    F32  tempF32;


    if(HeiToSpeCtrlPara.IsOpen == FALSE)
        return 0;
    //PitchOpenTh<0 俯冲时暂时不耦合，等到飞机平飞后再耦合
    if(cPitch < -HeiToSpeCtrlPara.PitchOpenTh)
        return 0;
    error = aHei - cHei;
    //只在爬升高度较大时才耦合到速度
    if(error < HeiToSpeCtrlPara.ErrTh)
        return 0;
    tempF32 = (error - HeiToSpeCtrlPara.ErrTh) * HeiToSpeCtrlPara.KP;
    if(tempF32 > HeiToSpeCtrlPara.Limit)
        tempF32 = HeiToSpeCtrlPara.Limit;
    return tempF32;

}

void Plan::DirectGuide(DirectGuidInf *dGInf, CtrlStruct *FlightCtrlResult)
{
    F32 v;
    F32 Hei2Spe=0,aSpe=0;

    //新加入合法性检查，2012.12.26
    //改写：zhangjy
    if(dGInf->Gd_Ang<0)dGInf->Gd_Ang=0;	 //此处使用角度
    else if(dGInf->Gd_Ang>360) dGInf->Gd_Ang=360;

    //小型UAV飞行目标高度不可能超出此范围
    if(dGInf->Gd_Height<0.1) dGInf->Gd_Height=0.1;
    else if(dGInf->Gd_Height>10000) dGInf->Gd_Height=10000;

    FlightCtrlResult->Yaw = dGInf->Gd_Ang;
    //偏航直接赋值，采用北东地坐标系 ，该角度为与正北夹角，单位为°，顺时针为正，逆时针为负
    //////////////////////////////////////////////////////////滚转，滚转辅助转弯
    FlightCtrlResult->Roll = HeadingToRoll(FlightCtrlResult->Yaw,FlightState.Yaw,FlightState.dYaw);

    //////////////////////////////////////////////////////////高度，计算期望高度和实际高度差
    if(TargetHeight!=dGInf->Gd_Height)//获得期望高度，直接给出
    {
        TargetHeight=dGInf->Gd_Height;
        IsHeiUpdate=TRUE;
    }
    //else IsHeiUpdate=FALSE;//原来没有这个
    //////////////////////////////////////////////////////////俯仰，由高度差计算俯仰控制
    //高度变化很小，不更新高度，自己修改，原来开使用了浮点数等号比较
    //if(FlightCtrlResult->GuiState.HeightErr>0.01||FlightCtrlResult->GuiState.HeightErr<-0.01) IsHeiUpdate=TRUE;
    //else IsHeiUpdate=FALSE;
    //不能一直需要更新姿态数据，靶标数据和垂直速度，否则会导致高度误差计算一直为0，详细参加函数HeightToPitch
    //该问题是2012.8.17高度误差累积问题的原因？尚待测试（2012.8.18,测试失败，问题依旧存在）！
    //俯仰控制指令生成

    FlightCtrlResult->Pitch = HeightToPitch(TargetHeight,FlightState.Height,-FlightState.VelD,TRUE,IsHeiUpdate);
    Hei2Spe = HeightToSpeed(TargetHeight,FlightState.Height,FlightState.Pitch);
    IsHeiUpdate = FALSE;
    //滚转补偿,由于滚转引起的掉高
    FlightCtrlResult->Pitch += RollToPitch(FlightCtrlResult->Roll);
    FlightCtrlResult->Pitch = ValueLimit(FlightCtrlResult->Pitch,HeiPIDCtrlPara.Limit[Vn], -HeiPIDCtrlPara.Limit[Vn]);
    //////////////////////////////////////////////////////////速度
    if(HorGuiCtrlPara.IsUseAirspeed == TRUE)
    {
        v = FlightState.CaliAirspeed;//由空速，直接使用空速
    }
    else//否则使用北东地坐标系速度的合成作为实际速度，由GPS给出，应该是地速
    {
        //v= FlightState->VelX;//采用航点模式的速度策略，原来采用航线模式的速度，根据注释，航线模式也采用过该速度
        v = sqrt(FlightState.VelE * FlightState.VelE + FlightState.VelN * FlightState.VelN + FlightState.VelD * FlightState.VelD);
    }
    //这里可能有问题，因为跟踪算法计算的速度是水平系的，需要考虑长机VD才可以完成全速度计算！
    //发现分析：2013.4.21
    //by zhangjy
    //修订：zhangjy，需要此时长机的VD速度

    aSpe = dGInf->Gd_V+ Hei2Spe;//补偿速度叠加在控制目标上，这个速度是3维的
    FlightCtrlResult->Throttle = SpeedToThrottle(aSpe,v);

}








