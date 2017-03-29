#ifndef _IMU_H_
#define _IMU_H_

#include <smarteye_common/IMUMsg.h>

#include <stdio.h>
namespace smarteye {
class IMUInfo
{
public:
    IMUInfo();
    ~IMUInfo();
    smarteye_common::IMUMsg IMUData;
    void IMUProcess();
};

}


#endif

