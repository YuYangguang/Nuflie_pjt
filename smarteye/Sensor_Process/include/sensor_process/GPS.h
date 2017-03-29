#ifndef _GPS_H_
#define _GPS_H_

#include <smarteye_common/GPSMsg.h>
#include <stdio.h>
namespace smarteye {
class GPSInfo
{
public:
    GPSInfo();
    ~GPSInfo();
    smarteye_common::GPSMsg GPSData;
    void GPSProcess();



};
}

#endif

