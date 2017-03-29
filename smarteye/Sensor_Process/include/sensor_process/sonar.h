#ifndef _SONAR_H_
#define _SONAR_H_

#include <smarteye_common/sonarMsg.h>
#include <stdio.h>

namespace smarteye{
class SonarInfo
{
public:
    SonarInfo();
    ~SonarInfo();
    smarteye_common::sonarMsg sonarMsg;
    void sonarProcess();
};

}

#endif
