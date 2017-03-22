#ifndef _CAMERA_H_
#define _CAMERA_H_

#include <smarteye_common/cameraMsg.h>
#include <stdio.h>
#include <stdlib.h>
namespace smarteye {
class cameraInfo
{
    public:
    cameraInfo();
    ~cameraInfo();
    smarteye_common::cameraMsg cameraData;
    void imageProcess();
};

}

#endif
