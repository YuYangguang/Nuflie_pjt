
#ifndef _MATHCAL_H__
#define _MATHCAL_H__

#include "smarteye/core/core.hpp"
#include <math.h>

namespace smarteye
{

F32 Sign233(F32 x);
F32 Fsg(F32 x, F32 d);
F32 Fhan(F32 x1, F32 x2, F32 r, F32 h);
F32 ValueLimit2(F32 value);
F32 ValueLimit(F32 value,F32 max,F32 min);


}

#endif
