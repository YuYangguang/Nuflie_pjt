
#ifndef _MATHCAL_H__
#define _MATHCAL_H__

#include "smarteye/core/core.hpp"
#include <math.h>

namespace smarteye
{

F32 Sign233(F32 x)
{
    if(x>0)
        return 1;
    else if(x<0)
        return -1;
    else
        return 0;
}

F32 Fsg(F32 x, F32 d)
{
    F32 temp1;
    F32 temp2;
    temp1 = Sign233(x + d);
    temp2 = Sign233(x - d);
    return (0.5 * (temp1 - temp2));

}

F32 Fhan(F32 x1, F32 x2, F32 r, F32 h)
{
    F32 d;
    F32 a, a0, a1, a2;
    F32 y;
    F32 output;
    d = r * h * h;
    a0 = h * x2;
    y = x1 + a0;
    a1 = sqrt(d * (d + 8.0 * fabs(y)));
    a2 = a0 + Sign233(y) * (a1 - d) / 2.0;
    a = (a0 + y) * Fsg(y, d) + a2 * (1.0 - Fsg(y, d));
    output = -r * (a / d) * Fsg(a, d) - r * Sign233(a) * (1.0 - Fsg(a, d));

    return output;
}

F32 ValueLimit2(F32 value)
{
    S32 temp;
    temp = value * 10;
    value = (F32)temp / 10.0;
    return value;
}

F32 ValueLimit(F32 value,F32 max,F32 min)
{
    if(value > max)
        return max;
    else if(value < min)
        return min;
    else
        return value;
}

}

#endif
