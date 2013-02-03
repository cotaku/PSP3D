#include "math.h"
#include "debug.h"
#include <stdio.h>

float sin(float r)
{
    // By Ailsa

    float result = r, temp = r;
    float den = r, fac = 1.0;
    int n = 1, sign = 1;
    while ((temp > 1e-5) || (temp < -1e-5))        
    {
        n++, fac *= n, den *= r;
        n++, fac *= n, den *= r;
        temp = den / fac;
        sign = -sign;
        result = sign > 0 ? result + temp : result - temp;
    }
    return result;
}

float rsqrt(float a)
{
    if (a == 0)
    {
        DebugPrint("Divided by 0!\n");
        return 1;
    }

    // By Carmack

    int i;
    float x2, y;
    const float threehalfs = 1.5;

    char buf[16];

    x2 = a*0.5;
    y = a;

    sprintf(buf,"%x",y);
    i = *(int*)&y;  // evil floating point bit level hacking
    i = 0x5f3759df - (i>>1); // what the fuck?
    sprintf(buf,"%x",i);
    y = *(float*)&i;

    y = y * (threehalfs - (x2 * y * y)); // 1st iteration

    return y;
}
