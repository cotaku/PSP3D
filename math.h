#ifndef __MATH_H__
#define __MATH_H__

#include <psptypes.h>
#include <pspmath.h>

#define EPSILON 1e-8
#define PI 3.1415926
#define PI_2 1.5707963
#define DegreeToRadian(d) (d * 0.0174533)

#define min(a, b) (((a) < (b)) ? (a) : (b))
#define max(a, b) (((a) > (b)) ? (a) : (b))
#define swap(a, b) (((a) ^= (b)), ((b) ^= (a)), ((a) ^= (b)))

template <class T>
inline T Min3(T a, T b, T c)
{
    T t = min(a, b);
    return t < c ? t : c;
}

template <class T>
inline T Max3(T a, T b, T c)
{
    T t = max(a, b);
    return t > c ? t : c;
}

inline int abs(int a)
{
    int const mask = a >> sizeof(int) * 8 - 1;
    return (a ^ mask) - mask;
}

inline float fabs(float a)
{
    return a >= 0 ? a : -a;
}

float sin(float r);

inline float cos(float r)
{
    return sin(PI_2 - r);
}

inline float tan(float r)
{
    return sin(r) / cos(r);
}

float rsqrt(float a);

inline void SphericalToCartesian(float r, float theta, float phi, float* x, float* y, float* z)
{
    *x = r * sin(phi) * cos(theta);
    *y = r * sin(phi) * sin(theta);
    *z = r * cos(phi);
}

//inline void CartesianToSpherical(float x, float y, float z, float* r, float* theta, float* phi)
//{
//    r = 1 / rsqrt(x*x+y*y+z*z);
//    theta = 
//}

#endif
