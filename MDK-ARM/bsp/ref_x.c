#include "ref_x.h"

#include <math.h>

float force_leg_pos(double x)
{
    // Fourier3 coefficients
    const double a0 = -0.7682;
    const double a1 =  0.572;
    const double b1 = -0.1764;
    const double a2 =  0.1366;
    const double b2 = -0.0447;
    const double a3 = -0.02925;
    const double b3 =  0.2065;
    const double w  =  2.317;

    double wx = w * x;

    float y = a0
        + a1 * cos(wx)      + b1 * sin(wx)
        + a2 * cos(2 * wx)  + b2 * sin(2 * wx)
        + a3 * cos(3 * wx)  + b3 * sin(3 * wx);

    return y;
}

float other_leg_pos(double x)
{
    const float a0 = -0.2921f;
    const float a1 = -0.03752f;
    const float b1 =  0.1982f;
    const float a2 =  0.04183f;
    const float b2 =  0.07817f;
    const float a3 =  0.08828f;
    const float b3 =  0.02173f;
    const float w  =  2.453f;

    float wx = w * x;

    float y = a0
        + a1 * cosf(wx)       + b1 * sinf(wx)
        + a2 * cosf(2 * wx)   + b2 * sinf(2 * wx)
        + a3 * cosf(3 * wx)   + b3 * sinf(3 * wx);

    return y;
}

float force_leg_vel(double x)
{
    const double a1 =  0.572;
    const double b1 = -0.1764;
    const double a2 =  0.1366;
    const double b2 = -0.0447;
    const double a3 = -0.02925;
    const double b3 =  0.2065;
    const double w  =  2.317;

    double wx = w * x;

    double dy = (-a1 * w * sin(wx) + b1 * w * cos(wx))
              + (-2*a2 * w * sin(2*wx) + 2*b2 * w * cos(2*wx))
              + (-3*a3 * w * sin(3*wx) + 3*b3 * w * cos(3*wx));

    return dy;
}

float other_leg_vel(float x)
{
    const float a1 = -0.03752f;
    const float b1 =  0.1982f;
    const float a2 =  0.04183f;
    const float b2 =  0.07817f;
    const float a3 =  0.08828f;
    const float b3 =  0.02173f;
    const float w  =  2.453f;

    float wx = w * x;

    float dy = (-a1 * w * sinf(wx) + b1 * w * cosf(wx))
             + (-2*a2 * w * sinf(2*wx) + 2*b2 * w * cosf(2*wx))
             + (-3*a3 * w * sinf(3*wx) + 3*b3 * w * cosf(3*wx));

    return dy;
}


