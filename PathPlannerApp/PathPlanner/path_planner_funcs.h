#pragma once

#define _USE_MATH_DEFINES
#include <math.h>

#define EPS	(1.0f)
#define PI ((float)M_PI)

template <typename T> int sgn(T val) 
{
    return (T(0) < val) - (val < T(0));
}