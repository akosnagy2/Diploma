#pragma once

#define _USE_MATH_DEFINES
#include <math.h>

#define EPS	(0.1f)
#define PI ((float)M_PI)

//0-nál 0-t ad vissza
template <typename T> int sgn(T val) 
{
    return (T(0) < val) - (val < T(0));
}

//0-nál 1-et ad vissza
template <typename T> int sgnC(T val) 
{
    return (T(0) <= val) - (val < T(0));
}