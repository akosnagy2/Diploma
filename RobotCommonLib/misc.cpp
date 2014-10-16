#include "misc.h"
#define _USE_MATH_DEFINES
#include <math.h>

float wrapAngle(float phi)
{
	phi = fmod(phi + M_PI, 2.0f * M_PI);
	if(phi < 0)
		phi += 2 * M_PI;
	return phi - M_PI;
}