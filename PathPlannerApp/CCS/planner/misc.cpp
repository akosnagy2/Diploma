#include "misc.h"

#define _USE_MATH_DEFINES
#include <cmath>
double wrapAngle(double phi)
{
	phi = fmod(phi + M_PI, 2 * M_PI);
	if (phi < 0)
		phi += 2 * M_PI;
	return phi - M_PI;
}

double r2d(double phi)
{
	return phi / M_PI * 180;
}

double d2r(double phi)
{
	return phi / 180 * M_PI;
}

double scalarProduct(double ax, double ay, double bx, double by)
{
	return ax*bx + ay*by;
}

bool almostEqualRelative(double A, double B, double maxRelDiff)
{
	// Calculate the difference.
	double diff = fabs(A - B);
	A = fabs(A);
	B = fabs(B);
	// Find the largest
	float largest = (B > A) ? B : A;

	if (diff <= largest * maxRelDiff)
		return true;
	return false;
}

int sgn(double val) {
    return (0 < val) - (val < 0);
}