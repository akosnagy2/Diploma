#pragma once

namespace PathPlanner
{

	struct Angle
	{
		static float Corrigate(float angle);
		static float DirectedAngleDist(float thetaStart, float thetaEnd, int turnDir);
	};

}