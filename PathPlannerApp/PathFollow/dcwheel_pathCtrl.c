#include "dcwheel_pathCtrl.h"
#define _USE_MATH_DEFINES
#include "math.h"
#include "limits.h"


#define ENABLE 1
#define DISABLE 0
#define PI (float)M_PI
#define max(a, b) ((a > b) ? a : b)
#define min(a, b) ((a < b) ? a : b)

static float getDistance(PositionTypedef a, PositionTypedef b)
{
	return sqrtf( (a.x - b.x)*(a.x - b.x)  + (a.y - b.y)*(a.y - b.y) );
}

static float getDistanceOnPath(uint32_t a, uint32_t b, PositionTypedef* path)
{
	float dist = 0.0f;
	while (a < b)
		dist += getDistance(path[a], path[a++]);
	return dist;
}

static int getClosestPointOnPath(PositionTypedef robot, PositionTypedef* path, uint32_t path_len)
{
	uint32_t index = 0, i;
	float min = getDistance(robot, path[index]);
	float t;

	for (i = 1; i < path_len; i++)
	{
		t = getDistance(robot, path[i]);
		if (t < min)
		{
			min = t;
			index = i;
		}
	}
	return index;
}

static float getDirection(PositionTypedef a, PositionTypedef b)
{
	return atan2((b.y - a.y), (b.x - a.x));
}

uint16_t PathCtrl_Init(volatile PathCtrlTypedef* ctrl, uint16_t timer_index, float maxBeta, float maxW, uint16_t sample_ms, void (*function)(), void (*pathStopFunction)())
{
	ctrl->timer_index = timer_index;
	ctrl->function = function;
	ctrl->enable = DISABLE;
	ctrl->sample_s = ((float)sample_ms) / 1000.0f;

	ctrl->pathSegmentsLen = 0;
	ctrl->pathStop = pathStopFunction;

	ctrl->maxBeta = maxBeta;
	ctrl->maxW = maxW;
	ctrl->brake = 0;
	ctrl->state = STATE_INIT;

	//Reset
	PathCtrl_SetState(ctrl, DISABLE);

	return 1;
}

void PathCtrl_BBXInit(volatile PathCtrlTypedef* ctrl, long robot_speed_index, long robot_angspeed_index, long left_speed_index, long right_speed_index)
{
	ctrl->robot_speed_index = robot_speed_index;
	ctrl->robot_angspeed_index= robot_angspeed_index;
	ctrl->left_speed_index = left_speed_index;
	ctrl->right_speed_index = right_speed_index;
}

void PathCtrl_BBXReset(volatile PathCtrlTypedef* ctrl)
{

}

void PathCtrl_SetRobotPar(volatile PathCtrlTypedef* ctrl, float robotWheelDist)
{
	ctrl->robotWheelDist = robotWheelDist;
}

void PathCtrl_SetPars(volatile PathCtrlTypedef* ctrl, float distP, float distI, float oriP, float oriD)
{
	ctrl->distParP = distP;
	ctrl->distParI = distI;
	ctrl->oriParP = oriP;
	ctrl->oriParD = oriD;
}

void PathCtrl_SetPathSegments(volatile PathCtrlTypedef* ctrl, PathSegmentTypedef* pathSegments, uint16_t pathSegmentsLength)
{
	ctrl->pathSegments = pathSegments;
	ctrl->pathSegmentsLen = pathSegmentsLength;
}

void PathCtrl_SetState(volatile PathCtrlTypedef* ctrl, uint16_t start)
{
	if (ctrl->pathSegmentsLen == 0) //No path, disable Path Ctrl
		start = DISABLE; //TODO: ink�bb return

	ctrl->enable = start;

	if (start == ENABLE)
	{
		//PathCtrl_BBXReset(ctrl);
		//cntrl_timer_start(ctrl->timer_index);
	}
	else
	{
		//SpeedCtrl_SetSetpoint(&LeftSpeedCtrl, 0);
		//SpeedCtrl_SetSetpoint(&RightSpeedCtrl, 0);

		//cntrl_timer_stop(ctrl->timer_index);

		ctrl->timeIndex = 0;
		ctrl->segmentIndex = 0;
		ctrl->distError = 0.0;
		ctrl->predictError = 0.0;
		ctrl->distPrevError = 0.0;
		ctrl->robotPrevVel = 0.0;
		ctrl->robotPrevAngVel = 0.0;
		ctrl->robotSumDist = 0.0;
		ctrl->pathSumDist = 0.0;
	}
}

static uint16_t checkAngle(float start, float end, float val, uint16_t forward)
{
	if ((min(start, end) < val) && (val < max(start, end)))
	{
		if (start < end)
			return forward;
		else
			return !forward;
	}
	else
	{
		if (start < end)
			return !forward;
		else
			return forward;
	}
}

static void corrigateAngle(float *angle, uint16_t zero_to_2pi)
{
	float minLim, maxLim;

	if (zero_to_2pi)
	{
		minLim = 0.0f;
		maxLim = 2*PI;
	}
	else
	{
		minLim = -PI;
		maxLim = PI;
	}

	while (*angle > maxLim)
		*angle -= (2*PI);

	while (*angle < minLim)
		*angle += (2*PI);
}

static uint16_t turnLoop(volatile PathCtrlTypedef* ctrl, float destTheta)
{
	int16_t k;
	uint16_t ret = 0;
	float pp, beta;
	float phi = ctrl->robotPos.phi;

	//Corrigate direction
	corrigateAngle(&destTheta, 1);

	//Corrigate direction
	corrigateAngle(&phi, 1);

 	if (fabs(destTheta - phi) > PI)
	{
		if (destTheta > phi)
			beta = -ctrl->maxBeta; //Decrease
		else
			beta = ctrl->maxBeta; //Increase
	}
	else
	{
		if (destTheta > phi)
			beta = ctrl->maxBeta; //Increase
		else
			beta = -ctrl->maxBeta; //Decrease
	}

	//Calculate maximum angular velocity
	if (ctrl->brake)
		ctrl->robotAngVel = ctrl->robotPrevAngVel;
	else
		ctrl->robotAngVel = ctrl->robotPrevAngVel + beta * ctrl->sample_s;

	if (ctrl->robotAngVel > ctrl->maxW) //Saturation
		ctrl->robotAngVel = ctrl->maxW;

	if (ctrl->robotAngVel < -ctrl->maxW) //Saturation
		ctrl->robotAngVel = -ctrl->maxW;

	//Check angular velocity
	k = ceilf(fabs(ctrl->robotAngVel) / (fabs(beta) * ctrl->sample_s));
	pp = phi + k*ctrl->robotAngVel*ctrl->sample_s - k*(k-1)*0.5f*beta*ctrl->sample_s*ctrl->sample_s;

	//Corrigate direction
	corrigateAngle(&pp, 1);

	if (!checkAngle(phi, destTheta, pp, (beta > 0.0))) //Check theta at predicted stop position
	{
		ctrl->brake = 1; //Start brake
		ctrl->robotAngVel = ctrl->robotPrevAngVel - beta * ctrl->sample_s;
		if (fabs(ctrl->robotPrevAngVel) < fabs(beta*ctrl->sample_s))
			ctrl->robotAngVel = ctrl->robotPrevAngVel;
	}

	//Decrease ang. vel. when we are near to destTheta
	if (!checkAngle(phi, destTheta, (phi + ctrl->robotAngVel * ctrl->sample_s), (beta > 0.0))) //Corrigate ang. vel. near destTheta
	{
		ctrl->robotAngVel = destTheta - phi;
		if (ctrl->robotAngVel > PI)
			ctrl->robotAngVel = 2*PI - ctrl->robotAngVel;
		if (ctrl->robotAngVel < -PI)
			ctrl->robotAngVel = 2*PI + ctrl->robotAngVel;
		ctrl->robotAngVel = ctrl->robotAngVel / ctrl->sample_s;
		ret = 1;
	}

	return ret;
}

static void GetEndPosition(PositionTypedef a, PositionTypedef b, float radius, PositionTypedef center, PositionTypedef *res, PositionTypedef robot)
{
	//http://mathworld.wolfram.com/Circle-LineIntersection.html
	float dx, dy, dr, sgn;
	PositionTypedef r0, r1;

	//Transform to (0,0)
	a.x -= center.x;
	a.y -= center.y;

	dx = -a.x;
	dy = -a.y;
	dr = sqrtf(dx*dx + dy*dy);

	//No intersection
	{
		sgn = ((dy < 0.0f) ? -1.0f : 1.0f);
		r0.x =  sgn*dx*radius;
		r1.x =  -sgn*dx*radius;
		r0.x /= (dr);
		r1.x /= (dr);

		r0.y = fabs(dy)*radius;
		r1.y = -fabs(dy)*radius;
		r0.y /= (dr);
		r1.y /= (dr);
	}

	//Transform back to center
	r0.x += center.x;
	r0.y += center.y;
	r1.x += center.x;
	r1.y += center.y;

	a.x += center.x;
	a.y += center.y;

	//Decide two intersection
	if (getDistance(a, r0) > getDistance(a, r1))
		*res = r0;
	else
		*res = r1;
}

static void pathLoop(volatile PathCtrlTypedef* ctrl)
{
	PositionTypedef ref, a, b;
	static float rad = 0.0f;
	if (ctrl->pathSegments[ctrl->segmentIndex].dir == BACKWARD)
	{
		ctrl->robotPrevVel *= -1.0f;
	}
	//Robot velocity
	ctrl->robotVel = getDistance(ctrl->pathSegments[ctrl->segmentIndex].path[ctrl->timeIndex], ctrl->pathSegments[ctrl->segmentIndex].path[ctrl->timeIndex + 1]);
	//if (ctrl->distError > 30.0f)
	ctrl->robotVel += ctrl->distError * ctrl->distParP + ctrl->pathSumDist * ctrl->distParI ; //TODO: nem m�k�dik j�l!!!!!!!
	ctrl->robotVel = 2.0 * ctrl->robotVel/ctrl->sample_s - ctrl->robotPrevVel;

	if (ctrl->pathSegments[ctrl->segmentIndex].dir == BACKWARD)
	{
		ctrl->robotPos.phi += PI;
		corrigateAngle((float*)&ctrl->robotPos.phi, 0);
		ctrl->robotVel *= -1.0f;
	}

	//if (ctrl->robot_speed_index)
		//bbx_print("|n|%d|v|%d\r\n", ctrl->robot_speed_index, (long)(ctrl->robotVel));

	//Robot angular velocity
	if (ctrl->predictIndex == ctrl->pathSegments[ctrl->segmentIndex].path_len - 1)
	{
		a = ctrl->pathSegments[ctrl->segmentIndex].path[ctrl->pathSegments[ctrl->segmentIndex].path_len - 2];
		b = ctrl->pathSegments[ctrl->segmentIndex].path[ctrl->pathSegments[ctrl->segmentIndex].path_len - 1];

		rad += 10.0f;
		GetEndPosition(a, b, rad, b, &ref, ctrl->robotPos);
	}
	else
	{
		rad = 0.0f;
		ref = ctrl->pathSegments[ctrl->segmentIndex].path[ctrl->predictIndex];
	}

	ctrl->robotAngVel = getDirection(ctrl->robotPos, ref);
	ctrl->robotAngVel -= ctrl->robotPos.phi;

	corrigateAngle((float*)&ctrl->robotAngVel, 0);

	ctrl->robotAngVel = ctrl->robotAngVel * ctrl->oriParP + (ctrl->robotAngVel - ctrl->robotPrevAngVel*ctrl->sample_s) * ctrl->oriParD;

	if (ctrl->robotAngVel > PI)
		ctrl->robotAngVel = PI;

	if (ctrl->robotAngVel < -PI)
		ctrl->robotAngVel = -PI;

	ctrl->robotAngVel /= ctrl->sample_s;
	
	//if (ctrl->robot_angspeed_index)
		//bbx_print("|n|%d|v|%d\r\n", ctrl->robot_angspeed_index, (long)(ctrl->robotAngVel));
}

float getTrackingError(PositionTypedef p0, PositionTypedef p1, float curvature, PositionTypedef robotPos, PositionTypedef *center)
{
	//http://mathforum.org/library/drmath/view/53027.html
	float q, h, dist, sign;
	PositionTypedef p3;

	if (fabs(curvature) > 0.0001f)
	{
		p3.x = (p0.x + p1.x)*0.5f;
		p3.y = (p0.y + p1.y)*0.5f;

		q = sqrtf( (p0.x-p1.x)*(p0.x-p1.x) + (p0.y-p1.y)*(p0.y-p1.y) );
		h = sqrtf(1.0f/(curvature*curvature) - q*q/4);

		if (curvature > 0.0f)
		{
			center->x = p3.x - h * (p0.y - p1.y) / q;
			center->y = p3.y - h * (p1.x - p0.x) / q;
		}
		else
		{
			center->x = p3.x + h * (p0.y - p1.y) / q;
			center->y = p3.y + h * (p1.x - p0.x) / q;
		}
	}
	else
	{
		center->x = p1.y - p0.y;
		center->y = p0.x - p1.x;
		center->x += p0.x;
		center->y += p0.y;
	}

	//Determinate robot distance from prediction line
	dist = fabs((center->x - p0.x)*(p0.y - robotPos.y) - (p0.x - robotPos.x)*(center->y - p0.y));
	dist /= sqrtf((center->x - p0.x)*(center->x - p0.x) + (center->y - p0.y)*(center->y - p0.y));

	sign = (p0.x - center->x)*(robotPos.y - center->y) - (p0.y - center->y)*(robotPos.x - center->x);
	if ((sign < 0.0f) && (curvature > 0.0f))
		dist *= -1.0f;

	return dist;
}

void PathCtrl_Loop(volatile PathCtrlTypedef* ctrl, float *leftV, float *rightV)
{
	PositionTypedef center;
	//int16_t v_left, v_right;
	if (ctrl->enable == DISABLE)
		return;

	//Set predict length 
	ctrl->predictIndex = min(ctrl->timeIndex + ctrl->predictLength, ctrl->pathSegments[ctrl->segmentIndex].path_len - 1);

	//Tracking error
	if (ctrl->timeIndex < ctrl->pathSegments[ctrl->segmentIndex].path_len - 1)
	{
		ctrl->distError = getTrackingError(ctrl->pathSegments[ctrl->segmentIndex].path[ctrl->timeIndex],  ctrl->pathSegments[ctrl->segmentIndex].path[ctrl->timeIndex + 1],  ctrl->pathSegments[ctrl->segmentIndex].curvature[ctrl->timeIndex], ctrl->robotPos, &center);
		ctrl->pathSumDist += ctrl->distError;
		ctrl->debugData[0] = ctrl->distError;
		/*
		if (ctrl->distError < 1.0f)
		{
			ctrl->predictCntr = ctrl->predictLengthImpulse - ctrl->predictLength;
			ctrl->predictLength = ctrl->predictLengthImpulse;
			ctrl->predictIndex = min(ctrl->timeIndex + ctrl->predictLength, ctrl->pathSegments[ctrl->segmentIndex].path_len - 1);
		}
		*/
	}

	//Predict Tracking error
	if (ctrl->predictIndex < ctrl->pathSegments[ctrl->segmentIndex].path_len - 1)
	{
		ctrl->predictError = getTrackingError(ctrl->pathSegments[ctrl->segmentIndex].path[ctrl->predictIndex],  ctrl->pathSegments[ctrl->segmentIndex].path[ctrl->predictIndex + 1],  ctrl->pathSegments[ctrl->segmentIndex].curvature[ctrl->predictIndex], ctrl->robotPos, &center);				
		ctrl->debugData[1] = ctrl->predictError;
		ctrl->debugData[2] = center.x;
		ctrl->debugData[3] = center.y;
		ctrl->debugData[4] = center.x + (ctrl->pathSegments[ctrl->segmentIndex].path[ctrl->predictIndex].x - center.x)*100;
		ctrl->debugData[5] = center.y + (ctrl->pathSegments[ctrl->segmentIndex].path[ctrl->predictIndex].y - center.y)*100;
	}
	

	switch (ctrl->state)
	{
		case STATE_INIT:
			ctrl->robotVel = 0.0;
			ctrl->robotAngVel = 0.0;
			ctrl->brake = 0;
			ctrl->predictCntr = ctrl->predictLengthImpulse - ctrl->predictLength;
			ctrl->predictLength = ctrl->predictLengthImpulse;
			ctrl->state = STATE_PATHFOLLOW;
			break;

		case STATE_PATHFOLLOW:
			if (ctrl->timeIndex + 1 == ctrl->pathSegments[ctrl->segmentIndex].path_len)
			{
				ctrl->state = STATE_PRE_STOP;			
				ctrl->robotVel = 0.0;
				ctrl->robotAngVel = 0.0;
				ctrl->timeIndex = 0;	
				ctrl->segmentIndex++;
				if (ctrl->segmentIndex == ctrl->pathSegmentsLen)
				{
					PathCtrl_SetState(ctrl, DISABLE);
					*leftV = 0.0f;
					*rightV = 0.0f;
					//ctrl->pathStop();
					return;
				}
				break;				
			}

			//Path following
			pathLoop(ctrl);

			//Reset predict length
			if (ctrl->predictCntr > 0)
			{
				ctrl->predictCntr--;
				ctrl->predictLength--;
			}
			ctrl->timeIndex++;			
			break;

		case STATE_PRE_STOP:
			ctrl->robotVel = 0.0;
			ctrl->robotAngVel = 0.0;
			ctrl->state = STATE_TURN;
			break;

		case STATE_TURN:
			if (turnLoop(ctrl, ctrl->pathSegments[ctrl->segmentIndex].path[0].phi))
			{ 
				ctrl->state = STATE_INIT;
			}
			break;
	}

	//Robot wheels velocity
	*leftV = (float)(ctrl->robotVel - ctrl->robotWheelDist * ctrl->robotAngVel * 0.5f);
	*rightV = (float)(ctrl->robotVel + ctrl->robotWheelDist * ctrl->robotAngVel * 0.5f);

	//if (ctrl->left_speed_index)
	//	bbx_print("|n|%d|v|%d\r\n", ctrl->left_speed_index, (long)v_left);
	//if (ctrl->right_speed_index)
	//	bbx_print("|n|%d|v|%d\r\n", ctrl->right_speed_index, (long)v_right);

	//SpeedCtrl_SetSetpoint(&RightSpeedCtrl, v_right);
	//SpeedCtrl_SetSetpoint(&LeftSpeedCtrl, v_left);

	ctrl->robotPrevPos = ctrl->robotPos;
	ctrl->distPrevError = ctrl->distError;
	ctrl->robotPrevAngVel = ctrl->robotAngVel;
	ctrl->robotPrevVel = ctrl->robotVel;
}