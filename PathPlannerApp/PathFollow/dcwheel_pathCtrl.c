#include "dcwheel_pathCtrl.h"
#define _USE_MATH_DEFINES
#include "math.h"
#include "limits.h"


#define ENABLE 1
#define DISABLE 0
#define PI M_PI
#define max(a, b) ((a > b) ? a : b)
#define min(a, b) ((a < b) ? a : b)

static float getDistance(PositionTypedef a, PositionTypedef b)
{
	return sqrt( (a.x - b.x)*(a.x - b.x)  + (a.y - b.y)*(a.y - b.y) );
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

	ctrl->path_len = 0;
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

void PathCtrl_SetRobotPar(volatile PathCtrlTypedef* ctrl, float robotWheelDist, float predictLength)
{
	ctrl->predictLength = predictLength;
	ctrl->robotWheelDist = robotWheelDist;
}

void PathCtrl_SetPars(volatile PathCtrlTypedef* ctrl, float distP, float distD, float oriP, float oriD)
{
	ctrl->distParP = distP;
	ctrl->distParD = 0.0;
	ctrl->distParI = distD; //!!!!!!!!!!!!!!!!
	ctrl->oriParP = oriP;
	ctrl->oriParD = oriD;
}

void PathCtrl_SetPath(volatile PathCtrlTypedef* ctrl, PositionTypedef* path, uint16_t path_len)
{
	ctrl->path = path;
	ctrl->path_len = path_len;
}

void PathCtrl_SetState(volatile PathCtrlTypedef* ctrl, uint16_t start)
{
	if (ctrl->path_len == 0) //No path, disable Path Ctrl
		start = DISABLE; //TODO: inkább return

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
		ctrl->distError = 0.0;
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
		*angle -= (float)(2*PI);

	while (*angle < minLim)
		*angle += (float)(2*PI);
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
	k = ceil(fabs(ctrl->robotAngVel) / (fabs(beta) * ctrl->sample_s));
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

static void pathLoop(volatile PathCtrlTypedef* ctrl, uint16_t backward)
{
	if (backward)
	{
		ctrl->robotPrevVel *= -1.0f;
	}
	//Robot velocity
	ctrl->robotVel = getDistance(ctrl->path[ctrl->timeIndex], ctrl->path[ctrl->timeIndex + 1]);
	ctrl->robotVel += ctrl->distError * ctrl->distParP + (ctrl->distError - ctrl->distPrevError) * ctrl->distParD + ctrl->pathSumDist * ctrl->distParI ; //TODO: nem mûködik jól!!!!!!!
	ctrl->robotVel = 2.0 * ctrl->robotVel/ctrl->sample_s - ctrl->robotPrevVel;

	if (backward)
	{
		ctrl->robotPos.phi += PI;
		corrigateAngle((float*)&ctrl->robotPos.phi, 0);
		ctrl->robotVel *= -1.0f;
	}

	//if (ctrl->robot_speed_index)
		//bbx_print("|n|%d|v|%d\r\n", ctrl->robot_speed_index, (long)(ctrl->robotVel));

	//Robot angular velocity
	ctrl->robotAngVel = getDirection(ctrl->robotPos, ctrl->path[ctrl->timeIndex + ctrl->predictLength]);
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

void PathCtrl_Loop(volatile PathCtrlTypedef* ctrl, float *leftV, float *rightV)
{
	int16_t v_left, v_right;
	static int16_t count = 0;

	if (ctrl->enable == DISABLE)
		return;

	//Path ending
	if((ctrl->timeIndex + ctrl->predictLength) >= (ctrl->path_len - 1))
	{
		PathCtrl_SetState(ctrl, DISABLE);
		*leftV = 0.0f;
		*rightV = 0.0f;
		//ctrl->pathStop();
		return;
	}

	//Tracking error
	ctrl->distError = getDistance(ctrl->robotPos, ctrl->path[ctrl->timeIndex]);
	ctrl->pathSumDist += ctrl->distError;

	switch (ctrl->state)
	{
		case STATE_INIT:
			ctrl->robotVel = 0.0;
			ctrl->robotAngVel = 0.0;
			ctrl->brake = 0;
			ctrl->predictLength += 10;
			count = 10;

			if (ctrl->path[ctrl->timeIndex + 1].phi == 1000.0)
				ctrl->state = STATE_TURN;
			else
				ctrl->state = STATE_PATHFOLLOW;
			break;

		case STATE_PATHFOLLOW:
			if (ctrl->path[ctrl->timeIndex + 2].phi == 1000.0)
			{
				ctrl->state = STATE_PRE_STOP;			
				ctrl->robotVel = 0.0;
				ctrl->robotAngVel = 0.0;
				ctrl->timeIndex++;	
				break;				
			}

			//Forward/backward path following
			if (ctrl->path[ctrl->timeIndex + 1].phi > 1.5f*PI)
				pathLoop(ctrl, 1);
			else
				pathLoop(ctrl, 0);
			
			//Reset predict length
			if (count > 0)
			{
				count--;
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
			if (turnLoop(ctrl, ctrl->path[ctrl->timeIndex + ctrl->predictLength+1].phi))
			{ 
				ctrl->state = STATE_INIT;
				ctrl->timeIndex += ctrl->predictLength + 1;
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