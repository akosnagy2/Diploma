#ifndef DCWHEEL_PATHCTRL_H_
#define DCWHEEL_PATHCTRL_H_

#ifdef __cplusplus
extern "C" {
#endif


#include "Position.h"
#include <stdint.h>
/*
typedef struct{
	float	x;
	float	y;
	float	phi;
} PositionTypedef;
*/
#define PositionTypedef Position

typedef enum {
	STATE_INIT,
	STATE_PATHFOLLOW,
	STATE_PRE_STOP,
	STATE_TURN
} StateTypedef;

typedef struct{
	//Common
	float				sample_s;
	uint16_t			timer_index;
	uint16_t			enable;
	void 				(*function)();
	//PathFollow
	uint16_t			predictLength;
	uint32_t			timeIndex;
	float				distParP;
	float				distParD;
	float				distParI;
	float	 			oriParP;
	float				oriParD;
	float				robotWheelDist;
	float				distError;
	float				distPrevError;
	float				robotSumDist;
	float				pathSumDist;
	float				robotPrevVel;
	float				robotPrevAngVel;
	PositionTypedef		robotPos;
	PositionTypedef		robotPrevPos;
	float				robotVel;
	float				robotAngVel;
	float				maxBeta;
	float				maxW;
	//Path
	PositionTypedef*    path;
	uint16_t			path_len;
	void				(*pathStop)();
	StateTypedef		state;
	uint16_t			brake;
	//BlackBox
	long				robot_speed_index;
	long 				robot_angspeed_index;
	long				left_speed_index;
	long				right_speed_index;
} PathCtrlTypedef;

//Path Controller Initialization
uint16_t PathCtrl_Init(volatile PathCtrlTypedef* ctrl, uint16_t timer_index, float maxBeta, float maxW, uint16_t sample_ms, void (*function)(), void (*pathStopFunction)());
void PathCtrl_BBXInit(volatile PathCtrlTypedef* ctrl, long robot_speed_index, long robot_angspeed_index, long left_speed_index, long right_speed_index);
void PathCtrl_BBXReset(volatile PathCtrlTypedef* ctrl);

void PathCtrl_SetPars(volatile PathCtrlTypedef* ctrl, float distP, float distD, float oriP, float oriD);
void PathCtrl_SetRobotPar(volatile PathCtrlTypedef* ctrl, float robotWheelDist, float predictLength);
void PathCtrl_SetPath(volatile PathCtrlTypedef* ctrl, PositionTypedef* path, uint16_t path_len);
void PathCtrl_SetState(volatile PathCtrlTypedef* ctrl, uint16_t start);

//Loop
void PathCtrl_Loop(volatile PathCtrlTypedef* ctrl, float *leftV, float *rightV);

#ifdef __cplusplus
}
#endif

#endif /* DCWHEEL_PATHCTRL_H_ */