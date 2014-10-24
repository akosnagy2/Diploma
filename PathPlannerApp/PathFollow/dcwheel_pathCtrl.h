#ifndef DCWHEEL_PATHCTRL_H_
#define DCWHEEL_PATHCTRL_H_

#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>

#define DCWHEEL_POINT_SEARCH_LIMIT 25
#define DCWHEEL_END_DISTANCE_UNIT 5.0
#define DCWHEEL_SEGMENT_END_DISTANCE 5.0

typedef struct
{
	float	x;
	float	y;
	float	phi;
} PositionTypedef;

typedef enum
{
	FORWARD,
	BACKWARD
} DirectionTypedef;

typedef enum {
	STATE_INIT,
	STATE_PATHFOLLOW,
	STATE_PRE_STOP,
	STATE_TURN
} StateTypedef;

typedef struct
{
	PositionTypedef*    path;
	float*				velocity;
	uint16_t			path_len;
	DirectionTypedef	dir;
} PathSegmentTypedef;

typedef struct
{
	//Common
	float				sample_s;
	uint16_t			timer_index;
	uint16_t			enable;
	void 				(*function)();
	//PathFollow
	uint16_t			predictDistanceLength;
	uint16_t			predictSampleLength;
	uint16_t			predictIndex;
	uint32_t			timeIndex;
	uint32_t			segmentIndex;
	float	 			oriParP;
	float				oriParD;
	float				predictError;
	float				distError;
	float				robotWheelDist;
	float				robotPrevAngVel;
	PositionTypedef		robotPos;
	PositionTypedef		shadowRobotPos;
	uint16_t			robotPosFlag;
	PositionTypedef		robotPrevPos;
	float				robotVel;
	float				robotAngVel;
	float				maxBeta;
	float				maxW;
	float				debugData[6];
	//Path
	PathSegmentTypedef* pathSegments;
	uint16_t			pathSegmentsLen;
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

void PathCtrl_SetPars(volatile PathCtrlTypedef* ctrl, float oriP, float oriD);
void PathCtrl_SetRobotPar(volatile PathCtrlTypedef* ctrl, float robotWheelDist);
void PathCtrl_SetPathSegments(volatile PathCtrlTypedef* ctrl, PathSegmentTypedef* pathSegments, uint16_t pathSegmentsLength);
void PathCtrl_SetState(volatile PathCtrlTypedef* ctrl, uint16_t start);

//Loop
void PathCtrl_Loop(volatile PathCtrlTypedef* ctrl, float *leftV, float *rightV);

#ifdef __cplusplus
}
#endif

#endif /* DCWHEEL_PATHCTRL_H_ */