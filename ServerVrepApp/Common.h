#pragma once

#include <iostream>
#include <fstream>
#include <string>
#include "boost\asio\ip\tcp.hpp"
#include "simpleInConnection.h"
#include "CtrlMessage.h"	
#include "Pos2dMessage.h"
#include "PathMessage.h"

using boost::asio::ip::tcp;
using namespace std;

typedef struct
{
	float PathMaxVelocity;
	float PathMaxAcceleration;
	float PathMaxTangentAcceleration;
	float PathMaxAngularVelocity;
} PathParamsTypedef;


typedef struct 
{
	 float motorMaxSpeed;
	 float motorMaxAccel;
	 float motorSmoothFactor;
	 float motorMultFactor;
} MotorParamsTypedef;

typedef struct
{
	float TimeStep;
	MotorParamsTypedef LeftMotor;
	MotorParamsTypedef RightMotor;
	float WheelDiameter;
	float WheelBase;
	float PredictLength;
	float PredictLengthImpulse;
	float DistPar_P;
	float DistPar_D;
	float OriPar_P;
	float OriPar_D;
	PathParamsTypedef PathPars;
} PathFollowParamsTypedef;

typedef struct
{
	PathFollowParamsTypedef PathFollow;
	float iterationMax;
	float fixPathProb;
	float roadmapProb;
	float randSeed;
	float envFile;
	float rMin;
	float ds;
} PathPlannerParamsTypedef;


void SimulationLoop(PathFollowParamsTypedef &followPars, CSimpleInConnection &connection, tcp::iostream &client, ofstream &logFile);
bool ReceivePars(CSimpleInConnection& connection, deque<float> &pars);
void ForwardPath(CSimpleInConnection& connection, tcp::iostream& s);
int ReceiveRobotPosition(CSimpleInConnection& connection,float& leftJointPos, float& rightJointPos, Config& robotPos);
int SendRobotData(CSimpleInConnection& connection,float leftJointPos, float rightJointPos, Config& robotPos, Config& rabitPos, vector<float> info_val);
void ModelDifferentialMotors(float& leftSpeed, float& rightSpeed, float newLeftSpeed, float newRightSpeed, MotorParamsTypedef& leftMotor, MotorParamsTypedef& rightMotor, PathFollowParamsTypedef &pars);