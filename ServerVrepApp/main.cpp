#include <iostream>
#include <fstream>
#include <string>
#include "boost\asio\ip\tcp.hpp"
#include "CtrlMessage.h"	
#include "Pos2dMessage.h"
#include "PathMessage.h"
#include "simpleInConnection.h"

#define _USE_MATH_DEFINES
#include <math.h>

using boost::asio::ip::tcp;
using namespace std;

typedef struct 
{
	 float motorMaxSpeed;
	 float motorMaxAccel;
	 float motorSmoothFactor;
	 float motorMultFactor;
} MotorParamsTypedef;

typedef struct
{
	float PathMaxVelocity;
	float PathMaxAcceleration;
	float PathMaxTangentAcceleration;
	float PathMaxAngularVelocity;
} PathParamsTypedef;

typedef struct
{
	float TimeStep;
	MotorParamsTypedef LeftMotor;
	MotorParamsTypedef RightMotor;
	float WheelDiameter;
	float WheelBase;
	float PredictLength;
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
	float envFile;
} PathPlannerParamsTypedef;

ofstream dFile[6];

boost::system::error_code SetupServer(boost::asio::io_service& io_service, int port, tcp::iostream& s)
{
	tcp::endpoint endpoint(tcp::v4(), port);	
	tcp::acceptor acceptor(io_service, endpoint);

	boost::system::error_code ec;
	acceptor.accept(*s.rdbuf(), ec);
	
	return ec;
}

void ForwardPath(CSimpleInConnection& connection, tcp::iostream& s)
{
	int receivedDataLength;
	char* receivedData=connection.receiveData(receivedDataLength);
	if (receivedDataLength)
	{
		PathMessage path;
		for (int i = 0; i < receivedDataLength / (sizeof(float) * 3); i++)
		{
			Config pos;
			pos.p.x = ((float*)receivedData)[3*i];
			pos.p.y = ((float*)receivedData)[3*i + 1];
			pos.phi = ((float*)receivedData)[3*i + 2];
			path.path.push_back(pos);
		}
		path.send(s);
	}
	delete[] receivedData;
}

void ParsePathFollowPars(deque<float> &parsIn, PathFollowParamsTypedef &parsOut)
{
	parsOut.TimeStep = parsIn.front(); parsIn.pop_front();

	parsOut.LeftMotor.motorMaxSpeed = parsIn.front(); parsIn.pop_front();
	parsOut.LeftMotor.motorMaxAccel = parsIn.front(); parsIn.pop_front();
	parsOut.LeftMotor.motorMultFactor = parsIn.front(); parsIn.pop_front();
	parsOut.LeftMotor.motorSmoothFactor = parsIn.front(); parsIn.pop_front();

	parsOut.RightMotor.motorMaxSpeed = parsIn.front(); parsIn.pop_front();
	parsOut.RightMotor.motorMaxAccel = parsIn.front(); parsIn.pop_front();
	parsOut.RightMotor.motorMultFactor = parsIn.front(); parsIn.pop_front();
	parsOut.RightMotor.motorSmoothFactor = parsIn.front(); parsIn.pop_front();

	parsOut.PredictLength = parsIn.front(); parsIn.pop_front();

	parsOut.DistPar_P = parsIn.front(); parsIn.pop_front();
	parsOut.DistPar_D = parsIn.front(); parsIn.pop_front();
	parsOut.OriPar_P = parsIn.front(); parsIn.pop_front();
	parsOut.OriPar_D = parsIn.front(); parsIn.pop_front();

	parsOut.WheelBase = parsIn.front(); parsIn.pop_front();
	parsOut.WheelDiameter = parsIn.front(); parsIn.pop_front();
	
	parsOut.PathPars.PathMaxVelocity = parsIn.front(); parsIn.pop_front();
	parsOut.PathPars.PathMaxAcceleration = parsIn.front(); parsIn.pop_front();
	parsOut.PathPars.PathMaxTangentAcceleration = parsIn.front(); parsIn.pop_front();
	parsOut.PathPars.PathMaxAngularVelocity = parsIn.front(); parsIn.pop_front();
}

void ForwardPathFollowPars(tcp::iostream &client, PathFollowParamsTypedef &pars)
{
	PackedMessage pathMsg;

	pathMsg.values.push_back(pars.PredictLength);	//PredictLength
	pathMsg.values.push_back(pars.DistPar_P);	//distPar_P
	pathMsg.values.push_back(pars.DistPar_D);	//distPar_D
	pathMsg.values.push_back(pars.OriPar_P);	//oriPar_P
	pathMsg.values.push_back(pars.OriPar_D);	//oriPar_D
	pathMsg.values.push_back(pars.TimeStep);	//timeStep
	pathMsg.values.push_back(pars.WheelBase);	//wheelDistance
	pathMsg.values.push_back(pars.PathPars.PathMaxVelocity);	//pathMaxSpeed
	pathMsg.values.push_back(pars.PathPars.PathMaxAcceleration);	//pathMaxAccel
	pathMsg.values.push_back(pars.PathPars.PathMaxTangentAcceleration);	//pathMaxTangentAccel
	pathMsg.values.push_back(pars.PathPars.PathMaxAngularVelocity);	//pathMaxAngularSpeed

	pathMsg.send(client);
}

int ReceiveRobotPosition(CSimpleInConnection& connection,float& leftJointPos, float& rightJointPos, Config& robotPos)
{
	int receivedDataLength;
	char* receivedData=connection.receiveData(receivedDataLength);
	if (receivedData!=NULL)
	{ 
		leftJointPos = ((float*)receivedData)[0];
		rightJointPos = ((float*)receivedData)[1];
		robotPos.p.x = ((float*)receivedData)[2];
		robotPos.p.y = ((float*)receivedData)[3];
		robotPos.phi = ((float*)receivedData)[4];

		delete[] receivedData;
		return 0;
	}

	return -1;
}

int SendRobotData(CSimpleInConnection& connection,float leftJointPos, float rightJointPos, Config& robotPos, Config& rabitPos, float errorSum, float distError)
{
	float msg[9];

	msg[0] = robotPos.p.x;
	msg[1] = robotPos.p.y;
	msg[2] = robotPos.phi;
	msg[3] = leftJointPos;
	msg[4] = rightJointPos;
	msg[5] = rabitPos.p.x;
	msg[6] = rabitPos.p.y;
	msg[7] = errorSum;
	msg[8] = distError;
	
	if (!connection.replyToReceivedData((char*)msg,sizeof(msg)))
		return -1;

	return 0;
}

void ModelMotors(float& leftSpeed, float& rightSpeed, float newLeftSpeed, float newRightSpeed, MotorParamsTypedef& leftMotor, MotorParamsTypedef& rightMotor, PathFollowParamsTypedef &pars)
{
	float prevLeftSpeed = leftSpeed;
	float prevRightSpeed = rightSpeed;

	leftSpeed += (newLeftSpeed - leftSpeed) * leftMotor.motorSmoothFactor;
	rightSpeed += (newRightSpeed - rightSpeed) * rightMotor.motorSmoothFactor; 
	leftSpeed *= leftMotor.motorMultFactor;
	rightSpeed *= rightMotor.motorMultFactor;

	if (leftSpeed > leftMotor.motorMaxSpeed)
		leftSpeed = leftMotor.motorMaxSpeed;
	if (leftSpeed < -leftMotor.motorMaxSpeed)
		leftSpeed = -leftMotor.motorMaxSpeed;

	if (rightSpeed > rightMotor.motorMaxSpeed)
		rightSpeed = rightMotor.motorMaxSpeed;
	if (rightSpeed < -rightMotor.motorMaxSpeed)
		rightSpeed = -rightMotor.motorMaxSpeed;

	dFile[2] << leftSpeed << endl;
	dFile[3] << rightSpeed << endl;

	float leftAccel = (leftSpeed - prevLeftSpeed) / pars.TimeStep;
	float rightAccel = (rightSpeed - prevRightSpeed) / pars.TimeStep;

	if (leftAccel > leftMotor.motorMaxAccel)
		leftSpeed = prevLeftSpeed + leftMotor.motorMaxAccel * pars.TimeStep;
	if (leftAccel < -leftMotor.motorMaxAccel)
		leftSpeed = prevLeftSpeed - leftMotor.motorMaxAccel * pars.TimeStep;

	if (rightAccel > rightMotor.motorMaxAccel)
		rightSpeed = prevRightSpeed + rightMotor.motorMaxAccel * pars.TimeStep;
	if (rightAccel < -rightMotor.motorMaxAccel)
		rightSpeed = prevRightSpeed - rightMotor.motorMaxAccel * pars.TimeStep;

	dFile[4] <<  (leftSpeed - prevLeftSpeed) / pars.TimeStep << endl;
	dFile[5] << (rightSpeed - prevRightSpeed) / pars.TimeStep << endl;
}


bool ReceivePars(CSimpleInConnection& connection, deque<float> &pars)
{
	int len;
	char* receivedData = connection.receiveData(len);

	if (len)
	{
		for (int i = 0; i < len/sizeof(float); i++)		
			pars.push_back( ((float*)receivedData)[i] );		

		delete[] receivedData;
		return true;
	}

	return false;
}

void ForwardPathPlannerPars(tcp::iostream &client, PathPlannerParamsTypedef &pars)
{
	PackedMessage pathMsg;
	PathFollowParamsTypedef p = pars.PathFollow;

	pathMsg.values.push_back(p.PredictLength);	//PredictLength
	pathMsg.values.push_back(p.DistPar_P);	//distPar_P
	pathMsg.values.push_back(p.DistPar_D);	//distPar_D
	pathMsg.values.push_back(p.OriPar_P);	//oriPar_P
	pathMsg.values.push_back(p.OriPar_D);	//oriPar_D
	pathMsg.values.push_back(p.TimeStep);	//timeStep
	pathMsg.values.push_back(p.WheelBase);	//wheelDistance
	pathMsg.values.push_back(p.PathPars.PathMaxVelocity);	//pathMaxSpeed
	pathMsg.values.push_back(p.PathPars.PathMaxAcceleration);	//pathMaxAccel
	pathMsg.values.push_back(p.PathPars.PathMaxTangentAcceleration);	//pathMaxTangentAccel
	pathMsg.values.push_back(p.PathPars.PathMaxAngularVelocity);	//pathMaxAngularSpeed
	pathMsg.values.push_back(pars.iterationMax);
	pathMsg.values.push_back(pars.fixPathProb);
	pathMsg.values.push_back(pars.roadmapProb);
	pathMsg.values.push_back(pars.envFile);

	pathMsg.send(client);
}

void ParsePathPlannerPars(deque<float> &parsIn, PathPlannerParamsTypedef &parsOut)
{
	ParsePathFollowPars(parsIn, parsOut.PathFollow);

	parsOut.iterationMax = parsIn.front(); parsIn.pop_front();
	parsOut.fixPathProb = parsIn.front(); parsIn.pop_front();
	parsOut.roadmapProb = parsIn.front(); parsIn.pop_front();

	parsOut.envFile = parsIn.front(); parsIn.pop_front();
}

void SimulationLoop(PathFollowParamsTypedef &followPars, CSimpleInConnection &connection, tcp::iostream &client, ofstream &logFile)
{
	int timeIndex = 0;
	float leftSpeed = 0.0f;
	float rightSpeed = 0.0f;
	float robotSpeed;
	float prevRobotSpeed = 0.0f;
	float robotAngularSpeed;

	// This is the server loop
	while (true)
	{ 
		float leftJointPos;
		float rightJointPos;
		Config robotPos;
		CtrlMessage ctrl_msg;
		Pos2dMessage pos_msg;
		Pos2dMessage rabit_msg;
		PackedMessage info_msg;

		//Receive robot position from V-Rep Client
		if (ReceiveRobotPosition(connection,leftJointPos,rightJointPos,robotPos))
			break;

		//Send robot position to the PathFollow Client
		pos_msg.pos = robotPos;
		pos_msg.send(client);

		//Receive result from PathFollow Client
		ctrl_msg.receive(client);	
		rabit_msg.receive(client);
		info_msg.receive(client);

		//Apply motor models
		ModelMotors(leftSpeed, rightSpeed,(float)ctrl_msg.ctrl_sig[0], (float)ctrl_msg.ctrl_sig[1], followPars.LeftMotor, followPars.RightMotor, followPars);

		logFile << "Robot target left vel.: " << leftSpeed << ", right vel.: " << rightSpeed << endl;
		logFile << "Simulation timeindex: " << timeIndex++ << endl;
		logFile << "Robot pos: " <<  robotPos.p.x << ", " << robotPos.p.y << endl;
		logFile << "Robot ori: " <<  robotPos.phi << endl;

		//Wheels -> Robot
		robotSpeed = (leftSpeed + rightSpeed) / 2.0f;
		robotAngularSpeed = (rightSpeed - leftSpeed) / followPars.WheelBase;

		dFile[0] << robotSpeed << endl;
		dFile[1] << (robotSpeed - prevRobotSpeed) / followPars.TimeStep << endl;
		prevRobotSpeed = robotSpeed;

		//Position
		robotPos.phi += robotAngularSpeed * followPars.TimeStep / 2.0f;
		robotPos.p.x += cos(robotPos.phi) * robotSpeed * followPars.TimeStep;
 		robotPos.p.y += sin(robotPos.phi) * robotSpeed * followPars.TimeStep;
		robotPos.phi += robotAngularSpeed  * followPars.TimeStep / 2.0f;

		leftJointPos += leftSpeed*followPars.TimeStep*2/followPars.WheelDiameter;
		rightJointPos += rightSpeed*followPars.TimeStep*2/followPars.WheelDiameter;

		//Send data to V-Rep Client
		if (SendRobotData(connection,leftJointPos, rightJointPos, robotPos, rabit_msg.pos, (float)info_msg.values[0], (float)info_msg.values[1]))
			break;
	}
}

int PathPlannerServer(deque<float> &pars, CSimpleInConnection &connection, tcp::iostream &client, ofstream &logFile)
{
	cout << "Connected to PathPlanner Client." << endl;
	logFile << "Connected to PathPlanner Client." << endl;
	
	PathPlannerParamsTypedef serverPars;
	PathMessage path;

	ParsePathPlannerPars(pars, serverPars);

	//Forward parameters to Client
	ForwardPathPlannerPars(client, serverPars);		

	//Receive (sampled) path from PathPlanner
	path.receive(client);
		
	float* path_f = new float[path.path.size()*2];
	for (int i = 0; i < path.path.size(); i++)
	{
		path_f[i*2] = path.path[i].p.x;
		path_f[i*2 + 1] = path.path[i].p.y;
	}		

	//Send (sampled) path to V-Rep Client
	if (!connection.replyToReceivedData((char*)path_f,path.path.size() * 2 * sizeof(path_f)))
		return -1;

	delete[] path_f;

	SimulationLoop(serverPars.PathFollow, connection, client, logFile);

	client.close();
	return 0;
}

int RobotPilotServer(deque<float> &pars, CSimpleInConnection &connection, tcp::iostream &client, ofstream &logFile)
{
	return -1;
}

int PathFollowServer(deque<float> &pars, CSimpleInConnection &connection, tcp::iostream &client, ofstream &logFile)
{
	cout << "Connected to PathFollow Client." << endl;
	logFile << "Connected to PathFollow Client." << endl;
	int timeIndex = 0;
	float leftSpeed = 0.0f;
	float rightSpeed = 0.0f;
	float prevRobotSpeed = 0.0f;
	
	PathFollowParamsTypedef serverPars;
	PathMessage path;

	ParsePathFollowPars(pars, serverPars);

	//Forward parameters to Client
	ForwardPathFollowPars(client, serverPars);		
		
	//Receive, forward path from V-Rep Client
	ForwardPath(connection, client);

	//Receive (sampled) path from PathFollow
	path.receive(client);
		
	float* path_f = new float[path.path.size()*2];
	for (int i = 0; i < path.path.size(); i++)
	{
		path_f[i*2] = path.path[i].p.x;
		path_f[i*2 + 1] = path.path[i].p.y;
	}		

	//Send (sampled) path to V-Rep Client
	if (!connection.replyToReceivedData((char*)path_f,path.path.size() * 2 * sizeof(path_f)))
		return -1;

	delete[] path_f;

	SimulationLoop(serverPars, connection, client, logFile);

	client.close();

	return 0;
}

int main(int argc,char* argv[])
{
	int portNb=0;
	boost::asio::io_service io_service;
	tcp::iostream client;
	boost::system::error_code e;
	ofstream logFile;
	
	if (argc == 2)
	{
		portNb = atoi(argv[1]);
	}
	else
	{
		cout << "Indicate following arguments: portNumber!" << endl;
		return -1;
	}

	cout << "Waiting for the Client..." << endl;
	if ((e = SetupServer(io_service, 168, client)))
	{
		cout << "Client connection failed." << endl;
		cout << "Error Code: " << e.message() << endl;
		return -1;
	}
	cout << "Client connected." << endl;

	logFile.open ("log.txt");
	cout << "Logfile: log.txt." << endl;
	logFile << "Log starting." << endl;
	
	CSimpleInConnection connection(portNb, 20000, 50, 47);
	logFile << "Connecting to client..." << endl;

	dFile[0].open("robotV.txt");
	dFile[1].open("robotA.txt");
	dFile[2].open("robotVL.txt");
	dFile[3].open("robotVR.txt");
	dFile[4].open("robotAL.txt");
	dFile[5].open("robotAR.txt");

	if (connection.connectToClient())
	{
		deque<float> vrepPars;
		
		//Receive all params from V-Rep
		ReceivePars(connection, vrepPars);	

		//Switch App
		int app = (int)vrepPars.front();
		vrepPars.pop_front();
		if (app == 2)
			PathFollowServer(vrepPars, connection, client, logFile);
		else if (app == 4)
			PathPlannerServer(vrepPars, connection, client, logFile);
		else if (app == 8)
			RobotPilotServer(vrepPars, connection, client, logFile);
		
	}
	else
		logFile << "Failed to connect to client." << endl ;

	logFile << "Log endig." << endl;
	logFile.close();

	dFile[0].close();
	dFile[1].close();
	dFile[2].close();
	dFile[3].close();
	dFile[4].close();
	dFile[5].close();
	
	cout << "Program exiting.";

	return 0;
}