#include <iostream>
#include <fstream>
#include <string>
#include <boost/asio.hpp>
#include "CtrlMessage.h"	
#include "Pos2dMessage.h"
#include "PathMessage.h"
#include "simpleInConnection.h"

#define _USE_MATH_DEFINES
#include <math.h>

#define CAR_LIKE_ROBOT 1

using boost::asio::ip::tcp;
using namespace std;

typedef struct 
{
	 float motorMaxSpeed;
	 float motorMaxAccel;
	 float motorSmoothFactor;
	 float motorMultFactor;
} motorStruct;

typedef struct
{
	float axisDistance;
	float minimalTurnRadius;
	float fiMax;
} carStruct;

motorStruct robotLeftMotor;
motorStruct robotRightMotor;
carStruct carData;
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
	if (receivedData!=NULL)
	{
		PathMessage path;
		for (int i = 0; i < receivedDataLength / (sizeof(float) * 3); i++)
		{
			Position pos;
			pos.x = ((float*)receivedData)[3*i];
			pos.y = ((float*)receivedData)[3*i + 1];
			pos.phi = ((float*)receivedData)[3*i + 2];
			path.path.push_back(pos);
		}
		path.send(s);
	}
	delete[] receivedData;
}

int ReceiveForwardPars(CSimpleInConnection& connection, tcp::iostream& path_s, float& dt, motorStruct& motor, carStruct& car, float& wheelDistance, float& wheelDiameter) {
	int receivedDataLength;
	PackedMessage pathMsg;
	char* receivedData = connection.receiveData(receivedDataLength);
	if (receivedData != NULL)
	{
		dt = ((float*)receivedData)[0];

		motor.motorMaxSpeed = ((float*)receivedData)[1];
		motor.motorMaxAccel = ((float*)receivedData)[2];
		motor.motorMultFactor = ((float*)receivedData)[3];
		motor.motorSmoothFactor = ((float*)receivedData)[4];

		pathMsg.values.push_back(((float*)receivedData)[9]);	//PredictLength
		pathMsg.values.push_back(((float*)receivedData)[10]);	//distPar_P
		pathMsg.values.push_back(((float*)receivedData)[11]);	//distPar_D
		pathMsg.values.push_back(((float*)receivedData)[12]);	//oriPar_P
		pathMsg.values.push_back(((float*)receivedData)[13]);	//oriPar_D
		wheelDistance = ((float*)receivedData)[14];
		wheelDiameter = ((float*)receivedData)[15];
		pathMsg.values.push_back(dt);	//timeStep
		pathMsg.values.push_back(wheelDistance);	//wheelDistance
		pathMsg.values.push_back(((float*)receivedData)[16]);	//pathMaxSpeed
		pathMsg.values.push_back(((float*)receivedData)[17]);	//pathMaxAccel
		pathMsg.values.push_back(((float*)receivedData)[18]);	//pathMaxTangentAccel
		pathMsg.values.push_back(((float*)receivedData)[19]);	//pathMaxAngularSpeed

		car.axisDistance = ((float*)receivedData)[20];
		car.minimalTurnRadius = ((float*)receivedData)[21];
		car.fiMax = ((float*)receivedData)[22];

		pathMsg.send(path_s);

		delete[] receivedData;
		return 0;
	}

	return -1;
}

int ReceiveForwardPars(CSimpleInConnection& connection,tcp::iostream& path_s, float& dt, motorStruct& leftMotor, motorStruct& rightMotor, float& wheelDistance, float& wheelDiameter)
{
	int receivedDataLength;
	PackedMessage pathMsg;
	char* receivedData=connection.receiveData(receivedDataLength);
	if (receivedData!=NULL)
	{
		dt = ((float*)receivedData)[0];
		
		leftMotor.motorMaxSpeed = ((float*)receivedData)[1];
		leftMotor.motorMaxAccel = ((float*)receivedData)[2];
		leftMotor.motorMultFactor = ((float*)receivedData)[3];
		leftMotor.motorSmoothFactor = ((float*)receivedData)[4];

		rightMotor.motorMaxSpeed = ((float*)receivedData)[5];
		rightMotor.motorMaxAccel = ((float*)receivedData)[6];
		rightMotor.motorMultFactor = ((float*)receivedData)[7];
		rightMotor.motorSmoothFactor = ((float*)receivedData)[8];

		pathMsg.values.push_back(((float*)receivedData)[9]);	//PredictLength
		pathMsg.values.push_back(((float*)receivedData)[10]);	//distPar_P
		pathMsg.values.push_back(((float*)receivedData)[11]);	//distPar_D
		pathMsg.values.push_back(((float*)receivedData)[12]);	//oriPar_P
		pathMsg.values.push_back(((float*)receivedData)[13]);	//oriPar_D
		wheelDistance = ((float*)receivedData)[14];
		wheelDiameter = ((float*)receivedData)[15];
		pathMsg.values.push_back(dt);	//timeStep
		pathMsg.values.push_back(wheelDistance);	//wheelDistance
		pathMsg.values.push_back(((float*)receivedData)[16]);	//pathMaxSpeed
		pathMsg.values.push_back(((float*)receivedData)[17]);	//pathMaxAccel
		pathMsg.values.push_back(((float*)receivedData)[18]);	//pathMaxTangentAccel
		pathMsg.values.push_back(((float*)receivedData)[19]);	//pathMaxAngularSpeed

		pathMsg.send(path_s);

		delete[] receivedData;
		return 0;
	}

	return -1;
}

int ReceiveRobotPosition(CSimpleInConnection& connection,float& leftJointPos, float& rightJointPos, Position& robotPos)
{
	int receivedDataLength;
	char* receivedData=connection.receiveData(receivedDataLength);
	if (receivedData!=NULL)
	{ 
		leftJointPos = ((float*)receivedData)[0];
		rightJointPos = ((float*)receivedData)[1];
		robotPos.x = ((float*)receivedData)[2];
		robotPos.y = ((float*)receivedData)[3];
		robotPos.phi = ((float*)receivedData)[4];

		delete[] receivedData;
		return 0;
	}

	return -1;
}

int SendRobotData(CSimpleInConnection& connection,float leftJointPos, float rightJointPos, Position& robotPos, Position& rabitPos, float errorSum, float distError)
{
	float msg[9];

	msg[0] = robotPos.x;
	msg[1] = robotPos.y;
	msg[2] = robotPos.phi;
	msg[3] = leftJointPos;
	msg[4] = rightJointPos;
	msg[5] = rabitPos.x;
	msg[6] = rabitPos.y;
	msg[7] = errorSum;
	msg[8] = distError;
	
	if (!connection.replyToReceivedData((char*)msg,sizeof(msg)))
		return -1;

	return 0;
}

void ModelMotors(float& leftSpeed, float& rightSpeed, float newLeftSpeed, float newRightSpeed, motorStruct& leftMotor, motorStruct& rightMotor, float timeStep)
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

	float leftAccel = (leftSpeed - prevLeftSpeed) / timeStep;
	float rightAccel = (rightSpeed - prevRightSpeed) / timeStep;

	if (leftAccel > leftMotor.motorMaxAccel)
		leftSpeed = prevLeftSpeed + leftMotor.motorMaxAccel * timeStep;
	if (leftAccel < -leftMotor.motorMaxAccel)
		leftSpeed = prevLeftSpeed - leftMotor.motorMaxAccel * timeStep;

	if (rightAccel > rightMotor.motorMaxAccel)
		rightSpeed = prevRightSpeed + rightMotor.motorMaxAccel * timeStep;
	if (rightAccel < -rightMotor.motorMaxAccel)
		rightSpeed = prevRightSpeed - rightMotor.motorMaxAccel * timeStep;

	dFile[4] <<  (leftSpeed - prevLeftSpeed) / timeStep << endl;
	dFile[5] << (rightSpeed - prevRightSpeed) / timeStep << endl;
}

int main(int argc,char* argv[])
{
	int portNb=0;
	boost::asio::io_service io_service;
	tcp::iostream path_s;
	boost::system::error_code path_e;
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

	cout << "Waiting for the PathFollow Client..." << endl;
	if ((path_e = SetupServer(io_service,168,path_s)))
	{
		cout << "PathFollow Client connection failed." << endl;
		cout << "Error Code: " << path_e.message() << endl;
		return -1;
	}
	cout << "PathFollow Client connected." << endl;

	logFile.open ("log.txt");
	cout << "Logfile: log.txt." << endl;
	logFile << "Log starting." << endl;
	
	CSimpleInConnection connection(portNb,20000,50,47);
	logFile << "Connecting to client..." << endl;

	dFile[0].open("robotV.txt");
	dFile[1].open("robotA.txt");
	dFile[2].open("robotVL.txt");
	dFile[3].open("robotVR.txt");
	dFile[4].open("robotAL.txt");
	dFile[5].open("robotAR.txt");

	if (connection.connectToClient())
	{
		logFile << "Connected with client." << endl;
		float wheelDiameter = 85.0f;
		float wheelDistance = 229.2f;
		float timeStep;
		int timeIndex = 0;
		float leftSpeed = 0.0f;
		float rightSpeed = 0.0f;
		float robotSpeed;
		float prevRobotSpeed = 0.0f;
		float robotAngularSpeed;
		PathMessage path;

		//Receive parameters from V-Rep Client
#if CAR_LIKE_ROBOT
		ReceiveForwardPars(connection, path_s, timeStep, robotLeftMotor, carData, wheelDistance, wheelDiameter);
#else
		ReceiveForwardPars(connection, path_s, timeStep, robotLeftMotor, robotRightMotor, wheelDistance, wheelDiameter);
#endif
		
		//Receive path from V-Rep Client
		ForwardPath(connection, path_s);

		//Receive (sampled) path from PathFollow
		path.receive(path_s);
		
		float* path_f = new float[path.path.size()*2];
		for (int i = 0; i < path.path.size(); i++)
		{
			path_f[i*2] = path.path[i].x;
			path_f[i*2 + 1] = path.path[i].y;
		}		

		//Send (sampled) path to V-Rep Client
		if (!connection.replyToReceivedData((char*)path_f,path.path.size() * 2 * sizeof(path_f)))
			return -1;

		delete[] path_f;

		// This is the server loop
		while (true)
		{ 
			float leftJointPos;
			float rightJointPos;
			Position robotPos;
			CtrlMessage ctrl_msg;
			Pos2dMessage pos_msg;
			Pos2dMessage rabit_msg;
			PackedMessage info_msg;

			//Receive robot position from V-Rep Client
			ReceiveRobotPosition(connection,leftJointPos,rightJointPos,robotPos);
				
			//Send robot position to the PathFollow Client
			pos_msg.pos = robotPos;
			pos_msg.send(path_s);

			//Receive result from PathFollow Client
			ctrl_msg.receive(path_s);	
			rabit_msg.receive(path_s);
			info_msg.receive(path_s);

			//Apply motor models
			ModelMotors(leftSpeed,rightSpeed,(float)ctrl_msg.ctrl_sig[0], (float)ctrl_msg.ctrl_sig[1],robotLeftMotor,robotRightMotor,timeStep);

			logFile << "Robot target left vel.: " << leftSpeed << ", right vel.: " << rightSpeed << endl;
			logFile << "Simulation timeindex: " << timeIndex++ << endl;
			logFile << "Robot pos: " <<  robotPos.x << ", " << robotPos.y << endl;
			logFile << "Robot ori: " <<  robotPos.phi << endl;

			//Wheels -> Robot
			robotSpeed = (leftSpeed + rightSpeed) / 2.0f;
			robotAngularSpeed = (rightSpeed - leftSpeed) / wheelDistance;

			dFile[0] << robotSpeed << endl;
			dFile[1] << (robotSpeed - prevRobotSpeed) / timeStep << endl;
			prevRobotSpeed = robotSpeed;

			//Position
			robotPos.phi += robotAngularSpeed * timeStep / 2.0f;
			robotPos.x += cos(robotPos.phi) * robotSpeed * timeStep;
 			robotPos.y += sin(robotPos.phi) * robotSpeed * timeStep;
			robotPos.phi += robotAngularSpeed  * timeStep / 2.0f;

			//TODO a végén a pozíciót nekem elõrébb kell rakni a tengelytáv felével

			leftJointPos += leftSpeed*timeStep*2/wheelDiameter;
			rightJointPos += rightSpeed*timeStep*2/wheelDiameter;

			//Send data to V-Rep Client
			SendRobotData(connection,leftJointPos, rightJointPos, robotPos, rabit_msg.pos, (float)info_msg.values[0], (float)info_msg.values[1]);
		}
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

	path_s.close();
	
	cout << "Program exiting.";

	return 0;
}