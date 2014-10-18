#include "PathFollowServer.h"

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

	parsOut.PredictSampleLength = parsIn.front(); parsIn.pop_front();
	parsOut.PredictDistanceLength = parsIn.front(); parsIn.pop_front();

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

	pathMsg.values.push_back(0.0f);		
	pathMsg.values.push_back(pars.PredictSampleLength);	//PredictSampleLength
	pathMsg.values.push_back(pars.PredictDistanceLength);	//PredictDistanceLength
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
	
	vector<float> path_vrep;
	for (auto &s : path.path)
	{
		for (auto &p : s.path)
		{
			path_vrep.push_back(p.p.x);
			path_vrep.push_back(p.p.y);
		}
	}
	
	//Send (sampled) path to V-Rep Client
	if (!connection.replyToReceivedData((char*)path_vrep._Myfirst, path_vrep.size()*sizeof(float)))
		return -1;

	SimulationLoop(serverPars, connection, client, logFile);

	client.close();

	return 0;
}

