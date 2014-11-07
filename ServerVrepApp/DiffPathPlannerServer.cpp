#include "DiffPathPlannerServer.h"
#include "CtrlMessage.h"

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

void ParsePathPlannerPars(deque<float> &parsIn, PathPlannerParamsTypedef &parsOut)
{
	parsOut.app.appType = (App::AppTypedef)(int)parsIn.front(); parsIn.pop_front();

	ParsePathFollowPars(parsIn, parsOut.PathFollow);

	parsOut.iterationMax = parsIn.front(); parsIn.pop_front();
	parsOut.fixPathProb = parsIn.front(); parsIn.pop_front();
	parsOut.roadmapProb = parsIn.front(); parsIn.pop_front();
	parsOut.randSeed = parsIn.front(); parsIn.pop_front();
	parsOut.envFile = parsIn.front(); parsIn.pop_front();
	parsOut.rMin = parsIn.front(); parsIn.pop_front();
	parsOut.ds = parsIn.front(); parsIn.pop_front();
}

void ForwardPathPlannerPars(tcp::iostream &client, PathPlannerParamsTypedef &pars)
{
	PackedMessage pathMsg;
	PathFollowParamsTypedef p = pars.PathFollow;

	pathMsg.values.push_back(pars.app.appType); // robot type
	pathMsg.values.push_back(p.PredictSampleLength);	//PredictSampleLength
	pathMsg.values.push_back(p.PredictDistanceLength);	//PredictDistanceLength
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
	pathMsg.values.push_back(pars.randSeed);
	pathMsg.values.push_back(pars.envFile);
	pathMsg.values.push_back(pars.rMin);
	pathMsg.values.push_back(pars.ds);

	pathMsg.send(client);
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

	if (serverPars.app.isPathFollow())
	{
		//Receive, forward path from V-Rep Client
		ForwardPath(connection, client);
	}

	if (serverPars.app.isPathPlanner())
	{
		//Receive (sampled) path from PathPlanner
		path.receive(client);
		
		vector<float> path_vrep = ConvertVrepPath(path);
	
		//Send (sampled) path to V-Rep Client
		if (!connection.replyToReceivedData((char*)path_vrep._Myfirst, path_vrep.size()*sizeof(float)))
			return -1;
	}

	//Receive (sampled) path from PathPlanner
	path.path.clear();
	path.receive(client);
		
	vector<float> path_vrep2 = ConvertVrepPath(path);
	
	//Send (sampled) path to V-Rep Client
	if (!connection.replyToReceivedData((char*)path_vrep2._Myfirst, path_vrep2.size()*sizeof(float)))
		return -1;

	SimulationLoop(serverPars.PathFollow, connection, client, logFile);

	client.close();
	return 0;
}