#include "CarPathPlannerServer.h"
#include "CtrlMessage.h"
#include "App.h"

void CarParsePathFollowPars(deque<float> &parsIn, CarPathFollowParamsTypedef &parsOut)
{
	parsOut.TimeStep = parsIn.front(); parsIn.pop_front();

	parsOut.motor.motorMaxSpeed = parsIn.front(); parsIn.pop_front();
	parsOut.motor.motorMaxAccel = parsIn.front(); parsIn.pop_front();
	parsOut.motor.motorMultFactor = parsIn.front(); parsIn.pop_front();
	parsOut.motor.motorSmoothFactor = parsIn.front(); parsIn.pop_front();

	parsOut.MaxSteerSpeed = parsIn.front(); parsIn.pop_front();

	parsOut.PredictLength = parsIn.front(); parsIn.pop_front();
	parsOut.DistPar_P = parsIn.front(); parsIn.pop_front();
	parsOut.DistPar_D = parsIn.front(); parsIn.pop_front();
	parsOut.w0 = parsIn.front(); parsIn.pop_front();
	parsOut.ksi = parsIn.front(); parsIn.pop_front();

	parsOut.WheelDistance = parsIn.front(); parsIn.pop_front();
	parsOut.WheelDiameter = parsIn.front(); parsIn.pop_front();

	parsOut.PathPars.PathMaxVelocity = parsIn.front(); parsIn.pop_front();
	parsOut.PathPars.PathMaxAcceleration = parsIn.front(); parsIn.pop_front();

	parsOut.AxisDistance = parsIn.front(); parsIn.pop_front();
	parsOut.FiMax = parsIn.front(); parsIn.pop_front();
}

void CarParsePathPlannerPars(deque<float> &parsIn, CarPathPlannerParamsTypedef &parsOut)
{
	parsOut.app.appType = (App::AppTypedef)(int)parsIn.front(); parsIn.pop_front();

	CarParsePathFollowPars(parsIn, parsOut.PathFollow);

	parsOut.iterationMax = parsIn.front(); parsIn.pop_front();
	parsOut.fixPathProb = parsIn.front(); parsIn.pop_front();
	parsOut.roadmapProb = parsIn.front(); parsIn.pop_front();
	parsOut.randSeed = parsIn.front(); parsIn.pop_front();
	parsOut.envFile = parsIn.front(); parsIn.pop_front();
	parsOut.rMin = parsIn.front(); parsIn.pop_front();
	parsOut.ds = parsIn.front(); parsIn.pop_front();

	parsOut.reversePenaltyFactor = parsIn.front(); parsIn.pop_front();
	parsOut.useIntermediateS = parsIn.front(); parsIn.pop_front();
	parsOut.insertCount = parsIn.front(); parsIn.pop_front();
	parsOut.dx = parsIn.front(); parsIn.pop_front();
}

void CarForwardPathPlannerPars(tcp::iostream &client, CarPathPlannerParamsTypedef &pars)
{
	PackedMessage pathMsg;
	CarPathFollowParamsTypedef p = pars.PathFollow;

	pathMsg.values.push_back(pars.app.appType);					// robot type
	pathMsg.values.push_back(p.PredictLength);					// PredictLength
	pathMsg.values.push_back(p.DistPar_P);						// distPar_P
	pathMsg.values.push_back(p.DistPar_D);						// distPar_D
	pathMsg.values.push_back(p.w0);								// w0
	pathMsg.values.push_back(p.ksi);							// ksi
	pathMsg.values.push_back(p.TimeStep);						// timeStep
	pathMsg.values.push_back(p.WheelDistance);					// wheelDistance
	pathMsg.values.push_back(p.PathPars.PathMaxVelocity);		// pathMaxSpeed
	pathMsg.values.push_back(p.PathPars.PathMaxAcceleration);	// pathMaxAccel
	pathMsg.values.push_back(p.AxisDistance);					// axisDistance
	pathMsg.values.push_back(p.FiMax);							// fiMax

	pathMsg.values.push_back(pars.iterationMax);
	pathMsg.values.push_back(pars.fixPathProb);
	pathMsg.values.push_back(pars.roadmapProb);
	pathMsg.values.push_back(pars.randSeed);
	pathMsg.values.push_back(pars.envFile);
	pathMsg.values.push_back(pars.rMin);
	pathMsg.values.push_back(pars.ds);

	pathMsg.values.push_back(pars.reversePenaltyFactor);
	pathMsg.values.push_back(pars.useIntermediateS);
	pathMsg.values.push_back(pars.insertCount);
	pathMsg.values.push_back(pars.dx);

	pathMsg.send(client);
}

int CarPathPlannerServer(deque<float> &pars, CSimpleInConnection &connection, tcp::iostream &client, ofstream &logFile)
{
	cout << "Connected to PathPlanner Client." << endl;
	logFile << "Connected to PathPlanner Client." << endl;

	CarPathPlannerParamsTypedef serverPars;
	PathMessage path;

	CarParsePathPlannerPars(pars, serverPars);

	//Forward parameters to Client
	CarForwardPathPlannerPars(client, serverPars);

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

	CarSimulationLoop(serverPars.PathFollow, connection, client, logFile);

	client.close();
	return 0;
}