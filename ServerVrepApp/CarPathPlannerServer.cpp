#include "CarPathPlannerServer.h"
#include "CtrlMessage.h"

void CarParsePathPlannerPars(deque<float> &parsIn, CarPathPlannerParamsTypedef &parsOut)
{
	CarParsePathFollowPars(parsIn, parsOut.PathFollow);

	parsOut.iterationMax = parsIn.front(); parsIn.pop_front();
	parsOut.fixPathProb = parsIn.front(); parsIn.pop_front();
	parsOut.roadmapProb = parsIn.front(); parsIn.pop_front();
	parsOut.randSeed = parsIn.front(); parsIn.pop_front();
	parsOut.envFile = parsIn.front(); parsIn.pop_front();
	parsOut.rMin = parsIn.front(); parsIn.pop_front();
	parsOut.ds = parsIn.front(); parsIn.pop_front();
}

void CarForwardPathPlannerPars(tcp::iostream &client, CarPathPlannerParamsTypedef &pars)
{
	PackedMessage pathMsg;
	CarPathFollowParamsTypedef p = pars.PathFollow;

	pathMsg.values.push_back(1.0f);								// robot type = car
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

	//Receive (sampled) path from PathPlanner
	path.receive(client);
		
	vector<float> path_vrep = ConvertVrepPath(path);
	
	//Send (sampled) path to V-Rep Client
	if (!connection.replyToReceivedData((char*)path_vrep._Myfirst, path_vrep.size()*sizeof(float)))
		return -1;

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