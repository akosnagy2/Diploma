#include "RobotPilotServer.h"
#include "PathPlannerServer.h"
#include "PathMessage.h"

void RobotLoop(CSimpleInConnection &connection, tcp::iostream &client, ofstream &logFile)
{
	// This is the server loop
	while (true)
	{ 
		Pos2dMessage pos_msg;

		//Receive result from RobotPilot Client
		if (!pos_msg.receive(client))
			break;

		logFile << "Robot pos: " <<  pos_msg.pos.p.x << ", " << pos_msg.pos.p.y << endl;

		//Send data to V-Rep Client
		vector<float> msg;
		msg.push_back(pos_msg.pos.p.x);
		msg.push_back(pos_msg.pos.p.y);
		msg.push_back(pos_msg.pos.phi);

		if (!connection.replyToReceivedData((char*)msg._Myfirst,msg.size()*sizeof(float)))
			break;		
	}
}

int RobotPilotServer(deque<float> &pars, CSimpleInConnection &connection, tcp::iostream &client, ofstream &logFile)
{
	cout << "Connected to RobotPilot Client." << endl;
	logFile << "Connected to RobotPilot Client." << endl;
	
	PathPlannerParamsTypedef serverPars;
	PathMessage path;

	ParsePathPlannerPars(pars, serverPars);

	//Forward parameters to Client
	ForwardPathPlannerPars(client, serverPars);		

	//Receive (sampled) path from PathPlanner
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

	RobotLoop(connection, client, logFile);

	client.close();
	return 0;
}
