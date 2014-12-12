#include "CarRobotPilotServer.h"

#include "CarPathPlannerServer.h"
#include "SimpleSerial.h"
#include "App.h"

#include <string>
#include <iomanip>
#include <iostream>

int CarRobotPilotServer(deque<float> &pars, CSimpleInConnection &connection, tcp::iostream &client, ofstream &logFile)
{
	cout << "Connected to PathPlanner Client." << endl;
	logFile << "Connected to PathPlanner Client." << endl;

	CarPathPlannerParamsTypedef serverPars;
	PathMessage path;

	CarParsePathPlannerPars(pars, serverPars);

	//Forward parameters to Client
	CarForwardPathPlannerPars(client, serverPars);

	if(serverPars.app.isPathFollow())
	{
		//Receive, forward path from V-Rep Client
		ForwardPath(connection, client);
	}

	if(serverPars.app.isPathPlanner())
	{
		//Receive (sampled) path from PathPlanner
		path.receive(client);

		vector<float> path_vrep = ConvertVrepPath(path);

		//Send (sampled) path to V-Rep Client
		if(!connection.replyToReceivedData((char*) path_vrep._Myfirst, path_vrep.size()*sizeof(float)))
			return -1;
	}

	//Receive (sampled) path from PathPlanner
	path.path.clear();
	path.receive(client);

	vector<float> path_vrep2 = ConvertVrepPath(path);

	//Send (sampled) path to V-Rep Client
	if(!connection.replyToReceivedData((char*) path_vrep2._Myfirst, path_vrep2.size()*sizeof(float)))
		return -1;

	CarRobotPilotLoop(serverPars.PathFollow, connection, client, logFile);

	client.close();
	return 0;
}

void CarRobotPilotLoop(CarPathFollowParamsTypedef &followPars, CSimpleInConnection &connection, tcp::iostream &client, ofstream &logFile)
{
	bool init = true;
	SimpleSerial serial("COM5", 115200);
	std::ofstream posLog("poslog.txt");

	// This is the server loop
	while(true)
	{
		float leftJointPos;
		float rightJointPos;
		Config robotPos;
		CtrlMessage ctrl_msg;
		Pos2dMessage pos_msg;
		Pos2dMessage rabit_msg;
		PackedMessage info_msg;

		//Receive robot position from V-Rep Client
		if(ReceiveRobotPosition(connection, leftJointPos, rightJointPos, robotPos))
			break;

		if(init)
		{
			init = false;
			ostringstream setstr;
			setstr.precision(3);
			setstr << std::fixed << "S," << robotPos.p.x * 0.001 << "," << robotPos.p.y * 0.001 << "," << robotPos.phi;
			serial.writeLine(setstr.str());
		}

		std::string serialPos = serial.readLine();
		sscanf(serialPos.c_str(), "O,%f,%f,%f\r\n", &robotPos.p.x, &robotPos.p.y, &robotPos.phi);

		//Send robot position to the PathFollow Client
		robotPos.p.x *= 1000.0f;
		robotPos.p.y *= 1000.0f;
		pos_msg.pos = robotPos;
		pos_msg.send(client);

		//Receive result from PathFollow Client
		if(!ctrl_msg.receive(client))
			break;
		if(!rabit_msg.receive(client))
			break;
		if(!info_msg.receive(client))
			break;

		float v = (float) ctrl_msg.ctrl_sig[0];
		float fi = (float) ctrl_msg.ctrl_sig[1];

		ostringstream strs;
		strs.precision(3);
		strs << std::fixed << "I," << v*0.001f << "," << fi;
		serial.writeLine(strs.str());

		//Send data to V-Rep Client
		if(SendRobotData(connection, v, fi, robotPos, rabit_msg.pos, info_msg.values))
			break;

		posLog << robotPos.p.x << " " << robotPos.p.y << std::endl;
	}
}