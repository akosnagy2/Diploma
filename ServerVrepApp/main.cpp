#include <iostream>
#include <fstream>
#include <string>
#include "boost\asio\ip\tcp.hpp"
#include "CtrlMessage.h"	
#include "Pos2dMessage.h"
#include "PathMessage.h"
#include "simpleInConnection.h"
#include "DiffPathPlannerServer.h"
#include "DiffRobotPilotServer.h"
#include "CarPathPlannerServer.h"
#include "CarRobotPilotServer.h"
#include "App.h"

#define _USE_MATH_DEFINES
#include <math.h>

using boost::asio::ip::tcp;
using namespace std;

boost::system::error_code SetupServer(boost::asio::io_service& io_service, int port, tcp::iostream& s)
{
	tcp::endpoint endpoint(tcp::v4(), port);
	tcp::acceptor acceptor(io_service, endpoint);

	boost::system::error_code ec;
	acceptor.accept(*s.rdbuf(), ec);

	return ec;
}

int main(int argc, char* argv[])
{
	int portNb = 0;
	boost::asio::io_service io_service;
	tcp::iostream client;
	boost::system::error_code e;
	ofstream logFile;

	if(argc == 2)
	{
		portNb = atoi(argv[1]);
	}
	else
	{
		cout << "Indicate following arguments: portNumber!" << endl;
		return -1;
	}

	cout << "Waiting for the Client..." << endl;
	if((e = SetupServer(io_service, 168, client)))
	{
		cout << "Client connection failed." << endl;
		cout << "Error Code: " << e.message() << endl;
		return -1;
	}
	cout << "Client connected." << endl;

	logFile.open("log.txt");
	cout << "Logfile: log.txt." << endl;
	logFile << "Log starting." << endl;

	CSimpleInConnection connection(portNb, 20000, 50, 47);
	logFile << "Connecting to client..." << endl;

	if(connection.connectToClient())
	{
		deque<float> vrepPars;

		//Receive all params from V-Rep
		ReceivePars(connection, vrepPars);

		//Switch App
		App app;
		app.appType = (App::AppTypedef) (int) vrepPars.front();

		switch(app.appType)
		{
			case App::DifferentialPathFollow:
				PathPlannerServer(vrepPars, connection, client, logFile);
				break;
			case App::DifferentialPathPlanner:
				PathPlannerServer(vrepPars, connection, client, logFile);
				break;
			case App::DifferentialRobotPilot:
				RobotPilotServer(vrepPars, connection, client, logFile);
				break;
			case App::CarPathFollow:
			case App::CarPathPlanner:
				CarPathPlannerServer(vrepPars, connection, client, logFile);
				break;
			case App::CarRobotPilotFollow:
			case App::CarRobotPilotPlanner:
				CarRobotPilotServer(vrepPars, connection, client, logFile);
				break;
			default:
				break;
		}
	}
	else
		logFile << "Failed to connect to client." << endl;

	logFile << "Log endig." << endl;
	logFile.close();

	cout << "Program exiting.";

	return 0;
}