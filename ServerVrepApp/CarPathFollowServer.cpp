#include "CarPathFollowServer.h"
#include "CarLikeRobot.h"
#include "Geometry/Config.h"

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

void CarForwardPathFollowPars(tcp::iostream &client, CarPathFollowParamsTypedef &pars)
{
	PackedMessage pathMsg;

	pathMsg.values.push_back(1.0f);									// robot type = car
	pathMsg.values.push_back(pars.PredictLength);					// PredictLength
	pathMsg.values.push_back(pars.DistPar_P);						// distPar_P
	pathMsg.values.push_back(pars.DistPar_D);						// distPar_D
	pathMsg.values.push_back(pars.w0);								// w0
	pathMsg.values.push_back(pars.ksi);								// ksi
	pathMsg.values.push_back(pars.TimeStep);						// timeStep
	pathMsg.values.push_back(pars.WheelDistance);					// wheelDistance
	pathMsg.values.push_back(pars.PathPars.PathMaxVelocity);		// pathMaxSpeed
	pathMsg.values.push_back(pars.PathPars.PathMaxAcceleration);	// pathMaxAccel
	pathMsg.values.push_back(pars.AxisDistance);					// axisDistance
	pathMsg.values.push_back(pars.FiMax);							// fiMax

	pathMsg.send(client);
}

int CarPathFollowServer(deque<float> &pars, CSimpleInConnection &connection, tcp::iostream &client, ofstream &logFile)
{
	cout << "Connected to PathFollow Client." << endl;
	logFile << "Connected to PathFollow Client." << endl;
	int timeIndex = 0;
	float leftSpeed = 0.0f;
	float rightSpeed = 0.0f;
	float prevRobotSpeed = 0.0f;

	CarPathFollowParamsTypedef serverPars;
	PathMessage path;

	CarParsePathFollowPars(pars, serverPars);

	//Forward parameters to Client
	CarForwardPathFollowPars(client, serverPars);

	//Receive, forward path from V-Rep Client
	ForwardPath(connection, client);

	//Receive (sampled) path from PathFollow
	path.receive(client);

	vector<float> path_vrep;
	for(auto &s : path.path) {
		for(auto &p : s.path) {
			path_vrep.push_back(p.p.x);
			path_vrep.push_back(p.p.y);
		}
	}

	//Send (sampled) path to V-Rep Client
	if(!connection.replyToReceivedData((char*) path_vrep._Myfirst, path_vrep.size()*sizeof(float)))
		return -1;

	CarSimulationLoop(serverPars, connection, client, logFile);

	client.close();

	return 0;
}

void CarSimulationLoop(CarPathFollowParamsTypedef &followPars, CSimpleInConnection &connection, tcp::iostream &client, ofstream &logFile)
{
	bool init = true;
	CarLikeRobot car;
	car.setParameters(followPars.motor.motorMaxAccel, followPars.motor.motorMaxSpeed,
		followPars.motor.motorSmoothFactor, followPars.motor.motorMultFactor,
		followPars.MaxSteerSpeed, followPars.TimeStep);
	car.setAxisDistance(followPars.AxisDistance);
	car.setFiMax(followPars.FiMax);
	car.setWheelDiameter(followPars.WheelDiameter);
	car.setWheelDistance(followPars.WheelDistance);

	// This is the server loop
	while(true) {
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

		if(init) {
			init = false;
			car.setPosition(robotPos);
		}

		//Send robot position to the PathFollow Client
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
		car.modelRobot(v, fi);

		//Send data to V-Rep Client
		if(SendRobotData(connection, car.getModelSpeed(), car.getModelSteer(), car.getPosition(), rabit_msg.pos, info_msg.values))
			break;
	}
}