#include "Common.h"
#include "CarLikeRobot.h"

bool ReceivePars(CSimpleInConnection& connection, deque<float> &pars)
{
	int len;
	char* receivedData = connection.receiveData(len);

	if (len)
	{
		for (int i = 0; i < len/(int)sizeof(float); i++)		
			pars.push_back( ((float*)receivedData)[i] );		

		delete[] receivedData;
		return true;
	}

	return false;
}

void ForwardPath(CSimpleInConnection& connection, tcp::iostream& s)
{
	int receivedDataLength;
	char* receivedData=connection.receiveData(receivedDataLength);
	int offset = 0;
	if (receivedDataLength)
	{
		PathMessage path;
		
		while (offset < receivedDataLength/(int)sizeof(float))
		{
			PathSegment seg;
			seg.direction = (bool)(((float*)receivedData)[offset]);
			int size = (int)(((float*)receivedData)[offset + 1]);
			for (int i = 0; i < size; i++)
			{
				Config pos;
				pos.p.x = ((float*)receivedData)[offset + 2 + 3*i];
				pos.p.y = ((float*)receivedData)[offset + 2 + 3*i + 1];
				pos.phi = ((float*)receivedData)[offset + 2 + 3*i + 2];
				seg.path.push_back(pos);
			}
			offset += size*3 + 2;
			path.path.push_back(seg);
		}
		path.send(s);
	}
	delete[] receivedData;
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

vector<float> ConvertVrepPath(PathMessage &path)
{
	vector<float> path_vrep;
	for (auto &s : path.path)
	{
		for (auto &p : s.path)
		{
			path_vrep.push_back(p.p.x);
			path_vrep.push_back(p.p.y);
		}
	}

	return path_vrep;
}

int SendRobotData(CSimpleInConnection& connection,float leftJointPos, float rightJointPos, Config& robotPos, Config& rabitPos, vector<float> info_val)
{
	vector<float> msg;

	msg.push_back(robotPos.p.x);
	msg.push_back(robotPos.p.y);
	msg.push_back(robotPos.phi);
	msg.push_back(leftJointPos);
	msg.push_back(rightJointPos);
	msg.push_back(rabitPos.p.x);
	msg.push_back(rabitPos.p.y);
	msg.insert(msg.end(), info_val.begin(), info_val.end());
	
	if (!connection.replyToReceivedData((char*)msg._Myfirst,sizeof(float)*msg.size()))
		return -1;

	return 0;
}

void ModelDifferentialMotors(float& leftSpeed, float& rightSpeed, float newLeftSpeed, float newRightSpeed, MotorParamsTypedef& leftMotor, MotorParamsTypedef& rightMotor, PathFollowParamsTypedef &pars)
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

	//dFile[2] << leftSpeed << endl;
	//dFile[3] << rightSpeed << endl;

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

	//dFile[4] <<  (leftSpeed - prevLeftSpeed) / pars.TimeStep << endl;
	//dFile[5] << (rightSpeed - prevRightSpeed) / pars.TimeStep << endl;
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
		if (!ctrl_msg.receive(client))
			break;
		if (!rabit_msg.receive(client))
			break;
		if (!info_msg.receive(client))
			break;

		//Apply motor models
		ModelDifferentialMotors(leftSpeed, rightSpeed,(float)ctrl_msg.ctrl_sig[0], (float)ctrl_msg.ctrl_sig[1], followPars.LeftMotor, followPars.RightMotor, followPars);

		logFile << "Robot target left vel.: " << leftSpeed << ", right vel.: " << rightSpeed << endl;
		logFile << "Simulation timeindex: " << timeIndex++ << endl;
		logFile << "Robot pos: " <<  robotPos.p.x << ", " << robotPos.p.y << endl;
		logFile << "Robot ori: " <<  robotPos.phi << endl;

		//Wheels -> Robot
		robotSpeed = (leftSpeed + rightSpeed) / 2.0f;
		robotAngularSpeed = (rightSpeed - leftSpeed) / followPars.WheelBase;

		//dFile[0] << robotSpeed << endl;
		//dFile[1] << (robotSpeed - prevRobotSpeed) / followPars.TimeStep << endl;
		prevRobotSpeed = robotSpeed;

		//Position
		robotPos.phi += robotAngularSpeed * followPars.TimeStep / 2.0f;
		robotPos.p.x += cos(robotPos.phi) * robotSpeed * followPars.TimeStep;
 		robotPos.p.y += sin(robotPos.phi) * robotSpeed * followPars.TimeStep;
		robotPos.phi += robotAngularSpeed  * followPars.TimeStep / 2.0f;

		leftJointPos += leftSpeed*followPars.TimeStep*2/followPars.WheelDiameter;
		rightJointPos += rightSpeed*followPars.TimeStep*2/followPars.WheelDiameter;

		//Send data to V-Rep Client
		if (SendRobotData(connection,leftJointPos, rightJointPos, robotPos, rabit_msg.pos, info_msg.values))
			break;
	}
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