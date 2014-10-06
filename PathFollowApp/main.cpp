#include <iostream>
#include <fstream>
#include <string>
#include <boost/asio.hpp>
#include "CtrlMessage.h"
#include "Pos2dMessage.h"
#include "PathMessage.h"
#include "PathFollow\dcwheel_pathCtrl.h"
#include "PathProfile\path_profile_top.h"

#define _USE_MATH_DEFINES
#include <math.h>

using boost::asio::ip::tcp;
using namespace std;

PathMessage path_geo_msg;
PathMessage path_samp_msg;

int predictLength = 5;
float distPar_P = 0.0f;
float distPar_D = 1.1f;
float oriPar_P = 0.1f;
float oriPar_D = 0.15f;
float timeStep = 0.1f;
float wheelDistance = 254.0f;
float pathMaxSpeed = 200.0f;
float pathMaxAccel = 100.0f;
float pathMaxTangentAccel = 100.0f;
float pathMaxAngularSpeed = 1.628f;

void LoadPathFromFile(string filename)
{
	string line;
	ifstream fs(filename);
	Config pos;

	while (getline(fs,line))
	{
		string::size_type sz;
		if (filename.substr(filename.find('.')) == ".csv") //Load from .csv (V-REP)
		{
			char* temp;
			temp = strtok((char*)line.c_str(),",");
			pos.p.x = stof(temp,&sz);
			temp = strtok(NULL, ",");
			pos.p.y = stof(temp,&sz);
			path_geo_msg.path.push_back(pos);
		}
		else //Load from .txt
		{
			pos.p.x = stof(line,&sz);
			pos.p.y = stof(line.substr(sz));
			path_geo_msg.path.push_back(pos);
		}
	} 
}

void LoadPathFromTCP(tcp::iostream &s)
{
	PackedMessage parMsg;

	parMsg.receive(s);			
	predictLength = (int)parMsg.values[0];
	distPar_P = (float)parMsg.values[1];
	distPar_D = (float)parMsg.values[2];
	oriPar_P = (float)parMsg.values[3];
	oriPar_D = (float)parMsg.values[4];
	timeStep = (float)parMsg.values[5];
	wheelDistance = (float)parMsg.values[6];
	pathMaxSpeed = (float)parMsg.values[7];
	pathMaxAccel = (float)parMsg.values[8];
	pathMaxTangentAccel = (float)parMsg.values[9];
	pathMaxAngularSpeed = (float)parMsg.values[10];

	path_geo_msg.receive(s);
}

int main()
{
		tcp::iostream s("127.0.0.1","168");

		if (!s) //Load from file
		{
			std::cout << "Unable to connect: " << s.error().message() << endl;
			std::cout << "Try to load path.txt..." << endl;

			LoadPathFromFile("path.txt");
		}
		else //Load from V-REP
		{
			std::cout << "Connected to V-REP Server: " << s.error().message() << endl;
			LoadPathFromTCP(s);
		}

		//Calc sampled path
		setLimits(pathMaxSpeed, pathMaxAccel, pathMaxTangentAccel, pathMaxAngularSpeed, timeStep, wheelDistance, predictLength);
		profile_top(path_geo_msg.path, path_samp_msg.path);
		
		if (!s) //Exit when no V-REP
			return 1;

		//Send sampled path
		path_samp_msg.send(s);

		//Init path follow
		std::vector<PositionTypedef> path_dsp((int)path_samp_msg.path.size());
		for (int i = 0; i < (int)path_samp_msg.path.size(); i++)
		{
			path_dsp[i].x = path_samp_msg.path[i].p.x;
			path_dsp[i].y = path_samp_msg.path[i].p.y;
			path_dsp[i].phi = path_samp_msg.path[i].phi;
		}
		PathCtrlTypedef pathFollow;
		PathCtrl_Init(&pathFollow, 0, 2*pathMaxAccel/wheelDistance,pathMaxAngularSpeed, (timeStep*1000), 0, 0);
		PathCtrl_SetPars(&pathFollow, distPar_P,distPar_D,oriPar_P,oriPar_D);
		PathCtrl_SetPath(&pathFollow, path_dsp._Myfirst, path_dsp.size());
		PathCtrl_SetRobotPar(&pathFollow, wheelDistance, predictLength);
		PathCtrl_SetState(&pathFollow, 1);

		while (s.good())
		{
			CtrlMessage ctrl_out;
			Config act_pos;
			Pos2dMessage pos_in;
			Pos2dMessage rabitPos;
			PackedMessage info;
			float leftV = 0,rightV = 0;
			
			//Receive robot position
			pos_in.receive(s);
			act_pos = pos_in.pos;
			
			//Corrigate phi to [-Pi, Pi] range
			while (act_pos.phi > (float)M_PI)
				act_pos.phi -= (float)(2*M_PI);

			while (act_pos.phi < -(float)M_PI)
				act_pos.phi += (float)(2*M_PI);

			//Process path follow
			pathFollow.robotPos.x = act_pos.p.x;
			pathFollow.robotPos.y = act_pos.p.y;
			pathFollow.robotPos.phi = act_pos.phi;
			PathCtrl_Loop(&pathFollow, &leftV, &rightV);
			
			//Info
			info.values.push_back(pathFollow.distError);
			info.values.push_back(pathFollow.pathSumDist);				
			
			//Robot motors control signals
			ctrl_out.ctrl_sig.push_back(leftV);
			ctrl_out.ctrl_sig.push_back(rightV);
			ctrl_out.src = 1;

			std::cout << "Target speed: " << leftV << ", " << rightV << endl;

 			ctrl_out.send(s);
			rabitPos.send(s);
			info.send(s);			
		}

		std::cout << "Server closed the connection." << endl;
	
	return 0;
}