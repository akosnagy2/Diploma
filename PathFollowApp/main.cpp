#include <iostream>
#include <fstream>
#include <string>
#include <boost/asio.hpp>
#include "CtrlMessage.h"
#include "Pos2dMessage.h"
#include "PathMessage.h"
#include "PathPlannerApp\PathFollow\dcwheel_pathCtrl.h"
#include "PathPlannerApp\PathProfile\path_profile_top.h"

#define _USE_MATH_DEFINES
#include <math.h>

using boost::asio::ip::tcp;
using namespace std;

float predictLength = 5;
float predictLengthImpulse = 15;
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
/*
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
*/
void LoadPathFromTCP(tcp::iostream &s, PathMessage &path)
{
	PackedMessage parMsg;

	parMsg.receive(s);			
	predictLength = (int)parMsg.values[0];
	predictLengthImpulse = (int)parMsg.values[1];
	distPar_P = (float)parMsg.values[2];
	distPar_D = (float)parMsg.values[3];
	oriPar_P = (float)parMsg.values[4];
	oriPar_D = (float)parMsg.values[5];
	timeStep = (float)parMsg.values[6];
	wheelDistance = (float)parMsg.values[7];
	pathMaxSpeed = (float)parMsg.values[8];
	pathMaxAccel = (float)parMsg.values[9];
	pathMaxTangentAccel = (float)parMsg.values[10];
	pathMaxAngularSpeed = (float)parMsg.values[11];

	path.receive(s);
}

int main()
{
	vector<PathPlanner::PathSegment> geoPath;
	vector<PathPlanner::PathSegment> sampPath;
	PathMessage vrepPath;
	tcp::iostream s("127.0.0.1","168");

	if (!s) //Load from file
	{
		std::cout << "Unable to connect: " << s.error().message() << endl;
		std::cout << "Try to load path.txt..." << endl;

	//	LoadPathFromFile("path.txt", geoPath);
	}
	else //Load from V-REP
	{
		std::cout << "Connected to V-REP Server: " << s.error().message() << endl;
		LoadPathFromTCP(s, vrepPath);
		geoPath = vrepPath.path;
		for (auto &s : geoPath)
			s.curvature.clear();
	}

	//Calc sampled path
	setLimits(pathMaxSpeed, pathMaxAccel, pathMaxTangentAccel, pathMaxAngularSpeed, timeStep, wheelDistance, predictLength);
	profile_top(geoPath, sampPath);	
	
	//Send back to V-REP
	vrepPath.path = sampPath;
	vrepPath.send(s);

	//Convert Sampled PathSegments to PathFollow PathSegments
	vector<vector<PositionTypedef>> path_dsp_points;
	vector<vector<float>> path_dsp_curv;
	vector<PathSegmentTypedef> path_dsp;
	for (auto &ps : sampPath)
	{
		vector<PositionTypedef> pathDSP;
		vector<float> curvDSP;
		for (int i = 0; i < ps.path.size(); i++) //All points in a path segment
		{
			PositionTypedef pointDSP;
			pointDSP.x = ps.path[i].p.x;
			pointDSP.y = ps.path[i].p.y;
			pointDSP.phi = ps.path[i].phi;
			pathDSP.push_back(pointDSP);
			curvDSP.push_back(ps.curvature[i]);
		}
		path_dsp_points.push_back(pathDSP);
		path_dsp_curv.push_back(curvDSP);
		PathSegmentTypedef segmentDSP;
		segmentDSP.dir = ps.direction ? FORWARD : BACKWARD;
		segmentDSP.path = path_dsp_points.back()._Myfirst;
		segmentDSP.curvature = path_dsp_curv.back()._Myfirst;
		segmentDSP.path_len = (uint16_t)path_dsp_points.back().size();
		path_dsp.push_back(segmentDSP);
	}

	PathCtrlTypedef pathFollow;
	PathCtrl_Init(&pathFollow, 0, 2*pathMaxAccel/wheelDistance,pathMaxAngularSpeed, (timeStep*1000), 0, 0);
	PathCtrl_SetPars(&pathFollow, distPar_P,distPar_D,oriPar_P,oriPar_D);
	PathCtrl_SetPathSegments(&pathFollow, path_dsp._Myfirst, path_dsp.size());
	PathCtrl_SetRobotPar(&pathFollow, wheelDistance);
	pathFollow.predictLength = predictLength;
	pathFollow.predictLengthImpulse = predictLengthImpulse;
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
			info.values.push_back(pathFollow.debugData[0]);
			info.values.push_back(pathFollow.debugData[1]);			
			info.values.push_back(pathFollow.debugData[2]);
			info.values.push_back(pathFollow.debugData[3]);	
			info.values.push_back(pathFollow.debugData[4]);
			info.values.push_back(pathFollow.debugData[5]);	
			
			//Robot motors control signals
			ctrl_out.ctrl_sig.push_back(leftV);
			ctrl_out.ctrl_sig.push_back(rightV);
			ctrl_out.src = 1;

			std::cout << "Target speed: " << leftV << ", " << rightV << endl;

 			ctrl_out.send(s);
			rabitPos.pos.p.x = pathFollow.pathSegments[pathFollow.segmentIndex].path[pathFollow.predictIndex].x;
			rabitPos.pos.p.y = pathFollow.pathSegments[pathFollow.segmentIndex].path[pathFollow.predictIndex].y;
			rabitPos.send(s);
			info.send(s);			
		}

		std::cout << "Server closed the connection." << endl;
	
	return 0;
}