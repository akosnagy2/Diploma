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

float predictDistanceLength = 10;
float predictSampleLength = 10;
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
	predictSampleLength = (int)parMsg.values[0];
	predictDistanceLength = (int)parMsg.values[1];
	oriPar_P = (float)parMsg.values[2];
	oriPar_D = (float)parMsg.values[3];
	timeStep = (float)parMsg.values[4];
	wheelDistance = (float)parMsg.values[5];
	pathMaxSpeed = (float)parMsg.values[6];
	pathMaxAccel = (float)parMsg.values[7];
	pathMaxTangentAccel = (float)parMsg.values[8];
	pathMaxAngularSpeed = (float)parMsg.values[9];

	path.receive(s);
}

void ConvertPathToDSPPath(vector<PathSegment> &sampPath, vector<vector<float>> &path_dsp_vel, vector<vector<PositionTypedef>> &path_dsp_points, vector<PathSegmentTypedef> &path_dsp)
{
	int i = 0;
	for (auto &ps : sampPath)
	{
		vector<PositionTypedef> pathDSP;
		for (int i = 0; i < ps.path.size(); i++) //All points in a path segment
		{
			PositionTypedef pointDSP;
			pointDSP.x = ps.path[i].p.x;
			pointDSP.y = ps.path[i].p.y;
			pointDSP.phi = ps.path[i].phi;
			pathDSP.push_back(pointDSP);
		}
		path_dsp_points.push_back(pathDSP);
		PathSegmentTypedef segmentDSP;
		segmentDSP.dir = ps.direction ? FORWARD : BACKWARD;
		segmentDSP.path = path_dsp_points.back()._Myfirst;
		segmentDSP.velocity = path_dsp_vel[i++]._Myfirst;
		segmentDSP.path_len = (uint16_t)path_dsp_points.back().size();
		path_dsp.push_back(segmentDSP);
	}
}

int main()
{
	vector<PathPlanner::PathSegment> geoPath;
	vector<PathPlanner::PathSegment> sampPath;
	vector<vector<float>> path_dsp_vel;
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
	if (geoPath.size() == 0)
		return -1;

	setLimits(pathMaxSpeed, pathMaxAccel, pathMaxTangentAccel, pathMaxAngularSpeed, timeStep, wheelDistance);
	profile_top(geoPath, sampPath, path_dsp_vel);	
	
	//Send back to V-REP
	vrepPath.path = sampPath;
	vrepPath.send(s);

	//Convert Sampled PathSegments to PathFollow PathSegments
	vector<PathSegmentTypedef> path_dsp;
	vector<vector<PositionTypedef>> path_dsp_points;
	ConvertPathToDSPPath(sampPath, path_dsp_vel, path_dsp_points, path_dsp);

	PathCtrlTypedef pathFollow;
	PathCtrl_Init(&pathFollow, 0, 2*pathMaxAccel/wheelDistance,pathMaxAngularSpeed, (timeStep*1000), 0, 0);
	PathCtrl_SetPars(&pathFollow, oriPar_P, oriPar_D);
	PathCtrl_SetPathSegments(&pathFollow, path_dsp._Myfirst, path_dsp.size());
	PathCtrl_SetRobotPar(&pathFollow, wheelDistance);
	pathFollow.predictSampleLength = predictSampleLength;
	pathFollow.predictDistanceLength = predictDistanceLength;
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
			info.values.push_back(pathFollow.pathSegments[pathFollow.segmentIndex].path[pathFollow.timeIndex].x);
			info.values.push_back(pathFollow.pathSegments[pathFollow.segmentIndex].path[pathFollow.timeIndex].y);
			
			//Robot motors control signals
			ctrl_out.ctrl_sig.push_back(leftV);
			ctrl_out.ctrl_sig.push_back(rightV);
			ctrl_out.src = 1;

			cout << "Index: " << pathFollow.timeIndex << endl;
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