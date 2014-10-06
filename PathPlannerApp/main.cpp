#include "PathPlanner\Scene.h"
#include "PathPlanner\Config.h"
#include <chrono>
#include "tiny_obj_loader.h"
#include <boost/asio/ip/tcp.hpp>
#include "CtrlMessage.h"
#include "PathMessage.h"
#include "PathProfile\path_profile_top.h"
#include "Pos2dMessage.h"
#include "PathPlannerApp\PathFollow\dcwheel_pathCtrl.h"

using namespace std;
using namespace std::chrono;
using boost::asio::ip::tcp;

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


//TODO: nincs checkBack!!!

vector<PathPlanner::Point> LoadPathFromFile(string filename)
{
	vector<PathPlanner::Point> path;
	string line;
	ifstream fs(filename);
	PathPlanner::Point pos;

	while (getline(fs,line))
	{
		string::size_type sz;
		{
			pos.x = stof(line,&sz);
			pos.y = stof(line.substr(sz));
			path.push_back(pos);
		}
	} 

	return path;
}

PathPlanner::Polygon ParseObjShape(tinyobj::shape_t &shp)
{
	PathPlanner::Polygon poly;

	for (int i = 0; i < (int)shp.mesh.positions.size(); i += 3)
		poly.AddPoint(PathPlanner::Point(shp.mesh.positions[i], shp.mesh.positions[i+1])); //Load x, y coord. Skip z coord.

	return poly;
}

bool ParseObj(string objName, PathPlanner::Scene &scene)
{
	bool ret = true;
	vector<tinyobj::shape_t> shapes;
	vector<tinyobj::material_t> materials;

	//Load obj file by TinyObjLoader
	tinyobj::LoadObj(shapes, materials, objName.c_str());

	for (auto& elem : shapes)
	{
		if (elem.name == "Robot")
			scene.SetRobotShape(ParseObjShape(elem).TransformToLocal(scene.GetStartConfig())); //Load Robot shape -> Transform to Local Frame -> Set in Scene
		else if (elem.name.find("Obstacle") != string::npos)
			scene.AddEnv(ParseObjShape(elem)); //Load ObstacleX shape
		else if (elem.name != "Field") //Do not load field, it was loaded with params
			ret = false;
	}

	return ret;
}

bool LoadParams(tcp::iostream &s, PathPlanner::Scene &sc, string &envFileName)
{
	PackedMessage parMsg;

	if (!s) 
	{
		cout << "Unable to connect: " << s.error().message() << endl;
		return false;
	}

	cout << "Connected to V-REP Server: " << s.error().message() << endl;
	
	parMsg.receive(s);	
	
	//PathProfile, PathFollow params
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
	
	//PathPlanner params
	sc.AddField((float)parMsg.values[11], (float)parMsg.values[12]);
	sc.SetStartConfig(PathPlanner::Config((float)parMsg.values[13], (float)parMsg.values[14], (float)parMsg.values[15]));
	sc.SetGoalConfig(PathPlanner::Config((float)parMsg.values[16], (float)parMsg.values[17], (float)parMsg.values[18]));
	sc.SetRTRParameters((int)parMsg.values[19], (float)parMsg.values[20], (float)parMsg.values[21]);

	envFileName = "frame" + to_string((int)parMsg.values[22]) + ".obj";

	return true;
}

PathPlanner::Scene sc;

int main()
{
	chrono::high_resolution_clock::time_point start, stop;
	string envFileName = "frame1.obj";

	//Load params from V-REP via ServerApp
	tcp::iostream s("127.0.0.1",to_string(168));
	if (!LoadParams(s, sc, envFileName))
		return -1;

	/*
	sc.AddField(32.0f, 15.0f);
	sc.SetRTRParameters(1000, 0.0f, 1.0f);
	sc.SetStartConfig(PathPlanner::Config(9.95f, 11.3f, PI/2));
	sc.SetGoalConfig(PathPlanner::Config(22.15f, 0.6f, PI/2));
	*/
	
	sc.SetFixPrePath(LoadPathFromFile("RTRPath.txt"));
	
	//Set params to scene object
	ParseObj("..\\Frame\\" + envFileName, sc);

	//TODO: nem pontos robot shape esetén egész más lefolyása van
	PathPlanner::Polygon rob;
	rob.AddPoint(PathPlanner::Point(3550.0f, 1000.0f));
	rob.AddPoint(PathPlanner::Point(-450.0f, 1000.0f));
	rob.AddPoint(PathPlanner::Point(-450.0f, -1000.0f));
	rob.AddPoint(PathPlanner::Point(3550.0f, -1000.0f));
	sc.SetRobotShape(rob); 

	start = high_resolution_clock::now();
	sc.PrePlanner();
	sc.RTRPlanner();
	
	vector<PathPlanner::Config> &geoPath = sc.ExtractPath();
	PathMessage path_samp_msg;
	
	//Calc sampled path
	setLimits(pathMaxSpeed, pathMaxAccel, pathMaxTangentAccel, pathMaxAngularSpeed, timeStep, wheelDistance, predictLength);
	profile_top(geoPath, path_samp_msg.path);	
	path_samp_msg.send(s);

	stop = high_resolution_clock::now();
	cout << duration_cast<chrono::microseconds>(stop-start).count() << " us." << endl;

	//Init path follow
	vector<PositionTypedef> path_dsp((int)path_samp_msg.path.size());
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
		info.values.push_back(0.0f);
		info.values.push_back(0.0f);				
			
		//Robot motors control signals
		ctrl_out.ctrl_sig.push_back(leftV);
		ctrl_out.ctrl_sig.push_back(rightV);
		ctrl_out.src = 1;

		std::cout << "Target speed: " << leftV << ", " << rightV << endl;

 		ctrl_out.send(s);
		rabitPos.send(s);
		info.send(s);			
	}


	return 0;
}