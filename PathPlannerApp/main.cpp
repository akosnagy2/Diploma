#include "PathPlanner\Scene.h"
#include "Geometry\Config.h"
#include <chrono>
#include "tiny_obj_loader.h"

#include <boost/asio/ip/tcp.hpp>
#include <boost/asio.hpp>
#include <boost/system/error_code.hpp>

#include "CtrlMessage.h"
#include "PathMessage.h"
#include "PathProfile\path_profile_top.h"
#include "Pos2dMessage.h"

#include "PathFollow\dcwheel_pathCtrl.h"
#include "PathFollow/CarLineFollower.h"
#include "PathFollow/CarPathController.h"

#include "CarLikeRobot.h"
#include "App.h"

using namespace std;
using namespace std::chrono;
using boost::asio::ip::tcp;

float predictDistanceLength = 5.0;
float predictSampleLength = 2.0;
float oriPar_P = 0.3f;
float oriPar_D = 0.1f;
float lineW0 = 2.0f;
float lineKsi = 1.0f;
float timeStep = 0.05f;
float wheelDistance = 254.0f;
float pathMaxSpeed = 200.0f;
float pathMaxAccel = 200.0f;
float pathMaxTangentAccel = 200.0f;
float pathMaxAngularSpeed = 1.0f;

CarLikeRobot robotData;
static App robotType;

//TODO: nincs checkBack;

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

void ConvertPathToDSPPath(vector<PathSegment> &sampPath, vector<vector<PositionTypedef>> &path_dsp_points, vector<PathSegmentTypedef> &path_dsp)
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
		segmentDSP.velocity = ps.velocity._Myfirst;
		segmentDSP.path_len = (uint16_t)path_dsp_points.back().size();
		path_dsp.push_back(segmentDSP);
	}
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
			scene.SetRobotShape(ParseObjShape(elem)); //Load Robot shape -> Set in Scene
		else if (elem.name.find("Obstacle") != string::npos)
			scene.AddEnv(ParseObjShape(elem)); //Load ObstacleX shape
		else if (elem.name == "StartConfig")
			scene.SetStartConfig(Config(elem.mesh.positions[0], elem.mesh.positions[1], elem.mesh.positions[2])); //Start Config
		else if (elem.name == "GoalConfig")
			scene.SetGoalConfig(Config(elem.mesh.positions[0], elem.mesh.positions[1], elem.mesh.positions[2])); //Goal Config
		else if (elem.name == "Field") 
			scene.AddField(elem.mesh.positions[6], elem.mesh.positions[4]); //Load field
		else
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
	robotType.appType = (App::AppTypedef)(int)parMsg.values[0];
	if(robotType.isCarLikeRobot()) {
		predictSampleLength = (float)parMsg.values[1];
		predictDistanceLength = (float)parMsg.values[2];
		lineW0 = (float) parMsg.values[4];
		lineKsi = (float) parMsg.values[5];
		timeStep = (float) parMsg.values[6];
		wheelDistance = (float) parMsg.values[7];
		pathMaxSpeed = (float) parMsg.values[8];
		pathMaxAccel = (float) parMsg.values[9];
		//pathMaxTangentAccel = pathMaxAccel;
		robotData.setAxisDistance((float) parMsg.values[10]);
		robotData.setFiMax((float) parMsg.values[11]);
		robotData.setWheelDistance(wheelDistance);
		sc.SetRobotMinimumRadius(robotData.getAxisDistance() / tan(robotData.getFiMax()));

		//PathPlanner params
		sc.SetRTRParameters((int)parMsg.values[12], (float)parMsg.values[13], (float)parMsg.values[14], (int)parMsg.values[15]);
		envFileName = "frame" + to_string((int)parMsg.values[16]) + "c.obj";
		sc.SetRobotMinimumRadius((float)parMsg.values[17]);
		sc.SetRobotWheelBase(wheelDistance);
		sc.SetPathDeltaS((float)parMsg.values[18]);
		sc.SetCCSParameters(parMsg.values[19], parMsg.values[20], parMsg.values[21], parMsg.values[22]);
	} else { 
		predictSampleLength = (float)parMsg.values[1];
		predictDistanceLength = (float)parMsg.values[2];
		oriPar_P = (float) parMsg.values[3];
		oriPar_D = (float) parMsg.values[4];
		timeStep = (float) parMsg.values[5];
		wheelDistance = (float) parMsg.values[6];
		pathMaxSpeed = (float) parMsg.values[7];
		pathMaxAccel = (float) parMsg.values[8];
		pathMaxTangentAccel = (float) parMsg.values[9];
		pathMaxAngularSpeed = (float) parMsg.values[10];

		//PathPlanner params
		sc.SetRTRParameters((int) parMsg.values[11], (float) parMsg.values[12], (float) parMsg.values[13], (int) parMsg.values[14]);
		envFileName = "frame" + to_string((int) parMsg.values[15]) + ".obj";
		sc.SetRobotMinimumRadius((float) parMsg.values[16]);
		sc.SetRobotWheelBase(wheelDistance);
		sc.SetPathDeltaS((float) parMsg.values[17]);
	}
	return true;
}

PathPlanner::Scene sc;

int main()
{
	chrono::high_resolution_clock::time_point start, stop;
	string envFileName;
	vector<PathPlanner::PathSegment> geoPath;
	vector<PathPlanner::PathSegment> sampPath;
	vector<vector<float>> path_dsp_vel;
	PathMessage ccsPath;

	//Load params from V-REP via ServerApp
	tcp::iostream s("127.0.0.1",to_string(168));
	
	if (s)
	{
		cout << "V-REP Mode" << endl;
		if (!LoadParams(s, sc, envFileName))
			return -1;
	}
	else
	{
		cout << "Manual Mode" << endl;
		envFileName = "frame6.obj";
		sc.SetRobotMinimumRadius(50.0f);
		sc.SetRobotWheelBase(wheelDistance);
		sc.SetPathDeltaS(10.0);
		sc.SetRTRParameters(3000, 0.0f, 0.2f, 350);
		robotType.appType = App::DifferentialPathPlanner;
	}
	
	if (robotType.isPathPlanner())
	{
		PathMessage rtrPath;
		cout << "PathPlanner Mode" << endl;

		//Debug
		sc.SetFixPrePath(LoadPathFromFile("RTRPath.txt"));
	
		//Set params to scene object
		if (!ParseObj("..\\Frame\\" + envFileName, sc))
			return -1;

		//Generate Pre Path
		start = high_resolution_clock::now();
		cout << "Pre Planner starting..." << endl;
		sc.PrePlanner();
		stop = high_resolution_clock::now();
		cout << "Pre Planner ended, " <<duration_cast<chrono::microseconds>(stop-start).count() << " us." << endl;

		//Generate RTR Path
		start = high_resolution_clock::now();
		cout << "RTR Planner starting..." << endl;
		sc.RTRPlanner();
		sc.GenerateRTRPath();
		stop = high_resolution_clock::now();
		cout << "RTR Planner ended, " <<duration_cast<chrono::microseconds>(stop-start).count() << " us." << endl;
		
		//Send RTR Path
		cout << "Sending RTR Path to V-REP..." << endl;
		rtrPath.path = sc.GetPath();
		rtrPath.send(s);

		start = high_resolution_clock::now();
		cout << "C*CS Planner starting..." << endl;
		bool success = sc.CCSPlanner();
		stop = high_resolution_clock::now();
		cout << "C*CS Planner ended, " <<duration_cast<chrono::microseconds>(stop-start).count() << " us." << endl;
		
		if(!success && robotType.isCarLikeRobot()) 
			return -1;		

		geoPath = sc.GetPath();	
	}
	else
	{
		cout << "PathFollow Mode" << endl;
		ccsPath.path.clear();
		ccsPath.receive(s);
		geoPath = ccsPath.path;			
	}
	
	if (geoPath.size() == 0)
		return -1;

	//Calc sampled path
	start = high_resolution_clock::now();
	cout << "Time Parameterization starting..." << endl;
	if (robotType.isCarLikeRobot())
	{
		setCarLimits(&robotData, pathMaxSpeed, pathMaxAccel, pathMaxTangentAccel, timeStep);
		profile_top(geoPath, sampPath, true);	
	}
	else 
	{
		setLimits(pathMaxSpeed, pathMaxAccel, pathMaxTangentAccel, pathMaxAngularSpeed, timeStep, wheelDistance);	
		profile_top(geoPath, sampPath, false);	
	}	
	stop = high_resolution_clock::now();
	cout << "Time Parameterization ended, " <<duration_cast<chrono::microseconds>(stop-start).count() << " us." << endl;
	
	//Send back to V-REP
	cout << "Sending Time Parameterized C*CS Path to V-REP..." << endl;
	ccsPath.path = sampPath;
	ccsPath.send(s);

	cout << "Path Follow Controlling starting..." << endl;
	if (robotType.isDifferentialRobot())
	{
		//Convert Sampled PathSegments to PathFollow PathSegments
		vector<PathSegmentTypedef> path_dsp;
		vector<vector<PositionTypedef>> path_dsp_points;
		ConvertPathToDSPPath(sampPath, path_dsp_points, path_dsp);

		PathCtrlTypedef pathFollow;
		PathCtrl_Init(&pathFollow, 0, (timeStep*1000), 0, 0);
		PathCtrl_SetPars(&pathFollow, oriPar_P, oriPar_D);
		PathCtrl_SetPathSegments(&pathFollow, path_dsp._Myfirst, path_dsp.size());
		PathCtrl_SetRobotPar(&pathFollow, wheelDistance);
		pathFollow.predictSampleLength = predictSampleLength;
		pathFollow.predictDistanceLength = predictDistanceLength;
		pathFollow.maxVel = pathMaxSpeed;
		pathFollow.maxAccel = pathMaxAccel;
		pathFollow.maxAngAccel = 2*pathMaxAccel/wheelDistance;
		pathFollow.maxAngVel = pathMaxAngularSpeed;
		PathCtrl_SetState(&pathFollow, 1);

		while(s.good()) {
			CtrlMessage ctrl_out;
			Config act_pos;
			Pos2dMessage pos_in;
			Pos2dMessage rabitPos;
			PackedMessage info;
			float leftV = 0, rightV = 0;

			//Receive robot position
			pos_in.receive(s);
			act_pos = pos_in.pos;

			//Corrigate phi to [-Pi, Pi] range
			while(act_pos.phi > (float) M_PI)
				act_pos.phi -= (float) (2 * M_PI);

			while(act_pos.phi < -(float) M_PI)
				act_pos.phi += (float) (2 * M_PI);

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
 
			rabitPos.pos.p.x = pathFollow.pathSegments[pathFollow.segmentIndex].path[pathFollow.predictIndex].x;
			rabitPos.pos.p.y = pathFollow.pathSegments[pathFollow.segmentIndex].path[pathFollow.predictIndex].y;

			ctrl_out.send(s);
			rabitPos.send(s);
			info.send(s);
		}
	} else {
		CarLineFollower follower(robotData, lineW0, lineKsi, predictSampleLength);
		CarPathController pathController(sampPath, robotData, follower, predictSampleLength);

		boost::asio::io_service io;
		boost::asio::serial_port serial_port(io);
		boost::system::error_code ec;
		
		//serial_port.open("COM3", ec);
		//if(ec == boost::system::errc::no_such_file_or_directory)
		//{
		//	cout << "Could not open serial port" << endl;
		//}
		//serial_port.set_option(boost::asio::serial_port::baud_rate(115200));
		
		while(s.good()) {
			CtrlMessage ctrl_out;
			Config act_pos;
			Pos2dMessage pos_in;
			Pos2dMessage rabitPos;
			PackedMessage info;

			//Receive robot position
			pos_in.receive(s);
			act_pos = pos_in.pos;

			//Corrigate phi to [-Pi, Pi] range
			while(act_pos.phi > (float) M_PI)
				act_pos.phi -= (float) (2 * M_PI);

			while(act_pos.phi < -(float) M_PI)
				act_pos.phi += (float) (2 * M_PI);

			pathController.Loop(act_pos);
			rabitPos.pos = pathController.getRabbit();

			info.values.push_back(pathController.getTrackError());
			//info.values.push_back(speedController.getSumError());

			ctrl_out.ctrl_sig.push_back(pathController.getV());
			ctrl_out.ctrl_sig.push_back(pathController.getFi());
			ctrl_out.src = 1;

			std::cout << "Target speed: " << pathController.getV() << ", angle: " << pathController.getFi() << endl;

			ctrl_out.send(s);
			rabitPos.send(s);
			info.send(s);
		}
		serial_port.close();
	}

	return 0;
}