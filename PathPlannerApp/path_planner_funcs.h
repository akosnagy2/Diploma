#pragma once

float corrigateAngle(float angle);
float directedAngleDist(float thetaStart, float thetaEnd, bool turnDir);

#include <vector>
#include <boost/numeric/ublas/matrix_sparse.hpp>
#include <Fade_2D.h>
#include "Point.h"
#include "Triangle.h"
#include "Polygon.h"
#include "Config.h"
#include "Tree.h"
#define _USE_MATH_DEFINES
#include <math.h>

using namespace std;
using namespace boost::numeric;
using namespace GEOM_FADE2D;

#define EPS	(0.001f)
#define PI ((float)M_PI)

template<typename T> void printMat(ublas::matrix<T> mat);
bool insideTriangle(Point a, Triangle t);
bool lineSegmentIntersection(Config line, Point s1, Point s2, Point &intersect);

class Scene
{
public:
	Scene()
	{
	}
	~Scene()
	{

	}
	void AddField(Polygon fi)
	{
		field = fi;
		//Create extended env
		if (envs.size())
		{
			envsx = envs;
			envsx.push_back(field);
		}
	}
	void AddEnv(Polygon env)
	{
		envs.push_back(env);
		//Create extended env
		envsx = envs;
		if (field.ps.size())
			envsx.push_back(field);
	}
	void SetStartConfig(Config pos)
	{
		robotStart = pos;
	}
	void SetGoalConfig(Config pos)
	{
		robotGoal = pos;
	}
	void SetRobotShape(Polygon shp)
	{
		robotShape = shp;
	}
	float GetRobotWidth()
	{
		vector<float> shp_y;

		//Get local frame y abs values
		for (int i = 0; i < (int)robotShape.ps.size(); i++)
			shp_y.push_back(fabs(robotShape.ps[i].y));

		//Return max 
		return (*max_element(shp_y.begin(), shp_y.end()));
	}
	bool PrePlanner();
	bool RTRPlanner();
	void DrawPrePath();
private:
	bool PathFinder();
	void Triangulate();
	void TCI_Extension(Config q, ConfigInterval &forwardMaxTCI, ConfigInterval &BackwardMaxTCI);
	void InitRTTrees();
private:
	vector<Polygon> envs, envsx;
	Polygon field;
	vector<Triangle> cells;

	ublas::mapped_matrix<int> adjN;	
	ublas::mapped_matrix<float> roadmap_adj;
	vector<Config> roadmap_node;
	vector<Config> prepath;

	Config robotStart;
	Config robotGoal;
	Polygon robotShape;

	Tree startTree;
	Tree goalTree;
};