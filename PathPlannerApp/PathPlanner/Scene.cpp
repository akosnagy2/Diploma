#include "Scene.h"
#include "CCSPlanner.h"

void PathPlanner::Scene::AddField(float xMax, float yMax)
{
	fieldXLength = xMax;
	fieldYLength = yMax;

	//Create field
	field.ps.clear();
	field.AddPoint(Point(0.0f, 0.0f));
	field.AddPoint(Point(0.0f, yMax));
	field.AddPoint(Point(xMax, yMax));
	field.AddPoint(Point(xMax, 0.0f));

	//Create extended env, when env is available
	if (envs.size())
	{
		envsx = envs;		
		envsx.push_back(field);
	}
}

void PathPlanner::Scene::AddEnv(Polygon env)
{
	envs.push_back(env);

	//Create extended env, when field is available
	envsx = envs;
	if (field.ps.size())
		envsx.push_back(field);
}

float PathPlanner::Scene::GetRobotWidth()
{
	vector<float> shp_y;

	//Get local frame y abs values
	for (int i = 0; i < (int)robotShape.ps.size(); i++)
		shp_y.push_back(fabs(robotShape.ps[i].y));

	//Return max 
	return (*max_element(shp_y.begin(), shp_y.end()));
}

void PathPlanner::Scene::SetRTRParameters(int _maxIteration, float _fixPathProbability, float _roadmapProbability, int randSeed)
{
	maxIteration = _maxIteration;
	fixPathProbability = _fixPathProbability;
	roadmapProbability = _roadmapProbability;
	if (randSeed ==  -1)
		generator.seed(rd());
	else
		generator.seed(randSeed);
}

void PathPlanner::Scene::SetCCSParameters(float _reversePentaltyFactor, float _useIntermediateS, float _insertCount, float _dx)
{
	reversePenaltyFactor = _reversePentaltyFactor;
	useIntermediateS = _useIntermediateS > 0.0f;
	insertCount = (int) _insertCount;
	dx = _dx;
}

bool PathPlanner::Scene::CCSPlanner()
{
	return CCSWrapper(*this, pathC);
}

void PathPlanner::Scene::CalcEnvCenter()
{
	envsxC.clear();
	envsxR.clear();

	for (auto &e : envsx)
	{
		//Center
		Point cent(0.0f, 0.0f);
		for (auto &p : e.ps)
		{
			cent = cent + p;
		}

		cent = cent / e.ps.size();
		envsxC.push_back(cent);
		
		//Radius
		float rad = 0.0f;
		for (auto &p : e.ps)
		{
			float d = Point::Distance(p, cent);
			if (d > rad)
				rad = d;
		}
		envsxR.push_back(rad);
	}
}