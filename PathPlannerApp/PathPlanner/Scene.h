#pragma once

#include <Fade_2D.h>
#include "Tree.h"
#include "Triangle.h"
#include "Config.h"
#include "ConfigInterval.h"
#include "Point.h"
#include "Polygon.h"
#include <vector>
#include <boost/numeric/ublas/matrix_sparse.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>

using namespace std;
using namespace boost::numeric;
using namespace GEOM_FADE2D;

namespace PathPlanner
{

	class Scene
	{
	public:
		Scene()
		{
		}
		~Scene()
		{

		}
		void AddField(float xMax, float yMax)
		{
			fieldXLength = xMax;
			fieldYLength = yMax;

			//Create field
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
		void AddEnv(Polygon env)
		{
			envs.push_back(env);

			//Create extended env, when field is available
			envsx = envs;
			if (field.ps.size())
				envsx.push_back(field);
		}
		void SetStartConfig(Config pos)
		{
			robotStart = pos;
		}
		Config GetStartConfig()
		{
			return robotStart;
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
		void SetRTRParameters(int _maxIteration, float _fixPathProbability, float _roadmapProbability)
		{
			maxIteration = _maxIteration;
			fixPathProbability = _fixPathProbability;
			roadmapProbability = _roadmapProbability;
		}
		void SetFixPrePath(vector<Point> &fPath)
		{
			fixPrePath = fPath;
		}
		void SetFixPrePath(vector<Config> &fPath)
		{
			for (vector<Config>::iterator it = fPath.begin(); it != fPath.end(); ++it)
				fixPrePath.push_back(it->p);
		}
		bool PrePlanner();
		bool RTRPlanner();
		void DrawPrePath();
		bool TurnToPos(Config qStart, Point pos, int turnDir, bool headToGoal, ConfigInterval &maxRCI);
		vector<Config>& ExtractPath();
	private:
		void RTRIteration(bool start);
		bool PathFinder();
		void Triangulate();
		void TCI_Extension(Config q, ConfigInterval &forwardMaxTCI, ConfigInterval &BackwardMaxTCI);
		void InitRTTrees();
		ALCCandidate GetALC(Point &GP, bool start, bool preferredDir);
		bool MergeTreesGetPath();
		int treeTCIMergeability(Tree &tree, ConfigInterval TCI, ConfigInterval &mergingRCI);
		float maxCollFreeTurnAmountPointVsLineseg(Point p, Point center, float dThetaMax, int turnDir, Line s1);
		vector<ConfigInterval> ObtainPath(int startMergeID, int goalMergeID, ConfigInterval mergeRCI, float &pathLength);
		int circleSegLineSegIntersect(Config qStart, float dTheta, float radius, Line s1, Point res[2]);
		//tciTCIMergeability - Checks whether the given TCIs can be merged by a single RCI
		bool tciTCIMergeability(ConfigInterval start, ConfigInterval end, ConfigInterval &mergingRCI, Point &mergingPoint);
		Point GetGuidePoint(bool startPoint);
		bool TurnAndExtend(Tree &tree, Point &p, ALCCandidate &alc, bool headToGoal, vector<int> &recentTCIIDs);
		void OptimizePath();
		void PathTCIExtension(vector<ConfigInterval> &pathExt);
		void ReversePath(vector<ConfigInterval> &path);
	private:
		vector<Polygon> envs, envsx;

		Polygon field;
		float fieldXLength, fieldYLength;
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

		vector<int> recentTCIStartIDs;
		vector<int> recentTCIGoalIDs;

		int maxIteration;
		float fixPathProbability;
		float roadmapProbability;

		vector<Point> fixPrePath;
		int fixPrePathStartIdx, fixPrePathGoalIdx;
		
		vector<ConfigInterval> pathCI;
		vector<Config> pathC;
	};

}