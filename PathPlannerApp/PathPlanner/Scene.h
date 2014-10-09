#pragma once

#include <Fade_2D.h>
#include "Tree.h"
#include "Triangle.h"
#include "Config.h"
#include "ConfigInterval.h"
#include "Point.h"
#include "Polygon.h"
#include "PathPlannerApp\PathSegment.h"
#include <vector>
#include <boost/numeric/ublas/matrix_sparse.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <random>

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
		void AddField(float xMax, float yMax);
		void AddEnv(Polygon env);
		vector<Polygon>& GetEnv()
		{
			return envs;
		}
		float GetFieldXLength()
		{
			return fieldXLength;
		}
		float GetFieldYLength()
		{
			return fieldYLength;
		}
		void SetStartConfig(Config pos)
		{
			robotStart = pos;
		}
		Config& GetStartConfig()
		{
			return robotStart;
		}
		void SetGoalConfig(Config pos)
		{
			robotGoal = pos;
		}
		Config& GetGoalConfig()
		{
			return robotGoal;
		}
		void SetRobotShape(Polygon shp)
		{
			robotShape = shp;		
		}
		Polygon& GetRobotShape()
		{
			return robotShape;
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
		vector<PathSegment>& GetCCSPath()
		{
			return pathC;
		}
		float GetRobotWidth();
		void SetRTRParameters(int _maxIteration, float _fixPathProbability, float _roadmapProbability, int randSeed);
		void CCSPlanner();
		bool PrePlanner();
		bool RTRPlanner();
		void DrawPrePath();
		bool TurnToPos(Config qStart, Point pos, int turnDir, bool headToGoal, ConfigInterval &maxRCI);
		vector<Config> Scene::ExtractPath(int interpolate);
		void DrawScene(int iteration);
		void DrawPath();
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
		int circleSegLineSegIntersect(Point center, float angleStart, float dTheta, float radius, Line s1, Point res[2]);
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
		public:Polygon robotShape;

		Tree startTree;
		Tree goalTree;

		vector<int> recentTCIStartIDs;
		vector<int> recentTCIGoalIDs;

		int maxIteration;
		float fixPathProbability;
		float roadmapProbability;

		vector<Point> fixPrePath;
		int fixPrePathStartIdx, fixPrePathGoalIdx;

		default_random_engine generator;
		random_device rd;

		vector<ConfigInterval> pathCI;
		vector<PathSegment> pathC;
	};

}