#pragma once

#include <Fade_2D.h>
#include "Tree.h"
#include "Geometry\Triangle.h"
#include "Geometry\Config.h"
#include "ConfigInterval.h"
#include "Geometry\Point.h"
#include "Geometry\Polygon.h"
#include "PathSegment.h"
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
		vector<PathSegment>& GetPath()
		{
			return pathC;
		}
		vector<ConfigInterval>& GetCIPath()
		{
			return pathCI;
		}
		void SetPathDeltaS(float ds)
		{
			pathDeltaS = ds;
		}
		float GetPathDeltaS()
		{
			return pathDeltaS;
		}
		void SetRobotWheelBase(float wb)
		{
			robotWheelBase = wb;
		}
		float GetRobotWheelBase()
		{
			return robotWheelBase;
		}
		void SetRobotMinimumRadius(float r)
		{
			robotMinimumRadius = r;
		}
		float GetRobotMinimumRadius()
		{
			return robotMinimumRadius;
		}
		float GetRobotWidth();
		void SetRTRParameters(int _maxIteration, float _fixPathProbability, float _roadmapProbability, int randSeed);
		bool CCSPlanner();
		bool PrePlanner();
		bool RTRPlanner();
		void DrawPrePath();
		bool TurnToPos(Config qStart, Point pos, int turnDir, bool headToGoal, ConfigInterval &maxRCI);
		void GenerateRTRPath();
		void DrawScene(int iteration);
		void DrawPath();

<<<<<<< HEAD
=======
		vector<Config> getPrePath() { return prepath; }

>>>>>>> origin/car-like-robot
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
		bool tciTCIMergeability(ConfigInterval start, ConfigInterval end, ConfigInterval &mergingRCI, Point &mergingPoint);
		Point GetGuidePoint(bool startPoint);
		bool TurnAndExtend(Tree &tree, Point &p, ALCCandidate &alc, bool headToGoal, vector<int> &recentTCIIDs);
		void OptimizePath();
		void PathTCIExtension(vector<ConfigInterval> &pathExt);
		void ReversePath(vector<ConfigInterval> &path);
	private:
		vector<Polygon> envs; //Obstacles
		vector<Polygon> envsx; //Obstacles with field

		Polygon field;
		float fieldXLength, fieldYLength;
		
		//PrePlanner
		vector<Triangle> cells;
		ublas::mapped_matrix<int> adjN;	
		ublas::mapped_matrix<float> roadmap_adj;
		vector<Config> roadmap_node;
		vector<Config> prepath;

		//Robot
		Config robotStart;
		Config robotGoal;
		Polygon robotShape;
		float robotWheelBase;
		float robotMinimumRadius;

		//RTR
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

		//Path
		vector<ConfigInterval> pathCI;
		vector<PathSegment> pathC;
		float pathDeltaS;
	};

}