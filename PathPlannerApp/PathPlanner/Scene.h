#pragma once

#include <Fade_2D.h>
#include "Tree.h"
#include "Geometry\Triangle.h"
#include "Geometry\Config.h"
#include "ConfigInterval.h"
#include "Geometry\Point.h"
#include "Geometry\Polygon.h"
#include "Geometry/Frame.h"
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
		Scene() {}
		~Scene() {}

		//Getters, setters
		void SetFixPrePath(vector<Point> &fPath) { fixPrePath = fPath; }
		void SetFixPrePath(vector<Config> &fPath);
		vector<PathSegment>& GetPath() { return pathC; }
		vector<ConfigInterval>& GetCIPath() { return pathCI; }
		void SetPathDeltaS(float ds) { pathDeltaS = ds; }
		float GetPathDeltaS() { return pathDeltaS; }
		void SetRobotWheelBase(float wb) { robotWheelBase = wb; }
		float GetRobotWheelBase() { return robotWheelBase; }
		void SetRobotMinimumRadius(float r) { robotMinimumRadius = r; }
		float GetRobotMinimumRadius() { return robotMinimumRadius; }
		void SetFrame(Frame& f) { frame = f; }
		Frame& GetFrame() { return frame; }
		float GetRobotWidth();
		void SetRTRParameters(int _maxIteration, float _fixPathProbability, float _roadmapProbability, int randSeed);

		//Planners
		bool PrePlanner();
		bool RTRPlanner();
		void DrawPrePath();
		bool TurnToPos(Config qStart, Point pos, int turnDir, bool headToGoal, ConfigInterval &maxRCI);
		void GenerateRTRPath();
		void DrawScene(int iteration);
		void DrawPath();
		vector<Config> getPrePath() { return prepath; }

		/* C*CS */
		bool CCSPlanner();
		void SetCCSParameters(float _reversePentaltyFactor, float _useIntermediateS, float _insertCount, float _dx);
		float GetReversePenaltyFactor() { return reversePenaltyFactor; }
		bool IsUseIntermediateS() { return useIntermediateS; }
		int GetInsertCount() { return insertCount; }
		float GetDx() { return dx; }

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
		//PrePlanner
		vector<Triangle> cells;
		ublas::mapped_matrix<int> adjN;	
		ublas::mapped_matrix<float> roadmap_adj;
		vector<Config> roadmap_node;
		vector<Config> prepath;

		//Robot
		float robotWheelBase;
		float robotMinimumRadius;

		//Frame
		Frame frame;

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

		//C*CS
		float reversePenaltyFactor;
		bool useIntermediateS;
		int insertCount;
		float dx;

		//Path
		vector<ConfigInterval> pathCI;
		vector<PathSegment> pathC;
		float pathDeltaS;
	};

}