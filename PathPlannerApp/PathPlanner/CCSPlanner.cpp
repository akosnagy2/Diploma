#include "Environment.h"
#include "OccupancyGrid.h"
#include "misc.h"
#include "ARMBuilder.h"
#include "LocalPlanner.h"
#include "AlternativePlanner.h"
#include "Point.h"
#include "Scene.h"
#include "CCSPlanner.h"

void AddCCSToPath(vector<CCS> ccs, vector<PathPlanner::PathSegment> &path, float ds);

Shape ShapeToShape(PathPlanner::Polygon poly)
{
	pointList pl;
	for(auto &p : poly.ps)
	{
		pl.push_back(Point(p.x, p.y));
	}

	return Shape(pl);
}


std::ofstream logFile;

void _writeLog(string s)
{
	logFile << s << endl;
}

bool CCSWrapper(PathPlanner::Scene &s, vector<PathPlanner::PathSegment> &path)
{
	bool ret;
	ccsStateTypedef state = ccsStateTypedef::arm;
	logFile.open("CCS_log.txt");

	//Environment
	Environment env(s.GetFieldXLength(), s.GetFieldYLength());

	//Obstacles
	for(auto &e : s.GetEnv())
		env.addObstacle(ShapeToShape(e));

	//Robot
	Robot robot;
	Configuration start(s.GetStartConfig().p.x, s.GetStartConfig().p.y, s.GetStartConfig().phi);
	Configuration goal(s.GetGoalConfig().p.x, s.GetGoalConfig().p.y, s.GetGoalConfig().phi);

	robot.setStart(start);
	robot.setGoal(goal);
	robot.setBody(ShapeToShape(s.GetRobotShape()));

	robot.setAxleDistance(s.GetRobotMinimumRadius());
	robot.setWheelBase(s.GetRobotWheelBase());
	robot.setPhiMax(M_PI_4); //No phiMax, only Rmin

	//Get path from RTR
	configurationList config;
	for(auto &e : s.GetCIPath())
	{
		if(e.type == PathPlanner::TranslationCI)
			config.push_back(Configuration(e.q0.p.x, e.q0.p.y, e.q0.phi));
	}
	config.push_back(Configuration(s.GetCIPath().back().q1.p.x, s.GetCIPath().back().q1.p.y, s.GetCIPath().back().q1.phi));

	Scene sc(env, robot);
	sc.setReversePenalty(1);
	double ds = min(s.GetFieldXLength() / 50.0, s.GetFieldYLength() / 50.0);
	ds = 50.0;
	sc.setDx(ds);
	sc.setDy(ds);
	OccupancyGrid oG(sc);

	/* Initialize path planner */
	vector<CCS> ccsVec;
	ARM arm;
	LocalPlanner lp(sc);
	unsigned startIndex = 0;
	unsigned endIndex = config.size() - 1;
	unsigned insertCount = 0;
	double nextDist = 0;
	bool isConfigLeft = true;
	while(isConfigLeft)
	{
		switch(state)
		{
			case ccsStateTypedef::arm:
				arm = ARMBuilder(config[startIndex], oG, sc).getARM();
				logFile << "ARM created from configuration #" << startIndex << "." << endl;
				state = ccsStateTypedef::localPlanner;
				break;

			case ccsStateTypedef::localPlanner:
				if(endIndex != config.size() - 1)
					nextDist = Point::distance(config[endIndex].position, config[endIndex + 1].position);
				else
					nextDist = 0.0;

				lp = LocalPlanner(arm, config[endIndex], nextDist, sc);
				logFile << "Local Planner between configurations #" << startIndex << " and #" << endIndex << "." << endl;
				if(lp.hasSolution())
					state = ccsStateTypedef::solution;
				else
					state = ccsStateTypedef::fail;
				break;

			case ccsStateTypedef::solution:
				startIndex = endIndex;
				/* Check if this was the last segment */
				if(startIndex != config.size() - 1)
				{
					CCS ccs = lp.getShortest();
					//TODO itt lehene kiírni a CCS eredményt ha érdekel valakit
					ccsVec.push_back(ccs);

					/* Insert intermediate configuration */
					//TODO itt el kéne gondolkozni rajta, hogy az S szakaszt kihagyjuk-e
					configurationList::iterator it = config.begin();
					it += ++startIndex;
					config.insert(it, ccs.getMiddle().getEndConfig());

					state = ccsStateTypedef::arm;
					endIndex = config.size() - 1;
				}
				else
				{
					isConfigLeft = false;
				}
				break;

			case ccsStateTypedef::fail:
				if(endIndex - startIndex == 1)
				{
					isConfigLeft = false;
					//TODO: Ide jön a c_cS
				}
				else
				{
					/* Select new configuration from predefined path */
					endIndex = (endIndex - startIndex - 1) / 2 + 1 + startIndex;
				}
				break;

			default:
				isConfigLeft = false;
				break;
		}
	}

	if(startIndex != config.size() - 1)
	{
		logFile << "Fail!" << endl;
		cout << "Fail!" << endl;
		ret = false;
	}
	else
	{
		logFile << "Success!" << endl;
		cout << "Success!" << endl;
		startIndex = 0;
		AddCCSToPath(ccsVec, path, s.GetPathDeltaS());
		ret = true;
	}

	logFile.close();
	return ret;
}

PathPlanner::Config ConfigToConfig(Configuration c)
{
	return PathPlanner::Config((float) c.position.x, (float) c.position.y, (float) c.orientation);
}

vector<PathPlanner::Config> ConfigListToConfigVector(configurationList c)
{
	vector<PathPlanner::Config> cc;
	for(auto e : c)
		cc.push_back(ConfigToConfig(e));
	return cc;
}

void AddCCSToPath(vector<CCS> ccs, vector<PathPlanner::PathSegment> &path, float ds)
{
	bool lastDir = ccs.front().getFirst().getDirection();

	//Clear path
	path.clear();

	//First segment
	PathPlanner::PathSegment p;
	p.direction = lastDir;
	path.push_back(p);

	for(auto c : ccs)
	{
		Arc first = c.getFirst();
		Arc second = c.getMiddle();

		//First Arc
		vector<PathPlanner::Config> c1 = ConfigListToConfigVector(first.getPoints(ds));
		if(first.getDirection() != lastDir)
		{
			//Close last segment
			path.back().path.push_back(c1.front());
			path.back().curvature.push_back(path.back().curvature.back());

			//Direction changed -> new segment
			PathPlanner::PathSegment p;
			p.direction = first.getDirection();
			path.push_back(p);

			//Set last direction
			lastDir = first.getDirection();
		}

		path.back().path.insert(path.back().path.end(), c1.begin(), c1.end() - 1);
		path.back().curvature.insert(path.back().curvature.end(), c1.size() - 1, (float) (1 / first.getRadius()));

		//Second Arc
		vector<PathPlanner::Config> c2 = ConfigListToConfigVector(second.getPoints(ds));
		if(second.getDirection() != lastDir)
		{
			//Close last segment
			path.back().path.push_back(c2.front());
			path.back().curvature.push_back(path.back().curvature.back());

			//Direction changed -> new segment
			PathPlanner::PathSegment p;
			p.direction = second.getDirection();
			path.push_back(p);

			//Set last direction
			lastDir = second.getDirection();
		}

		path.back().path.insert(path.back().path.end(), c2.begin(), c2.end() - 1);
		path.back().curvature.insert(path.back().curvature.end(), c2.size() - 1, (float) (1 / second.getRadius()));
	}

	//Last C*CS's segment
	Segment seg = ccs.back().getLast();
	vector<PathPlanner::Config> c3 = ConfigListToConfigVector(seg.getPoints(ds));
	if(seg.getDirection() != lastDir)
	{
		//Close last segment
		path.back().path.push_back(c3.front());
		path.back().curvature.push_back(path.back().curvature.back());

		//Direction changed -> new segment
		PathPlanner::PathSegment p;
		p.direction = seg.getDirection();
		path.push_back(p);

		//Set last direction
		lastDir = seg.getDirection();
	}

	path.back().path.insert(path.back().path.end(), c3.begin(), c3.end());
	path.back().curvature.insert(path.back().curvature.end(), c3.size(), 0.0);
}