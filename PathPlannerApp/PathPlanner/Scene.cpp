#include "Scene.h"

#include "Environment.h"
#include "OccupancyGrid.h"
#include "misc.h"
#include "ARMBuilder.h"
#include "LocalPlanner.h"
#include "AlternativePlanner.h"
#include "PathPlannerApp\CCS\geometry\Point.h"
#include "PathPlannerApp\PathSegment.h"

void PathPlanner::Scene::AddField(float xMax, float yMax)
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

void CCSWrapper(PathPlanner::Scene &s, vector<PathPlanner::PathSegment> &path);
void AddCCSToPath(vector<CCS> ccs, vector<PathPlanner::PathSegment> &path);

void PathPlanner::Scene::CCSPlanner()
{
	CCSWrapper(*this, pathC);
}

Shape ShapeToShape(PathPlanner::Polygon poly)
{
	pointList pl;
	for (auto &p : poly.ps)
	{					
		pl.push_back(Point(p.x, p.y));
	}

	return Shape(pl);
}


std::ofstream logFile;

void _writeLog(string s)
{
	logFile << s;
}

void CCSWrapper(PathPlanner::Scene &s, vector<PathPlanner::PathSegment> &path)
{
	logFile.open("CCS_log.txt");

	//Environment
	Environment env(s.GetFieldXLength(), s.GetFieldYLength());

	//Obstacles
	for (auto &e : s.GetEnv())
	{
		env.addObstacle(ShapeToShape(e));
	}

	//Robot
	Robot robot;
	Configuration start(s.GetStartConfig().p.x, s.GetStartConfig().p.y, s.GetStartConfig().phi);
	Configuration goal(s.GetGoalConfig().p.x, s.GetGoalConfig().p.y, s.GetGoalConfig().phi);
	robot.setStart(start); 
	robot.setGoal(goal);
	robot.setBody(ShapeToShape(s.GetRobotShape()));

    robot.setAxleDistance(310.0);
    robot.setWheelBase(155.0);
    robot.setWheelDiameter(70.0);
    robot.setWheelWidth(25.0);
    robot.setPhiMax(80.0);

	//Get path from RTR
	configurationList config;
	for (auto &e : s.ExtractPath(0))
	{
		config.push_back(Configuration(e.p.x, e.p.y, e.phi));
	}

	Scene sc(env, robot);
    sc.setReversePenalty(1);
    sc.setDx(50);
    sc.setDy(50);
    OccupancyGrid oG(sc);

	/* Initialize path planner */
    unsigned startIndex = 0;
    unsigned endIndex = config.size() - 1;
    unsigned insertCount = 0;
    ARM arm = ARMBuilder(config[startIndex], oG, sc).getARM();

    std::ostringstream stringStream;
    stringStream << "ARM created from configuration #" << startIndex << ".";
    _writeLog(stringStream.str());
    stringStream.str("");

	vector<CCS> ccsVec;
	/* Loop through possible configuration pairs until solution found */
    while (startIndex != config.size() - 1 && endIndex > startIndex && insertCount < 10) {
        /* Get the distance of the next configuration */
        double nextDist = 0;
        if (endIndex != config.size() - 1) {
            nextDist = Point::distance(config[endIndex].position, config[endIndex + 1].position);
        }

        /* Search path between the two configurations */
        LocalPlanner lp(arm, config[endIndex], nextDist, sc);
        stringStream << "Local Planner between configurations #" << startIndex << " and #" << endIndex << ".";
        _writeLog(stringStream.str());
        stringStream.str("");

        /* Last check has a solution */
        if (lp.hasSolution()) {
            _writeLog("Has solution");
            CCS ccs = lp.getShortest();
            stringStream << ccs << std::endl;
            _writeLog(stringStream.str());
            stringStream.str("");

            /* Insert new tangential configuration */
            startIndex = endIndex;
            if (startIndex != config.size() - 1) {
                configurationList::iterator it = config.begin();
                it += ++startIndex;
                config.insert(it, ccs.getMiddle().getEndConfig());

                /* Calculate ARM from the new configuration */
                arm = ARMBuilder(config[startIndex], oG, sc).getARM();
                stringStream << "ARM created from configuration #" << startIndex << ".";
                _writeLog(stringStream.str());
                stringStream.str("");
            }
            endIndex = config.size() - 1;
			ccsVec.push_back(ccs);
			
            /* Draw the found path 
            _drawCCS(ccs, (startIndex == config.size() - 1));
            waitToRun();
            if (terminated) {
                return;
            }
			*/

        } else {
            if (endIndex - startIndex == 1) {
                endIndex = startIndex;
            } else {
                /* Select new configuration from predefined path */
                endIndex = (endIndex - startIndex - 1) / 2 + 1 + startIndex;
            }
        }
    }
	
    if (startIndex != config.size() - 1) {
        _writeLog("Fail!");
    } else 
	{
        _writeLog("Success!");
		startIndex = 0;
		AddCCSToPath(ccsVec, path);
    }

	logFile.close();
}

PathPlanner::Config ConfigToConfig(Configuration c)
{
	return PathPlanner::Config(c.position.x, c.position.y, c.orientation);
}

vector<PathPlanner::Config> ConfigListToConfigVector(configurationList c)
{
	vector<PathPlanner::Config> cc;
	for (auto e : c)
		cc.push_back(ConfigToConfig(e));
	return cc;
}

void AddCCSToPath(vector<CCS> ccs, vector<PathPlanner::PathSegment> &path)
{
	bool lastDir = ccs.front().getFirst().getDirection();

	//First segment
	PathPlanner::PathSegment p;
	p.direction = lastDir;
	path.push_back(p);

	for (auto c : ccs)
	{
		Arc first = c.getFirst();
		Arc second = c.getMiddle();

		//First Arc
		vector<PathPlanner::Config> c1 = ConfigListToConfigVector(first.getPoints(10.0));
		if (first.getDirection() != lastDir) 
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
		path.back().curvature.insert(path.back().curvature.end(), c1.size() - 1, 1/first.getRadius());

		//Second Arc
		vector<PathPlanner::Config> c2 = ConfigListToConfigVector(second.getPoints(10.0));
		if (second.getDirection() != lastDir) 
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
		path.back().curvature.insert(path.back().curvature.end(), c2.size() - 1, 1/second.getRadius());
	}

	//Last C*CS's segment
	Segment seg = ccs.back().getLast();	
	vector<PathPlanner::Config> c3 = ConfigListToConfigVector(seg.getPoints(10.0));
	if (seg.getDirection() != lastDir) 
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