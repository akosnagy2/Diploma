#include "PathPlannerApp\path_planner_funcs.h"
#include "Polygon.h"
#include "Scene.h"
#include <chrono>

using namespace std;
using namespace std::chrono;

vector<Point> LoadPathFromFile(string filename)
{
	vector<Point> path;
	string line;
	ifstream fs(filename);
	Point pos;

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

int main()
{
	Scene sc;
	sc.AddField(32.0f, 15.0f);
	sc.SetFixPrePath(LoadPathFromFile("RTRPath.txt"));

	Polygon env1;
	env1.AddPoint(Point(11.0f, 0.0f));
	env1.AddPoint(Point(21.0f, 0.0f));
	env1.AddPoint(Point(21.0f, 6.0f));
	env1.AddPoint(Point(11.0f, 6.0f));
	sc.AddEnv(env1);

	Polygon env2;
	env2.AddPoint(Point(11.0f, 9.0f));
	env2.AddPoint(Point(21.0f, 9.0f));
	env2.AddPoint(Point(21.0f, 15.0f));
	env2.AddPoint(Point(11.0f, 15.0f));
	sc.AddEnv(env2);

	sc.SetStartConfig(Config(9.95f, 11.3f, PI/2));
	sc.SetGoalConfig(Config(22.15f, 0.6f, PI/2));

	Polygon rob;
	rob.AddPoint(Point(3.55f, 1.0f));
	rob.AddPoint(Point(-0.45f, 1.0f));
	rob.AddPoint(Point(-0.45f, -1.0f));
	rob.AddPoint(Point(3.55f, -1.0f));
	sc.SetRobotShape(rob);

	//sc.PrePlanner();
	//sc.DrawPrePath();


	sc.SetRTRParameters(1000, 0.0f, 1.0f);

	chrono::high_resolution_clock::time_point start, stop;
	start = high_resolution_clock::now();
	sc.RTRPlanner();
	stop = high_resolution_clock::now();
	cout << duration_cast<chrono::microseconds>(stop-start).count() << " us." << endl;

	return 0;
}