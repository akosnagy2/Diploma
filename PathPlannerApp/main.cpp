#include "PathPlannerApp\path_planner_funcs.h"

using namespace std;

int main()
{
	Polygon field;
	field.AddPoint(Point(0.0f, 0.0f));
	field.AddPoint(Point(32.0f, 0.0f));
	field.AddPoint(Point(32.0f, 15.0f));
	field.AddPoint(Point(0.0f, 15.0f));
	Scene sc("scene.ps");
	sc.AddField(field);

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

	sc.SetStartPosition(Position(9.95f, 11.3f, 3.14f/2));
	sc.SetGoalPosition(Position(22.15f, 0.6f, 3.14f/2));

	Polygon rob;
	rob.AddPoint(Point(3.55f, 1.0f));
	rob.AddPoint(Point(-0.45f, 1.0f));
	rob.AddPoint(Point(-0.45f, -1.0f));
	rob.AddPoint(Point(3.55f, -1.0f));
	sc.SetRobotShape(rob);

	sc.PrePlanner();
	sc.DrawPrePath();

	return 0;
}