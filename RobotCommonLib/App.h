#pragma once

class App
{
private:
	static const int separator = 12;
public:
	bool isCarLikeRobot(void)
	{
		if(appType > separator)
			return true;
		else
			return false;
	}

	bool isDifferentialRobot(void)
	{
		return !isCarLikeRobot();
	}

	bool isPathFollow(void)
	{
		if(appType == DifferentialPathFollow || appType == CarPathFollow || appType == CarRobotPilotFollow)
			return true;
		else
			return false;
	}

	bool isPathPlanner(void)
	{
		if(appType == DifferentialPathPlanner || appType == CarPathPlanner)
			return true;
		else
			return false;
	}

	typedef enum
	{
		DifferentialPathFollow = 2,
		DifferentialPathPlanner = 4,
		DifferentialRobotPilot = 8,
		CarPathFollow = 16,
		CarPathPlanner = 32,
		CarRobotPilotFollow = 64,
		CarRobotPilotPlanner = 128,
	} AppTypedef;

	AppTypedef appType;
};


