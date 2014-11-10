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
		if(appType == DifferentialPathFollow || appType == CarPathFollow)
			return true;
		else
			return false;
	}

	bool isPathPlanner(void)
	{
		if(appType == DifferentialPathPlanner || appType == CarPathPlanner
			|| appType == DifferentialRobotPilot || appType == CarRobotPilot)
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
		CarRobotPilot = 64,
	} AppTypedef;

	AppTypedef appType;
};


