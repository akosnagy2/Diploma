#ifndef _PATHMESSAGE__
#define _PATHMESSAGE__

#include "PackedMessage.h"
#include "PathPlannerApp\PathPlanner\Config.h"

#define PATHMESSAGE_TYPE_CODE "PATH"

using namespace PathPlanner;

class PathMessage
{
public:
	std::vector<Config> path;

	PathMessage()
	{};

	PathMessage(Config* pos, int numOfPos)
	{
		for (int i = 0; i < numOfPos; i++)
			path.push_back(pos[i]);
	}

	void send(iostream &stream);
	int receive(iostream &stream);

	friend std::ostream & operator<<(std::ostream &os, const PathMessage &gp);
};

#endif /* _PATHMESSAGE__ */		
