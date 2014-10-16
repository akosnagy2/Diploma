#pragma once

#include "Configuration.h"
#include "Path.h"
#include "Rectangle.h"


class Environment
{
public:
	Environment(double width, double height);
	void addObstacle(Shape& shape);

        Rectangle& getBoundary();
        shapeList& getObstacles();
        double getMinDistance(const Segment& seg);
        
        double getWidth();
        double getHeight();
        bool isInsideObstacle(const Point& p);
        bool isCollisionFree(Path& path);
        pointList getAllObstaclePoints();
private:
	shapeList obstacles;
	Rectangle boundary;
};

