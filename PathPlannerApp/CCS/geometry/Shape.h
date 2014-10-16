#pragma once

#include <vector>
#include "Point.h"
#include "Segment.h"

class Shape
{
public:
	Shape();
	Shape(pointList& list);
	pointList& getPoints();
	const Point& getPoint(int index);
	int getNumberOfPoints();
	Point getInsidePoint();
	bool isInside(const Point& p);
        double getMinDistance(const Segment& segment);
        bool isIntersect(Path& path);
        Shape& transform(const Configuration& conf);
protected:
	void setPoints(pointList& list);
        pointList points;	
};

typedef std::vector<Shape> shapeList;