#pragma once

#include "Point.h"
#include "Path.h"
#include "Arc.h"
#include <utility>
#include <vector>

class Segment : public Path {
public:
    Segment(const Point& p1, const Point& p2, bool dir = true);
    Segment(const Configuration& start, const Configuration& end, bool dir);
    
    bool isIntersect(const Segment& s, Point& p) const;
    bool isIntersect(const Segment& s) const;
    bool isIntersect(const Arc& arc) const;
    bool isIntersect(Path& path) const;
    bool isProjectedInside(const Point& p, Point& projection) const;
    double getDistance(const Segment& s) const;
    Point getA() const;
    Point getB() const;
    double getOrientation() const;

    virtual Segment& translateToRobotCornerArc(const Point& pos);
    virtual Segment& translateToObstacleCornerArc(const Point& pos);
    virtual configurationList getPoints(double dr);
    virtual Segment* copy();
   
    virtual ~Segment() {}
};

typedef std::vector<Segment> segmentList;