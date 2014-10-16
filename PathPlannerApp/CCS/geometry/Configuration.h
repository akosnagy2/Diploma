#pragma once
#include "Point.h"

#include <deque>

class Configuration {
public:
    Configuration();
    Configuration(const Point& p, double theta);
    Configuration(double x, double y, double theta);
    Point inverseTransform(const Point& point) const;
    Configuration& inverseTransform(const Configuration& frame);
    Point transform(const Point& point) const;
    Configuration& transform(const Configuration& frame);
    Point translate(const Point& point, double dist);

    Point position;
    double orientation;
};

std::ostream& operator<<(std::ostream& os, const Configuration& cf);

typedef std::deque<Configuration> configurationList;
