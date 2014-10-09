#include <math.h>

#include "Environment.h"
#include "Shape.h"
#include "misc.h"

Environment::Environment(double width, double height)
: boundary(Point(0, 0), width, height) {
}

void Environment::addObstacle(Shape& shape) {
    obstacles.push_back(shape);
}

Rectangle& Environment::getBoundary() {
    return boundary;
}

double Environment::getWidth() {
    return boundary.getWidth();
}

double Environment::getHeight() {
    return boundary.getHeight();
}

bool Environment::isInsideObstacle(const Point& p) {
    bool retval = false;
    for (unsigned i = 0; i < obstacles.size(); i++)
        retval |= obstacles[i].isInside(p);
    return retval;
}

shapeList& Environment::getObstacles() {
    return obstacles;
}

double Environment::getMinDistance(const Segment& seg) {
    double minDist = INFINITY;

    obstacles.push_back(boundary);
    for (unsigned i = 0; i < obstacles.size(); i++) {
        double obstDist = obstacles[i].getMinDistance(seg);
        if (obstDist < minDist) {
            minDist = obstDist;
        }
    }
    obstacles.pop_back();

    return minDist;
}

bool Environment::isCollisionFree(Path& path) {
    bool collision = false;
    obstacles.push_back(boundary);
    for(unsigned i = 0; i<obstacles.size() && !collision; i++) {
        collision |= obstacles[i].isIntersect(path);
    }
    obstacles.pop_back();
    return !collision;
}

pointList Environment::getAllObstaclePoints() {
    pointList points;
    obstacles.push_back(boundary);
    for(unsigned i = 0; i<obstacles.size(); i++) {
        pointList& obsPoints = obstacles[i].getPoints();
        for (unsigned j = 0; j < obsPoints.size(); j++) {
            points.push_back(obsPoints[j]);
        }
    }
    obstacles.pop_back();
    return points;
}
