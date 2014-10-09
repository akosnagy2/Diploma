#include "Shape.h"

#include "misc.h"
#include <cmath>

Shape::Shape(pointList& list) :
points(list) {
}

Shape::Shape() {
}

void Shape::setPoints(pointList& list) {
    points = list;
}

pointList& Shape::getPoints() {
    return points;
}

const Point& Shape::getPoint(int index) {
    return points[index];
}

int Shape::getNumberOfPoints() {
    return points.size();
}

Point Shape::getInsidePoint() {
    Point p;
    for (unsigned i = 0; i < points.size(); i++) {
        p += points[i];
    }
    p /= (double) points.size();
    return p;
}

bool Shape::isInside(const Point& p) {
    int size = points.size();
    double sumAngle = 0;
    for (int i = 0, j; i < size; i++) {
        if (i == size - 1)
            j = 0;
        else
            j = i + 1;

        Point v1 = points[i] - p;
        Point v2 = points[j] - p;
        sumAngle += wrapAngle(atan2(v2.getY(), v2.getX()) - atan2(v1.getY(), v1.getX()));
    }

    if (fabs(sumAngle) > 1)
        return true;
    else
        return false;
}

double Shape::getMinDistance(const Segment& segment) {
    double minDist = INFINITY;
    for (unsigned i = 0; i < points.size(); i++) {
        double segDist = segment.getDistance(Segment(points[i], points[(i + 1) % points.size()]));
        if (segDist < minDist)
            minDist = segDist;
    }
    return minDist;
}

bool Shape::isIntersect(Path& path) {
    bool intersect = false;
    for (unsigned i = 0; i < points.size() && !intersect; i++) {
        Segment s(points[i], points[(i + 1) % points.size()]);
        intersect |= s.isIntersect(path);
    }
    return intersect;
}

Shape& Shape::transform(const Configuration& conf) {
    for(unsigned i = 0; i<points.size(); i++) {
        points[i] = conf.transform(points[i]);
    }
    return *this;
}
