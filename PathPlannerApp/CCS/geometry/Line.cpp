/* 
 * File:   Line.cpp
 * Author: Gábor
 * 
 * Created on 2014. március 29., 14:19
 */

#include <float.h>
#include <math.h>

#include "Line.h"
#include "Vector.h"

Line::Line(const Point& point, double angle)
: point(point)
, angle(angle) {
}

Line::Line(const Configuration& config)
: point(config.position)
, angle(config.orientation) {
}

double Line::getAngle() const {
    return angle;
}

const Point& Line::getPoint() const {
    return point;
}

Line::Line(const Segment& segment)
: point(segment.getA())
, angle(segment.getOrientation()) {
}

bool Line::isIntersect(const Line& line, Point& intersection) {
    /* 2D normal vectors */
    double A1 = -sin(this->angle);
    double B1 = cos(this->angle);
    double C1 = A1 * this->point.x + B1 * this->point.y;
    double A2 = -sin(line.angle);
    double B2 = cos(line.angle);
    double C2 = A2 * line.point.x + B2 * line.point.y;

    /* Determinant check (parallel if zero) */
    double det = A1 * B2 - A2 * B1;
    if (fabs(det) > DBL_EPSILON) {
        intersection.x = (B2 * C1 - B1 * C2) / det;
        intersection.y = (A1 * C2 - A2 * C1) / det;
        return true;
    } else {
        return false;
    }
}

Point Line::getProjection(const Point& p) const {
    Vector e(cos(angle), sin(angle));
    Vector x(p-point);
    
    return point + e * (x * e);
}
