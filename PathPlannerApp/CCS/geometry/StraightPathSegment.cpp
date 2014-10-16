/* 
 * File:   StraightPathSegment.cpp
 * Author: Gábor
 * 
 * Created on 2014. április 19., 2:14
 */

#include <math.h>

#include "StraightPathSegment.h"
#include "misc.h"

StraightPathSegment::StraightPathSegment(Segment& segment, bool dir) {
    double angle = segment.getOrientation();
    if (dir) {
        start = Configuration(segment.getA(), angle);
        end = Configuration(segment.getB(), angle);
    } else {
        start = Configuration(segment.getA(), wrapAngle(angle + M_PI));
        end = Configuration(segment.getB(), wrapAngle(angle + M_PI));
    }
    this->dir = dir;
    length = Point::distance(start.position, end.position);
}

configurationList StraightPathSegment::getPoints(double dr) {
    configurationList points;
    double conf = start.orientation;
    Point startPoint = start.position;
    dr *= dir ? 1 : -1;
    double dx = dr * cos(conf);
    double dy = dr * sin(conf);
    for (int i = 0; fabs(i * dr) < length; i++) {
        Point p(i*dx, i*dy);
        points.push_back(Configuration(startPoint + p, conf));
    }
    points.push_back(end);
    return points;
}
