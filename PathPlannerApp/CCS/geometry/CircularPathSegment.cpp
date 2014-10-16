/* 
 * File:   CircularPathSegment.cpp
 * Author: Gábor
 * 
 * Created on 2014. április 19., 23:36
 */

#include <math.h>

#include "CircularPathSegment.h"
#include "Arc.h"
#include "misc.h"

CircularPathSegment::CircularPathSegment(Arc& arc) {
    this->start = arc.getStartConfig();
    this->end = arc.getEndConfig();
    this->length = arc.getDistance();
    this->radius = arc.getRadius();
    this->center = arc.getCenter();
    this->dir = arc.getDirection();
}

configurationList CircularPathSegment::getPoints(double dr) {
    configurationList config;
    double dfi = dr / radius;
    double theta0 = start.orientation;
    double theta1 = end.orientation;
    
    dfi *= dir ? 1 : -1;
    
    for(int i = 0; fabs(dfi) <= fabs(wrapAngle(theta1 - theta0)); i++) {
        double x1 = center.x + radius*sin(theta0);
        double y1 = center.y - radius*cos(theta0);
        config.push_back(Configuration(x1, y1, theta0));
        theta0 = wrapAngle(theta0 + dfi);
    }
    
    config.push_back(end);
    return config;
}
