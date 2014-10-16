/* 
 * File:   Arc.cpp
 * Author: Gábor
 * 
 * Created on 2014. április 2., 19:32
 */

#include <math.h>
#include <float.h>
#include <cmath>

#include "Arc.h"
#include "misc.h"
#include "Point.h"
#include "ARMBuilder.h"
#include "Vector.h"

Arc::Arc(const Configuration& start, const Point& end, bool shorter) {
    Point p = start.inverseTransform(end);

    if (p.x != 0.0 && fabs(p.y / p.x) < 1e-5) {
        this->radius = INFINITY;
        this->dTheta = 0;
        this->start = start;
        this->end = Configuration(end, start.orientation);
        this->length = fabs(p.x);
        this->dir = (p.x >= 0);
    } else if (p.x == 0 && fabs(p.y) < DBL_EPSILON) {
        this->radius = 0;
        this->dTheta = 0;
        this->start = start;
        this->end = start;
        this->length = 0;
        this->dir = false;
    } else {
        this->radius = (p.x * p.x + p.y * p.y) / (2 * p.y);
        if (p.y >= 0) {
            this->dTheta = atan2(p.x, (p.x * p.x - p.y * p.y) / (2 * p.y));
        } else {
            this->dTheta = -atan2(p.x, -(p.x * p.x - p.y * p.y) / (2 * p.y));

            //if atan2 called like atan2(0,y) then is returns 0 instead of -PI
            if (dTheta == 0) {
                dTheta = -M_PI;
            }
        }
        if (!shorter) {
            this->dTheta = -sgn(this->dTheta) * (2 * M_PI - fabs(this->dTheta));
        }
        this->start = start;
        this->end = Configuration(end, wrapAngle(this->dTheta + start.orientation));
        this->length = fabs(this->radius * this->dTheta);
        this->dir = (this->radius * this->dTheta >= 0);
    }
}

double Arc::getRadius() const {
    return radius;
}

/**
 * Translate arc with pos as a vector
 * @param pos Relative to absolute frame.
 * @return Translated arc
 */
Arc& Arc::translateToRobotCornerArc(const Point& pos) {
    if (isinf(radius)) {
        start.position = start.transform(pos);
        end.position = end.transform(pos);
    } else {
        Point center = getCenter();
        start.position = start.transform(pos);
        Vector r(start.position - center);
        start.orientation = wrapAngle(r.getFi() + sgn(radius) * M_PI_2);
        end.position = end.transform(pos);
        radius = sgn(radius) * r.getLength();
    }
    return *this;
}

Arc& Arc::translateToObstacleCornerArc(const Point& pos) {
    if (isinf(radius)) {
        end.position = pos - end.position + start.position;
        start.position = pos;
    } else {
        Point center = getCenter();
        Vector r(pos - center);
        start.position = pos;
        start.orientation = wrapAngle(r.getFi() + sgn(radius) * M_PI_2);
        radius = sgn(radius) * r.getLength();
        dTheta = -dTheta;
        end.position.x = center.x + fabs(radius) * cos(start.orientation - sgn(radius)*M_PI_2 + dTheta);
        end.position.y = center.y + fabs(radius) * sin(start.orientation - sgn(radius)*M_PI_2 + dTheta);
        end.orientation = wrapAngle(start.orientation + dTheta);
    }
    return *this;
}

double Arc::getDTheta() const {
    return dTheta;
}

bool Arc::getDirection() const {
    return dir;
}

std::ostream& operator<<(std::ostream& os, Arc& arc) {
    os << arc.getStartConfig() << ";" << arc.getEndConfig() << ";" << arc.getRadius() << ";" << arc.getDTheta() << ";" << arc.getLength() << ";" << arc.getDirection();
    return os;
}

Point Arc::getCenter() const {
    return Point(start.position.x - radius * sin(start.orientation), start.position.y + radius * cos(start.orientation));
}

configurationList Arc::getPoints(double dr) {
    configurationList config;
    Point center = getCenter();
    double dfi = dr / radius;
    double theta0 = start.orientation;
    double theta1 = end.orientation;
    
    dfi *= dir ? 1 : -1;
    
    for(int i = 0; fabs(dfi/2) < fabs(wrapAngle(theta1 - theta0)); i++) {
        double x1 = center.x + radius*sin(theta0);
        double y1 = center.y - radius*cos(theta0);
        config.push_back(Configuration(x1, y1, theta0));
        theta0 = wrapAngle(theta0 + dfi);
    }
    
    config.push_back(end);
    return config;
}

Arc* Arc::copy() {
    return new Arc(*this);
}
