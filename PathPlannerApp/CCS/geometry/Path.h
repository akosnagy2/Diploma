/* 
 * File:   Path.h
 * Author: Gábor
 *
 * Created on 2014. április 25., 12:06
 */

#ifndef PATH_H
#define	PATH_H

#include "Configuration.h"

class Path {
public:
    virtual configurationList getPoints(double dr) = 0;
    virtual Path& translateToRobotCornerArc(const Point& pos) = 0;
    virtual Path& translateToObstacleCornerArc(const Point& pos) = 0;
    virtual Path* copy() = 0;

    double getLength() const;
    bool getDirection();
    void setDirection(bool dir);
    Configuration getStartConfig() const;
    Configuration getEndConfig() const;
	double getRadius() const;
    
    virtual ~Path() {}
protected:
    double length;
    Configuration start;
    Configuration end;
    bool dir;
	double radius;
};

#endif	/* PATH_H */

