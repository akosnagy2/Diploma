/* 
 * File:   PathSegment.h
 * Author: Gábor
 *
 * Created on 2014. április 19., 1:35
 */

#ifndef PATHSEGMENT_H
#define	PATHSEGMENT_H

#include "Configuration.h"

class PathSegment {
public:
    double getLength() {return length;}
    virtual configurationList getPoints(double dr) = 0;
protected:
    Configuration start;
    Configuration end;
    double length;
    bool dir;
};

#endif	/* PATHSEGMENT_H */

