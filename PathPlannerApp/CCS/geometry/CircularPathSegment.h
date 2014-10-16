/* 
 * File:   CircularPathSegment.h
 * Author: Gábor
 *
 * Created on 2014. április 19., 23:36
 */

#ifndef CIRCULARPATHSEGMENT_H
#define	CIRCULARPATHSEGMENT_H

#include "PathSegment.h"

#include "Arc.h"

class CircularPathSegment : public PathSegment {
public:
    CircularPathSegment(Arc& arc);
    virtual configurationList getPoints(double dr);

private:
    Point center;
    double radius;
};

#endif	/* CIRCULARPATHSEGMENT_H */

