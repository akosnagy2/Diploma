/* 
 * File:   StraightPathSegment.h
 * Author: Gábor
 *
 * Created on 2014. április 19., 2:14
 */

#ifndef STRAIGHTPATHSEGMENT_H
#define	STRAIGHTPATHSEGMENT_H

#include "PathSegment.h"
#include "Segment.h"


class StraightPathSegment : public PathSegment {
public:
    StraightPathSegment(Segment& segment, bool dir);
    virtual configurationList getPoints(double dr);
};

#endif	/* STRAIGHTPATHSEGMENT_H */

