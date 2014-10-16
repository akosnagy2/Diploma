/* 
 * File:   Line.h
 * Author: Gábor
 *
 * Created on 2014. március 29., 14:19
 */

#ifndef LINE_H
#define	LINE_H

#include "Point.h"
#include "Configuration.h"
#include "Segment.h"


class Line {
public:
    Line(const Point& point, double angle);
    Line(const Configuration& config);
    Line(const Segment& segment);
    double getAngle() const;
    const Point& getPoint() const;
    bool isIntersect(const Line& line, Point& intersection);
    Point getProjection(const Point& p) const;
private:
    Point point;
    double angle;
};

#endif	/* LINE_H */

