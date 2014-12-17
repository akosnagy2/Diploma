/* 
 * File:   Arc.h
 * Author: Gábor
 *
 * Created on 2014. április 2., 19:32
 */

#ifndef ARC_H
#define	ARC_H

#include "Configuration.h"
#include "Path.h"

class Arc : public Path {
public:
    Arc(const Configuration& start, const Point& end, bool shorter = true);
    
    double getDTheta() const;
    bool getDirection() const;
    Point getCenter() const;
    
    virtual Arc& translateToRobotCornerArc(const Point& pos);
    virtual Arc& translateToObstacleCornerArc(const Point& pos);
    virtual configurationList getPoints(double dr);
    virtual Arc* copy();

    virtual ~Arc() {}
private:
    double dTheta;
};

std::ostream& operator<<(std::ostream& os,Arc& arc);

typedef std::vector<Arc> arcList;

#endif	/* ARC_H */

