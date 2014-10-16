/* 
 * File:   Preplanner.h
 * Author: Gábor
 *
 * Created on 2014. március 25., 10:48
 */

#ifndef PREPLANNER_H
#define	PREPLANNER_H

#include "Shape.h"
#include "Triangulator.h"


class Preplanner {
public:
    Preplanner(Robot& robot, pointList& points, adjacencyMatrix& connections);
    configurationList& getPath();
    bool isPathExist();
private:
    configurationList configPath;
    bool isPath;
};

#endif	/* PREPLANNER_H */

