/* 
 * File:   Planner.h
 * Author: Gábor
 *
 * Created on 2014. április 27., 2:00
 */

#ifndef PLANNER_H
#define	PLANNER_H

#include "Scene.h"
#include "OccupancyGrid.h"
#include "ARMBuilder.h"

class Planner {
public:
    Planner(Scene& scene, configurationList& prePath);
private:
    Scene& scene;
    OccupancyGrid og;
    configurationList& prePath;
    
    unsigned startIndex;
    unsigned endIndex;
    unsigned insertCount;
    ARM arm;
};

#endif	/* PLANNER_H */

