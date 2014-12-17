/* 
 * File:   LocalPlanner.h
 * Author: Gábor
 *
 * Created on 2014. április 8., 15:25
 */

#pragma once

#include "ARMBuilder.h"
#include "CCS.h"


class LocalPlanner {
public:
    LocalPlanner(ARM& arm, Configuration& goal, double nextDist, Scene& scene);
    bool hasSolution();
    CCS getShortest();
private:
    arcList getTangentArcs(const Configuration& middle, Configuration& goal);
    double calcuateDistance(unsigned middleIndex);
    double calcuateDistance(Arc& first, Arc& middle, Segment& end);
    
    arcList middleSegments;
    segmentList lastSegments;
    std::vector<unsigned> firstMiddleSegmentsMap;
    std::vector<double> fullDistances;
    Scene& scene;
    ARM& arm;
    Configuration& goal;
    double nextDist;
};
