/* 
 * File:   AlternativePlanner.h
 * Author: Gábor
 *
 * Created on 2014. április 26., 14:13
 */

#ifndef ALTERNATIVEPLANNER_H
#define	ALTERNATIVEPLANNER_H

#include <deque>

#include "CCS.h"
#include "Scene.h"
#include "Configuration.h"

class AlternativePlanner {
public:
    AlternativePlanner(Configuration& start, Configuration& goal, Scene& scene);
    bool hasSolution();
    CCS getShortest();
private:
    void calculateARMs();
    void pushCCS(Configuration& cI, Configuration& cG, bool dir);
    void calculateConfigs(double theta, double r, int k);
    
    std::deque<CCS> solutions;
    Scene& scene;
    Configuration& start;
    Configuration& goal;
};

#endif	/* ALTERNATIVEPLANNER_H */

