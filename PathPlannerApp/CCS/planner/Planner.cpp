/* 
 * File:   Planner.cpp
 * Author: Gábor
 * 
 * Created on 2014. április 27., 2:00
 */

#include "Planner.h"
#include "ARMBuilder.h"

Planner::Planner(Scene& scene, configurationList& prePath)
: scene(scene)
, og(scene)
, prePath(prePath) {
    startIndex = 0;
    endIndex = prePath.size();
    insertCount = 0;
    arm = ARMBuilder(prePath[startIndex], og, scene).getARM();
}
