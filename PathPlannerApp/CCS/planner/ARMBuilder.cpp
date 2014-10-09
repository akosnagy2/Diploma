/* 
 * File:   ARMBuilder.cpp
 * Author: Gábor
 * 
 * Created on 2014. március 27., 15:40
 */

#include "ARMBuilder.h"
#include "Arc.h"
#include "Shape.h"
#include <iostream>
#include <math.h>

ARMBuilder::ARMBuilder(Configuration& startConfig, OccupancyGrid& og, Scene& sc)
: start(startConfig)
, og(og)
, scene(sc) {
    unsigned n = og.getXSize();
    unsigned m = og.getYSize();

    for (unsigned i = 0; i < n; i++) {
        for (unsigned j = 0; j < m; j++) {
            if (!og.isOccupied(i, j)) {
                Arc a(startConfig, og.getPoint(i, j));
                if (fabs(a.getRadius()) > scene.getRobotRMin()) {
                    if (scene.isAdmissible(a)) {
                        reachableArcs.push_back(a);
                    }
                }

                Arc b(startConfig, og.getPoint(i, j), false);
                if (fabs(b.getRadius()) > scene.getRobotRMin()) {
                    if (scene.isAdmissible(b)) {
                        reachableArcs.push_back(b);
                    }
                }
            }
        }
    }

//    for (unsigned i = 0; i < reachableArcs.size(); i++) {
//        std::cout << reachableArcs[i] << std::endl;
//    }
}

ARM& ARMBuilder::getARM() {
    return reachableArcs;
}
