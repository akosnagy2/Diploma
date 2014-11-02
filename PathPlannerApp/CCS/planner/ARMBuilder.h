/* 
 * File:   ARMBuilder.h
 * Author: Gábor
 *
 * Created on 2014. március 27., 15:40
 */

#ifndef ARMBUILDER_H
#define	ARMBUILDER_H

#include "Configuration.h"
#include "Arc.h"
#include "OccupancyGrid.h"
#include "Scene.h"

typedef arcList ARM;

class ARMBuilder {
public:
	ARMBuilder();
    ARMBuilder(Configuration& startConfig, OccupancyGrid& og, Scene& sc);
    ARM& getARM();
private:
    ARM reachableArcs;
};

#endif	/* ARMBUILDER_H */

