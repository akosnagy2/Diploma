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

ARMBuilder::ARMBuilder()
{}

ARMBuilder::ARMBuilder(Configuration& startConfig, OccupancyGrid& og, Scene& sc)
{
	unsigned n = og.getXSize();
	unsigned m = og.getYSize();

	for(unsigned i = 0; i < n; i++)
	{
		for(unsigned j = 0; j < m; j++)
		{
			if(!og.isOccupied(i, j))
			{
				Arc a(startConfig, og.getPoint(i, j));
				if(fabs(a.getRadius()) > sc.getRobotRMin())
				{
					if(sc.isAdmissible(a))
					{
						reachableArcs.push_back(a);
					}
				}

				Arc b(startConfig, og.getPoint(i, j), false);
				if(fabs(b.getRadius()) > sc.getRobotRMin())
				{
					if(sc.isAdmissible(b))
					{
						reachableArcs.push_back(b);
					}
				}
			}
		}
	}
}

ARM& ARMBuilder::getARM()
{
	return reachableArcs;
}
