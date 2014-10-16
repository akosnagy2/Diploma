/* 
 * File:   OccupancyGrid.h
 * Author: Gábor
 *
 * Created on 2014. március 24., 13:04
 */

#ifndef OCCUPANCYGRID_H
#define	OCCUPANCYGRID_H

#include "Scene.h"
#include <vector>

class OccupancyGrid {
public:
    OccupancyGrid(Scene& scene);
    unsigned getXSize();
    unsigned getYSize();
    bool isOccupied(unsigned x, unsigned y);
    double getX(unsigned i);
    double getY(unsigned i);
    Point getPoint(unsigned i, unsigned j);
private:
    std::vector<std::vector<bool> > grid;
    std::vector<double> x;
    std::vector<double> y;
};

#endif	/* OCCUPANCYGRID_H */

