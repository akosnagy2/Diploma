/* 
 * File:   OccupancyGrid.cpp
 * Author: Gábor
 * 
 * Created on 2014. március 24., 13:04
 */

#include "OccupancyGrid.h"
#include <cmath>

OccupancyGrid::OccupancyGrid(Scene& scene) {
    double dx = scene.getDx();
    double dy = scene.getDy();
    Environment& env = scene.getEnvironment();
    
    double minX = 0 - dx / 2;
    double maxX = env.getWidth() + dx / 2;
    double minY = 0 - dy / 2;
    double maxY = env.getHeight() + dx / 2;

    x = std::vector<double>(floor((maxX - minX) / dx) + 1);
    y = std::vector<double>(floor((maxY - minY) / dy) + 1);
    
    for(unsigned i = 0; i< x.size(); i++)
        x[i] = minX + i * dx;
    
    for(unsigned i = 0; i< y.size(); i++)
        y[i] = minY + i * dy;
    
    grid = std::vector<std::vector<bool> >(x.size(), std::vector<bool>(y.size(), false));
    for(unsigned i=0; i<x.size(); i++)
        for(unsigned j=0;j<y.size();j++)
            if(x[i] < 0 || y[j] < 0 || x[i] > env.getWidth() || y[j] > env.getHeight() || env.isInsideObstacle(Point(x[i], y[j])))
                grid[i][j] = true;
}

unsigned OccupancyGrid::getXSize() {
    return x.size();
}

unsigned OccupancyGrid::getYSize() {
    return y.size();
}

bool OccupancyGrid::isOccupied(unsigned x, unsigned y) {
    return grid[x][y];
}

double OccupancyGrid::getX(unsigned i) {
    return x[i];
}

double OccupancyGrid::getY(unsigned i) {
    return y[i];
}

Point OccupancyGrid::getPoint(unsigned i, unsigned j) {
    return Point(x[i], y[j]);
}
