/* 
 * File:   Triangulator.h
 * Author: Gábor
 *
 * Created on 2014. március 24., 19:48
 */

#ifndef TRIANGULATOR_H
#define	TRIANGULATOR_H

#include "Environment.h"
#include "Triangle.h"
#include "Robot.h"

typedef std::vector<Triangle> triangleList;
typedef std::vector<std::vector<double> > adjacencyMatrix;

class Triangulator {
public:
    Triangulator(Environment& env, Robot& robot);
    triangleList& getTriangles();
    pointList& getCommonPoints();
    adjacencyMatrix& getCommonPointConnection();

private:
    Environment& env;
    Robot& robot;
    std::vector<std::vector<char> > adjN;
    triangleList triangles;
    pointList commonPoints;
    adjacencyMatrix commonPointDistances;
    unsigned startIndex;
    unsigned goalIndex;
    
    void triangulateEnv();
    void createAdj();
    void insertStartGoal();
    void removeCloseEdges();
};

#endif	/* TRIANGULATOR_H */

