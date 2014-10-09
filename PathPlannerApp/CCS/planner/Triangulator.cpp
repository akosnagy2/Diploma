/* 
 * File:   Triangulator.cpp
 * Author: Gábor
 * 
 * Created on 2014. március 24., 19:48
 */

#include "Triangulator.h"

extern "C" {
#define REAL double
#define ANSI_DECLARATORS
#include "lib/triangle.h"
}
#include "Segment.h"
#include <cstdlib>
#include <cmath>

Triangulator::Triangulator(Environment& env, Robot& robot) :
env(env),
robot(robot) {
    triangulateEnv();
    createAdj();
    insertStartGoal();
    removeCloseEdges();
}

triangleList& Triangulator::getTriangles() {
    return triangles;
}

pointList& Triangulator::getCommonPoints() {
    return commonPoints;
}

adjacencyMatrix& Triangulator::getCommonPointConnection() {
    return commonPointDistances;
}

void Triangulator::triangulateEnv() {
    struct triangulateio in, out;

    /* Initialize structure */
    in.numberofpoints = 0;
    in.numberofpointattributes = 0;
    in.holelist = 0;
    in.numberofsegments = 0;
    in.numberofregions = 0;

    /* Set unused pointers */
    in.segmentmarkerlist = (int *) 0;
    in.pointattributelist = (REAL *) 0;
    in.pointmarkerlist = (int *) 0;
    in.regionlist = (REAL *) 0;
    in.pointlist = (REAL *) 0;
    in.segmentlist = (int *) 0;
    in.holelist = (REAL *) 0;

    /* Count the total number of points */
    in.numberofpoints += env.getBoundary().getNumberOfPoints();
    shapeList& obstacles = env.getObstacles();
    for (unsigned i = 0; i < obstacles.size(); i++) {
        in.numberofpoints += obstacles[i].getNumberOfPoints();
    }

    /* Set the counters */
    in.numberofsegments = in.numberofpoints;
    in.numberofholes = obstacles.size();

    try {
        /* Allocate memory */
        in.pointlist = new REAL[in.numberofpoints * 2];
        in.segmentlist = new int[in.numberofsegments * 2];
        in.holelist = new REAL[in.numberofholes * 2];

        int pointIndex = 0;
        int segmentIndex = 0;
        int holeIndex = 0;

        /* Insert the boundary points and segments */
        const pointList& pList = env.getBoundary().getPoints();
        for (unsigned i = 0; i < pList.size(); i++) {
            /* Save the point */
            in.pointlist[pointIndex * 2] = pList[i].getX();
            in.pointlist[pointIndex * 2 + 1] = pList[i].getY();
            if (i != 0) {
                /* Save the segment */
                in.segmentlist[segmentIndex * 2] = pointIndex - 1;
                in.segmentlist[segmentIndex * 2 + 1] = pointIndex;
                segmentIndex++;
            }
            pointIndex++;
        }
        /* Save the last segment */
        in.segmentlist[segmentIndex * 2] = pointIndex - 1;
        in.segmentlist[segmentIndex * 2 + 1] = 0;
        segmentIndex++;

        for (unsigned i = 0; i < obstacles.size(); i++) {
            pointList pList = obstacles[i].getPoints();
            int firstIndex = pointIndex;
            /* Iterate through the points of each shape */
            for (unsigned j = 0; j < pList.size(); j++) {
                /* Save the point */
                in.pointlist[pointIndex * 2] = pList[j].getX();
                in.pointlist[pointIndex * 2 + 1] = pList[j].getY();
                if (j != 0) {
                    /* Save the segment */
                    in.segmentlist[segmentIndex * 2] = pointIndex - 1;
                    in.segmentlist[segmentIndex * 2 + 1] = pointIndex;
                    segmentIndex++;
                }
                pointIndex++;
            }
            /* Save the last segment */
            in.segmentlist[segmentIndex * 2] = pointIndex - 1;
            in.segmentlist[segmentIndex * 2 + 1] = firstIndex;
            segmentIndex++;

            /* Save a hole point for every shapes */
            Point p = obstacles[i].getInsidePoint();
            in.holelist[holeIndex * 2] = p.getX();
            in.holelist[holeIndex * 2 + 1] = p.getY();
            holeIndex++;
        }

        /* Set the output structure */
        out.pointlist = (REAL *) 0;
        out.pointattributelist = (REAL *) 0;
        out.pointmarkerlist = (int *) 0;
        out.trianglelist = (int *) 0;
        out.triangleattributelist = (REAL *) 0;
        out.neighborlist = (int *) 0;
        out.segmentlist = (int *) 0;
        out.segmentmarkerlist = (int *) 0;
        out.edgelist = (int *) 0;
        out.edgemarkerlist = (int *) 0;

        /*	Do the triangualtion.
         *  p switch: .poly like, points, segments and holes
         *  z switch: zero indexing
         *  n switch: neighbor output
         *  Q switch: quiet mode, replace with V or VV for verbose */
        char params[] = "pznQ";
        triangulate(params, &in, &out, (struct triangulateio *) 0);

        /* Store triangles with their points */
        for (int i = 0; i < out.numberoftriangles; i++) {
            pointList pointVector;
            for (int j = 0; j < 3; j++) {
                double x = out.pointlist[out.trianglelist[i * 3 + j] * 2];
                double y = out.pointlist[out.trianglelist[i * 3 + j] * 2 + 1];
                pointVector.push_back(Point(x, y));
            }
            triangles.push_back(Triangle(pointVector));
        }

        adjN = std::vector<std::vector<char> >(out.numberoftriangles, std::vector<char>(out.numberoftriangles, -1));

        /* Strore middle points ins neghbour cells */
        /* Go through the neighbor list */
        for (int i = 0; i < out.numberoftriangles; i++) {
            /* Go through every edge */
            for (int j = 0; j < 3; j++) {
                int neighborCellIndex = out.neighborlist[i * 3 + j];
                if (neighborCellIndex > i) {
                    Point a, b;
                    switch (j) {
                        case 0:
                            a = triangles[i].getPoint(2);
                            b = triangles[i].getPoint(1);
                            break;
                        case 1:
                            a = triangles[i].getPoint(2);
                            b = triangles[i].getPoint(0);
                            break;
                        case 2:
                            a = triangles[i].getPoint(1);
                            b = triangles[i].getPoint(0);
                            break;
                        default:
                            //throw std::exception("This should be never reached!");
                            break;
                    }
                    double x = (a.getX() + b.getX()) / 2;
                    double y = (a.getY() + b.getY()) / 2;
                    commonPoints.push_back(Point(x, y));
                    adjN[i][neighborCellIndex] = commonPoints.size() - 1;
                    adjN[neighborCellIndex][i] = commonPoints.size() - 1;
                }
            }
        }

        startIndex = commonPoints.size();
        commonPoints.push_back(robot.getStart().position);
        goalIndex = commonPoints.size();
        commonPoints.push_back(robot.getGoal().position);


        /* Free memory */
        free(out.pointlist);
        free(out.pointattributelist);
        free(out.pointmarkerlist);
        free(out.trianglelist);
        free(out.triangleattributelist);
        free(out.neighborlist);
        free(out.segmentlist);
        free(out.segmentmarkerlist);
        free(out.edgelist);
        free(out.edgemarkerlist);

        /* Free memory */
        delete[] in.pointlist;
        delete[] in.segmentlist;
        delete[] in.holelist;

    } catch (std::exception& e) {
        /* Free memory */
        delete[] in.pointlist;
        delete[] in.segmentlist;
        delete[] in.holelist;

        throw e;
    }
}

void Triangulator::createAdj() {
    commonPointDistances = adjacencyMatrix(commonPoints.size(), std::vector<double>(commonPoints.size()));
    for (unsigned i = 0; i < adjN.size(); i++) {
        std::vector<int> connectionIndexes;
        for (unsigned j = 0; j < adjN[i].size(); j++) {
            if (adjN[i][j] != -1) {
                connectionIndexes.push_back(adjN[i][j]);
            }
        }

        if (connectionIndexes.size() > 1) {
            commonPointDistances[connectionIndexes[0]][connectionIndexes[1]] = Point::distance(commonPoints[connectionIndexes[0]], commonPoints[connectionIndexes[1]]);
            commonPointDistances[connectionIndexes[1]][connectionIndexes[0]] = commonPointDistances[connectionIndexes[0]][connectionIndexes[1]];
            if (connectionIndexes.size() == 3) {
                commonPointDistances[connectionIndexes[0]][connectionIndexes[2]] = Point::distance(commonPoints[connectionIndexes[0]], commonPoints[connectionIndexes[2]]);
                commonPointDistances[connectionIndexes[1]][connectionIndexes[2]] = Point::distance(commonPoints[connectionIndexes[1]], commonPoints[connectionIndexes[2]]);
                commonPointDistances[connectionIndexes[2]][connectionIndexes[0]] = commonPointDistances[connectionIndexes[0]][connectionIndexes[2]];
                commonPointDistances[connectionIndexes[2]][connectionIndexes[1]] = commonPointDistances[connectionIndexes[1]][connectionIndexes[2]];
            }
        }
    }
}

void Triangulator::insertStartGoal() {
    /* Search the containing triangle indexes */
    int startTriangleIndex = -1;
    int goalTriangleIndex = -1;
    for (unsigned i = 0; i < triangles.size(); i++) {
        if (triangles[i].isInside(commonPoints[startIndex])) {
            startTriangleIndex = i;
        }
        if (triangles[i].isInside(commonPoints[goalIndex])) {
            goalTriangleIndex = i;
        }
    }

    /* Insert start and goal point edges */
    for (unsigned i = 0; i < adjN.size(); i++) {
        int commonPointIndex = adjN[startTriangleIndex][i];
        if (commonPointIndex != -1) {
            commonPointDistances[startIndex][commonPointIndex] = Point::distance(commonPoints[startIndex], commonPoints[commonPointIndex]);
            commonPointDistances[commonPointIndex][startIndex] = commonPointDistances[startIndex][commonPointIndex];
        }

        commonPointIndex = adjN[goalTriangleIndex][i];
        if (commonPointIndex != -1) {
            commonPointDistances[goalIndex][commonPointIndex] = Point::distance(commonPoints[goalIndex], commonPoints[commonPointIndex]);
            commonPointDistances[commonPointIndex][goalIndex] = commonPointDistances[goalIndex][commonPointIndex];
        }
    }
}

void Triangulator::removeCloseEdges() {
    /* Remove edges if its too close to obstacles */
    shapeList obstacles = env.getObstacles();
    obstacles.push_back(env.getBoundary());
    double robotMaxWidth = robot.getMaxWidth();
    for (unsigned i = 0; i < commonPointDistances.size() - 1; i++) {
        for (unsigned j = i + 1; j < commonPointDistances[i].size(); j++) {
            if (commonPointDistances[i][j] != 0) {
                Segment seg(commonPoints[i], commonPoints[j]);
                if (robotMaxWidth >= env.getMinDistance(seg)) {
                    commonPointDistances[i][j] = 0;
                    commonPointDistances[j][i] = 0;
                }
            }
        }
    }
    obstacles.pop_back();

}
