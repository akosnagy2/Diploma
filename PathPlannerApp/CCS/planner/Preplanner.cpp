/* 
 * File:   Preplanner.cpp
 * Author: Gábor
 * 
 * Created on 2014. március 25., 10:48
 */

#include "Preplanner.h"

#include <cfloat>
#include <limits>
#include <algorithm>
#include <cmath>
#include "misc.h"

Preplanner::Preplanner(Robot& robot, pointList& points, adjacencyMatrix& connections) {
    /* Search for a shortest path in the roadmap */

    const unsigned N = points.size();

    /*
     * Meaning of done
     * 0 - unvisited cell
     * 1 - ALIVE - A neighbour of this cell was already visited by the algorithm 
     * 2 - DEAD - This cell wasl already visited by the algorithm 
     */
    std::vector<char> done(N, 0);

    /* Cost values of vertices */
    std::vector<double> cost(N, 0.0);

    /* Pointers for path restoration */
    std::vector<char> pointers(N, -1);

    /* The vertex queue (FIFO), cost of vertices */
    std::vector<double> queue(N, std::numeric_limits<double>::infinity());
    unsigned queueCount = 0;

    /* Calculate goal and start point */
    unsigned goalIndex = points.size() - 1;
    //    for (unsigned i = 0; i < points; i++)
    //        if (points[i] == robot.getGoal().position)
    //            goalIndex = i;

    unsigned startIndex = points.size() - 2;
    //    for (unsigned i = 0; i < points; i++)
    //        if (points[i] == robot.getStart().position)
    //            startIndex = i;

    /* Insert the start point first */
    done[goalIndex] = 1;
    cost[goalIndex] = 0;
    queue[goalIndex] = 0;
    queueCount++;

    isPath = false;

    while (queueCount > 0 && !isPath) {
        /* Take the cheapest element from queue */
        unsigned i = std::distance(queue.begin(), std::min_element(queue.begin(), queue.end()));
        /* Set its state to DEAD */
        done[i] = 2;
        queue[i] = std::numeric_limits<double>::infinity();
        queueCount--;

        if (i == startIndex) {
            isPath = true;
        } else {
            /* Visit all neighbours */
            for (unsigned k = 0; k < N; k++) {
                if (connections[i][k] > 0) {
                    /* This is an unvisited neighbour, put it into queue */
                    if (done[k] == 0) {
                        done[k] = 1;
                        cost[k] = cost[i] + connections[i][k];
                        queue[k] = cost[k];
                        queueCount++;
                        pointers[k] = i;
                    }/* This is a visited neighbour, update if needed */
                    else if (done[k] == 1) {
                        if (cost[k] > cost[i] + connections[i][k]) {
                            cost[k] = cost[i] + connections[i][k];
                            queue[k] = cost[k];
                            pointers[k] = 1;
                        }
                    }
                }
            }
        }
    }

    std::vector<unsigned> path;
    if (!isPath)
        return;

    unsigned index = startIndex;
    path.push_back(index);

    while (index != goalIndex) {
        index = pointers[index];
        path.push_back(index);
    }

    for (unsigned i = 0; i < path.size(); i++) {
        if (i == 0 && path[0] == startIndex) {
            configPath.push_back(robot.getStart());
        } else if (i == path.size() - 1) {
            configPath.push_back(robot.getGoal());
        } else {
            /* Skip the point, if the line is straight (prev and next angle is the same) */
            Point& current = points[path[i]];
            Point& next = points[path[i + 1]];
            Point& prev = points[path[i - 1]];

            double nextAngle = atan2(next.y - current.y, next.x - current.x);
            double prevAngle = atan2(current.y - prev.y, current.x - prev.x);

            if (!almostEqualRelative(nextAngle, prevAngle, DBL_EPSILON * 5)) {
                configPath.push_back(Configuration(points[path[i]], nextAngle));
            }
        }
    }

    if (configPath.size() == 2) {
        /* Insert the last one again */
        configPath.push_back(configPath[1]);

        /* Half the distance */
        configPath[1].position = (configPath[0].position + configPath[1].position) / 2;

        /* Set the orientation to the next angle */
        configPath[1].orientation = atan2(configPath[2].position.y - configPath[1].position.y, configPath[2].position.x - configPath[1].position.x);
    }

}

configurationList& Preplanner::getPath() {
    return configPath;
}

bool Preplanner::isPathExist() {
    return isPath;
}
