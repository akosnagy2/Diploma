/* 
 * File:   LocalPlanner.cpp
 * Author: Gábor
 * 
 * Created on 2014. április 8., 15:25
 */

#include <float.h>
#include <math.h>
#include <stdexcept>

#include "LocalPlanner.h"

#include "Line.h"
#include "misc.h"
#include "Vector.h"

LocalPlanner::LocalPlanner(ARM& arm, Configuration& goal, double nextDist, Scene& scene)
: scene(scene)
, arm(arm)
, goal(goal)
, nextDist(nextDist) {
    for (unsigned i = 0; i < arm.size(); i++) {
        unsigned shortestIndex = 0;
        double shortestLength = INFINITY;
        bool hasAdmissibleArc = false;

        arcList middleArcs = getTangentArcs(arm[i].getEndConfig(), goal);
        for (unsigned j = 0; j < middleArcs.size(); j++) {
            if (fabs(middleArcs[j].getRadius()) - scene.getRobotRMin() > -2 * DBL_EPSILON) {
                if (nextDist != 0 || fabs(wrapAngle(middleArcs[j].getEndConfig().orientation - goal.orientation)) < M_PI_2) {
                    if (scene.isAdmissible(middleArcs[j])) {
                        Arc endArc(middleArcs[j].getEndConfig(), goal.position);
                        if (scene.isAdmissible(endArc)) {
                            /* All segments are admissible */
                            if (!hasAdmissibleArc) {
                                hasAdmissibleArc = true;
                                shortestIndex = j;
                            }

                            Point newEnd = goal.translate(goal.position, nextDist);
                            Segment end(newEnd, middleArcs[j].getEndConfig().position);
                            double distCurr = calcuateDistance(arm[i], middleArcs[j], end);
                            
                            if (shortestLength > distCurr) {
                                shortestIndex = j;
                                shortestLength = distCurr;
                            }
                        }
                    }
                }
            }
        }

        if (hasAdmissibleArc) {
            /* Calculate last segment and direction */
            Segment endSeg(middleArcs[shortestIndex].getEndConfig().position, goal.translate(goal.position, nextDist), false);
            Vector tempVector(endSeg);
            double f1 = tempVector.getFi();
            double f2 = middleArcs[shortestIndex].getEndConfig().orientation;
            if (wrapAngle(f1 - f2 + M_PI_2) > 0) {
                endSeg.setDirection(true);
            }
            
            middleSegments.push_back(middleArcs[shortestIndex]);
            firstMiddleSegmentsMap.push_back(i);
            lastSegments.push_back(endSeg);
            fullDistances.push_back(calcuateDistance(middleSegments.size() - 1));
        }
    }
}

arcList LocalPlanner::getTangentArcs(const Configuration& middle, Configuration& goal) {
    arcList middleArcs;

    Line goalLine(goal);
    Line middleLine(middle);
    Point intersection;
    Point projection = goalLine.getProjection(middle.position);
    if (!goalLine.isIntersect(middleLine, intersection) || Point::distance(intersection, middle.position) > 1e4) {
        /* Configurations are parallel */
        double r = Point::distance(middle.position, projection);
        if (r < 0.001) {
            middleArcs.push_back(Arc(middle, middle.position));
        } else {
            middleArcs.push_back(Arc(middle, projection));
            middleArcs.push_back(Arc(middle, projection, false));
        }
    } else {
        if (Point::distance(middle.position, projection) > 0.001) {
            double dist = Point::distance(middle.position, intersection);
            middleArcs.push_back(Arc(middle, goal.translate(intersection, dist)));
            middleArcs.push_back(Arc(middle, goal.translate(intersection, dist), false));
            middleArcs.push_back(Arc(middle, goal.translate(intersection, -dist)));
            middleArcs.push_back(Arc(middle, goal.translate(intersection, -dist), false));
        }
    }
    return middleArcs;
}

CCS LocalPlanner::getShortest() {
    if (!hasSolution()) {
        throw std::logic_error("Has no solution!");
    }

    unsigned minIndex = 0;
    for (unsigned i = 1; i < fullDistances.size() - 1; i++) {
        if (fullDistances[minIndex] > fullDistances[i]) {
            minIndex = i;
        }
    }

    return CCS(arm[firstMiddleSegmentsMap[minIndex]], middleSegments[minIndex]
            , lastSegments[minIndex]);
}

bool LocalPlanner::hasSolution() {
    return (middleSegments.size() > 0);
}

double LocalPlanner::calcuateDistance(unsigned middleIndex) {
    Arc& firstArc = arm[firstMiddleSegmentsMap[middleIndex]];
    Arc& middleArc = middleSegments[middleIndex];
    Segment& lastSegment = lastSegments[middleIndex];

    return calcuateDistance(firstArc, middleArc, lastSegment);
}

double LocalPlanner::calcuateDistance(Arc& first, Arc& middle, Segment& end) {
    double distance = 0;
    double reversePenalty = scene.getReversePenalty();

    if (first.getDirection()) {
        distance += first.getLength();
    } else {
        distance += first.getLength() * reversePenalty;
    }

    if (middle.getDirection()) {
        distance += middle.getLength();
    } else {
        distance += middle.getLength() * reversePenalty;
    }

    if (end.getDirection()) {
        distance += end.getLength();
    } else {
        distance += end.getLength() * reversePenalty;
    }

    return distance;
}
