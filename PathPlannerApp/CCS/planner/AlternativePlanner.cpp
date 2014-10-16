/* 
 * File:   AlternativePlanner.cpp
 * Author: Gábor
 * 
 * Created on 2014. április 26., 14:13
 */

#include <math.h>
#include <stdexcept>

#include "AlternativePlanner.h"
#include "misc.h"

AlternativePlanner::AlternativePlanner(Configuration& start, Configuration& goal, Scene& scene)
: scene(scene)
, start(start)
, goal(goal) {
    calculateARMs();
    for(std::deque<CCS>::iterator it = solutions.begin(); it != solutions.end();) {
        if (!scene.isAdmissible(it->getFirst())
                || !scene.isAdmissible(it->getMiddle())
                || !scene.isAdmissible(it->getLast())) {
            it = solutions.erase(it);
        } else {
            it++;
        }
    }
}

void AlternativePlanner::calculateARMs() {
    Configuration qI = start;
    qI.inverseTransform(goal);
    double r;
    double thetaIhAbs;

    if (fabs(qI.position.y) / (3 + cos(qI.orientation)) >= scene.getRobotRMin()) {
        r = -qI.position.y / (3 + cos(qI.orientation));
        thetaIhAbs = M_PI;

        calculateConfigs(thetaIhAbs, r, 0);
    } else if (fabs(qI.position.y) > 0.01) {
        r = -sgn(qI.position.y) * scene.getRobotRMin();
        thetaIhAbs = acos(-fabs(qI.position.y) / 2 / scene.getRobotRMin() + (cos(qI.orientation) + 1) / 2);

        calculateConfigs(thetaIhAbs, r, 0);
        calculateConfigs(thetaIhAbs, r, 1);
    } else {
        r = scene.getRobotRMin();
        thetaIhAbs = acos(-fabs(qI.position.y) / 2 / scene.getRobotRMin() + (cos(qI.orientation) + 1) / 2);

        calculateConfigs(thetaIhAbs, r, 0);
        calculateConfigs(thetaIhAbs, r, 1);
        
        r = -scene.getRobotRMin();
        thetaIhAbs = acos(-fabs(qI.position.y) / 2 / scene.getRobotRMin() + (cos(qI.orientation) + 1) / 2);

        calculateConfigs(thetaIhAbs, r, 0);
        calculateConfigs(thetaIhAbs, r, 1);
    }
}

void AlternativePlanner::pushCCS(Configuration& cI, Configuration& cG, bool dir) {
    Arc a11(start, cI.position);
    Arc a12(start, cI.position, false);
    Arc a21(cI, cG.position);
    Arc a22(cI, cG.position, false);
    Segment s(cG, goal, dir);

    solutions.push_back(CCS(a11, a21, s));
    solutions.push_back(CCS(a11, a22, s));
    solutions.push_back(CCS(a12, a21, s));
    solutions.push_back(CCS(a12, a22, s));
}

void AlternativePlanner::calculateConfigs(double theta, double r, int k) {
    Configuration qI = start;
    qI.inverseTransform(goal);

    double thIh = ((k % 2) ? -1 : 1) * theta;
    double xIh = qI.position.x - r * (sin(qI.orientation) - sin(thIh));
    double yIh = qI.position.y + r * (cos(qI.orientation) - cos(thIh));
    Configuration qIh(xIh, yIh, thIh);

    double xGh = xIh + r * sin(thIh);
    Configuration qGh(xGh, 0, 0);

    qIh.transform(goal);
    qGh.transform(goal);

    pushCCS(qIh, qGh, xGh < 0);
}

CCS AlternativePlanner::getShortest() {
    if (!hasSolution()) {
        throw std::logic_error("Has no solution!");
    }
    
    double revPen = scene.getReversePenalty();
    double minLen = INFINITY;
    unsigned minIndex = 0;
    
    for (unsigned i = 1; i < solutions.size() - 1; i++) {
        double currLen = solutions[i].getLength(revPen);
        if (minLen > currLen) {
            minIndex = i;
            minLen = currLen;
        }
    }
    
    return solutions[minIndex];
}

bool AlternativePlanner::hasSolution() {
    return solutions.size() > 0;
}
