#include "Robot.h"
#include "misc.h"
#include <cmath>

Robot::Robot() {
}

void Robot::setStart(Configuration s) {
    start = s;
}

void Robot::setGoal(Configuration g) {
    goal = g;
}

void Robot::setBody(Shape b) {
    body = b;
}

void Robot::setAxleDistance(double L) {
    axleDistance = L;
}

void Robot::setWheelBase(double wb) {
    wheelBase = wb;
}

void Robot::setWheelDiameter(double wd) {
    wheelDiameter = wd;
}

void Robot::setWheelWidth(double ww) {
    wheelWidth = ww;
}

/* Works with degree */
void Robot::setPhiMax(double phi) {
    phiMax = wrapAngle(phi);
}

Configuration& Robot::getStart() {
    return start;
}

Configuration& Robot::getGoal() {
    return goal;
}

Shape& Robot::getBody() {
    return body;
}

double Robot::getAxleDistance() {
    return axleDistance;
}

double Robot::getWheelBase() {
    return wheelBase;
}

double Robot::getWheelDiameter() {
    return wheelDiameter;
}

double Robot::getWheelWidth() {
    return wheelWidth;
}

double Robot::getPhiMax() {
    return phiMax;
}

double Robot::getRMin() {
    return axleDistance / tan(phiMax);
}

double Robot::getMaxWidth() {
    int maxIndex = 0;

    for (unsigned i = 0; i < body.getPoints().size() - 1; i++)
        for (unsigned j = i + 1; j < body.getPoints().size(); j++)
            if (body.getPoints()[i].y < body.getPoints()[j].y)
                maxIndex = j;

    return body.getPoints()[maxIndex].y;
}