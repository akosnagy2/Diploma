/* 
 * File:   Scene.cpp
 * Author: Gábor
 * 
 * Created on 2014. április 10., 12:51
 */

#include "Scene.h"

Scene::Scene(Environment& env, Robot& robot)
: env(env)
, robot(robot) {
    dx = 1;
    dy = 1;
    reversePenalty = 1;
}

/**
 * Check if the arc is admissible for the robot.
 * 
 * @param arc Arc of the robot's main point
 * @return True if collision free
 */
bool Scene::isAdmissible(Path& path) {
    bool admissible = true;
    Shape robotBody = robot.getBody();
    pointList robotPoints = robotBody.getPoints();
    for (unsigned i = 0; i < robotPoints.size() && admissible; i++) {
        Path* movedPath = path.copy();
        admissible &= env.isCollisionFree(movedPath->translateToRobotCornerArc(robotPoints[i]));
        delete movedPath;
    }

    if (admissible) {
        robotBody.transform(path.getStartConfig());
        pointList obstaclePoints = env.getAllObstaclePoints();
        for (unsigned i = 0; i < obstaclePoints.size() && admissible; i++) {
            Path* movedPath = path.copy();
            admissible &= !robotBody.isIntersect(movedPath->translateToObstacleCornerArc(obstaclePoints[i]));
            delete movedPath;
        }
    }
    return admissible;
}

double Scene::getRobotRMin() {
    return robot.getRMin();
}

double Scene::getReversePenalty() {
    return reversePenalty;
}

double Scene::getDx() {
    return dx;
}

double Scene::getDy() {
    return dy;
}

void Scene::setDx(double dx) {
    this->dx = dx;
}

void Scene::setDy(double dy) {
    this->dy = dy;
}

void Scene::setReversePenalty(double revPen) {
    this->reversePenalty = revPen;
}

Environment& Scene::getEnvironment() {
    return env;
}
