/* 
 * File:   PlannerThread.cpp
 * Author: Gábor
 * 
 * Created on 2014. március 22., 9:58
 */

#include "PlannerThread.h"
#include "Environment.h"
#include "OccupancyGrid.h"
#include "misc.h"
#include "Preplanner.h"
#include "ARMBuilder.h"
#include "LocalPlanner.h"
#include "AlternativePlanner.h"

#include <QPointF>
#include <QJsonArray>
#include <QJsonDocument>
#include <QFileDialog>
#include <QFile>
#include <QDebug>

#include <sstream>
#include <math.h>
#include <cmath>
#include <iostream>

void PlannerThread::run() {
    terminated = false;

    calcTime = 0;
    startTime = clock();
    if (file.isEmpty()) {
        writeLog("No file selected!");
        return;
    }

    QFile input(file);
    if (!input.open(QIODevice::ReadOnly | QIODevice::Text)) {
        writeLog("File open error!");
        return;
    }

    QByteArray jsonStream;
    jsonStream = input.readAll();
    QJsonDocument jsonEnv = QJsonDocument::fromJson(jsonStream);
    if (!jsonEnv.isObject()) {
        writeLog("Invalid JSON!");
        return;
    }

    QJsonObject envObj = jsonEnv.object();

    QJsonValue boundaryVal = envObj["boundary"];
    if (boundaryVal.isUndefined() || boundaryVal.isNull()) {
        _writeLog("No boundary section!");
        return;
    }
    QJsonObject boundaryObj = boundaryVal.toObject();
    Environment env(boundaryObj["width"].toDouble(), boundaryObj["height"].toDouble());
    _drawBoundary(env.getBoundary());

    QJsonValue obstVal = envObj["obstacles"];
    if (obstVal.isUndefined() || obstVal.isNull()) {
        _writeLog("No obstacle section!");
        return;
    }
    QJsonArray obstArray = obstVal.toArray();
    for (int i = 0; i < obstArray.size(); i++) {
        pointList points;
        QJsonArray obstPoints = obstArray[i].toObject()["points"].toArray();
        for (int j = 0; j < obstPoints.size(); j++) {
            double x = obstPoints[j].toObject()["x"].toDouble();
            double y = obstPoints[j].toObject()["y"].toDouble();
            points.push_back(Point(x, y));
        }
        Shape obst(points);
        env.addObstacle(obst);
        _drawObstacle(obst);
    }

    QJsonValue robotVal = envObj["robot"];
    if (robotVal.isUndefined() || robotVal.isNull()) {
        _writeLog("No robot section!");
        return;
    }
    Robot robot;
    QJsonObject robotObj = robotVal.toObject();

    QJsonObject jsonConfig = robotObj["start"].toObject();
    robot.setStart(Configuration(jsonConfig["x"].toDouble(), jsonConfig["y"].toDouble(), d2r(jsonConfig["phi"].toDouble())));
    jsonConfig = robotObj["goal"].toObject();
    robot.setGoal(Configuration(jsonConfig["x"].toDouble(), jsonConfig["y"].toDouble(), d2r(jsonConfig["phi"].toDouble())));

    pointList bodyPoints;
    QJsonArray robotBody = robotObj["body"].toArray();
    for (int j = 0; j < robotBody.size(); j++) {
        double x = robotBody[j].toObject()["x"].toDouble();
        double y = robotBody[j].toObject()["y"].toDouble();
        bodyPoints.push_back(Point(x, y));
    }
    robot.setBody(Shape(bodyPoints));

    robot.setAxleDistance(robotObj["L"].toDouble());
    robot.setWheelBase(robotObj["WB"].toDouble());
    robot.setWheelDiameter(robotObj["WD"].toDouble());
    robot.setWheelWidth(robotObj["WW"].toDouble());
    robot.setPhiMax(robotObj["phiMax"].toDouble());

    _drawRobotConfigs(robot.getStart(), robot.getGoal());

    Triangulator triang(env, robot);
    _writeLog("Triangulation done.");
    _drawTriangulation(triang.getTriangles());

    waitToRun();
    if (terminated) {
        return;
    }

    _drawPrePath(triang.getCommonPoints(), triang.getCommonPointConnection());

    waitToRun();
    if (terminated) {
        return;
    }

    Preplanner prep(robot, triang.getCommonPoints(), triang.getCommonPointConnection());
    if (!prep.isPathExist()) {
        _writeLog("Path not found.");
        return;
    }
    _writeLog("Path found.");
    configurationList config = prep.getPath();
    _drawPath(config);
    _writeLog("Configuration list created.");

    waitToRun();
    if (terminated) {
        return;
    }

    QJsonValue algoVal = envObj["algo"];
    if (algoVal.isUndefined() || algoVal.isNull()) {
        _writeLog("No algo section!");
        return;
    }
    QJsonObject algoObj = algoVal.toObject();

    Scene sc(env, robot);
    sc.setReversePenalty(algoObj["reversePenaltyFactor"].toDouble());
    sc.setDx(algoObj["dx"].toDouble());
    sc.setDy(algoObj["dy"].toDouble());

    OccupancyGrid oG(sc);
    _writeLog("Occupancy grid created.");

    /* Initialize path planner */
    unsigned startIndex = 0;
    unsigned endIndex = config.size() - 1;
    unsigned insertCount = 0;
    ARM arm = ARMBuilder(config[startIndex], oG, sc).getARM();

    std::ostringstream stringStream;
    stringStream << "ARM created from configuration #" << startIndex << ".";
    _writeLog(stringStream.str());
    stringStream.str("");

    /* Loop through possible configuration pairs until solution found */
    while (startIndex != config.size() - 1 && endIndex > startIndex && insertCount < 10) {
        /* Get the distance of the next configuration */
        double nextDist = 0;
        if (endIndex != config.size() - 1) {
            nextDist = Point::distance(config[endIndex].position, config[endIndex + 1].position);
        }

        /* Search path between the two configurations */
        LocalPlanner lp(arm, config[endIndex], nextDist, sc);
        stringStream << "Local Planner between configurations #" << startIndex << " and #" << endIndex << ".";
        _writeLog(stringStream.str());
        stringStream.str("");

        /* Last check has a solution */
        if (lp.hasSolution()) {
            _writeLog("Has solution");
            CCS ccs = lp.getShortest();
            stringStream << ccs << std::endl;
            _writeLog(stringStream.str());
            stringStream.str("");

            /* Insert new tangential configuration */
            startIndex = endIndex;
            if (startIndex != config.size() - 1) {
                configurationList::iterator it = config.begin();
                it += ++startIndex;
                config.insert(it, ccs.getMiddle().getEndConfig());

                /* Calculate ARM from the new configuration */
                arm = ARMBuilder(config[startIndex], oG, sc).getARM();
                stringStream << "ARM created from configuration #" << startIndex << ".";
                _writeLog(stringStream.str());
                stringStream.str("");
            }
            endIndex = config.size() - 1;

            /* Draw the found path */
            _drawCCS(ccs, (startIndex == config.size() - 1));
            waitToRun();
            if (terminated) {
                return;
            }

        } else {
            if (endIndex - startIndex == 1) {
                endIndex = startIndex;
            } else {
                /* Select new configuration from predefined path */
                endIndex = (endIndex - startIndex - 1) / 2 + 1 + startIndex;
            }
        }
    }

    if (startIndex != config.size() - 1) {
        _writeLog("Fail!");
    } else {
        _writeLog("Success!");
    }

    calcTime += clock() - startTime;
    stringStream << "Runtime: " << calcTime << "ms";
    _writeLog(stringStream.str());
    stringStream.str("");
}

void PlannerThread::_drawBoundary(Rectangle & rect) {
    double w = rect.getPoint(2).x - rect.getPoint(0).x;
    double h = rect.getPoint(2).y - rect.getPoint(0).y;

    emit drawBoundary(w, h);
}

void PlannerThread::_drawObstacle(Shape & shape) {
    QPolygonF poly;
    pointList pList = shape.getPoints();

    for (unsigned i = 0; i < pList.size(); i++) {
        poly << QPointF(pList[i].x, pList[i].y);
    }

    emit drawObstacle(poly);
}

void PlannerThread::_writeLog(std::string log) {
    emit writeLog(log.c_str());
}

void PlannerThread::_drawTriangulation(triangleList & triangles) {
    for (unsigned i = 0; i < triangles.size(); i++) {
        QPolygonF poly;
        pointList pList = triangles[i].getPoints();
        for (unsigned j = 0; j < pList.size(); j++) {
            poly << QPointF(pList[j].x, pList[j].y);
        }
        emit drawTriangle(poly);
    }
}

void PlannerThread::_drawPrePath(pointList& points, adjacencyMatrix & connections) {
    /* Draw the points first */
    for (unsigned i = 0; i < points.size(); i++)
        emit drawPrePathPoint(QPointF(points[i].x, points[i].y));

    /* Draw the segments */
    for (unsigned i = 0; i < connections.size() - 1; i++)
        for (unsigned j = i + 1; j < connections[i].size(); j++)
            if (connections[i][j] > 0.0)
                emit drawPrePathSegment(QLineF(QPointF(points[i].x, points[i].y), QPointF(points[j].x, points[j].y)));
}

void PlannerThread::_drawPath(configurationList & config) {
    for (unsigned i = 0; i < config.size(); i++) {
        if (i == 0) {
            QPointF curr(config[i].position.x, config[i].position.y);
            emit drawPathPoint(curr);
            //emit drawRobotConfig(curr, config[i].orientation);
        } else {
            QPointF prev(config[i - 1].position.x, config[i - 1].position.y);
            QPointF curr(config[i].position.x, config[i].position.y);

            emit drawPathPoint(curr);
            emit drawPathSegment(QLineF(prev, curr));

            if (i == config.size() - 1) {
                //emit drawRobotConfig(curr, config[i].orientation);
            }
        }
    }
}

void PlannerThread::setFile(QString file) {
    this->file = file;
}

void PlannerThread::_drawARM(ARM & arm) {
    for (unsigned i = 0; i < arm.size(); i++) {
        emit drawPoint(QPointF(arm[i].getEndConfig().position.x, arm[i].getEndConfig().position.y));
    }
}

void PlannerThread::_drawOG(OccupancyGrid & og) {
    for (unsigned i = 0; i < og.getXSize(); i++)
        for (unsigned j = 0; j < og.getYSize(); j++)
            if (!og.isOccupied(i, j))
                emit drawPoint(QPointF(og.getPoint(i, j).x, og.getPoint(i, j).y));
}

void PlannerThread::_drawCCS(CCS& ccs, bool last) {
    Arc first = ccs.getFirst();
    if (fabs(first.getRadius()) > 500) {
        Point a = first.getStartConfig().position;
        Point b = first.getEndConfig().position;
        emit drawPathStraight(QLineF(a.x, a.y, b.x, b.y), first.getDirection());
    } else {
        Point c = first.getCenter();
        double startAngle = first.getStartConfig().orientation;
        double spanAngle = first.getDTheta();
        emit drawPathArc(QPointF(c.x, c.y), first.getRadius(), startAngle, spanAngle);
    }

    Arc second = ccs.getMiddle();
    Point c = second.getCenter();
    double startAngle = second.getStartConfig().orientation;
    double spanAngle = second.getDTheta();
    emit drawPathArc(QPointF(c.x, c.y), second.getRadius(), startAngle, spanAngle);

    if (last) {
        Segment seg = ccs.getLast();
        emit drawPathStraight(QLineF(seg.getA().x, seg.getA().y, seg.getB().x, seg.getB().y), ccs.getLast().getDirection());
    }
}

void PlannerThread::continueExec() {
    waitCond.wakeAll();
}

void PlannerThread::endThread() {
    terminated = true;
    waitCond.wakeAll();
}

void PlannerThread::setWaitToContinue(bool wait) {
    waitToContinue = wait;
}

void PlannerThread::waitToRun() {
    if (waitToContinue) {
        calcTime += clock() - startTime;
        sync.lock();
        waitCond.wait(&sync);
        sync.unlock();
        startTime = clock();
    }
}

void PlannerThread::_drawRobotConfigs(Configuration& start, Configuration& end) {
    QPointF p(start.position.x, start.position.y);
    emit drawRobotConfig(p, start.orientation);
    p = QPointF(end.position.x, end.position.y);
    emit drawRobotConfig(p, end.orientation);
}
