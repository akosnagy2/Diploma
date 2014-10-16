/* 
 * File:   PlannerThread.h
 * Author: Gábor
 *
 * Created on 2014. március 22., 9:58
 */

#ifndef PLANNERTHREAD_H
#define	PLANNERTHREAD_H

#include <QThread>
#include <QPolygonF>
#include <string>
#include <QLineF>
#include <QJsonObject>
#include <QWaitCondition>
#include <QMutex>

#include <ctime>

#include "Rectangle.h"
#include "Environment.h"
#include "Triangulator.h"
#include "ARMBuilder.h"
#include "CCS.h"

class PlannerThread : public QThread {
    Q_OBJECT
signals:
    void writeLog(QString log);
    void drawBoundary(double w, double h);
    void drawObstacle(QPolygonF obst);
    void drawTriangle(QPolygonF tri);
    void drawPrePathPoint(QPointF p);
    void drawPrePathSegment(QLineF line);
    void drawPathPoint(QPointF p);
    void drawPathSegment(QLineF line);
    void drawPoint(QPointF p);
    void drawPathArc(QPointF center, qreal r, qreal theta, qreal dTheta);
    void drawPathStraight(QLineF line, bool dir);
    void drawRobotConfig(QPointF pos, qreal ori);

private:
    void run();
    QString lastLog;
    QString file;
    QWaitCondition waitCond;
    QMutex sync;
    clock_t calcTime;
    clock_t startTime;
    bool waitToContinue;
    bool terminated;

    void _writeLog(std::string log);
    void _drawBoundary(Rectangle& rect);
    void _drawObstacle(Shape& shape);
    void _drawTriangulation(triangleList& triangles);
    void _drawPrePath(pointList& points, adjacencyMatrix& connections);
    void _drawPath(configurationList& config);
    void _drawARM(ARM& arm);
    void _drawOG(OccupancyGrid& og);
    void _drawCCS(CCS& ccs, bool last);
    void _drawRobotConfigs(Configuration& start, Configuration& end);
    
    void waitToRun();

public:
    void setFile(QString file);
    void setWaitToContinue(bool wait);
    void endThread();

public slots:
    void continueExec();
};

#endif	/* PLANNERTHREAD_H */

