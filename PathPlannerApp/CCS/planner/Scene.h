/* 
 * File:   Scene.h
 * Author: Gábor
 *
 * Created on 2014. április 10., 12:51
 */

#ifndef SCENE_H
#define	SCENE_H

#include "Environment.h"
#include "Robot.h"
#include "Arc.h"

class Scene {
public:
    Scene(Environment& env, Robot& robot);
    
    bool isAdmissible(Path& path);
    
    double getRobotRMin();
    double getReversePenalty();
    double getDx();
    double getDy();
    Environment& getEnvironment();
    
    void setReversePenalty(double revPen);
    void setDx(double dx);
    void setDy(double dy);
private:
    Environment& env;
    Robot& robot;
    double reversePenalty;
    double dx;
    double dy;
};

#endif	/* SCENE_H */

