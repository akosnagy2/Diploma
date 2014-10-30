/*
 * path_profile_top.h
 *
 *  Created on: Sep 10, 2014
 *      Author: akos
 */

#ifndef PATH_PROFILE_TOP_H_
#define PATH_PROFILE_TOP_H_

#include <vector>
#include "path_profile.h"
#include "PathSegment.h"
#include "CarLikeRobot.h"

#define SAVE_PROFILE false

void setLimits(float _maxV, float _maxA, float _maxAt, float _maxW, float _sampleT, float _robotWheelBase);
void setCarLimits(CarLikeRobot* pR, float _maxV, float _maxA, float _maxAt, float _sampleT);
void profile_top(vector<PathSegment> &path, vector<PathSegment> &resultPath, bool rType);

#endif /* PATH_PROFILE_TOP_H_ */
