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

#define SAVE_PROFILE true

void setLimits(float _maxV, float _maxA, float _maxAt, float _maxW, float _sampleT, float _robotWheelBase, int _predictLen);
void profile_top(std::vector<Config> &path, std::vector<Config> &resPath);

#endif /* PATH_PROFILE_TOP_H_ */
