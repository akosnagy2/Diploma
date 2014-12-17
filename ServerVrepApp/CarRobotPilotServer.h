#pragma once

#include <deque>
#include "Common.h"

int CarRobotPilotServer(deque<float> &pars, CSimpleInConnection &connection, tcp::iostream &client, ofstream &logFile);
void CarRobotPilotLoop(CarPathFollowParamsTypedef &followPars, CSimpleInConnection &connection, tcp::iostream &client, ofstream &logFile);