#pragma once

#include "simpleInConnection.h"
#include "boost\asio\ip\tcp.hpp"
#include <deque>
#include "CarPathFollowServer.h"
#include <iostream>
#include <fstream>
#include "Common.h"

using boost::asio::ip::tcp;
using namespace std;

void CarParsePathPlannerPars(deque<float> &parsIn, CarPathPlannerParamsTypedef &parsOut);
void CarForwardPathPlannerPars(tcp::iostream &client, PathPlannerParamsTypedef &pars);
int CarPathPlannerServer(deque<float> &pars, CSimpleInConnection &connection, tcp::iostream &client, ofstream &logFile);