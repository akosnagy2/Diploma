#pragma once

#include "simpleInConnection.h"
#include "boost\asio\ip\tcp.hpp"
#include <deque>
#include <iostream>
#include <fstream>
#include "Common.h"

using boost::asio::ip::tcp;
using namespace std;

void CarParsePathFollowPars(deque<float> &parsIn, CarPathFollowParamsTypedef &parsOut);
void CarParsePathPlannerPars(deque<float> &parsIn, CarPathPlannerParamsTypedef &parsOut);
void CarForwardPathPlannerPars(tcp::iostream &client, CarPathPlannerParamsTypedef &pars);
int CarPathPlannerServer(deque<float> &pars, CSimpleInConnection &connection, tcp::iostream &client, ofstream &logFile);