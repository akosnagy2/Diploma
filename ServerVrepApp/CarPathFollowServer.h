#pragma once

#include <deque>
#include "simpleInConnection.h"
#include "boost\asio\ip\tcp.hpp"
#include "PathMessage.h"
#include <iostream>
#include <fstream>
#include "Common.h"

using boost::asio::ip::tcp;
using namespace std;

void CarParsePathFollowPars(deque<float> &parsIn, CarPathFollowParamsTypedef &parsOut);
void CarForwardPathFollowPars(tcp::iostream &client, CarPathFollowParamsTypedef &pars);
int CarPathFollowServer(deque<float> &pars, CSimpleInConnection &connection, tcp::iostream &client, ofstream &logFile);
void CarSimulationLoop(CarPathFollowParamsTypedef &followPars, CSimpleInConnection &connection, tcp::iostream &client, ofstream &logFile);