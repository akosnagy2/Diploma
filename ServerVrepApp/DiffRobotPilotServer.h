#pragma once

#include <deque>
#include "simpleInConnection.h"
#include "boost\asio\ip\tcp.hpp"
#include "Pos2dMessage.h"
#include <iostream>
#include <fstream>
#include "Common.h"

using boost::asio::ip::tcp;
using namespace std;

void RobotLoop(CSimpleInConnection &connection, tcp::iostream &client, ofstream &logFile);
int RobotPilotServer(deque<float> &pars, CSimpleInConnection &connection, tcp::iostream &client, ofstream &logFile);