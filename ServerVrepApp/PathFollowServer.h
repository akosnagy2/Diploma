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

void ParsePathFollowPars(deque<float> &parsIn, PathFollowParamsTypedef &parsOut);
void ForwardPathFollowPars(tcp::iostream &client, PathFollowParamsTypedef &pars);
int PathFollowServer(deque<float> &pars, CSimpleInConnection &connection, tcp::iostream &client, ofstream &logFile);