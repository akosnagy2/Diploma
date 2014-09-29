#include "PathMessage.h"


void PathMessage::send(iostream &stream)
{
	stream << (*this);
}

int PathMessage::receive(iostream &stream)
{
	Json::Value JSON_msg;
	Json::Reader reader;
	std::string recvMsg;

	stream >> recvMsg;

	bool parsingSuccessful = reader.parse(recvMsg, JSON_msg);

	if (parsingSuccessful)
	{
		if (JSON_msg["type"].asString() != PATHMESSAGE_TYPE_CODE)
			return 0;

		for (Json::Value::iterator it = JSON_msg["Positions"].begin(); it != JSON_msg["Positions"].end(); it++)
		{
			Position pos;
			pos.x = (*it)["x"].asFloat();
			pos.y = (*it)["y"].asFloat();
			pos.phi = (*it)["phi"].asFloat();

			this->path.push_back(pos);
		}
			
	}

	return parsingSuccessful;
}

ostream & operator<<(ostream &os, const PathMessage &msg)
{
	Json::Value	JSON_msg;
	Json::Value JSON_array;

	JSON_msg["type"] = PATHMESSAGE_TYPE_CODE;

	for (vector<Position>::const_iterator it = msg.path.begin(); it != msg.path.end(); it++)
	{
		Json::Value	JSON_pos;

		JSON_pos["x"] = (*it).x;
		JSON_pos["y"] = (*it).y;
		JSON_pos["phi"] = (*it).phi;
		JSON_array.append(JSON_pos);

	}

	JSON_msg["Positions"] = JSON_array;

	Json::FastWriter fastWriter;
	return os << fastWriter.write(JSON_msg);
}