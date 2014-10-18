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

		for (Json::Value::iterator it = JSON_msg["Segments"].begin(); it != JSON_msg["Segments"].end(); it++)
		{
			PathSegment seg;
			seg.direction = (*it)["Direction"].asBool();

			for (Json::Value::iterator it2 = (*it)["Path"].begin(); it2 != (*it)["Path"].end(); it2++)
			{
				seg.path.push_back(Config((*it2)["x"].asFloat(), (*it2)["y"].asFloat(), (*it2)["phi"].asFloat()));				
				seg.curvature.push_back((*it2)["curvature"].asFloat());
			}

			this->path.push_back(seg);
		}
			
	}

	return parsingSuccessful;
}

ostream & operator<<(ostream &os, const PathMessage &msg)
{
	Json::Value	JSON_msg;
	Json::Value JSON_path;

	JSON_msg["type"] = PATHMESSAGE_TYPE_CODE;

	for (vector<PathSegment>::const_iterator it = msg.path.begin(); it != msg.path.end(); it++)
	{
		Json::Value	JSON_seg, JSON_seg_path;
		JSON_seg["Direction"] = it->direction;

		for (int i = 0; i < (int)it->path.size(); i++)
		{
			Json::Value	JSON_pos;
			JSON_pos["x"] = it->path[i].p.x;
			JSON_pos["y"] = it->path[i].p.y;
			JSON_pos["phi"] = it->path[i].phi;
			if (it->curvature.size())
				JSON_pos["curvature"] = it->curvature[i];

			JSON_seg_path.append(JSON_pos);
		}

		JSON_seg["Path"] = JSON_seg_path;
		JSON_path.append(JSON_seg);
	}

	JSON_msg["Segments"] = JSON_path;

	Json::FastWriter fastWriter;
	return os << fastWriter.write(JSON_msg);
}