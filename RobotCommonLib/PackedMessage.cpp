#include "PackedMessage.h"

void PackedMessage::send(iostream &stream)	
{
	stream << (*this);
}

int PackedMessage::receive(iostream &stream)
{
	Json::Value JSON_msg;
	Json::Reader reader;
	std::string recvMsg;

	stream >> recvMsg;

	bool parsingSuccessful = reader.parse(recvMsg, JSON_msg);

	if (parsingSuccessful)
	{
		//TODO: ha nem küldünk át mindent
		src = JSON_msg["src"].asInt();
		dst = JSON_msg["dst"].asInt();
		subj = JSON_msg["subj"].asInt();

		for (Json::Value::iterator it = JSON_msg["values"].begin(); it != JSON_msg["values"].end(); it++)
			values.push_back((*it).asFloat());

		type = JSON_msg["type"].asString();
		timestamp = JSON_msg["timestap"].asInt64();
		binarySize = JSON_msg["binarySize"].asInt();
	}

	return parsingSuccessful;
}

ostream & operator<<(ostream &os, const PackedMessage &msg)
{
	//TODO: ne küldjünk át mindent
	Json::Value	JSON_msg;
	Json::Value JSON_array;
	JSON_msg["src"] = msg.src;
	JSON_msg["dst"] = msg.dst;
	JSON_msg["subj"] = msg.subj;

	for (vector<float>::const_iterator it = msg.values.begin(); it != msg.values.end(); it++)
		JSON_array.append(*it);

	JSON_msg["values"] = JSON_array;
	JSON_msg["type"] = msg.type;
	JSON_msg["timestap"] = msg.timestamp;
	JSON_msg["binarySize"] = msg.binarySize;

	Json::FastWriter fastWriter;
	return os << fastWriter.write(JSON_msg);
}

