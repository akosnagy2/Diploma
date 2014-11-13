#pragma once

#include <json/value.h>
#include <iostream>

class ISerializable
{
public:
	friend std::ostream& operator<<(std::ostream& os, ISerializable& obj);
	friend std::istream& operator>>(std::istream& is, ISerializable& obj);

	virtual Json::Value getJsonValue() = 0;
	virtual void setFromJson(Json::Value& value) = 0;
};