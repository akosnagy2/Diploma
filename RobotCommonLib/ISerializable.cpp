#include "ISerializable.h"

#include "json/writer.h"
#include "json/reader.h"

std::ostream& operator<<(std::ostream& os, ISerializable& obj)
{
	return os << obj.getJsonValue();
}

std::istream& operator>>(std::istream& is, ISerializable& obj)
{
	Json::Value val;
	is >> val;
	obj.setFromJson(val);
	return is;
}