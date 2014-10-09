#ifndef _PACKEDMESSAGE__
#define _PACKEDMESSAGE__

#include <vector>
#include <boost/asio.hpp>
#include <string>
#include "json/json.h"

using boost::asio::ip::tcp;
using namespace std;

class PackedMessage
{
public:
	int				src; 
	int				dst; 
	int				subj;
	vector<double>	values;
	string			type;
	long long		timestamp;
	int				binarySize;
	char*			binaryData;

	PackedMessage(int _src, int _dst, int _subj, vector<double> _values, string _type, long long _timestamp, int _binarySize, char* _binaryData) 
		: src(_src), dst(_dst), subj(_subj), values(_values) ,type(_type), timestamp(_timestamp), binarySize(_binarySize), binaryData(_binaryData) {};
	PackedMessage()
	{
		src = 0;
		dst = 0;
		subj = 0;
		type = "";
		timestamp = 0;
		binarySize = 0;
		binaryData = NULL;

	};

	void send(iostream &stream);
	int receive(iostream &stream);
	friend std::ostream & operator<<(std::ostream &os, const PackedMessage &gp);
};

#endif /* _PACKEDMESSAGE__ */		
