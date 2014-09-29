#ifndef _POS2DMESSAGE__
#define _POS2DMESSAGE__

#include "PackedMessage.h"
#include "Position.h"

#define POS2DMESSAGE_TYPE_CODE "POS2D"

class Pos2dMessage
{
public:
	Position pos;
	int src, subj;

	Pos2dMessage() 
	{
		pos.x = 0;
		pos.y = 0;
		pos.phi = 0;
		src = 0;
		subj = 0;
	}

	Pos2dMessage(Position _pos, int _src, int _subj) : pos(_pos), src(_src), subj(_subj) {};

	Pos2dMessage(float _x, float _y, float _phi, int _src, int _subj)
	{
		this->pos.x = _x;
		this->pos.y = _y;
		this->pos.phi = _phi;
		this->src = _src;
		this->subj = _subj;
	};

	PackedMessage get();
	int set(PackedMessage &msg);

	void send(iostream &stream);
	int receive(iostream &stream);
};

#endif /* _POS2DMESSAGE__ */		
