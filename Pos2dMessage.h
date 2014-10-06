#ifndef _POS2DMESSAGE__
#define _POS2DMESSAGE__

#include "PackedMessage.h"
#include "PathPlannerApp\PathPlanner\Config.h"
#define POS2DMESSAGE_TYPE_CODE "POS2D"

using namespace PathPlanner;

class Pos2dMessage
{
public:
	Config pos;
	int src, subj;

	Pos2dMessage() 
	{
		pos.p.x = 0;
		pos.p.y = 0;
		pos.phi = 0;
		src = 0;
		subj = 0;
	}

	Pos2dMessage(Config _pos, int _src, int _subj) : pos(_pos), src(_src), subj(_subj) {};

	Pos2dMessage(float _x, float _y, float _phi, int _src, int _subj)
	{
		this->pos.p.x = _x;
		this->pos.p.y = _y;
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
