#ifndef _CTRLMESSAGE__
#define _CTRLMESSAGE__

#include "PackedMessage.h"

#define CTRLMESSAGE_TYPE_CODE "CTRL"

class CtrlMessage
{
public:
	vector<double> ctrl_sig;
	int src, subj;

	CtrlMessage() 
	{
		src = 0;
		subj = 0;
	}

	CtrlMessage(int _src, int _subj) : src(_src), subj(_subj) {};

	PackedMessage get();
	int set(PackedMessage &msg);

	void send(iostream &stream);
	int receive(iostream &stream);
};

#endif /* _CTRLMESSAGE__ */	