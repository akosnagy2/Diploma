#include "CtrlMessage.h"

PackedMessage CtrlMessage::get()
{
	PackedMessage msg;

	msg.src = this->src;
	msg.subj = this->subj;
	msg.values = this->ctrl_sig;
	//TODO: kitalálni melyik elem mit jelentsen
	msg.type = CTRLMESSAGE_TYPE_CODE;

	return msg;
}

int CtrlMessage::set(PackedMessage &msg)
{
	if (msg.type != CTRLMESSAGE_TYPE_CODE)
		return 0;

	this->src = msg.src;
	this->subj = msg.subj;
	this->ctrl_sig = msg.values;

	return 1;
}

void CtrlMessage::send(iostream &stream)
{
	PackedMessage msg = this->get();
	msg.send(stream);
}

int CtrlMessage::receive(iostream &stream)
{
	PackedMessage msg;

	if (!msg.receive(stream))
		return 0;

	return this->set(msg);
}