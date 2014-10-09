#include "Pos2dMessage.h"

PackedMessage Pos2dMessage::get()
{
	PackedMessage msg;

	msg.src = this->src;
	msg.subj = this->subj;
	msg.values.push_back((double)this->pos.x);
	msg.values.push_back((double)this->pos.y);
	msg.values.push_back((double)this->pos.phi);
	//TODO: timestap-et kezelni
	msg.type = POS2DMESSAGE_TYPE_CODE;

	return msg;
}

int Pos2dMessage::set(PackedMessage &msg)
{
	if (msg.type != POS2DMESSAGE_TYPE_CODE)
		return 0;

	if (msg.values.size() != 3)
		return 0;

	this->src = msg.src;
	this->subj = msg.subj;
	this->pos.x = (float)msg.values[0];
	this->pos.y = (float)msg.values[1];
	this->pos.phi = (float)msg.values[2];

	return 1;
}

void Pos2dMessage::send(iostream &stream)
{
	PackedMessage msg = this->get();
	msg.send(stream);
}

int Pos2dMessage::receive(iostream &stream)
{
	PackedMessage msg;

	if (!msg.receive(stream))
		return 0;

	return this->set(msg);
}