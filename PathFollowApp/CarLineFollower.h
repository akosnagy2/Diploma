#pragma once
#include "CarLikeRobot.h"
class CarLineFollower
{
public:
	CarLineFollower(CarLikeRobot &robot, float w0, float ksi);
	~CarLineFollower();

	void setPoles();
	float getFi(float v, float delta, float p, float predictLength);
private:
	CarLikeRobot &robot;
	float b; //karakterisztikus polinom egy�that�k
	float c;
};

