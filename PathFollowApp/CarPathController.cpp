#include "CarPathController.h"

#define _USE_MATH_DEFINES
#include <math.h>

CarPathController::CarPathController(std::vector<Position> &path, CarLineFollower &lf, CarSpeedController &sc, int predict) :
path(path),
predict(predict),
lineFollower(lf),
speedController(sc)
{
	index = 0;
	state = init;
	status = true;
}


CarPathController::~CarPathController()
{}

void CarPathController::Loop(Position nextPos)
{
	bool reverse = false;
	if(!status)
		return;

	if(index + predict >= path.size() - 1) {
		status = false;
		v = 0.0f;
		fi = 0.0f;
		return;
	}

	switch(state) {
		case init:
			v = 0.0f;
			fi = 0.0f;
			count = 10;
			predict += count;

			if(path[index + 1].phi == TURN_PHI)
				state = turn;
			else
				state = pathFollow;
			break;

		case pathFollow:
		{
			if(path[index + 2].phi == TURN_PHI)
				state = preStop;

			if(path[index + 1].phi > 1.5f*M_PI) {
				reverse = true;
			} else {
				reverse = false;
			}

			/* Calculate speed control signal */
			float distError = CarPathController::getDistance(nextPos, path[index]);
			float nextDist = CarPathController::getDistance(path[index], path[index + 1]);
			v = speedController.getVelocity(distError, nextDist);

			/* Calculate steer control signal */
			Position predictPoint = path[index + predict];
			float delta = nextPos.phi - predictPoint.phi;
			/* Virtual sensor middile position */
			Position intersection = getSensorCenter(nextPos, predictPoint);
			float p = getDistance(intersection, predictPoint);
			if(abs(atan2(predictPoint.y - intersection.y, predictPoint.x - intersection.x)) > M_PI_2) {
				p *= -1.0f;
			}
			float predictLength = getDistance(intersection, nextPos);
			fi = lineFollower.getFi(v, delta, p, predictLength);

			if(reverse) {
				fi *= -1.0f;
				v *= -1.0f;
			}

			rabbit = path[index + predict];
			index++;

			if(count > 0) {
				count--;
				predict--;
			}
			break;
		}
		case preStop:
			v = 0.0f;
			fi = 0.0f;
			state = turn;
			break;
		case turn:
			fi = path[index + predict + 1].phi;
			index += predict;
			state = pathFollow;
			break;
		default:
			status = false;
			v = 0.0f;
			fi = 0.0f;
			break;
	}
}

float CarPathController::getDistance(Position a, Position b)
{
	return sqrt((a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y));
}

Position CarPathController::getSensorCenter(Position car, Position point)
{
	Position intersection;
	float teta = wrapAngle(car.phi);
	if(abs(abs(teta) - M_PI_2) < EPS) {
		intersection.x = car.x;
		intersection.y = point.y;
		intersection.phi = signbit(teta) ? M_PI : 0.0f;
	} else if(abs(teta) < EPS) {
		intersection.x = point.x;
		intersection.y = car.y;
		intersection.phi = -M_PI_2;
	} else if(abs(abs(teta) - M_PI) < EPS) {
		intersection.x = point.x;
		intersection.y = car.y;
		intersection.phi = M_PI_2;
	} else {
		float tg = tan(teta);
		intersection.x = (point.x - tg*car.y + tg*point.y + tg*tg*car.x) / (tg*tg + 1);
		intersection.y = (car.y - tg*car.x + tg*point.x + tg*tg*point.y) / (tg*tg + 1);
		intersection.phi = atan(-1 / tg);
	}
	return intersection;
}

float CarPathController::wrapAngle(float phi)
{
	phi = fmod(phi + M_PI, 2 * M_PI);
	if(phi < 0)
		phi += 2 * M_PI;
	return phi - M_PI;
}