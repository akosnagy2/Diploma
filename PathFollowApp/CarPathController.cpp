#include "CarPathController.h"
#include "PathShifter.h"

#define _USE_MATH_DEFINES
#include <math.h>
#include "misc.h"

CarPathController::CarPathController(std::vector<Position> &path, CarLikeRobot &car, CarLineFollower &lf, CarSpeedController &sc, float predict) :
path(path),
car(car),
predict(predict),
lineFollower(lf),
speedController(sc)
{
	index = 0;
	predictIndex = 0;
	state = init;
	status = true;

	PathShifter shifter(path, car);
	frontPath = shifter.getShiftedPath();
}


CarPathController::~CarPathController()
{}

void CarPathController::Loop(Position nextPos)
{
	bool reverse = false;
	if(!status)
		return;

	if(index + 3 > path.size()) {
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
//			predict += count;

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
			/* Virtual sensor middile position */
			Position intersection = getSensorCenter(nextPos);
			Position predictPoint = frontPath[predictIndex];
			float delta = nextPos.phi - predictPoint.phi;
			float p = getDistance(intersection, predictPoint);
			if(predictPoint.phi - nextPos.phi > 0.0f) {
				p *= -1.0f;
			}
			float predictLength = getDistance(intersection, nextPos);
			fi = lineFollower.getFi(v, delta, p, predictLength);

			if(reverse) {
				//fi *= -1.0f;
				v *= -1.0f;
			}

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
			fi = path[index + 1].phi;
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

Position CarPathController::getSensorCenter(Position carPosition)
{
	predictIndex = index;
	float dist = predict + car.getAxisDistance();
	while(getDistance(carPosition, frontPath[predictIndex]) < dist && predictIndex < path.size() - 1)
		predictIndex++;
	Position intersection;
	Position point = frontPath[predictIndex];
	float teta = wrapAngle(carPosition.phi);
	if(abs(abs(teta) - M_PI_2) < EPS) {
		intersection.x = carPosition.x;
		intersection.y = point.y;
		intersection.phi = signbit(teta) ? M_PI : 0.0f;
	} else if(abs(teta) < EPS) {
		intersection.x = point.x;
		intersection.y = carPosition.y;
		intersection.phi = -M_PI_2;
	} else if(abs(abs(teta) - M_PI) < EPS) {
		intersection.x = point.x;
		intersection.y = carPosition.y;
		intersection.phi = M_PI_2;
	} else {
		float tg = tan(teta);
		intersection.x = (point.x - tg*carPosition.y + tg*point.y + tg*tg*carPosition.x) / (tg*tg + 1);
		intersection.y = (carPosition.y - tg*carPosition.x + tg*point.x + tg*tg*point.y) / (tg*tg + 1);
		intersection.phi = atan(-1 / tg);
	}
	return intersection;
}