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
			predict += 10;
			count = 10;

			if(path[index + 1].phi == TURN_PHI)
				state = turn;
			else
				state = pathFollow;
			break;

		case pathFollow:
			if(path[index + 2].phi == TURN_PHI)
				state = preStop;

			if(path[index + 1].phi > 1.5f*M_PI) {
				float distError = CarPathController::getDistance(nextPos, path[index]);
				float nextDist = CarPathController::getDistance(path[index], path[index + 1]);
				v = speedController.getVelocity(distError, nextDist);

				Position predictPoint = path[index + predict];
				float delta = nextPos.phi - predictPoint.phi;
				float p = 0.0f;
				float predictLength;
				fi = lineFollower.getFi(v, delta, p, predictLength);
			} else {

			}

			index++;
			break;
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
