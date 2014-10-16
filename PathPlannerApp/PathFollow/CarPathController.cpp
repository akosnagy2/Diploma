#include "CarPathController.h"
#include "..\PathPlannerApp\PathFollow\PathShifter.h"

#define _USE_MATH_DEFINES
#include <math.h>
#include "misc.h"

CarPathController::CarPathController(std::vector<PathSegment> &paths, CarLikeRobot &car, CarLineFollower &lf, CarSpeedController &sc, float predict) :
paths(paths),
car(car),
predict(predict),
lineFollower(lf),
speedController(sc)
{
	index = 0;
	predictIndex = 0;
	pathIndex = 0;
	state = init;
	status = true;

	for(auto &ps : paths) {
		PathShifter shifter(ps, car);
		frontPath.push_back(shifter.getShiftedPath());
	}

}


CarPathController::~CarPathController()
{}

void CarPathController::Loop(Config nextPos)
{
	if(!status)
		return;

	//if(index + 3 > paths[pathIndex].path.size()) {
	//	status = false;
	//	v = 0.0f;
	//	fi = 0.0f;
	//	return;
	//}

	switch(state) {
		case init:
			v = 0.0f;
			fi = 0.0f;
			state = pathFollow;
			break;

		case pathFollow:
		{
			if(index + 1 == paths[pathIndex].path.size()) {
				state = preStop;
				v = 0.0f;
				fi = 0.0f;
				index = 0;
				pathIndex++;
				if(pathIndex == paths.size()) {
					status = false;
					return;
				}
				break;
			}
				
			/* Calculate speed control signal */
			float distError = Config::Distance(nextPos, paths[pathIndex].path[index]);
			float nextDist = Config::Distance(paths[pathIndex].path[index], paths[pathIndex].path[index + 1]);
			v = speedController.getVelocity(distError, nextDist);

			/* Calculate steer control signal */
			/* Virtual sensor middile position */
			Config intersection = getSensorCenter(nextPos);
			float delta = nextPos.phi - predictPoint.phi;
			float p = Config::Distance(intersection, predictPoint);
			if(predictPoint.phi - nextPos.phi > 0.0f) {
				p *= -1.0f;
			}
			float predictLength = Config::Distance(intersection, nextPos);
			fi = lineFollower.getFi(v, delta, p, predictLength);

			if(!paths[pathIndex].direction) {
				//fi *= -1.0f;
				v *= -1.0f;
			}

			index++;
			break;
		}
		case preStop:
			v = 0.0f;
			fi = 0.0f;
			state = turn;
			break;
		case turn:
			fi = 0.0f; //TODO valahogy ki kéne számolni hogy hova kell tekerni.
			index += predict;
			state = init;
			break;
		default:
			status = false;
			v = 0.0f;
			fi = 0.0f;
			break;
	}
}

Config CarPathController::getSensorCenter(Config carPosition)
{
	predictIndex = index;
	float dist = predict + car.getAxisDistance();
	while(Config::Distance(carPosition, frontPath[pathIndex].path[predictIndex]) < dist && predictIndex < paths[pathIndex].path.size() - 1)
		predictIndex++;
	Config intersection;
	if(Config::Distance(carPosition, frontPath[pathIndex].path[predictIndex]) < dist && predictIndex == paths[pathIndex].path.size() - 1) {
		float distError = dist - Config::Distance(carPosition, frontPath[pathIndex].path[predictIndex]);
		predictPoint.p.x = frontPath[pathIndex].path[predictIndex].p.x + cos(frontPath[pathIndex].path[predictIndex].phi) * distError;
		predictPoint.p.y = frontPath[pathIndex].path[predictIndex].p.y + sin(frontPath[pathIndex].path[predictIndex].phi) * distError;
		predictPoint.phi = frontPath[pathIndex].path[predictIndex].phi;
	} else {
		predictPoint = frontPath[pathIndex].path[predictIndex];
	}
	float teta = wrapAngle(carPosition.phi);
	if(abs(abs(teta) - M_PI_2) < EPS) {
		intersection.p.x = carPosition.p.x;
		intersection.p.y = predictPoint.p.y;
		intersection.phi = signbit(teta) ? M_PI : 0.0f;
	} else if(abs(teta) < EPS) {
		intersection.p.x = predictPoint.p.x;
		intersection.p.y = carPosition.p.y;
		intersection.phi = -M_PI_2;
	} else if(abs(abs(teta) - M_PI) < EPS) {
		intersection.p.x = predictPoint.p.x;
		intersection.p.y = carPosition.p.y;
		intersection.phi = M_PI_2;
	} else {
		float tg = tan(teta);
		intersection.p.x = (predictPoint.p.x - tg*carPosition.p.y + tg*predictPoint.p.y + tg*tg*carPosition.p.x) / (tg*tg + 1);
		intersection.p.y = (carPosition.p.y - tg*carPosition.p.x + tg*predictPoint.p.x + tg*tg*predictPoint.p.y) / (tg*tg + 1);
		intersection.phi = atan(-1 / tg);
	}
	return intersection;
}