#include "CarPathController.h"
#include "..\PathPlannerApp\PathFollow\PathShifter.h"
#include "Geometry/Angle.h"
#include <boost/math/special_functions.hpp>

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

			if(!paths[pathIndex].direction) {
				nextPos.phi = wrapAngle(nextPos.phi + M_PI);
			}

			/* Calculate speed control signal */
			float distError = Config::Distance(nextPos, paths[pathIndex].path[index]);
			float nextDist = Config::Distance(paths[pathIndex].path[index], paths[pathIndex].path[index + 1]);
			v = speedController.getVelocity(distError, nextDist);

			/* Calculate steer control signal */
			/* Virtual sensor middile position */
			Config intersection = getSensorCenter(nextPos);
			float delta = wrapAngle(nextPos.phi - predictPoint.phi);

			float p = Config::Distance(intersection, predictPoint);

			Config x = Config(predictPoint.p.x + 1000 * cos(predictPoint.phi), predictPoint.p.y + 1000 * sin(predictPoint.phi), predictPoint.phi);
			if((x.p.x - predictPoint.p.x)*(intersection.p.y - predictPoint.p.y)
				- (intersection.p.x - predictPoint.p.x)*(x.p.y - predictPoint.p.y) < 0.0f) {
				p *= -1.0f;
			}
			fi = lineFollower.getFi(delta, p);

			if(!paths[pathIndex].direction) {
				fi *= -1.0f;
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
	Config intersection;
	int predictIndex = index;
	float dist = predict + car.getAxisDistance();
	predictPoint = frontPath[pathIndex].path[predictIndex];

	while(predictIndex < paths[pathIndex].path.size() - 1 && Config::Distance(carPosition, predictPoint) < dist) {
		predictPoint = frontPath[pathIndex].path[++predictIndex];
	}

	if(predictIndex == paths[pathIndex].path.size() - 1 && Config::Distance(carPosition, predictPoint) < dist) {
		float distError = dist - Config::Distance(carPosition, frontPath[pathIndex].path[predictIndex]);
		predictPoint.p.x = predictPoint.p.x + cos(predictPoint.phi) * distError;
		predictPoint.p.y = predictPoint.p.y + sin(predictPoint.phi) * distError;
	}

	if(!paths[pathIndex].direction)
		predictPoint.phi = wrapAngle(predictPoint.phi + M_PI);

	float teta = wrapAngle(carPosition.phi);
	if(abs(abs(teta) - M_PI_2) < EPS) {
		intersection.p.x = carPosition.p.x;
		intersection.p.y = predictPoint.p.y;
		intersection.phi = boost::math::signbit(teta) ? M_PI : 0.0f;
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