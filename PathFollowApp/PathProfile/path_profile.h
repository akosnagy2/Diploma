#ifndef PATH_PROFILE_H_
#define PATH_PROFILE_H_

#include <vector>
#include "PathPlannerApp\PathPlanner\Config.h"
#include "path_profile_funcs.h"

using namespace PathPlanner;

enum ERROR_CODE
{
	ERROR_NO_ERROR,
	ERROR_NO_PATH,
	ERROR_NO_VELOCITY,
	ERROR_NO_DISTANCE,
	ERROR_NO_TIME,
	ERROR_NO_CURVATURE,
	ERROR_INVALID_ARGUMENT
};

class Profile
{
public:
	std::vector<Config>	path;	 //path coords
	std::vector<float>		deltaS;  //delta distances
	std::vector<float>		deltaSc; //delta distances on curvature
	std::vector<float>		s;		 //distances
	std::vector<float>		sc;		 //distances on curvature
	std::vector<float>		t;		 //times
	std::vector<float>		deltaT;  //delta times
	std::vector<float>		v;		 //velocities
	std::vector<float>		c;		 //curvature
	std::vector<float>		a;		 //acceleration

	std::string	profileName;

	Profile(std::string fileName);
	Profile(std::vector<Config> _path);
	Profile(Profile &prof, int start, int end);

	int AddPoint(Config point);
	int AddPoint(float velocity);
	int SetSampleTime(float sample_s);
	int SetSampleTime(float sample_s, int _length);

	int RemovePoint(int index);

	int CalcDistanceFromPathDistance();
	int CalcPathDistance();
	int CalcDistanceFromVelocity();
	int CalcDistanceFromVelocity(int from, int to);
	int CalcTime();
	int CalcAcceleration();
	int CalcTangentAcceleration();
	int CalcCurvature();
	int CalcVelocity();
};


#endif /* PATH_PROFILE_H_ */
