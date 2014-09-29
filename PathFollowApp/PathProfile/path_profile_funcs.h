#ifndef PATH_PROFILE_FUNCS_H_
#define PATH_PROFILE_FUNCS_H_

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/multi_array.hpp>

#define EPS 0.001f
#define sgn(num) ((num < 0.0f) ? -1.0f : 1.0f)
#define lerp(v0, v1, t) (v0 + (v1 - v0) * t)

using namespace boost::numeric::ublas;

#ifdef USE_ROBOTPILOT
typedef struct path_pos
{
	float x; //[mm]
	float y; //[mm]
	float phi; //[rad]
} path_pos;
#else
#include "Position.h"
#define path_pos Position
#endif



void curv2D(std::vector<path_pos>& path, std::vector<float>& curv);
void inverse3(boost::multi_array<float,2>& input, boost::multi_array<float,3>& output);
void circleTransform(path_pos p0, path_pos p1, float radius, path_pos &center);
void circleTransform_opt(path_pos p0, path_pos p1, float radius, path_pos &center);
int circleCircleIntersect_opt(path_pos center1, path_pos center2, float radius1, float radius2, path_pos& res0, path_pos& res1);
int circleLineIntersect(path_pos p1, path_pos p2, float radius, path_pos center, path_pos& res0, path_pos& res1);
int circleLineIntersect_opt(path_pos p1, path_pos p2, float radius, path_pos center, path_pos& res0, path_pos& res1);
float getDistance(path_pos &p0, path_pos &p1);
float getDirection(path_pos &a, path_pos &b);
void linearInterpolation(std::vector<float> &sourceX, std::vector<float> &sourceY, std::vector<float> &destX, std::vector<float> &destY);
int solve2ndOrder(float a, float b, float c, float& res0, float& res1);
void ProfileSave(std::string filename, std::vector<float> &data);
void ProfileSave(std::string filename, std::vector<float> &t, std::vector<float> &data);
void ProfileSave(std::string filename, std::vector<path_pos> &path);

#endif /* PATH_PROFILE_FUNCS_H_ */
