#ifndef PATH_PROFILE_FUNCS_H_
#define PATH_PROFILE_FUNCS_H_

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/multi_array.hpp>
#include "PathPlannerApp\PathPlanner\Config.h"

#define C_EPS 0.0001f
#define EPS 0.001f
#define sgn(num) ((num < 0.0f) ? -1.0f : 1.0f)
#define lerp(v0, v1, t) (v0 + (v1 - v0) * t)

using namespace PathPlanner;

void curv2D(std::vector<Config>& path, std::vector<float>& curv);
void inverse3(boost::multi_array<float,2>& input, boost::multi_array<float,3>& output);
void circleTransform(Point p0, Point p1, float radius, Point &center);
void circleTransform_opt(Point p0, Point p1, float radius, Point &center);
int circleCircleIntersect_opt(Point center1, Point center2, float radius1, float radius2, Point& res0, Point& res1);
int circleLineIntersect(Point p1, Point p2, float radius, Point center, Point& res0, Point& res1);
int circleLineIntersect_opt(Point p1, Point p2, float radius, Point center, Point& res0, Point& res1);
float getDistance(Point &p0, Point &p1);
float getDirection(Point &a, Point &b);
void linearInterpolation(std::vector<float> &sourceX, std::vector<float> &sourceY, std::vector<float> &destX, std::vector<float> &destY);
int solve2ndOrder(float a, float b, float c, float& res0, float& res1);
void ProfileSave(std::string filename, std::vector<float> &data);
void ProfileSave(std::string filename, std::vector<float> &t, std::vector<float> &data);
void ProfileSave(std::string filename, std::vector<Config> &path);

#endif /* PATH_PROFILE_FUNCS_H_ */
