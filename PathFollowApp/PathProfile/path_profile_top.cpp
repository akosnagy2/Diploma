#include <limits>
#include <fstream>
#include "path_profile_top.h"
#include "path_profile_funcs.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <chrono>
#include <boost/date_time/posix_time/posix_time.hpp>
#include "PathPlannerApp\PathPlanner\Line.h"

using namespace std::chrono;
using namespace std;

//Maximum robot velocity
float maxV;
//Maximum wheel acceleration
float maxA;
//Maximum wheel tangent acceleration
float maxAt;
//Maximum robot angle velocity
float maxW;
//Path follow sample time
float sampleT;
//Robot wheel base
float robotWheelBase;
//Prediction length
int predictLen;

static float generateSampledPath(Profile &geoProf, Profile &sampProf, std::vector<int> &segment);
static void generatePathPointEnd(Profile &geoProf, Profile &sampProf, int i, Config &result);
static int generatePathPoint(Profile &geoProf, Profile &sampProf, int i, Config &result, bool &start, std::vector<int> &segment);
static void checkBack(Profile &geoProf, Profile &sampProf, float errorS, std::vector<int> &segment);
static int checkBack_backLimit(Profile &prof, int front_limit, int &back_limit, float &deltaV, float deltaS);
static int checkBack_frontLimit(Profile &prof, int back_limit, int &front_limit, float &deltaV, float deltaS);
static void checkProfile(Profile &prof, bool saveProfiles, std::string profile_name, ofstream &log);
static void checkGeoProfile(Profile &prof, bool saveProfiles, ofstream &log);
static void checkSampProfile(Profile &prof, bool saveProfiles, ofstream &log);
static void profile(Profile &geoProfile, std::vector<Config> &resPath, std::ofstream &logfile);


void setLimits(float _maxV, float _maxA, float _maxAt, float _maxW, float _sampleT, float _robotWheelBase, int _predictLen)
{
	maxV = _maxV;
	maxA = _maxA;
	maxAt = _maxAt;
	maxW = _maxW;
	sampleT = _sampleT;
	robotWheelBase = _robotWheelBase;
	predictLen = _predictLen;
}

static int checkBack_backLimit(Profile &prof, int front_limit, int &back_limit, float &deltaV, float deltaS)
{
	int n;
	float v, at, ac, a, w;
	for (int i = front_limit; i > 0; i--)
	{
		//Go back on profile
		if (deltaS < 0.0f)
			while ((prof.a[i] < 0.0f) && (i > 1)) i--;
		else
			while ((prof.a[i] > 0.0f) && (i > 1)) i--;

		//Set current back_limit
		n = front_limit - i + 1;
		deltaV = deltaS / (sampleT * n);
		v = prof.v[i] + deltaV;
		at = (v - prof.v[i-1])/(sampleT);
		ac = (powf(v,2))/(1/prof.c[i]);
		a = sqrtf(powf(ac,2) + powf(at,2));
		w = fabs(prof.c[i]) * v;

		//Check constraints
		if ((fabs(a) <= maxA) && (fabs(at) <= maxAt)  && (fabs(w) <= maxW) && (v > 0.0f)  && (v < maxV))
		{
			back_limit = i;
			return 0;
		}
	}
	return 1;
}

static int checkBack_frontLimit(Profile &prof, int back_limit, int &front_limit, float &deltaV, float deltaS)
{
	int n;
	float v, at, ac, a, w;
	for (int i = prof.v.size() - 2; i > back_limit; i--)
	{
		//Go back on profile
		if (deltaS < 0.0f)
			while ((prof.a[i] > 0.0f) && (i > back_limit + 1)) i--;
		else
			while ((prof.a[i] < 0.0f) && (i > back_limit + 1)) i--;

		//Set current back_limit
		n = i - back_limit + 1;
		deltaV = deltaS / (sampleT * n);
		v = prof.v[i] + deltaV;
		at = (prof.v[i+1] - v)/(sampleT);
		ac = (powf(v,2))/(1/prof.c[i]);
		a = sqrtf(powf(ac,2) + powf(at,2));
		w = fabs(prof.c[i]) * v;

		//Check constraints
		if ((fabs(a) <= maxA) && (fabs(at) <= maxAt)  && (fabs(w) <= maxW) && (v > 0.0f) && (v < maxV))
		{
			front_limit = i;
			return 0;
		}
	}
	return 1;
}

static void checkBack(Profile &geoProf, Profile &sampProf, float errorS, std::vector<int> &segment)
{
	int front_limit, back_limit, front_limit_t;
	int length = sampProf.path.size();
	float deltaV;
	bool success = false;

	//Estimate a front limit for the path
	if (errorS > 0.0f) //Geometric path end point is farther
	{
		front_limit = length - 2;
		while(sampProf.a[front_limit] < 0.0f) front_limit--;
	}
	else //Sampled path end point is farther
	{
		front_limit = length - 2;
	}

	//Calculate final limits
	do
	{
		checkBack_backLimit(sampProf,front_limit,back_limit,deltaV,errorS);
		checkBack_frontLimit(sampProf,back_limit,front_limit_t,deltaV,errorS);
		if (front_limit != front_limit_t)
		{
			front_limit = front_limit_t;
		}
		else
		{
			success = true;
			break;
		}
	} while ((front_limit > 0) && (back_limit >= 0));

	if (!success) //Return when correction failed
		return;

	for (int i = back_limit; i != front_limit + 1; i++ )
	{
		sampProf.v[i] += deltaV;
	}

	//Regenerate midified path
	sampProf.CalcDistanceFromVelocity(back_limit - 1, length - 1);;

	bool start = true;
	for (int i = back_limit; i < length; i++)
	{
		generatePathPoint(geoProf, sampProf, i, sampProf.path[i], start, segment);
	}
}

static float generateSampledPath(Profile &geoProf, Profile &sampProf, std::vector<int> &segment)
{
	int length = sampProf.v.size();
	Config result;
	bool start = true;
	float errorS = 1.0f;

	//Starting path_pos is same
	sampProf.AddPoint(geoProf.path.front());

	//Generate sampled path points
	int i;
	segment[0] = 0;
	for (i = 1; i < length; i++)
	{
		if (generatePathPoint(geoProf, sampProf, i, result, start, segment))
			break;
		sampProf.AddPoint(result);
	}

	//Generate path points after last geometric point
	for (; i < length; i++)
	{
		generatePathPointEnd(geoProf, sampProf, i, result);
		sampProf.AddPoint(result);
		errorS = -1.0f;
	}

	//Calculate end points distance error(signed)
	errorS *= getDistance(sampProf.path.back().p, geoProf.path.back().p);
	return errorS;
}

static void generatePathPointEnd(Profile &geoProf, Profile &sampProf, int i, Config &result)
{
	int length = geoProf.path.size();
	Config res0, res1;
	static bool first = true;

	if (Line::CircleLineIntersect(Line(sampProf.path[i-1].p, geoProf.path[length - 1].p), sampProf.deltaSc[i-1], sampProf.path[i-1].p, res0.p, res1.p)) //Interpolate with line
	{
		//Decide based on distance from segment end point
		if (getDistance(geoProf.path[length - 1].p, res0.p) < getDistance(geoProf.path[length - 1].p, res1.p))
			result = (first) ? res0 : res1;
		else
			result = (first) ? res1 : res0;

		first = false;
	}
}

static int generatePathPoint(Profile &geoProf, Profile &sampProf, int i, Config &result, bool &start, std::vector<int> &segment)
{
	int length = geoProf.path.size();
	Config res0, res1;
	Point &center = sampProf.path[i-1].p;
	float rad = sampProf.deltaSc[i-1];

	for (int l = segment[i-1]; l < length - 1; l++)
	{
		Point &segStart = geoProf.path[l].p;
		Point &segEnd = geoProf.path[l + 1].p;

		//Pathpoint in the next segment
		if (getDistance(segEnd, center) < (rad - EPS))
		{
			if (l == length - 2) //No next segment, return
				return 1;
			else
				continue;
		}

		if (fabs(geoProf.c[l]) < EPS)	//Interpolate with line
		{
			if (Line::CircleLineIntersect(Line(segStart, segEnd), rad, center, res0.p, res1.p))
			{
				segment[i] = l;

				//Decide based on distance from segment end point
				if (getDistance(segEnd, res0.p) < getDistance(segEnd, res1.p))
					result = res0;
				else
					result = res1;

				break;
			}
		}
		else //Interpolate with circle
		{
			static Point segCent; //Segment center point based on curvature
			static float rangeA, rangeB;

			if (!(l == segment[i-1]) || start) //New segment, calculate range
			{
				start = false;

				//Get center point for the current segment
				circleTransform_opt(segStart, segEnd, 1/geoProf.c[l], segCent);

				//Get segment start point angle
				rangeA = atan2(segStart.y - segCent.y, segStart.x - segCent.x);

				//Get segment end point angle
				rangeB = atan2(segEnd.y - segCent.y, segEnd.x - segCent.x);
			}

			segment[i] = l;

			if (circleCircleIntersect_opt(center, segCent, rad, fabs(1/geoProf.c[l]), res0.p, res1.p))
			{
				//Results points angle
				float curT1 = atan2(res0.p.y - segCent.y, res0.p.x - segCent.x);
				float curT2 = atan2(res1.p.y - segCent.y, res1.p.x - segCent.x);

				//Decide based on circle arc and range
				if (fabs(rangeA - rangeB) < M_PI)
				{
					if ((curT1 >= min(rangeA, rangeB)) && (curT1 <= max(rangeA,rangeB)))
					{
						rangeA = curT1;
						result = res0;
					}
					else
					{
						rangeA = curT2;
						result = res1;
					}
				}
				else
				{
					if (!((curT1 >= min(rangeA, rangeB)) && (curT1 <= max(rangeA,rangeB))))
					{
						rangeA = curT1;
						result = res0;
					}
					else
					{
						rangeA = curT2;
						result = res1;
					}
				}

				break;
			}
		}
	}
	return 0;
}

static void removeSamePositions(Profile &prof)
{
	int length = prof.path.size();

	for (int i = 0; i < length - 1; i++)
	{
		if (prof.deltaS[i] <= EPS)
		{
				prof.RemovePoint(i);
			length--;
		}
	}
}

static float GetVmax(Profile &prof, std::vector<float> &deltaS, int i, bool left)
{
	float a,b,c,p,res0,res1,vmax,vmax2;

	//Left or right wheel
	if (left)
		p = (1.0f - robotWheelBase*0.5f*prof.c[i]);
	else
		p = (1.0f + robotWheelBase*0.5f*prof.c[i]);

	//Solve eq. for maximum velocity
	a = powf(p, 4) / (powf(2*deltaS[i],2)) + powf((prof.c[i]*p),2);
	b = -(2.0f*powf(prof.v[i+1],2)*powf(p,4)) / (powf(2*deltaS[i],2));
	c = -powf(maxA,2) + (powf(prof.v[i+1],4)*powf(p,4))/(powf(2*deltaS[i],2));

	//Solution always exists, not necessary to check
	solve2ndOrder(a,b,c,res0,res1);
	if (res0 >= 0.0f)
		res0 = sqrtf(res0);
	else
		res0 = 0.0f;

	if (res1 >= 0.0f)
		res1 = sqrtf(res1);
	else
		res1 = 0.0f;

	//Solution based on maxA
	vmax = max(res0, res1);
	//Solution based on maxAt
	vmax2 = sqrtf(powf(prof.v[i+1],2) + 2*maxAt*deltaS[i]/(powf(p,2)));

	return min(vmax, vmax2);
}

static void correctAccelBackward(Profile &prof, std::vector<float> &Vmax, std::vector<float> &deltaSl, std::vector<float> &deltaSr, std::vector<float> &acpL, std::vector<float> &acpR, int i)
{
	//Check accel. constraints backward
	float aL, aR;
	bool goBack = false;
	do
	{
		aL = (powf(prof.v[i+1],2) - powf(prof.v[i],2)) * powf((1 - robotWheelBase*0.5f*prof.c[i]),2) / (2*deltaSl[i]);
		aR = (powf(prof.v[i+1],2) - powf(prof.v[i],2)) * powf((1 + robotWheelBase*0.5f*prof.c[i]),2) / (2*deltaSr[i]);

		prof.a[i] = (aL + aR)*0.5f;

		//Check wheels accel. constraints
		float vmL = Vmax[i], vmR = Vmax[i];
		if (fabs(aL) >= fabs(min(sqrtf(powf(maxA, 2) - powf(acpL[i], 2)), maxAt)))
		{
			vmL = GetVmax(prof, deltaSl,i,true);
			goBack = true;
		}
		else
			goBack = false;

		if (fabs(aR) >= fabs(min(sqrtf(powf(maxA, 2) - powf(acpR[i], 2)), maxAt)))
		{
			vmR = GetVmax(prof, deltaSr,i,false);
			goBack = true;
		}

		if (goBack)
		{
			Vmax[i] = min(min(vmL*(1 - EPS), vmR*(1 - EPS)), Vmax[i]);
			prof.v[i] = Vmax[i];
			acpL[i] = (powf(prof.v[i], 2)*prof.c[i]) * (1.0f - prof.c[i]*robotWheelBase*0.5f);
			acpR[i] = (powf(prof.v[i], 2)*prof.c[i]) * (1.0f + prof.c[i]*robotWheelBase*0.5f);

			i--;
		}

	} while (goBack);
}

static void generateVelocityProfile_opt(Profile &prof)
{
	int length = prof.path.size();
	std::vector<float> acpL(length);
	std::vector<float> acpR(length);
	std::vector<float> Vmax(length);
	std::vector<float> deltaSl(length-1);
	std::vector<float> deltaSr(length-1);

	prof.v.resize(prof.path.size());
	prof.a.resize(prof.path.size() - 1);
	prof.deltaSc.resize(prof.path.size()-1);
	prof.sc.resize(prof.path.size());

	//Set default velocity constraint for endpoints
	Vmax[0] = 0.0f;
	Vmax[length - 1] = 0.0f;

	int i = 0;
	while (i != length - 1)
	{
		//Set default velocity constraints
		if (i < length - 2)
			Vmax[i+1] = min(maxV, maxW/fabs(prof.c[i+1]));

		//Get travelled distance for the wheels, robot
		float r, alfa;
		if (fabs(prof.c[i]) < EPS) //Straight line
		{
			prof.deltaSc[i] = deltaSl[i] = deltaSr[i] = prof.deltaS[i];
		}
		else
		{
			r = 1/prof.c[i];
			alfa = acos(1 - 0.5f*powf((prof.deltaS[i]/r),2));

			//DeltaS always positive
			deltaSl[i] = alfa*fabs((r - robotWheelBase*0.5f));
			deltaSr[i] = alfa*fabs((r + robotWheelBase*0.5f));
			prof.deltaSc[i] = alfa*fabs(r);
		}
		prof.sc[i+1] = prof.sc[i] + prof.deltaSc[i];

		//Centripetal accel. for wheels
		acpL[i] = (powf(prof.v[i], 2)*prof.c[i]) * (1.0f - prof.c[i]*robotWheelBase*0.5f);
		acpR[i] = (powf(prof.v[i], 2)*prof.c[i]) * (1.0f + prof.c[i]*robotWheelBase*0.5f);

		//Cent. accel. alone exceeds the accel. constraint
		if (max(fabs(acpL[i]), fabs(acpR[i])) >= maxA)
		{
			float vmL = numeric_limits<float>::infinity(), vmR = numeric_limits<float>::infinity();

			//Calc max velocity, meets contraints
			if (fabs(acpL[i]) >= maxA)
				vmL = sqrtf(fabs(maxA/(prof.c[i] - robotWheelBase*0.5f*powf(prof.c[i], 2))));
			if (fabs(acpR[i]) >= maxA)
				vmR = sqrtf(fabs(maxA/(prof.c[i] + robotWheelBase*0.5f*powf(prof.c[i], 2))));

			Vmax[i] = min(min(vmL*(1 - EPS), vmR*(1 - EPS)), Vmax[i]);

			prof.v[i] = Vmax[i];

			acpL[i] = (powf(prof.v[i], 2)*prof.c[i]) * (1.0f - prof.c[i]*robotWheelBase*0.5f);
			acpR[i] = (powf(prof.v[i], 2)*prof.c[i]) * (1.0f + prof.c[i]*robotWheelBase*0.5f);

			//Check accel. constraints backward
			correctAccelBackward(prof,Vmax,deltaSl, deltaSr,acpL,acpR, i - 1);
		}

		//Accel. constraints for wheels
		float alimitL = min(sqrtf(powf(maxA, 2) - powf(acpL[i], 2)), maxAt);
		float alimitR = min(sqrtf(powf(maxA, 2) - powf(acpR[i], 2)), maxAt);

		//Calculate wheel accel. based on constraints
		float ar, al;
		ar = alimitL * (1 + robotWheelBase*0.5f*prof.c[i]) / (1 - robotWheelBase*0.5f*prof.c[i]);
		al = alimitR * (1 - robotWheelBase*0.5f*prof.c[i]) / (1 + robotWheelBase*0.5f*prof.c[i]);
		if (fabs(ar) > alimitR)
			ar = alimitR;
		else if (fabs(al) > alimitL)
			al = alimitL;
		else
		{
			if ((al + alimitR) > (ar + alimitL))
				ar = alimitR;
			else
				al = alimitL;
		}

		//Set robot accel.
		prof.a[i] = (al + ar)*0.5f;

		//Set robot velocity
		prof.v[i+1] = sqrtf(powf(prof.v[i], 2) + 2*prof.a[i]*prof.deltaSc[i]);

		//Check robot velocity constraint
		if (prof.v[i+1] > Vmax[i+1])
		{
			prof.v[i+1] = Vmax[i+1];

			correctAccelBackward(prof,Vmax,deltaSl,deltaSr,acpL,acpR, i);
		}
		i++;
	}
}

static void checkProfile(Profile &prof, bool saveProfiles, std::string profile_name, ofstream &log)
{
	std::vector<float> w(prof.v.size());
	std::vector<float> at_left(prof.v.size());
	std::vector<float> at_right(prof.v.size());
	std::vector<float> a_left(prof.v.size());
	std::vector<float> a_right(prof.v.size());

	float acp_left, acp_right;
	for (int i = 0; i < prof.v.size() - 1; i++)
	{
		//Check angular velocity
		w[i] = prof.v[i] * prof.c[i];
		if ((fabs(w[i]) > (maxW + EPS)) || (boost::math::isnan(w[i])))
			log << "Profile Check: angular velocity error in " << i << ". point." << endl;

		//Check robot velocity
		if ((fabs(prof.v[i]) > (maxV + EPS)) || (boost::math::isnan(prof.v[i])))
			log << "Profile Check: robot velocity error in " << i << ". point." << endl;

		//Check wheels tangent accel.
		at_left[i] = (prof.v[i+1] - prof.v[i])*(1.0f - prof.c[i]*robotWheelBase*0.5f)/prof.deltaT[i];
		at_right[i] = (prof.v[i+1] - prof.v[i])*(1.0f + prof.c[i]*robotWheelBase*0.5f)/prof.deltaT[i];
		if ((fabs(at_left[i]) > (maxAt + EPS)) || (boost::math::isnan(at_left[i])))
			log << "Profile Check: left wheel tangent acceleration error in " << i << ". point." << endl;
		if ((fabs(at_right[i]) > (maxAt + EPS)) || (boost::math::isnan(at_right[i])))
			log << "Profile Check: right wheel tangent acceleration error in " << i << ". point." << endl;
		if (fabs((at_right[i]/at_left[i]) - ((1 + robotWheelBase*0.5f*prof.c[i]) / (1 - robotWheelBase*0.5f*prof.c[i]))) > EPS)
			log << "Profile Check: wheels tangent accelerations ratio error in " << i << ". point." << endl;

		//Check wheels accel.
		acp_left = (powf(prof.v[i], 2)*prof.c[i]) * (1.0f - prof.c[i]*robotWheelBase*0.5f);
		a_left[i] = sqrtf(powf(acp_left,2) + powf(at_left[i],2));
		acp_right = (powf(prof.v[i], 2)*prof.c[i]) * (1.0f + prof.c[i]*robotWheelBase*0.5f);
		a_right[i] = sqrtf(powf(acp_right,2) + powf(at_right[i],2));
		if ((fabs(a_left[i]) > (maxA + EPS)) || (boost::math::isnan(a_left[i])))
			log << "Profile Check: left wheel acceleration error in " << i << ". point." << endl;
		if ((fabs(a_right[i]) > (maxA + EPS)) || (boost::math::isnan(a_right[i])))
			log << "Profile Check: right wheel acceleration error in " << i << ". point." << endl;
	}

	if (saveProfiles)
	{
		ProfileSave(profile_name + "_W.txt",prof.t, w);
		ProfileSave(profile_name + "_V.txt",prof.t, prof.v);
		ProfileSave(profile_name + "_A.txt",prof.t, prof.a);
		ProfileSave(profile_name + "_A_Left.txt",prof.t, a_left);
		ProfileSave(profile_name + "_A_Right.txt",prof.t, a_right);
		ProfileSave(profile_name + "_At_Left.txt",prof.t, at_left);
		ProfileSave(profile_name + "_At_Right.txt",prof.t, at_right);
		ProfileSave(profile_name + "_Path.txt",prof.path);
		ProfileSave(profile_name + "_Sc.txt",prof.t,prof.sc);
		ProfileSave(profile_name + "_S.txt",prof.t,prof.s);
		ProfileSave(profile_name + "_C.txt",prof.sc, prof.c);

		log << "Profile Check: profiles saved." << endl;
	}
}

static void interpolateVelocity(Profile &geoProf, Profile &sampProf)
{
	linearInterpolation(geoProf.t, geoProf.v, sampProf.t, sampProf.v);
}

static void interpolateCurvature(Profile &geoProf, Profile &sampProf)
{
	linearInterpolation(geoProf.sc, geoProf.c, sampProf.sc, sampProf.c);
}


static void checkGeoProfile(Profile &prof, bool saveProfiles, ofstream &log)
{
	checkProfile(prof, saveProfiles,"prof_Geo" + prof.profileName, log);
}

static void checkSampProfile(Profile &prof, bool saveProfiles, ofstream &log)
{
	checkProfile(prof, saveProfiles,"prof_Samp" + prof.profileName, log);
}

//TODO: V-rep-ben tesztelni; új görbület számítás: jobb, gyorsabb lehetne
//TODO: tolatás, egy helyben fordulás, nem egyenlő távolságú geometriai pályával tesztelni
//TODO: profile méretet külön tárolni, oo-bá tenni az egészet

static void profile(Profile &geoProfile, std::vector<Config> &resPath, std::ofstream &logfile)
{
	int geoLength, sampLength;
	std::chrono::high_resolution_clock::time_point start, stop;

	/*
	 * Geometric profile--------------------------------------------------------------------
	 */

	//Get path points distance, remove same points
	start = high_resolution_clock::now();
	//geoProfile.CalcPathDistance();
	removeSamePositions(geoProfile);
	geoLength = geoProfile.path.size();
	stop = high_resolution_clock::now();
	logfile << "Geometric profile: " << geoProfile.path.size() - geoLength << " points removed, duration: " <<  duration_cast<chrono::microseconds>(stop-start).count() << " us." << endl;

	//Curvature estimation
	start = high_resolution_clock::now();
	geoProfile.CalcCurvature();
	stop = high_resolution_clock::now();
	logfile << "Geometric profile: curvature estimated, duration: " <<  duration_cast<chrono::microseconds>(stop-start).count() << " us." << endl;

	//Velocity profile generation
	start = high_resolution_clock::now();
	generateVelocityProfile_opt(geoProfile);
	geoProfile.CalcTime();
	stop = high_resolution_clock::now();
	logfile << "Geometric profile: profile created, duration: " <<  duration_cast<chrono::microseconds>(stop-start).count() << " us." << endl;

	//Check and save profiles
	start = high_resolution_clock::now();
	checkGeoProfile(geoProfile, SAVE_PROFILE, logfile);
	stop = high_resolution_clock::now();
	logfile << "Geometric profile: profile checked, duration: " <<  duration_cast<chrono::microseconds>(stop-start).count() << " us." << endl;

	/*
	 * Timing rescaled profile--------------------------------------------------------------------
	 */
	Profile sampProfile(geoProfile.profileName);
	logfile << "Sampled profile generation started." << endl;
	logfile << "Sampled profile: sampling time: " << sampleT << " s" << endl;

	//Determinate sampled path length based on sample time
	sampLength = (int)ceil(geoProfile.t[geoLength - 1] / sampleT) + 1;
	sampProfile.SetSampleTime(sampleT, sampLength);
	logfile << "Sampled profile: path length: " << sampLength << endl;

	//Linear interpolate sampled path velocity
	start = high_resolution_clock::now();
	interpolateVelocity(geoProfile, sampProfile);
	stop = high_resolution_clock::now();
	logfile << "Sampled profile: velocity interpolation, duration: " <<  duration_cast<chrono::microseconds>(stop-start).count() << " us." << endl;

	//Calculate sampled path points distances
	start = high_resolution_clock::now();
	sampProfile.CalcDistanceFromVelocity();
	stop = high_resolution_clock::now();
	logfile << "Sampled profile: distance calculation, duration: " <<  duration_cast<chrono::microseconds>(stop-start).count() << " us." << endl;

	//Generate sampled path points coordinates
	start = high_resolution_clock::now();
	std::vector<int> segment(sampProfile.v.size());
	float errorS = generateSampledPath(geoProfile, sampProfile, segment);
	stop = high_resolution_clock::now();
	logfile << "Sampled profile: path points generation, duration: " <<  duration_cast<chrono::microseconds>(stop-start).count() << " us." << endl;

	//Estimate curvature, acceleration
	start = high_resolution_clock::now();
	sampProfile.CalcPathDistance();
	interpolateCurvature(geoProfile, sampProfile);
	sampProfile.CalcAcceleration();
	stop = high_resolution_clock::now();
	logfile << "Sampled profile: curvature, acceleration estimation, duration: " <<  duration_cast<chrono::microseconds>(stop-start).count() << " us." << endl;

	//Check and correct path back
	start = high_resolution_clock::now();
	//checkBack(geoProfile, sampProfile, errorS, segment);
	stop = high_resolution_clock::now();
	logfile << "Sampled profile: path end point correction, duration: " <<  duration_cast<chrono::microseconds>(stop-start).count() << " us." << endl;

	//Set output path
	resPath = sampProfile.path;

	//Sampled points orientation
	if (fabs(getDirection(resPath[0].p, resPath[1].p) - geoProfile.path[0].phi) < EPS)
	{
		for (int i = 0; i < resPath.size() - 1; i++) //Forward direction
			resPath[i].phi = getDirection(resPath[i].p, resPath[i + 1].p); //[-pi, pi] forward range
	}
	else
	{
		for (int i = 0; i < resPath.size() - 1; i++) //Backward direction
			resPath[i].phi = getDirection(resPath[i].p, resPath[i + 1].p) + M_PI*3; //[2*pi, 4*pi] backward range
	}
	resPath[resPath.size() - 1].phi = resPath[resPath.size() - 2].phi;

	//Check and save profiles
	start = high_resolution_clock::now();
	checkSampProfile(sampProfile, SAVE_PROFILE, logfile);
	stop = high_resolution_clock::now();
	logfile << "Sampled profile: profile checked, duration: " <<  duration_cast<chrono::microseconds>(stop-start).count() << " us." << endl;

	logfile << "Time parameterized path generation ended " << boost::posix_time::second_clock::local_time().date() << " " << boost::posix_time::second_clock::local_time().time_of_day() << endl;
}

void profile_top(std::vector<Config> &path, std::vector<Config> &resPath)
{
	int length = path.size();
	std::vector<Config> sampPathSegment;
	std::ofstream logfile("logFile.txt", ios_base::app);
	logfile << "Time parameterized path generation started " << boost::posix_time::second_clock::local_time().date() << " " << boost::posix_time::second_clock::local_time().time_of_day() << endl;

	logfile << "Robot parameters:" << endl;
	logfile << "Robot wheel base: " << robotWheelBase << " m" << endl;
	logfile << "Robot maximum velocity: " << maxV << " m/s" << endl;
	logfile << "Robot maximum angular velocity: " << maxW << " 1/s" << endl;
	logfile << "Robot wheel maximum acceleration: " << maxA << " m/s^2" << endl;
	logfile << "Robot wheel maximum tangent acceleration: " << maxAt << " m/s^2" << endl;

	Profile geoProfile(path);
	geoProfile.CalcPathDistance();

	int start = 0;
	for (int i = 0; i < length; i++)
	{
		if ((i == length - 1) || (geoProfile.deltaS[i] <= EPS))
		{
			Profile geoProfileSegment(geoProfile, start, i);

			profile(geoProfileSegment, sampPathSegment, logfile);

			resPath.insert(resPath.end(), sampPathSegment.begin(), sampPathSegment.end());
			Config p = sampPathSegment.back();
			p.phi = 1000.0f;
			for (int j = 0; j < predictLen; j++)
				resPath.insert(resPath.end(), p);

			start = i + 1;
		}
	}
}