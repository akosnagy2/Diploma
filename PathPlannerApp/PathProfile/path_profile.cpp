#include "path_profile.h"
#include "path_profile_funcs.h"

#include <boost/math/special_functions.hpp>

int Profile::CalcTime()
{
	if (v.size() == 0)
		return ERROR_NO_VELOCITY;

	if (deltaSc.size() == 0)
		return ERROR_NO_DISTANCE;

	t.resize(v.size());
	deltaT.resize(v.size() - 1);

	t[0] = 0.0f;
	for (int i = 0; i < (int)v.size() - 1; i++)
	{
		deltaT[i] = (2 * deltaSc[i]) / (v[i+1] + v[i]);

		t[i+1] = t[i] + deltaT[i];
	}
	return ERROR_NO_ERROR;
}

int Profile::CalcDistanceFromPathDistance()
{
	if (s.size() == 0)
		return ERROR_NO_DISTANCE;
	if (c.size() == 0)
		return ERROR_NO_CURVATURE;

	sc.resize(path.size());
	deltaSc.resize(path.size() - 1);

	//Get travelled distance for robot
	float alfa, r;
	for (int i = 0; i < (int)s.size() - 2; i++)
	{
		if (fabs(c[i]) < EPS) //Straight line
		{
			deltaSc[i] = deltaS[i];
		}
		else
		{
			r = 1/c[i];
			alfa = acosf(1.0f - 0.5f * powf((deltaS[i]/r),2));

			//DeltaS always positive
			deltaSc[i] = alfa*fabs(r);
		}
		sc[i+1] = sc[i] + deltaSc[i];
	}
	return ERROR_NO_ERROR;
}

int Profile::CalcPathDistance()
{
	if (path.size() == 0)
		return ERROR_NO_PATH;

	s.resize(path.size());
	deltaS.resize(path.size() - 1);

	s[0] = 0.0f;
	for (int i = 0; i < (int)path.size() - 1; i++)
	{
		deltaS[i] = sqrtf( powf((path[i+1].p.x - path[i].p.x),2) + powf((path[i+1].p.y - path[i].p.y),2) );

		s[i+1] = s[i] + deltaS[i];
	}

	return ERROR_NO_ERROR;
}

int Profile::CalcDistanceFromVelocity()
{
	if (v.size() == 0)
		return ERROR_NO_VELOCITY;

	if (t.size() == 0)
		return ERROR_NO_TIME;

	sc.resize(v.size());
	deltaSc.resize(v.size() - 1);

	sc[0] = 0.0f;
	for (int i = 0; i < (int)v.size() - 1; i++)
	{
		deltaSc[i] = (v[i+1] + v[i]) * deltaT[i] / 2.0f;

		sc[i+1] = sc[i] + deltaSc[i];
	}

	return ERROR_NO_ERROR;
}

int Profile::CalcDistanceFromVelocity(int from, int to)
{
	if (v.size() == 0)
		return ERROR_NO_VELOCITY;

	if (t.size() == 0)
		return ERROR_NO_TIME;

	sc.resize(v.size());
	deltaSc.resize(v.size() - 1);

	if (to > (int)v.size() - 1)
		return ERROR_INVALID_ARGUMENT;

	if ((from < 0) || (from > to))
		return ERROR_INVALID_ARGUMENT;

	for (int i = from; i < to; i++)
	{
		if (i == 0)
		{
			sc[0] = 0.0f;
		}
		else
		{
			deltaSc[i] = (v[i+1] + v[i]) * deltaT[i] / 2.0f;
			sc[i+1] = sc[i] + deltaSc[i];
		}
	}

	return ERROR_NO_ERROR;
}

int Profile::CalcTangentAcceleration()
{
	if (v.size() == 0)
		return ERROR_NO_VELOCITY;

	if (deltaT.size() == 0)
		return ERROR_NO_TIME;

	a.resize(v.size() - 1);

	for (int i = 0; i < (int)v.size() - 1; i++)
	{
		a[i] = (v[i+1] - v[i])/(deltaT[i]);
	}

	return ERROR_NO_ERROR;
}

int Profile::CalcAcceleration()
{
	if (v.size() == 0)
		return ERROR_NO_VELOCITY;

	if (deltaT.size() == 0)
		return ERROR_NO_TIME;

	if (c.size() == 0)
		return ERROR_NO_CURVATURE;

	a.resize(v.size() - 1);

	for (int i = 0; i < (int)v.size() - 1; i++)
	{
		float a_trans = (v[i+1] - v[i])/(deltaT[i]);
		float a_cent = (powf(v[i],2))/(1/c[i]);

		if (boost::math::isnan(a_cent))
			a_cent = 0.0f;
		if (boost::math::isnan(a_trans))
			a_trans = 0.0f;

		a[i] = sqrtf(powf(a_cent,2) + powf(a_trans,2));
		if (a_trans < 0.0f)
			a[i] *= -1.0f;
	}

	return ERROR_NO_ERROR;
}

int Profile::CalcVelocity()
{
	if (deltaSc.size() == 0)
		return ERROR_NO_DISTANCE;

	if (deltaT.size() == 0)
		return ERROR_NO_TIME;

	v.resize(sc.size());

	v[0] = 0.0f;
	for (int i = 0; i < (int)sc.size() - 1; i++)
	{
		v[i+1] = 2 * deltaSc[i] / deltaT[i] - v[i];
	}

	return ERROR_NO_ERROR;
}

int Profile::CalcCurvature()
{
	if (path.size() == 0)
		return ERROR_NO_PATH;

	c.resize(path.size());

	curv2D(path, c);

	//Corrigate curvature
	for (int i = 0; i < (int)c.size() - 1; i++)
		if ((c[i] != 0) && (2*fabs(1/c[i]) <= deltaS[i]))
			c[i] = 1/(deltaS[i] * 0.5f * sgn(c[i]) * (1 + EPS));

	return ERROR_NO_ERROR;
}

int Profile::AddPoint(Config point)
{
	path.push_back(point);
	return ERROR_NO_ERROR;
}

int Profile::AddPoint(float velocity)
{
	v.push_back(velocity);
	return ERROR_NO_ERROR;
}

int Profile::SetSampleTime(float sample_s)
{
	if (path.size() == 0)
		return ERROR_NO_PATH;

	t.resize(path.size());
	deltaT.resize(path.size() - 1);

	t[0] = 0.0f;
	for (int i = 0; i < (int)path.size() - 1; i++)
	{
		deltaT[i] = sample_s;

		t[i+1] = t[i] + deltaT[i];
	}

	return ERROR_NO_ERROR;
}

int Profile::SetSampleTime(float sample_s, int _length)
{
	t.resize(_length);
	deltaT.resize(_length - 1);

	t[0] = 0.0f;
	for (int i = 0; i < _length - 1; i++)
	{
		deltaT[i] = sample_s;

		t[i+1] = t[i] + deltaT[i];
	}

	return ERROR_NO_ERROR;
}

int Profile::RemovePoint(int index)
{
	if (deltaS.size())
		deltaS.erase(deltaS.begin() + index-1);

	if (path.size())
		path.erase(path.begin() + index);
	
	if (s.size())
		s.erase(s.begin() + index);
	
	if (sc.size())
		sc.erase(sc.begin() + index);
	
	if (deltaSc.size())
		deltaSc.erase(deltaSc.begin() + index-1);
	
	if (a.size())
		a.erase(a.begin() + index);
	
	if (c.size())
		c.erase(c.begin() + index);
	
	if (deltaT.size())
		deltaT.erase(deltaT.begin() + index-1);
	
	if (t.size())
		t.erase(t.begin() + index);
	
	if (v.size())
		v.erase(v.begin() + index);

	return ERROR_NO_ERROR;
}

Profile::Profile(std::string fileName)
{
	profileName = fileName;
}

Profile::Profile(std::vector<Config> _path)
{
	path = _path;
}

Profile::Profile(Profile &prof, int start, int end)
{
	if (prof.path.size()) //TODO: ez így undorító...
		if (end == prof.path.size() - 1) //Endpoint 
			this->path = std::vector<Config>(prof.path.begin() + start,  prof.path.end());
		else
			this->path = std::vector<Config>(prof.path.begin() + start,  prof.path.begin() + end + 1);		
	
	if (prof.deltaS.size())
		if (end == prof.path.size() - 1) //Endpoint
			this->deltaS = std::vector<float>(prof.deltaS.begin() + start, prof.deltaS.end());
		else
			this->deltaS = std::vector<float>(prof.deltaS.begin() + start, prof.deltaS.begin() + end);

	profileName = "Profile_" + std::to_string(start) + "_" + std::to_string(end);
}
