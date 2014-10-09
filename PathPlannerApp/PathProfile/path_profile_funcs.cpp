#include "path_profile_funcs.h"
#include <string>
#include <fstream>

using namespace boost::numeric::ublas;

void curv2D(std::vector<Config> &path, std::vector<float> &curv)
{
	//http://www.mathworks.com/matlabcentral/fileexchange/32696-2d-line-curvature-and-normals
	int length = path.size();
	std::vector<float> Ta(length);
	std::vector<float> Tb(length);
	std::vector<int> Na(length);
	std::vector<int> Nb(length);
	std::vector<int> Naa(length);
	std::vector<int> Nbb(length);
	boost::multi_array<float,2> a(boost::extents[length][3]);
	boost::multi_array<float,2> b(boost::extents[length][3]);
	matrix<float> x(length,3);
	matrix<float> y(length,3);
	boost::multi_array<float,2> M(boost::extents[length][9]);
	boost::multi_array<float,3> invM(boost::extents[length][3][3]);

	//Get left and right neighbor of each points
	for (int i = 0; i < length - 1; i++)
	{
		Na[i] = i + 2;
		Nb[i] = i;
	}
	Nb[length-1] = length-1;

	//Check for end of line points, without a left or right neighbor
	std::vector<int> checkNa;
	std::vector<int> checkNb;
	for (int i = 0; i < length; i++)
	{
		if (Na[i] == 0)
			checkNa.push_back(i);
		if (Nb[i] == 0)
			checkNb.push_back(i);
	}
	Naa = Na;
	Nbb = Nb;
	for (int i = 0; i < checkNa.size(); i++)
	{
		Naa[checkNa[i]] = checkNa[i] + 1;
	}
	for (int i = 0; i < checkNb.size(); i++)
	{
		Nbb[checkNb[i]] = checkNb[i] + 1;
	}

	//If no left neighbor use two right neighbors, and the same for right...
	for (int i = 0; i < checkNa.size(); i++)
	{
		Na[checkNa[i]] = Nbb[Nbb[checkNa[i]]-1];
	}
	for (int i = 0; i < checkNb.size(); i++)
	{
		Nb[checkNb[i]] = Naa[Naa[checkNb[i]]-1];
	}

	//Correct for sampeling differences
	for (int i = 0; i < length; i++)
	{
		//TODO: Na,Nb-ben eltolva kéne tárolni az indexeket
		Ta[i] = -sqrtf( powf((path[i].p.x - path[Na[i]-1].p.x),2) + powf((path[i].p.y - path[Na[i]-1].p.y),2));
		Tb[i] = sqrtf( powf((path[i].p.x - path[Nb[i]-1].p.x),2) + powf((path[i].p.y - path[Nb[i]-1].p.y),2));
	}

	//If no left neighbor use two right neighbors, and the same for right...
	for (int i = 0; i < checkNa.size(); i++)
	{
		Ta[checkNa[i]] = -Ta[checkNa[i]];
	}
	for (int i = 0; i < checkNb.size(); i++)
	{
		Tb[checkNb[i]] = -Tb[checkNb[i]];
	}

	//Fit a polygons to the vertices
	//x=a(3)*t^2 + a(2)*t + a(1)
	//y=b(3)*t^2 + b(2)*t + b(1)
	//we know the x,y of every vertice and set t=0 for the vertices, and
	//t=Ta for left vertices, and t=Tb for right vertices,
	for (int i = 0; i < length; i++)
	{
		x(i,0) = path[Na[i]-1].p.x;
		x(i,1) = path[i].p.x;
		x(i,2) = path[Nb[i]-1].p.x;

		y(i,0) = path[Na[i]-1].p.y;
		y(i,1) = path[i].p.y;
		y(i,2) = path[Nb[i]-1].p.y;

		M[i][0] = 1.0f;
		M[i][1] = -Ta[i];
		M[i][2] = powf(Ta[i],2);
		M[i][3] = 1.0f;
		M[i][4] = 0.0f;
		M[i][5] = 0.0f;
		M[i][6] = 1.0f;
		M[i][7] = -Tb[i];
		M[i][8] = powf(Tb[i],2);
	}
	inverse3(M, invM);

	for (int i = 0; i < length; i++)
	{
		a[i][0] = invM[i][0][0]*x(i,0) + invM[i][1][0]*x(i,1) + invM[i][2][0]*x(i,2);
		a[i][1] = invM[i][0][1]*x(i,0) + invM[i][1][1]*x(i,1) + invM[i][2][1]*x(i,2);
		a[i][2] = invM[i][0][2]*x(i,0) + invM[i][1][2]*x(i,1) + invM[i][2][2]*x(i,2);

		b[i][0] = invM[i][0][0]*y(i,0) + invM[i][1][0]*y(i,1) + invM[i][2][0]*y(i,2);
		b[i][1] = invM[i][0][1]*y(i,0) + invM[i][1][1]*y(i,1) + invM[i][2][1]*y(i,2);
		b[i][2] = invM[i][0][2]*y(i,0) + invM[i][1][2]*y(i,1) + invM[i][2][2]*y(i,2);

		//Calculate the curvature from the fitted polygon
		curv[i] = 2 * (a[i][1]*b[i][2] - a[i][2]*b[i][1]) / powf((powf(a[i][1],2) + powf(b[i][1],2)),1.5);
	}
}

void inverse3(boost::multi_array<float,2>& input, boost::multi_array<float,3>& output)
{
	std::vector<float> detM(input.size());

	for (int i = 0; i < input.size(); i++)
	{
		detM[i] = input[i][0] * input[i][4] * input[i][8] - input[i][0] * input[i][7] * input[i][5] - input[i][3] * input[i][1] * input[i][8] + input[i][3] * input[i][7] * input[i][2] + input[i][6] * input[i][1] * input[i][5] - input[i][6] * input[i][4] * input[i][2];

		output[i][0][0] = input[i][4] * input[i][8] - input[i][7]*input[i][5];
		output[i][0][1] = -(input[i][3] * input[i][8] - input[i][6]*input[i][5]);
		output[i][0][2] = input[i][3] * input[i][7] - input[i][6]*input[i][4];

		output[i][1][0] = -(input[i][1] * input[i][8] - input[i][7]*input[i][2]);
		output[i][1][1] = input[i][0] * input[i][8] - input[i][6]*input[i][2];
		output[i][1][2] = -(input[i][0] * input[i][7] - input[i][6]*input[i][1]);

		output[i][2][0] = input[i][1] * input[i][5] - input[i][4]*input[i][2];
		output[i][2][1] = -(input[i][0] * input[i][5] - input[i][3]*input[i][2]);
		output[i][2][2] = input[i][0] * input[i][4] - input[i][3]*input[i][1];

		output[i][0][0] /= detM[i];
		output[i][0][1] /= detM[i];
		output[i][0][2] /= detM[i];
		output[i][1][0] /= detM[i];
		output[i][1][1] /= detM[i];
		output[i][1][2] /= detM[i];
		output[i][2][0] /= detM[i];
		output[i][2][1] /= detM[i];
		output[i][2][2] /= detM[i];
	}
}

int solve2ndOrder(float a, float b, float c, float& res0, float& res1)
{
	float disc = powf(b,2) - 4*a*c;
	if (disc < 0.0f)
		return 1;
	else
	{
		res0 = (-b + sqrtf(disc))/(2*a);
		res1 = (-b - sqrtf(disc))/(2*a);
		return 0;
	}
}
/*
int circleLineIntersect(Point p1, Point p2, float radius, Point center, Point& res0, Point& res1)
{
	float m;
	float e;
	float a,b,c;
	float r0,r1;

	if (fabs(p2.x - p1.x) < EPS)
	{
		// Line: x = const
		res0.x = res1.x = p1.x;

		a = 1;
		b = -2*center.y;
		c = powf(center.x,2) + powf(p1.x,2) + powf(center.y,2) - 2*center.x*p1.x - powf(radius,2);

		if (solve2ndOrder(a,b,c,r0,r1))
			return 1;

		res0.y = r0;
		res1.y = r1;
	}
	else
	{
		// Line: y = m*x + c
		m = (p2.y - p1.y)/(p2.x - p1.x);
		e = p1.y - m*p1.x;

		a = 1 + powf(m,2);
		b = 2*m*e - 2*center.x - 2*m*center.y;
		c = powf(center.x,2) + powf(e,2) + powf(center.y,2) - 2*center.y*e - powf(radius,2);

		if (solve2ndOrder(a,b,c,r0,r1))
			return 1;

		res0.x = r0;
		res1.x = r1;

		res0.y = m*res0.x + e;
		res1.y = m*res1.x + e;
	}

	return 0;
}

int circleLineIntersect_opt(Point p1, Point p2, float radius, Point center, Point& res0, Point& res1)
{
	float dx, dy, dr, D, disc;
	int ret;
	p1.x -= center.x;
	p2.x -= center.x;
	p1.y -= center.y;
	p2.y -= center.y;

	dx = p2.x - p1.x;
	dy = p2.y - p1.y;
	dr = sqrtf(dx*dx + dy*dy);
	D = p1.x*p2.y - p2.x*p1.y;
	disc = powf(radius,2)*powf(dr,2) - powf(D,2);


	//No intersection
	if (disc < 0.0f)
		return 0;
	else if (disc == 0.0f)	//Tangent (one intersection)
	{
		res0.x = D*dy/powf(dr,2);
		res0.y = -D*dx/powf(dr,2);
		res1 = res0;
		ret = 1;
	}
	else   //Two intersection
	{
		res0.x = sgn(dy)*dx*sqrtf(disc) + D*dy;
		res1.x = -sgn(dy)*dx*sqrtf(disc) + D*dy;
		//res1.x = -res0.x + D*dy;
		res0.x /= powf(dr,2);
		res1.x /= powf(dr,2);

		res0.y = fabs(dy)*sqrtf(disc) - D*dx;
		res1.y = -fabs(dy)*sqrtf(disc) - D*dx;
		//res1.y = -res0.y - D*dx;
		res0.y /= powf(dr,2);
		res1.y /= powf(dr,2);

		ret = 2;
	}

	res0.x += center.x;
	res1.x += center.x;
	res0.y += center.y;
	res1.y += center.y;
	return ret;
}
*/

void circleTransform(Point p0, Point p1, float radius, Point &center)
{
	//http://mathforum.org/library/drmath/view/53027.html
	float q, h;
	Point p3;

	p3.x = (p0.x + p1.x)*0.5f;
	p3.y = (p0.y + p1.y)*0.5f;

	q = sqrtf(powf((p0.x-p1.x),2) + powf((p0.y-p1.y),2));
	h = sqrtf(powf(radius,2) - powf(q/2,2));

	if (radius > 0.0f)
	{
		center.x = p3.x - h * (p0.y - p1.y) / q;
		center.y = p3.y - h * (p1.x - p0.x) / q;
	}
	else
	{
		center.x = p3.x + h * (p0.y - p1.y) / q;
		center.y = p3.y + h * (p1.x - p0.x) / q;
	}
}

void circleTransform_opt(Point p0, Point p1, float radius, Point &center)
{
	//http://mathforum.org/library/drmath/view/53027.html
	float h, q;
	Point p3, c0;

	p3.x = (p0.x + p1.x)*0.5f;
	p3.y = (p0.y + p1.y)*0.5f;

	q = sqrtf((p0.x-p1.x)*(p0.x-p1.x) + (p0.y-p1.y)*(p0.y-p1.y));
	h = sqrtf(radius*radius/(q*q) - 0.25f);

	c0.x = p3.x - h * (p0.y - p1.y);
	c0.y = p3.y - h * (p1.x - p0.x);

	//Get c0 center direction
	float cr = (c0.x-p1.x)*(p0.y-p1.y)-(p0.x-p1.x)*(c0.y-p1.y);

	if (radius*cr > 0)
	{
		center = c0; //c0 direction = radius direction
	}
	else
	{
		center.x = p3.x + h * (p0.y - p1.y);
		center.y = p3.y + h * (p1.x - p0.x);
	}
}

int circleCircleIntersect_opt(Point center1, Point center2, float radius1, float radius2, Point& res0, Point& res1)
{
	//http://paulbourke.net/geometry/circlesphere/
	float d, k, t1;
	Point diff;

	diff.x = center2.x - center1.x;
	diff.y = center2.y - center1.y;

	d = sqrtf(diff.x*diff.x + diff.y*diff.y);

	//No intersection
	if (d > radius1 + radius2)
		return 0;
	else if (d < fabs(radius1 - radius2))
		return 0;

	k = (d*d + radius1*radius1 - radius2*radius2)/(2.0f*d);

	t1 = sqrtf(radius1*radius1 - k*k)/d;

	res0.x = center1.x + diff.x*k/d + diff.y*t1;
	res0.y = center1.y + diff.y*k/d - diff.x*t1;

	res1.x = center1.x + diff.x*k/d - diff.y*t1;
	res1.y = center1.y + diff.y*k/d + diff.x*t1;

	return 2;
}

float getDistance(Point &p0, Point &p1)
{
	return sqrtf(powf(p0.x - p1.x,2) + powf(p0.y - p1.y,2));
}

float getDirection(Point &a, Point &b)
{
	return atan2f((b.y - a.y), (b.x - a.x));
}

void linearInterpolation(std::vector<float> &sourceX, std::vector<float> &sourceY, std::vector<float> &destX, std::vector<float> &destY)
{
	int sourceLength = sourceX.size();
	int destLength = destX.size();

	destY.resize(destLength);

	for (int i = 0; i < destLength; i++)
	{
		int j = 0;
		float c = 0.0;

		if (destX[i] > sourceX.back())
		{
			destY[i] = destY.back();
			continue;
		}

		//Linear interpolation
		while ((j < (sourceLength - 1)) && (destX[i] >= sourceX[j]))
			j++;
		
		c =	lerp(sourceY[j-1], sourceY[j], (destX[i] - sourceX[j-1]) / (sourceX[j] - sourceX[j-1]));
		
		destY[i] = c;
	}
}

void ProfileSave(std::string filename, std::vector<float> &data)
{
	std::ofstream file;
	file.open(filename);
	for (int i = 0; i < data.size(); i++)
		file << data[i] << std::endl;
	file.close();
}

void ProfileSave(std::string filename, std::vector<float> &t, std::vector<float> &data)
{
	if (t.size() == 0)
		return;
	if (data.size() == 0)
		return;
	if (t.size() != data.size())
		return;

	std::ofstream file;
	file.open(filename);
	for (int i = 0; i < data.size(); i++)
		file << t[i] << " " << data[i] << std::endl;
	file.close();
}

void ProfileSave(std::string filename, std::vector<Config> &path)
{
	std::ofstream file;
	file.open(filename);
	for (int i = 0; i < path.size(); i++)
		file << path[i].p.x << " " << path[i].p.y << std::endl;
	file.close();
}
