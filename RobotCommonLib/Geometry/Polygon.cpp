#include "Polygon.h"
#include <boost/numeric/ublas/matrix_sparse.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/lu.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <limits>

using namespace boost::numeric;
using namespace PathPlanner;

Polygon::Polygon(float width, float heigth)
{
	ps.push_back(Point(0, 0));
	ps.push_back(Point(width, 0));
	ps.push_back(Point(width, heigth));
	ps.push_back(Point(0, heigth));
}

template<class T>
bool InvertMatrix(const ublas::matrix<T>& input, ublas::matrix<T>& inverse)
{
	typedef ublas::permutation_matrix<size_t> pmatrix;

	// create a working copy of the input
	ublas::matrix<T> A(input);

	// create a permutation matrix for the LU-factorization
	pmatrix pm(A.size1());

	// perform LU-factorization
	int res = ublas::lu_factorize(A, pm);
	if (res != 0)
		return false;

	// create identity matrix of "inverse"
	inverse.assign(ublas::identity_matrix<T> (A.size1()));

	// backsubstitute to get the inverse
	ublas::lu_substitute(A, pm, inverse);

	return true;
}

Polygon Polygon::TransformToWorld(Config q)
{
	Polygon polyWorld;

	ublas::matrix<float> T = ublas::identity_matrix<float>(3);
	T(0,0) = cosf(q.phi);
	T(0,1) = -sinf(q.phi);
	T(0,2) = q.p.x;

	T(1,0) = sinf(q.phi);
	T(1,1) = cosf(q.phi);
	T(1,2) = q.p.y;
	ublas::vector<float> col(3);

	for (int i = 0; i < (int)this->ps.size(); i++)
	{
		col[0] = this->ps[i].x;
		col[1] = this->ps[i].y;
		col[2] = 1.0;

		ublas::vector<float> prod = ublas::prod(T, col);
		polyWorld.AddPoint(Point(prod[0], prod[1]));
	}

	return polyWorld;
}

Polygon Polygon::TransformToLocal(Config q)
{
	Polygon polyWorld;

	ublas::matrix<float> T = ublas::identity_matrix<float>(3);
	T(0,0) = cosf(q.phi);
	T(0,1) = -sinf(q.phi);
	T(0,2) = q.p.x;

	T(1,0) = sinf(q.phi);
	T(1,1) = cosf(q.phi);
	T(1,2) = q.p.y;

	ublas::matrix<float> Tinv =  ublas::identity_matrix<float>(3);

	InvertMatrix<float>(T, Tinv);
	ublas::vector<float> col(3);
	for (int i = 0; i < (int)this->ps.size(); i++)
	{
		col[0] = this->ps[i].x;
		col[1] = this->ps[i].y;
		col[2] = 1.0;

		ublas::vector<float> prod = ublas::prod(Tinv, col);
		polyWorld.AddPoint(Point(prod[0], prod[1]));
	}

	return polyWorld;
}

Json::Value Polygon::getJsonValue()
{
	Json::Value val;
	for(auto &p : ps)
	{
		val["points"].append(p.getJsonValue());
	}
	val["closed"] = true;
	return val;
}

void Polygon::setFromJson(Json::Value& value)
{
	ps.clear();
	for(auto &jp : value["points"])
	{
		Point p;
		p.setFromJson(jp);
		ps.push_back(p);
	}
}

float Polygon::getWidth()
{
	float minx = numeric_limits<float>::infinity();
	float maxx = 0.0f;

	for(auto &p : ps)
	{
		if(p.x < minx)
			minx = p.x;
		if(p.x > maxx)
			maxx = p.x;
	}

	return maxx - minx;
}

float Polygon::getHeight()
{
	float miny = numeric_limits<float>::infinity();
	float maxy = 0.0f;

	for(auto &p : ps)
	{
		if(p.y < miny)
			miny = p.y;
		if(p.y > maxy)
			maxy = p.y;
	}

	return maxy - miny;
}