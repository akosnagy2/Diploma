#ifndef PATH_PLANNER_FUNCS_H_
#define PATH_PLANNER_FUNCS_H_

#include <vector>
#include <boost/numeric/ublas/matrix_sparse.hpp>
#include <Fade_2D.h>
//#include <boost/numeric/ublas/io.hpp>

using namespace std;
using namespace boost::numeric;
using namespace GEOM_FADE2D;

struct Point
{
	Point()
	{}
	Point(float x, float y) 
	{
		this->x = x;
		this->y = y;
	}
	bool operator==(Point &b)
	{
		if ((this->x == b.x) && (this->y == b.y))
			return true;
		else
			return false;
	}
	Point operator+(Point &b)
	{
		Point a(this->x + b.x, this->y + b.y);
		return a;
	}
	Point operator-(Point &b)
	{
		Point a(this->x - b.x, this->y - b.y);
		return a;
	}
	Point operator/(float div)
	{
		Point a(this->x/div, this->y/div);
		return a;
	}
	static float Distance(Point &a, Point &b)
	{
		return sqrtf((a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y));
	}
	static float Dot(Point &a, Point &b)
	{
		return a.x*b.x + a.y*b.y;
	}
	float x;	// [mm]
	float y;	// [mm]
};

struct Config
{
	Config()
	{}
	Config(float x, float y, float phi)
	{
		p.x = x;
		p.y = y;
		this->phi = phi;
	}
	Config(Point p, float phi)
	{
		this->p = p;
		this->phi = phi;
	}
	static float Distance(Config &a, Config &b)
	{
		return Point::Distance(a.p, b.p);
	}
	Point p;
	float phi;
};


struct Triangle
{
	Triangle(Point a, Point b, Point c)
	{
		this->p[0] = a;
		this->p[1] = b;
		this->p[2] = c;
	}
	Triangle()
	{}
	bool EdgeIntersect(Triangle &b, Point &i1, Point &i2)
	{
		bool p = false;
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				if (this->p[i] == b.p[j])								
				{
					if (p)
					{
						i2 = this->p[i];
						return true;
					}
					else 
					{
						i1 = this->p[i];
						p = true;
					}
				}
			}
		}
		return false;
	}
	Point p[3];
};

struct Polygon
{
	void AddPoint(Point p)
	{
		ps.push_back(p);
	}
	Polygon()
	{}
	vector<Point> ps;
};

class Scene
{
public:
	Scene(string vis_name)
	{
	}
	~Scene()
	{

	}
	void AddField(Polygon fi)
	{
		field = fi;
	}
	void AddEnv(Polygon env)
	{
		envs.push_back(env);
	}
	void SetStartConfig(Config pos)
	{
		robotStart = pos;
	}
	void SetGoalConfig(Config pos)
	{
		robotGoal = pos;
	}
	void SetRobotShape(Polygon shp)
	{
		robotShape = shp;
	}
	float GetRobotWidth()
	{
		vector<float> shp_y;

		//Get local frame y abs values
		for (int i = 0; i < (int)robotShape.ps.size(); i++)
			shp_y.push_back(fabs(robotShape.ps[i].y));

		//Return max 
		return (*max_element(shp_y.begin(), shp_y.end()));
	}
	bool PrePlanner();
	bool RTRPlanner();
	void DrawPrePath();
private:
	bool PathFinder();
	void Triangulate();
private:
	vector<Polygon> envs;
	Polygon field;
	vector<Triangle> cells;

	ublas::mapped_matrix<int> adjN;	
	ublas::mapped_matrix<float> roadmap_adj;
	vector<Config> roadmap_node;
	vector<Config> prepath;

	Config robotStart;
	Config robotGoal;
	Polygon robotShape;
};


template<typename T> void printMat(ublas::matrix<T> mat);
float corrigateAngle(float angle, uint16_t zero_to_2pi);
bool insideTriangle(Point a, Triangle t);
Zone2* generateObjectZone(Polygon &obj, Fade_2D &dt);

#endif /* PATH_PLANNER_FUNCS_H_ */