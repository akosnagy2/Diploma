#include "path_planner_funcs.h"
#include <queue>       
#include "boost\geometry\geometries\segment.hpp"
#include "boost\geometry\geometries\point_xy.hpp"
#include "boost\geometry\geometry.hpp"
#define _USE_MATH_DEFINES
#include <math.h>
#include <stdio.h>
#include <float.h>

using namespace GEOM_FADE2D;

#define DEBUG

bool Scene::PathFinder(vector<int> &path)
{
	int N = roadmap_node.size();
	int start_node = roadmap_node.size() - 2;
	int goal_node = roadmap_node.size() - 1;
	bool success = false;

	vector<float> Q(N, numeric_limits<float>::infinity()), C(N);
	vector<int> Done(N), P(N);
	int Q_cnt = 0;

	//Insert start point first
	Done[start_node] = 1;
	C[start_node] = 0.0f;
	Q[start_node] = 0.0f;
	Q_cnt++;

	int i;
	while (Q_cnt > 0)
	{
		//Take the cheapest element from FIFO
		i = min_element(Q.begin(), Q.end()) - Q.begin();

		Done[i] = 2;  //Set its state to DEAD
		Q[i] = numeric_limits<float>::infinity(); //Disable the corresponding Q element
		Q_cnt--;

		if (i == goal_node)
		{
			//goal point reached
			success = true;
			break;
		}
		else
		{
			//visit all neighbours
			for (int j = 0; j < N; j++)
			{
				if (roadmap_adj(i, j) > 0)
				{
					if (Done[j] == 0)
					{
						//this is an unvisited neighbour put it into the queue
						Done[j] = 1;
						C[j] = C[i] + roadmap_adj(i, j);
						Q[j] = C[j];
						P[j] = i;
						Q_cnt++;
					}
					else if (Done[j] == 1)
					{
						//this is an already visited neighbour update its value if needed
						if (C[j] > C[i] + roadmap_adj(i, j))
						{
							C[j] = C[i] + roadmap_adj(i, j);
							Q[j] = C[j];
							P[j] = i;
						}
					}
				}
			}
		}
	}


	if (success) //Obtain the path
	{
		path.push_back(goal_node);

		int index = goal_node;
		while (index != start_node)
		{
			index = P[index];
			path.push_back(index);
		}

		//Path: Start -> Goal
		reverse(path.begin(), path.end());

		return true;
	}
	else
		return false;

}

template<typename T>
void printMat(ublas::matrix<T> mat)
{
	for (int i = 0; i < (int)mat.size1(); i++)
	{
		cout << i << ". sor:" << endl;
		for (int j = 0; j < (int)mat.size2(); j++)
		{		
			cout << mat(i, j) << " ";			
		}
		cout << endl;
	}
}

float corrigateAngle(float angle, uint16_t zero_to_2pi)
{
	float minLim, maxLim;
	float ret = angle;

	if (zero_to_2pi)
	{
		minLim = 0.0f;
		maxLim = 2*M_PI;
	}
	else
	{
		minLim = -M_PI;
		maxLim = M_PI;
	}

	while (ret > maxLim)
		ret -= (float)(2*M_PI);

	while (ret < minLim)
		ret += (float)(2*M_PI);

	return ret;
}

bool insideTriangle(Point a, Triangle t)
{
	//http://www.blackpawn.com/texts/pointinpoly/
	
	// Compute vectors        
	Point v0 = t.p[2] - t.p[0];
	Point v1 = t.p[1] - t.p[0];
	Point v2 = a - t.p[0];

	// Compute dot products
	float dot00 = Point::Dot(v0, v0);
	float dot01 = Point::Dot(v0, v1);
	float dot02 = Point::Dot(v0, v2);
	float dot11 = Point::Dot(v1, v1);
	float dot12 = Point::Dot(v1, v2);

	// Compute barycentric coordinates
	float invDenom = 1 / (dot00 * dot11 - dot01 * dot01);
	float u = (dot11 * dot02 - dot01 * dot12) * invDenom;
	float v = (dot00 * dot12 - dot01 * dot02) * invDenom;

	// Check if point is in triangle
	return ((u >= 0) && (v >= 0) && (u + v < 1));
}

Zone2* generateObjectZone(Polygon &obj, Fade_2D &dt)
{
	vector<Segment2> seg;

	seg.reserve(obj.ps.size());

	//Create FADE2D segments(edges)
	for (int i = 0; i < (int)(obj.ps.size() - 1); ++i)
	{
		Point2 p0(obj.ps[i].x, obj.ps[i].y);
		Point2 p1(obj.ps[i+1].x, obj.ps[i+1].y);
		seg.push_back(Segment2(p0, p1));
	}
	Point2 p0(obj.ps.back().x, obj.ps.back().y);
	Point2 p1(obj.ps.front().x, obj.ps.front().y);
	seg.push_back(Segment2(p0, p1));

	ConstraintGraph2* cstrs = dt.createConstraint(seg, CIS_IGNORE_DELAUNAY);
	Zone2* zone = dt.createZone(cstrs, ZL_OUTSIDE);

	return zone;
}

void Scene::Triangulate(Visualizer2 &vis)
{
	vector<Point2> fi;

	fi.reserve(field.ps.size());
	//Convert field to FADE2D Point2s and add to triangulation
	for (vector<Point>::iterator it = field.ps.begin(); it != field.ps.end(); ++it)
		fi.push_back(Point2(it->x, it->y));
	
	dt.insert(fi);

	//Generate zones for scene environment
	vector<Zone2*> zoneEnv;
	zoneEnv.reserve(envs.size());
	for (vector<Polygon>::iterator it = envs.begin(); it != envs.end(); ++it)
		zoneEnv.push_back(generateObjectZone((*it), dt));
		
	dt.applyConstraintsAndZones();

	Zone2* zone = zoneEnv.front();
	for (vector<Zone2*>::iterator it = (zoneEnv.begin() + 1); it != zoneEnv.end(); ++it)
		zone = zoneIntersection(zone, *it);

	//Get output triangles
	zone->getTriangles(tri);

	cells.clear();
	cells.reserve(tri.size());
	for (vector<Triangle2*>::iterator it = tri.begin(); it != tri.end(); ++it)
	{
		Point a( (float)((*it)->getCorner(0)->x()), (float)((*it)->getCorner(0)->y()) );
		Point b( (float)((*it)->getCorner(1)->x()), (float)((*it)->getCorner(1)->y()) );
		Point c( (float)((*it)->getCorner(2)->x()), (float)((*it)->getCorner(2)->y()) );
		cells.push_back(Triangle(a, b, c));
	}
}

//Minek kéne itt konvex hull, ha úgyis 3szögek?
int Scene::TriangularDecomp(bool visualize)
{
	Visualizer2 vis("scene.ps");

	//Triangulate using Fade2D
	Triangulate(vis);
	
	//Get adj matrix, roadmap_node
	int k = cells.size();
	adjN = ublas::zero_matrix<int>(k);

	Point i1, i2;
	int n = 1;
	for (int i = 0; i < k; i++)
	{
		for (int j = i + 1; j < k; j++)
		{
			if (cells[i].EdgeIntersect(cells[j], i1, i2))
			{
				adjN(i, j) = n;
				adjN(j, i) = n++;

				roadmap_node.push_back(Position((i1 + i2)/2.0, 0.0));
			}
		}
	}

	//Add robot start, target position to roadmap_node
	roadmap_node.push_back(robotStart);
	roadmap_node.push_back(robotGoal);


	Color colorFill(0.0,0.0,1.0,0.05,true); 
	Color colorLine(0.0,0.0,0.0,0.05, false); 
	Color colorLabel(0.0,0.0,0.0,0.1, false); 
	
	for(vector<Triangle2*>::iterator it = tri.begin(); it != tri.end(); ++it)
	{
		//Display cell
		//vis.addObject(**it, colorFill);
		vis.addObject(**it, colorLine);
			
		//Display cell label
		double x,y;
		x = (**it).getCorner(0)->x() + (**it).getCorner(1)->x() + (**it).getCorner(2)->x();
		x /= 3.0;
		y = (**it).getCorner(0)->y() + (**it).getCorner(1)->y() + (**it).getCorner(2)->y();
		y /= 3.0;
		vis.addObject(Label(Point2(x,y), "Cell " + std::to_string(it - tri.begin()) + ". "), colorLabel);
	}


	for(vector<Position>::iterator it = roadmap_node.begin(); it != roadmap_node.end(); ++it)
	{
		Point2 p(it->p.x, it->p.y);
		vis.addObject(Label(p, "Node " + std::to_string(it - roadmap_node.begin()) + ". "), Color(0.0,0.0,0.0,0.1, false));
	}


	Point2 p0, p1, p2;
	//Build adjacency matrix
	roadmap_adj = ublas::zero_matrix<float>(roadmap_node.size());
	vector<int> cb_i;
	for (int i = 0; i < k; i++)
	{
		cb_i.clear();

		for (int j = 0; j < k; j++)
		{
			if (adjN(i, j) != 0)
			{
				cb_i.push_back(adjN(i,j) - 1);
			}
		}

		if (cb_i.size() == 3)
		{
			roadmap_adj(cb_i[0], cb_i[1]) = Position::Distance(roadmap_node[cb_i[0]], roadmap_node[cb_i[1]]); 
			roadmap_adj(cb_i[0], cb_i[2]) = Position::Distance(roadmap_node[cb_i[0]], roadmap_node[cb_i[2]]); 
			roadmap_adj(cb_i[1], cb_i[2]) = Position::Distance(roadmap_node[cb_i[1]], roadmap_node[cb_i[2]]); 

			roadmap_adj(cb_i[1], cb_i[0]) = roadmap_adj(cb_i[0], cb_i[1]);
			roadmap_adj(cb_i[2], cb_i[0]) = roadmap_adj(cb_i[0], cb_i[2]);
			roadmap_adj(cb_i[2], cb_i[1]) = roadmap_adj(cb_i[1], cb_i[2]);

			p0.set(roadmap_node[cb_i[0]].p.x, roadmap_node[cb_i[0]].p.y);
			p1.set(roadmap_node[cb_i[1]].p.x, roadmap_node[cb_i[1]].p.y);
			p2.set(roadmap_node[cb_i[2]].p.x, roadmap_node[cb_i[2]].p.y);
			Segment2 s0(p0, p1), s1(p0, p2), s2(p1, p2);
			vis.addObject(s0, Color(0.0,0.0,0.0,0.1, false));
			vis.addObject(s1, Color(0.0,0.0,0.0,0.1, false));
			vis.addObject(s2, Color(0.0,0.0,0.0,0.1, false));

		}
		else if (cb_i.size() == 2)
		{
			roadmap_adj(cb_i[0], cb_i[1]) = Position::Distance(roadmap_node[cb_i[0]], roadmap_node[cb_i[1]]); 

			roadmap_adj(cb_i[1], cb_i[0]) = roadmap_adj(cb_i[0], cb_i[1]);

			p0.set(roadmap_node[cb_i[0]].p.x, roadmap_node[cb_i[0]].p.y);
			p1.set(roadmap_node[cb_i[1]].p.x, roadmap_node[cb_i[1]].p.y);
			Segment2 s0(p0, p1);
			vis.addObject(s0, Color(0.0,0.0,0.0,0.1, false));
		}
	}

	vis.writeFile();

	//Determine containing triangles for start and goal points
	int start_index, goal_index;
	for(vector<Triangle>::iterator it = cells.begin(); it != cells.end(); ++it)
	{
		if (insideTriangle(robotStart.p, *it))
			start_index = it - cells.begin();

		if (insideTriangle(robotGoal.p, *it))
			goal_index = it - cells.begin();		
	}

	//Insert edges into the graph connecting start and goal points to the existing vertices belonging to the containing triangles
	int start_node = roadmap_node.size() - 2;
	int goal_node = roadmap_node.size() - 1;
	for (int i = 0; i < (int)adjN.size2(); i++)
	{
		if (adjN(start_index, i) != 0)
		{
			roadmap_adj(start_node, adjN(start_index, i) - 1) = Position::Distance(roadmap_node[start_node], roadmap_node[adjN(start_index, i) - 1]); 

			roadmap_adj(adjN(start_index, i) - 1, start_node) = roadmap_adj(start_node, adjN(start_index, i) - 1); 
		}

		if (adjN(goal_index, i) != 0)
		{
			roadmap_adj(goal_node, adjN(goal_index, i) - 1) = Position::Distance(roadmap_node[goal_node], roadmap_node[adjN(goal_index, i) - 1]); 

			roadmap_adj(adjN(goal_index, i) - 1, goal_node) = roadmap_adj(goal_node, adjN(goal_index, i) - 1); 
		}
	}

	//Check clearance of edges in the roadmap. If one edge goes too close to an obstacle, remove it from the graph
	vector<Polygon> obstacles = envs;
	obstacles.push_back(field);
	float robotWidth = GetRobotWidth();
	for (int i = 0; i < (int)roadmap_adj.size1() - 2; i++)
	{
		for (int j = 0; j < (int)roadmap_adj.size2() - 2; j++)
		{
			if (roadmap_adj(i, j) != 0.0f)
			{
				boost::geometry::model::d2::point_xy<float> p0(roadmap_node[i].p.x, roadmap_node[i].p.y);
				boost::geometry::model::d2::point_xy<float> p1(roadmap_node[j].p.x, roadmap_node[j].p.y);

				boost::geometry::model::segment<boost::geometry::model::d2::point_xy<float>> seg0(p0, p1);			

				for (int k = 0; k < (int)obstacles.size(); k++)
				{
					int size = obstacles[k].ps.size();
					for (int l = 0; l < size; l++)
					{
						boost::geometry::model::d2::point_xy<float> p2(obstacles[k].ps[l].x, obstacles[k].ps[l].y);
						boost::geometry::model::d2::point_xy<float> p3(obstacles[k].ps[((l + 1) % size)].x, obstacles[k].ps[((l + 1) % size)].y);

						boost::geometry::model::segment<boost::geometry::model::d2::point_xy<float>> seg1(p2, p3);	

						float clearance = (float)boost::geometry::distance(seg0, seg1);

						/*  If the edge clearance is not greater than the robot
							"width" (the maximum absolute y-coordinate of the 
							robot shape in the local frame) then the edge is rejected
						*/
						if (clearance <= robotWidth)
						{
							roadmap_adj(i, j) = 0.0f;
							roadmap_adj(j, i) = 0.0f;
						}
					}
				}
			}
		}
	}
 
	vector<int> pathIndex;
	PathFinder(pathIndex);
	
	prepath.reserve(pathIndex.size());
	for(vector<int>::iterator it = pathIndex.begin(); it != pathIndex.end(); ++it)
	{
		prepath.push_back(roadmap_node[(*it)]);
	}

	for(vector<Position>::iterator it = prepath.begin() + 1; it != prepath.end() - 1;)
	{
		float next_seg_angle = atan2f((*(it+1)).p.y - (*(it)).p.y, (*(it+1)).p.x - (*(it)).p.x);
		float prev_seg_angle = atan2f((*(it)).p.y - (*(it-1)).p.y, (*(it)).p.x - (*(it-1)).p.x);
		
		if ((abs(corrigateAngle(next_seg_angle - prev_seg_angle, 0))) > numeric_limits<float>::epsilon())
		{
			(*it).phi = next_seg_angle;
			++it;
		}
		else
		{
			it = prepath.erase(it);
		}

	}

	//Print debug
	#ifdef DEBUG
	printMat<int>(adjN);
	cout << endl;
	printMat<float>(roadmap_adj);
	cout << endl;
	#endif

	return 0;
}