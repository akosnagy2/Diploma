#include "..\Geometry\Common.h"   
#include "boost\geometry\geometries\segment.hpp"
#include "boost\geometry\geometries\point_xy.hpp"
#include "boost\geometry\geometry.hpp"
#define _USE_MATH_DEFINES
#include <math.h>
#include "Scene.h"
#include "..\Geometry\Triangle.h"

using namespace boost::geometry::model;
using namespace PathPlanner;

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

void Scene::DrawPrePath()
{
}

void Scene::Triangulate()
{
	Fade_2D dt;
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
	vector<Triangle2*> tri;
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

bool Scene::PathFinder()
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
		prepath.push_back(roadmap_node[goal_node]);

		int index = goal_node;
		while (index != start_node)
		{
			index = P[index];
			prepath.push_back(roadmap_node[index]);
		}

		//Path: Start -> Goal
		reverse(prepath.begin(), prepath.end());

		return true;
	}
	else
		return false;

}

bool Scene::PrePlanner()
{
	//Triangulate using Fade2D
	Triangulate();
	
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

				roadmap_node.push_back(Config((i1 + i2)/2.0, 0.0));
			}
		}
	}

	//Add robot start, target Config to roadmap_node
	roadmap_node.push_back(robotStart);
	roadmap_node.push_back(robotGoal);

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
			roadmap_adj(cb_i[0], cb_i[1]) = Config::Distance(roadmap_node[cb_i[0]], roadmap_node[cb_i[1]]); 
			roadmap_adj(cb_i[0], cb_i[2]) = Config::Distance(roadmap_node[cb_i[0]], roadmap_node[cb_i[2]]); 
			roadmap_adj(cb_i[1], cb_i[2]) = Config::Distance(roadmap_node[cb_i[1]], roadmap_node[cb_i[2]]); 

			roadmap_adj(cb_i[1], cb_i[0]) = roadmap_adj(cb_i[0], cb_i[1]);
			roadmap_adj(cb_i[2], cb_i[0]) = roadmap_adj(cb_i[0], cb_i[2]);
			roadmap_adj(cb_i[2], cb_i[1]) = roadmap_adj(cb_i[1], cb_i[2]);
		}
		else if (cb_i.size() == 2)
		{
			roadmap_adj(cb_i[0], cb_i[1]) = Config::Distance(roadmap_node[cb_i[0]], roadmap_node[cb_i[1]]); 

			roadmap_adj(cb_i[1], cb_i[0]) = roadmap_adj(cb_i[0], cb_i[1]);
		}
	}

	//Determine containing triangles for start and goal points
	int start_index, goal_index;
	for(vector<Triangle>::iterator it = cells.begin(); it != cells.end(); ++it)
	{
		if (it->PointInside(robotStart.p))
			start_index = it - cells.begin();

		if (it->PointInside(robotGoal.p))
			goal_index = it - cells.begin();		
	}

	//Insert edges into the graph connecting start and goal points to the existing vertices belonging to the containing triangles
	int start_node = roadmap_node.size() - 2;
	int goal_node = roadmap_node.size() - 1;
	for (int i = 0; i < (int)adjN.size2(); i++)
	{
		if (adjN(start_index, i) != 0)
		{
			roadmap_adj(start_node, adjN(start_index, i) - 1) = Config::Distance(roadmap_node[start_node], roadmap_node[adjN(start_index, i) - 1]); 

			roadmap_adj(adjN(start_index, i) - 1, start_node) = roadmap_adj(start_node, adjN(start_index, i) - 1); 
		}

		if (adjN(goal_index, i) != 0)
		{
			roadmap_adj(goal_node, adjN(goal_index, i) - 1) = Config::Distance(roadmap_node[goal_node], roadmap_node[adjN(goal_index, i) - 1]); 

			roadmap_adj(adjN(goal_index, i) - 1, goal_node) = roadmap_adj(goal_node, adjN(goal_index, i) - 1); 
		}
	}

	//Check clearance of edges in the roadmap. If one edge goes too close to an obstacle, remove it from the graph
	float robotWidth = GetRobotWidth();
	for (int i = 0; i < (int)roadmap_adj.size1() - 2; i++)
	{
		for (int j = 0; j < (int)roadmap_adj.size2() - 2; j++)
		{
			if (roadmap_adj(i, j) != 0.0f)
			{
				d2::point_xy<float> p0(roadmap_node[i].p.x, roadmap_node[i].p.y);
				d2::point_xy<float> p1(roadmap_node[j].p.x, roadmap_node[j].p.y);

				segment<d2::point_xy<float>> seg0(p0, p1);			

				for (int k = 0; k < (int)envsx.size(); k++)
				{
					int size = envsx[k].ps.size();
					for (int l = 0; l < size; l++)
					{
						d2::point_xy<float> p2(envsx[k].ps[l].x, envsx[k].ps[l].y);
						d2::point_xy<float> p3(envsx[k].ps[((l + 1) % size)].x, envsx[k].ps[((l + 1) % size)].y);

						segment<d2::point_xy<float>> seg1(p2, p3);						

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
 
	//Search for a shortest path in the roadmap
	if (!PathFinder())
		return false;

	//Get orientation for the path
	for(vector<Config>::iterator it = prepath.begin() + 1; it != prepath.end() - 1;)
	{
		float next_seg_angle = atan2f((*(it+1)).p.y - (*(it)).p.y, (*(it+1)).p.x - (*(it)).p.x);
		float prev_seg_angle = atan2f((*(it)).p.y - (*(it-1)).p.y, (*(it)).p.x - (*(it-1)).p.x);
		
		if ((abs(Angle::Corrigate(next_seg_angle - prev_seg_angle))) > numeric_limits<float>::epsilon())
		{
			(*it).phi = next_seg_angle;
			++it;
		}
		else
		{ //Remove points with same orientation
			it = prepath.erase(it);
		}
	}

	//Workaround in the case if the whole path consists of only one segment:
	//Insert one intermediate configuration at the half of the segment, 
	//pointing to the goal Config
	if (prepath.size() == 2)
	{
		Config inter;
		inter.p = (prepath.front().p + prepath.back().p)/2;
		inter.phi = atan2f(prepath.back().p.y - prepath.front().p.y, prepath.back().p.x - prepath.front().p.x);
		prepath.insert(prepath.begin(), inter);
	}

	return true;
}