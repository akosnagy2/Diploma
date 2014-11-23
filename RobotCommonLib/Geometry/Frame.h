#pragma once

#include "Polygon.h"

#include <string>
#include <iostream>
#include "ISerializable.h"
#include "tiny_obj_loader.h"

using namespace PathPlanner;

#define FRAME_TYPE_CODE		"frame"

class Frame : public ISerializable
{
public:
	Frame() {}
	Frame(std::string file) { Load(file); }

	void Load(std::string file);

	/* Getters */
	vector<PathPlanner::Polygon>& getObstacles() { return obstacles; }
	PathPlanner::Polygon& getField() { return field; }
	PathPlanner::Polygon& getRobotShape() { return robotBody; }
	PathPlanner::Config& getStart() { return start; }
	PathPlanner::Config& getGoal() { return goal; }

	/* Setters */
	void addObstacle(PathPlanner::Polygon& poly) { obstacles.push_back(poly); }
	void setField(PathPlanner::Polygon& field) { this->field = field; }

	~Frame() {}

	virtual Json::Value getJsonValue();
	virtual void setFromJson(Json::Value& value);
private:
	vector<PathPlanner::Polygon> obstacles;
	PathPlanner::Polygon field;
	PathPlanner::Polygon robotBody;
	Config start;
	Config goal;

	static PathPlanner::Polygon ParseObjShape(tinyobj::shape_t &shp);
};

