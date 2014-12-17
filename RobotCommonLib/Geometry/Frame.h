#pragma once

#include "Polygon.h"

#include <string>
#include <iostream>
#include "ISerializable.h"
#include "tiny_obj_loader.h"

using namespace PathPlanner;

#define FRAME_TYPE_CODE		"frame"

class Frame : ISerializable
{
public:
	Frame() {}
	Frame(std::string file) { Load(file); }

	void Load(std::string file);

	/* Getters */
	vector<Polygon>& getObstacles() { return obstacles; }
	Polygon& getField() { return field; }

	/* Setters */
	void addObstacle(Polygon& poly) { obstacles.push_back(poly); }
	void setField(Polygon& field) { this->field = field; }

	~Frame() {}

	virtual Json::Value getJsonValue();
	virtual void setFromJson(Json::Value& value);
private:
	vector<Polygon> obstacles;
	Polygon field;
	Polygon robotBody;
	Config start;
	Config goal;

	Polygon ParseObjShape(tinyobj::shape_t &shp);
};

