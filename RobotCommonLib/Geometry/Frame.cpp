#include "Frame.h"

#include <json/json.h>
#include <string>

void Frame::Load(std::string file)
{
	bool ret = true;
	vector<tinyobj::shape_t> shapes;
	vector<tinyobj::material_t> materials;

	//Load obj file by TinyObjLoader
	tinyobj::LoadObj(shapes, materials, file.c_str());

	for(auto& elem : shapes)
	{
		if(elem.name == "Robot")
			robotBody = ParseObjShape(elem);
		else if(elem.name.find("Obstacle") != string::npos)
			obstacles.push_back(ParseObjShape(elem)); //Load ObstacleX shape
		else if(elem.name == "StartConfig")
			start = Config(elem.mesh.positions[0], elem.mesh.positions[1], elem.mesh.positions[2]); //Start Config
		else if(elem.name == "GoalConfig")
			goal = Config(elem.mesh.positions[0], elem.mesh.positions[1], elem.mesh.positions[2]); //Goal Config
		else if(elem.name == "Field")
			field = Polygon(elem.mesh.positions[6], elem.mesh.positions[4]); //Load field
	}
}

Json::Value Frame::getJsonValue()
{
	Json::Value val;

	val["type"] = FRAME_TYPE_CODE;
	val["boundary"]["width"] = field.getWidth();
	val["boundary"]["height"] = field.getHeight();
	for(auto &o : obstacles)
	{
		val["obstacles"].append(o.getJsonValue());
	}

	val["start"] = start.getJsonValue();
	val["goal"] = goal.getJsonValue();
	val["robot"] = robotBody.getJsonValue();

	return val;
}

void Frame::setFromJson(Json::Value& value)
{
	if(value["type"].asString().compare(FRAME_TYPE_CODE))
	{
		field = Polygon(value["boundary"]["width"].asFloat(), value["boundary"]["width"].asFloat());
		for(auto &jo : value["obstacles"])
		{
			Polygon p;
			p.setFromJson(jo);
			obstacles.push_back(p);
		}
		start.setFromJson(value["start"]);
		goal.setFromJson(value["goal"]);
		robotBody.setFromJson(value["robot"]);
	}
}

Polygon ParseObjShape(tinyobj::shape_t &shp)
{
	PathPlanner::Polygon poly;

	for(int i = 0; i < (int) shp.mesh.positions.size(); i += 3)
		poly.AddPoint(PathPlanner::Point(shp.mesh.positions[i], shp.mesh.positions[i + 1])); //Load x, y coord. Skip z coord.

	return poly;
}