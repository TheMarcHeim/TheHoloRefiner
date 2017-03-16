#pragma once

#include<vector>
#include<string>
#include "tiny_obj_loader.h"

//use this for internal representation
namespace modelRep {
	//triangle
	struct Triangle
	{
		int t0, t1, t2;
	};
	ref class ModelRepresentation sealed
	{
	public:
		ModelRepresentation();

	private:
		std::vector<Eigen::Vector2d> vertices;
		std::vector<Triangle> triangles;
		bool loadFile(std::string path);
	};
}
