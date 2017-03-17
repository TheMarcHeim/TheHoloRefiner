#pragma once

#include<vector>
#include<string>
#include<Eigen/Dense>

//use this for internal representation
namespace modelRep {
	//triangle
	struct Triangle
	{
		Eigen::Vector3d* t0;
		Eigen::Vector3d* t1;
		Eigen::Vector3d* t2;
	};
	class ModelRepresentation
	{
	public:
		ModelRepresentation();
		~ModelRepresentation();
		bool loadFile(std::string path);
		int nTriang;
	private:
		Eigen::Vector3d* vertices;
		std::vector<Triangle> triangles;
	};
}
