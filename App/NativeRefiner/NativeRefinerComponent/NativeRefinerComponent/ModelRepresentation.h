#pragma once

#include<vector>
#include<string>
#include<Eigen/Dense>

//use this for internal representation
namespace modelRep {
	//triangle
	struct Triangle
	{
		int t0;
		int t1;
		int t2;
	};
	class ModelRepresentation
	{
	public:
		ModelRepresentation();
		~ModelRepresentation();
		bool loadFile(std::string path);
		int nTriang;
	private:
		std::vector<Eigen::Vector3d> vertices;
		std::vector<Triangle> triangles;
	};
}
