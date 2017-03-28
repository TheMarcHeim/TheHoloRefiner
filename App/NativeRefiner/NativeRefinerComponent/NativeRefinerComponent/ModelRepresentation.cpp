#include "pch.h"
#include "ModelRepresentation.h"
#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"
#include <iostream>




modelRep::ModelRepresentation::ModelRepresentation()
{
}

modelRep::ModelRepresentation::~ModelRepresentation()
{
}

bool modelRep::ModelRepresentation::loadFile(std::string path)
{
	std::cout << "test";
	//load object data
	std::vector<tinyobj::shape_t> shapes;
	std::vector<tinyobj::material_t> materials;
	std::string mtl_basedir = "";
	tinyobj::attrib_t attrib;
	std::string err;


	bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &err, path.c_str(), nullptr, true);
	
	//if we do not have exactly one shape exit
	if (shapes.size() != 1)return false;

	//load vertices
	int i;
	for (i = 0; i * 3 < attrib.vertices.size(); i++) {
		vertices.push_back(Eigen::Vector3d(attrib.vertices[i * 3], attrib.vertices[i * 3 + 1], attrib.vertices[i * 3 + 2]));
	}
	//load triangles(have to be triangles)
	//we have a fully linked model
	std::vector<tinyobj::index_t>* triangleIndices = &shapes[0].mesh.indices;
	for (i = 0; i * 3 < triangleIndices->size(); i++) {
		Triangle newT;
		newT.t0 = (*triangleIndices)[3 * i].vertex_index;
		newT.t1 = (*triangleIndices)[3 * i + 1].vertex_index;
		newT.t2 = (*triangleIndices)[3 * i + 2].vertex_index;
		triangles.push_back(newT);
	}
	
	nTriang = triangles.size();

	return true;
}
