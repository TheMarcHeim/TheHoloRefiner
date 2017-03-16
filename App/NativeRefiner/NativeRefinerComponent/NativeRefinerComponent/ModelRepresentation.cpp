#include "pch.h"
#include "ModelRepresentation.h"


modelRep::ModelRepresentation::ModelRepresentation()
{
}

bool modelRep::ModelRepresentation::loadFile(std::string path)
{
	std::vector<tinyobj::shape_t> shapes;
	std::vector<tinyobj::material_t> materials;
	std::string mtl_basedir = "";
	tinyobj::attrib_t attrib;
	std::string err;
	bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &err, path.c_str(), nullptr, true);

	return true;
}
