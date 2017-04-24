#include "pch.h"
#include "ModelRepresentation.h"
#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"
#include <iostream>
#include <igl/upsample.h>
#include <igl/per_vertex_normals.h>
#include <igl/per_face_normals.h>
#include <igl/per_corner_normals.h>


modelRep::ModelRepresentation::ModelRepresentation()
{
}

modelRep::ModelRepresentation::~ModelRepresentation()
{
}

void modelRep::ModelRepresentation::subDivide()
{
	igl::upsample(V, F, 1);
}

float modelRep::ModelRepresentation::AdjustVertex(imageRep::ImageRepresentation& I, imageRep::ImageRepresentation& J, int vertex)
{

	/* TO DO
	
	To be answered:
	What do we iterate over, i.e. given set of images, which image pairs do we pick? Given a vertex v, there is a set I_v of images i 
	in which vertex n is visible (-> normal at v times vector from camera i to vertex v greater than zero and assuming convex shapes). 
	
	One, albeit quite expensive, approach could be:
	
		- For every image pair in I_v, calculate the distance by which vertex v is to be adjusted, then take the average 
		and apply this for the refinement step (i.e. let the different camera pairs vote. Here, we could also choose different weights for 
		the votes depending on the baseline of the respective image pairs, the viewing angle etc.)

	We could also only consider "begnine" image pairs, i.e. those which are most sensitive to a change in the position of the respective vertex and hence yield
	an accurate refinement distance. (i.e large angle between viewing direction and large baseline between the images). 


	Different tasks:
		- Subdivision
		- Calculate normal at a vertex from the normals of the adjacent triangles (area weighted average?)
		- Vertex <- cameras (inside image frame, not occluded)
		- Forming image pairs fulfilling a criterion yet to be defined (cf above)
		- Given two image patches, calculate the distance by which the vertex will be shifted
		- Move a vertex given the desired distance and the vertex normal


	Pseudo:...

	For numIt iterations do:
		For every vertex (i=0:nVert-1) of the object, 
			...
			...
	
	
	*/



	//I.setPositions(V);
	//J.setPositions(V);
	
}


void Refine(int numIt, double stepSize) {

	//subdivide
	//some logic to refine mesh via AdjustVertex

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


	//bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &err, path.c_str(), nullptr, true);
	/*
	//if we do not have exactly one shape exit
	if (shapes.size() != 1)return false;

	//load vertices
	int i;
	for (i = 0; i * 3 < attrib.vertices.size(); i++) {
	//append vertices
	vertices<<Eigen::Vector3f(attrib.vertices[i * 3], attrib.vertices[i * 3 + 1], attrib.vertices[i * 3 + 2]);
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
	nVert = vertices.cols();
	*/

	//we do it now with libigl
	igl::readOBJ(path.c_str(), V, F);
	computeNormals();
	nTriang = F.rows();
	nVert = V.rows();

	return true;
}

void modelRep::ModelRepresentation::computeNormals()
{
	igl::per_vertex_normals(V, F, VN);
}
