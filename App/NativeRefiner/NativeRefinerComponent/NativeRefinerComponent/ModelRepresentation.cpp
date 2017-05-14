#include "pch.h"
#include "ModelRepresentation.h"
#include <igl/upsample.h>
#include <igl/per_vertex_normals.h>
#include <igl/per_face_normals.h>
#include <igl/per_corner_normals.h>


modelRep::ModelRepresentation::ModelRepresentation()
{
	nStepsDepthSearch = 11;
}

modelRep::ModelRepresentation::~ModelRepresentation()
{
}

void modelRep::ModelRepresentation::subDivide()
{
	igl::upsample(V, F, 1);
}

bool modelRep::ModelRepresentation::loadFile(std::string path)
{
	//we do it now with libigl
	igl::readOBJ(path.c_str(), V, F);
	computeNormals();
	nTriang = F.rows();
	nVert = V.rows();
	nVertexObservations = Eigen::VectorXi::Zero(nVert);
	adjustmentScores = Eigen::MatrixXf::Zero(nStepsDepthSearch, nVert);
	return true;
}

void modelRep::ModelRepresentation::computeNormals()
{
	igl::per_vertex_normals(V, F, VN);
}
