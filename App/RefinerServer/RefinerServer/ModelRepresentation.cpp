#include "ModelRepresentation.h"
#include "stdafx.h"


ModelRepresentation::ModelRepresentation()
{
	nStepsDepthSearch = 11;
}

ModelRepresentation::~ModelRepresentation()
{
}

void ModelRepresentation::subDivide()
{
	igl::upsample(V, F, 1);
}

bool ModelRepresentation::loadFile(std::string path)
{
	//we do it now with libigl
	igl::readOBJ(path.c_str(), V, F);
	computeNormals();
	nTriang = F.rows();
	nVert = V.rows();
	CorrectV = Eigen::MatrixXd(V.rows(), V.cols());
	CorrectV << V.col(2), V.col(0), V.col(1);
	nVertexObservations = Eigen::VectorXi::Zero(nVert);
	adjustmentScores = Eigen::MatrixXf::Zero(nStepsDepthSearch, nVert);
	return true;
}

void ModelRepresentation::computeNormals()
{
	igl::per_vertex_normals(V, F, VN);
}
