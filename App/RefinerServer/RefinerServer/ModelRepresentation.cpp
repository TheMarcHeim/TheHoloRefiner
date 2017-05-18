#include "ModelRepresentation.h"
#include "stdafx.h"


ModelRepresentation::ModelRepresentation()
{
	nStepsDepthSearch = 21;
	stepSize = 0.005;
}

ModelRepresentation::~ModelRepresentation()
{
}

void ModelRepresentation::subDivide()
{
	igl::upsample(V, F, 1);
	computeNormals();
	nTriang = F.rows();
	nVert = V.rows();
	CorrectV = Eigen::MatrixXd(V.rows(), V.cols());
	CorrectV << V.col(2), V.col(0), V.col(1);
	nVertexObservations = Eigen::VectorXi::Zero(nVert);
	adjustmentScores = Eigen::MatrixXf::Zero(nStepsDepthSearch, nVert);
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

bool ModelRepresentation::saveFile(std::string path)
{
	//we do it now with libigl
	igl::writeOBJ(path.c_str(), V, F);
	return true;
}

void ModelRepresentation::computeNormals()
{
	igl::per_vertex_normals(V, F, VN);
}
