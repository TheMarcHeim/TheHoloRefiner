#include "ModelRepresentation.h"
#include "stdafx.h"


ModelRepresentation::ModelRepresentation()
{
	nStepsDepthSearch = 11;
	stepSize = 0.01; 
	refineTolerance = 0.001; // only adjust vertex if new one is this much better
	modelToWorldTransform << 0, 0, 1, 0.02,
							 1, 0, 0, 0.22,
							 0, 1, 0, -0.05; // sofa dataset
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
