#include "ModelRepresentation.h"
#include "stdafx.h"
#include <iostream>


ModelRepresentation::ModelRepresentation()
{

	nStepsDepthSearch = 51;
	stepSize = 0.0005; 
	refineTolerance = 0.0001; // only adjust vertex if new one is this much better
	//modelToWorldTransform << 0, 0, 1, 0.02,
	//						 1, 0, 0, 0.22,
	//						 0, 1, 0, -0.05,
	//						 0, 0, 0, 1; // sofa dataset

	// happy birthday dataset capture 1
	modelToWorldTransform << -1, 0, 0, -1.2379,
							  0, 0, 1, 1.3061,
							  0, 1, 0, -0.02,
								  0, 0, 0, 1;
}

ModelRepresentation::~ModelRepresentation()
{
}

void ModelRepresentation::subDivide()
{
	igl::upsample(V, F, 2);
	computeNormals();
	nTriang = F.rows();
	nVert = V.rows();
	nVertexObservations = Eigen::VectorXi::Zero(nVert);
	adjustmentScores = Eigen::MatrixXf::Zero(nStepsDepthSearch, nVert);
}

bool ModelRepresentation::loadFile(std::string path)
{
	//we do it now with libigl

	Eigen::MatrixXd tempV; 
	igl::readOBJ(path.c_str(), tempV, F);
	Eigen::MatrixXd tempVh = Eigen::MatrixXd::Constant(tempV.cols()+1, tempV.rows(), 1.0);
	tempVh.block(0,0,tempV.cols(), tempV.rows()) = tempV.transpose();
	V = (modelToWorldTransform*tempVh).transpose().block(0, 0, tempV.rows(), 3);
	computeNormals();
	nTriang = F.rows();
	nVert = V.rows();
	nVertexObservations = Eigen::VectorXi::Zero(nVert);
	adjustmentScores = Eigen::MatrixXf::Zero(nStepsDepthSearch, nVert);
	std::cout << "\n" << "Number of vertices: " << nVert << "\n";
	std::cout << "\n" << "Number of triangles: " << nTriang << "\n";

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
	igl::per_vertex_normals(V, F, VN);		// calculates normals and feeds them into VN (rowwise)
}

bool ModelRepresentation::computeCenter(int verticeID, Eigen::Vector3d& midpoint)
{
	std::set<int> vertices;
	int insertCount = 0;
	for (int i = 0; i < F.rows(); i++) {
		for (int c = 0; c < 3; c++) {
			if (F(i, c) == verticeID) {
				for (int cc = 0; cc < 3; cc++)
					{
						if (cc != c) {
							insertCount++;
							vertices.insert(F(i, cc));
						}
					}
				break;
			}
		}
	}
	if (insertCount != vertices.size() * 2) return false;
	midpoint = Eigen::Vector3d::Zero();
	for (auto vid : vertices) {
		midpoint += V.row(vid).transpose();
	}
	midpoint /= vertices.size();
	return true;
}

