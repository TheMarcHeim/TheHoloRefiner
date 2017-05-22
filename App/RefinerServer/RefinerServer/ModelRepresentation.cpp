#include "ModelRepresentation.h"
#include "stdafx.h"
#include <iostream>


ModelRepresentation::ModelRepresentation()
{
	nStepsDepthSearch = 11;
	stepSize = 0.01; 
	refineTolerance = 0.0; // only adjust vertex if new one is this much better
	modelToWorldTransform << 0, 0, 1, 0.02,
							 1, 0, 0, 0.19,
							 0, 1, 0, -0.06; // sofa dataset
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
	CorrectV = Eigen::MatrixXd(V.rows(), V.cols());
	CorrectV << V.col(2), V.col(0), V.col(1);

	nVertexObservations = Eigen::VectorXf::Zero(nVert);
	adjustmentScores = Eigen::MatrixXf::Zero(nStepsDepthSearch, nVert);
}

bool ModelRepresentation::loadFile(std::string path)
{
	//we do it now with libigl
	igl::readOBJ(path.c_str(), V, F);	// V: contains vertex coordiantes rowwise, F: triangle connectivity, per row 3 indices
	computeNormals();
	nTriang = F.rows();
	nVert = V.rows();

	std::cout << V.row(3) << std::endl;

	V.col(0).array() += modelToWorldTransform(2, 3);
	V.col(1).array() += modelToWorldTransform(0, 3);
	V.col(2).array() += modelToWorldTransform(1, 3);

	std::cout << V.row(3) << std::endl;

	CorrectV = Eigen::MatrixXd(V.rows(), V.cols());
	CorrectV << V.col(2), V.col(0), V.col(1);
	
	nVertexObservations = Eigen::VectorXf::Zero(nVert);
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

