#pragma once

#include<vector>
#include<string>
#include<Eigen/Dense>
#include<Eigen/Sparse>
#include<igl/cotmatrix.h>
#include<igl/readOBJ.h>
#include<ImageRepresentation.h>

//use this for internal representation
namespace modelRep {

	class ModelRepresentation
	{
	public:
		ModelRepresentation();
		~ModelRepresentation();
		
		/// <summary>
		/// Subdivide triangles into 4 equal subtriangles
		/// </summary>
		void subDivide();

		/// <summary>
		/// Load object file
		/// </summary>
		bool loadFile(std::string path);

		/// <summary>
		/// compute normals at vertices
		/// </summary>
		void computeNormals();
		
		int nTriang;
		int nVert;
		int nStepsDepthSearch;
		Eigen::MatrixXd V; //vertices
		Eigen::MatrixXi F;
		Eigen::MatrixXd VN; // vertex normals
		Eigen::MatrixXf adjustmentScores; //columns: vertices, rows: adjustmentScores
		Eigen::VectorXi nVertexObservations; // counter needed to normalize adjustmentScores

	private:
	};
}
