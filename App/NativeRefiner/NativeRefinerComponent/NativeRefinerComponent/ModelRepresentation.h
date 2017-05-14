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
		/// Make refinement step for image pair
		/// </summary>
		float AdjustVertex(imageRep::ImageRepresentation& I, imageRep::ImageRepresentation& J, int vertex);

		void Refine(int numIt, double stepSize);

		bool loadFile(std::string path);
		
		int nTriang;
		int nVert;


		Eigen::MatrixXd V; //vertices
		Eigen::MatrixXi F;
		Eigen::MatrixXd VN; // vertex normals (?)
	//	Eigen::MatrixXf adjustmentScores; //columns: vertices, rows: adjustmentScores
	//	Eigen::MatrixXi nVertexObservations; // counter needed to normalize adjustmentScores
		void computeNormals();
	private:
	};
}
