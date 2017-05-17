#pragma once

#include<string>
#include<Eigen/Dense>

//use this for internal representation

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
		/// Save object file
		/// </summary>
		bool saveFile(std::string path);


		/// <summary>
		/// compute normals at vertices
		/// </summary>
		void computeNormals();
		
		int nTriang;
		int nVert;
		int nStepsDepthSearch;
		double stepSize;

		Eigen::MatrixXd V; //vertices
		Eigen::MatrixXi F;
		Eigen::MatrixXd VN; // vertex normals
		Eigen::MatrixXf adjustmentScores; //columns: vertices, rows: adjustmentScores
		Eigen::VectorXi nVertexObservations; // counter needed to normalize adjustmentScores
	};
