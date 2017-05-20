#pragma once

#include<vector>
#include<string>
#include<Eigen/Dense>
#include<igl/cotmatrix.h>
#include<igl/upsample.h>
#include<igl/per_vertex_normals.h>
#include<igl/per_face_normals.h>
#include<igl/per_corner_normals.h>
#include<igl/readOBJ.h>
#include <igl/writeOBJ.h>

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
		double refineTolerance;

		Eigen::MatrixXd V; //vertices
		Eigen::MatrixXd CorrectV;
		Eigen::MatrixXi F;
		Eigen::MatrixXd VN; // vertex normals
		Eigen::MatrixXf adjustmentScores; //columns: vertices, rows: adjustmentScores
		Eigen::VectorXi nVertexObservations; // counter needed to normalize adjustmentScores
		Eigen::Matrix4f modelToWorldTransform; 
	};
