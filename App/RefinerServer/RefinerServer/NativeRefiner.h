#pragma once

#include "ModelRepresentation.h"
#include "ImageRepresentation.h"
#include <string>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <iostream>
#include "readParams.h"
//#include <igl/viewer/Viewer.h> didn't work until now


	class NativeRefiner
	{
	public:
		NativeRefiner();

		/// <summary>
		/// delete pictures
		/// </summary>
		void reset();

		/// <summary>
		/// add a calibrated image 
		/// the following can be acquired from the SDK
		/// </summary>
		void addPicture(std::string path,//file path to images
			Eigen::Matrix4d CameraViewTransform,//Stores the camera's extrinsic transform in the coordinate system
			Eigen::Matrix4d  CameraProjectionTransform);//Stores the camera's projection transform

		/// <summary>
		/// add initial model in specified file format---> https://github.com/TheMarcHeim/TheHoloRefiner/wiki/Mesh-file-format
		/// </summary>
		void addInitModel(std::string path);

		/// <summary>
		/// add initial model in specified file format---> https://github.com/TheMarcHeim/TheHoloRefiner/wiki/Mesh-file-format
		/// </summary>
		void saveRefinedModel(std::string path);

		/// <summary>
		/// Async task that refines the reconstruction
		/// </summary>
		std::string refine(int nReps);

		/// <summary>
		/// (current) number of vertices of reconstruction
		/// </summary>
		int getSize();

		/// <summary>
		/// number of images
		/// </summary>
		int getNImages();

		/// <summary>
		/// test projection
		/// </summary>
		void testPrj();

		/// <summary>
		/// Function that computes which vertices are visible from which cameras. 
		/// This data is stored in the visibility matrix (member of NativeRefiner)
		/// </summary>
		int computeVisibility();

		/// <summary>
		/// Function to compute visibility of a given vertex from a given view
		/// </summary>
		bool isVisible(int thisView, int thisVertex);

		/// <summary>
		/// Function to compute adjustment scores for a given vertex and two given views
		/// </summary>
		void computeVertexAdjustmentScores(int vertex, int view1, int view2);

		/// <summary>
		/// Function to compute adjustment scores for all pairs
		/// </summary>
		int computeAdjustmentScores();

		/// <summary>
		/// Function to compute adjustment scores for all pairs
		/// </summary>
		int adjustVertices();


		/// <summary>
		/// Function to print progress to Console
		/// </summary>
		void progressPrint(int n, int m);


		std::vector<ImageRepresentation, Eigen::aligned_allocator<Eigen::Matrix4f>> images;

	private:
		parameters params;
		ModelRepresentation model;
		int nImages;
		Eigen::MatrixXi visibility; //rows: vertices, columns: images
	};