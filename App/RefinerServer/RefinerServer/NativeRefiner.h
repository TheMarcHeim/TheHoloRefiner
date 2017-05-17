#pragma once

#include "ModelRepresentation.h"
#include "ImageRepresentation.h"
#include <string>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <iostream>
//#include <igl/viewer/Viewer.h> didn't work until now


	class NativeRefiner
	{
	public:
		NativeRefiner();

		//usage in c#:
		//first:
		//var refiner = new NativerRefinerComponent.NativeRefiner();
		//refiner.reset();
		//for all images:
		//	refiner.addpicture(path, CameraViewTransform, CameraProjectionTransform);
		//refiner.addInitModel(path)
		//var asyncOp = refiner.Refine()
		///lambda for progress
		//asyncOp.Progress = (asyncInfo, progress) =>
		//	{do UI update of progress bar};
		//var async pathOfRefinedMesh = await asyncOp



		/// <summary>
		/// delete pictures
		/// </summary>
		void reset();

		/// <summary>
		/// add a calibrated image 
		/// the following can be acquired from the SDK
		/// </summary>
		void addPicture(std::string path,//file path to images
			Eigen::Matrix4f CameraViewTransform,//Stores the camera's extrinsic transform in the coordinate system
			Eigen::Matrix4f  CameraProjectionTransform);//Stores the camera's projection transform

		/// <summary>
		/// add initial model in specified file format---> https://github.com/TheMarcHeim/TheHoloRefiner/wiki/Mesh-file-format
		/// </summary>
		void addInitModel(std::string path);

		/// <summary>
		/// Async task that refines the reconstruction
		/// </summary>
		std::string Refine();

		/// <summary>
		/// (current) number of vertices of reconstruction
		/// </summary>
		int getSize();

		/// <summary>
		/// number of images
		/// </summary>
		int getNImages();

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
		std::vector<ImageRepresentation, Eigen::aligned_allocator<Eigen::Matrix4f>> images;

	private:
		ModelRepresentation model;

		cv::Size patch_size;
		int nImages;
		Eigen::MatrixXi visibility; //rows: vertices, columns: images
	};