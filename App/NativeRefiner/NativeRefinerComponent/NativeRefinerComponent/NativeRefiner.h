#pragma once

#include <collection.h>
#include <ppl.h>
#include <amp.h>
#include <amp_math.h>
#include <windows.foundation.numerics.h>
#include "ModelRepresentation.h"
#include "ImageRepresentation.h"
#include <string>
#include <Eigen/StdVector>


namespace NativeRefinerComponent
{
	public delegate void ComputationFinishedHandler(int result);
	public ref class NativeRefiner sealed
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
		void addPicture(Platform::String^ path,//file path to images
			Windows::Foundation::Numerics::float4x4 CameraViewTransform,//Stores the camera's extrinsic transform in the coordinate system
			Windows::Foundation::Numerics::float4x4 CameraProjectionTransform);//Stores the camera's projection transform

		/// <summary>
		/// add initial model in specified file format---> https://github.com/TheMarcHeim/TheHoloRefiner/wiki/Mesh-file-format
		/// </summary>
		void addInitModel(Platform::String^ path);

		/// <summary>
		/// Async task that refines the reconstruction
		/// </summary>
		Windows::Foundation::IAsyncOperationWithProgress<Platform::String^, double>^ Refine();

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
		void computeAdjustmentScores();

	private:
		modelRep::ModelRepresentation model;
		std::vector<imageRep::ImageRepresentation, Eigen::aligned_allocator<Eigen::Matrix4f>> images;
		cv::Size patch_size;
		int nImages;
		Eigen::MatrixXi visibility; //rows: vertices, columns: images
	};
}