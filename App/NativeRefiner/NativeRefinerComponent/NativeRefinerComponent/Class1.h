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
#define NUMBER_STEPS_DEPTH_SEARCH  11 //should be an odd number, center = original vertex

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
		/// Async task that refines the reconstruction
		/// </summary>
		int computeVisibility();

		/// <summary>
		/// Async task that refines the reconstruction
		/// </summary>
		bool isVisible(int thisView, int thisVertex);

		/// <summary>
		/// Async task that refines the reconstruction
		/// </summary>
		void computeAdjustmentScores(float* adjustmentScores, int vertex, int view1, int view2);

	private:
		modelRep::ModelRepresentation model;
		std::vector<imageRep::ImageRepresentation, Eigen::aligned_allocator<Eigen::Matrix4f>> images;
		cv::Size patch_size;// (7, 7);
		int nImages;
		Eigen::MatrixXi visibility; //rows: vertices, columns: images
		//int adjustmentScores[NUMBER_STEPS_DEPTH_SEARCH];
	};
}