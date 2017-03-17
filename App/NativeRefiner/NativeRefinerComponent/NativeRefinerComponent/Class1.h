#pragma once

#include <collection.h>
#include <ppl.h>
#include <amp.h>
#include <amp_math.h>
#include <windows.foundation.numerics.h>
#include "ModelRepresentation.h"
#include <string>


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
		/// Async task that refines the 
		/// </summary>
		Windows::Foundation::IAsyncOperationWithProgress<Platform::String^, double>^ Refine();

		int getSize();

	private:
		modelRep::ModelRepresentation model;
    };
}
