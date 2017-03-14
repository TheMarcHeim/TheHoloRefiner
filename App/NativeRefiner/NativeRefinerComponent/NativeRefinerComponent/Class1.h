#pragma once

#include <collection.h>
#include <ppl.h>
#include <amp.h>
#include <amp_math.h>
#include <windows.foundation.numerics.h>

namespace NativeRefinerComponent
{
	public delegate void ComputationFinishedHandler(int result);
    public ref class NativeRefiner sealed
    {
    public:
		NativeRefiner();

		//delete pictures
		void reset();

		//add a calibrated image 
		//the following can be acquired from the SDK
		void addPicture(Platform::String^ path,//file path to images
			Windows::Foundation::Numerics::float4x4 CameraViewTransform,//Stores the camera's extrinsic transform in the coordinate system
			Windows::Foundation::Numerics::float4x4 CameraProjectionTransform);//Stores the camera's projection transform

		//Async task that processes the
		Windows::Foundation::IAsyncOperationWithProgress<Platform::String^, double>^ Refine();

    };
}
