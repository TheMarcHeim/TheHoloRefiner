﻿#include "pch.h"
#include "Class1.h"

#include <ppltasks.h>
#include <concurrent_vector.h>
#include <iostream>


using namespace NativeRefinerComponent;
using namespace Platform;
using namespace concurrency;
using namespace Platform::Collections;
using namespace Windows::Foundation::Collections;
using namespace Windows::Foundation;
using namespace Windows::UI::Core;



NativeRefiner::NativeRefiner()
{
}

void NativeRefinerComponent::NativeRefiner::reset()
{
	throw ref new Platform::NotImplementedException();
}

void NativeRefinerComponent::NativeRefiner::addPicture(Platform::String^ path, Windows::Foundation::Numerics::float4x4 CameraViewTransform, Windows::Foundation::Numerics::float4x4 CameraProjectionTransform)
{
	//convert from managed string :/
	std::wstring fooW(path->Begin());
	std::string upath(fooW.begin(), fooW.end());
	//add new image
	images.push_back(
		imageRep::ImageRepresentation(upath, CameraViewTransform, CameraProjectionTransform));
	nImages++;
}

void NativeRefinerComponent::NativeRefiner::addInitModel(Platform::String^ path)
{
	//convert from managed string :/
	std::wstring fooW(path->Begin());
	std::string upath(fooW.begin(), fooW.end());
	model.loadFile(upath);
	
}

Windows::Foundation::IAsyncOperationWithProgress<Platform::String^, double>^ NativeRefinerComponent::NativeRefiner::Refine()
{
	//maybe we use this later with events:
	//auto window = Windows::UI::Core::CoreWindow::GetForCurrentThread();
	//m_dispatcher = window->Dispatcher;
	
	//async task
	return create_async([this](progress_reporter<double> reporter)-> Platform::String^ {
		//do the refining
		//throw ref new Platform::NotImplementedException();

		//loop through all images
		//for (std::vector<imageRep::ImageRepresentation, Eigen::aligned_allocator<Eigen::Matrix4f>>::iterator it = images.begin(); it != images.end(); ++it) {
		//}

		int testImg = 0;
		int testVertex = 127;
		int bingo = 0;

		computeVisibility();


		
			//cv::namedWindow("test image",WINDOW_AUTOSIZE);
			//cv::imshow("test image", images[testImg]);
/*		cv::Point2i ctr(50, 50);
		const cv::Scalar color(0, 100, 250);
		cv::circle(images[testImg].ocvImage, ctr, 50, color);
		cv::imwrite("test.png", images[testImg].ocvImage);
	*/
		//}


		//return path
		return 	bingo.ToString();

		reporter.report(100.0);
	});


}

int NativeRefinerComponent::NativeRefiner::getSize() {
	return model.nTriang;
}

int NativeRefinerComponent::NativeRefiner::getNImages() {
	return images.size();
}

void NativeRefinerComponent::NativeRefiner::computeVisibility() {
	visibility = Eigen::MatrixXi::Zero(model.nVert, nImages);//rows: vertices, columns: images
	for (int i = 0; i < model.nVert; i++) {
		for (int j = 0; j < nImages; j++) {
			if (isVisible(j, i)) {
				visibility(i, j) = 1;	//rows: vertices, columns: images
			}
		}
	}
}

bool NativeRefinerComponent::NativeRefiner::isVisible(int thisView, int thisVertex) {

	double threshold = 0.0; // threshold for visibility
	bool visible = false;
	
	Eigen::Vector3d C = images[thisView].CameraViewTransform.block<3, 1>(0, 3).cast <double>(); //camera center
	Eigen::Vector3d p = model.V.block<1, 3>(thisVertex, 0).transpose(); // point in world frame (vertex)
	Eigen::Vector4d p_n(p(0), p(1), p(2), 1);
	
	Eigen::Matrix<double, 3, 4> T = images[thisView].CameraProjectionTransform.block<3, 3>(0, 0).cast <double>()*images[thisView].CameraViewTransform.block<3, 4>(0, 0).cast <double>();
	Eigen::Vector3d pixels_not_normalized = T* p_n; // project vertex into image
	double x_px = pixels_not_normalized(0) / pixels_not_normalized(2); // normalize points in 2d image space
	double y_px = pixels_not_normalized(1) / pixels_not_normalized(2);
	
	
	// check if vertex projects into image
	if (x_px < images[thisView].x_size && y_px < images[thisView].y_size && x_px >= 0 && y_px >= 0) {
		// check if patch is reasonably facing the camera
		if (model.VN.block<1, 3>(thisVertex, 0).transpose().dot((p - C).normalized()) > threshold) {
			visible = true;
		}
	}
	return visible;
}

void NativeRefinerComponent::NativeRefiner::computeAdjustmentScores(int* adjustmentScores, int vertex) {

	Eigen::Vector3d p(0, 0, 0);
	Eigen::Vector3d n(0, 0, 0);
	Eigen::Vector3d p_current(0, 0, 0);

	int c1 = 0;
	int c2 = 0;

	int step = 1;
	double step_size = 2;

	p = model.V.block<1, 3>(vertex, 0).transpose(); // point in world frame (vertex)
	n = model.VN.block<1, 3>(vertex, 0).transpose();

	p_current = p+step*step_size*n;
	


}