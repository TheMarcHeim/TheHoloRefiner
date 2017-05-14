#include "pch.h"
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
	std::printf("Printf works!");
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
		int bingo = 21340;
		int visCount = 0;
		int firstSight = 0;
		int secondSight = 0;
		cv::Size patch_size(3, 3);
		float correlation;

		bingo = computeVisibility();


		// loop through all vertices and images, find pairs and compute correlation
		
		for (int v = 0; v < model.nVert; v++) {
			visCount = 0;
			for (int i = 0; i < nImages; i++) {
				if (visibility(v,i)==1) {
					visCount++;
					
					if (visCount >= 2) {
						secondSight = i;
						correlation = images[i].computeDistortedPatchCorrelation(images[firstSight], model.VN.block<1, 3>(v, 0).transpose(), model.V.block<1, 3>(v, 0).transpose(), patch_size);
						return correlation.ToString();
					}
					else {
						firstSight = i;
					}

				}
			}
		}
		

		
		// Show vertex in a window
		/*
		cv::namedWindow("test image",WINDOW_AUTOSIZE);
		cv::imshow("test image", images[testImg]);
		cv::Point2i ctr(50, 50);
		const cv::Scalar color(0, 100, 250);
		cv::circle(images[testImg].ocvImage, ctr, 50, color);
		cv::imwrite("test.png", images[testImg].ocvImage);
	*/


		//return
		return 	"no correlation found";
		reporter.report(100.0);
	});


}

int NativeRefinerComponent::NativeRefiner::getSize() {
	return model.nTriang;
}

int NativeRefinerComponent::NativeRefiner::getNImages() {
	return images.size();
}

int NativeRefinerComponent::NativeRefiner::computeVisibility() {

	visibility = Eigen::MatrixXi::Zero(model.nVert, nImages);		//Binary matrix indicating if a vertex v is seen in image i, rows: vertices, columns: images 
	int nVis = 0;
	for (int v = 0; v < model.nVert; v++) {
		for (int i = 0; i < nImages; i++) {
			if (isVisible(v,i)) {
				visibility(v,i) = 1;	//rows: vertices, columns: images
				nVis++;
			}
		}
	}
	return nVis;
}

bool NativeRefinerComponent::NativeRefiner::isVisible(int thisVertex, int thisView) {

	double threshold = 0.0;				// threshold for visibility
	bool visible = false;
	
	Eigen::Vector3d C_w = images[thisView].CameraViewTransform.block<3, 1>(0, 3).cast <double>();						// camera center in world frame
	Eigen::Matrix<double, 3, 3> R_wc = images[thisView].CameraViewTransform.block<3, 3>(0, 0).cast <double>();			// camera orientation matrix (camera to world)	
	Eigen::Matrix<double, 3, 3> K = images[thisView].CameraProjectionTransform.block<3, 3>(0, 0).cast <double>();
	
	//Eigen::Vector3d P_w = model.V.block<1, 3>(thisVertex, 0).transpose();								// -> in form (y,z,x), we don't want this
	Eigen::Vector3d P_w(model.V(thisVertex,2), model.V(thisVertex, 0), model.V(thisVertex, 1));			// Instead, permute to get it in form (x,y,z) and to be consistent with new convention.
	Eigen::Vector3d N_w(model.VN(thisVertex, 2), model.VN(thisVertex, 0), model.VN(thisVertex, 1));		// Same for normals. IDEALLY we could reformat everything directly when loading the model TBD
	
	Eigen::Vector3d vertInCam = R_wc.transpose()*(P_w - C_w);											// ... in camera frame
	Eigen::Vector3d vertInImg = K*vertInCam;															// ... in image frame, homgeneous


	// check if vertex is in front of camera
	if (vertInCam(2) > 0) {
		double pix_u = (vertInImg(0) / vertInImg(2) + 1) / 2 * 2048;									// normalize and get pixel values
		double pix_v = (vertInImg(1) / vertInImg(2) + 1) / 2 * 1152;

		// check if vertex projects into image
		if (pix_u < images[thisView].x_size && pix_v < images[thisView].y_size && pix_u >= 0 && pix_v >= 0) {
			
			// check if patch is reasonably facing the camera using surface normal
			if (N_w.dot((P_w - C_w).normalized()) > threshold) {
				visible = true;
			}
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

		
	p << model.V(vertex, 2), model.V(vertex, 0), model.V(vertex, 1);  // Note permutation required to be consistent with new convention

	n << model.VN(vertex,2), model.VN(vertex,0), model.VN(vertex,1);  // Haven't yet checked whether these coordinates need to be permuted as well - but 
																	  // it is very unlikely that they use different conventions in the same .obj file ...

	p_current = p+step*step_size*n;

}