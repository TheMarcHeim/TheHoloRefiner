#include "pch.h"
#include "NativeRefiner.h"
#include <ppltasks.h>
#include <concurrent_vector.h>

using namespace NativeRefinerComponent;
using namespace Platform;
using namespace concurrency;
using namespace Platform::Collections;
using namespace Windows::Foundation::Collections;
using namespace Windows::Foundation;
using namespace Windows::UI::Core;


NativeRefiner::NativeRefiner()
{
	patch_size = cv::Size(7, 7); // to be experimented with - later implement in a parameter file preferably
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
		// compute adjustment scores
		//computeAdjustmentScores();

		//return
		return computeVisibility().ToString();// model.adjustmentScores(7, 348).ToString(); // return random adjustment score
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

		// check if vertex projects into image
		if(vertInImg(0)>= -vertInImg(2) && vertInImg(0) <= vertInImg(2) && vertInImg(1) >= -vertInImg(2) && vertInImg(1) <= vertInImg(2)){
	
			// check if patch is reasonably facing the camera using surface normal
			if (N_w.dot((C_w - P_w).normalized()) > threshold) {
				visible = true;
			}
		}
	}
	return visible;
}

// This function computes adjustment scores for just one given vertex
void NativeRefinerComponent::NativeRefiner::computeVertexAdjustmentScores(int vertex, int view1, int view2) {
	
	Eigen::Vector3d p(0, 0, 0);
	Eigen::Vector3d n(0, 0, 0);
	Eigen::Vector3d p_current(0, 0, 0);
	double step_size = 0.05; // to be experimented with - later implement in a parameter file preferably
		
	p << model.V(vertex, 2), model.V(vertex, 0), model.V(vertex, 1);  // Note permutation required to be consistent with new convention
	n << model.VN(vertex,2), model.VN(vertex,0), model.VN(vertex,1);  // Haven't yet checked whether these coordinates need to be permuted as well - but 
																  // it is very unlikely that they use different conventions in the same .obj file ...
	
	p_current = p - n*step_size*model.nStepsDepthSearch/2; // start at negative position along normal

	float temp; // just for visualisation in debugger

	model.nVertexObservations(vertex)++; // needed for averaging
	for (int i = 0; i < model.nStepsDepthSearch; i++) {
		p_current += step_size*n;
		model.adjustmentScores(i, vertex) *= (model.nVertexObservations(vertex)-1);
		model.adjustmentScores(i, vertex) += images[view2].computeDistortedPatchCorrelation(images[view1], n, p_current, patch_size);
		model.adjustmentScores(i, vertex) /= (model.nVertexObservations(vertex));
		temp = model.adjustmentScores(i, vertex); // to monitor in debugger while we don't have cout
	} 
}

// This function computes adjustment scores for all vertices and pairs
int NativeRefinerComponent::NativeRefiner::computeAdjustmentScores() {

	int bingo = 0; // dummy variable used for debugging
	int visCount = 0;
	int firstSight = 0;
	int secondSight = 0;
	
	// loop through all vertices and images, find pairs and compute 
	// Note: only looks at pairs including first image
	bingo = computeVisibility();
	for (int v = 0; v < model.nVert; v++) {
		visCount = 0;
		for (int i = 0; i < nImages; i++) {
			if (visibility(v, i) == 1) {
				visCount++;
				if (visCount >= 2) { // compute adjustment scores for "every" pair
					secondSight = i;
					computeVertexAdjustmentScores(v, firstSight, secondSight);
					bingo++; //just a dummy to stop calculation at some point
					if (bingo >= 1000)
						return v;
				}
				else {
					firstSight = i;
				}
			}
		}
	}
	return 0;
}

