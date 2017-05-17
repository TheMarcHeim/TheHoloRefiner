#include "NativeRefiner.h"
#include "stdafx.h"
#include <igl/Hit.h>
#include <igl/ray_mesh_intersect.h>

NativeRefiner::NativeRefiner()
{
	patch_size = cv::Size(9, 9); // to be experimented with - later implement in a parameter file preferably
}

void NativeRefiner::reset()
{
	std::cout << "Not implemented";
}

void NativeRefiner::addPicture(std::string path, Eigen::Matrix4f CameraViewTransform, Eigen::Matrix4f CameraProjectionTransform)
{
	//add new image
	images.push_back(
		ImageRepresentation(path, CameraViewTransform, CameraProjectionTransform));
	nImages++;
}

void NativeRefiner::addInitModel(std::string path)
{
	model.loadFile(path);
}

std::string NativeRefiner::Refine()
{
	computeVisibility();
	computeAdjustmentScores();
	return "done";// std::to_string(model.adjustmentScores(7, 348));// return random adjustment score
	//return std::to_string(computeVisibility());//  
}

int NativeRefiner::getSize() {
	return model.nTriang;
}

int NativeRefiner::getNImages() {
	return images.size();
}

int NativeRefiner::computeVisibility() {

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

bool NativeRefiner::isVisible(int thisVertex, int thisView) {

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


	if (visible) {
		//check if occluded
		//positional vector
		Eigen::Vector3d camToPos = P_w - C_w;
		Eigen::Vector3d dir = camToPos.normalized();
		//expected distance
		double expDist = camToPos.norm();
		//first hit
		igl::Hit hit;
		igl::ray_mesh_intersect<Eigen::Vector3d, Eigen::Vector3d, Eigen::MatrixXd, Eigen::MatrixXi>(C_w, dir, model.V, model.F, hit);

		const double tolerance = 0.95;
			//check distance
		if (hit.t < tolerance*expDist)
			visible = false;


	}
	return visible;
}

// This function computes adjustment scores for just one given vertex
void NativeRefiner::computeVertexAdjustmentScores(int vertex, int view1, int view2) {

	Eigen::Vector3d p(0, 0, 0);
	Eigen::Vector3d n(0, 0, 0);
	Eigen::Vector3d p_current(0, 0, 0);
	double step_size = 0.01; // to be experimented with - later implement in a parameter file preferably
		
	p << model.V(vertex, 2)+0.02, model.V(vertex, 0)+0.22, model.V(vertex, 1)-0.05;  // Note permutation required to be consistent with new convention
	//p << model.V(vertex, 2), model.V(vertex, 0), model.V(vertex, 1);
	n << model.VN(vertex,2), model.VN(vertex,0), model.VN(vertex,1);  // Haven't yet checked whether these coordinates need to be permuted as well - but 
																  // it is very unlikely that they use different conventions in the same .obj file ...
	
	p_current = p - n*step_size*model.nStepsDepthSearch/2; // start at negative position along normal

	model.nVertexObservations(vertex)++; // needed for averaging
	for (int i = 0; i < model.nStepsDepthSearch; i++) {
		p_current += step_size*n;
		model.adjustmentScores(i, vertex) *= (model.nVertexObservations(vertex)-1);
		model.adjustmentScores(i, vertex) += images[view2].computeDistortedPatchCorrelation(images[view1], n, p_current, patch_size);
		model.adjustmentScores(i, vertex) /= (model.nVertexObservations(vertex));
	} 
	std::cout << "Adjustment scores for Vertex " << vertex << " are \n" << model.adjustmentScores.block<11, 1>(0, vertex) << std::endl << std::endl;
}

// This function computes adjustment scores for all vertices and pairs
int NativeRefiner::computeAdjustmentScores() {

	int bingo = 0; // dummy variable used for debugging
	int visCount = 0;
	int firstSight = 0;
	int secondSight = 0;
	
	// loop through all vertices and images, find pairs and compute 
	// Note: only looks at pairs including first image
	for (int v = 0; v < model.nVert; v++) {
		visCount = 0;
		for (int i = 0; i < nImages; i++) {
			if (visibility(v, i) == 1) {
				visCount++;
				if (visCount >= 2) { // compute adjustment scores for "every" pair
					secondSight = i;
					computeVertexAdjustmentScores(v, firstSight, secondSight);
					bingo++; //just a dummy to stop calculation at some point
					if (bingo >= 50)
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

