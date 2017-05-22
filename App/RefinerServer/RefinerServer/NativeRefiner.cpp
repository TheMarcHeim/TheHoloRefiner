#include "NativeRefiner.h"
#include "stdafx.h"
#include <iostream>
#include <igl/Hit.h>
#include <igl/ray_mesh_intersect.h>


NativeRefiner::NativeRefiner()
{
	patch_size = cv::Size(9,9); // to be experimented with - later implement in a parameter file preferably
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

void NativeRefiner::saveRefinedModel(std::string path)
{
	model.saveFile(path);
}

std::string NativeRefiner::refine(int nReps)
{

	int nAdj;

	for(int i=0; i<nReps; i++){
		std::cout << "Computing visibility..." << std::endl;
		computeVisibility();
		std::cout << "Computing adjustment scores..." << std::endl;
		computeAdjustmentScores();
		std::cout << "Adjusting vertices..." << std::endl;
		nAdj = adjustVertices();
		//model.subDivide();
		std::cout << "Percentage of adjusted vertices: " << ((double)nAdj) / ((double)model.nVert) << std::endl;
		
		// save intermediate refinement steps...
		if (i < nReps - 1) {
			std::string path = "C:/SofaData/";
			std::string name = "sofa_refined_intermediate" + std::to_string(i+1) + ".obj";
			saveRefinedModel(path + name);
		}
	}
	return "done";

}

int NativeRefiner::getSize() {
	return model.nTriang;
}

int NativeRefiner::getNImages() {
	return images.size();
}

void NativeRefiner::testPrj() {

	Eigen::Vector3d worldPoint(-2.8, 0.0, -1.4);
	int thisView = 1;

	Eigen::Vector3d C_w = images[thisView].CameraViewTransform.block<3, 1>(0, 3).cast <double>();						// camera center in world frame
	Eigen::Matrix<double, 3, 3> R_wc = images[thisView].CameraViewTransform.block<3, 3>(0, 0).cast <double>();			// camera orientation matrix (camera to world)	
	Eigen::Matrix<double, 3, 3> K = images[thisView].CameraProjectionTransform.block<3, 3>(0, 0).cast <double>();

	Eigen::Vector3d vertInCam = R_wc.transpose()*(worldPoint - C_w);													// ... in camera frame
	Eigen::Vector3d vertInImg = K*vertInCam;

	Eigen::Vector2d rel(vertInImg(0) / vertInImg(2), vertInImg(1) / vertInImg(2));
	Eigen::Vector2d pix((rel(0) + 1) / 2 * 2048 , (rel(1) + 1) / 2 * 1152);


	std::cout << "relative img coords " << "\n" << rel << "\n";
	std::cout << "pixel coords " << "\n" << pix << "\n";

}

int NativeRefiner::computeVisibility() {
	visibility = Eigen::MatrixXi::Zero(model.nVert, nImages);		//Binary matrix indicating if a vertex v is seen in image i, rows: vertices, columns: images 
	int nVis = 0;
	for (int v = 0; v < model.nVert; v++) {
		for (int i = 0; i < nImages; i++) {	
				if (isVisible(v, i)) {
					visibility(v, i) = 1;	//rows: vertices, columns: images
					nVis++;
				}			
		}
		if (v % 100 == 0) {
			progressPrint(v, model.nVert);
		}
	}

	progressPrint(1, 1);
	std::cout << "\nfinished computing visibility. number of visible vertices: " << nVis << std::endl;
	std::ofstream visibilityFile("C:/SofaData/visibility.txt");
	if (visibilityFile.is_open()) {
		visibilityFile << visibility;
		visibilityFile.close();
	}
	return nVis;
}

bool NativeRefiner::isVisible(int thisVertex, int thisView) {

	double threshold = 0.0;				// threshold for visibility
	bool visible = false;
	
	Eigen::Vector3d C_w = images[thisView].CameraViewTransform.block<3, 1>(0, 3).cast <double>();						// camera center in world frame
	Eigen::Matrix<double, 3, 3> R_wc = images[thisView].CameraViewTransform.block<3, 3>(0, 0).cast <double>();			// camera orientation matrix (camera to world)	
	Eigen::Matrix<double, 3, 3> K = images[thisView].CameraProjectionTransform.block<3, 3>(0, 0).cast <double>();
	
	Eigen::Vector3d P_w = model.V.block<1, 3>(thisVertex, 0).transpose();		
	Eigen::Vector3d N_w = model.VN.block<1, 3>(thisVertex, 0).transpose();
	
	
	/*if (N_w(0) != N_w(0) || N_w(0) != N_w(0) || N_w(0) != N_w(0)) {
		std::cout << "got some NANs here, index:" << thisVertex << std::endl;
	}*/

	Eigen::Vector3d vertInCam = R_wc.transpose()*(P_w - C_w);											// ... in camera frame
	Eigen::Vector3d vertInImg = K*vertInCam;															// ... in image frame, homgeneous


	// check if vertex is in front of camera
	if (vertInCam(2) > 0) {

		// check if vertex projects into image
		if(vertInImg(0)>= -vertInImg(2) && vertInImg(0) <= vertInImg(2) && vertInImg(1) >= -vertInImg(2) && vertInImg(1) <= vertInImg(2)){
			// check if patch is reasonably facing the camera using surface normal

			if (N_w.normalized().dot((C_w - P_w).normalized()) > threshold) {
				visible = true;
			}
		}
	}


	if (visible) {
		//check if occluded
		//positional vector
		Eigen::Vector3d camToPos = P_w - C_w;
		Eigen::Vector3d dir = camToPos.normalized();
		double expDist = camToPos.norm(); 		//expected distance
		igl::Hit hit; //first hit
		const double tolerance = 0.05;
		bool hasHit = igl::ray_mesh_intersect<Eigen::Vector3d, Eigen::Vector3d, Eigen::MatrixXd, Eigen::MatrixXi>(C_w, dir, model.V, model.F, hit);
		
		//check distance
		if (hasHit && hit.t < expDist - tolerance)
		{
			visible = false;
		}
	}
	
	return visible;
}

// This function computes adjustment scores for just one given vertex
void NativeRefiner::computeVertexAdjustmentScores(int vertex, int view1, int view2) {

	Eigen::Vector3d p(0, 0, 0);
	Eigen::Vector3d n(0, 0, 0);
	Eigen::Vector3d p_current(0, 0, 0);


	p << model.V.block<1, 3>(vertex, 0).transpose();
	n << model.VN.block<1, 3>(vertex, 0).transpose();
	p_current = p - n*model.stepSize*model.nStepsDepthSearch/2; // start at negative position along normal


	float weight =  images[view1].getViewQuality(p, n, images[view2]);
	

	model.nVertexObservations(vertex)+=weight; // needed for averaging


	for (int i = 0; i < model.nStepsDepthSearch; i++) {
		p_current += model.stepSize*n;
		//std::cout <<"match: "<< images[view2].computeDistortedPatchCorrelation(images[view1], n, p_current, patch_size, 0)<<std::endl;
		//std::cout << "regularization: " << -(isInside ? dtmp.squaredNorm()*lambda : 0) << std::endl;
		model.adjustmentScores(i, vertex) *= (model.nVertexObservations(vertex)-weight);
		model.adjustmentScores(i, vertex) += weight*images[view2].computeDistortedPatchCorrelation(images[view1], n, p_current, patch_size, 0);		//set last argument to zero: calculate with grayscale patches, otherwise with color
		model.adjustmentScores(i, vertex) /= (model.nVertexObservations(vertex));
		if (model.adjustmentScores(i, vertex) != model.adjustmentScores(i, vertex)) {
			std::cout << "nan" << std::endl;
		}
	} 
	std::cout << "Adjustment scores for Vertex " << vertex << " are \n" << model.adjustmentScores.block<21, 1>(0, vertex) << std::endl << std::endl;

}

// This function computes adjustment scores for all vertices and pairs
int NativeRefiner::computeAdjustmentScores() {
	// loop through all vertices and images, find pairs and compute 

	model.adjustmentScores = Eigen::MatrixXf::Zero(model.nStepsDepthSearch, model.nVert);
	//TODO model.nVertexObservations = 0

	for (int v = 0; v < model.nVert; v++) {
		for (int firstSight = 0; firstSight < nImages; firstSight++) {
			if (visibility(v, firstSight) == 1 && firstSight < nImages - 1) {
				for (int secondSight = firstSight + 1; secondSight < nImages; secondSight++) {
					if (visibility(v, secondSight) == 1) {
						computeVertexAdjustmentScores(v, firstSight, secondSight);

					}
				}
			}
		}

		// regularization of mesh
		//const double lambda = 100000;
		/*const double lambda = 0;
		Eigen::Vector3d midPoint;
		bool isInside = model.computeCenter(v, midPoint);
		Eigen::Vector3d p(model.V(v, 2), model.V(v, 0), model.V(v, 1));
		Eigen::Vector3d n(model.VN(v, 2), model.VN(v, 0), model.VN(v, 1));
		Eigen::Vector3d p_current = p - n*model.stepSize*model.nStepsDepthSearch / 2;
		Eigen::Vector3d dtmp = midPoint - p_current;
		
		for (int i = 0; i < model.nStepsDepthSearch; i++) {
			p_current += model.stepSize*n;
			dtmp = midPoint - p_current;
			model.adjustmentScores(i, v) += (isInside ? dtmp.squaredNorm()*lambda : 0);
		}*/

		// print progress
		//if (v % 10 == 0) {
			progressPrint(v, model.nVert);
		//}
	}
	progressPrint(1, 1);
	std::cout << "\nfinished computing adjustmentScores." << std::endl;		
	return 0;
}


int NativeRefiner::adjustVertices() {

	int nAdj = 0;
	for (int v = 0; v < model.nVert; v++) { //loop through all vertices
		int bestVertex = model.nStepsDepthSearch / 2;
		float bestScore = model.adjustmentScores(bestVertex, v); //initial score
		for (int i = 0; i < model.nStepsDepthSearch; i++) {
			if (model.adjustmentScores(i, v) > bestScore ) {//+ model.refineTolerance*pow(i - model.nStepsDepthSearch / 2, 2)
				bestScore = model.adjustmentScores(i, v);
				bestVertex = i;
			}
		}
		if (bestScore > model.adjustmentScores(model.nStepsDepthSearch / 2, v) + model.refineTolerance*pow(bestVertex - model.nStepsDepthSearch / 2, 1.2)) {
			model.V.block<1, 3>(v, 0) += model.stepSize*(bestVertex - model.nStepsDepthSearch / 2)*model.VN.block<1, 3>(v, 0);
			nAdj++;
			std::cout << "adjusted Vertex " << v << " by " << (bestVertex - model.nStepsDepthSearch / 2) << std::endl;
		}
		if (v % 100 == 0) {
			progressPrint(v, model.nVert);
		}
	}
	progressPrint(1, 1);
	std::cout << "\nFinished adjusting Vertices.\nPercentage of adjusted vertices: " << ((double)nAdj) / ((double)model.nVert) << std::endl;

	// debatable if we should recompute normals here
	model.computeNormals();
	return nAdj;
}

void NativeRefiner::progressPrint(int n, int m) {
	int barWidth = 70;
	float progress = (((float)n) / ((float)m));
	std::cout << "[";
	int pos = barWidth * progress;
	for (int i = 0; i < barWidth; ++i) {
		if (i < pos) std::cout << "=";
		else if (i == pos) std::cout << ">";
		else std::cout << " ";
	}
	std::cout << "] " << int(progress*100.0) << " %\r";
	std::cout.flush();
}