#include "NativeRefiner.h"
#include "stdafx.h"
#include <iostream>
#include <igl/Hit.h>
#include <igl/ray_mesh_intersect.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include "opencv2/features2d/features2d.hpp"
#include "opencv/cv.h"
#include "opencv2\highgui\highgui.hpp"


NativeRefiner::NativeRefiner()
{
	loadParams("params.txt", params);
}

void NativeRefiner::reset()
{
	std::cout << "Not implemented";
}

void NativeRefiner::addPicture(std::string path, Eigen::Matrix4d CameraViewTransform, Eigen::Matrix4d CameraProjectionTransform)
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

	if(params.useSubdivision)
		model.subDivide();	

	for(int i=0; i<nReps; i++){
		std::cout << "Computing visibility..." << std::endl;
		computeVisibility();
		std::cout << "Computing adjustment scores..." << std::endl;
		computeAdjustmentScores();
		std::cout << "Adjusting vertices..." << std::endl;
		nAdj = adjustVertices();
		
		// save intermediate refinement steps...
		/*if (i < nReps - 1) {
			std::string path = "C:/HappyBirthday/intermediates/";
			std::string name = "sofa_refined_intermediate" + std::to_string(i+1) + ".obj";
			saveRefinedModel(params.path + name);
		}*/
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
	/*
	std::ofstream visibilityFile("C:/HappyBirthday/visibility.txt");
	if (visibilityFile.is_open()) {
		visibilityFile << visibility;
		visibilityFile.close();
	}*/
	return nVis;
}

bool NativeRefiner::isVisible(int thisVertex, int thisView) {
	bool visible = false;
	
	Eigen::Vector3d C_w = images[thisView].CameraViewTransform.block<3, 1>(0, 3).cast <double>();						// camera center in world frame
	Eigen::Matrix<double, 3, 3> R_wc = images[thisView].CameraViewTransform.block<3, 3>(0, 0).cast <double>();			// camera orientation matrix (camera to world)	
	Eigen::Matrix<double, 3, 3> K = images[thisView].CameraProjectionTransform.block<3, 3>(0, 0).cast <double>();
	
	Eigen::Vector3d P_w = model.V.block<1, 3>(thisVertex, 0).transpose();		
	Eigen::Vector3d N_w = model.VN.block<1, 3>(thisVertex, 0).transpose();
	

	Eigen::Vector3d vertInCam = R_wc.transpose()*(P_w - C_w);											// ... in camera frame
	Eigen::Vector3d vertInImg = K*vertInCam;															// ... in image frame, homgeneous


	// check if vertex is in front of camera
	if (vertInCam(2) > 0) {

		// check if vertex projects into image
		if(vertInImg(0)>= -vertInImg(2) && vertInImg(0) <= vertInImg(2) && vertInImg(1) >= -vertInImg(2) && vertInImg(1) <= vertInImg(2)){
			// check if patch is reasonably facing the camera using surface normal

			if (N_w.normalized().dot((C_w - P_w).normalized()) > params.min_angle_view) {
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
		bool hasHit = igl::ray_mesh_intersect<Eigen::Vector3d, Eigen::Vector3d, Eigen::MatrixXd, Eigen::MatrixXi>(C_w, dir, model.V, model.F, hit);
		
		//check distance
		if (hasHit && hit.t < expDist - params.occlustion_hitpoint_max_distance)
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
	p_current = p - n*params.stepSize*params.nStepsDepthSearch/2; // start at negative position along normal
	double correlation;
	double weight =  images[view1].getViewQuality(p, n, images[view2]);
	
	model.nVertexObservations(vertex)+=weight; // needed for averaging

	/* 
	// for live plotting
	cv::Mat plot;
	plot.create(cv::Size(200, params.nStepsDepthSearch), 0);// = cv::Mat::zeros(100, params.nStepsDepthSearch);
	plot.setTo(255);
	cv::namedWindow("plot", cv::WINDOW_NORMAL);
	int temp;
	*/


	for (int i = 0; i < params.nStepsDepthSearch; i++) {

		p_current += params.stepSize*n;
		model.adjustmentScores(i, vertex) *= (model.nVertexObservations(vertex)-weight);
		correlation = images[view2].computeDistortedPatchCorrelation(images[view1], n, p_current, params.patch_size, 0);		//set last argument to zero: calculate with grayscale patches, otherwise with color
		model.adjustmentScores(i, vertex) += weight*correlation;
		
		if (model.nVertexObservations(vertex) > 0.00001) {
			model.adjustmentScores(i, vertex) /= (model.nVertexObservations(vertex));
		}

		/*
		// live plotting
		temp = (int)(200-correlation * 200);
		if (temp >= 200) temp = 199;
		if (temp <= 0) temp = 0;
		plot.at<uint8_t>(temp, i) = 0;
		imshow("plot", plot);
		cv::waitKey(1);
		*/
	} 
}

// This function computes adjustment scores for all vertices and pairs
int NativeRefiner::computeAdjustmentScores() {
	// loop through all vertices and images, find pairs and compute 
	model.adjustmentScores = Eigen::MatrixXd::Zero(params.nStepsDepthSearch, model.nVert);
	model.nVertexObservations = Eigen::VectorXd::Zero(model.nVert);

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

		 // Printing to file
		/*
		std::string pathOut = "C:/HappyBirthday/AdjScrV_";
		std::string suffixOut = ".txt";
		int frequency = 100;
		if (v % frequency == 0) {

			std::string nameOut = std::to_string(v);
			std::ofstream AdjScrV;
			AdjScrV.open(pathOut + nameOut + suffixOut, std::ios::app);

			if (AdjScrV.is_open()) {
				AdjScrV << "Adjustment scores without regularization" << std::endl;
				AdjScrV << model.adjustmentScores.col(v) << std::endl;
				AdjScrV.close();
			}
		}
		*/

		// regularization of mesh
		Eigen::Vector3d midPoint;
		bool isInside = model.computeCenter(v, midPoint);
		Eigen::Vector3d p = model.V.row(v).transpose();
		Eigen::Vector3d n = model.VN.row(v).transpose();
		Eigen::Vector3d p_current = p - n*params.stepSize*params.nStepsDepthSearch / 2;
		
		//marcs anti oscillation modification
		midPoint = 0.66*midPoint + 0.34*p;

		Eigen::Vector3d dtmp = midPoint - p;
		//std::cout << "Dist for Vertex " << v << ": "<< dtmp.norm()<< std::endl;	

		for (int i = 0; i < params.nStepsDepthSearch; i++) {
			p_current += params.stepSize*n;
			dtmp = midPoint - p_current;
			//std::cout << model.adjustmentScores(i, v);
			model.adjustmentScores(i, v) -= (isInside ? dtmp.squaredNorm()*params.smoothing_lambda : 0);
			//std::cout <<"/"<< model.adjustmentScores(i, v) << std::endl;
			//std::cout << (isInside ? dtmp.squaredNorm()*lambda : 0) << std::endl;
		}
		
		//std::cout << "Adjustment scores for Vertex " << v << " are \n" << model.adjustmentScores.block<51, 1>(0, v) << std::endl << std::endl;

		/*
		// File print
		if (v % frequency == 0) {
			std::string nameOut = std::to_string(v);
			std::ofstream AdjScrV;
			AdjScrV.open(pathOut + nameOut + suffixOut, std::ios::app);

			if (AdjScrV.is_open()) {
				AdjScrV << "Adjustment scores with regularization" << std::endl;
				AdjScrV << model.adjustmentScores.col(v) << std::endl;
				AdjScrV.close();
			}
		}
		*/

		// print progress
		if (v % 10 == 0) {
			progressPrint(v, model.nVert);
		}
	}

	progressPrint(1, 1);
	std::cout << "\nfinished computing adjustmentScores." << std::endl;		
	return 0;
}

int NativeRefiner::adjustVertices() {

	int nAdj = 0;
	for (int v = 0; v < model.nVert; v++) { //loop through all vertices
		int bestVertex = params.nStepsDepthSearch / 2;
		double bestScore = model.adjustmentScores(bestVertex, v); //initial score
		for (int i = 0; i < params.nStepsDepthSearch; i++) {
			if (model.adjustmentScores(i, v) > bestScore ) {
				bestScore = model.adjustmentScores(i, v);
				bestVertex = i;
			}
		}
		if (bestScore > model.adjustmentScores(params.nStepsDepthSearch / 2, v) + params.refineTolerance){//*pow(abs(bestVertex - params.nStepsDepthSearch) / 2, 1.2)) {
			model.V.block<1, 3>(v, 0) += params.stepSize*(bestVertex - params.nStepsDepthSearch / 2)*model.VN.block<1, 3>(v, 0);
			nAdj++;
			//std::cout << "adjusted Vertex " << v << " by " << (bestVertex - model.nStepsDepthSearch / 2) << std::endl;
		}
		if (v % 10 == 0) {
			progressPrint(v, model.nVert);
		}
	}
	progressPrint(1, 1);
	std::cout << "\nFinished adjusting Vertices.\nPercentage of adjusted vertices: " << 100*nAdj /model.nVert << "%" << std::endl;

	// debatable if we should recompute normals here
	model.computeNormals();
	return nAdj;
}

void NativeRefiner::progressPrint(int n, int m) {
	int barWidth = 70;
	double progress = (((double)n) / ((double)m));
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