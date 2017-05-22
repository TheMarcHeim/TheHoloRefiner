#include "NativeRefiner.h"
#include "stdafx.h"
#include <iostream>
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

void NativeRefiner::saveRefinedModel(std::string path)
{
	model.saveFile(path);
}

std::string NativeRefiner::refine(int nReps)
{

	std::cout << "Number of before subdivision vertices: " << model.nVert << std::endl;
	//model.subDivide();
	std::cout << "Number of vertices after subdivision: " << model.nVert << std::endl;
	int nAdj;

	for(int i=0; i<nReps; i++){
		std::cout << "Computing Visibility..." << std::endl;
		computeVisibility();
		std::cout << "Computing Adjustment Scores..." << std::endl;
		computeAdjustmentScores();
		nAdj = adjustVertices();
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
	//open viewer
	
	visibility = Eigen::MatrixXi::Zero(model.nVert, nImages);		//Binary matrix indicating if a vertex v is seen in image i, rows: vertices, columns: images 
	int nVis = 0;
	for (int v = 0; v < model.nVert; v++) {
		for (int i = 0; i < nImages; i++) {	
				if (isVisible(v, i)) {
					visibility(v, i) = 1;	//rows: vertices, columns: images
					nVis++;
				}			
		}
	}

	std::cout << "finished computing visibility. Number of visible vertices: " << nVis << std::endl;
	
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
	
	//Eigen::Vector3d P_w = model.V.block<1, 3>(thisVertex, 0).transpose();								// -> in form (y,z,x), we don't want this
	Eigen::Vector3d P_w(model.V(thisVertex,2), model.V(thisVertex, 0), model.V(thisVertex, 1));			// Instead, permute to get it in form (x,y,z) and to be consistent with new convention.
	Eigen::Vector3d N_w(model.VN(thisVertex, 2), model.VN(thisVertex, 0), model.VN(thisVertex, 1));		// Same for normals. IDEALLY we could reformat everything directly when loading the model TBD
	
	
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
		//expected distance
		double expDist = camToPos.norm();
		//double expDist2 = sqrt(camToPos(0)*camToPos(0) + camToPos(1)*camToPos(1) + camToPos(2)*camToPos(2));
		//first hit
		igl::Hit hit;
		const double tolerance = 0.05;
		bool hasHit = igl::ray_mesh_intersect<Eigen::Vector3d, Eigen::Vector3d, Eigen::MatrixXd, Eigen::MatrixXi>(C_w, dir, model.CorrectV, model.F, hit);
		
		//check distance
		if (hasHit && hit.t < expDist - tolerance)
		{
			visible = false;
			//check hits
			/*
			Eigen::Vector3d pos1 = C_w + dir*hit.t;
			int VID = model.F(hit.id, 0);
			int VID1 = model.F(hit.id, 1);
			int VID2 = model.F(hit.id, 2);
			Eigen::Vector3d pos2 = model.CorrectV.row(VID);
			*/
		}
	}
	
	return visible;
}

// This function computes adjustment scores for just one given vertex
void NativeRefiner::computeVertexAdjustmentScores(int vertex, int view1, int view2) {

	Eigen::Vector3d p(0, 0, 0);
	Eigen::Vector3d n(0, 0, 0);
	Eigen::Vector3d p_current(0, 0, 0);

	p << model.V(vertex, 2), model.V(vertex, 0), model.V(vertex, 1);	// Note permutation required to be consistent with new convention

	n << model.VN(vertex,2), model.VN(vertex,0), model.VN(vertex,1);	// Haven't yet checked whether these coordinates need to be permuted as well - but 
																		// it is very unlikely that they use different conventions in the same .obj file ...
	p_current = p - n*model.stepSize*model.nStepsDepthSearch/2;			// start at negative position along normal

	float weight =  images[view1].getViewQuality(p, n, images[view2]);

	model.nVertexObservations(vertex)+=weight; // needed for averaging

	

	for (int i = 0; i < model.nStepsDepthSearch; i++) {
		p_current += model.stepSize*n;
		//regularization Term


		//std::cout <<"match: "<< images[view2].computeDistortedPatchCorrelation(images[view1], n, p_current, patch_size, 0)<<std::endl;
		//std::cout << "regularization: " << -(isInside ? dtmp.squaredNorm()*lambda : 0) << std::endl;
		model.adjustmentScores(i, vertex) *= (model.nVertexObservations(vertex)-weight);
		model.adjustmentScores(i, vertex) += weight*images[view2].computeDistortedPatchCorrelation(images[view1], n, p_current, patch_size, 0);		//set last argument to zero: calculate with grayscale patches, otherwise with color
		model.adjustmentScores(i, vertex) /= (model.nVertexObservations(vertex));
	} 



	//std::cout << "Adjustment scores for Vertex " << vertex << " are \n" << model.adjustmentScores.block<21, 1>(0, vertex) << std::endl << std::endl;
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

		const double lambda = 100000;
		Eigen::Vector3d midPoint;
		bool isInside = model.computeCenter(v, midPoint);

		Eigen::Vector3d p(model.V(v, 2), model.V(v, 0), model.V(v, 1));
		Eigen::Vector3d n(model.VN(v, 2), model.VN(v, 0), model.VN(v, 1));
		Eigen::Vector3d p_current = p - n*model.stepSize*model.nStepsDepthSearch / 2;
		Eigen::Vector3d dtmp = midPoint - p_current;

		for (int i = 0; i < model.nStepsDepthSearch; i++) {

			p_current += model.stepSize*n;
			dtmp = midPoint - p_current;
			model.adjustmentScores[i, v] += (isInside ? dtmp.squaredNorm()*lambda : 0);

		}
	}

	
	std::cout << "finished computing adjustmentScores." << std::endl;
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
		if (bestScore > model.adjustmentScores(model.nStepsDepthSearch / 2, v) + model.refineTolerance*pow(bestVertex - model.nStepsDepthSearch / 2, 2)) {
			model.V.block<1, 3>(v, 0) += model.stepSize*(bestVertex - model.nStepsDepthSearch / 2)*model.VN.block<1, 3>(v, 0);
			nAdj++;
			//std::cout << "adjusted Vertex " << v << " by " << (bestVertex - model.nStepsDepthSearch / 2) << std::endl;
		}
	}
	model.computeNormals();
	return nAdj;
}