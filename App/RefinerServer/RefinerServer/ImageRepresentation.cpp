#include "ImageRepresentation.h"
#include "stdafx.h"
#include <iostream>
#include <vector>




ImageRepresentation::ImageRepresentation(std::string filename,
	Eigen::Matrix4d pCameraViewTransform,
	Eigen::Matrix4d pCameraProjectionTransform) {
	// load parameters
	loadParams("params.txt", params);

	//load PNG
	std::vector<unsigned char> png;
	unsigned width, height;

	unsigned error = lodepng::load_file(png, filename);
	if (!error) error = lodepng::decode(image, width, height, png);	//needed?

	//prepare openCV buffers
	//ocvImage = cv::imread(filename, cv::ImreadModes::IMREAD_GRAYSCALE);
	ocvImage = cv::imread(filename, cv::ImreadModes::IMREAD_COLOR);			//BRG
	x_size = ocvImage.cols;
	y_size = ocvImage.rows; // really needed if we have width and height? (l. 12)

	if (params.gaussian > 0.001) {
		//gauss that shit
		cv::Mat newImg;
		cv::Size size;
		size.height = -1;
		size.width = -1;
		cv::GaussianBlur(ocvImage, newImg, size, params.gaussian);
		ocvImage = newImg;
	}

	if (params.downsample > 1) {
		cv::Size size;
		size.width = ocvImage.cols / params.downsample;
		size.height = ocvImage.rows / params.downsample;
		x_size /= params.downsample;
		y_size /= params.downsample;
		cv::Mat newImg;
		cv::resize(ocvImage, newImg, size);
		ocvImage = newImg;
	}

	/*CONVENTION:

							 | R11 R12 R13 C1 |		With:
	 CameraViewTransform =   | R21 R22 R23 C2 |				R mapping from CAMERA TO WORLD frame
							 | R31 R32 R33 C3 |				C camera coordinates in WORLD frame
							 | 0   0   0   1  |
	
	
								   | fx s  cx 0 |	
	 CameraProjectionTransform =   | 0	fy cy 0 |			Maps from CAMERA frame TO IMAGE coordinates (cu,cv,c,c). After normalization (*c^-1): (u,v) = (-1,-1) upper left, (u,v) = (1,1) lower right corner of image
								   | 0  0  1  0 |			Multiply by image resolution (2048x1152) to get pixel values: pix_u = (u+1)/2*2048, pix_v = (v+1)/2*1152
								   | 0  0  1  0 |
	*/

	//translate matrix to eigen

	/*
	CameraViewTransform <<  -pCameraViewTransform.m31, pCameraViewTransform.m32, pCameraViewTransform.m33,-pCameraViewTransform.m34,
							 pCameraViewTransform.m11,-pCameraViewTransform.m12,-pCameraViewTransform.m13, pCameraViewTransform.m14,
							 pCameraViewTransform.m21,-pCameraViewTransform.m22,-pCameraViewTransform.m23, pCameraViewTransform.m24,
							 pCameraViewTransform.m41, pCameraViewTransform.m42, pCameraViewTransform.m43, pCameraViewTransform.m44;*/

	
	CameraViewTransform << -pCameraViewTransform(2,0), pCameraViewTransform(2,1) , pCameraViewTransform(2,2), -pCameraViewTransform(2,3),
							pCameraViewTransform(0,0), -pCameraViewTransform(0,1), -pCameraViewTransform(0,2), pCameraViewTransform(0,3),
							pCameraViewTransform(1,0), -pCameraViewTransform(1,1), -pCameraViewTransform(1,2), pCameraViewTransform(1,3),
							pCameraViewTransform(3,0), pCameraViewTransform(3,1), pCameraViewTransform(3,2), pCameraViewTransform(3,3);

	/*std::cout << "\n";
	std::cout << "CVT" << "\n";
	std::cout << CameraViewTransform;
	std::cout << "\n";*/


	/*
	CameraProjectionTransform << pCameraProjectionTransform.m11, pCameraProjectionTransform.m12, -pCameraProjectionTransform.m13, pCameraProjectionTransform.m14,
								 pCameraProjectionTransform.m21, pCameraProjectionTransform.m22, -pCameraProjectionTransform.m23, pCameraProjectionTransform.m24,
								 pCameraProjectionTransform.m31, pCameraProjectionTransform.m32, -pCameraProjectionTransform.m33, pCameraProjectionTransform.m34,
								 pCameraProjectionTransform.m41, pCameraProjectionTransform.m42, -pCameraProjectionTransform.m43, pCameraProjectionTransform.m44;*/
	
	Eigen::Vector4d diag;
	diag << 1, 1, -1, -1;
	Eigen::Matrix4d diagM = diag.asDiagonal();
	CameraProjectionTransform = pCameraProjectionTransform*diagM;
	
}

//compute 3D projection of 2D point "pixel" onto surface defined by normal "surface_normal" and vertex "vertex"
Eigen::Vector3d ImageRepresentation::project2dto3d(Eigen::Vector3d surface_normal, Eigen::Vector3d vertex, cv::Point2d pixel){

	// get ex- & intrinsics of camera
	Eigen::Matrix3d R_wc = CameraViewTransform.block<3,3>(0,0).cast <double>();			// camera orientation matrix (camera to world)
	Eigen::Vector3d C_w = CameraViewTransform.block<3, 1>(0, 3).cast <double>();		// camera position in world frame (expressed in world frame)
	Eigen::Matrix3d K = CameraProjectionTransform.block<3,3>(0,0).cast <double>();		// camera calibration matrix

	// Relative pixel position
	double u = pixel.x /((double) x_size) * 2.0 - 1.0;									// from pixel values to relative values following the convention
	double v = pixel.y /((double) y_size) * 2.0 - 1.0;
	Eigen::Vector3d imgCoord(u, v, 1);													// homogenize

	// compute intersection point of viewing ray and plane
	Eigen::Vector3d p_temp = K.inverse()*imgCoord;										// image plane to camera frame
	Eigen::Vector3d p_w = R_wc*p_temp;													// camera frame to world frame

	double lambda = (vertex - C_w).dot(surface_normal) / (p_w.dot(surface_normal));
	Eigen::Vector3d P_w = C_w + lambda*p_w;												//point in world frame
	
	return P_w;
}

//Compute corrsponding patch in camera 2 given the patch in camera 1 (via reprojection and projective unwarping)
double ImageRepresentation::computeDistortedPatchCorrelation(ImageRepresentation& image2, Eigen::Vector3d surface_normal, Eigen::Vector3d vertex, cv::Size patch_size, int colorFlag) {

	
	// Nomencalture: 
	// hpoint:	homogenous 2D image plane point (->3D)
	// hPoint:	homgeneous 3D worl frame point (->4D)
	// p:		2D image plane point (pixel coordinates)
	// P:		3D world frame point		 
	// T_ab:	3x4 matrix describing transformation from frame b to frame a
	// K:		camera calibration matrix

	// images of camera 1 and camera 2
	cv::Mat img_c1 = ocvImage ;
	cv::Mat img_c2 = image2.ocvImage;
	cv::Mat patch1 = cv::Mat::zeros(patch_size,  img_c1.type());
	cv::Mat patch2 = cv::Mat::zeros(patch_size, img_c1.type());
	std::vector<cv::Mat> patch1BRG;
	std::vector<cv::Mat> patch2BRG;

	// get ex- & intrinsics of camera
	Eigen::Matrix<double, 3, 3> R_wc1 = CameraViewTransform.block<3, 3>(0, 0).cast <double>();			// camera1 orientation matrix (camera to world)
	Eigen::Vector3d C1_w = CameraViewTransform.block<3, 1>(0, 3).cast <double>();						// camera1 position in world frame (expressed in world frame)
	
	Eigen::Matrix<double, 3, 3> R_wc2 = image2.CameraViewTransform.block<3, 3>(0, 0).cast <double>();	// camera2 orientation matrix (camera to world)
	Eigen::Vector3d C2_w = image2.CameraViewTransform.block<3, 1>(0, 3).cast <double>();				// camera1 position in world frame (expressed in world frame)
	
	Eigen::Matrix3d K1 = CameraProjectionTransform.block<3, 3>(0, 0).cast <double>();					// camera1 calibration matrix 
	Eigen::Matrix3d K2 = image2.CameraProjectionTransform.block<3, 3>(0, 0).cast <double>();			// camera2 calibration matrix 

	// computing center point of patch (projection of vertex into image 1)
	Eigen::Vector3d vertInImg = K1*R_wc1.transpose()*(vertex - C1_w);									// projecting vertex into image 1, step 1												
	double u = vertInImg(0) / vertInImg(2);																// u-coordinates of vertex projected into image1 ...
	double v = vertInImg(1) / vertInImg(2);																// v-coordinates of vertex projected into image1 ...
	double pix_u = (u + 1) / 2 * x_size;																	// ... expressed in pixel coordinates
	double pix_v = (v + 1) / 2 * y_size;																	// ... expressed in pixel coordinates


		// check if out-of-bounds (conservative - could do striclty less-than)
	if (pix_u <= patch_size.width / 2 || pix_v <= patch_size.height / 2 || 
		pix_u >= x_size - patch_size.width / 2 || pix_v >= y_size - patch_size.height / 2){
		return 0.0;
	}
	
	
	// computing corners of patch in image 1 (homogeneous coordinates)
	cv::Point2d p1_c1(pix_u + patch_size.width / 2, pix_v + patch_size.height / 2);						// patch in image 1, lower right corner
	cv::Point2d p2_c1(pix_u - patch_size.width / 2, pix_v + patch_size.height / 2);						// patch in image 1, lower left corner
	cv::Point2d p3_c1(pix_u + patch_size.width / 2, pix_v - patch_size.height / 2);						// patch in image 1, upper right corner
	cv::Point2d p4_c1(pix_u - patch_size.width / 2, pix_v - patch_size.height / 2);						// patch in image 1, upper left corner

	// computing 3D projections of patch corners
	Eigen::Vector3d P1_w = project2dto3d(surface_normal, vertex, p1_c1);									// Corner points of projected patch from image 1 in world frame
	Eigen::Vector3d P2_w = project2dto3d(surface_normal, vertex, p2_c1);
	Eigen::Vector3d P3_w = project2dto3d(surface_normal, vertex, p3_c1);
	Eigen::Vector3d P4_w = project2dto3d(surface_normal, vertex, p4_c1);

	// compute projection of 3d points into image 2 (homgeneous coordinates) ...
	Eigen::Vector3d hp1_c2 = K2*R_wc2.transpose()*(P1_w - C2_w);
	Eigen::Vector3d hp2_c2 = K2*R_wc2.transpose()*(P2_w - C2_w);
	Eigen::Vector3d hp3_c2 = K2*R_wc2.transpose()*(P3_w - C2_w);
	Eigen::Vector3d hp4_c2 = K2*R_wc2.transpose()*(P4_w - C2_w);

	// and normalize them to get the pixel coordinates which define the source frame for the perspective transform	
	cv::Point2f p_c2[4];
	p_c2[0] = cv::Point2f((hp1_c2(0) / hp1_c2(2) + 1) / 2 * x_size, (hp1_c2(1) / hp1_c2(2) + 1) / 2 * y_size);			// source frame for perspective transform, i.e. warped patch in image 2
	p_c2[1] = cv::Point2f((hp2_c2(0) / hp2_c2(2) + 1) / 2 * x_size, (hp2_c2(1) / hp2_c2(2) + 1) / 2 * y_size);
	p_c2[2] = cv::Point2f((hp3_c2(0) / hp3_c2(2) + 1) / 2 * x_size, (hp3_c2(1) / hp3_c2(2) + 1) / 2 * y_size);
	p_c2[3] = cv::Point2f((hp4_c2(0) / hp4_c2(2) + 1) / 2 * x_size, (hp4_c2(1) / hp4_c2(2) + 1) / 2 * y_size);


	// check if second patch runs out of frame
	for (int i = 0; i < 4; i++) {
		if (p_c2[i].x <= patch_size.width / 2 || p_c2[i].y <= patch_size.height / 2 ||
			p_c2[i].x >= x_size - patch_size.width / 2 || p_c2[i].y >= y_size - patch_size.height / 2)
			return 0.0;
	}

	// preparing target frame for perspective transform
	cv::Point2f p_c1[4];
	p_c1[0] = cv::Point2f(patch_size.width, patch_size.height);											// target frame for perspective transform, lower right corner
	p_c1[1] = cv::Point2f(0, patch_size.height);
	p_c1[2] = cv::Point2f(patch_size.width, 0);
	p_c1[3] = cv::Point2f(0, 0);

	// compute perspective/affine transformation of new patch...
	cv::Mat M = cv::Mat::zeros(3, 3, img_c1.type());
	M = cv::getPerspectiveTransform(p_c2, p_c1);

	// and apply perspective transform to "undistort" patch in image 2 by mapping it onto target frame
	// yields patch2
	cv::warpPerspective(img_c2, patch2, M, patch2.size());						// extracting and undistorting patch2 from img_c2
	cv::Rect patch(p4_c1.x, p4_c1.y, patch_size.width, patch_size.height);		
	
	patch1 = cv::Mat(img_c1, patch);											// extracting patch 1 from img_c1	

	float correlation = 0.0;
	
	if (colorFlag == 0) {			//Grayscale
		cv::Mat correlationMat;
		cv::matchTemplate(patch1, patch2, correlationMat, cv::TemplateMatchModes::TM_CCORR_NORMED);
		correlation = correlationMat.at<float>(0, 0);

	}
	else {							//BRG

		cv::Mat correlationMat[3];

		cv::split(patch1, patch1BRG);
		cv::split(patch2, patch2BRG);


		if (params.fullnormalize) {
			//test it with proper normalization
			for (int i = 0; i < 3; i++) {
				cv::Scalar avg, sdv;
				cv::meanStdDev(patch1BRG[i], avg, sdv);
				cv::Mat image_32f;
				patch1BRG[i].convertTo(image_32f, CV_32F, 1 / sdv.val[0], -avg.val[0] / sdv.val[0]);
				patch1BRG[i] = image_32f;
			}

			//test it with proper normalization
			for (int i = 0; i < 3; i++) {
				cv::Scalar avg, sdv;
				cv::meanStdDev(patch2BRG[i], avg, sdv);
				cv::Mat image_32f;
				patch2BRG[i].convertTo(image_32f, CV_32F, 1 / sdv.val[0], -avg.val[0] / sdv.val[0]);
				patch2BRG[i] = image_32f;
			}
		}


		cv::matchTemplate(patch1BRG[0], patch2BRG[0], correlationMat[0], cv::TemplateMatchModes::TM_CCOEFF_NORMED);		//Blue channel
		cv::matchTemplate(patch1BRG[1], patch2BRG[1], correlationMat[1], cv::TemplateMatchModes::TM_CCOEFF_NORMED);		//Red channel
		cv::matchTemplate(patch1BRG[2], patch2BRG[2], correlationMat[2], cv::TemplateMatchModes::TM_CCOEFF_NORMED);		//Green channel

		//V1: Mean
		//correlation = (correlationMat[0].at<float>(0, 0) + correlationMat[1].at<float>(0, 0) + correlationMat [2].at<float>(0, 0)) / 3.0;
		
		//V2: Multiply
		correlation = (correlationMat[0].at<float>(0, 0) * correlationMat[1].at<float>(0, 0) * correlationMat[2].at<float>(0, 0));

	}

	// display images and patches, print stuff
	if (params.liveview) {
		cv::Mat left = img_c1.clone();
		cv::Mat right = img_c2.clone();
		std::cout << "Correlation is: " << correlation << std::endl;
		cv::namedWindow("img1", cv::WINDOW_NORMAL);
		cv::namedWindow("img2", cv::WINDOW_NORMAL);
		cv::namedWindow("patch1", CV_WINDOW_AUTOSIZE);
		cv::namedWindow("patch2", CV_WINDOW_AUTOSIZE);
		cv::rectangle(left, patch, cv::Scalar(250, 255, 255), 5, 8, 0);
		std::vector< cv::Point> contour;
		contour.push_back(cv::Point((int)p_c2[0].x, (int)p_c2[0].y));
		contour.push_back(cv::Point((int)p_c2[2].x, (int)p_c2[2].y));
		contour.push_back(cv::Point((int)p_c2[3].x, (int)p_c2[3].y));
		contour.push_back(cv::Point((int)p_c2[1].x, (int)p_c2[1].y));
		const cv::Point *pts = (const cv::Point*) cv::Mat(contour).data;
		int npts = cv::Mat(contour).rows;
		polylines(right, &pts, &npts, 1, true, cv::Scalar(255, 255, 255), 5, 8, 0);
		cv::resize(patch1, patch1, cv::Size(150, 150));
		cv::resize(patch2, patch2, cv::Size(150, 150));
		cv::imshow("img2", right);
		cv::imshow("img1", left);
		cv::imshow("patch1", patch1);
		cv::imshow("patch2", patch2);
		cv::waitKey(1);
	}
	

	return (double)correlation;
}


double ImageRepresentation::getViewQuality(Eigen::Vector3d vertex, Eigen::Vector3d normal, ImageRepresentation& image2) {

	Eigen::Vector3d VtoC1 = CameraViewTransform.block<3, 1>(0, 3).cast <double>() - vertex;						
	Eigen::Vector3d VtoC2 = image2.CameraViewTransform.block<3, 1>(0, 3).cast <double>() - vertex;

	
	double cosAngle = double(VtoC1.normalized().dot(VtoC2.normalized()));
	double angle = 4*acos(cosAngle);

	if (cosAngle <= 0) return 0;	

	double weight = (-cos(angle) + 1) / 2;	//viewing angle 45� -> weight 1, viewing angle 0� or >=90� -> weight 0

	weight *= VtoC1.normalized().dot(normal)*VtoC2.normalized().dot(normal);

	return weight;
	

}

