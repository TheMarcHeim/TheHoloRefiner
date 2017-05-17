#include "ImageRepresentation.h"
#include "stdafx.h"
#include <vector>


ImageRepresentation::ImageRepresentation(std::string filename,
	Eigen::Matrix4f pCameraViewTransform,
	Eigen::Matrix4f pCameraProjectionTransform)
	:filename(filename){
	//load PNG
	std::vector<unsigned char> png;
	unsigned width, height;

	unsigned error = lodepng::load_file(png, filename);
	if (!error) error = lodepng::decode(image, width, height, png);	//needed?

	//prepare openCV buffers
	ocvImage = cv::imread(filename, cv::ImreadModes::IMREAD_GRAYSCALE);
	x_size = ocvImage.cols;
	y_size = ocvImage.rows; // really needed if we have width and height? (l. 12)


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

	/*
	CameraProjectionTransform << pCameraProjectionTransform.m11, pCameraProjectionTransform.m12, -pCameraProjectionTransform.m13, pCameraProjectionTransform.m14,
								 pCameraProjectionTransform.m21, pCameraProjectionTransform.m22, -pCameraProjectionTransform.m23, pCameraProjectionTransform.m24,
								 pCameraProjectionTransform.m31, pCameraProjectionTransform.m32, -pCameraProjectionTransform.m33, pCameraProjectionTransform.m34,
								 pCameraProjectionTransform.m41, pCameraProjectionTransform.m42, -pCameraProjectionTransform.m43, pCameraProjectionTransform.m44;*/
	
	Eigen::Vector4f diag;
	diag << 1, 1, -1, -1;
	Eigen::Matrix4f diagM = diag.asDiagonal();
	CameraProjectionTransform = pCameraProjectionTransform*diagM;

}

//compute 3D projection of 2D point "pixel" onto surface defined by normal "surface_normal" and vertex "vertex"
Eigen::Vector3d ImageRepresentation::project2dto3d(Eigen::Vector3d surface_normal, Eigen::Vector3d vertex, cv::Point2d pixel){

	// get ex- & intrinsics of camera
	Eigen::Matrix3d R_wc = CameraViewTransform.block<3,3>(0,0).cast <double>();			// camera orientation matrix (camera to world)
	Eigen::Vector3d C_w = CameraViewTransform.block<3, 1>(0, 3).cast <double>();		// camera position in world frame (expressed in world frame)
	Eigen::Matrix3d K = CameraProjectionTransform.block<3,3>(0,0).cast <double>();		// camera calibration matrix

	// Relative pixel position
	double u = pixel.x /((double) x_size) * 2.0 - 1.0;											// from pixel values to relative values following the convention
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
float ImageRepresentation::computeDistortedPatchCorrelation(ImageRepresentation& image2, Eigen::Vector3d surface_normal, Eigen::Vector3d vertex, cv::Size patch_size) {

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
		pix_u >= x_size - patch_size.width / 2 || pix_v >= y_size - patch_size.height / 2)
		return 0;

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
	p_c2[0] = cv::Point2f((hp1_c2(0) / hp1_c2(2) + 1) / 2 * x_size, (hp1_c2(1) / hp1_c2(2) + 1) / 2 * y_size);								// source frame for perspective transform, i.e. warped patch in image 2
	p_c2[1] = cv::Point2f((hp2_c2(0) / hp2_c2(2) + 1) / 2 * x_size, (hp2_c2(1) / hp2_c2(2) + 1) / 2 * y_size);
	p_c2[2] = cv::Point2f((hp3_c2(0) / hp3_c2(2) + 1) / 2 * x_size, (hp3_c2(1) / hp3_c2(2) + 1) / 2 * y_size);
	p_c2[3] = cv::Point2f((hp4_c2(0) / hp4_c2(2) + 1) / 2 * x_size, (hp4_c2(1) / hp4_c2(2) + 1) / 2 * y_size);

	

	

	// preparing target frame for perspective transform
	cv::Point2f p_c1[4];
	p_c1[0] = cv::Point2f(patch_size.width, patch_size.height);											// target frame for perspective transform, lower right corner
	p_c1[1] = cv::Point2f(0, patch_size.height);
	p_c1[2] = cv::Point2f(patch_size.width, 0);
	p_c1[3] = cv::Point2f(0, 0);

	// compute perspective/affine transformation of new patch...
	// http://opencvexamples.blogspot.com/2014/01/perspective-transform.html 
	// Nico's comment: I disagree... according to docs.opencv.org M should be 3x3. In the above link it is initialized as 2x4..?
	// David's comment: ... it is initialized as 2x4 but then overwritten in line 20
	//cv::Mat M = cv::Mat::zeros(img_c1.rows, img_c1.cols, img_c1.type());								// transformation matrix
	cv::Mat M = cv::Mat::zeros(3, 3, img_c1.type());
	M = cv::getPerspectiveTransform(p_c2, p_c1);

	// and apply perspective transform to "undistort" patch in image 2 by mapping it onto target frame
	// cv::warpPerspective(img_c2, output, M, patch_size);												// POTENTIAL ERROR: last argument should be of cv-type "size"
	cv::warpPerspective(img_c2, patch2, M, patch2.size());												// POTENTIAL SOLUTION:

	// TO DO: (because our approach changed after the discussion with Torsten...)
	// So far, whoever calls this function has to calculate the patch in image 1 himself to correlate it with the one returned by this function - which is inefficient since we are doing the same at the beginning of this function as well...
	// Suggestion: return (either explicit or by reference) both the patch of image 1 and the corresponding patch of image 2. We could also extend this function to also do the correlation 
	// between the patches ...
	cv::Rect patch(p4_c1.x, p4_c1.y, patch_size.width, patch_size.height);
	cv::Mat correlation;
	patch1 = cv::Mat(img_c1, patch);
	cv::matchTemplate(patch1, patch2, correlation, cv::TemplateMatchModes::TM_CCORR_NORMED);

	// display images and patches, print stuff
	/*
	std::cout << "M is \n " << M << std::endl;
	std::cout << "Correlation is: " << correlation.at<float>(0, 0) << std::endl;
	cv::namedWindow("img1", cv::WINDOW_NORMAL);
	cv::namedWindow("img2", cv::WINDOW_NORMAL);
	cv::namedWindow("patch1", CV_WINDOW_AUTOSIZE);
	cv::namedWindow("patch2", CV_WINDOW_AUTOSIZE);
	cv::rectangle(img_c1, patch, cv::Scalar(250, 255, 255), 5, 8, 0);
	std::vector< cv::Point> contour;
	contour.push_back(cv::Point((int)p_c2[0].x, (int)p_c2[0].y));
	contour.push_back(cv::Point((int)p_c2[2].x, (int)p_c2[2].y));
	contour.push_back(cv::Point((int)p_c2[3].x, (int)p_c2[3].y));
	contour.push_back(cv::Point((int)p_c2[1].x, (int)p_c2[1].y));
	const cv::Point *pts = (const cv::Point*) cv::Mat(contour).data;
	int npts = cv::Mat(contour).rows;
	polylines(img_c2, &pts, &npts, 1, true, cv::Scalar(255, 255, 255), 5, 8, 0);
	//cv::resize(patch1, patch1, cv::Size(output.cols, output.rows));
	//cv::resize(patch2, patch2, cv::Size(patch2.cols, patch2.rows));
	cv::imshow("img2", img_c2);
	cv::imshow("img1", img_c1);
	cv::imshow("patch1", patch1);
	cv::imshow("patch2", patch2);
	cv::waitKey(1);
	*/
	

	return correlation.at<float>(0,0);
}
