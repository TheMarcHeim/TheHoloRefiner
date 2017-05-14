#include "pch.h"
#include "ImageRepresentation.h"
#include <iostream>


imageRep::ImageRepresentation::ImageRepresentation(std::string filename,
	Windows::Foundation::Numerics::float4x4 pCameraViewTransform,
	Windows::Foundation::Numerics::float4x4 pCameraProjectionTransform)
	:filename(filename){
	//load PNG
	std::vector<unsigned char> png;
	unsigned width, height;

	unsigned error = lodepng::load_file(png, filename);
	if (!error) error = lodepng::decode(image, width, height, png);

	//prepare openCV buffers
	ocvImage = cv::imread(filename, cv::ImreadModes::IMREAD_GRAYSCALE);
	x_size = ocvImage.cols;
	y_size = ocvImage.rows;


	//set ksize = 5 for local minima prevention
	//one for x and y
	cv::Sobel(ocvImage, dXImg, -1, 1, 0, 5);
	cv::Sobel(ocvImage, dYImg, -1, 0, 1, 5);


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
	CameraViewTransform <<  -pCameraViewTransform.m31, pCameraViewTransform.m32, pCameraViewTransform.m33,-pCameraViewTransform.m34,
							 pCameraViewTransform.m11,-pCameraViewTransform.m12,-pCameraViewTransform.m13, pCameraViewTransform.m14,
							 pCameraViewTransform.m21,-pCameraViewTransform.m22,-pCameraViewTransform.m23, pCameraViewTransform.m24,
							 pCameraViewTransform.m41, pCameraViewTransform.m42, pCameraViewTransform.m43, pCameraViewTransform.m44;

	CameraProjectionTransform << pCameraProjectionTransform.m11, pCameraProjectionTransform.m12, -pCameraProjectionTransform.m13, pCameraProjectionTransform.m14,
								 pCameraProjectionTransform.m21, pCameraProjectionTransform.m22, -pCameraProjectionTransform.m23, pCameraProjectionTransform.m24,
								 pCameraProjectionTransform.m31, pCameraProjectionTransform.m32, -pCameraProjectionTransform.m33, pCameraProjectionTransform.m34,
								 pCameraProjectionTransform.m41, pCameraProjectionTransform.m42, -pCameraProjectionTransform.m43, pCameraProjectionTransform.m44;

}

Eigen::Vector2f imageRep::ImageRepresentation::imageSpaceGradientCompare(ImageRepresentation& otherImage, Eigen::Vector2f ownPos, Eigen::Vector2f otherPos, int patchSize)
{
	//prepare patch rectangles
	int length = (patchSize - 1) << 1;
	cv::Rect ownRect(ownPos[0]- length, ownPos[1]-length, patchSize, patchSize);
	cv::Rect otherRect(otherPos[0] - length, otherPos[0] - length, patchSize, patchSize);
	cv::Mat XXX, YYY;
	//exploit matchTemplate function:
	//when used in this way it computes the derivation in x and y directions
	cv::matchTemplate(
		cv::Mat(dXImg, ownRect),
		cv::Mat(otherImage.ocvImage, otherRect),
		XXX, cv::TemplateMatchModes::TM_CCORR);
	cv::matchTemplate(
		cv::Mat(dYImg, ownRect),
		cv::Mat(otherImage.ocvImage, otherRect),
		YYY, cv::TemplateMatchModes::TM_CCORR);
	//extract eigen gradient vector
	//TODO: maybe change signs :P
	return Eigen::Vector2f(
		-XXX.at<float>(0,0),
		-YYY.at<float>(0,0));
}

void imageRep::ImageRepresentation::setPositions(Eigen::MatrixXd V)
{
	//problem: CameraProjectionTransform and CameraViewTransform are float matrices...
	//TODO: convert them to double
	//Vpos = (CameraProjectionTransform*CameraViewTransform*V.transpose()).transpose();
}

//compute 3D projection of 2D point "pixel" onto surface defined by normal "surface_normal" and vertex "vertex"
Eigen::Vector3d imageRep::ImageRepresentation::project2dto3d(Eigen::Vector3d surface_normal, Eigen::Vector3d vertex, cv::Point2d pixel){

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
float imageRep::ImageRepresentation::computeDistortedPatchCorrelation(ImageRepresentation& image2, Eigen::Vector3d surface_normal, Eigen::Vector3d vertex, cv::Size patch_size) {

	// Nomencalture: 
	// hpoint:	homogenous 2D image plane point (->3D)
	// hPoint:	homgeneous 3D worl frame point (->4D)
	// p:		2D image plane point (pixel coordinates)
	// P:		3D world frame point		 
	// T_ab:	3x4 matrix describing transformation from frame b to frame a
	// K:		camera calibration matrix

	// images of camera 1 and camera 2
	cv::Mat img_c1 = ocvImage;
	cv::Mat img_c2 = image2.ocvImage;
	cv::Mat output;

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
	p_c2[0] = cv::Point2f((hp1_c2(0) / hp1_c2(2) + 1) / 2 * 2048, (hp1_c2(1) / hp1_c2(2) + 1) / 2 * 1152);								// source frame for perspective transform, i.e. warped patch in image 2
	p_c2[1] = cv::Point2f((hp2_c2(0) / hp2_c2(2) + 1) / 2 * 2048, (hp2_c2(1) / hp2_c2(2) + 1) / 2 * 1152);
	p_c2[2] = cv::Point2f((hp3_c2(0) / hp3_c2(2) + 1) / 2 * 2048, (hp3_c2(1) / hp3_c2(2) + 1) / 2 * 1152);
	p_c2[3] = cv::Point2f((hp4_c2(0) / hp4_c2(2) + 1) / 2 * 2048, (hp4_c2(1) / hp4_c2(2) + 1) / 2 * 1152);

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
	cv::Mat M = cv::Mat::zeros(img_c1.rows, img_c1.cols, img_c1.type());								// transformation matrix
	M = cv::getPerspectiveTransform(p_c2, p_c1);

	// and apply perspective transform to "undistort" patch in image 2 by mapping it onto target frame
	// cv::warpPerspective(img_c2, output, M, patch_size);												// POTENTIAL ERROR: last argument should be of cv-type "size"
	cv::warpPerspective(img_c2, output, M, output.size());												// POTENTIAL SOLUTION:

	// TO DO: (because our approach changed after the discussion with Torsten...)
	// So far, whoever calls this function has to calculate the patch in image 1 himself to correlate it with the one returned by this function - which is inefficient since we are doing the same at the beginning of this function as well...
	// Suggestion: return (either explicit or by reference) both the patch of image 1 and the corresponding patch of image 2. We could also extend this function to also do the correlation 
	// between the patches ...
	cv::Rect patch(p4_c1.x, p4_c1.y, patch_size.width, patch_size.height);
	cv::Mat correlation;
	cv::matchTemplate(cv::Mat(img_c1, patch), output, correlation, cv::TemplateMatchModes::TM_CCORR_NORMED);
	return correlation.at<float>(0,0);
}
