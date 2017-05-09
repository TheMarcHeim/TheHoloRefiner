#include "pch.h"
#include "ImageRepresentation.h"


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


	//translate matrix to eigen
	CameraViewTransform << pCameraViewTransform.m11, pCameraViewTransform.m12, pCameraViewTransform.m13, pCameraViewTransform.m14,
		pCameraViewTransform.m21, pCameraViewTransform.m22, pCameraViewTransform.m23, pCameraViewTransform.m24,
		pCameraViewTransform.m31, pCameraViewTransform.m32, pCameraViewTransform.m33, pCameraViewTransform.m34,
		pCameraViewTransform.m41, pCameraViewTransform.m42, pCameraViewTransform.m43, pCameraViewTransform.m44;

	CameraProjectionTransform << pCameraProjectionTransform.m11, pCameraProjectionTransform.m12, pCameraProjectionTransform.m13, pCameraProjectionTransform.m14,
		pCameraProjectionTransform.m21, pCameraProjectionTransform.m22, pCameraProjectionTransform.m23, pCameraProjectionTransform.m24,
		pCameraProjectionTransform.m31, pCameraProjectionTransform.m32, pCameraProjectionTransform.m33, pCameraProjectionTransform.m34,
		pCameraProjectionTransform.m41, pCameraProjectionTransform.m42, pCameraProjectionTransform.m43, pCameraProjectionTransform.m44;
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

//compute 3D projection of 2D point "pixel" onto surface defined by normal "surface_normal" and vertex "vertex" (Raycasting ?)
Eigen::Vector3d imageRep::ImageRepresentation::project2dto3d(Eigen::Vector3d surface_normal, Eigen::Vector3d vertex, Eigen::Vector3d pixel){

	// get orientation of camera
	Eigen::Matrix3d R_cw = CameraViewTransform.block<3,3>(0,0).cast <double>();			// camera orientation matrix (maps from world CS to camera CS, expressed in world coordinates)
	Eigen::Vector3d C = CameraViewTransform.block<3, 1>(0, 3).cast <double>();			// camera position in world frame (expressed in world coordinates)
	Eigen::Matrix3d K = CameraProjectionTransform.block<3,3>(0,0).cast <double>();		// camera calibration matrix

	//get image coordinates of point
	Eigen::Vector3d p_temp = K.inverse()*pixel;											// projecting vector in camera frame
	Eigen::Vector3d p = R_cw.transpose()*p_temp;										// projecting vector in world frame
	
	// compute scale
	double lambda = (vertex - C).dot(surface_normal)/(p.dot(surface_normal));
	
	// compute world frame points
	Eigen::Vector3d P = C + lambda*p; //point in world frame
	
	return P;
}

//Compute corrsponding patch in camera 2 given the patch in camera 1 (via reprojection and projective unwarping)
void imageRep::ImageRepresentation::computeDistortedPatch(cv::Mat& output, ImageRepresentation& image2, Eigen::Vector3d surface_normal, Eigen::Vector3d vertex, cv::Size patch_size){
	
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
	
	// homogenize vertex
	Eigen::Vector4d hVertex(vertex(1), vertex(2), vertex(3), 1);										// POTENTIAL ERROR: do we require 1 or -1 at last entry? In other words, does TX_cw
	
	// computing center point of patch (projection of vertex into image 1)
	Eigen::Matrix<double, 3, 4> T1_cw = CameraViewTransform.block<3, 4>(0, 0).cast <double>();			// camera transform matrix of camera 1
	Eigen::Matrix<double, 3, 4> T2_cw = image2.CameraViewTransform.block<3, 4>(0, 0).cast <double>();	// camera transform matrix of camera 2
	Eigen::Matrix3d K1 = CameraProjectionTransform.block<3,3>(0,0).cast <double>();						// camera calibration matrix of camera 1
	Eigen::Matrix3d K2 = image2.CameraProjectionTransform.block<3, 3>(0, 0).cast <double>();			// camera calibration matrix of camera 2
	
	Eigen::Vector3d center_not_normalized = K1*T1_cw*hVertex;											// projecting vertex into image 1
	double center_x = center_not_normalized(0)/center_not_normalized(2);								// x-coordinates of vertex projected into image1
	double center_y = center_not_normalized(1)/center_not_normalized(2);								// y-coordinates of vertex projected into image1
	
	// computing corners of patch in image 1 (homogeneous coordinates)
	Eigen::Vector3d hp1_c1(center_x + patch_size.width / 2, center_y + patch_size.height / 2,1);		// patch in image 1, lower right corner
	Eigen::Vector3d hp2_c1(center_x - patch_size.width / 2, center_y + patch_size.height / 2,1);		// patch in image 1, lower left corner
	Eigen::Vector3d hp3_c1(center_x + patch_size.width / 2, center_y - patch_size.height / 2,1);		// patch in image 1, upper right corner
	Eigen::Vector3d hp4_c1(center_x - patch_size.width / 2, center_y - patch_size.height / 2,1);		// patch in image 1, upper left corner

	// computing 3D projections of patch corners
	Eigen::Vector3d P1 = project2dto3d(surface_normal, vertex, hp1_c1);									// Corner points of projected patch from image 1 in world frame
	Eigen::Vector3d P2 = project2dto3d(surface_normal, vertex, hp2_c1);
	Eigen::Vector3d P3 = project2dto3d(surface_normal, vertex, hp3_c1);
	Eigen::Vector3d P4 = project2dto3d(surface_normal, vertex, hp4_c1);

	// homogenize 3D corner points of projected patch
	Eigen::Vector4d hP1(P1(0), P1(1), P1(2), 1);														
	Eigen::Vector4d hP2(P2(0), P2(1), P2(2), 1);
	Eigen::Vector4d hP3(P3(0), P3(1), P3(2), 1);
	Eigen::Vector4d hP4(P4(0), P4(1), P4(2), 1);

	// compute projection of 3d points into image 2 (homgeneous coordinates) ...
	Eigen::Vector3d hp1_c2 = K2*T2_cw*hP1;																
	Eigen::Vector3d hp2_c2 = K2*T2_cw*hP2;																
	Eigen::Vector3d hp3_c2 = K2*T2_cw*hP3;																
	Eigen::Vector3d hp4_c2 = K2*T2_cw*hP4;																
	
	//  and normalize them to get the pixel coordinates which define the source frame for the perspective transform	
	cv::Point2f p_c2[4];
	p_c2[0] = cv::Point2f(hp1_c2(0) / hp1_c2(2), hp1_c2(1) / hp1_c2(2));								// source frame for perspective transform, i.e. warped patch in image 2
	p_c2[1] = cv::Point2f(hp2_c2(0) / hp2_c2(2), hp2_c2(1) / hp2_c2(2));
	p_c2[2] = cv::Point2f(hp3_c2(0) / hp3_c2(2), hp3_c2(1) / hp3_c2(2));
	p_c2[3] = cv::Point2f(hp4_c2(0) / hp4_c2(2), hp4_c2(1) / hp4_c2(2));

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

}
