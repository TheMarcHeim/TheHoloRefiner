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

//compute 3d projection of 2d point
// Eigen::Vector3d project2dto3d(ImageRepresentation& image, Eigen::vec<int,3,1> surface_normal, Eigen::vec<int,3,1> vertex, int u, int v){

// get orientation of camera
//Eigen::Matrix3d R_cw = image.CameraViewTransform.block<3,3>(0,0); //camera orientation matrix
//Eigen::Vector3d C_w = image.CameraViewTransform.block<3, 1>(0, 3); //camera position in world frame
// Eigen::Matrix3d K = image.CameraProjectionTransform.block<3,3>(0,0); // kamera calibration matrix

//get image coordinates of point
//Eigen::Vector3d p_c = K.inv()*Eigen::Vector3d(u,v,1); //in camera frame
//Eigen::Vector3d p_w = R_cw.transpose()*p_c; //in camera frame

// compute scale
// float lambda = (vertex - C_w).dot(surface_normal)/p_w.dot(surface_normal);

// compute world frame points
//Eigen::Vector3d P_w = C_w + lambda*p_w; //point in world frame

//return P_w;
//}


//compute warped patch coordinates
// Eigen::Matrix<double, 3,4> = computeDistortedPatch(ImageRepresentation& image1, ImageRepresentation& image2, Eigen::vec<int,3,1> surface_normal, Eigen::vec<int,4,1> vertex, int patch_size){

// compute center point
//Eigen::Matrix3d R1_cw = image1.CameraViewTransform.block<3,3>(0,0); //camera orientation matrix
//Eigen::Vector3d C1_w = image1.CameraViewTransform.block<3, 1>(0, 3); //camera position in world frame
// Eigen::Matrix3d K = image1.CameraProjectionTransform.block<3,3>(0,0); // kamera calibration matrix
//Eigen::Vector4d center_not_normalized = K*image1.CameraViewTransform1*vertex;
//double center_x = center_not_normalized(0)/center_not_normalized(2);
//double center_y = center_not_normalized(1)/center_not_normalized(2);

//compute all points in camera 1
//Eigen::Matrix<double, 2,4> p_c1;
//p_c1 << center_x + patch_size, center_x - patch_size, center_x + patch_size, center_x - patch_size,
//		center_y + patch_size, center_y + patch_size, center_y - patch_size, center_y - patch_size;

// compute 3d projections of those points
//...project2dto3d(ImageRepresentation& image, Eigen::vec<int,3,1> surface_normal, Eigen::vec<int,3,1> vertex, int u, int v)...

// compute projection of 3d points into new camera
//... K*image2.CameraViewTransform.block<3,4>(0,0)*points...

// compute perspective/affine transformation of new patch
//...M = cv2.getPerspectiveTransform(pts1, pts2)...


// apply perspective transform to "undistort" patch
//...dst = cv2.warpPerspective(img, M, (300, 300))...


//}