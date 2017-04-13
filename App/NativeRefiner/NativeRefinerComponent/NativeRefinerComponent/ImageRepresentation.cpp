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



//compute 3d projection of 2d point (onto surface defined by normal)
Eigen::Vector3d imageRep::ImageRepresentation::project2dto3d(ImageRepresentation& image, Eigen::Vector3d surface_normal, Eigen::Vector3d vertex, Eigen::Vector3d p){

	// get orientation of camera
	Eigen::Matrix3d R_cw = image.CameraViewTransform.block<3,3>(0,0); //camera orientation matrix
	Eigen::Vector3d C_w = image.CameraViewTransform.block<3, 1>(0, 3); //camera position in world frame
	Eigen::Matrix3d K = image.CameraProjectionTransform.block<3,3>(0,0); // kamera calibration matrix

	//get image coordinates of point
	Eigen::Vector3d p_c = K.inverse()*p; //in camera frame
	Eigen::Vector3d p_w = R_cw.transpose()*p_c; //in world frame

	// compute scale
	double lambda = (vertex - C_w).dot(surface_normal)/(p_w.dot(surface_normal));

	// compute world frame points
	Eigen::Vector3d P_w = C_w +lambda*p_w; //point in world frame

	return P_w;
}




//Compute corrsponding patch in camera 2 given the patch in camera 1 (via reprojection and projective unwarping)
 Eigen::Matrix<double, 3,4> imageRep::ImageRepresentation::computeDistortedPatch(ImageRepresentation& image1, ImageRepresentation& image2, Eigen::Vector3d surface_normal, Eigen::Vector3d vertex_in, cv::Size patch_size){

	// Images of camera 1 and camera 2
	cv::Mat img_c1 = image1.ocvImage;
	cv::Mat img_c2 = image2.ocvImage;

	// homogenize vertex
	Eigen::Vector4d vertex(vertex_in(1), vertex_in(2), vertex_in(3), 1);

	// compute center point
	Eigen::Matrix3d R1_cw = image1.CameraViewTransform.block<3,3>(0,0); //camera orientation matrix
	Eigen::Vector3d C1_w = image1.CameraViewTransform.block<3, 1>(0, 3); //camera position in world frame
	Eigen::Matrix3d K = image1.CameraProjectionTransform.block<3,3>(0,0); // kamera calibration matrix
	Eigen::Vector4d center_not_normalized = K*image1.CameraViewTransform*vertex;
	double center_x = center_not_normalized(0)/center_not_normalized(2);
	double center_y = center_not_normalized(1)/center_not_normalized(2);

	//compute all points in camera 1 (homogenous)
	Eigen::Vector3d p1_c1(center_x + patch_size.width, center_y + patch_size.height,1);
	Eigen::Vector3d p2_c1(center_x - patch_size.width, center_y + patch_size.height,1);
	Eigen::Vector3d p3_c1(center_x + patch_size.width, center_y - patch_size.height,1);
	Eigen::Vector3d p4_c1(center_x - patch_size.width, center_y - patch_size.height,1);

	cv::Point2f p_c1[4];
	p_c1[0] = cv::Point2f(p1_c1(0), p1_c1(1));
	p_c1[1] = cv::Point2f(p2_c1(0), p2_c1(1));
	p_c1[2] = cv::Point2f(p3_c1(0), p3_c1(1));
	p_c1[3] = cv::Point2f(p4_c1(0), p4_c1(1));

	// compute 3d projections of those points
	Eigen::Vector4d P1_c1 = imageRep::ImageRepresentation::project2dto3d(image1, surface_normal, vertex, p1_c1);
	Eigen::Vector4d P2_c1 = imageRep::ImageRepresentation::project2dto3d(image1, surface_normal, vertex, p2_c1);
	Eigen::Vector4d P3_c1 = imageRep::ImageRepresentation::project2dto3d(image1, surface_normal, vertex, p3_c1);
	Eigen::Vector4d P4_c1 = imageRep::ImageRepresentation::project2dto3d(image1, surface_normal, vertex, p4_c1);

	// compute projection of 3d points into camera 2
	Eigen::Vector4d temp_p1 = K*image2.CameraViewTransform.block<3, 4>(0, 0)*P1_c1;
	Eigen::Vector4d temp_p2 = K*image2.CameraViewTransform.block<3, 4>(0, 0)*P2_c1;
	Eigen::Vector4d temp_p3 = K*image2.CameraViewTransform.block<3, 4>(0, 0)*P3_c1;
	Eigen::Vector4d temp_p4 = K*image2.CameraViewTransform.block<3, 4>(0, 0)*P4_c1;

	// normalize points in new camera	
	cv::Point2f p_c2[4];
	p_c2[0] = cv::Point2f(temp_p1(0) / temp_p1(2), temp_p1(1) / temp_p1(2));
	p_c2[1] = cv::Point2f(temp_p2(0) / temp_p2(2), temp_p2(1) / temp_p2(2));
	p_c2[2] = cv::Point2f(temp_p3(0) / temp_p3(2), temp_p3(1) / temp_p3(2));
	p_c2[3] = cv::Point2f(temp_p4(0) / temp_p4(2), temp_p4(1) / temp_p4(2));
	
	// compute perspective/affine transformation of new patch... Interestingly, M encodes somehow patch sizes, location etc (?!) cf link below
	// http://opencvexamples.blogspot.com/2014/01/perspective-transform.html 
	//Nico's comment: I disagree... according to docs.opencv.org M should be 3x3. In the above link it is initialized as 2x4..?
	cv::Mat M = cv::Mat::zeros(img_c1.rows, img_c1.cols, img_c1.type());	
	M = cv::getPerspectiveTransform(p_c2, p_c1);

	// and apply perspective transform to "undistort" patch
	cv::warpPerspective(img_c2, img_c1, M, patch_size);
}
