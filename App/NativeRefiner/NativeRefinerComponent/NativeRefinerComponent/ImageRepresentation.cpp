#include "pch.h"
#include "ImageRepresentation.h"

imageRep::ImageRepresentation::ImageRepresentation(std::string filename,
	Windows::Foundation::Numerics::float4x4 pCameraViewTransform,
	Windows::Foundation::Numerics::float4x4 pCameraProjectionTransform){
	//load PNG
	std::vector<unsigned char> png;
	unsigned width, height;

	unsigned error = lodepng::load_file(png, filename);
	if (!error) error = lodepng::decode(image, width, height, png);

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

//compute affine transformation
// Eigen::Mat<int, 3,2> computeAffine(Eigen::vec<int,3,1> surface_normal, Eigen::vec<int,3,1> vertex, Windows::Foundation::Numerics::float4x4 pCameraViewTransform,
// Windows::Foundation::Numerics::float4x4 pCameraProjectionTransform, int u, int v, int patch_size, float focal_length){

//compute camera center
// Eigen::vec<float, 3,1> camera_center = (pCameraViewTransform*pCameraProjectionTransform).block<3,1>(3,0);

// compute scale
// float scale = (camera_center - vertex).norm() / focal_length;

// compute shear

//return matrix

//}