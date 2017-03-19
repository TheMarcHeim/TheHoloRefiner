#pragma once
#include <string>
#include <vector>
#include "lodepng.h"
#include<Eigen/Dense>

namespace imageRep {
	class ImageRepresentation
	{
	public:
		ImageRepresentation(std::string filename,
			Windows::Foundation::Numerics::float4x4 pCameraViewTransform,
			Windows::Foundation::Numerics::float4x4 pCameraProjectionTransform);
		std::vector<unsigned char> image;
		unsigned width, height;
		Eigen::Matrix4f CameraViewTransform;
		Eigen::Matrix4f CameraProjectionTransform;
	};
}
