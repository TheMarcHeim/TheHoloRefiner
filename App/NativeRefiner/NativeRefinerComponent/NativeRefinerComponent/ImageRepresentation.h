#pragma once
#include <string>
#include <vector>
#include "lodepng.h"
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include "opencv2/features2d/features2d.hpp"
#include "opencv/cv.h"

namespace imageRep {
	class ImageRepresentation
	{
	public:
		ImageRepresentation(std::string filename,
			Windows::Foundation::Numerics::float4x4 pCameraViewTransform,
			Windows::Foundation::Numerics::float4x4 pCameraProjectionTransform);


		/// <summary>
		/// This is used for the simplified case where only rectangular patches in image space are compared.
		/// It gives back the gradient of the autocorrelation corresponding to a shift of the 'other' image. 
		/// </summary>
		Eigen::Vector2f imageSpaceGradientCompare(ImageRepresentation& otherImage, Eigen::Vector2f ownPos, Eigen::Vector2f otherPos, int patchSize);

		Eigen::Vector3d project2dto3d(Eigen::Vector3d surface_normal, Eigen::Vector3d vertex, Eigen::Vector3d p);
		
		void computeDistortedPatch(cv::Mat& output, ImageRepresentation& image2, Eigen::Vector3d surface_normal, Eigen::Vector3d vertex, cv::Size patch_size);
		
		/// <summary>
		/// step that computes positions in image space 
		/// </summary>
		void setPositions(Eigen::MatrixXd V);

		const std::string filename;
		std::vector<unsigned char> image;
		cv::Mat ocvImage;
		unsigned width, height;
		Eigen::Matrix4f CameraViewTransform;
		Eigen::Matrix4f CameraProjectionTransform;
		Eigen::Matrix<double, -1, 2> Vpos;

	private:
		cv::Mat dXImg;
		cv::Mat dYImg;
	};
}
