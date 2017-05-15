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

	class ImageRepresentation
	{
	public:
		ImageRepresentation(std::string filename,
			Eigen::Matrix4f pCameraViewTransform,
			Eigen::Matrix4f pCameraProjectionTransform);

		/// <summary>
		/// compute 3D projection of 2D point "pixel" onto surface defined by normal "surface_normal" and vertex "vertex"
		/// </summary>
		Eigen::Vector3d project2dto3d(Eigen::Vector3d surface_normal, Eigen::Vector3d vertex, cv::Point2d p);
		
		/// <summary>
		/// Compute corrsponding patch in camera 2 given the patch in camera 1 (via reprojection and projective unwarping)
		/// </summary>
		float computeDistortedPatchCorrelation(ImageRepresentation& image2, Eigen::Vector3d surface_normal, Eigen::Vector3d vertex, cv::Size patch_size);
		
		const std::string filename;
		std::vector<unsigned char> image;
		cv::Mat ocvImage;
		unsigned width, height;
		int x_size, y_size;
		Eigen::Matrix4f CameraViewTransform;
		Eigen::Matrix4f CameraProjectionTransform;
	};
