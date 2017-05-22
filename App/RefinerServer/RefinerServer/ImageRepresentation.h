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
#include "opencv2\highgui\highgui.hpp"
#include <iostream>

	class ImageRepresentation
	{
	public:
		ImageRepresentation(std::string filename,
			Eigen::Matrix4f pCameraViewTransform,
			Eigen::Matrix4f pCameraProjectionTransform);

		/// <summary>
		/// compute 3D projection of 2D point "pixel" onto surface defined by normal "surface_normal" and vertex "vertex"
		/// </summary>
		Eigen::Vector3f project2dto3d(Eigen::Vector3f surface_normal, Eigen::Vector3f vertex, cv::Point2f p);
		
		/// <summary>
		/// Compute corrsponding patch in camera 2 given the patch in camera 1 (via reprojection and projective unwarping)
		/// </summary>
		float computeDistortedPatchCorrelation(ImageRepresentation& image2, Eigen::Vector3f surface_normal, Eigen::Vector3f vertex, cv::Size patch_size, int colorFlag);
		

		/// <summary>
		/// Compute corrsponding patch in camera 2 given the patch in camera 1 (without unwarping)
		/// </summary>
		float computePatchCorrelation(ImageRepresentation& image2, Eigen::Vector3f surface_normal, Eigen::Vector3f vertex, cv::Size patch_size);


		/// <summary>
		/// Compute view quality
		/// </summary>
		float getViewQuality(Eigen::Vector3f vertex, Eigen::Vector3f normal, ImageRepresentation& image2);


		const std::string filename;
		std::vector<unsigned char> image;
		cv::Mat ocvImage;
		unsigned width, height;
		int x_size, y_size;
		Eigen::Matrix4f CameraViewTransform;
		Eigen::Matrix4f CameraProjectionTransform;
	};
