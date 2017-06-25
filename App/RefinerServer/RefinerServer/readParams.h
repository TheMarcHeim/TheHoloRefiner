#pragma once
#include <string>
#include <opencv2/core/core.hpp>
#include "opencv/cv.h"
#include "opencv2\highgui\highgui.hpp"

struct parameters {
	std::string path = "C:/HappyBirthday/"; // path to folder containing images, matrices and mesh
	int nStepsDepthSearch = 51; // number of steps in depth search
	double stepSize = 0.005; // size in meters of one step
	double refineTolerance = 0.000; // min difference between current adj. score and max to adjust vertex
	cv::Size patch_size = cv::Size(9, 9); // size of a patch
	double smoothing_lambda = 0.005; // weight given to smoothing term
	double min_angle_view = 0.0; //minimum value of cross product b/w normal and viewing ray
	double occlusion_hitpoint_max_distance = 0.05; // max distance of hit point in ray casting
	int nRefinementIt = 2; //number of iterations in refinement procedure
	int subDivFactor = 1; // factor of subdivision
	int maxNimages = 30; // upper bound to the number of images
	bool useRGB = false; // using RGB or grayscale
	bool useSubdivision = true; // whether or not to subdivide the mesh
	double gaussian = 0; // factor for gaussian blurring
	int downsample = 0; // factor for downsampling
	bool liveview = false; // boolean to enable live demo
	bool fullnormalize = false; // boolean to choose full normalization
};

typedef struct parameters parameters;

void loadParams(std::string path, parameters& param, bool verbose = false);
