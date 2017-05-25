#include "readParams.h"
#include <opencv2/core/core.hpp>
#include "opencv/cv.h"
#include "opencv2\highgui\highgui.hpp"
#include <fstream>
#include <iostream>


void loadParams(std::string path, parameters& p, bool verbose) {
	std::ifstream fin(path);
	std::string line;
	std::istringstream sin;
	if(verbose)
	std::cout << "loading parameters..." << std::endl;
	while (std::getline(fin, line)) {
		sin.str(line.substr(line.find("=") + 1));
		if (line.find("path") != std::string::npos) {
			sin >> p.path;
			if(verbose)
			std::cout << "path set to: " << p.path << std::endl;
		}
		else if (line.find("nStepsDepthSearch") != std::string::npos) {
			sin >> p.nStepsDepthSearch;
			if (verbose)
			std::cout << "nStepsDepthSearch set to: " << p.nStepsDepthSearch << std::endl;
		}
		else if (line.find("stepSize") != std::string::npos) {
			sin >> p.stepSize;
			if (verbose)
			std::cout << "stepSize set to: " << p.stepSize << std::endl;
		}
		else if (line.find("refineTolerance") != std::string::npos) {
			sin >> p.refineTolerance;
			if (verbose)
			std::cout << "refineTolerance set to: " << p.refineTolerance << std::endl;
		}
		else if (line.find("patch_size") != std::string::npos) {
			int n;
			sin >> n;
			p.patch_size = cv::Size(n, n);
			if (verbose)
			std::cout << "patch_size set to: " << n << std::endl;
		}
		else if (line.find("smoothing_lambda") != std::string::npos) {
			sin >> p.smoothing_lambda;
			if (verbose)
			std::cout << "smoothing_lambda set to: " << p.smoothing_lambda << std::endl;
		}
		else if (line.find("min_angle_view") != std::string::npos) {
			sin >> p.min_angle_view;
			if (verbose)
			std::cout << "min_angle_view set to: " << p.min_angle_view << std::endl;
		}
		else if (line.find("occlustion_hitpoint_max_distance") != std::string::npos) {
			sin >> p.occlustion_hitpoint_max_distance;
			if (verbose)
			std::cout << "occlustion_hitpoint_max_distance set to: " << p.occlustion_hitpoint_max_distance << std::endl;
		}
		else if (line.find("nRefinementIt") != std::string::npos) {
			sin >> p.nRefinementIt;
			if (verbose)
			std::cout << "nRefinementIt set to: " << p.nRefinementIt << std::endl;
		}
		else if (line.find("useRGB") != std::string::npos) {
			sin >> p.useRGB;
			if (verbose)
			std::cout << "useRGB set to: " << p.useRGB << std::endl;
		}
		else if (line.find("useSubdivision") != std::string::npos) {
			sin >> p.useSubdivision;
			if (verbose)
			std::cout << "useSubdivision set to: " << p.useSubdivision << std::endl;
		}
		else if (line.find("maxNimages") != std::string::npos) {
			sin >> p.maxNimages;
			if (verbose)
			std::cout << "maxNimages set to: " << p.maxNimages << std::endl;
		}
		else if (line.find("gaussian") != std::string::npos) {
			sin >> p.gaussian;
			if (verbose)
				std::cout << "guassian set to: " << p.gaussian << std::endl;
		}
		else if (line.find("downsample") != std::string::npos) {
			sin >> p.downsample;
			if (verbose)
				std::cout << "downsample set to: " << p.downsample << std::endl;
		}
		else if (line.find("liveview") != std::string::npos) {
			sin >> p.liveview;
			if (verbose)
				std::cout << "liveview set to: " << p.liveview << std::endl;
		}
		else if (line.find("fullnormalize") != std::string::npos) {
			sin >> p.fullnormalize;
			if (verbose)
				std::cout << "fullnormalize set to: " << p.fullnormalize << std::endl;
		}
		sin.clear();
	}
}
