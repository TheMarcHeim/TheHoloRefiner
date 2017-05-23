// RefinerServer.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "NativeRefiner.h"
#include "Eigen/Dense"
#include <fstream>
#include <iostream>
#include <opencv2\core\core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include "opencv2/features2d/features2d.hpp"
#include "opencv/cv.h"
#include <Windows.h>
#include "readParams.h"


void loadMats(Eigen::Matrix4d& f, Eigen::Matrix4d& s, std::string path) {

	std::fstream file(path);
	std::string line;
	std::string line2;
	std::string line3;
	size_t pos = 0;
	std::string::size_type sz;     // alias of size_t
	
	int thisMat = 0;
	int i = 0;
	while (getline(file, line)) {
		if (line.length() == 0) {
			thisMat = 1;
			i = 0;
		}
		else if (thisMat == 0) {
			f(i, 0) = std::stod(line, &sz);
			line2 = line.substr(sz);
			f(i, 1) = std::stod(line2, &sz);
			line3 = line2.substr(sz);
			f(i, 2) = std::stod(line3, &sz);
			f(i, 3) = std::stod(line3.substr(sz));
			i++;
			sz = 0;
		}
		else {
			s(i, 0) = std::stod(line, &sz);
			line2 = line.substr(sz);
			s(i, 1) = std::stod(line2, &sz);
			line3 = line2.substr(sz);
			s(i, 2) = std::stod(line3, &sz);
			s(i, 3) = std::stod(line3.substr(sz));
			i++;
			sz = 0;
		}
	}
}
	

int main()
{
	// Instantiate refiner and load parameters
	NativeRefiner refiner;
	parameters params;
	loadParams("params.txt", params, true);


	// Declare locals
	std::string temp;
	std::string path_with_prefix = params.path + "*.png";
	Eigen::Matrix4d intrinsic;
	Eigen::Matrix4d extrinsic;
	int index = 0;



	// loading model
	//refiner.addInitModel(params.path + "sofa.obj");
	refiner.addInitModel(params.path + "spatialMap_part.obj");
	//refiner.addInitModel(params.path + "pureBuildupSofaConnected.obj"); 
	//refiner.addInitModel(params.path + "sofa.obj");
	//refiner.addInitModel(params.path + "pureBuildupSofaConnected.obj"); 
	//refiner.addInitModel(params.path + "spatialMap_part.obj");
	//refiner.addInitModel(params.path + "Kiste_GroundTruth_Pos.obj");


	// load all pictures and matrices	
		std::wstring search_path = std::wstring(path_with_prefix.begin(), path_with_prefix.end());
		WIN32_FIND_DATA fd;
		HANDLE hFind = ::FindFirstFile(search_path.c_str(), &fd);
		if (hFind != INVALID_HANDLE_VALUE) {
			do {
				if (!(fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)) {
					std::wstring ws = std::wstring(fd.cFileName);
					temp = std::string(ws.begin(), ws.end());
					loadMats(extrinsic, intrinsic, params.path+ temp + ".matr");
					refiner.addPicture(params.path + temp, extrinsic, intrinsic);

					std::cout << "loaded picture and matrix for " << temp << " \n";
					index++;
				}
			} while (::FindNextFile(hFind, &fd) && index<params.maxNimages);
			::FindClose(hFind);
		}
	std::cout << "Number of loaded images: " << refiner.getNImages() <<"\n";

	// refine
	std::string out = refiner.refine(params.nRefinementIt);
	std::cout << "Finished Refinement \n";

	// save refined
	refiner.saveRefinedModel(params.path + "sofa_refined.obj");
	std::cout << "Saved refined model\n";

	// Done. Now loop forever to keep terminal from closing
	std::cout << "done\n";

	do {
		std::cout << '\n' << "Press a key to quit...";
	} while (std::cin.get() != '\n');

    return 0;
}

