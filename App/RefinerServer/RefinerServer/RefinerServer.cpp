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


void loadMats(Eigen::Matrix4f& f, Eigen::Matrix4f& s, std::string path) {
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
			f(i, 0) = std::stof(line, &sz);
			line2 = line.substr(sz);
			f(i, 1) = std::stof(line2, &sz);
			line3 = line2.substr(sz);
			f(i, 2) = std::stof(line3, &sz);
			f(i, 3) = std::stof(line3.substr(sz));
			i++;
			sz = 0;
		}
		else {
			s(i, 0) = std::stof(line, &sz);
			line2 = line.substr(sz);
			s(i, 1) = std::stof(line2, &sz);
			line3 = line2.substr(sz);
			s(i, 2) = std::stof(line3, &sz);
			s(i, 3) = std::stof(line3.substr(sz));
			i++;
			sz = 0;
		}
	}
}
	

int main()
{

	NativeRefiner refiner;
	Eigen::Matrix4f intrinsic;
	Eigen::Matrix4f extrinsic;
	//std::string path = "C:/Users/Nico/Documents/3DVision/TheHoloRefiner/App/RefinerTest/RefinerTest/bin/x86/Debug/AppX/"; //Nico PC
	std::string path = "C:/SofaData/"; // Lab PC

	// loading model
	refiner.addInitModel(path + "sofa.obj");

	// adding picture 1
	loadMats(extrinsic, intrinsic, path + "CapturedImage11.29642.png.matr"); //lab
	refiner.addPicture(path + "CapturedImage11.29642.png", extrinsic, intrinsic);

	// adding picture 2
	loadMats(extrinsic, intrinsic, path + "CapturedImage20.98344.png.matr"); //lab
	refiner.addPicture(path + "CapturedImage20.98344.png", extrinsic, intrinsic);

	// adding picture 3
	loadMats(extrinsic, intrinsic, path + "CapturedImage30.988.png.matr"); //lab
	refiner.addPicture(path + "CapturedImage30.988.png", extrinsic, intrinsic);

	// adding picture 3
	loadMats(extrinsic, intrinsic, path + "CapturedImage61.05911.png.matr"); //lab
	refiner.addPicture(path + "CapturedImage61.05911.png", extrinsic, intrinsic);
	
	// sanity check
	std::cout << "Number of loaded images: " << refiner.getNImages() <<"\n";

	// refine
	std::string out = refiner.refine();

	//  save refined
	refiner.saveRefinedModel(path + "sofa_refined.obj");

	std::cout << out << "\n";

	while (1) { 

	}

    return 0;
}

