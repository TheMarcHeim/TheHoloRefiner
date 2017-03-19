#pragma once
#include <afxstr.h>
#include <atlimage.h>
#include <string>

ref class ImageRepresentation sealed
{
public:
	ImageRepresentation(std::string path);
	CImage image;
	
};

