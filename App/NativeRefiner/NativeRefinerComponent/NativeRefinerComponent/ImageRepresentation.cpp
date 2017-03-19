#include "pch.h"
#include "ImageRepresentation.h"



ImageRepresentation::ImageRepresentation(std::string path)
{
	image.Load(path);

}
