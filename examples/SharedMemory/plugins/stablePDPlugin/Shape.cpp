#include "Shape.h"
#include <assert.h>
#include <stdio.h>

bool cShape::ParseShape(const std::string& str, eShape& out_shape)
{
	bool succ = true;
	if (str == "null")
	{
		out_shape = eShapeNull;  
	}
	else if (str == "box")
	{
		out_shape = eShapeBox;
	}
	else if (str == "capsule")
	{
		out_shape = eShapeCapsule;
	}
	else if (str == "sphere")
	{
		out_shape = eShapeSphere;
	}
	else if (str == "cylinder")
	{
		out_shape = eShapeCylinder;
	}
	else if (str == "plane")
	{
		out_shape = eShapePlane;
	}
	else
	{
		printf("Unsupported body shape %s\n", str.c_str());
		assert(false);
	}
	return succ;
}