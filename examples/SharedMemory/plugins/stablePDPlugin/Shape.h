#pragma once

#include <string>

class cShape
{
public:
	enum eShape
	{
		eShapeNull,
		eShapeBox,
		eShapeCapsule,
		eShapeSphere,
		eShapeCylinder,
		eShapePlane,
		eShapeMax,
	};

	static bool ParseShape(const std::string& str, cShape::eShape& out_shape);
};