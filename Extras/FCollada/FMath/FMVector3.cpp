/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/

#include "StdAfx.h"
#include "FMath/FMVector2.h"
#include "FMath/FMVector3.h"
#include "FMath/FMVector4.h"

// Vector constants
const FMVector3 FMVector3::Zero(0.0f, 0.0f, 0.0f);
const FMVector3 FMVector3::XAxis(1.0f, 0.0f, 0.0f);
const FMVector3 FMVector3::YAxis(0.0f, 1.0f, 0.0f);
const FMVector3 FMVector3::ZAxis(0.0f, 0.0f, 1.0f);
const FMVector3 FMVector3::Origin = FMVector3::Zero;
const FMVector4 FMVector4::Zero(0.0f, 0.0f, 0.0f, 0.0f);

// Read in the vector from a source
FMVector3::FMVector3(const float* source, uint32 startIndex)
{
	x = source[startIndex];
	y = source[startIndex + 1];
	z = source[startIndex + 2];
}
