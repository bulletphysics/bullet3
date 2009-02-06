/*
 *	ICE / OPCODE - Optimized Collision Detection
 * http://www.codercorner.com/Opcode.htm
 * 
 * Copyright (c) 2001-2008 Pierre Terdiman,  pierre@codercorner.com

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Contains code for segments.
 *	\file		IceSegment.cpp
 *	\author		Pierre Terdiman
 *	\date		April, 4, 2000
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Segment class.
 *	A segment is defined by S(t) = mP0 * (1 - t) + mP1 * t, with 0 <= t <= 1
 *	Alternatively, a segment is S(t) = Origin + t * Direction for 0 <= t <= 1.
 *	Direction is not necessarily unit length. The end points are Origin = mP0 and Origin + Direction = mP1.
 *
 *	\class		Segment
 *	\author		Pierre Terdiman
 *	\version	1.0
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Precompiled Header
#include "Stdafx.h"

using namespace Opcode;

float Segment::SquareDistance(const Point& point, float* t)	const
{
	Point Diff = point - mP0;
	Point Dir = mP1 - mP0;
	float fT = Diff | Dir;

	if(fT<=0.0f)
	{
		fT = 0.0f;
	}
	else
	{
		float SqrLen= Dir.SquareMagnitude();
		if(fT>=SqrLen)
		{
			fT = 1.0f;
			Diff -= Dir;
		}
		else
		{
			fT /= SqrLen;
			Diff -= fT*Dir;
		}
	}

	if(t)	*t = fT;

	return Diff.SquareMagnitude();
}
