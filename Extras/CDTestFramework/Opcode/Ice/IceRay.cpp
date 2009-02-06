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
 *	Contains code for rays.
 *	\file		IceRay.cpp
 *	\author		Pierre Terdiman
 *	\date		April, 4, 2000
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Ray class.
 *	A ray is a half-line P(t) = mOrig + mDir * t, with 0 <= t <= +infinity
 *	\class		Ray
 *	\author		Pierre Terdiman
 *	\version	1.0
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*
	O = Origin = impact point
	i = normalized vector along the x axis
	j = normalized vector along the y axis = actually the normal vector in O
	D = Direction vector, norm |D| = 1
	N = Projection of D on y axis, norm |N| = normal reaction
	T = Projection of D on x axis, norm |T| = tangential reaction
	R = Reflexion vector

              ^y
              |
              |
              |
       _  _  _| _ _ _
       *      *      *|
        \     |     /
         \    |N   /  |
         R\   |   /D
           \  |  /    |
            \ | /
    _________\|/______*_______>x
               O    T

	Let define theta = angle between D and N. Then cos(theta) = |N| / |D| = |N| since D is normalized.

	j|D = |j|*|D|*cos(theta) => |N| = j|D

	Then we simply have:

	D = N + T

	To compute tangential reaction :

	T = D - N

	To compute reflexion vector :

	R = N - T = N - (D-N) = 2*N - D
*/

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Precompiled Header
#include "Stdafx.h"

using namespace Opcode;

float Ray::SquareDistance(const Point& point, float* t)	const
{
	Point Diff = point - mOrig;
	float fT = Diff | mDir;

	if(fT<=0.0f)
	{
		fT = 0.0f;
	}
	else
	{
		fT /= mDir.SquareMagnitude();
		Diff -= fT*mDir;
	}

	if(t) *t = fT;

	return Diff.SquareMagnitude();
}
