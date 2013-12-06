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
 *	Contains code for 3x3 matrices.
 *	\file		IceMatrix3x3.cpp
 *	\author		Pierre Terdiman
 *	\date		April, 4, 2000
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	3x3 matrix.
 *	DirectX-compliant, ie row-column order, ie m[Row][Col].
 *	Same as:
 *	m11  m12  m13  first row.
 *	m21  m22  m23  second row.
 *	m31  m32  m33  third row.
 *	Stored in memory as m11 m12 m13 m21...
 *
 *	Multiplication rules:
 *
 *	[x'y'z'] = [xyz][M]
 *
 *	x' = x*m11 + y*m21 + z*m31
 *	y' = x*m12 + y*m22 + z*m32
 *	z' = x*m13 + y*m23 + z*m33
 *
 *	\class		Matrix3x3
 *	\author		Pierre Terdiman
 *	\version	1.0
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Precompiled Header
#include "Stdafx.h"

using namespace Opcode;

// Cast operator
Matrix3x3::operator Matrix4x4() const
{
	return Matrix4x4(
	m[0][0],	m[0][1],	m[0][2],	0.0f,
	m[1][0],	m[1][1],	m[1][2],	0.0f,
	m[2][0],	m[2][1],	m[2][2],	0.0f,
	0.0f,		0.0f,		0.0f,		1.0f);
}
