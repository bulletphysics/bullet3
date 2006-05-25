// Bullet Continuous Collision Detection and Physics Library
// Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/
//
//
// Vector.cpp
//
// Copyright (c) 2006 Simon Hobbs
//
// This software is provided 'as-is', without any express or implied warranty. In no event will the authors be held liable for any damages arising from the use of this software.
//
// Permission is granted to anyone to use this software for any purpose, including commercial applications, and to alter it and redistribute it freely, subject to the following restrictions:
//
// 1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
//
// 2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
//
// 3. This notice may not be removed or altered from any source distribution.
#ifdef WIN32
#if _MSC_VER >= 1310
#include "Vector.h"
#include <float.h>

////////////////////////////////////////////////////////////////////////////////
// Vector3

bool Vector3::IsFinite() const
{
	if (_finite(GetX()) && _finite(GetY()) && _finite(GetZ()))
		return true;
	return false;
}


////////////////////////////////////////////////////////////////////////////////
// Point3

bool Point3::IsFinite() const
{
	if (_finite(GetX()) && _finite(GetY()) && _finite(GetZ()))
		return true;
	return false;
}


////////////////////////////////////////////////////////////////////////////////
// Vector4
bool Vector4::IsFinite() const
{
	if (_finite(GetX()) && _finite(GetY()) && _finite(GetZ()) && _finite(GetW()))
		return true;
	return false;
}

#endif
#endif //WIN32
