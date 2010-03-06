/*
CDTestFramework http://codercorner.com
Copyright (c) 2007-2008 Pierre Terdiman,  pierre@codercorner.com

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
#ifndef RENDERINGHELPERS_H
#define RENDERINGHELPERS_H

#include "Opcode.h"

	void	DrawLine(const Point& p0, const Point& p1, const Point& color, float line_width);
	void	DrawTriangle(const Point& p0, const Point& p1, const Point& p2, const Point& color);
	void	DrawSphere(const Sphere& sphere);
	void	DrawOBB(const OBB& obb);
	void	DrawCapsule(const Matrix4x4& world, const Point& p0, const Point& p1, float r);

#endif // RENDERINGHELPERS_H
