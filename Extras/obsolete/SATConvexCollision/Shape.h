// Bullet Continuous Collision Detection and Physics Library
// Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/
//
//
// Shape.h
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
//
// Shape.h
//
#ifndef BULLET_SHAPE_H
#define BULLET_SHAPE_H

#include "Maths.h"




struct btSeparation
{

	short m_featureA;
	short m_featureB;
	float m_dist;
	Vector3 m_axis;		// in world space
	
	// separators
	enum 
	{
		kFeatureNone,		// not separated
		kFeatureA,
		kFeatureB,
		kFeatureBoth
	};
	short m_separator;

	// contact between the 2 bodies (-1 if none)
	short m_contact;
};

///Shape provides a interface for Hull class (convex hull calculation).
class Shape
{
public:
	Shape();
	virtual ~Shape();

	//virtual void ComputeInertia(Point3& centerOfMass, Matrix33& inertia, float totalMass) const = 0;
	virtual void ComputeInertia(const Transform& transform, Point3& centerOfMass, Matrix33& inertia, float totalMass) const = 0;


	virtual Bounds3 ComputeBounds(const Transform& transform) const = 0;
};



#endif //BULLET_SHAPE_H
