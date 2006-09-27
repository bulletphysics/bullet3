/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BU_SHAPE
#define BU_SHAPE

#include <LinearMath/btPoint3.h>
#include <LinearMath/btMatrix3x3.h>
#include <BulletCollision/CollisionShapes/btConvexShape.h>


///PolyhedralConvexShape is an interface class for feature based (vertex/edge/face) convex shapes.
class btPolyhedralConvexShape : public btConvexShape
{

public:

	btPolyhedralConvexShape();

	//brute force implementations
	virtual btVector3	LocalGetSupportingVertexWithoutMargin(const btVector3& vec)const;
	virtual void	BatchedUnitVectorGetSupportingVertexWithoutMargin(const btVector3* vectors,btVector3* supportVerticesOut,int numVectors) const;
	
	virtual void	CalculateLocalInertia(btScalar mass,btVector3& inertia);



	virtual int	GetNumVertices() const = 0 ;
	virtual int GetNumEdges() const = 0;
	virtual void GetEdge(int i,btPoint3& pa,btPoint3& pb) const = 0;
	virtual void GetVertex(int i,btPoint3& vtx) const = 0;
	virtual int	GetNumPlanes() const = 0;
	virtual void GetPlane(btVector3& planeNormal,btPoint3& planeSupport,int i ) const = 0;
//	virtual int GetIndex(int i) const = 0 ; 

	virtual	bool IsInside(const btPoint3& pt,btScalar tolerance) const = 0;
	
	/// optional Hull is for optional Separating Axis Test Hull collision detection, see Hull.cpp
	class	Hull*	m_optionalHull;

};

#endif //BU_SHAPE
