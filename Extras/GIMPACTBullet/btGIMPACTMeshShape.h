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

#ifndef GIMPACT_MESH_SHAPE_H
#define GIMPACT_MESH_SHAPE_H

#include "BulletCollision/CollisionShapes/btConcaveShape.h"
#include "BulletCollision/CollisionShapes/btStridingMeshInterface.h"
#include <vector>

//#define GIMPACT_SHAPE_PROXYTYPE (MAX_BROADPHASE_COLLISION_TYPES + 1)

//! Handle representation for each mesh part
/*!
Each mesh part must have a GIMPACT trimesh data (GIM_TRIMESH_DATA).
*/
typedef unsigned long BT_GIMPACT_TRIMESH_DATA_HANDLE;


class BT_GIMPACT_TRIMESH_DATA_HANDLE_ARRAY: public std::vector<BT_GIMPACT_TRIMESH_DATA_HANDLE>
{
public:
   
};

class btGIMPACTMeshData
{
protected:
	void clearMeshParts();
	void addMeshPart(btStridingMeshInterface* meshInterface, int part);
	void processMeshParts(btStridingMeshInterface* meshInterface);
public:
	btStridingMeshInterface* m_meshInterface;	
	BT_GIMPACT_TRIMESH_DATA_HANDLE_ARRAY m_meshes;

	btGIMPACTMeshData(btStridingMeshInterface* meshInterface);
	virtual ~btGIMPACTMeshData(); 
};


//! Handle representation for each mesh part
/*!
Each mesh part must have a GIMPACT trimesh (GIM_TRIMESH).
*/
typedef void * BT_GIMPACT_TRIMESH_HANDLE;


class BT_GIMPACT_TRIMESH_HANDLE_ARRAY: public std::vector<BT_GIMPACT_TRIMESH_HANDLE>
{
public:
   
};

///
///Uses an interface to access the triangles to allow for sharing graphics/physics triangles.
class btGIMPACTMeshShape : public btConcaveShape
{
protected:
	btGIMPACTMeshData * m_meshdata;
	btVector3 m_scale;
	
	void clearMeshParts();	
	void processMeshParts(btGIMPACTMeshData * meshdata);

public:
	BT_GIMPACT_TRIMESH_HANDLE_ARRAY m_gim_trimesh_parts;


	btGIMPACTMeshShape(btGIMPACTMeshData * meshInterface);

	virtual ~btGIMPACTMeshShape();    


	virtual int	getShapeType() const
	{
		return GIMPACT_SHAPE_PROXYTYPE;
	}


	//! Function for retrieve triangles.
	/*!
	It gives the triangles in local space
	*/
	virtual void	processAllTriangles(btTriangleCallback* callback,const btVector3& aabbMin,const btVector3& aabbMax) const;


	//debugging
	virtual char*	getName()const {return "GIMPACT_SHAPE_PROXYTYPE";}

	virtual void prepareMeshes(const btTransform & trans) const;

	///getAabb returns the axis aligned bounding box in the coordinate frame of the given transform t.
	virtual void getAabb(const btTransform& t,btVector3& aabbMin,btVector3& aabbMax) const;

	virtual void	setLocalScaling(const btVector3& scaling) ;
	virtual const btVector3& getLocalScaling() const ;

	virtual void	calculateLocalInertia(btScalar mass,btVector3& inertia);	

};

#endif //GIMPACT_MESH_SHAPE_H
