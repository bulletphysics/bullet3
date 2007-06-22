/*! \file btGImpactShape.h
\author Francisco León Nájera
*/
/*
-----------------------------------------------------------------------------
This source file is part of GIMPACT Library.

For the latest info, see http://gimpact.sourceforge.net/

Copyright (c) 2006 Francisco Leon Najera. C.C. 80087371.
email: projectileman@yahoo.com

 This library is free software; you can redistribute it and/or
 modify it under the terms of EITHER:
   (1) The GNU Lesser General Public License as published by the Free
       Software Foundation; either version 2.1 of the License, or (at
       your option) any later version. The text of the GNU Lesser
       General Public License is included with this library in the
       file GIMPACT-LICENSE-LGPL.TXT.
   (2) The BSD-style license that is included with this library in
       the file GIMPACT-LICENSE-BSD.TXT.
   (3) The zlib/libpng license that is included with this library in
       the file GIMPACT-LICENSE-ZLIB.TXT.

 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files
 GIMPACT-LICENSE-LGPL.TXT, GIMPACT-LICENSE-ZLIB.TXT and GIMPACT-LICENSE-BSD.TXT for more details.

-----------------------------------------------------------------------------
*/


#ifndef GIMPACT_SHAPE_H
#define GIMPACT_SHAPE_H

#include "BulletCollision/CollisionShapes/btCollisionShape.h"
#include "BulletCollision/CollisionShapes/btTriangleShape.h"
#include "BulletCollision/CollisionShapes/btStridingMeshInterface.h"
#include "BulletCollision/CollisionShapes/btCollisionMargin.h"
#include "BulletCollision/CollisionDispatch/btCollisionWorld.h"
#include "BulletCollision/CollisionShapes/btConcaveShape.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btTransform.h"
#include "LinearMath/btMatrix3x3.h"
#include "LinearMath/btAlignedObjectArray.h"

#include "GIMPACT/core/gim_box_set.h"


enum eGIMPACT_SHAPE_TYPE
{
	CONST_GIMPACT_COMPOUND_SHAPE = 0,
	CONST_GIMPACT_TRIMESH_SHAPE_PART,
	CONST_GIMPACT_TRIMESH_SHAPE
};

//! Base class for gimpact shapes
class btGImpactShapeInterface : public btConcaveShape
{
protected:
    GIM_AABB m_localAABB;
    bool m_needs_update;
    btVector3  localScaling;

	//! use this function for perfofm refit in bounding boxes
    virtual void calcLocalAABB() = 0;


public:
	btGImpactShapeInterface()
	{
		m_localAABB.invalidate();
		m_needs_update = true;
		localScaling.setValue(1.f,1.f,1.f);
	}


	//! performs refit operation
	/*!
	Updates the entire Box set of this shape.
	\pre postUpdate() must be called for attemps to calculating the box set, else this function
		will does nothing.
	\post if m_needs_update == true, then it calls calcLocalAABB();
	*/
    SIMD_FORCE_INLINE void updateBound()
    {
    	if(!m_needs_update) return;
    	calcLocalAABB();
    	m_needs_update  = false;
    }

    //! If the Bounding box is not updated, then this class attemps to calculate it.
    /*!
    \post Calls updateBound() for update the box set.
    */
    void getAabb(const btTransform& t,btVector3& aabbMin,btVector3& aabbMax) const
    {
        GIM_AABB transformedbox = m_localAABB;
        transformedbox.appy_transform(t);
        aabbMin = transformedbox.m_min;
        aabbMax = transformedbox.m_max;
    }

    //! Tells to this object that is needed to refit the box set
    virtual void postUpdate()
    {
    	m_needs_update = true;
    }

	//! Obtains the local box, which is the global calculated box of the total of subshapes
	const GIM_AABB & getLocalBox()
	{
		return m_localAABB;
	}


    virtual int	getShapeType() const
    {
        return GIMPACT_SHAPE_PROXYTYPE;
    }

	//! Base method for determinig which kind of GIMPACT shape we get
	virtual eGIMPACT_SHAPE_TYPE getGImpactShapeType() = 0;

	//! Determines if this class has a hierarchy structure for sorting its primitives
	virtual bool hasBoxSet()  const = 0;

	/*!
	\post You must call updateBound() for update the box set.
	*/
	virtual void	setLocalScaling(const btVector3& scaling)
	{
		localScaling = scaling;
		postUpdate();
	}
	virtual const btVector3& getLocalScaling() const
	{
		return localScaling;
	}


	//! virtual method for ray collision
	virtual void rayTest(const btVector3& rayFrom, const btVector3& rayTo, btCollisionWorld::RayResultCallback& resultCallback)  const
	{
	}

	//! Function for retrieve triangles.
	/*!
	It gives the triangles in local space
	*/
	virtual void	processAllTriangles(btTriangleCallback* callback,const btVector3& aabbMin,const btVector3& aabbMax) const
	{
	}

	//!@}


};


//! btGIMPACTCompoundShape allows to handle multiple btCollisionShape objects at once
/*!
This allows for concave collision objects. This is more general then the Static Concave btTriangleMeshShape.
*/
class btGImpactCompoundShape	: public btGImpactShapeInterface
{
public:
	//! compound primitive manager
	class CompoundPrimitiveManager
	{
	public:
		btGImpactCompoundShape * m_compoundShape;

		CompoundPrimitiveManager(const CompoundPrimitiveManager& compound)
		{
			m_compoundShape = compound.m_compoundShape;
		}

		CompoundPrimitiveManager(btGImpactCompoundShape * compoundShape)
		{
			m_compoundShape = compoundShape;
		}

		CompoundPrimitiveManager()
		{
			m_compoundShape = NULL;
		}

		SIMD_FORCE_INLINE bool is_trimesh() const
		{
			return false;
		}

		SIMD_FORCE_INLINE GUINT get_primitive_count() const
		{
			return (GUINT )m_compoundShape->getNumChildShapes();
		}

		SIMD_FORCE_INLINE void get_primitive_box(GUINT prim_index ,GIM_AABB & primbox) const
		{
			btTransform prim_trans = m_compoundShape->getChildTransform(prim_index);
			const btCollisionShape* shape = m_compoundShape->getChildShape(prim_index);
			shape->getAabb(prim_trans,primbox.m_min,primbox.m_max);
		}

		SIMD_FORCE_INLINE void get_primitive_triangle(GUINT prim_index,GIM_TRIANGLE & triangle) const
		{
			btAssert(0);
		}

	};

	class BoxSetClass: public GIM_BOX_TREE_SET<CompoundPrimitiveManager>
	{
	public:
	};


protected:

	BoxSetClass m_box_set;
	btAlignedObjectArray<btTransform>		m_childTransforms;
	btAlignedObjectArray<btCollisionShape*>	m_childShapes;


	//! use this function for perfofm refit in bounding boxes
    virtual void calcLocalAABB()
    {
    	if(m_box_set.getNodeCount() == 0)
    	{
    		m_box_set.buildSet();
    	}
    	else
    	{
    		m_box_set.update();
    	}

    	m_localAABB = m_box_set.getGlobalBox();
    }
public:

	btGImpactCompoundShape()
	{
		m_box_set.setPrimitiveManager(CompoundPrimitiveManager(this));
	}

	virtual ~btGImpactCompoundShape()
	{
	}
	
	//! Obtains the primitive manager
	SIMD_FORCE_INLINE const CompoundPrimitiveManager & getPrimitiveManager()  const
	{
		return m_box_set.getPrimitiveManager();
	}

	//! Use this method for adding children
	void addChildShape(const btTransform& localTransform,btCollisionShape* shape)
	{
		m_childTransforms.push_back(localTransform);
		m_childShapes.push_back(shape);
	}
	
	//! Gets the number of children
	int	getNumChildShapes() const
	{
		return int (m_childShapes.size());
	}

	//! Gets the children
	btCollisionShape* getChildShape(int index)
	{
		return m_childShapes[index];
	}

	//! Gets the children
	const btCollisionShape* getChildShape(int index) const
	{
		return m_childShapes[index];
	}

	//! Gets the children transform
	btTransform	getChildTransform(int index) const
	{
		return m_childTransforms[index];
	}

	//! Sets the children transform
	/*!	
	\post You must call updateBound() for update the box set.	
	*/
	void setChildTransform(int index, const btTransform & transform)
	{
		m_childTransforms[index] = transform;
		postUpdate();
	}

	//! Calculates the exact inertia tensor for this shape
	virtual void	calculateLocalInertia(btScalar mass,btVector3& inertia);


	BoxSetClass * getBoxSet()
	{
		return &m_box_set;
	}

	virtual char*	getName()const
	{
		return "GImpactCompound";
	}

	virtual eGIMPACT_SHAPE_TYPE getGImpactShapeType()
	{
		return CONST_GIMPACT_COMPOUND_SHAPE;
	}

	virtual bool hasBoxSet()  const
	{
		if(m_box_set.getNodeCount() == 0) return false;
		return true;
	}

	virtual void rayTest(const btVector3& rayFrom, const btVector3& rayTo, btCollisionWorld::RayResultCallback& resultCallback)  const;


};

//! Helper class for colliding Bullet Triangle Shapes
/*!
This class implements a better getAabb method than the previous btTriangleShape class
*/
class btTriangleShapeEx: public btTriangleShape
{
public:
	btTriangleShapeEx(const btVector3& p0,const btVector3& p1,const btVector3& p2):	btTriangleShape(p0,p1,p2)
	{
	}

	virtual void getAabb(const btTransform& t,btVector3& aabbMin,btVector3& aabbMax)const
	{
		btVector3 tv0 = t(m_vertices1[0]);
		btVector3 tv1 = t(m_vertices1[1]);
		btVector3 tv2 = t(m_vertices1[2]);

		GIM_AABB trianglebox(tv0,tv1,tv2,m_collisionMargin);
		aabbMin = trianglebox.m_min;
		aabbMax = trianglebox.m_max;
	}


};


//! This class manages a sub part of a mesh supplied by the btStridingMeshInterface interface.
/*!
- Simply create this shape by passing the btStridingMeshInterface to the constructor btGImpactMeshShapePart, then you must call updateBound() after creating the mesh
- When making operations with this shape, you must call <b>lock</b> before accessing to the trimesh primitives, and then call <b>unlock</b>
- You can handle deformable meshes with this shape, by calling postUpdate() every time when changing the mesh vertices.

*/
class btGImpactMeshShapePart : public btGImpactShapeInterface
{
public:
	//! Trimesh primitive manager
	/*!
	Manages the info from btStridingMeshInterface object and controls the Lock/Unlock mechanism
	*/
	class TrimeshPrimitiveManager
	{
	public:
		btScalar m_margin;
		btStridingMeshInterface * m_meshInterface;
		btVector3 m_scale;
		int m_part;
		GUINT m_lock_count;
		const unsigned char *vertexbase;
		int numverts;
		PHY_ScalarType type;
		int stride;
		const unsigned char *indexbase;
		int indexstride;
		int  numfaces;
		PHY_ScalarType indicestype;

		TrimeshPrimitiveManager()
		{
			m_meshInterface = NULL;
			m_part = 0;
			m_margin = 0.1f;
			m_scale = btVector3(1.f,1.f,1.f);
			m_lock_count = 0;
			vertexbase = 0;
			numverts = 0;
			stride = 0;
			indexbase = 0;
			indexstride = 0;
			numfaces = 0;
		}

 		TrimeshPrimitiveManager(const TrimeshPrimitiveManager & manager)
		{
			m_meshInterface = manager.m_meshInterface;
			m_part = manager.m_part;
			m_margin = manager.m_margin;
			m_scale = manager.m_meshInterface->getScaling();
			m_lock_count = 0;
			vertexbase = 0;
			numverts = 0;
			stride = 0;
			indexbase = 0;
			indexstride = 0;
			numfaces = 0;

		}

		TrimeshPrimitiveManager(
			btStridingMeshInterface * meshInterface,	int part)
		{
			m_meshInterface = meshInterface;
			m_part = part;
			m_scale = m_meshInterface->getScaling();
			m_margin = 0.1f;
			m_lock_count = 0;
			vertexbase = 0;
			numverts = 0;
			stride = 0;
			indexbase = 0;
			indexstride = 0;
			numfaces = 0;

		}


		void lock()
		{
			if(m_lock_count>0)
			{
				m_lock_count++;
				return;
			}
			m_meshInterface->getLockedReadOnlyVertexIndexBase(
				&vertexbase,numverts,
				type, stride,&indexbase, indexstride, numfaces,indicestype,m_part);

			m_lock_count = 1;
		}

		void unlock()
		{
			if(m_lock_count == 0) return;
			if(m_lock_count>1)
			{
				--m_lock_count;
				return;
			}
			m_meshInterface->unLockReadOnlyVertexBase(m_part);
			vertexbase = NULL;
			m_lock_count = 0;
		}

		SIMD_FORCE_INLINE bool is_trimesh() const
		{
			return true;
		}

		SIMD_FORCE_INLINE GUINT get_primitive_count() const
		{
			return (GUINT )numfaces;
		}

		SIMD_FORCE_INLINE GUINT get_vertex_count() const
		{
			return (GUINT )numverts;
		}

		SIMD_FORCE_INLINE void get_indices(GUINT face_index,GUINT &i0,GUINT &i1,GUINT &i2) const
		{
			if(indicestype == PHY_SHORT)
			{
				GUSHORT * s_indices = (GUSHORT *)(indexbase + face_index*indexstride);
				i0 = s_indices[0];
				i1 = s_indices[1];
				i2 = s_indices[2];
			}
			else
			{
				GUINT * i_indices = (GUINT *)(indexbase + face_index*indexstride);
				i0 = i_indices[0];
				i1 = i_indices[1];
				i2 = i_indices[2];
			}
		}

		SIMD_FORCE_INLINE void get_vertex(GUINT vertex_index, btVector3 & vertex) const
		{
			if(indicestype == PHY_DOUBLE)
			{
				double * dvertices = (double *)(vertexbase + vertex_index*stride);
				vertex[0] = btScalar(dvertices[0]*m_scale[0]);
				vertex[1] = btScalar(dvertices[1]*m_scale[1]);
				vertex[2] = btScalar(dvertices[2]*m_scale[2]);
			}
			else
			{
				float * svertices = (float *)(vertexbase + vertex_index*stride);
				vertex[0] = svertices[0]*m_scale[0];
				vertex[1] = svertices[1]*m_scale[1];
				vertex[2] = svertices[2]*m_scale[2];
			}
		}

		SIMD_FORCE_INLINE void get_primitive_box(GUINT prim_index ,GIM_AABB & primbox) const
		{
			GIM_TRIANGLE  triangle;
			get_primitive_triangle(prim_index,triangle);
			primbox = triangle.get_box();
		}

		SIMD_FORCE_INLINE void get_primitive_triangle(GUINT prim_index,GIM_TRIANGLE & triangle) const
		{
			GUINT indices[3];
			get_indices(prim_index,indices[0],indices[1],indices[2]);
			get_vertex(indices[0],triangle.m_vertices[0]);
			get_vertex(indices[1],triangle.m_vertices[1]);
			get_vertex(indices[2],triangle.m_vertices[2]);
			triangle.m_margin = m_margin;
		}

		SIMD_FORCE_INLINE void get_bullet_triangle(GUINT prim_index,btTriangleShapeEx & triangle) const
		{
			GUINT indices[3];
			get_indices(prim_index,indices[0],indices[1],indices[2]);
			get_vertex(indices[0],triangle.m_vertices1[0]);
			get_vertex(indices[1],triangle.m_vertices1[1]);
			get_vertex(indices[2],triangle.m_vertices1[2]);
			triangle.setMargin(m_margin);
		}

	};

	class BoxSetClass: public GIM_BOX_TREE_SET<TrimeshPrimitiveManager>
	{
	public:
	};


protected:
	BoxSetClass m_box_set;

	//! use this function for perfofm refit in bounding boxes
    virtual void calcLocalAABB()
    {
		lock();
    	if(m_box_set.getNodeCount() == 0)
    	{
    		m_box_set.buildSet();
    	}
    	else
    	{
    		m_box_set.update();
    	}
		unlock();

    	m_localAABB = m_box_set.getGlobalBox();
    }
public:

	btGImpactMeshShapePart()
	{
	}

	btGImpactMeshShapePart(const btGImpactMeshShapePart & meshpart)
	{
		m_box_set.setPrimitiveManager(meshpart.getPrimitiveManager());
	}


	btGImpactMeshShapePart(btStridingMeshInterface * meshInterface,	int part)
	{
		m_box_set.setPrimitiveManager(TrimeshPrimitiveManager(meshInterface,part));
	}

	virtual ~btGImpactMeshShapePart()
	{
	}

	SIMD_FORCE_INLINE const TrimeshPrimitiveManager & getPrimitiveManager() const
	{
		return m_box_set.getPrimitiveManager();
	}

	SIMD_FORCE_INLINE void lock() const
	{
		void * dummy = (void*)(& m_box_set.getPrimitiveManager());
		TrimeshPrimitiveManager * dummymanager = static_cast<TrimeshPrimitiveManager *>(dummy);
		dummymanager->lock();
	}

	SIMD_FORCE_INLINE void unlock() const
	{
		void * dummy = (void*)(&m_box_set.getPrimitiveManager());
		TrimeshPrimitiveManager * dummymanager = static_cast<TrimeshPrimitiveManager *>(dummy);
		dummymanager->unlock();
	}

	virtual void	calculateLocalInertia(btScalar mass,btVector3& inertia);

	SIMD_FORCE_INLINE BoxSetClass * getBoxSet()
	{
		return &m_box_set;
	}

	virtual char*	getName()const
	{
		return "GImpactMeshShapePart";
	}

	virtual eGIMPACT_SHAPE_TYPE getGImpactShapeType()
	{
		return CONST_GIMPACT_TRIMESH_SHAPE_PART;
	}

	virtual bool hasBoxSet() const
	{
		if(m_box_set.getNodeCount() == 0) return false;
		return true;
	}

	SIMD_FORCE_INLINE GUINT getTriangleCount() const
	{
		return m_box_set.getPrimitiveManager().get_primitive_count();
	}

	SIMD_FORCE_INLINE void getTriangle(GUINT triangle_index, GIM_TRIANGLE & triangle) const
	{
		m_box_set.getPrimitiveManager().get_primitive_triangle(triangle_index,triangle);
	}

	SIMD_FORCE_INLINE void getBulletTriangle(GUINT prim_index,btTriangleShapeEx & triangle) const
	{
		m_box_set.getPrimitiveManager().get_bullet_triangle(prim_index,triangle);
	}

	SIMD_FORCE_INLINE GUINT getVertexCount() const
	{
		return m_box_set.getPrimitiveManager().get_vertex_count();
	}

	SIMD_FORCE_INLINE void getVertex(GUINT vertex_index, btVector3 & vertex) const
	{
		m_box_set.getPrimitiveManager().get_vertex(vertex_index,vertex);
	}

	SIMD_FORCE_INLINE void setMargin(btScalar margin)
    {
    	m_box_set.getPrimitiveManager().m_margin = margin;
    	postUpdate();
    }

    SIMD_FORCE_INLINE btScalar getMargin() const
    {
    	return m_box_set.getPrimitiveManager().m_margin;
    }

    virtual void	setLocalScaling(const btVector3& scaling)
    {
    	m_box_set.getPrimitiveManager().m_scale = scaling;
    	postUpdate();
    }

    virtual const btVector3& getLocalScaling() const
    {
    	return m_box_set.getPrimitiveManager().m_scale;
    }

    SIMD_FORCE_INLINE GUINT getPart() const
    {
    	return (GUINT)m_box_set.getPrimitiveManager().m_part;
    }

    virtual void rayTest(const btVector3& rayFrom, const btVector3& rayTo, btCollisionWorld::RayResultCallback& resultCallback)  const;

	virtual void	processAllTriangles(btTriangleCallback* callback,const btVector3& aabbMin,const btVector3& aabbMax) const;
};


//! This class manages a mesh supplied by the btStridingMeshInterface interface.
/*!
Set of btGImpactMeshShapePart parts 
- Simply create this shape by passing the btStridingMeshInterface to the constructor btGImpactMeshShape, then you must call updateBound() after creating the mesh

- You can handle deformable meshes with this shape, by calling postUpdate() every time when changing the mesh vertices.

*/

class btGImpactMeshShape : public btGImpactShapeInterface
{
protected:
	btAlignedObjectArray<btGImpactMeshShapePart*> m_mesh_parts;
	void buildMeshParts(btStridingMeshInterface * meshInterface)
	{
		for (int i=0;i<meshInterface->getNumSubParts() ;++i )
		{
			btGImpactMeshShapePart * newpart = new btGImpactMeshShapePart(meshInterface,i);
			m_mesh_parts.push_back(newpart);
		}
	}

	//! use this function for perfofm refit in bounding boxes
    virtual void calcLocalAABB()
    {
    	m_localAABB.invalidate();
    	int i = m_mesh_parts.size();
    	while(i--)
    	{
    		m_mesh_parts[i]->updateBound();
    		m_localAABB.merge(m_mesh_parts[i]->getLocalBox());
    	}
    }

public:
	btGImpactMeshShape(btStridingMeshInterface * meshInterface)
	{
		buildMeshParts(meshInterface);
	}

	virtual ~btGImpactMeshShape()
	{
		int i = m_mesh_parts.size();
    	while(i--)
    	{
			btGImpactMeshShapePart * part = m_mesh_parts[i];
			delete part;
    	}
		m_mesh_parts.clear();
	}

	int getMeshPartCount()
	{
		return m_mesh_parts.size();
	}

	btGImpactMeshShapePart * getMeshPart(int index)
	{
		return m_mesh_parts[index];
	}



	const btGImpactMeshShapePart * getMeshPart(int index) const
	{
		return m_mesh_parts[index];
	}

	
	virtual void	setLocalScaling(const btVector3& scaling)
	{
		localScaling = scaling;

		int i = m_mesh_parts.size();
    	while(i--)
    	{
			btGImpactMeshShapePart * part = m_mesh_parts[i];
			part->setLocalScaling(scaling);
    	}

		m_needs_update = true;
	}

	//! Tells to this object that is needed to refit all the meshes
    virtual void postUpdate()
    {
		int i = m_mesh_parts.size();
    	while(i--)
    	{
			btGImpactMeshShapePart * part = m_mesh_parts[i];
			part->postUpdate();
    	}

    	m_needs_update = true;
    }


	virtual void	calculateLocalInertia(btScalar mass,btVector3& inertia);
	virtual eGIMPACT_SHAPE_TYPE getGImpactShapeType()
	{
		return CONST_GIMPACT_TRIMESH_SHAPE;
	}

	virtual bool hasBoxSet()  const
	{
		return false;
	}

	virtual char*	getName()const
	{
		return "GImpactMesh";
	}

	virtual void rayTest(const btVector3& rayFrom, const btVector3& rayTo, btCollisionWorld::RayResultCallback& resultCallback)  const;

	//! Function for retrieve triangles.
	/*!
	It gives the triangles in local space
	*/
	virtual void	processAllTriangles(btTriangleCallback* callback,const btVector3& aabbMin,const btVector3& aabbMax) const;
};


#endif //GIMPACT_MESH_SHAPE_H
