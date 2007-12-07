/*
This source file is part of GIMPACT Library.

For the latest info, see http://gimpact.sourceforge.net/

Copyright (c) 2007 Francisco Leon Najera. C.C. 80087371.
email: projectileman@yahoo.com


This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "btGImpactConvexDecompositionShape.h"
#include "BulletCollision/CollisionShapes/btConvexHullShape.h"

#include "ConvexBuilder.h"

class GIM_ConvexDecomposition : public ConvexDecomposition::ConvexDecompInterface
{
protected:
	btGImpactConvexDecompositionShape * m_compoundShape;

	btAlignedObjectArray<btCollisionShape*> m_convexShapes;


public:
	int   	mBaseCount;
	int		mHullCount;
	bool m_transformSubShapes;

	GIM_ConvexDecomposition(btGImpactConvexDecompositionShape * compoundShape,bool transformSubShapes)
	{
		mBaseCount = 0;
		mHullCount = 0;
		m_compoundShape = compoundShape;
		m_transformSubShapes = transformSubShapes;
	}

	virtual ~GIM_ConvexDecomposition()
	{
		int i;
		for (i=0;i<m_convexShapes.size();i++)
		{
			btCollisionShape* shape = m_convexShapes[i];
			delete shape;
		}

	}

	virtual void ConvexDecompResult(ConvexDecomposition::ConvexResult &result)
	{

		//calc centroid, to shift vertices around center of mass
		btVector3 centroid(0,0,0);
		btAlignedObjectArray<btVector3> vertices;

		if(m_transformSubShapes)
		{

			//const unsigned int *src = result.mHullIndices;
			for (unsigned int i=0; i<result.mHullVcount; i++)
			{
				btVector3 vertex(result.mHullVertices[i*3],result.mHullVertices[i*3+1],result.mHullVertices[i*3+2]);

				centroid += vertex;

			}
			centroid *= 1.f/(float(result.mHullVcount) );
		}

		// collect vertices
		for (unsigned int i=0; i<result.mHullVcount; i++)
		{
			btVector3 vertex(result.mHullVertices[i*3],result.mHullVertices[i*3+1],result.mHullVertices[i*3+2]);

			if(m_transformSubShapes)
			{
				vertex -= centroid ;
			}
			vertices.push_back(vertex);
		}

		// build convex shape

		btCollisionShape* convexShape = new btConvexHullShape(
				&(vertices[0].getX()),vertices.size(),sizeof(btVector3));
		m_convexShapes.push_back(convexShape);

		convexShape->setMargin(m_compoundShape->getMargin());

		if(m_transformSubShapes)
		{
			btTransform trans;
			trans.setIdentity();
			trans.setOrigin(centroid);

			// add convex shape

			m_compoundShape->addChildShape(trans,convexShape);
		}
		else
		{
			btTransform trans;
			trans.setIdentity();
			//trans.setOrigin(centroid);

			// add convex shape

			m_compoundShape->addChildShape(trans,convexShape);

			//m_compoundShape->addChildShape(convexShape);
		}
	}

	void processDecomposition(int part)
	{
		btGImpactMeshShapePart::TrimeshPrimitiveManager * trimeshInterface =
				m_compoundShape->getTrimeshInterface(part);


		trimeshInterface->lock();

		//collect vertices
		btAlignedObjectArray<float> vertices;
		vertices.reserve(trimeshInterface->get_vertex_count()*3);

		for(int vi = 0;vi<trimeshInterface->get_vertex_count();vi++)
		{
			btVector3 vec;
			trimeshInterface->get_vertex(vi,vec);
			vertices.push_back(vec[0]);
			vertices.push_back(vec[1]);
			vertices.push_back(vec[2]);
		}


		//collect indices
		btAlignedObjectArray<unsigned int> indices;
		indices.reserve(trimeshInterface->get_primitive_count()*3);


		for(int i = 0;i<trimeshInterface->get_primitive_count();i++)
		{
			int i0, i1,i2;
			trimeshInterface->get_indices(i,i0,i1,i2);
			indices.push_back(i0);
			indices.push_back(i1);
			indices.push_back(i2);
		}

		trimeshInterface->unlock();



		unsigned int depth = 5;
		float cpercent     = 5;
		float ppercent     = 15;
		unsigned int maxv  = 16;
		float skinWidth    = 0.0;


		ConvexDecomposition::DecompDesc desc;
		desc.mVcount       = trimeshInterface->get_vertex_count();
		desc.mVertices     = &vertices[0];
		desc.mTcount       = trimeshInterface->get_primitive_count();
		desc.mIndices      = &indices[0];
		desc.mDepth        = depth;
		desc.mCpercent     = cpercent;
		desc.mPpercent     = ppercent;
		desc.mMaxVertices  = maxv;
		desc.mSkinWidth    = skinWidth;
		desc.mCallback = this;

		//convexDecomposition.performConvexDecomposition(desc);

		ConvexBuilder cb(desc.mCallback);
		cb.process(desc);
	}




};



void btGImpactConvexDecompositionShape::buildConvexDecomposition(bool transformSubShapes)
{

	m_decomposition = new GIM_ConvexDecomposition(this,transformSubShapes);

	int part_count = m_trimeshInterfaces.size();
	for (int i = 0;i<part_count ;i++ )
	{
		m_decomposition->processDecomposition(i);
	}

	postUpdate();
}

btGImpactConvexDecompositionShape::~btGImpactConvexDecompositionShape()
{
	delete m_decomposition;
}
void btGImpactConvexDecompositionShape::processAllTriangles(btTriangleCallback* callback,const btVector3& aabbMin,const btVector3& aabbMax) const
{

	int part_count = m_trimeshInterfaces.size();
	for (int part = 0;part<part_count ;part++ )
	{
		void * ptr = (void * )&m_trimeshInterfaces[part];

		btGImpactMeshShapePart::TrimeshPrimitiveManager * trimeshInterface =
			static_cast<btGImpactMeshShapePart::TrimeshPrimitiveManager *>(ptr);

		trimeshInterface->lock();

		btPrimitiveTriangle triangle;


		int i = trimeshInterface->get_primitive_count();
		while(i--)
		{
			trimeshInterface->get_primitive_triangle(i,triangle);
			callback->processTriangle(triangle.m_vertices,part,i);
		}

		trimeshInterface->unlock();
	}


}
