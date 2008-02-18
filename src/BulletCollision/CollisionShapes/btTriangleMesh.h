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


#ifndef TRIANGLE_MESH_H
#define TRIANGLE_MESH_H

#include "btTriangleIndexVertexArray.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"

///btTriangleMesh provides storage for a concave triangle mesh. It can be used as data for the btTriangleMeshShape.
///It allows either 32bit or 16bit indices, and 4 (x-y-z-w) or 3 (x-y-z) component vertices.
///btTriangleMesh will duplicate/keep all mesh data. 
///If you prefer, you can avoid using btTriangleMesh and directly use btTriangleIndexVertexArray or derive your own class from btStridingMeshInterface. This allows to share render and collision meshes.
class btTriangleMesh : public btTriangleIndexVertexArray
{
	btAlignedObjectArray<btVector3>	m_4componentVertices;
	btAlignedObjectArray<float>		m_3componentVertices;

	btAlignedObjectArray<int>		m_32bitIndices;
	btAlignedObjectArray<short int>		m_16bitIndices;
	bool	m_use32bitIndices;
	bool	m_use4componentVertices;


	public:
		btTriangleMesh (bool use32bitIndices=true,bool use4componentVertices=true);

		bool	getUse32bitIndices() const
		{
			return m_use32bitIndices;
		}

		bool	getUse4componentVertices() const
		{
			return m_use4componentVertices;
		}
		
		void	addTriangle(const btVector3& vertex0,const btVector3& vertex1,const btVector3& vertex2);
		
		int getNumTriangles() const;

		virtual void	preallocateVertices(int numverts){(void) numverts;}
		virtual void	preallocateIndices(int numindices){(void) numindices;}

		
};

#endif //TRIANGLE_MESH_H

