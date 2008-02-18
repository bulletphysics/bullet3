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

#include "btTriangleMesh.h"
#include <assert.h>


btTriangleMesh::btTriangleMesh ()
:m_use32bitIndices(true),
m_use4componentVertices(true)
{

}

void	btTriangleMesh::getLockedVertexIndexBase(unsigned char **vertexbase, int& numverts,PHY_ScalarType& type, int& stride,unsigned char **indexbase,int & indexstride,int& numfaces,PHY_ScalarType& indicestype,int subpart)
{
	(void)subpart;
	if (m_use4componentVertices)
	{
		numverts = m_4componentVertices.size();
		*vertexbase = (unsigned char*)&m_4componentVertices[0];
		type = PHY_FLOAT;
		stride = sizeof(btVector3);
	} else
	{
		numverts = m_3componentVertices.size();
		*vertexbase = (unsigned char*)&m_3componentVertices[0];
		type = PHY_FLOAT;
		stride = 3*sizeof(btScalar);
	}

	if (m_use32bitIndices)
	{
		numfaces = m_32bitIndices.size()/3;
		*indexbase = (unsigned char*) &m_32bitIndices[0];
		indicestype = PHY_INTEGER;
		indexstride = 3*sizeof(int);
	} else
	{
		numfaces = m_16bitIndices.size()/3;
		*indexbase = (unsigned char*) &m_16bitIndices[0];
		indicestype = PHY_SHORT;
		indexstride = 3*sizeof(short int);
	}

}

void	btTriangleMesh::getLockedReadOnlyVertexIndexBase(const unsigned char **vertexbase, int& numverts,PHY_ScalarType& type, int& stride,const unsigned char **indexbase,int & indexstride,int& numfaces,PHY_ScalarType& indicestype,int subpart) const
{
	(void)subpart;

	if (m_use4componentVertices)
	{
		numverts = m_4componentVertices.size();
		*vertexbase = (unsigned char*)&m_4componentVertices[0];
		type = PHY_FLOAT;
		stride = sizeof(btVector3);
	} else
	{
		numverts = m_3componentVertices.size();
		*vertexbase = (unsigned char*)&m_3componentVertices[0];
		type = PHY_FLOAT;
		stride = 3*sizeof(btScalar);
	}

	
	if (m_use32bitIndices)
	{
		numfaces = m_32bitIndices.size()/3;
		*indexbase = (unsigned char*) &m_32bitIndices[0];
		indicestype = PHY_INTEGER;
		indexstride = 3*sizeof(int);
	} else
	{
		numfaces = m_16bitIndices.size()/3;
		*indexbase = (unsigned char*) &m_16bitIndices[0];
		indicestype = PHY_SHORT;
		indexstride = 3*sizeof(short int);
	}
}



int		btTriangleMesh::getNumSubParts() const
{
	return 1;
}
