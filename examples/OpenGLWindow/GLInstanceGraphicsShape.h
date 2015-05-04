#ifndef GL_INSTANCE_GRAPHICS_SHAPE_H
#define GL_INSTANCE_GRAPHICS_SHAPE_H

#include "Bullet3Common/b3AlignedObjectArray.h"

struct GLInstanceVertex
{
	float xyzw[4];
	float normal[3];
	float uv[2];
};
struct GLInstanceGraphicsShape
{
	b3AlignedObjectArray<GLInstanceVertex>*	m_vertices;
	int				m_numvertices;
	b3AlignedObjectArray<int>* 		m_indices;
	int				m_numIndices;
	float			m_scaling[4];
};

#endif //GL_INSTANCE_GRAPHICS_SHAPE_H

