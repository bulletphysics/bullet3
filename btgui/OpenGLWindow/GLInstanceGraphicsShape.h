#ifndef GL_INSTANCE_GRAPHICS_SHAPE_H
#define GL_INSTANCE_GRAPHICS_SHAPE_H

#include "BulletCommon/btAlignedObjectArray.h"

struct GLInstanceVertex
{
	float xyzw[4];
	float normal[3];
	float uv[2];
};
struct GLInstanceGraphicsShape
{
	btAlignedObjectArray<GLInstanceVertex>*	m_vertices;
	int				m_numvertices;
	btAlignedObjectArray<int>* 		m_indices;
	int				m_numIndices;
	float			m_scaling[4];
};

#endif //GL_INSTANCE_GRAPHICS_SHAPE_H

