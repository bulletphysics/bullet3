#ifndef GL_INSTANCE_RENDERER_INTERNAL_DATA_H
#define GL_INSTANCE_RENDERER_INTERNAL_DATA_H

#include "OpenGLInclude.h"
#include "BulletCommon/btAlignedObjectArray.h"

struct GLInstanceRendererInternalData
{
	
	btAlignedObjectArray<GLfloat> m_instance_positions_ptr;
	btAlignedObjectArray<GLfloat> m_instance_quaternion_ptr;
	btAlignedObjectArray<GLfloat> m_instance_colors_ptr;
	btAlignedObjectArray<GLfloat> m_instance_scale_ptr;

	int 								m_vboSize;
	GLuint              m_vbo;
};

#endif //GL_INSTANCE_RENDERER_INTERNAL_DATA_H
