#ifndef PRIM_INTERNAL_DATA
#define PRIM_INTERNAL_DATA

#include "OpenGLInclude.h"

struct PrimInternalData
{
	GLuint m_shaderProg;
	GLint m_viewmatUniform;
	GLint m_projMatUniform;
    GLint m_positionUniform;
    GLint m_colourAttribute;
    GLint m_positionAttribute;
    GLint m_textureAttribute;
    GLuint m_vertexBuffer;
    GLuint m_vertexArrayObject;
    GLuint  m_indexBuffer;
    GLuint m_texturehandle;
};

#endif //PRIM_INTERNAL_DATA
