#ifndef NO_OPENGL3
/*
Copyright (c) 2012 Advanced Micro Devices, Inc.

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
//Originally written by Erwin Coumans


#define MAX_POINTS_IN_BATCH 1024
#define MAX_LINES_IN_BATCH 1024
#define MAX_TRIANGLES_IN_BATCH 8192

#include "OpenGLInclude.h"
#include "../CommonInterfaces/CommonWindowInterface.h"
#ifdef B3_USE_GLFW

#else
#ifndef __APPLE__
#ifndef glVertexAttribDivisor

#ifndef NO_GLEW

#define glVertexAttribDivisor glVertexAttribDivisorARB
#endif //NO_GLEW
#endif //glVertexAttribDivisor
#ifndef GL_COMPARE_REF_TO_TEXTURE
#define GL_COMPARE_REF_TO_TEXTURE GL_COMPARE_R_TO_TEXTURE
#endif //GL_COMPARE_REF_TO_TEXTURE
#ifndef glDrawElementsInstanced
#ifndef NO_GLEW
#define glDrawElementsInstanced glDrawElementsInstancedARB
#endif //NO_GLEW
#endif
#endif //__APPLE__
#endif//B3_USE_GLFW
#include "GLInstancingRenderer.h"

#include <string.h>
#include <stdio.h>

#include "Bullet3Common/b3Vector3.h"
#include "Bullet3Common/b3Quaternion.h"
#include "Bullet3Common/b3Transform.h"
#include "Bullet3Common/b3Matrix3x3.h"
#include "Bullet3Common/b3ResizablePool.h"

#include "LoadShader.h"

#include "GLInstanceRendererInternalData.h"

//GLSL shader strings, embedded using build3/stringify
#include "Shaders/pointSpriteVS.h"
#include "Shaders/pointSpritePS.h"
#include "Shaders/instancingVS.h"
#include "Shaders/instancingPS.h"
#include "Shaders/linesPS.h"
#include "Shaders/linesVS.h"

#include "GLRenderToTexture.h"
#include "stb_image/stb_image_write.h"



static const char* triangleVertexShaderText =
"#version 330\n"
"precision highp float;"
"uniform mat4 MVP;\n"
"uniform vec3 vCol;\n"
"layout (location = 0) in vec3 vPos;\n"
"layout (location = 1) in vec2 vUV;\n"

"out vec3 clr;\n"
"out vec2 uv0;\n"
"void main()\n"
"{\n"
"    gl_Position = MVP * vec4(vPos,1);\n"
"    clr = vCol;\n"
"    uv0 = vUV;\n"
"}\n";


static const char* triangleFragmentShader =
"#version 330\n"
"precision highp float;"
"in vec3 clr;\n"
"in vec2 uv0;"
"out vec4 color;"
"uniform sampler2D Diffuse;"
"void main()\n"
"{\n"
"    vec4 texel = texture(Diffuse,uv0);\n"
"    color = vec4(clr,texel.r)*texel;\n"
"}\n";


const char *szGeometryProg =
	"#version 400\n"
	"#extension GL_ARB_shader_viewport_layer_array: enable\n"
	"#extension GL_NV_viewport_array : enable\n"
	"\n"
	"layout(invocations = 16) in;\n"
	"layout (triangles) in;\n"
	"layout (triangle_strip, max_vertices = 3) out;\n"
	"uniform mat4 MVP;\n"
		"in Fragment\n"
		"{\n"
		"     vec4 color;\n"
		"} fragment_[];\n"
		"in Vert\n"
		"{\n"
		"	vec2 texcoord;\n"
		"} vert_[];\n"
		"in vec3 lightPos_[],cameraPosition_[], normal_[],ambient_[];\n"
		"in vec4 ShadowCoord_[];\n"
		"in vec4 vertexPos_[];\n"
		"in float materialShininess_[];\n"
		"in vec3 lightSpecularIntensity_[];\n"
		"in vec3 materialSpecularColor_[];\n"


		"out vec4 ShadowCoord;\n"
		"out Fragment\n"
		"{\n"
		"     vec4 color;\n"
		"} fragment;\n"
		"out Vert\n"
		"{\n"
		"	vec2 texcoord;\n"
		"} vert;\n"

		"out vec3 lightPos,normal,ambient;\n"
		"out vec4 vertexPos;\n"
		"out vec3 cameraPosition;\n"
		"out float materialShininess;\n"
		"out vec3 lightSpecularIntensity;\n"
		"out vec3 materialSpecularColor;\n"

		"\n"
		"void main(void)\n"
		"{\n"
		"    for (int i = 0; i < gl_in.length(); i++)\n"
		"    {\n"
		"        gl_Position = MVP[gl_InvocationID] * vertexPos_[i];\n"
		"		 ShadowCoord = ShadowCoord_[i];"
		"		 fragment.color = fragment_[i].color;\n"
		"		 vert.texcoord = vert_[i].texcoord;\n"
		"		 lightPos = lightPos_[i];\n"
		"		 normal = normal_[i];\n"
		"		 ambient = ambient_[i];\n"
		"		 vertexPos = vertexPos_[i];\n"
		"		 cameraPosition = cameraPosition_[i];\n"
		"		 materialShininess = materialShininess_[i];\n"
		"		 lightSpecularIntensity = lightSpecularIntensity_[i];\n"
		"		 materialSpecularColor = materialSpecularColor_[i];\n"
		"        gl_ViewportIndex = gl_InvocationID;\n"
		"        EmitVertex();\n"
		"    }\n"
		"    EndPrimitive();\n"
		"}\n";


static InternalDataRenderer* sData2;

GLint lineWidthRange[2]={1,1};

enum
{
	eGfxTransparency=1,
	eGfxHasTexture = 2,
};

struct b3GraphicsInstance
{
	GLuint	m_cube_vao;
	GLuint	m_index_vbo;
	GLuint	m_textureIndex;
	int m_numIndices;
	int m_numVertices;


	int m_batchSize;
	int m_numGraphicsInstances;
	b3AlignedObjectArray<int> m_tempObjectUids;
	int m_instanceOffset;
	int m_vertexArrayOffset;
	int	m_primitiveType;
	float m_materialShinyNess;
	b3Vector3 m_materialSpecularColor;
	int m_flags;	// texture, etc

	b3GraphicsInstance()
	:m_cube_vao(-1),
		m_index_vbo(-1),
		m_textureIndex(-1),
		m_numIndices(-1),
		m_numVertices(-1),
		m_batchSize(16),
		m_numGraphicsInstances(0),
		m_instanceOffset(0),
		m_vertexArrayOffset(0),
		m_primitiveType(B3_GL_TRIANGLES),
		m_materialShinyNess(41),
		m_materialSpecularColor(b3MakeVector3(.5,.5,.5)),
		m_flags(0)
	{
	}

};


static void checkError(const char *functionName)
{
    GLenum error;
    while (( error = glGetError() ) != GL_NO_ERROR)
    {
        fprintf (stderr, "GL error 0x%X detected in %s\n", error, functionName);
    }
}


struct InternalTextureHandle
{
    GLuint  m_glTexture;
    int m_width;
    int m_height;
	int m_enableFiltering;
};

struct b3PublicGraphicsInstanceData
{
	int m_shapeIndex;
	int m_internalInstanceIndex;
	GLfloat m_position[4];
	GLfloat m_orientation[4];
	GLfloat m_color[4];
	GLfloat m_scale[4];

	void clear()
	{
	}

};

typedef b3PoolBodyHandle<b3PublicGraphicsInstanceData> b3PublicGraphicsInstance;

struct InternalDataRenderer : public GLInstanceRendererInternalData
{
    CommonCameraInterface* m_activeCamera;

	b3Vector3 m_lightPos;
	b3Vector3 m_lightSpecularIntensity;

	GLuint				m_defaultTexturehandle;
	b3AlignedObjectArray<InternalTextureHandle>	m_textureHandles;
	GLuint				m_renderFrameBuffer;



	b3ResizablePool< b3PublicGraphicsInstance> m_publicGraphicsInstances;

	InternalDataRenderer() :
		m_activeCamera(0),
		m_renderFrameBuffer(0)
	{
		m_lightPos=b3MakeVector3(-50,30,40);
		m_lightSpecularIntensity.setValue(1,1,1);
	}



};

struct	GLInstanceRendererInternalData* GLInstancingRenderer::getInternalData()
{
	return m_data;
}


static GLuint	triangleShaderProgram;
static GLint	triangle_mvp_location=-1;
static GLint	triangle_vpos_location=-1;
static GLint	triangle_vUV_location=-1;
static GLint	triangle_vcol_location=-1;
static GLuint	triangleVertexBufferObject=0;
static GLuint	triangleVertexArrayObject=0;
static GLuint	triangleIndexVbo=0;

static GLuint   linesShader;        // The line renderer
static GLuint   instancingShader;        // The instancing renderer
static GLuint   instancingShaderPointSprite;        // The point sprite instancing renderer

static GLint	lines_ModelViewMatrix=0;
static GLint	lines_ProjectionMatrix=0;
static GLint	lines_position=0;
static GLint	lines_colour=0;
GLuint lineVertexBufferObject=0;
GLuint lineVertexArrayObject=0;
GLuint lineIndexVbo = 0;

GLuint linesVertexBufferObject=0;
GLuint linesVertexArrayObject=0;
GLuint linesIndexVbo = 0;

static GLint	ModelViewMatrix=0;
static GLint	ProjectionMatrix=0;
static GLint	regularLightDirIn=0;

static GLint	uniform_texture_diffuse = 0;
static GLint	screenWidthPointSprite=0;
static GLint	ModelViewMatrixPointSprite=0;
static GLint	ProjectionMatrixPointSprite=0;


GLInstancingRenderer::GLInstancingRenderer(int maxNumObjectCapacity, int maxShapeCapacityInBytes)
	:
	m_textureenabled(true),
	m_textureinitialized(false),
	m_screenWidth(0),
	m_screenHeight(0),
	m_upAxis(1),
	m_planeReflectionShapeIndex(-1)
{

	m_data = new InternalDataRenderer;
	m_data->m_maxNumObjectCapacity = maxNumObjectCapacity;
	m_data->m_maxShapeCapacityInBytes=maxShapeCapacityInBytes;

	m_data->m_totalNumInstances = 0;

	sData2 = m_data;

	m_data->m_instance_positions_ptr.resize(m_data->m_maxNumObjectCapacity*4);
	m_data->m_instance_quaternion_ptr.resize(m_data->m_maxNumObjectCapacity*4);
	m_data->m_instance_colors_ptr.resize(m_data->m_maxNumObjectCapacity*4);
	m_data->m_instance_scale_ptr.resize(m_data->m_maxNumObjectCapacity*3);

}

void GLInstancingRenderer::removeAllInstances()
{
	m_data->m_totalNumInstances = 0;

	for (int i=0;i<m_graphicsInstances.size();i++)
	{
		if (m_graphicsInstances[i]->m_index_vbo)
		{
			glDeleteBuffers(1,&m_graphicsInstances[i]->m_index_vbo);
		}
		if (m_graphicsInstances[i]->m_cube_vao)
		{
			glDeleteVertexArrays(1,&m_graphicsInstances[i]->m_cube_vao);
		}
		delete m_graphicsInstances[i];
	}
	m_graphicsInstances.clear();
	m_data->m_publicGraphicsInstances.exitHandles();
	m_data->m_publicGraphicsInstances.initHandles();
}


GLInstancingRenderer::~GLInstancingRenderer()
{
	glDeleteTextures(1,&m_data->m_defaultTexturehandle);

	removeAllInstances();

	sData2=0;

	if (m_data)
	{
		if (m_data->m_vbo)
			glDeleteBuffers(1,&m_data->m_vbo);
	}
	delete m_data;
}




int GLInstancingRenderer::getShapeIndexFromInstance(int srcIndex)
{
	b3PublicGraphicsInstance* pg = m_data->m_publicGraphicsInstances.getHandle(srcIndex);
	if (pg)
	{
		return pg->m_shapeIndex;
	}
	return -1;
}



bool GLInstancingRenderer::readSingleInstanceTransformToCPU(float* position, float* orientation, int shapeIndex)
{
	b3PublicGraphicsInstance* pg = m_data->m_publicGraphicsInstances.getHandle(shapeIndex);
	if (pg)
	{
		int srcIndex = pg->m_internalInstanceIndex;

		if ((srcIndex<m_data->m_totalNumInstances) && (srcIndex>=0))
		{
			position[0] = m_data->m_instance_positions_ptr[srcIndex*4+0];
			position[1] = m_data->m_instance_positions_ptr[srcIndex*4+1];
			position[2] = m_data->m_instance_positions_ptr[srcIndex*4+2];

			orientation[0] = m_data->m_instance_quaternion_ptr[srcIndex*4+0];
			orientation[1] = m_data->m_instance_quaternion_ptr[srcIndex*4+1];
			orientation[2] = m_data->m_instance_quaternion_ptr[srcIndex*4+2];
			orientation[3] = m_data->m_instance_quaternion_ptr[srcIndex*4+3];
			return true;
		}
	}

	return false;
}

void GLInstancingRenderer::writeSingleInstanceTransformToCPU(const float* position, const float* orientation, int srcIndex2)
{

	b3PublicGraphicsInstance* pg = m_data->m_publicGraphicsInstances.getHandle(srcIndex2);
	b3Assert(pg);

	if (pg==0)
		return;

	int srcIndex = pg->m_internalInstanceIndex;

	b3Assert(srcIndex<m_data->m_totalNumInstances);
	b3Assert(srcIndex>=0);
	m_data->m_instance_positions_ptr[srcIndex*4+0]=position[0];
	m_data->m_instance_positions_ptr[srcIndex*4+1]=position[1];
	m_data->m_instance_positions_ptr[srcIndex*4+2]=position[2];
	m_data->m_instance_positions_ptr[srcIndex*4+3]=1;

	m_data->m_instance_quaternion_ptr[srcIndex*4+0]=orientation[0];
	m_data->m_instance_quaternion_ptr[srcIndex*4+1]=orientation[1];
	m_data->m_instance_quaternion_ptr[srcIndex*4+2]=orientation[2];
	m_data->m_instance_quaternion_ptr[srcIndex*4+3]=orientation[3];

}


void GLInstancingRenderer::readSingleInstanceTransformFromCPU(int srcIndex2, float* position, float* orientation)
{
	b3PublicGraphicsInstance* pg = m_data->m_publicGraphicsInstances.getHandle(srcIndex2);
	b3Assert(pg);
	int srcIndex = pg->m_internalInstanceIndex;


	b3Assert(srcIndex<m_data->m_totalNumInstances);
	b3Assert(srcIndex>=0);
	position[0] = m_data->m_instance_positions_ptr[srcIndex*4+0];
	position[1] = m_data->m_instance_positions_ptr[srcIndex*4+1];
	position[2] = m_data->m_instance_positions_ptr[srcIndex*4+2];

	orientation[0] = m_data->m_instance_quaternion_ptr[srcIndex*4+0];
	orientation[1] = m_data->m_instance_quaternion_ptr[srcIndex*4+1];
	orientation[2] = m_data->m_instance_quaternion_ptr[srcIndex*4+2];
	orientation[3] = m_data->m_instance_quaternion_ptr[srcIndex*4+3];
}
void GLInstancingRenderer::writeSingleInstanceColorToCPU(const double* color, int srcIndex2)
{
	b3PublicGraphicsInstance* pg = m_data->m_publicGraphicsInstances.getHandle(srcIndex2);
	b3Assert(pg);
	int srcIndex = pg->m_internalInstanceIndex;

	int shapeIndex = pg->m_shapeIndex;
	b3GraphicsInstance* gfxObj = m_graphicsInstances[shapeIndex];
	m_data->m_instance_colors_ptr[srcIndex*4+0]=float(color[0]);
	m_data->m_instance_colors_ptr[srcIndex*4+1]=float(color[1]);
	m_data->m_instance_colors_ptr[srcIndex*4+2]=float(color[2]);
	m_data->m_instance_colors_ptr[srcIndex*4+3]=float(color[3]);
}

void GLInstancingRenderer::writeSingleInstanceColorToCPU(const float* color, int srcIndex2)
{
	b3PublicGraphicsInstance* pg = m_data->m_publicGraphicsInstances.getHandle(srcIndex2);
	b3Assert(pg);
	int srcIndex = pg->m_internalInstanceIndex;
	int shapeIndex = pg->m_shapeIndex;
	b3GraphicsInstance* gfxObj = m_graphicsInstances[shapeIndex];
	m_data->m_instance_colors_ptr[srcIndex*4+0]=color[0];
	m_data->m_instance_colors_ptr[srcIndex*4+1]=color[1];
	m_data->m_instance_colors_ptr[srcIndex*4+2]=color[2];
	m_data->m_instance_colors_ptr[srcIndex*4+3]=color[3];
}

void GLInstancingRenderer::writeSingleInstanceScaleToCPU(const float* scale, int srcIndex2)
{
	b3PublicGraphicsInstance* pg = m_data->m_publicGraphicsInstances.getHandle(srcIndex2);
	b3Assert(pg);
	int srcIndex = pg->m_internalInstanceIndex;

	m_data->m_instance_scale_ptr[srcIndex*3+0]=scale[0];
	m_data->m_instance_scale_ptr[srcIndex*3+1]=scale[1];
	m_data->m_instance_scale_ptr[srcIndex*3+2]=scale[2];
}

void GLInstancingRenderer::writeSingleInstanceSpecularColorToCPU(const double* specular, int srcIndex2)
{
	b3PublicGraphicsInstance* pg = m_data->m_publicGraphicsInstances.getHandle(srcIndex2);
	b3Assert(pg);
	int graphicsIndex = pg->m_internalInstanceIndex;

	int totalNumInstances = 0;

	int gfxObjIndex = -1;

	for (int i=0;i<m_graphicsInstances.size();i++)
	{
		totalNumInstances+=m_graphicsInstances[i]->m_numGraphicsInstances;
		if (srcIndex2<totalNumInstances)
		{
			gfxObjIndex = i;
			break;
		}
	}
	if (gfxObjIndex>0)
	{
		m_graphicsInstances[gfxObjIndex]->m_materialSpecularColor[0] = specular[0];
		m_graphicsInstances[gfxObjIndex]->m_materialSpecularColor[1] = specular[1];
		m_graphicsInstances[gfxObjIndex]->m_materialSpecularColor[2] = specular[2];
	}
}
void GLInstancingRenderer::writeSingleInstanceSpecularColorToCPU(const float* specular, int srcIndex2)
{
	b3PublicGraphicsInstance* pg = m_data->m_publicGraphicsInstances.getHandle(srcIndex2);
	b3Assert(pg);
	int srcIndex = pg->m_internalInstanceIndex;

	int totalNumInstances = 0;

	int gfxObjIndex = -1;

	for (int i=0;i<m_graphicsInstances.size();i++)
	{
		totalNumInstances+=m_graphicsInstances[i]->m_numGraphicsInstances;
		if (srcIndex2<totalNumInstances)
		{
			gfxObjIndex = i;
			break;
		}
	}
	if (gfxObjIndex>0)
	{
		m_graphicsInstances[gfxObjIndex]->m_materialSpecularColor[0] = specular[0];
		m_graphicsInstances[gfxObjIndex]->m_materialSpecularColor[1] = specular[1];
		m_graphicsInstances[gfxObjIndex]->m_materialSpecularColor[2] = specular[2];
	}
}


void GLInstancingRenderer::writeSingleInstanceScaleToCPU(const double* scale, int srcIndex2)
{
	b3PublicGraphicsInstance* pg = m_data->m_publicGraphicsInstances.getHandle(srcIndex2);
	b3Assert(pg);
	int srcIndex = pg->m_internalInstanceIndex;

	m_data->m_instance_scale_ptr[srcIndex*3+0]=scale[0];
	m_data->m_instance_scale_ptr[srcIndex*3+1]=scale[1];
	m_data->m_instance_scale_ptr[srcIndex*3+2]=scale[2];
}

void GLInstancingRenderer::writeSingleInstanceTransformToGPU(float* position, float* orientation, int objectUniqueId)
{
	glBindBuffer(GL_ARRAY_BUFFER, m_data->m_vbo);
	//glFlush();

	b3PublicGraphicsInstance* pg = m_data->m_publicGraphicsInstances.getHandle(objectUniqueId);
	b3Assert(pg);
	int objectIndex = pg->m_internalInstanceIndex;

	char* orgBase =  (char*)glMapBuffer( GL_ARRAY_BUFFER,GL_READ_WRITE);
	//b3GraphicsInstance* gfxObj = m_graphicsInstances[k];
	int totalNumInstances= 0;
	for (int k=0;k<m_graphicsInstances.size();k++)
	{
		b3GraphicsInstance* gfxObj = m_graphicsInstances[k];
		totalNumInstances+=gfxObj->m_numGraphicsInstances;
	}

	int POSITION_BUFFER_SIZE = (totalNumInstances*sizeof(float)*4);

	char* base = orgBase;

	float* positions = (float*)(base+m_data->m_maxShapeCapacityInBytes);
	float* orientations = (float*)(base+m_data->m_maxShapeCapacityInBytes + POSITION_BUFFER_SIZE);

	positions[objectIndex*4] = position[0];
	positions[objectIndex*4+1] = position[1];
	positions[objectIndex*4+2] = position[2];
	positions[objectIndex*4+3] = position[3];

	orientations [objectIndex*4] = orientation[0];
	orientations [objectIndex*4+1] = orientation[1];
	orientations [objectIndex*4+2] = orientation[2];
	orientations [objectIndex*4+3] = orientation[3];

	glUnmapBuffer( GL_ARRAY_BUFFER);
	//glFlush();
}


void GLInstancingRenderer::writeTransforms()
{
	{
		//B3_PROFILE("b3Assert(glGetError() 1");
		b3Assert(glGetError() ==GL_NO_ERROR);
	}
	{
		//B3_PROFILE("glBindBuffer");
		glBindBuffer(GL_ARRAY_BUFFER, m_data->m_vbo);
	}

	{
		//B3_PROFILE("glFlush()");
		//without the flush, the glBufferSubData can spike to really slow (seconds slow)
		glFlush();
	}

	{
		//B3_PROFILE("b3Assert(glGetError() 2");
		b3Assert(glGetError() ==GL_NO_ERROR);
	}


#ifdef B3_DEBUG
	{
		//B3_PROFILE("m_data->m_totalNumInstances == totalNumInstances");
		int totalNumInstances= 0;
		for (int k=0;k<m_graphicsInstances.size();k++)
		{
			b3GraphicsInstance* gfxObj = m_graphicsInstances[k];
			totalNumInstances+=gfxObj->m_numGraphicsInstances;
		}
		b3Assert(m_data->m_totalNumInstances == totalNumInstances);
	}
#endif//B3_DEBUG

	int POSITION_BUFFER_SIZE = (m_data->m_totalNumInstances*sizeof(float)*4);
	int ORIENTATION_BUFFER_SIZE = (m_data->m_totalNumInstances*sizeof(float)*4);
	int COLOR_BUFFER_SIZE = (m_data->m_totalNumInstances*sizeof(float)*4);
//	int SCALE_BUFFER_SIZE = (totalNumInstances*sizeof(float)*3);

	//	printf("m_data->m_totalNumInstances = %d\n", m_data->m_totalNumInstances);
	{
		// B3_PROFILE("glBufferSubData pos");
		glBufferSubData(	GL_ARRAY_BUFFER,m_data->m_maxShapeCapacityInBytes,m_data->m_totalNumInstances*sizeof(float)*4,
 						&m_data->m_instance_positions_ptr[0]);
	}
	{
		// B3_PROFILE("glBufferSubData orn");
		glBufferSubData(	GL_ARRAY_BUFFER,m_data->m_maxShapeCapacityInBytes+POSITION_BUFFER_SIZE,m_data->m_totalNumInstances*sizeof(float)*4,
 						&m_data->m_instance_quaternion_ptr[0]);
	}
	{
		// B3_PROFILE("glBufferSubData color");
		glBufferSubData(	GL_ARRAY_BUFFER,m_data->m_maxShapeCapacityInBytes+ POSITION_BUFFER_SIZE+ORIENTATION_BUFFER_SIZE, m_data->m_totalNumInstances*sizeof(float)*4,
 						&m_data->m_instance_colors_ptr[0]);
	}
	{
		// B3_PROFILE("glBufferSubData scale");
		glBufferSubData(	GL_ARRAY_BUFFER, m_data->m_maxShapeCapacityInBytes+POSITION_BUFFER_SIZE+ORIENTATION_BUFFER_SIZE+COLOR_BUFFER_SIZE,m_data->m_totalNumInstances*sizeof(float)*3,
					 	&m_data->m_instance_scale_ptr[0]);
	}

	{
		// B3_PROFILE("glBindBuffer 2");
		glBindBuffer(GL_ARRAY_BUFFER, 0);//m_data->m_vbo);
	}
	{
		// B3_PROFILE("b3Assert(glGetError() 4");
		b3Assert(glGetError() ==GL_NO_ERROR);
	}

}

int GLInstancingRenderer::registerGraphicsInstance(int shapeIndex, const double* pos1, const double* orn1, const double* color1, const double* scaling1)
{
    float pos[4] = {(float)pos1[0],(float)pos1[1],(float)pos1[2],(float)pos1[3]};
    float orn[4] = {(float)orn1[0],(float)orn1[1],(float)orn1[2],(float)orn1[3]};
    float color[4] = {(float)color1[0],(float)color1[1],(float)color1[2],(float)color1[3]};
    float scaling[4] = {(float)scaling1[0],(float)scaling1[1],(float)scaling1[2],(float)scaling1[3]};
    return registerGraphicsInstance(shapeIndex,pos,orn,color,scaling);
}

void GLInstancingRenderer::rebuildGraphicsInstances()
{
	m_data->m_totalNumInstances = 0;

	b3AlignedObjectArray<int> usedObjects;
	m_data->m_publicGraphicsInstances.getUsedHandles(usedObjects);

	for (int i=0;i<usedObjects.size();i++)
	{
		int srcIndex2 = usedObjects[i];
		b3PublicGraphicsInstance* pg = m_data->m_publicGraphicsInstances.getHandle(srcIndex2);
		b3Assert(pg);
		int srcIndex = pg->m_internalInstanceIndex;

		pg->m_position[0] = m_data->m_instance_positions_ptr[srcIndex*4+0];
		pg->m_position[1] = m_data->m_instance_positions_ptr[srcIndex*4+1];
		pg->m_position[2] = m_data->m_instance_positions_ptr[srcIndex*4+2];
		pg->m_orientation[0] = m_data->m_instance_quaternion_ptr[srcIndex*4+0];
		pg->m_orientation[1] = m_data->m_instance_quaternion_ptr[srcIndex*4+1];
		pg->m_orientation[2] = m_data->m_instance_quaternion_ptr[srcIndex*4+2];
		pg->m_orientation[3] = m_data->m_instance_quaternion_ptr[srcIndex*4+3];
		pg->m_color[0] = m_data->m_instance_colors_ptr[srcIndex*4+0];
		pg->m_color[1] = m_data->m_instance_colors_ptr[srcIndex*4+1];
		pg->m_color[2] = m_data->m_instance_colors_ptr[srcIndex*4+2];
		pg->m_color[3] = m_data->m_instance_colors_ptr[srcIndex*4+3];
		pg->m_scale[0] = m_data->m_instance_scale_ptr[srcIndex*3+0];
		pg->m_scale[1] = m_data->m_instance_scale_ptr[srcIndex*3+1];
		pg->m_scale[2] = m_data->m_instance_scale_ptr[srcIndex*3+2];
	}
	for (int i=0;i<m_graphicsInstances.size();i++)
	{
		m_graphicsInstances[i]->m_numGraphicsInstances = 0;
		m_graphicsInstances[i]->m_instanceOffset = 0;
		m_graphicsInstances[i]->m_tempObjectUids.clear();
	}
	for (int i=0;i<usedObjects.size();i++)
	{
		int srcIndex2 = usedObjects[i];
		b3PublicGraphicsInstance* pg = m_data->m_publicGraphicsInstances.getHandle(srcIndex2);
		if (pg && pg->m_shapeIndex < m_graphicsInstances.size() && pg->m_shapeIndex >=0)
		{
			m_graphicsInstances[pg->m_shapeIndex]->m_tempObjectUids.push_back(srcIndex2);
		}
	}

	int curOffset = 0;
	m_data->m_totalNumInstances = 0;

	for (int i=0;i<m_graphicsInstances.size();i++)
	{
		m_graphicsInstances[i]->m_instanceOffset = curOffset;
		m_graphicsInstances[i]->m_numGraphicsInstances = 0;

		for (int g=0;g<m_graphicsInstances[i]->m_tempObjectUids.size();g++)
		{
			curOffset++;
			int objectUniqueId = m_graphicsInstances[i]->m_tempObjectUids[g];
			b3PublicGraphicsInstance* pg = m_data->m_publicGraphicsInstances.getHandle(objectUniqueId);

			registerGraphicsInstanceInternal(objectUniqueId,pg->m_position,pg->m_orientation,pg->m_color,pg->m_scale);
		}
	}

}

void GLInstancingRenderer::removeGraphicsInstance(int instanceUid)
{
	b3PublicGraphicsInstance* pg = m_data->m_publicGraphicsInstances.getHandle(instanceUid);
	b3Assert(pg);
	if (pg)
	{
		m_data->m_publicGraphicsInstances.freeHandle(instanceUid);
		rebuildGraphicsInstances();
	}
}


int GLInstancingRenderer::registerGraphicsInstanceInternal(int newUid, const float* position, const float* quaternion, const float* color, const float* scaling)
{
	b3PublicGraphicsInstance* pg = m_data->m_publicGraphicsInstances.getHandle(newUid);
	int shapeIndex = pg->m_shapeIndex;

	b3GraphicsInstance* gfxObj = m_graphicsInstances[shapeIndex];
	int index = gfxObj->m_numGraphicsInstances + gfxObj->m_instanceOffset;
	pg->m_internalInstanceIndex = index;

	int maxElements = m_data->m_instance_positions_ptr.size();
	if (index*4<maxElements)
	{
		m_data->m_instance_positions_ptr[index*4]=position[0];
		m_data->m_instance_positions_ptr[index*4+1]=position[1];
		m_data->m_instance_positions_ptr[index*4+2]=position[2];
		m_data->m_instance_positions_ptr[index*4+3]=1;

		m_data->m_instance_quaternion_ptr[index*4]=quaternion[0];
		m_data->m_instance_quaternion_ptr[index*4+1]=quaternion[1];
		m_data->m_instance_quaternion_ptr[index*4+2]=quaternion[2];
		m_data->m_instance_quaternion_ptr[index*4+3]=quaternion[3];

		m_data->m_instance_colors_ptr[index*4]=color[0];
		m_data->m_instance_colors_ptr[index*4+1]=color[1];
		m_data->m_instance_colors_ptr[index*4+2]=color[2];
		m_data->m_instance_colors_ptr[index*4+3]=color[3];

		m_data->m_instance_scale_ptr[index*3] = scaling[0];
		m_data->m_instance_scale_ptr[index*3+1] = scaling[1];
		m_data->m_instance_scale_ptr[index*3+2] = scaling[2];

		gfxObj->m_numGraphicsInstances++;
		m_data->m_totalNumInstances++;
	} else
	{
		b3Error("registerGraphicsInstance out of range, %d\n", maxElements);
		return -1;
	}
	return newUid;//gfxObj->m_numGraphicsInstances;
}

int GLInstancingRenderer::registerGraphicsInstance(int shapeIndex, const float* position, const float* quaternion, const float* color, const float* scaling)
{
	int newUid = m_data->m_publicGraphicsInstances.allocHandle();
	b3PublicGraphicsInstance* pg = m_data->m_publicGraphicsInstances.getHandle(newUid);
	pg->m_shapeIndex = shapeIndex;

	//b3Assert(shapeIndex == (m_graphicsInstances.size()-1));
	b3Assert(m_graphicsInstances.size()<m_data->m_maxNumObjectCapacity-1);
	if (shapeIndex == (m_graphicsInstances.size()-1))
	{
		registerGraphicsInstanceInternal(newUid, position,quaternion,color,scaling);
	} else
	{

		int srcIndex = m_data->m_totalNumInstances++;
		pg->m_internalInstanceIndex = srcIndex;

		m_data->m_instance_positions_ptr[srcIndex*4+0] = position[0];
		m_data->m_instance_positions_ptr[srcIndex*4+1] = position[1];
		m_data->m_instance_positions_ptr[srcIndex*4+2] = position[2];
		m_data->m_instance_positions_ptr[srcIndex*4+3] = 1.;

		m_data->m_instance_quaternion_ptr[srcIndex*4+0] = quaternion[0];
		m_data->m_instance_quaternion_ptr[srcIndex*4+1] = quaternion[1];
		m_data->m_instance_quaternion_ptr[srcIndex*4+2] = quaternion[2];
		m_data->m_instance_quaternion_ptr[srcIndex*4+3] = quaternion[3];

		m_data->m_instance_colors_ptr[srcIndex*4+0] = color[0];
		m_data->m_instance_colors_ptr[srcIndex*4+1] = color[1];
		m_data->m_instance_colors_ptr[srcIndex*4+2] = color[2];
		m_data->m_instance_colors_ptr[srcIndex*4+3] = color[3];

		m_data->m_instance_scale_ptr[srcIndex*3+0] = scaling[0];
		m_data->m_instance_scale_ptr[srcIndex*3+1] = scaling[1];
		m_data->m_instance_scale_ptr[srcIndex*3+2] = scaling[2];


		rebuildGraphicsInstances();
	}

	return newUid;
}

void GLInstancingRenderer::removeTexture(int textureIndex)
{
	if ((textureIndex >= 0) && (textureIndex < m_data->m_textureHandles.size()))
	{
		InternalTextureHandle& h = m_data->m_textureHandles[textureIndex];
		glDeleteTextures(1, &h.m_glTexture);
	}
}

int	GLInstancingRenderer::registerTexture(const unsigned char* texels, int width, int height, bool flipPixelsY)
{
	
	B3_PROFILE("GLInstancingRenderer::registerTexture");
	b3Assert(glGetError() ==GL_NO_ERROR);
	glActiveTexture(GL_TEXTURE0);
	int textureIndex = m_data->m_textureHandles.size();
  //  const GLubyte*	image= (const GLubyte*)texels;
	GLuint textureHandle;
	glGenTextures(1,(GLuint*)&textureHandle);
	glBindTexture(GL_TEXTURE_2D,textureHandle);

	b3Assert(glGetError() ==GL_NO_ERROR);

	InternalTextureHandle h;
    h.m_glTexture = textureHandle;
    h.m_width = width;
    h.m_height = height;
	h.m_enableFiltering = true;
	m_data->m_textureHandles.push_back(h);
	if (texels)
	{
		B3_PROFILE("updateTexture");
		updateTexture(textureIndex, texels, flipPixelsY);
	}
	return textureIndex;
}


void    GLInstancingRenderer::replaceTexture(int shapeIndex, int textureId)
{
	if ((shapeIndex >=0) && (shapeIndex < m_graphicsInstances.size()))
	{
		b3GraphicsInstance* gfxObj = m_graphicsInstances[shapeIndex];
		if (textureId>=0 && textureId < m_data->m_textureHandles.size())
		{
			gfxObj->m_textureIndex = textureId;
			gfxObj->m_flags |= eGfxHasTexture;

		}
	}
}

void    GLInstancingRenderer::updateTexture(int textureIndex, const unsigned char* texels, bool flipPixelsY)
{
	B3_PROFILE("updateTexture");
    if ((textureIndex>=0) && (textureIndex < m_data->m_textureHandles.size()))
    {
        glActiveTexture(GL_TEXTURE0);
        b3Assert(glGetError() ==GL_NO_ERROR);
        InternalTextureHandle& h = m_data->m_textureHandles[textureIndex];
		glBindTexture(GL_TEXTURE_2D,h.m_glTexture);
        b3Assert(glGetError() ==GL_NO_ERROR);

		if (flipPixelsY)
		{
			B3_PROFILE("flipPixelsY");
			//textures need to be flipped for OpenGL...
			b3AlignedObjectArray<unsigned char> flippedTexels;
			flippedTexels.resize(h.m_width* h.m_height * 3);

			for (int j = 0; j < h.m_height; j++)
			{
				for (int i = 0; i < h.m_width; i++)
				{
					flippedTexels[(i + j*h.m_width) * 3] =      texels[(i + (h.m_height - 1 -j )*h.m_width) * 3];
					flippedTexels[(i + j*h.m_width) * 3+1] =    texels[(i + (h.m_height - 1 - j)*h.m_width) * 3+1];
					flippedTexels[(i + j*h.m_width) * 3+2] =    texels[(i + (h.m_height - 1 - j)*h.m_width) * 3+2];
				}
			}

			glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, h.m_width,h.m_height,0,GL_RGB,GL_UNSIGNED_BYTE,&flippedTexels[0]);
		} else
		{
			glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, h.m_width,h.m_height,0,GL_RGB,GL_UNSIGNED_BYTE,&texels[0]);
		}
        b3Assert(glGetError() ==GL_NO_ERROR);
		if (h.m_enableFiltering)
		{
			B3_PROFILE("glGenerateMipmap");
	        glGenerateMipmap(GL_TEXTURE_2D);
		}
        b3Assert(glGetError() ==GL_NO_ERROR);
    }
}

void GLInstancingRenderer::activateTexture(int textureIndex)
{
    glActiveTexture(GL_TEXTURE0);

    if (textureIndex>=0 && textureIndex < m_data->m_textureHandles.size())
    {
        glBindTexture(GL_TEXTURE_2D,m_data->m_textureHandles[textureIndex].m_glTexture);
    } else
    {
        glBindTexture(GL_TEXTURE_2D,0);
    }
}

void GLInstancingRenderer::updateShape(int shapeIndex, const float* vertices)
{
	b3GraphicsInstance* gfxObj = m_graphicsInstances[shapeIndex];
	int numvertices = gfxObj->m_numVertices;

	glBindBuffer(GL_ARRAY_BUFFER, m_data->m_vbo);
	int vertexStrideInBytes = 9*sizeof(float);
	int sz = numvertices*vertexStrideInBytes;
#if 0
	char* dest=  (char*)glMapBuffer( GL_ARRAY_BUFFER,GL_WRITE_ONLY);//GL_WRITE_ONLY
	memcpy(dest+vertexStrideInBytes*gfxObj->m_vertexArrayOffset,vertices,sz);
	glUnmapBuffer( GL_ARRAY_BUFFER);
#else
	glBufferSubData(	GL_ARRAY_BUFFER,vertexStrideInBytes*gfxObj->m_vertexArrayOffset,sz,
 						vertices);
#endif

}

int GLInstancingRenderer::registerShape(const float* vertices, int numvertices, const int* indices, int numIndices,int primitiveType, int textureId)
{
	b3GraphicsInstance* gfxObj = new b3GraphicsInstance;

	if (textureId>=0)
	{
		gfxObj->m_textureIndex = textureId;
		gfxObj->m_flags |= eGfxHasTexture;
	}

	gfxObj->m_primitiveType = primitiveType;

	if (m_graphicsInstances.size())
	{
		b3GraphicsInstance* prevObj = m_graphicsInstances[m_graphicsInstances.size()-1];
		gfxObj->m_instanceOffset = prevObj->m_instanceOffset + prevObj->m_numGraphicsInstances;
		gfxObj->m_vertexArrayOffset = prevObj->m_vertexArrayOffset + prevObj->m_numVertices;
	} else
	{
		gfxObj->m_instanceOffset = 0;
	}

	m_graphicsInstances.push_back(gfxObj);
	gfxObj->m_numIndices = numIndices;
	gfxObj->m_numVertices = numvertices;



	int vertexStrideInBytes = 9*sizeof(float);
	int sz = numvertices*vertexStrideInBytes;
	int totalUsed = vertexStrideInBytes*gfxObj->m_vertexArrayOffset+sz;
	b3Assert(totalUsed<m_data->m_maxShapeCapacityInBytes);
	if (totalUsed>=m_data->m_maxShapeCapacityInBytes)
	{
		return -1;
	}

	glBindBuffer(GL_ARRAY_BUFFER, m_data->m_vbo);
	glGenBuffers(1, &gfxObj->m_index_vbo);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, gfxObj->m_index_vbo);
	int indexBufferSizeInBytes = gfxObj->m_numIndices*sizeof(int);

	glBufferData(GL_ELEMENT_ARRAY_BUFFER, indexBufferSizeInBytes, NULL, GL_STATIC_DRAW);
	glBufferSubData(GL_ELEMENT_ARRAY_BUFFER,0,indexBufferSizeInBytes,indices);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	glGenVertexArrays(1, &gfxObj->m_cube_vao);
	glBindVertexArray(gfxObj->m_cube_vao);
	glBindBuffer(GL_ARRAY_BUFFER, m_data->m_vbo);
	glBindVertexArray(0);
	glBindBuffer(GL_ARRAY_BUFFER,0);
	glBindVertexArray(0);


	return m_graphicsInstances.size()-1;
}




void GLInstancingRenderer::InitShaders()
{

	int POSITION_BUFFER_SIZE = (m_data->m_maxNumObjectCapacity*sizeof(float)*4);
	int ORIENTATION_BUFFER_SIZE = (m_data->m_maxNumObjectCapacity*sizeof(float)*4);
	int COLOR_BUFFER_SIZE = (m_data->m_maxNumObjectCapacity*sizeof(float)*4);
	int SCALE_BUFFER_SIZE = (m_data->m_maxNumObjectCapacity*sizeof(float)*3);

	{
		triangleShaderProgram = gltLoadShaderPair(triangleVertexShaderText,triangleFragmentShader);

		triangle_mvp_location = glGetUniformLocation(triangleShaderProgram, "MVP");
		triangle_vcol_location = glGetUniformLocation(triangleShaderProgram, "vCol");

		glLinkProgram(triangleShaderProgram);
		glUseProgram(triangleShaderProgram);

		glGenVertexArrays(1, &triangleVertexArrayObject);
		glBindVertexArray(triangleVertexArrayObject);

		glGenBuffers(1, &triangleVertexBufferObject);
		glGenBuffers(1, &triangleIndexVbo);

		int sz = MAX_TRIANGLES_IN_BATCH*sizeof(GfxVertexFormat0);
		glBindVertexArray(triangleVertexArrayObject);
		glBindBuffer(GL_ARRAY_BUFFER, triangleVertexBufferObject);
		glBufferData(GL_ARRAY_BUFFER, sz, 0, GL_DYNAMIC_DRAW);

		glBindVertexArray(0);
	}

	linesShader = gltLoadShaderPair(linesVertexShader,linesFragmentShader);
	lines_ModelViewMatrix = glGetUniformLocation(linesShader, "ModelViewMatrix");
	lines_ProjectionMatrix = glGetUniformLocation(linesShader, "ProjectionMatrix");
	lines_colour=glGetUniformLocation(linesShader, "colour");
	lines_position=glGetAttribLocation(linesShader, "position");
	glLinkProgram(linesShader);
	glUseProgram(linesShader);

	{
		glGenVertexArrays(1, &linesVertexArrayObject);
		glBindVertexArray(linesVertexArrayObject);

		glGenBuffers(1, &linesVertexBufferObject);
		glGenBuffers(1, &linesIndexVbo);

		int sz = MAX_LINES_IN_BATCH*sizeof(b3Vector3);
		glBindVertexArray(linesVertexArrayObject);
		glBindBuffer(GL_ARRAY_BUFFER, linesVertexBufferObject);
		glBufferData(GL_ARRAY_BUFFER, sz, 0, GL_DYNAMIC_DRAW);

		glBindVertexArray(0);
	}
	{
		glGenVertexArrays(1, &lineVertexArrayObject);
		glBindVertexArray(lineVertexArrayObject);

		glGenBuffers(1, &lineVertexBufferObject);
		glGenBuffers(1, &lineIndexVbo);

		int sz = MAX_POINTS_IN_BATCH*sizeof(b3Vector3);
		glBindVertexArray(lineVertexArrayObject);
		glBindBuffer(GL_ARRAY_BUFFER, lineVertexBufferObject);
		glBufferData(GL_ARRAY_BUFFER, sz, 0, GL_DYNAMIC_DRAW);

		glBindVertexArray(0);
	}

	glGetIntegerv(GL_SMOOTH_LINE_WIDTH_RANGE, lineWidthRange);

	instancingShader = gltLoadShaderPair(instancingVertexShader,instancingFragmentShader);
	glLinkProgram(instancingShader);
	glUseProgram(instancingShader);
	ModelViewMatrix = glGetUniformLocation(instancingShader, "ModelViewMatrix");
	ProjectionMatrix = glGetUniformLocation(instancingShader, "ProjectionMatrix");
	uniform_texture_diffuse = glGetUniformLocation(instancingShader, "Diffuse");
	regularLightDirIn  = glGetUniformLocation(instancingShader,"lightDirIn");
	glUseProgram(0);

	instancingShaderPointSprite = gltLoadShaderPair(pointSpriteVertexShader,pointSpriteFragmentShader);
	glUseProgram(instancingShaderPointSprite);
	ModelViewMatrixPointSprite = glGetUniformLocation(instancingShaderPointSprite, "ModelViewMatrix");
	ProjectionMatrixPointSprite = glGetUniformLocation(instancingShaderPointSprite, "ProjectionMatrix");
	screenWidthPointSprite = glGetUniformLocation(instancingShaderPointSprite, "screenWidth");
	glUseProgram(0);


	glGenBuffers(1, &m_data->m_vbo);
    checkError("glGenBuffers");
	glBindBuffer(GL_ARRAY_BUFFER, m_data->m_vbo);

	int size = m_data->m_maxShapeCapacityInBytes  + POSITION_BUFFER_SIZE+ORIENTATION_BUFFER_SIZE+COLOR_BUFFER_SIZE+SCALE_BUFFER_SIZE;
	m_data->m_vboSize = size;

	glBufferData(GL_ARRAY_BUFFER, size, 0, GL_DYNAMIC_DRAW);  //GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER,0);
	glBindVertexArray(0);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}




void GLInstancingRenderer::init()
{
	b3Assert(glGetError() ==GL_NO_ERROR);

	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LESS);

    b3Assert(glGetError() ==GL_NO_ERROR);

	{
		B3_PROFILE("texture");
		if(m_textureenabled)
		{
			if(!m_textureinitialized)
			{
				glActiveTexture(GL_TEXTURE0);

				GLubyte*	image=new GLubyte[256*256*3];
				for(int y=0;y<256;++y)
				{
					GLubyte*	pi=image+y*256*3;
					for(int x=0;x<256;++x)
					{
						if (x<2||y<2||x>253||y>253)
						{
							pi[0]=255;//0;
							pi[1]=255;//0;
							pi[2]=255;//0;
						} else
						{
							pi[0]=255;
							pi[1]=255;
							pi[2]=255;
						}

						pi+=3;
					}
				}

				glGenTextures(1,(GLuint*)&m_data->m_defaultTexturehandle);
				glBindTexture(GL_TEXTURE_2D,m_data->m_defaultTexturehandle);
				b3Assert(glGetError() ==GL_NO_ERROR);

				glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 256,256,0,GL_RGB,GL_UNSIGNED_BYTE,image);
				glGenerateMipmap(GL_TEXTURE_2D);

				b3Assert(glGetError() ==GL_NO_ERROR);

				delete[] image;
				m_textureinitialized=true;
			}

			b3Assert(glGetError() ==GL_NO_ERROR);

			glBindTexture(GL_TEXTURE_2D,m_data->m_defaultTexturehandle);
			b3Assert(glGetError() ==GL_NO_ERROR);

		} else
		{
			glDisable(GL_TEXTURE_2D);
			b3Assert(glGetError() ==GL_NO_ERROR);
		}
	}
	// glEnable(GL_COLOR_MATERIAL);

	b3Assert(glGetError() ==GL_NO_ERROR);

	// glEnable(GL_CULL_FACE);
	// glCullFace(GL_BACK);


	for(int i = 0; i < 16; i++) {
		glViewportIndexedf(i, 0, (m_screenHeight / 16)*i, m_screenWidth, m_screenHeight / 16);
	}

}

void	GLInstancingRenderer::resize(int width, int height)
{
	m_screenWidth = width;
	m_screenHeight = height;
}


const CommonCameraInterface* GLInstancingRenderer::getActiveCamera() const
{
	return m_data->m_activeCamera;
}

CommonCameraInterface* GLInstancingRenderer::getActiveCamera()
{
	return m_data->m_activeCamera;
}


void GLInstancingRenderer::setActiveCamera(CommonCameraInterface* cam)
{
	m_data->m_activeCamera = cam;
}

void GLInstancingRenderer::setLightSpecularIntensity(const float lightSpecularIntensity[3])
{
	m_data->m_lightSpecularIntensity[0] = lightSpecularIntensity[0];
	m_data->m_lightSpecularIntensity[1] = lightSpecularIntensity[1];
	m_data->m_lightSpecularIntensity[2] = lightSpecularIntensity[2];
}


void GLInstancingRenderer::setLightPosition(const float lightPos[3])
{
	m_data->m_lightPos[0] = lightPos[0];
	m_data->m_lightPos[1] = lightPos[1];
	m_data->m_lightPos[2] = lightPos[2];
}

void GLInstancingRenderer::setLightPosition(const double lightPos[3])
{
	m_data->m_lightPos[0] = lightPos[0];
	m_data->m_lightPos[1] = lightPos[1];
	m_data->m_lightPos[2] = lightPos[2];
}


void writeTextureToPng(int textureWidth, int textureHeight, const char* fileName, int numComponents)
{
	b3Assert(glGetError() ==GL_NO_ERROR);
	glPixelStorei(GL_PACK_ALIGNMENT,4);

	glReadBuffer(GL_NONE);
	float* orgPixels = (float*)malloc(textureWidth*textureHeight*numComponents*4);
	char* pixels = (char*)malloc(textureWidth*textureHeight*numComponents*4);
	glReadPixels(0,0,textureWidth, textureHeight, GL_DEPTH_COMPONENT, GL_FLOAT, orgPixels);
	b3Assert(glGetError() ==GL_NO_ERROR);
	for (int j=0;j<textureHeight;j++)
	{
		for (int i=0;i<textureWidth;i++)
		{
			float val = orgPixels[(j*textureWidth+i)];
			if (val!=1.f)
			{
				//printf("val[%d,%d]=%f\n", i,j,val);
			}
			pixels[(j*textureWidth+i)*numComponents]=char(orgPixels[(j*textureWidth+i)]*255.f);
			pixels[(j*textureWidth+i)*numComponents+1]=0;//255.f;
			pixels[(j*textureWidth+i)*numComponents+2]=0;//255.f;
			pixels[(j*textureWidth+i)*numComponents+3]=127;
		}
	}

	stbi_write_png(fileName, textureWidth,textureHeight, numComponents, pixels, textureWidth*numComponents);
	free(pixels);
}


struct PointerCaster
{
	union {
		int m_baseIndex;
		GLvoid* m_pointer;
	};

	PointerCaster()
	:m_pointer(0)
		{
		}


};



#if 0
static void    b3CreateFrustum(
                        float left,
                        float right,
                        float bottom,
                        float top,
                        float nearVal,
                        float farVal,
                        float frustum[16])
{

    frustum[0*4+0] = (float(2) * nearVal) / (right - left);
    frustum[0*4+1] = float(0);
    frustum[0*4+2] = float(0);
    frustum[0*4+3] = float(0);

    frustum[1*4+0] = float(0);
    frustum[1*4+1] = (float(2) * nearVal) / (top - bottom);
    frustum[1*4+2] = float(0);
    frustum[1*4+3] = float(0);

    frustum[2*4+0] = (right + left) / (right - left);
    frustum[2*4+1] = (top + bottom) / (top - bottom);
    frustum[2*4+2] = -(farVal + nearVal) / (farVal - nearVal);
    frustum[2*4+3] = float(-1);

    frustum[3*4+0] = float(0);
    frustum[3*4+1] = float(0);
    frustum[3*4+2] = -(float(2) * farVal * nearVal) / (farVal - nearVal);
    frustum[3*4+3] = float(0);

}
#endif

static void b3Matrix4x4Mul(GLfloat aIn[4][4], GLfloat bIn[4][4], GLfloat result[4][4])
{
	for (int j=0;j<4;j++)
		for (int i=0;i<4;i++)
			result[j][i] = aIn[0][i] * bIn[j][0] + aIn[1][i] * bIn[j][1] + aIn[2][i] * bIn[j][2] + aIn[3][i] * bIn[j][3];
}

static void b3Matrix4x4Mul16(GLfloat aIn[16], GLfloat bIn[16], GLfloat result[16])
{
	for (int j=0;j<4;j++)
		for (int i=0;i<4;i++)
			result[j*4+i] = aIn[0*4+i] * bIn[j*4+0] + aIn[1*4+i] * bIn[j*4+1] + aIn[2*4+i] * bIn[j*4+2] + aIn[3*4+i] * bIn[j*4+3];
}


static void b3CreateDiagonalMatrix(GLfloat value, GLfloat result[4][4])
{
	for (int i=0;i<4;i++)
	{
		for (int j=0;j<4;j++)
		{
			if (i==j)
			{
				result[i][j] = value;
			} else
			{
				result[i][j] = 0.f;
			}
		}
	}
}

static void b3CreateOrtho(GLfloat left, GLfloat right, GLfloat bottom, GLfloat top, GLfloat zNear, GLfloat zFar, GLfloat result[4][4])
{
	b3CreateDiagonalMatrix(1.f,result);

	result[0][0] = 2.f / (right - left);
	result[1][1] = 2.f / (top - bottom);
	result[2][2] = - 2.f / (zFar - zNear);
	result[3][0] = - (right + left) / (right - left);
	result[3][1] = - (top + bottom) / (top - bottom);
	result[3][2] = - (zFar + zNear) / (zFar - zNear);
}

static void    b3CreateLookAt(const b3Vector3& eye, const b3Vector3& center,const b3Vector3& up, GLfloat result[16])
{
    b3Vector3 f = (center - eye).normalized();
    b3Vector3 u = up.normalized();
    b3Vector3 s = (f.cross(u)).normalized();
    u = s.cross(f);

    result[0*4+0] = s.x;
    result[1*4+0] = s.y;
    result[2*4+0] = s.z;

	result[0*4+1] = u.x;
    result[1*4+1] = u.y;
    result[2*4+1] = u.z;

    result[0*4+2] =-f.x;
    result[1*4+2] =-f.y;
    result[2*4+2] =-f.z;

	result[0*4+3] = 0.f;
    result[1*4+3] = 0.f;
    result[2*4+3] = 0.f;

    result[3*4+0] = -s.dot(eye);
    result[3*4+1] = -u.dot(eye);
    result[3*4+2] = f.dot(eye);
    result[3*4+3] = 1.f;
}




void GLInstancingRenderer::drawTexturedTriangleMesh(float worldPosition[3], float worldOrientation[4], const float* vertices, int numvertices, const unsigned int* indices, int numIndices, float colorRGBA[4], int textureIndex, int vertexLayout)
{
	float projectionMatrix[16], viewMatrix[16];
	m_data->m_activeCamera->getCameraProjectionMatrix(projectionMatrix);
	m_data->m_activeCamera->getCameraViewMatrix(viewMatrix);

	int sz = sizeof(GfxVertexFormat0);

	glActiveTexture(GL_TEXTURE0);
	activateTexture(textureIndex);
	checkError("activateTexture");

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	glUseProgram(triangleShaderProgram);

	b3Quaternion orn(worldOrientation[0],worldOrientation[1],worldOrientation[2],worldOrientation[3]);
	b3Vector3 pos = b3MakeVector3(worldPosition[0],worldPosition[1],worldPosition[2]);


	b3Transform worldTrans(orn,pos);
	b3Scalar worldMatUnk[16];
	worldTrans.getOpenGLMatrix(worldMatUnk);
	float modelMat[16];
	for (int i=0;i<16;i++)
	{
		modelMat[i] = worldMatUnk[i];
	}
	float viewProjection[16];
	b3Matrix4x4Mul16(projectionMatrix,viewMatrix,viewProjection);
	float MVP[16];
	b3Matrix4x4Mul16(viewProjection,modelMat,MVP);
	glUniformMatrix4fv(triangle_mvp_location, 1, GL_FALSE, (const GLfloat*) MVP);
	checkError("glUniformMatrix4fv");

	glUniform3f(triangle_vcol_location,colorRGBA[0],colorRGBA[1],colorRGBA[2]);
	checkError("glUniform3f");

	glBindVertexArray(triangleVertexArrayObject);
	checkError("glBindVertexArray");

	glBindBuffer(GL_ARRAY_BUFFER, triangleVertexBufferObject);
	checkError("glBindBuffer");

	glBufferData(GL_ARRAY_BUFFER, sizeof(GfxVertexFormat0)*numvertices, 0, GL_DYNAMIC_DRAW);
	glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(GfxVertexFormat0)*numvertices, vertices);

	PointerCaster posCast;
	posCast.m_baseIndex = 0;
	PointerCaster uvCast;
	uvCast.m_baseIndex = 8*sizeof(float);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(GfxVertexFormat0), posCast.m_pointer);
	glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(GfxVertexFormat0), uvCast.m_pointer);
	checkError("glVertexAttribPointer");
	glEnableVertexAttribArray(0);
	glEnableVertexAttribArray(1);

	glVertexAttribDivisor(0,0);
	glVertexAttribDivisor(1,0);
	checkError("glVertexAttribDivisor");

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, triangleIndexVbo);
	int indexBufferSizeInBytes = numIndices*sizeof(int);

	glBufferData(GL_ELEMENT_ARRAY_BUFFER, numIndices*sizeof(int), NULL, GL_DYNAMIC_DRAW);
	glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, 0, numIndices*sizeof(int), indices);

	glDrawElements(GL_TRIANGLES, numIndices, GL_UNSIGNED_INT, 0);

	//glDrawElements(GL_TRIANGLES, numIndices, GL_UNSIGNED_INT,indices);
	checkError("glDrawElements");

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D,0);
	glUseProgram(0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ARRAY_BUFFER,0);
	glBindVertexArray(0);
	checkError("glBindVertexArray");

}


void GLInstancingRenderer::drawPoint(const double* position, const double color[4], double pointDrawSize)
{
	float pos[4]={(float)position[0],(float)position[1],(float)position[2],0};
	float clr[4] = {(float)color[0],(float)color[1],(float)color[2],(float)color[3]};
	drawPoints(pos,clr,1,3*sizeof(float),float(pointDrawSize));
}

void GLInstancingRenderer::drawPoint(const float* positions, const float color[4], float pointDrawSize)
{
	drawPoints(positions,color,1,3*sizeof(float),pointDrawSize);
}

void GLInstancingRenderer::drawPoints(const float* positions, const float color[4], int numPoints, int pointStrideInBytes, float pointDrawSize)
{
	float projectionMatrix[16], viewMatrix[16];
	m_data->m_activeCamera->getCameraProjectionMatrix(projectionMatrix);
	m_data->m_activeCamera->getCameraViewMatrix(viewMatrix);

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D,0);

	b3Assert(glGetError() ==GL_NO_ERROR);
	glUseProgram(linesShader);
	glUniformMatrix4fv(lines_ProjectionMatrix, 1, false, &projectionMatrix[0]);
	glUniformMatrix4fv(lines_ModelViewMatrix, 2, false, &viewMatrix[0]);
	glUniform4f(lines_colour,color[0],color[1],color[2],color[3]);

	glPointSize(pointDrawSize);
	glBindVertexArray(lineVertexArrayObject);

    glBindBuffer(GL_ARRAY_BUFFER, lineVertexBufferObject);

	int maxPointsInBatch = MAX_POINTS_IN_BATCH;
	int remainingPoints = numPoints;
	int offsetNumPoints= 0;
	int curPointsInBatch;
	while( (curPointsInBatch = b3Min(maxPointsInBatch, remainingPoints)) > 0 )
	{
		glBufferSubData(GL_ARRAY_BUFFER, 0, curPointsInBatch*pointStrideInBytes, positions + offsetNumPoints*(pointStrideInBytes / sizeof(float)));
		glEnableVertexAttribArray(0);
		int numFloats = 3;    // pointStrideInBytes / sizeof(float);
		glVertexAttribPointer(0, numFloats, GL_FLOAT, GL_FALSE, pointStrideInBytes, 0);
		glDrawArrays(GL_POINTS, 0, curPointsInBatch);
		remainingPoints -= curPointsInBatch;
		offsetNumPoints += curPointsInBatch;
	}

	glBindVertexArray(0);
	glPointSize(1);
	glUseProgram(0);
}

void GLInstancingRenderer::drawLines(const float* positions, const float color[4], int numPoints, int pointStrideInBytes, const unsigned int* indices, int numIndices, float lineWidthIn)
{
	float projectionMatrix[16], viewMatrix[16];
	m_data->m_activeCamera->getCameraProjectionMatrix(projectionMatrix);
	m_data->m_activeCamera->getCameraViewMatrix(viewMatrix);

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D,0);

	float lineWidth = lineWidthIn;
	b3Clamp(lineWidth,(float)lineWidthRange[0],(float)lineWidthRange[1]);
	glLineWidth(lineWidth);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D,0);

	b3Assert(glGetError() ==GL_NO_ERROR);
	glUseProgram(linesShader);
	glUniformMatrix4fv(lines_ProjectionMatrix, 1, false, &projectionMatrix[0]);
	glUniformMatrix4fv(lines_ModelViewMatrix, 2, false, &viewMatrix[0]);
	glUniform4f(lines_colour,color[0],color[1],color[2],color[3]);

	glBindVertexArray(linesVertexArrayObject);
	b3Assert(glGetError() ==GL_NO_ERROR);

    glBindBuffer(GL_ARRAY_BUFFER, linesVertexBufferObject);
	glBufferData(GL_ARRAY_BUFFER, numPoints*pointStrideInBytes, 0,GL_DYNAMIC_DRAW);
	glBufferSubData(GL_ARRAY_BUFFER, 0, numPoints*pointStrideInBytes, positions);
	b3Assert(glGetError() ==GL_NO_ERROR);

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ARRAY_BUFFER, linesVertexBufferObject);
	glEnableVertexAttribArray(0);
	b3Assert(glGetError() ==GL_NO_ERROR);

	int numFloats = 3;
	glVertexAttribPointer(0, numFloats, GL_FLOAT, GL_FALSE, pointStrideInBytes, 0);
	b3Assert(glGetError() ==GL_NO_ERROR);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, linesIndexVbo);
	int indexBufferSizeInBytes = numIndices*sizeof(int);

	glBufferData(GL_ELEMENT_ARRAY_BUFFER, indexBufferSizeInBytes, NULL, GL_DYNAMIC_DRAW);
	glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, 0, indexBufferSizeInBytes, indices);
	glDrawElements(GL_LINES, numIndices, GL_UNSIGNED_INT, 0);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	b3Assert(glGetError() ==GL_NO_ERROR);
	glBindVertexArray(0);
	b3Assert(glGetError() ==GL_NO_ERROR);
	glPointSize(1);
    b3Assert(glGetError() ==GL_NO_ERROR);
	glUseProgram(0);
}

void GLInstancingRenderer::drawLine(const double fromIn[4], const double toIn[4], const double colorIn[4], double lineWidthIn)
{
	float from[4]={float(fromIn[0]),float(fromIn[1]),float(fromIn[2]),float(fromIn[3])};
	float to[4]={float(toIn[0]),float(toIn[1]),float(toIn[2]),float(toIn[3])};
	float color[4]={float(colorIn[0]),float(colorIn[1]),float(colorIn[2]),float(colorIn[3])};
	float lineWidth=float(lineWidthIn);
	drawLine(from,to,color,lineWidth);
}
void GLInstancingRenderer::drawLine(const float from[4], const float to[4], const float color[4], float lineWidth)
{
	float projectionMatrix[16], viewMatrix[16];
	m_data->m_activeCamera->getCameraProjectionMatrix(projectionMatrix);
	m_data->m_activeCamera->getCameraViewMatrix(viewMatrix);

	b3Assert(glGetError() ==GL_NO_ERROR);

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D,0);
	b3Assert(glGetError() ==GL_NO_ERROR);

	glUseProgram(linesShader);
	b3Assert(glGetError() ==GL_NO_ERROR);

	glUniformMatrix4fv(lines_ProjectionMatrix, 1, false, &projectionMatrix[0]);
	glUniformMatrix4fv(lines_ModelViewMatrix, 2, false, &viewMatrix[0]);
	glUniform4f(lines_colour,color[0],color[1],color[2],color[3]);
	b3Assert(glGetError() ==GL_NO_ERROR);

	const float vertexPositions[] = {
		from[0],from[1],from[2],1,
		to[0],to[1],to[2],1
	};
	int sz = sizeof(vertexPositions);
	b3Assert(glGetError() ==GL_NO_ERROR);


	b3Clamp(lineWidth,(float)lineWidthRange[0],(float)lineWidthRange[1]);
	glLineWidth(lineWidth);
	b3Assert(glGetError() ==GL_NO_ERROR);

	glBindVertexArray(lineVertexArrayObject);
	b3Assert(glGetError() ==GL_NO_ERROR);

    glBindBuffer(GL_ARRAY_BUFFER, lineVertexBufferObject);
	b3Assert(glGetError() ==GL_NO_ERROR);

	glBufferSubData(GL_ARRAY_BUFFER, 0,sz, vertexPositions);
	b3Assert(glGetError() ==GL_NO_ERROR);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ARRAY_BUFFER, lineVertexBufferObject);
	b3Assert(glGetError() ==GL_NO_ERROR);

	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 0, 0);
	b3Assert(glGetError() ==GL_NO_ERROR);

	glDrawArrays(GL_LINES, 0, 2);
	b3Assert(glGetError() ==GL_NO_ERROR);

	glBindVertexArray(0);
	glLineWidth(1);

	b3Assert(glGetError() ==GL_NO_ERROR);
	glUseProgram(0);
}


void GLInstancingRenderer::renderScene()
{
	float projectionMatrix[16], viewMatrix[16];
	m_data->m_activeCamera->getCameraProjectionMatrix(projectionMatrix);
	m_data->m_activeCamera->getCameraViewMatrix(viewMatrix);

	//we need to get the viewport dims, because on Apple Retina the viewport dimension is different from screenWidth
	GLint dims[4];
	glGetIntegerv(GL_VIEWPORT, dims);
	
	B3_PROFILE("GLInstancingRenderer::RenderScene");
	{
		B3_PROFILE("init");
		init();
	}

	glEnable(GL_CULL_FACE);
	glCullFace(GL_BACK);
    b3Assert(glGetError() ==GL_NO_ERROR);

    //	glBindBuffer(GL_ARRAY_BUFFER, 0);
	{
		B3_PROFILE("glFlush2");
		glBindBuffer(GL_ARRAY_BUFFER, m_data->m_vbo);
		//glFlush();
	}
	b3Assert(glGetError() ==GL_NO_ERROR);

	int totalNumInstances = 0;
	for (int i=0;i<m_graphicsInstances.size();i++)
	{
		totalNumInstances+=m_graphicsInstances[i]->m_numGraphicsInstances;
	}

	int curOffset = 0;
	int POSITION_BUFFER_SIZE = (totalNumInstances*sizeof(float)*4);
	int ORIENTATION_BUFFER_SIZE = (totalNumInstances*sizeof(float)*4);
	int COLOR_BUFFER_SIZE = (totalNumInstances*sizeof(float)*4);
//		int SCALE_BUFFER_SIZE = (totalNumInstances*sizeof(float)*3);


	for (int i=0;i<m_graphicsInstances.size();i++)
	{
		b3GraphicsInstance* gfxObj = m_graphicsInstances[i];

		if (gfxObj->m_numGraphicsInstances)
		{
			glActiveTexture(GL_TEXTURE0);
			GLuint curBindTexture = 0;
			if (gfxObj->m_flags & eGfxHasTexture)
			{
				curBindTexture = m_data->m_textureHandles[gfxObj->m_textureIndex].m_glTexture;

				glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_BASE_LEVEL,0);

				if (m_data->m_textureHandles[gfxObj->m_textureIndex].m_enableFiltering)
				{
					glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
					glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
				} else
				{
					glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
					glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
				}
			}
			else
			{
				curBindTexture = m_data->m_defaultTexturehandle;
			}

			glBindTexture(GL_TEXTURE_2D,curBindTexture);
			b3Assert(glGetError() ==GL_NO_ERROR);

			glBindVertexArray(gfxObj->m_cube_vao);

			int vertexStride = 9*sizeof(float);
			PointerCaster vertex;
			vertex.m_baseIndex = gfxObj->m_vertexArrayOffset*vertexStride;


			glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 9*sizeof(float), vertex.m_pointer);
			glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 0, (GLvoid *)(curOffset*4*sizeof(float)+m_data->m_maxShapeCapacityInBytes));
			glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, 0, (GLvoid *)(curOffset*4*sizeof(float)+m_data->m_maxShapeCapacityInBytes+POSITION_BUFFER_SIZE));

			PointerCaster uv;
			uv.m_baseIndex = 7*sizeof(float)+vertex.m_baseIndex;

			PointerCaster normal;
			normal.m_baseIndex = 4*sizeof(float)+vertex.m_baseIndex;

			glVertexAttribPointer(3, 2, GL_FLOAT, GL_FALSE, 9*sizeof(float), uv.m_pointer);
			glVertexAttribPointer(4, 3, GL_FLOAT, GL_FALSE, 9*sizeof(float), normal.m_pointer);
			glVertexAttribPointer(5, 4, GL_FLOAT, GL_FALSE, 0, (GLvoid *)(curOffset*4*sizeof(float)+m_data->m_maxShapeCapacityInBytes+POSITION_BUFFER_SIZE+ORIENTATION_BUFFER_SIZE));
			glVertexAttribPointer(6, 3, GL_FLOAT, GL_FALSE, 0, (GLvoid *)(curOffset*3*sizeof(float)+m_data->m_maxShapeCapacityInBytes+POSITION_BUFFER_SIZE+ORIENTATION_BUFFER_SIZE+COLOR_BUFFER_SIZE));

			glEnableVertexAttribArray(0);
			glEnableVertexAttribArray(1);
			glEnableVertexAttribArray(2);
			glEnableVertexAttribArray(3);
			glEnableVertexAttribArray(4);
			glEnableVertexAttribArray(5);
			glEnableVertexAttribArray(6);
			glVertexAttribDivisor(0, 0);
			glVertexAttribDivisor(1, 1);
			glVertexAttribDivisor(2, 1);
			glVertexAttribDivisor(3, 0);
			glVertexAttribDivisor(4, 0);
			glVertexAttribDivisor(5, 1);
			glVertexAttribDivisor(6, 1);

			int indexCount = gfxObj->m_numIndices;
			GLvoid* indexOffset = 0;

			glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, gfxObj->m_index_vbo);
			{
				B3_PROFILE("glDrawElementsInstanced");

				if (gfxObj->m_primitiveType==B3_GL_POINTS)
				{
					glUseProgram(instancingShaderPointSprite);
					glUniformMatrix4fv(ProjectionMatrixPointSprite, 1, false, &projectionMatrix[0]);
					glUniformMatrix4fv(ModelViewMatrixPointSprite, 1, false, &viewMatrix[0]);
					glUniform1f(screenWidthPointSprite,float(m_screenWidth));
					b3Assert(glGetError() ==GL_NO_ERROR);
					glPointSize(20);
#ifndef __APPLE__
					glEnable(GL_POINT_SPRITE_ARB);
#endif
					glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);
					glDrawElementsInstanced(GL_POINTS, indexCount, GL_UNSIGNED_INT, indexOffset, gfxObj->m_numGraphicsInstances);
				} else
				{
					glUseProgram(instancingShader);
					glUniformMatrix4fv(ProjectionMatrix, 1, false, &projectionMatrix[0]);
					glUniformMatrix4fv(ModelViewMatrix, 1, false, &viewMatrix[0]);

					b3Vector3 gLightDir = m_data->m_lightPos;
					gLightDir.normalize();
					glUniform3f(regularLightDirIn,gLightDir[0],gLightDir[1],gLightDir[2]);
					glUniform1i(uniform_texture_diffuse, 0);
					glDrawElementsInstanced(GL_TRIANGLES, indexCount, GL_UNSIGNED_INT, indexOffset, gfxObj->m_numGraphicsInstances);
				}
			}

			curOffset += gfxObj->m_numGraphicsInstances;
		}
	}

	{
		B3_PROFILE("glFlush");
		glFlush();
	}

    b3Assert(glGetError() ==GL_NO_ERROR);
	{
		B3_PROFILE("glUseProgram(0);");
		glUseProgram(0);
		glBindBuffer(GL_ARRAY_BUFFER,0);
		glBindVertexArray(0);
	}

	glDisable(GL_CULL_FACE);
	b3Assert(glGetError() ==GL_NO_ERROR);
}


void GLInstancingRenderer::CleanupShaders()
{
}

void GLInstancingRenderer::setPlaneReflectionShapeIndex(int index)
{
	m_planeReflectionShapeIndex = index;
}

void GLInstancingRenderer::clearZBuffer()
{
	glClear(GL_DEPTH_BUFFER_BIT);
}

int GLInstancingRenderer::getMaxShapeCapacity() const
{
	return m_data->m_maxShapeCapacityInBytes;
}
int GLInstancingRenderer::getInstanceCapacity() const
{
	return m_data->m_maxNumObjectCapacity;
}

void GLInstancingRenderer::setRenderFrameBuffer(unsigned int renderFrameBuffer)
{
	m_data->m_renderFrameBuffer = (GLuint) renderFrameBuffer;
}

int GLInstancingRenderer::getTotalNumInstances() const
{
    return m_data->m_totalNumInstances;
}


#endif //NO_OPENGL3
