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


///todo: make this configurable in the gui
bool useShadowMap=true;//false;//true;
int shadowMapWidth=8192;
int shadowMapHeight=8192;
float shadowMapWorldSize=100;
float WHEEL_MULTIPLIER=0.01f;
float MOUSE_MOVE_MULTIPLIER = 0.4f;
#define MAX_POINTS_IN_BATCH 1024
#define MAX_LINES_IN_BATCH 1024


#include "OpenGLInclude.h"
#include "b3gWindowInterface.h"
#include "Bullet3Common/b3MinMax.h"

#ifndef __APPLE__
#ifndef glVertexAttribDivisor
#define glVertexAttribDivisor glVertexAttribDivisorARB
#endif //glVertexAttribDivisor
#ifndef GL_COMPARE_REF_TO_TEXTURE
#define GL_COMPARE_REF_TO_TEXTURE GL_COMPARE_R_TO_TEXTURE
#endif //GL_COMPARE_REF_TO_TEXTURE
#ifndef glDrawElementsInstanced
#define glDrawElementsInstanced glDrawElementsInstancedARB
#endif
#endif //__APPLE__
#include "GLInstancingRenderer.h"

#include <string.h>
//#include "DemoSettings.h"
#include <stdio.h>

#include "Bullet3Common/b3Vector3.h"
#include "Bullet3Common/b3Quaternion.h"
#include "Bullet3Common/b3Matrix3x3.h"
#include "LoadShader.h"

#include "GLInstanceRendererInternalData.h"

//GLSL shader strings, embedded using build3/stringify
#include "OpenGLWindow/Shaders/pointSpriteVS.h"
#include "OpenGLWindow/Shaders/pointSpritePS.h"
#include "OpenGLWindow/Shaders/instancingVS.h"
#include "OpenGLWindow/Shaders/instancingPS.h"
#include "OpenGLWindow/Shaders/createShadowMapInstancingVS.h"
#include "OpenGLWindow/Shaders/createShadowMapInstancingPS.h"
#include "OpenGLWindow/Shaders/useShadowMapInstancingVS.h"
#include "OpenGLWindow/Shaders/useShadowMapInstancingPS.h"
#include "OpenGLWindow/Shaders/linesPS.h"
#include "OpenGLWindow/Shaders/linesVS.h"

#include "OpenGLWindow/GLRenderToTexture.h"




//#include "../../opencl/gpu_rigidbody_pipeline/b3GpuNarrowphaseAndSolver.h"//for m_maxNumObjectCapacity

static InternalDataRenderer* sData2;

GLint lineWidthRange[2]={1,1};
static b3Vector3 gLightPos=b3MakeVector3(-5,200,-40);

struct b3GraphicsInstance
{
	GLuint               m_cube_vao;
	GLuint               m_index_vbo;
	GLuint				m_texturehandle;

	int m_numIndices;
	int m_numVertices;

	int m_numGraphicsInstances;

	int m_instanceOffset;
	int m_vertexArrayOffset;
	int	m_primitiveType;

	b3GraphicsInstance()
	:m_cube_vao(-1),
		m_index_vbo(-1),
		m_texturehandle(0),
		m_numIndices(-1),
		m_numVertices(-1),
		m_numGraphicsInstances(0),
		m_instanceOffset(0),
		m_vertexArrayOffset(0),
		m_primitiveType(B3_GL_TRIANGLES)
	{
	}

};




bool m_ortho = false;

static GLfloat projectionMatrix[16];
static GLfloat modelviewMatrix[16];

//static GLfloat depthLightModelviewMatrix[16];

static void checkError(const char *functionName)
{
    GLenum error;
    while (( error = glGetError() ) != GL_NO_ERROR)
    {
        fprintf (stderr, "GL error 0x%X detected in %s\n", error, functionName);
    }
}



extern int gShapeIndex;










struct InternalDataRenderer : public GLInstanceRendererInternalData
{


	b3Vector3 m_cameraPosition;
	b3Vector3 m_cameraTargetPosition;
	float m_cameraDistance;
	b3Vector3 m_cameraUp;
	float m_azi;
	float m_ele;

	float m_mouseXpos;
	float m_mouseYpos;
	bool m_mouseInitialized;
	int m_leftMouseButton;
	int m_middleMouseButton;
	int m_rightMouseButton;




	GLuint				m_defaultTexturehandle;
	b3AlignedObjectArray<GLuint>	m_textureHandles;

	GLRenderToTexture*	m_shadowMap;
	GLuint				m_shadowTexture;
	int	m_altPressed;
	int	m_controlPressed;

	InternalDataRenderer() :
		m_cameraPosition(b3MakeVector3(0,0,0)),
		m_cameraTargetPosition(b3MakeVector3(15,2,-24)),
		m_cameraDistance(150),
		m_cameraUp(b3MakeVector3(0,1,0)),
		m_azi(100.f),//135.f),
		//m_ele(25.f),
		m_ele(25.f),
		m_mouseInitialized(false),
		m_leftMouseButton(0),
		m_middleMouseButton(0),
		m_rightMouseButton(0),
		m_shadowMap(0),
		m_shadowTexture(0),
		m_altPressed(0),
		m_controlPressed(0)
	{


	}

	void wheelCallback( float deltax, float deltay)
    {
		if (!m_leftMouseButton)
		{

			if (deltay<0 || m_cameraDistance>1)
			{
				m_cameraDistance -= deltay*0.1f;
				if (m_cameraDistance<1)
					m_cameraDistance=1;
			} else
			{
				b3Vector3 fwd = m_cameraTargetPosition-m_cameraPosition;
				fwd.normalize();
				m_cameraTargetPosition += fwd*deltay*WHEEL_MULTIPLIER;
			}
        } else
		{
			if (b3Fabs(deltax)>b3Fabs(deltay))
			{
				b3Vector3 fwd = m_cameraTargetPosition-m_cameraPosition;
				b3Vector3 side = m_cameraUp.cross(fwd);
				side.normalize();
				m_cameraTargetPosition += side * deltax*WHEEL_MULTIPLIER;

			} else
			{
				m_cameraTargetPosition -= m_cameraUp * deltay*WHEEL_MULTIPLIER;

			}
		}
	}

	void mouseMoveCallback(float x, float y)
	{
		if (m_altPressed || m_controlPressed)
		{
			float xDelta = x-m_mouseXpos;
			float yDelta = y-m_mouseYpos;

			if (m_leftMouseButton)
			{
	//			if (b3Fabs(xDelta)>b3Fabs(yDelta))
	//			{
					m_azi -= xDelta*MOUSE_MOVE_MULTIPLIER;
	//			} else
	//			{
					m_ele += yDelta*MOUSE_MOVE_MULTIPLIER;
	//			}
			}
			if (m_middleMouseButton)
			{
				m_cameraTargetPosition += m_cameraUp * yDelta*0.01;


				b3Vector3 fwd = m_cameraTargetPosition-m_cameraPosition;
				b3Vector3 side = m_cameraUp.cross(fwd);
				side.normalize();
				m_cameraTargetPosition += side * xDelta*0.01;

			}
			if (m_rightMouseButton)
			{
					m_cameraDistance -= xDelta*0.01f;
					m_cameraDistance -= yDelta*0.01f;
					if (m_cameraDistance<1)
						m_cameraDistance=1;
					if (m_cameraDistance>1000)
						m_cameraDistance=1000;
			}
		}

		//printf("m_azi/pitch = %f\n", m_azi);
//		printf("m_ele/yaw = %f\n", m_ele);

		m_mouseXpos = x;
		m_mouseYpos = y;
		m_mouseInitialized = true;
	}

	void mouseButtonCallback(int button, int state, float x, float y)
	{

		if (button==0)
			m_leftMouseButton=state;
		if (button==1)
			m_middleMouseButton=state;

		if (button==2)
			m_rightMouseButton=state;

		m_mouseXpos = x;
		m_mouseYpos = y;
		m_mouseInitialized = true;
	}
	void keyboardCallback(int key, int state)
	{
		if (key==B3G_CONTROL)
		{
			m_controlPressed=state;
		}
		if (key==B3G_ALT)
		{
			m_altPressed = state;
		}
	}

};

struct	GLInstanceRendererInternalData* GLInstancingRenderer::getInternalData()
{
	return m_data;
}

void b3DefaultWheelCallback(float deltax, float deltay)
{
	if (sData2)
		sData2->wheelCallback(deltax,deltay);
}
void b3DefaultMouseButtonCallback(int button, int state, float x, float y)
{
	if (sData2)
		sData2->mouseButtonCallback(button, state, x, y);
}
void b3DefaultMouseMoveCallback( float x, float y)
{
	if (sData2)
		sData2->mouseMoveCallback( x, y);
}

void b3DefaultKeyboardCallback(int key, int state)
{
	if (sData2)
		sData2->keyboardCallback(key,state);
}


static GLuint               linesShader;        // The line renderer
static GLuint               useShadowMapInstancingShader;        // The shadow instancing renderer
static GLuint               createShadowMapInstancingShader;        // The shadow instancing renderer
static GLuint               instancingShader;        // The instancing renderer
static GLuint               instancingShaderPointSprite;        // The point sprite instancing renderer




//static bool                 done = false;

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


static GLint	useShadow_ModelViewMatrix=0;
static GLint	useShadow_MVP=0;
static GLint  useShadow_lightDirIn=0;

static GLint	useShadow_ProjectionMatrix=0;
static GLint	useShadow_DepthBiasModelViewMatrix=0;
static GLint    useShadow_uniform_texture_diffuse = 0;
static GLint	useShadow_shadowMap = 0;

static GLint	createShadow_depthMVP=0;

static GLint	ModelViewMatrix=0;
static GLint	ProjectionMatrix=0;

static GLint                uniform_texture_diffuse = 0;

static GLint	screenWidthPointSprite=0;
static GLint	ModelViewMatrixPointSprite=0;
static GLint	ProjectionMatrixPointSprite=0;
//static GLint	uniform_texture_diffusePointSprite= 0;



GLInstancingRenderer::GLInstancingRenderer(int maxNumObjectCapacity, int maxShapeCapacityInBytes)
	:m_maxNumObjectCapacity(maxNumObjectCapacity),
	m_maxShapeCapacityInBytes(maxShapeCapacityInBytes),
	m_textureenabled(true),
	m_textureinitialized(false),
	m_screenWidth(0),
	m_screenHeight(0),
	m_upAxis(1)
{

	m_data = new InternalDataRenderer;
	m_data->m_totalNumInstances = 0;

	sData2 = m_data;

	m_data->m_instance_positions_ptr.resize(m_maxNumObjectCapacity*4);
	m_data->m_instance_quaternion_ptr.resize(m_maxNumObjectCapacity*4);
	m_data->m_instance_colors_ptr.resize(m_maxNumObjectCapacity*4);
	m_data->m_instance_scale_ptr.resize(m_maxNumObjectCapacity*3);

}

void GLInstancingRenderer::removeAllInstances()
{
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
}

GLInstancingRenderer::~GLInstancingRenderer()
{
	delete m_data->m_shadowMap;
	glDeleteTextures(1,&m_data->m_shadowTexture);
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









void GLInstancingRenderer::writeSingleInstanceTransformToCPU(const float* position, const float* orientation, int srcIndex)
{
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

/*	m_data->m_instance_colors_ptr[srcIndex*4+0]=color[0];
	m_data->m_instance_colors_ptr[srcIndex*4+1]=color[1];
	m_data->m_instance_colors_ptr[srcIndex*4+2]=color[2];
	m_data->m_instance_colors_ptr[srcIndex*4+3]=color[3];
	*/
}


void GLInstancingRenderer::writeSingleInstanceColorToCPU(float* color, int srcIndex)
{

	m_data->m_instance_colors_ptr[srcIndex*4+0]=color[0];
	m_data->m_instance_colors_ptr[srcIndex*4+1]=color[1];
	m_data->m_instance_colors_ptr[srcIndex*4+2]=color[2];
	m_data->m_instance_colors_ptr[srcIndex*4+3]=color[3];
}



void GLInstancingRenderer::writeSingleInstanceTransformToGPU(float* position, float* orientation, int objectIndex)
{
	glBindBuffer(GL_ARRAY_BUFFER, m_data->m_vbo);
	glFlush();

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

	float* positions = (float*)(base+m_maxShapeCapacityInBytes);
	float* orientations = (float*)(base+m_maxShapeCapacityInBytes + POSITION_BUFFER_SIZE);


	positions[objectIndex*4] = position[0];
	positions[objectIndex*4+1] = position[1];
	positions[objectIndex*4+2] = position[2];
	positions[objectIndex*4+3] = position[3];

	orientations [objectIndex*4] = orientation[0];
	orientations [objectIndex*4+1] = orientation[1];
	orientations [objectIndex*4+2] = orientation[2];
	orientations [objectIndex*4+3] = orientation[3];

	glUnmapBuffer( GL_ARRAY_BUFFER);
	glFlush();
}


void GLInstancingRenderer::writeTransforms()
{

	b3Assert(glGetError() ==GL_NO_ERROR);


	glBindBuffer(GL_ARRAY_BUFFER, m_data->m_vbo);
	glFlush();

	b3Assert(glGetError() ==GL_NO_ERROR);


	char* orgBase =  (char*)glMapBuffer( GL_ARRAY_BUFFER,GL_READ_WRITE);
	if (orgBase)
	{


		int totalNumInstances= 0;

		for (int k=0;k<m_graphicsInstances.size();k++)
		{
			b3GraphicsInstance* gfxObj = m_graphicsInstances[k];
			totalNumInstances+=gfxObj->m_numGraphicsInstances;
		}

		m_data->m_totalNumInstances = totalNumInstances;

		for (int k=0;k<m_graphicsInstances.size();k++)
		{
			//int k=0;
			b3GraphicsInstance* gfxObj = m_graphicsInstances[k];



			int POSITION_BUFFER_SIZE = (totalNumInstances*sizeof(float)*4);
			int ORIENTATION_BUFFER_SIZE = (totalNumInstances*sizeof(float)*4);
			int COLOR_BUFFER_SIZE = (totalNumInstances*sizeof(float)*4);
		//	int SCALE_BUFFER_SIZE = (totalNumInstances*sizeof(float)*3);

			char* base = orgBase;

			float* positions = (float*)(base+m_maxShapeCapacityInBytes);
			float* orientations = (float*)(base+m_maxShapeCapacityInBytes + POSITION_BUFFER_SIZE);
			float* colors= (float*)(base+m_maxShapeCapacityInBytes + POSITION_BUFFER_SIZE+ORIENTATION_BUFFER_SIZE);
			float* scaling= (float*)(base+m_maxShapeCapacityInBytes + POSITION_BUFFER_SIZE+ORIENTATION_BUFFER_SIZE+COLOR_BUFFER_SIZE);

			//static int offset=0;
			//offset++;


			for (int i=0;i<gfxObj->m_numGraphicsInstances;i++)
			{

				int srcIndex=i+gfxObj->m_instanceOffset;

				positions[srcIndex*4] = m_data->m_instance_positions_ptr[srcIndex*4];
				positions[srcIndex*4+1] = m_data->m_instance_positions_ptr[srcIndex*4+1];
				positions[srcIndex*4+2] = m_data->m_instance_positions_ptr[srcIndex*4+2];
				positions[srcIndex*4+3] = m_data->m_instance_positions_ptr[srcIndex*4+3];

				orientations[srcIndex*4]=m_data->m_instance_quaternion_ptr[srcIndex*4];
				orientations[srcIndex*4+1]=m_data->m_instance_quaternion_ptr[srcIndex*4+1];
				orientations[srcIndex*4+2]=m_data->m_instance_quaternion_ptr[srcIndex*4+2];
				orientations[srcIndex*4+3]=m_data->m_instance_quaternion_ptr[srcIndex*4+3];

				colors[srcIndex*4]=m_data->m_instance_colors_ptr[srcIndex*4];
				colors[srcIndex*4+1]=m_data->m_instance_colors_ptr[srcIndex*4+1];
				colors[srcIndex*4+2]=m_data->m_instance_colors_ptr[srcIndex*4+2];
				colors[srcIndex*4+3]=m_data->m_instance_colors_ptr[srcIndex*4+3];

				scaling[srcIndex*3]=m_data->m_instance_scale_ptr[srcIndex*3];
				scaling[srcIndex*3+1]=m_data->m_instance_scale_ptr[srcIndex*3+1];
				scaling[srcIndex*3+2]=m_data->m_instance_scale_ptr[srcIndex*3+2];

			}
		}
	} else
	{
		b3Error("ERROR glMapBuffer failed\n");
	}
	b3Assert(glGetError() ==GL_NO_ERROR);

	glUnmapBuffer( GL_ARRAY_BUFFER);
	//if this glFinish is removed, the animation is not always working/blocks
	//@todo: figure out why
	glFlush();
	glBindBuffer(GL_ARRAY_BUFFER, 0);//m_data->m_vbo);

    b3Assert(glGetError() ==GL_NO_ERROR);

}

int GLInstancingRenderer::registerGraphicsInstance(int shapeIndex, const double* pos1, const double* orn1, const double* color1, const double* scaling1)
{
    float pos[4] = {(float)pos1[0],(float)pos1[1],(float)pos1[2],(float)pos1[3]};
    float orn[4] = {(float)orn1[0],(float)orn1[1],(float)orn1[2],(float)orn1[3]};
    float color[4] = {(float)color1[0],(float)color1[1],(float)color1[2],(float)color1[3]};
    float scaling[4] = {(float)scaling1[0],(float)scaling1[1],(float)scaling1[2],(float)scaling1[3]};
    return registerGraphicsInstance(shapeIndex,pos,orn,color,scaling);
}


int GLInstancingRenderer::registerGraphicsInstance(int shapeIndex, const float* position, const float* quaternion, const float* color, const float* scaling)
{
	b3Assert(shapeIndex == (m_graphicsInstances.size()-1));
	b3Assert(m_graphicsInstances.size()<m_maxNumObjectCapacity-1);

	b3GraphicsInstance* gfxObj = m_graphicsInstances[shapeIndex];

	int index = gfxObj->m_numGraphicsInstances + gfxObj->m_instanceOffset;

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
	} else
	{
		b3Error("registerGraphicsInstance out of range, %d\n", maxElements);
		return -1;
	}
	return index;//gfxObj->m_numGraphicsInstances;
}


int	GLInstancingRenderer::registerTexture(const unsigned char* texels, int width, int height)
{
	b3Assert(glGetError() ==GL_NO_ERROR);

	int textureIndex = m_data->m_textureHandles.size();
	const GLubyte*	image= (const GLubyte*)texels;
	GLuint textureHandle;
	glGenTextures(1,(GLuint*)&textureHandle);
	glBindTexture(GL_TEXTURE_2D,textureHandle);

	b3Assert(glGetError() ==GL_NO_ERROR);

	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width,height,0,GL_RGB,GL_UNSIGNED_BYTE,image);


	b3Assert(glGetError() ==GL_NO_ERROR);

	glGenerateMipmap(GL_TEXTURE_2D);
	b3Assert(glGetError() ==GL_NO_ERROR);


	m_data->m_textureHandles.push_back(textureHandle);
	return textureIndex;
}

void GLInstancingRenderer::updateShape(int shapeIndex, const float* vertices)
{
	b3GraphicsInstance* gfxObj = m_graphicsInstances[shapeIndex];
	int numvertices = gfxObj->m_numVertices;

	glBindBuffer(GL_ARRAY_BUFFER, m_data->m_vbo);
	char* dest=  (char*)glMapBuffer( GL_ARRAY_BUFFER,GL_WRITE_ONLY);//GL_WRITE_ONLY
	int vertexStrideInBytes = 9*sizeof(float);
	int sz = numvertices*vertexStrideInBytes;
	memcpy(dest+vertexStrideInBytes*gfxObj->m_vertexArrayOffset,vertices,sz);
	glUnmapBuffer( GL_ARRAY_BUFFER);
}

int GLInstancingRenderer::registerShape(const float* vertices, int numvertices, const int* indices, int numIndices,int primitiveType, int textureId)
{
	b3GraphicsInstance* gfxObj = new b3GraphicsInstance;

	if (textureId>=0)
	{
		gfxObj->m_texturehandle = m_data->m_textureHandles[textureId];
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


	glBindBuffer(GL_ARRAY_BUFFER, m_data->m_vbo);
	char* dest=  (char*)glMapBuffer( GL_ARRAY_BUFFER,GL_WRITE_ONLY);//GL_WRITE_ONLY
	int vertexStrideInBytes = 9*sizeof(float);
	int sz = numvertices*vertexStrideInBytes;
#ifdef B3_DEBUG
	int totalUsed = vertexStrideInBytes*gfxObj->m_vertexArrayOffset+sz;
	b3Assert(totalUsed<this->m_maxShapeCapacityInBytes);
#endif//B3_DEBUG

	memcpy(dest+vertexStrideInBytes*gfxObj->m_vertexArrayOffset,vertices,sz);
	glUnmapBuffer( GL_ARRAY_BUFFER);

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

	int POSITION_BUFFER_SIZE = (m_maxNumObjectCapacity*sizeof(float)*4);
	int ORIENTATION_BUFFER_SIZE = (m_maxNumObjectCapacity*sizeof(float)*4);
	int COLOR_BUFFER_SIZE = (m_maxNumObjectCapacity*sizeof(float)*4);
	int SCALE_BUFFER_SIZE = (m_maxNumObjectCapacity*sizeof(float)*3);

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

	//glGetIntegerv(GL_ALIASED_LINE_WIDTH_RANGE, range);
	glGetIntegerv(GL_SMOOTH_LINE_WIDTH_RANGE, lineWidthRange);




	useShadowMapInstancingShader = gltLoadShaderPair(useShadowMapInstancingVertexShader,useShadowMapInstancingFragmentShader);

	glLinkProgram(useShadowMapInstancingShader);
	glUseProgram(useShadowMapInstancingShader);
	useShadow_ModelViewMatrix = glGetUniformLocation(useShadowMapInstancingShader, "ModelViewMatrix");
	useShadow_MVP = 		glGetUniformLocation(useShadowMapInstancingShader, "MVP");
	useShadow_ProjectionMatrix = glGetUniformLocation(useShadowMapInstancingShader, "ProjectionMatrix");
	useShadow_DepthBiasModelViewMatrix = glGetUniformLocation(useShadowMapInstancingShader, "DepthBiasModelViewProjectionMatrix");
	useShadow_uniform_texture_diffuse = glGetUniformLocation(useShadowMapInstancingShader, "Diffuse");
	useShadow_shadowMap = glGetUniformLocation(useShadowMapInstancingShader,"shadowMap");
	useShadow_lightDirIn = glGetUniformLocation(useShadowMapInstancingShader,"lightDirIn");

	createShadowMapInstancingShader = gltLoadShaderPair(createShadowMapInstancingVertexShader,createShadowMapInstancingFragmentShader);
	glLinkProgram(createShadowMapInstancingShader);
	glUseProgram(createShadowMapInstancingShader);
	createShadow_depthMVP = glGetUniformLocation(createShadowMapInstancingShader, "depthMVP");

	glUseProgram(0);

	instancingShader = gltLoadShaderPair(instancingVertexShader,instancingFragmentShader);
	glLinkProgram(instancingShader);
	glUseProgram(instancingShader);
	ModelViewMatrix = glGetUniformLocation(instancingShader, "ModelViewMatrix");
	ProjectionMatrix = glGetUniformLocation(instancingShader, "ProjectionMatrix");
	uniform_texture_diffuse = glGetUniformLocation(instancingShader, "Diffuse");
	glUseProgram(0);

		instancingShaderPointSprite = gltLoadShaderPair(pointSpriteVertexShader,pointSpriteFragmentShader);
	glUseProgram(instancingShaderPointSprite);
	ModelViewMatrixPointSprite = glGetUniformLocation(instancingShaderPointSprite, "ModelViewMatrix");
	ProjectionMatrixPointSprite = glGetUniformLocation(instancingShaderPointSprite, "ProjectionMatrix");
	screenWidthPointSprite = glGetUniformLocation(instancingShaderPointSprite, "screenWidth");

	glUseProgram(0);

	//GLuint offset = 0;

	glGenBuffers(1, &m_data->m_vbo);
    checkError("glGenBuffers");

	glBindBuffer(GL_ARRAY_BUFFER, m_data->m_vbo);


	int size = m_maxShapeCapacityInBytes  + POSITION_BUFFER_SIZE+ORIENTATION_BUFFER_SIZE+COLOR_BUFFER_SIZE+SCALE_BUFFER_SIZE;
	m_data->m_vboSize = size;

	glBufferData(GL_ARRAY_BUFFER, size, 0, GL_DYNAMIC_DRAW);//GL_STATIC_DRAW);

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

//	glClearColor(float(0.),float(0.),float(0.4),float(0));

    b3Assert(glGetError() ==GL_NO_ERROR);


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
//					const int	t=y>>5;
					GLubyte*	pi=image+y*256*3;
					for(int x=0;x<256;++x)
					{
						if (x<2||y<2||x>253||y>253)
						{
							pi[0]=0;
							pi[1]=0;
							pi[2]=0;
						} else
						{
							pi[0]=255;
							pi[1]=255;
							pi[2]=255;
						}

						/*
						const int		s=x>>5;
						const GLubyte	b=180;
						GLubyte			c=b+((s+t&1)&1)*(255-b);
						pi[0]=c;
						pi[1]=c;
						pi[2]=c;
						*/

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
	//glEnable(GL_COLOR_MATERIAL);

	b3Assert(glGetError() ==GL_NO_ERROR);

	//	  glEnable(GL_CULL_FACE);
	//	  glCullFace(GL_BACK);
}


void    b3CreateFrustum(
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


void b3Matrix4x4Mul(GLfloat aIn[4][4], GLfloat bIn[4][4], GLfloat result[4][4])
{
	for (int j=0;j<4;j++)
		for (int i=0;i<4;i++)
			result[j][i] = aIn[0][i] * bIn[j][0] + aIn[1][i] * bIn[j][1] + aIn[2][i] * bIn[j][2] + aIn[3][i] * bIn[j][3];
}

void b3Matrix4x4Mul16(GLfloat aIn[16], GLfloat bIn[16], GLfloat result[16])
{
	for (int j=0;j<4;j++)
		for (int i=0;i<4;i++)
			result[j*4+i] = aIn[0*4+i] * bIn[j*4+0] + aIn[1*4+i] * bIn[j*4+1] + aIn[2*4+i] * bIn[j*4+2] + aIn[3*4+i] * bIn[j*4+3];
}


void b3CreateDiagonalMatrix(GLfloat value, GLfloat result[4][4])
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

void b3CreateOrtho(GLfloat left, GLfloat right, GLfloat bottom, GLfloat top, GLfloat zNear, GLfloat zFar, GLfloat result[4][4])
{
	b3CreateDiagonalMatrix(1.f,result);

	result[0][0] = 2.f / (right - left);
	result[1][1] = 2.f / (top - bottom);
	result[2][2] = - 2.f / (zFar - zNear);
	result[3][0] = - (right + left) / (right - left);
	result[3][1] = - (top + bottom) / (top - bottom);
	result[3][2] = - (zFar + zNear) / (zFar - zNear);
}

void    b3CreateLookAt(const b3Vector3& eye, const b3Vector3& center,const b3Vector3& up, GLfloat result[16])
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


void	GLInstancingRenderer::resize(int width, int height)
{
	m_screenWidth = width;
	m_screenHeight = height;
}

void GLInstancingRenderer::updateCamera(int upAxis)
{

    b3Assert(glGetError() ==GL_NO_ERROR);

	m_upAxis = upAxis;
	int m_forwardAxis(-1);
	switch (upAxis)
	{
    case 1:
    	m_forwardAxis = 2;
    	m_data->m_cameraUp = b3MakeVector3(0,1,0);
    	gLightPos = b3MakeVector3(-50.f,100,30);
    	break;
    case 2:
			m_forwardAxis = 1;
			m_data->m_cameraUp = b3MakeVector3(0,0,1);
			gLightPos = b3MakeVector3(-50.f,30,100);
			break;
    default:
    b3Assert(0);
	};


    float m_frustumZNear=1;
    float m_frustumZFar=10000.f;


//    m_azi=m_azi+0.01;
	b3Scalar rele = m_data->m_ele * b3Scalar(0.01745329251994329547);// rads per deg
	b3Scalar razi = m_data->m_azi * b3Scalar(0.01745329251994329547);// rads per deg


	b3Quaternion rot(m_data->m_cameraUp,razi);

  b3Vector3 eyePos = b3MakeVector3(0,0,0);
	eyePos[m_forwardAxis] = -m_data->m_cameraDistance;

	b3Vector3 forward = b3MakeVector3(eyePos[0],eyePos[1],eyePos[2]);
	if (forward.length2() < B3_EPSILON)
	{
		forward.setValue(1.f,0.f,0.f);
	}
	b3Vector3 right = m_data->m_cameraUp.cross(forward);
	b3Quaternion roll(right,-rele);

	eyePos = b3Matrix3x3(rot) * b3Matrix3x3(roll) * eyePos;

	m_data->m_cameraPosition[0] = eyePos.x;
	m_data->m_cameraPosition[1] = eyePos.y;
	m_data->m_cameraPosition[2] = eyePos.z;
	m_data->m_cameraPosition += m_data->m_cameraTargetPosition;

	if (m_screenWidth == 0 && m_screenHeight == 0)
		return;

	b3Scalar aspect;
	b3Vector3 extents;

	aspect = m_screenWidth / (b3Scalar)m_screenHeight;
	extents.setValue(aspect * 1.0f, 1.0f,0);


	if (m_screenWidth > m_screenHeight)
	{
		b3CreateFrustum(-aspect * m_frustumZNear, aspect * m_frustumZNear, -m_frustumZNear, m_frustumZNear, m_frustumZNear, m_frustumZFar,projectionMatrix);
	} else
	{
		b3CreateFrustum(-aspect * m_frustumZNear, aspect * m_frustumZNear, -m_frustumZNear, m_frustumZNear, m_frustumZNear, m_frustumZFar,projectionMatrix);
	}

	b3CreateLookAt(m_data->m_cameraPosition,m_data->m_cameraTargetPosition,m_data->m_cameraUp,modelviewMatrix);


}


void	GLInstancingRenderer::getCameraPosition(float cameraPos[4])
{
	cameraPos[0] = m_data->m_cameraPosition[0];
	cameraPos[1] = m_data->m_cameraPosition[1];
	cameraPos[2] = m_data->m_cameraPosition[2];
	cameraPos[3] = 1.f;
}

void	GLInstancingRenderer::setCameraDistance(float dist)
{
	m_data->m_cameraDistance = dist;
}

void	GLInstancingRenderer::setCameraYaw(float yaw)
{
	m_data->m_ele = yaw;
}
void	GLInstancingRenderer::setCameraPitch(float pitch)
{
	m_data->m_azi = pitch;
}

float	GLInstancingRenderer::getCameraYaw() const
{
	return m_data->m_ele;
}
float	GLInstancingRenderer::getCameraPitch() const
{
	return m_data->m_azi;
}

void	GLInstancingRenderer::setCameraTargetPosition(float cameraPos[4])
{
		m_data->m_cameraTargetPosition = b3MakeVector3(cameraPos[0],cameraPos[1],cameraPos[2]);
}

void	GLInstancingRenderer::getCameraTargetPosition(float cameraPos[4]) const
{
	cameraPos[0] = m_data->m_cameraTargetPosition.x;
	cameraPos[1] = m_data->m_cameraTargetPosition.y;
	cameraPos[2] = m_data->m_cameraTargetPosition.z;
}


float	GLInstancingRenderer::getCameraDistance() const
{
	return m_data->m_cameraDistance;
}


void GLInstancingRenderer::getMouseDirection(float* dir, int x, int y)
{
	float top = 1.f;
	float bottom = -1.f;
	float nearPlane = 1.f;
	float tanFov = (top-bottom)*0.5f / nearPlane;
	float fov = b3Scalar(2.0) * b3Atan(tanFov);

	b3Vector3	rayFrom = m_data->m_cameraPosition;
	b3Vector3 rayForward = (m_data->m_cameraTargetPosition-m_data->m_cameraPosition);
	rayForward.normalize();
	float farPlane = 10000.f;
	rayForward*= farPlane;

//	b3Vector3 rightOffset;
	b3Vector3 vertical = m_data->m_cameraUp;

	b3Vector3 hor;
	hor = rayForward.cross(vertical);
	hor.normalize();
	vertical = hor.cross(rayForward);
	vertical.normalize();

	float tanfov = tanf(0.5f*fov);


	hor *= 2.f * farPlane * tanfov;
	vertical *= 2.f * farPlane * tanfov;

	b3Scalar aspect;

	aspect = m_screenWidth / (b3Scalar)m_screenHeight;

	hor*=aspect;


	b3Vector3 rayToCenter = rayFrom + rayForward;
	b3Vector3 dHor = hor * 1.f/float(m_screenWidth);
	b3Vector3 dVert = vertical * 1.f/float(m_screenHeight);


	b3Vector3 rayTo = rayToCenter - 0.5f * hor + 0.5f * vertical;
	rayTo += b3Scalar(x) * dHor;
	rayTo -= b3Scalar(y) * dVert;

	dir[0] = rayTo[0];
	dir[1] = rayTo[1];
	dir[2] = rayTo[2];

}



//#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
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
			pixels[(j*textureWidth+i)*numComponents+3]=255;


			//pixels[(j*textureWidth+i)*+1]=val;
			//pixels[(j*textureWidth+i)*numComponents+2]=val;
			//pixels[(j*textureWidth+i)*numComponents+3]=255;
		}

		/*	pixels[(j*textureWidth+j)*numComponents]=255;
			pixels[(j*textureWidth+j)*numComponents+1]=0;
			pixels[(j*textureWidth+j)*numComponents+2]=0;
			pixels[(j*textureWidth+j)*numComponents+3]=255;
			*/

	}
	if (0)
	{
		//swap the pixels
		unsigned char tmp;

		for (int j=0;j<textureHeight/2;j++)
		{
			for (int i=0;i<textureWidth;i++)
			{
				for (int c=0;c<numComponents;c++)
				{
					tmp = pixels[(j*textureWidth+i)*numComponents+c];
					pixels[(j*textureWidth+i)*numComponents+c]=
					pixels[((textureHeight-j-1)*textureWidth+i)*numComponents+c];
					pixels[((textureHeight-j-1)*textureWidth+i)*numComponents+c] = tmp;
				}
			}
		}
	}

	stbi_write_png(fileName, textureWidth,textureHeight, numComponents, pixels, textureWidth*numComponents);

	free(pixels);

}


void GLInstancingRenderer::renderScene()
{
	//avoid some Intel driver on a Macbook Pro to lock-up
	//todo: figure out what is going on on that machine

	glFlush();

	if (useShadowMap)
	{

		renderSceneInternal(B3_CREATE_SHADOWMAP_RENDERMODE);
	//	glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
		renderSceneInternal(B3_USE_SHADOWMAP_RENDERMODE);

	} else
	{
		renderSceneInternal();
	}

}

void GLInstancingRenderer::drawPoint(const float* positions, const float color[4], float pointDrawSize)
{
	drawPoints(positions,color,1,3*sizeof(float),pointDrawSize);
}
void GLInstancingRenderer::drawPoints(const float* positions, const float color[4], int numPoints, int pointStrideInBytes, float pointDrawSize)
{

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D,0);

	b3Assert(glGetError() ==GL_NO_ERROR);glUseProgram(linesShader);
	glUniformMatrix4fv(lines_ProjectionMatrix, 1, false, &projectionMatrix[0]);
	glUniformMatrix4fv(lines_ModelViewMatrix, 1, false, &modelviewMatrix[0]);
	glUniform4f(lines_colour,color[0],color[1],color[2],color[3]);

	glPointSize(pointDrawSize);
	glBindVertexArray(lineVertexArrayObject);

    glBindBuffer(GL_ARRAY_BUFFER, lineVertexBufferObject);

	int maxPointsInBatch = MAX_POINTS_IN_BATCH;
	int remainingPoints = numPoints;
	int offsetNumPoints= 0;
	while (1)
	{
		int curPointsInBatch = b3Min(maxPointsInBatch, remainingPoints);
		if (curPointsInBatch)
		{

			glBufferSubData(GL_ARRAY_BUFFER, 0, curPointsInBatch*pointStrideInBytes, positions + offsetNumPoints*(pointStrideInBytes / sizeof(float)));
			glEnableVertexAttribArray(0);
			int numFloats = 3;// pointStrideInBytes / sizeof(float);
			glVertexAttribPointer(0, numFloats, GL_FLOAT, GL_FALSE, pointStrideInBytes, 0);
			glDrawArrays(GL_POINTS, 0, curPointsInBatch);
			remainingPoints -= curPointsInBatch;
			offsetNumPoints += curPointsInBatch;
		}
		 else
		 {
			 break;
		 }
	}

	glBindVertexArray(0);
	glPointSize(1);
}

void GLInstancingRenderer::drawLines(const float* positions, const float color[4], int numPoints, int pointStrideInBytes, const unsigned int* indices, int numIndices, float pointDrawSize)
{

	glBindBuffer(GL_ARRAY_BUFFER, m_data->m_vbo);

	b3Assert(glGetError() ==GL_NO_ERROR);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D,0);

	b3Assert(glGetError() ==GL_NO_ERROR);glUseProgram(linesShader);
	glUniformMatrix4fv(lines_ProjectionMatrix, 1, false, &projectionMatrix[0]);
	glUniformMatrix4fv(lines_ModelViewMatrix, 1, false, &modelviewMatrix[0]);
	glUniform4f(lines_colour,color[0],color[1],color[2],color[3]);

//	glPointSize(pointDrawSize);
	glBindVertexArray(linesVertexArrayObject);

	b3Assert(glGetError() ==GL_NO_ERROR);
    glBindBuffer(GL_ARRAY_BUFFER, linesVertexBufferObject);

	{

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
	}

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
//	for (int i=0;i<numIndices;i++)
//		printf("indicec[i]=%d]\n",indices[i]);
	b3Assert(glGetError() ==GL_NO_ERROR);
	glBindVertexArray(0);
	b3Assert(glGetError() ==GL_NO_ERROR);
	glPointSize(1);
    b3Assert(glGetError() ==GL_NO_ERROR);

}

void GLInstancingRenderer::drawLine(const float from[4], const float to[4], const float color[4], float lineWidth)
{
	b3Assert(glGetError() ==GL_NO_ERROR);

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D,0);
	b3Assert(glGetError() ==GL_NO_ERROR);



	glUseProgram(linesShader);

	b3Assert(glGetError() ==GL_NO_ERROR);

	glUniformMatrix4fv(lines_ProjectionMatrix, 1, false, &projectionMatrix[0]);
	glUniformMatrix4fv(lines_ModelViewMatrix, 1, false, &modelviewMatrix[0]);
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


	{
		glBufferSubData(GL_ARRAY_BUFFER, 0,sz, vertexPositions);
	}


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
void GLInstancingRenderer::renderSceneInternal(int renderMode)
{

//	glEnable(GL_DEPTH_TEST);

	GLint dims[4];
        glGetIntegerv(GL_VIEWPORT, dims);
	//we need to get the viewport dims, because on Apple Retina the viewport dimension is different from screenWidth
	//printf("dims=%d,%d,%d,%d\n",dims[0],dims[1],dims[2],dims[3]);
	// Accept fragment if it closer to the camera than the former one
	//glDepthFunc(GL_LESS);

	// Cull triangles which normal is not towards the camera
	//glEnable(GL_CULL_FACE);



	 B3_PROFILE("GLInstancingRenderer::RenderScene");

	 {
		B3_PROFILE("init");
		init();
	 }


    b3Assert(glGetError() ==GL_NO_ERROR);

	float depthProjectionMatrix[4][4];
	GLfloat depthModelViewMatrix[4][4];
	//GLfloat depthModelViewMatrix2[4][4];

	// Compute the MVP matrix from the light's point of view
	if (renderMode==B3_CREATE_SHADOWMAP_RENDERMODE)
	{
		glEnable(GL_CULL_FACE);
		glCullFace(GL_FRONT);

		if (!m_data->m_shadowMap)
		{
			glActiveTexture(GL_TEXTURE0);

			glGenTextures(1,&m_data->m_shadowTexture);
			glBindTexture(GL_TEXTURE_2D,m_data->m_shadowTexture);
			//glTexImage2D(GL_TEXTURE_2D,0,GL_DEPTH_COMPONENT16,m_screenWidth,m_screenHeight,0,GL_DEPTH_COMPONENT,GL_FLOAT,0);
			//glTexImage2D(GL_TEXTURE_2D,0,GL_DEPTH_COMPONENT32,m_screenWidth,m_screenHeight,0,GL_DEPTH_COMPONENT,GL_FLOAT,0);
			glTexImage2D(GL_TEXTURE_2D, 0,GL_DEPTH_COMPONENT16, shadowMapWidth, shadowMapHeight, 0,GL_DEPTH_COMPONENT, GL_FLOAT, 0);

			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

			float l_ClampColor[] = {1.0, 1.0, 1.0, 1.0};
			glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, l_ClampColor);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
//			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
//			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_REF_TO_TEXTURE);

			m_data->m_shadowMap=new GLRenderToTexture();
			m_data->m_shadowMap->init(shadowMapWidth, shadowMapHeight,m_data->m_shadowTexture,RENDERTEXTURE_DEPTH);
		}
		m_data->m_shadowMap->enable();
		glViewport(0,0,shadowMapWidth,shadowMapHeight);
		//glClearColor(1,1,1,1);
		glClear(GL_DEPTH_BUFFER_BIT);
		//glClearColor(0.3,0.3,0.3,1);

//		m_data->m_shadowMap->disable();
	//	return;
	//	glEnable(GL_CULL_FACE);
//		glCullFace(GL_BACK); // Cull back-facing triangles -> draw only front-facing triangles

	b3Assert(glGetError() ==GL_NO_ERROR);
	} else
	{
		glDisable(GL_CULL_FACE);
		//glCullFace(GL_BACK);

	}

	b3CreateOrtho(-shadowMapWorldSize,shadowMapWorldSize,-shadowMapWorldSize,shadowMapWorldSize,1,300,depthProjectionMatrix);//-14,14,-14,14,1,200, depthProjectionMatrix);
	float depthViewMatrix[4][4];
	b3Vector3 center = b3MakeVector3(0,0,0);
	b3Vector3 up =b3MakeVector3(0,1,0);
	b3CreateLookAt(gLightPos,center,up,&depthViewMatrix[0][0]);
	//b3CreateLookAt(lightPos,m_data->m_cameraTargetPosition,b3Vector3(0,1,0),(float*)depthModelViewMatrix2);

	GLfloat depthModelMatrix[4][4];
	b3CreateDiagonalMatrix(1.f,depthModelMatrix);

	b3Matrix4x4Mul(depthViewMatrix, depthModelMatrix, depthModelViewMatrix);

	GLfloat depthMVP[4][4];
	b3Matrix4x4Mul(depthProjectionMatrix,depthModelViewMatrix,depthMVP);

	GLfloat biasMatrix[4][4]={
			0.5, 0.0, 0.0, 0.0,
			0.0, 0.5, 0.0, 0.0,
			0.0, 0.0, 0.5, 0.0,
			0.5, 0.5, 0.5, 1.0
	};

	GLfloat depthBiasMVP[4][4];
	b3Matrix4x4Mul(biasMatrix,depthMVP,depthBiasMVP);


	//float m_frustumZNear=0.1;
    //float m_frustumZFar=100.f;



	//b3CreateFrustum(-m_frustumZNear, m_frustumZNear, -m_frustumZNear, m_frustumZNear, m_frustumZNear, m_frustumZFar,(float*)depthProjectionMatrix);


    //b3CreateLookAt(lightPos,m_data->m_cameraTargetPosition,b3Vector3(0,0,1),(float*)depthModelViewMatrix);

	{
		B3_PROFILE("updateCamera");
	//	updateCamera();
	}

b3Assert(glGetError() ==GL_NO_ERROR);


	//	glBindBuffer(GL_ARRAY_BUFFER, 0);
	{
		B3_PROFILE("glFlush2");

		glBindBuffer(GL_ARRAY_BUFFER, m_data->m_vbo);
		glFlush();
	}
b3Assert(glGetError() ==GL_NO_ERROR);

	int totalNumInstances = 0;

	for (int i=0;i<m_graphicsInstances.size();i++)
	{
		totalNumInstances+=m_graphicsInstances[i]->m_numGraphicsInstances;
	}

	int curOffset = 0;
	//GLuint lastBindTexture = 0;


	for (int i=0;i<m_graphicsInstances.size();i++)
	{

		b3GraphicsInstance* gfxObj = m_graphicsInstances[i];
		if (gfxObj->m_numGraphicsInstances)
		{
			glActiveTexture(GL_TEXTURE0);
			GLuint curBindTexture = 0;
			if (gfxObj->m_texturehandle)
				curBindTexture = gfxObj->m_texturehandle;
			else
				curBindTexture = m_data->m_defaultTexturehandle;

//disable lazy evaluation, it just leads to bugs
			//if (lastBindTexture != curBindTexture)
			{
				glBindTexture(GL_TEXTURE_2D,curBindTexture);
			}
			//lastBindTexture = curBindTexture;

b3Assert(glGetError() ==GL_NO_ERROR);
	//	int myOffset = gfxObj->m_instanceOffset*4*sizeof(float);

		int POSITION_BUFFER_SIZE = (totalNumInstances*sizeof(float)*4);
		int ORIENTATION_BUFFER_SIZE = (totalNumInstances*sizeof(float)*4);
		int COLOR_BUFFER_SIZE = (totalNumInstances*sizeof(float)*4);
//		int SCALE_BUFFER_SIZE = (totalNumInstances*sizeof(float)*3);

		glBindVertexArray(gfxObj->m_cube_vao);


		int vertexStride = 9*sizeof(float);
		PointerCaster vertex;
		vertex.m_baseIndex = gfxObj->m_vertexArrayOffset*vertexStride;


		glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 9*sizeof(float), vertex.m_pointer);
		glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 0, (GLvoid *)(curOffset*4*sizeof(float)+m_maxShapeCapacityInBytes));
		glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, 0, (GLvoid *)(curOffset*4*sizeof(float)+m_maxShapeCapacityInBytes+POSITION_BUFFER_SIZE));

		PointerCaster uv;
		uv.m_baseIndex = 7*sizeof(float)+vertex.m_baseIndex;

		PointerCaster normal;
		normal.m_baseIndex = 4*sizeof(float)+vertex.m_baseIndex;

		glVertexAttribPointer(3, 2, GL_FLOAT, GL_FALSE, 9*sizeof(float), uv.m_pointer);
		glVertexAttribPointer(4, 3, GL_FLOAT, GL_FALSE, 9*sizeof(float), normal.m_pointer);
		glVertexAttribPointer(5, 4, GL_FLOAT, GL_FALSE, 0, (GLvoid *)(curOffset*4*sizeof(float)+m_maxShapeCapacityInBytes+POSITION_BUFFER_SIZE+ORIENTATION_BUFFER_SIZE));
		glVertexAttribPointer(6, 3, GL_FLOAT, GL_FALSE, 0, (GLvoid *)(curOffset*3*sizeof(float)+m_maxShapeCapacityInBytes+POSITION_BUFFER_SIZE+ORIENTATION_BUFFER_SIZE+COLOR_BUFFER_SIZE));

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
					glUniformMatrix4fv(ModelViewMatrixPointSprite, 1, false, &modelviewMatrix[0]);
					glUniform1f(screenWidthPointSprite,m_screenWidth);

					//glUniform1i(uniform_texture_diffusePointSprite, 0);
					  b3Assert(glGetError() ==GL_NO_ERROR);
					  glPointSize(20);

#ifndef __APPLE__
					glEnable(GL_POINT_SPRITE_ARB);
//					glTexEnvi(GL_POINT_SPRITE_ARB, GL_COORD_REPLACE_ARB, GL_TRUE);
#endif

					glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);
					glDrawElementsInstanced(GL_POINTS, indexCount, GL_UNSIGNED_INT, indexOffset, gfxObj->m_numGraphicsInstances);
				} else
				{
					switch (renderMode)
					{

					case B3_DEFAULT_RENDERMODE:
						{
							glUseProgram(instancingShader);
							glUniformMatrix4fv(ProjectionMatrix, 1, false, &projectionMatrix[0]);
							glUniformMatrix4fv(ModelViewMatrix, 1, false, &modelviewMatrix[0]);
							glUniform1i(uniform_texture_diffuse, 0);
							glDrawElementsInstanced(GL_TRIANGLES, indexCount, GL_UNSIGNED_INT, indexOffset, gfxObj->m_numGraphicsInstances);
							break;
						}
						case B3_CREATE_SHADOWMAP_RENDERMODE:
						{
                            /*printf("createShadowMapInstancingShader=%d\n",createShadowMapInstancingShader);
                            printf("createShadow_depthMVP=%d\n",createShadow_depthMVP);
                            printf("indexOffset=%d\n",indexOffset);
                            printf("gfxObj->m_numGraphicsInstances=%d\n",gfxObj->m_numGraphicsInstances);
                            printf("indexCount=%d\n",indexCount);
				*/
							glUseProgram(createShadowMapInstancingShader);
							glUniformMatrix4fv(createShadow_depthMVP, 1, false, &depthMVP[0][0]);
							glDrawElementsInstanced(GL_TRIANGLES, indexCount, GL_UNSIGNED_INT, indexOffset, gfxObj->m_numGraphicsInstances);
							break;
						}

					case B3_USE_SHADOWMAP_RENDERMODE:
						{
							glUseProgram(useShadowMapInstancingShader);
							glUniformMatrix4fv(useShadow_ProjectionMatrix, 1, false, &projectionMatrix[0]);
							glUniformMatrix4fv(useShadow_ModelViewMatrix, 1, false, &modelviewMatrix[0]);
							float MVP[16];
							b3Matrix4x4Mul16(projectionMatrix,modelviewMatrix,MVP);
							glUniformMatrix4fv(useShadow_MVP, 1, false, &MVP[0]);
							b3Vector3 gLightDir = gLightPos;
							gLightDir.normalize();
							glUniform3f(useShadow_lightDirIn,gLightDir[0],gLightDir[1],gLightDir[2]);
							glUniformMatrix4fv(useShadow_DepthBiasModelViewMatrix, 1, false, &depthBiasMVP[0][0]);
							glActiveTexture(GL_TEXTURE1);
							glBindTexture(GL_TEXTURE_2D, m_data->m_shadowTexture);
							glUniform1i(useShadow_shadowMap,1);
							glDrawElementsInstanced(GL_TRIANGLES, indexCount, GL_UNSIGNED_INT, indexOffset, gfxObj->m_numGraphicsInstances);
							break;
						}
					default:
						{
						//	b3Assert(0);
						}
					};
				}


				//glDrawElementsInstanced(GL_LINE_LOOP, indexCount, GL_UNSIGNED_INT, (void*)indexOffset, gfxObj->m_numGraphicsInstances);
			}
		}
		curOffset+= gfxObj->m_numGraphicsInstances;
	}

	{
			B3_PROFILE("glFlush");
			glFlush();
		}
	if (renderMode==B3_CREATE_SHADOWMAP_RENDERMODE)
	{
	//	writeTextureToPng(shadowMapWidth,shadowMapHeight,"shadowmap.png",4);
		m_data->m_shadowMap->disable();
		glViewport(dims[0],dims[1],dims[2],dims[3]);

	}

    b3Assert(glGetError() ==GL_NO_ERROR);
	{
		B3_PROFILE("glUseProgram(0);");
		glUseProgram(0);
		glBindBuffer(GL_ARRAY_BUFFER,0);
		glBindVertexArray(0);
	}




	b3Assert(glGetError() ==GL_NO_ERROR);
}


void GLInstancingRenderer::CleanupShaders()
{
}

void GLInstancingRenderer::enableShadowMap()
{
	glActiveTexture(GL_TEXTURE0);
	//glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, m_data->m_shadowTexture);
	//glBindTexture(GL_TEXTURE_2D, m_data->m_defaultTexturehandle);

}
