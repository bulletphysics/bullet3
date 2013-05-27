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



#include "OpenGLInclude.h"
#include "GLInstancingRenderer.h"

#include <string.h>
//#include "DemoSettings.h"
#include <stdio.h>
#include <assert.h>
#include "Bullet3Common/b3Vector3.h"
#include "Bullet3Common/b3Quaternion.h"
#include "Bullet3Common/b3Quickprof.h"
#include "Bullet3Common/b3Matrix3x3.h"
#include "LoadShader.h"



//#include "../../opencl/gpu_rigidbody_pipeline/b3GpuNarrowphaseAndSolver.h"//for m_maxNumObjectCapacity

static InternalDataRenderer* sData2;

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

	b3GraphicsInstance() :m_cube_vao(-1),m_index_vbo(-1),m_numIndices(-1),m_numVertices(-1),m_numGraphicsInstances(0),m_instanceOffset(0),m_vertexArrayOffset(0),m_primitiveType(B3_GL_TRIANGLES),m_texturehandle(0)
	{
	}

};




bool m_ortho = false;

static GLfloat projectionMatrix[16];
static GLfloat modelviewMatrix[16];

static void checkError(const char *functionName)
{
    GLenum error;
    while (( error = glGetError() ) != GL_NO_ERROR)
    {
        fprintf (stderr, "GL error 0x%X detected in %s\n", error, functionName);
    }
}



extern int gShapeIndex;








#include "GLInstanceRendererInternalData.h"

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
	int m_mouseButton;
	
	GLuint				m_defaultTexturehandle;
	b3AlignedObjectArray<GLuint>	m_textureHandles;

	InternalDataRenderer() :
		m_cameraPosition(b3Vector3(0,0,0)),
		m_cameraTargetPosition(b3Vector3(15,2,-24)),
		m_cameraDistance(150),
		m_cameraUp(0,1,0),
		m_azi(100.f),//135.f),
		//m_ele(25.f),
		m_ele(25.f),
		m_mouseInitialized(false),
		m_mouseButton(0)
	{
		

	}

	void wheelCallback( float deltax, float deltay)
    {
		if (!m_mouseButton)
		{
			if (b3Fabs(deltax)>b3Fabs(deltay))
			{
				m_azi -= deltax*0.1;
				
			} else
			{
				//m_cameraDistance -= deltay*0.1;
				b3Vector3 fwd = m_cameraTargetPosition-m_cameraPosition;
				fwd.normalize();
				m_cameraTargetPosition += fwd*deltay*0.1;
			}
        } else
		{
			if (b3Fabs(deltax)>b3Fabs(deltay))
			{
				b3Vector3 fwd = m_cameraTargetPosition-m_cameraPosition;
				b3Vector3 side = m_cameraUp.cross(fwd);
				side.normalize();
				m_cameraTargetPosition += side * deltax*0.1;

			} else
			{
				m_cameraTargetPosition -= m_cameraUp * deltay*0.1;

			}
		}
	}

	void mouseMoveCallback(float x, float y)
	{
		if (m_mouseButton)
		{
			float xDelta = x-m_mouseXpos;
			float yDelta = y-m_mouseYpos;
//			if (b3Fabs(xDelta)>b3Fabs(yDelta))
//			{
				m_azi += xDelta*0.1;
//			} else
//			{
				m_ele += yDelta*0.1;
//			}
		}
		
		m_mouseXpos = x;
		m_mouseYpos = y;
		m_mouseInitialized = true;
	}

	void mouseButtonCallback(int button, int state, float x, float y)
	{
		m_mouseButton=state;
		m_mouseXpos = x;
		m_mouseYpos = y;
		m_mouseInitialized = true;
	}
	void keyboardCallback(unsigned char key, int x, int y)
	{
		printf("world\n");
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
}


static GLuint               instancingShader;        // The instancing renderer
static GLuint               instancingShaderPointSprite;        // The point sprite instancing renderer




static bool                 done = false;
static GLint                angle_loc = 0;
static GLint	ModelViewMatrix=0;
static GLint	ProjectionMatrix=0;
static GLint                uniform_texture_diffuse = 0;

static GLint	screenWidthPointSprite=0;
static GLint	ModelViewMatrixPointSprite=0;
static GLint	ProjectionMatrixPointSprite=0;
static GLint	uniform_texture_diffusePointSprite= 0;



GLInstancingRenderer::GLInstancingRenderer(int maxNumObjectCapacity, int maxShapeCapacityInBytes)
	:m_maxNumObjectCapacity(maxNumObjectCapacity),
	m_maxShapeCapacityInBytes(maxShapeCapacityInBytes),
	m_textureenabled(true),
	m_textureinitialized(false)
{

	m_data = new InternalDataRenderer;
	sData2 = m_data;

	m_data->m_instance_positions_ptr.resize(m_maxNumObjectCapacity*4);
	m_data->m_instance_quaternion_ptr.resize(m_maxNumObjectCapacity*4);
	m_data->m_instance_colors_ptr.resize(m_maxNumObjectCapacity*4);
	m_data->m_instance_scale_ptr.resize(m_maxNumObjectCapacity*3);

}

GLInstancingRenderer::~GLInstancingRenderer()
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
	}
	sData2=0;

	if (m_data)
	{
		if (m_data->m_vbo)
			glDeleteBuffers(1,&m_data->m_vbo);
	}
	delete m_data;
}




//used for dynamic loading from disk (default switched off)
//#define MAX_SHADER_LENGTH   8192
//static GLubyte shaderText[MAX_SHADER_LENGTH];

static const char* vertexShader= \
"#version 330\n"
"precision highp float;\n"
"\n"
"\n"
"\n"
"layout (location = 0) in vec4 position;\n"
"layout (location = 1) in vec4 instance_position;\n"
"layout (location = 2) in vec4 instance_quaternion;\n"
"layout (location = 3) in vec2 uvcoords;\n"
"layout (location = 4) in vec3 vertexnormal;\n"
"layout (location = 5) in vec4 instance_color;\n"
"layout (location = 6) in vec3 instance_scale;\n"
"\n"
"\n"
"uniform float angle = 0.0;\n"
"uniform mat4 ModelViewMatrix;\n"
"uniform mat4 ProjectionMatrix;\n"
"\n"
"out Fragment\n"
"{\n"
"     vec4 color;\n"
"} fragment;\n"
"\n"
"out Vert\n"
"{\n"
"	vec2 texcoord;\n"
"} vert;\n"
"\n"
"\n"
"vec4 quatMul ( in vec4 q1, in vec4 q2 )\n"
"{\n"
"    vec3  im = q1.w * q2.xyz + q1.xyz * q2.w + cross ( q1.xyz, q2.xyz );\n"
"    vec4  dt = q1 * q2;\n"
"    float re = dot ( dt, vec4 ( -1.0, -1.0, -1.0, 1.0 ) );\n"
"    return vec4 ( im, re );\n"
"}\n"
"\n"
"vec4 quatFromAxisAngle(vec4 axis, in float angle)\n"
"{\n"
"    float cah = cos(angle*0.5);\n"
"    float sah = sin(angle*0.5);\n"
"	float d = inversesqrt(dot(axis,axis));\n"
"	vec4 q = vec4(axis.x*sah*d,axis.y*sah*d,axis.z*sah*d,cah);\n"
"	return q;\n"
"}\n"
"//\n"
"// vector rotation via quaternion\n"
"//\n"
"vec4 quatRotate3 ( in vec3 p, in vec4 q )\n"
"{\n"
"    vec4 temp = quatMul ( q, vec4 ( p, 0.0 ) );\n"
"    return quatMul ( temp, vec4 ( -q.x, -q.y, -q.z, q.w ) );\n"
"}\n"
"vec4 quatRotate ( in vec4 p, in vec4 q )\n"
"{\n"
"    vec4 temp = quatMul ( q, p );\n"
"    return quatMul ( temp, vec4 ( -q.x, -q.y, -q.z, q.w ) );\n"
"}\n"
"\n"
"out vec3 lightDir,normal,ambient;\n"
"\n"
"void main(void)\n"
"{\n"
"	vec4 q = instance_quaternion;\n"
"	ambient = vec3(0.3,.3,0.3);\n"
"		\n"
"		\n"
"	vec4 local_normal = (quatRotate3( vertexnormal,q));\n"
"	vec3 light_pos = vec3(-0.3,0.1,0.1);\n"
"	normal = local_normal.xyz;\n"//normalize(ModelViewMatrix * local_normal).xyz;\n"
"\n"
"	lightDir = normalize(light_pos);//gl_LightSource[0].position.xyz));\n"
"//	lightDir = normalize(vec3(gl_LightSource[0].position));\n"
"		\n"
"	vec4 axis = vec4(1,1,1,0);\n"
"	vec4 localcoord = quatRotate3( position.xyz*instance_scale,q);\n"
"	vec4 vertexPos = ProjectionMatrix * ModelViewMatrix *(instance_position+localcoord);\n"
"\n"
"	gl_Position = vertexPos;\n"
"	\n"
"	fragment.color = instance_color;\n"
"	vert.texcoord = uvcoords;\n"
"}\n"
;


static const char* fragmentShader= \
"#version 330\n"
"precision highp float;\n"
"\n"
"in Fragment\n"
"{\n"
"     vec4 color;\n"
"} fragment;\n"
"\n"
"in Vert\n"
"{\n"
"	vec2 texcoord;\n"
"} vert;\n"
"\n"
"uniform sampler2D Diffuse;\n"
"\n"
"in vec3 lightDir,normal,ambient;\n"
"\n"
"out vec4 color;\n"
"\n"
"void main_textured(void)\n"
"{\n"
"    color =  fragment.color;//texture2D(Diffuse,vert.texcoord);//fragment.color;\n"
"}\n"
"\n"
"void main(void)\n"
"{\n"
"    vec4 texel = fragment.color*texture(Diffuse,vert.texcoord);//fragment.color;\n"
"	vec3 ct,cf;\n"
"	float intensity,at,af;\n"
"	intensity = max(dot(lightDir,normalize(normal)),0);\n"
"	cf = intensity*vec3(1.0,1.0,1.0)+ambient;"
"	af = 1.0;\n"
"		\n"
"	ct = texel.rgb;\n"
"	at = texel.a;\n"
"		\n"
"	color  = vec4(ct * cf, at * af);	\n"
"}\n"
;


static const char* vertexShaderPointSprite= \
"#version 330\n"
"precision highp float;\n"
"\n"
"\n"
"\n"
"layout (location = 0) in vec4 position;\n"
"layout (location = 1) in vec4 instance_position;\n"
"layout (location = 3) in vec2 uvcoords;\n"
"layout (location = 4) in vec3 vertexnormal;\n"
"layout (location = 5) in vec4 instance_color;\n"
"layout (location = 6) in vec3 instance_scale;\n"
"\n"
"\n"
"uniform float screenWidth = 700.f;\n"
"uniform mat4 ModelViewMatrix;\n"
"uniform mat4 ProjectionMatrix;\n"
"\n"
"out Fragment\n"
"{\n"
"     vec4 color;\n"
"} fragment;\n"
"\n"
"\n"
"\n"
"//\n"
"// vector rotation via quaternion\n"
"//\n"
"\n"
"out vec3 ambient;\n"
"\n"
"void main(void)\n"
"{\n"
"	ambient = vec3(0.3,.3,0.3);\n"
"		\n"
"		\n"
"	vec4 axis = vec4(1,1,1,0);\n"
"	vec4 vertexPos = ProjectionMatrix * ModelViewMatrix *(instance_position);\n"
"	vec3 posEye = vec3(ModelViewMatrix * vec4(instance_position.xyz, 1.0));\n"
"   float dist = length(posEye);\n"
"	float pointRadius = 1.f;\n"
"    gl_PointSize = instance_scale.x * pointRadius * (screenWidth / dist);\n"
"\n"
"	gl_Position = vertexPos;\n"
"	\n"
"	fragment.color = instance_color;\n"
"}\n"
;


static const char* fragmentShaderPointSprite= \
"#version 330\n"
"precision highp float;\n"
"\n"
"in Fragment\n"
"{\n"
"     vec4 color;\n"
"} fragment;\n"
"\n"
"\n"
"in vec3 ambient;\n"
"\n"
"out vec4 color;\n"
"\n"
"void main_textured(void)\n"
"{\n"
"    color =  fragment.color;//texture2D(Diffuse,vert.texcoord);//fragment.color;\n"
"}\n"
"\n"
"void main(void)\n"
"{\n"
"	vec3 N;\n"
"	N.xy = gl_PointCoord.st*vec2(2.0, -2.0) + vec2(-1.0, 1.0);\n"
"    float mag = dot(N.xy, N.xy);\n"
"    if (mag > 1.0) discard; \n"
"    vec4 texel = vec4(1,0,0,1);\n"//fragment.color*texture(Diffuse,vert.texcoord);//fragment.color;\n"
"	vec3 ct;\n"
"	float at,af;\n"
"	af = 1.0;\n"
"		\n"
"	ct = texel.rgb;\n"
"	at = texel.a;\n"
"		\n"
" vec3 lightDir= vec3(1,0,0);\n"
"	float diffuse = max(0.0, dot(lightDir, N));\n"
"	color  = vec4(ct * diffuse, at * af);	\n"
"}\n"
;



void GLInstancingRenderer::writeSingleInstanceTransformToCPU(float* position, float* orientation, int srcIndex)
{
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
	
	glBindBuffer(GL_ARRAY_BUFFER, m_data->m_vbo);
	glFlush();
	
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
		printf("ERROR glMapBuffer failed\n");
	}
	glUnmapBuffer( GL_ARRAY_BUFFER);
	//if this glFinish is removed, the animation is not always working/blocks
	//@todo: figure out why
	glFlush();
	glBindBuffer(GL_ARRAY_BUFFER, 0);//m_data->m_vbo);

    GLint err = glGetError();
    assert(err==GL_NO_ERROR);
    
}


int GLInstancingRenderer::registerGraphicsInstance(int shapeIndex, const float* position, const float* quaternion, const float* color, const float* scaling)
{
	b3Assert(shapeIndex == (m_graphicsInstances.size()-1));
	b3Assert(m_graphicsInstances.size()<m_maxNumObjectCapacity-1);

	b3GraphicsInstance* gfxObj = m_graphicsInstances[shapeIndex];

	int index = gfxObj->m_numGraphicsInstances + gfxObj->m_instanceOffset;
	


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
	return gfxObj->m_numGraphicsInstances;
}


int	GLInstancingRenderer::registerTexture(const unsigned char* texels, int width, int height)
{
	int textureIndex = m_data->m_textureHandles.size();
	const GLubyte*	image= (const GLubyte*)texels;
	GLuint textureHandle;
	glGenTextures(1,(GLuint*)&textureHandle);
	glBindTexture(GL_TEXTURE_2D,textureHandle);
	GLenum err;
	err = glGetError();
	assert(err==GL_NO_ERROR);
	err = glGetError();
	assert(err==GL_NO_ERROR);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width,height,0,GL_RGB,GL_UNSIGNED_BYTE,image);
	glGenerateMipmap(GL_TEXTURE_2D);
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



	instancingShaderPointSprite = gltLoadShaderPair(vertexShaderPointSprite,fragmentShaderPointSprite);
	glUseProgram(instancingShaderPointSprite);
	ModelViewMatrixPointSprite = glGetUniformLocation(instancingShaderPointSprite, "ModelViewMatrix");
	ProjectionMatrixPointSprite = glGetUniformLocation(instancingShaderPointSprite, "ProjectionMatrix");
	screenWidthPointSprite = glGetUniformLocation(instancingShaderPointSprite, "screenWidth");
	

	instancingShader = gltLoadShaderPair(vertexShader,fragmentShader);
	glLinkProgram(instancingShader);
	glUseProgram(instancingShader);
	angle_loc = glGetUniformLocation(instancingShader, "angle");
	ModelViewMatrix = glGetUniformLocation(instancingShader, "ModelViewMatrix");
	ProjectionMatrix = glGetUniformLocation(instancingShader, "ProjectionMatrix");
	uniform_texture_diffuse = glGetUniformLocation(instancingShader, "Diffuse");
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
	GLint err = glGetError();
    assert(err==GL_NO_ERROR);
    
    err = glGetError();
	assert(err==GL_NO_ERROR);
    
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LESS);

    err = glGetError();
	assert(err==GL_NO_ERROR);
    
	glClearColor(float(0.7),float(0.7),float(0.7),float(0));
	
    err = glGetError();
	assert(err==GL_NO_ERROR);

	
    err = glGetError();
	assert(err==GL_NO_ERROR);
    
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
				err = glGetError();
				assert(err==GL_NO_ERROR);
	#if 0

				glTexEnvf(GL_TEXTURE_ENV,GL_TEXTURE_ENV_MODE,GL_MODULATE);
				err = glGetError();
				assert(err==GL_NO_ERROR);
            
				glTexParameterf(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR_MIPMAP_LINEAR);
				err = glGetError();
				assert(err==GL_NO_ERROR);
            
				glTexParameterf(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
				err = glGetError();
				assert(err==GL_NO_ERROR);
            
				glTexParameterf(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_REPEAT);
				err = glGetError();
				assert(err==GL_NO_ERROR);
            
				glTexParameterf(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_REPEAT);
				err = glGetError();
				assert(err==GL_NO_ERROR);
            
          
	#endif
				 err = glGetError();
				assert(err==GL_NO_ERROR);
				glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 256,256,0,GL_RGB,GL_UNSIGNED_BYTE,image);
				glGenerateMipmap(GL_TEXTURE_2D);
			
				err = glGetError();
				assert(err==GL_NO_ERROR);
            
				delete[] image;
				m_textureinitialized=true;
			}
			
			err = glGetError();
			assert(err==GL_NO_ERROR);
        
			glBindTexture(GL_TEXTURE_2D,m_data->m_defaultTexturehandle);
			err = glGetError();
			assert(err==GL_NO_ERROR);
        

		} else
		{
			glDisable(GL_TEXTURE_2D);
			err = glGetError();
			assert(err==GL_NO_ERROR);
		}
	}
	//glEnable(GL_COLOR_MATERIAL);
	 
	err = glGetError();
	assert(err==GL_NO_ERROR);

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



void    b3CreateLookAt(const b3Vector3& eye, const b3Vector3& center,const b3Vector3& up, GLfloat result[16])
{
    b3Vector3 f = (center - eye).normalized();
    b3Vector3 u = up.normalized();
    b3Vector3 s = (f.cross(u)).normalized();
    u = s.cross(f);
    
    result[0*4+0] = s.getX();
    result[1*4+0] = s.getY();
    result[2*4+0] = s.getZ();
    result[0*4+1] = u.getX();
    result[1*4+1] = u.getY();
    result[2*4+1] = u.getZ();
    result[0*4+2] =-f.getX();
    result[1*4+2] =-f.getY();
    result[2*4+2] =-f.getZ();
    
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

void GLInstancingRenderer::updateCamera() 
{

    GLint err = glGetError();
    assert(err==GL_NO_ERROR);
    

	int m_forwardAxis(2);

    float m_frustumZNear=1;
    float m_frustumZFar=10000.f;
    

//    m_azi=m_azi+0.01;
	b3Scalar rele = m_data->m_ele * b3Scalar(0.01745329251994329547);// rads per deg
	b3Scalar razi = m_data->m_azi * b3Scalar(0.01745329251994329547);// rads per deg


	b3Quaternion rot(m_data->m_cameraUp,razi);


	b3Vector3 eyePos(0,0,0);
	eyePos[m_forwardAxis] = -m_data->m_cameraDistance;

	b3Vector3 forward(eyePos[0],eyePos[1],eyePos[2]);
	if (forward.length2() < B3_EPSILON)
	{
		forward.setValue(1.f,0.f,0.f);
	}
	b3Vector3 right = m_data->m_cameraUp.cross(forward);
	b3Quaternion roll(right,-rele);

	eyePos = b3Matrix3x3(rot) * b3Matrix3x3(roll) * eyePos;

	m_data->m_cameraPosition[0] = eyePos.getX();
	m_data->m_cameraPosition[1] = eyePos.getY();
	m_data->m_cameraPosition[2] = eyePos.getZ();
	m_data->m_cameraPosition += m_data->m_cameraTargetPosition;

	if (m_screenWidth == 0 && m_screenHeight == 0)
		return;

	b3Scalar aspect;
	b3Vector3 extents;

	aspect = m_screenWidth / (b3Scalar)m_screenHeight;
	extents.setValue(aspect * 1.0f, 1.0f,0);
	
	
	if (m_ortho)
	{
		// reset matrix
		
		
		extents *= m_data->m_cameraDistance;
		//b3Vector3 lower = m_data->m_cameraTargetPosition - extents;
		//b3Vector3 upper = m_data->m_cameraTargetPosition + extents;
		//gluOrtho2D(lower.x, upper.x, lower.y, upper.y);
				//glTranslatef(100,210,0);
	} else
	{
//		glFrustum (-aspect, aspect, -1.0, 1.0, 1.0, 10000.0);
	}
    
    
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

void	GLInstancingRenderer::setCameraTargetPosition(float cameraPos[4])
{
		m_data->m_cameraTargetPosition = b3Vector3(cameraPos[0],cameraPos[1],cameraPos[2]);
}

void	GLInstancingRenderer::getCameraTargetPosition(float cameraPos[4]) const
{
	cameraPos[0] = m_data->m_cameraTargetPosition.getX();
	cameraPos[1] = m_data->m_cameraTargetPosition.getY();
	cameraPos[2] = m_data->m_cameraTargetPosition.getZ();
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

	b3Vector3 rightOffset;
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


void GLInstancingRenderer::RenderScene(void)
{
	 B3_PROFILE("GLInstancingRenderer::RenderScene");

	 {
		B3_PROFILE("init");
		init();
	 }

    GLint err = glGetError();
    assert(err==GL_NO_ERROR);
    
	{
		B3_PROFILE("updateCamera");
		updateCamera();
	}
    err = glGetError();
    assert(err==GL_NO_ERROR);
    
    
	//render coordinate system
#if 0
    glBegin(GL_LINES);
	err = glGetError();
    assert(err==GL_NO_ERROR);
    
    glColor3f(1,0,0);
	glVertex3f(0,0,0);
	glVertex3f(1,0,0);
	glColor3f(0,1,0);
	glVertex3f(0,0,0);
	glVertex3f(0,1,0);
	glColor3f(0,0,1);
	glVertex3f(0,0,0);
	glVertex3f(0,0,1);
	glEnd();
#endif
    
    
	//do a finish, to make sure timings are clean
	//	glFinish();



	//	glBindBuffer(GL_ARRAY_BUFFER, 0);
	{
		B3_PROFILE("glFlush2");

		glBindBuffer(GL_ARRAY_BUFFER, m_data->m_vbo);
		glFlush();
	}
    err = glGetError();
    assert(err==GL_NO_ERROR);
    
	//updatePos();

//	simulationLoop();

	//useCPU = true;

	int totalNumInstances = 0;

	for (int i=0;i<m_graphicsInstances.size();i++)
	{
		totalNumInstances+=m_graphicsInstances[i]->m_numGraphicsInstances;
	}

	int curOffset = 0;
	GLuint lastBindTexture = 0;


	for (int i=0;i<m_graphicsInstances.size();i++)
	{
		
		b3GraphicsInstance* gfxObj = m_graphicsInstances[i];
		if (gfxObj->m_numGraphicsInstances)
		{

			GLuint curBindTexture = 0;
			if (gfxObj->m_texturehandle)
				curBindTexture = gfxObj->m_texturehandle;
			else
				curBindTexture = m_data->m_defaultTexturehandle;

			if (lastBindTexture != curBindTexture)
			{
				glBindTexture(GL_TEXTURE_2D,curBindTexture);
			}
			lastBindTexture = curBindTexture;

			err = glGetError();
			assert(err==GL_NO_ERROR);
	//	int myOffset = gfxObj->m_instanceOffset*4*sizeof(float);

		int POSITION_BUFFER_SIZE = (totalNumInstances*sizeof(float)*4);
		int ORIENTATION_BUFFER_SIZE = (totalNumInstances*sizeof(float)*4);
		int COLOR_BUFFER_SIZE = (totalNumInstances*sizeof(float)*4);
//		int SCALE_BUFFER_SIZE = (totalNumInstances*sizeof(float)*3);

		glBindVertexArray(gfxObj->m_cube_vao);

		
		int vertexStride = 9*sizeof(float);
		int vertexBase = gfxObj->m_vertexArrayOffset*vertexStride;

		glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 9*sizeof(float), (GLvoid*)vertexBase);
		glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 0, (GLvoid *)(curOffset*4*sizeof(float)+m_maxShapeCapacityInBytes));
		glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, 0, (GLvoid *)(curOffset*4*sizeof(float)+m_maxShapeCapacityInBytes+POSITION_BUFFER_SIZE));
		int uvoffset = 7*sizeof(float)+vertexBase;
		int normaloffset = 4*sizeof(float)+vertexBase;

		glVertexAttribPointer(3, 2, GL_FLOAT, GL_FALSE, 9*sizeof(float), (GLvoid *)uvoffset);
		glVertexAttribPointer(4, 3, GL_FLOAT, GL_FALSE, 9*sizeof(float), (GLvoid *)normaloffset);
		glVertexAttribPointer(5, 4, GL_FLOAT, GL_FALSE, 0, (GLvoid *)(curOffset*4*sizeof(float)+m_maxShapeCapacityInBytes+POSITION_BUFFER_SIZE+ORIENTATION_BUFFER_SIZE));
		glVertexAttribPointer(6, 3, GL_FLOAT, GL_FALSE, 0, (GLvoid *)(curOffset*3*sizeof(float)+m_maxShapeCapacityInBytes+POSITION_BUFFER_SIZE+ORIENTATION_BUFFER_SIZE+COLOR_BUFFER_SIZE));

		glEnableVertexAttribArray(0);
		glEnableVertexAttribArray(1);
		glEnableVertexAttribArray(2);
		glEnableVertexAttribArray(3);
		glEnableVertexAttribArray(4);
		glEnableVertexAttribArray(5);
		glEnableVertexAttribArray(6);
		glVertexAttribDivisorARB(0, 0);
		glVertexAttribDivisorARB(1, 1);
		glVertexAttribDivisorARB(2, 1);
		glVertexAttribDivisorARB(3, 0);
		glVertexAttribDivisorARB(4, 0);
		glVertexAttribDivisorARB(5, 1);
		glVertexAttribDivisorARB(6, 1);
        
		
		

		{
			B3_PROFILE("glFlush");
			glFlush();
		}
		
			int indexCount = gfxObj->m_numIndices;
			int indexOffset = 0;

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
					  err = glGetError();
    assert(err==GL_NO_ERROR);
					glPointSize(20);

#ifndef __APPLE__
					glEnable(GL_POINT_SPRITE_ARB);
					glTexEnvi(GL_POINT_SPRITE_ARB, GL_COORD_REPLACE_ARB, GL_TRUE);
#endif

					glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);
					glDrawElementsInstanced(GL_POINTS, indexCount, GL_UNSIGNED_INT, (void*)indexOffset, gfxObj->m_numGraphicsInstances);
				} else
				{
					glUseProgram(instancingShader);
					glUniform1f(angle_loc, 0);
					glUniformMatrix4fv(ProjectionMatrix, 1, false, &projectionMatrix[0]);
					glUniformMatrix4fv(ModelViewMatrix, 1, false, &modelviewMatrix[0]);
					glUniform1i(uniform_texture_diffuse, 0);
					glDrawElementsInstanced(GL_TRIANGLES, indexCount, GL_UNSIGNED_INT, (void*)indexOffset, gfxObj->m_numGraphicsInstances);
				}

				
				//glDrawElementsInstanced(GL_LINE_LOOP, indexCount, GL_UNSIGNED_INT, (void*)indexOffset, gfxObj->m_numGraphicsInstances);
			}
		}
		curOffset+= gfxObj->m_numGraphicsInstances;
	}
    err = glGetError();
    assert(err==GL_NO_ERROR);
	{
		B3_PROFILE("glUseProgram(0);");
		glUseProgram(0);
		glBindBuffer(GL_ARRAY_BUFFER,0);
		glBindVertexArray(0);
	}
	
	err = glGetError();
	assert(err==GL_NO_ERROR);
	
}


void GLInstancingRenderer::CleanupShaders()
{
}
