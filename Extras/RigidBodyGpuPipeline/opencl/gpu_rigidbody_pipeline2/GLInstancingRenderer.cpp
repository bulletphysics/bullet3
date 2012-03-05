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
#include "DemoSettings.h"
#include <stdio.h>
#include <assert.h>
#include "LinearMath/btVector3.h"
#include "LinearMath/btQuaternion.h"
#include "LinearMath/btQuickprof.h"
#include "LinearMath/btMatrix3x3.h"

#include "../../opencl/gpu_rigidbody_pipeline/btGpuNarrowphaseAndSolver.h"//for MAX_CONVEX_BODIES_CL

struct btGraphicsInstance
{
	GLuint               m_cube_vao;
	GLuint               m_index_vbo;
	int m_numIndices;
	int m_numVertices;

	int m_numGraphicsInstances;
	
	int m_instanceOffset;
	int m_vertexArrayOffset;

	btGraphicsInstance() :m_cube_vao(-1),m_index_vbo(-1),m_numIndices(-1),m_numVertices(-1),m_numGraphicsInstances(0),m_instanceOffset(0),m_vertexArrayOffset(0)
	{
	}

};



bool m_ortho = false;
int m_glutScreenWidth = 1024;
int m_glutScreenHeight = 768;



extern int gShapeIndex;


btVector3 m_cameraPosition(0,0,0);//will be overridden by a position computed from azi/ele
btVector3 m_cameraTargetPosition(30,-5,-20);
btScalar m_cameraDistance = 95;
btVector3 m_cameraUp(0,1,0);
float m_azi=95.f;
float m_ele=15.f;




int VBOsize =0;



struct InternalDataRenderer
{
	GLfloat* m_instance_positions_ptr;
	GLfloat* m_instance_quaternion_ptr;
	GLfloat* m_instance_colors_ptr;
	GLfloat* m_instance_scale_ptr;

	InternalDataRenderer() :m_instance_positions_ptr (0),m_instance_quaternion_ptr(0),m_instance_colors_ptr(0),m_instance_scale_ptr(0)
	{
	}

};

static GLuint               instancingShader;        // The instancing renderer

GLuint               cube_vbo;

static GLuint				m_texturehandle;

static bool                 done = false;
static GLint                angle_loc = 0;
static GLint ModelViewMatrix;
static GLint ProjectionMatrix;



GLInstancingRenderer::GLInstancingRenderer()
{

	m_data = new InternalDataRenderer;

	m_data->m_instance_positions_ptr = (GLfloat*)new float[MAX_CONVEX_BODIES_CL*4];
	m_data->m_instance_quaternion_ptr = (GLfloat*)new float[MAX_CONVEX_BODIES_CL*4];
	m_data->m_instance_colors_ptr = (GLfloat*)new float[MAX_CONVEX_BODIES_CL*4];
	m_data->m_instance_scale_ptr = (GLfloat*)new float[MAX_CONVEX_BODIES_CL*3];

}

GLInstancingRenderer::~GLInstancingRenderer()
{
	delete m_data;
}


static GLint                uniform_texture_diffuse = 0;

//used for dynamic loading from disk (default switched off)
#define MAX_SHADER_LENGTH   8192
static GLubyte shaderText[MAX_SHADER_LENGTH];

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
"	vec3 light_pos = vec3(-0.8,1,-0.6);\n"
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
"    color =  texture2D(Diffuse,vert.texcoord);//fragment.color;\n"
"}\n"
"\n"
"void main(void)\n"
"{\n"
"    vec4 texel = fragment.color*texture2D(Diffuse,vert.texcoord);//fragment.color;\n"
"	vec3 ct,cf;\n"
"	float intensity,at,af;\n"
"	intensity = max(dot(lightDir,normalize(normal)),.2);\n"
"	cf = intensity*vec3(1.0,1.0,1.0)+ambient;"
"	af = 1.0;\n"
"		\n"
"	ct = texel.rgb;\n"
"	at = texel.a;\n"
"		\n"
"	color  = vec4(ct * cf, at * af);	\n"
"}\n"
;


// Load the shader from the source text
void gltLoadShaderSrc(const char *szShaderSrc, GLuint shader)
{
	GLchar *fsStringPtr[1];

	fsStringPtr[0] = (GLchar *)szShaderSrc;
	glShaderSource(shader, 1, (const GLchar **)fsStringPtr, NULL);
}


GLuint gltLoadShaderPair(const char *szVertexProg, const char *szFragmentProg)
{
	// Temporary Shader objects
	GLuint hVertexShader;
	GLuint hFragmentShader; 
	GLuint hReturn = 0;   
	GLint testVal;

	// Create shader objects
	hVertexShader = glCreateShader(GL_VERTEX_SHADER);
	hFragmentShader = glCreateShader(GL_FRAGMENT_SHADER);

	gltLoadShaderSrc(vertexShader, hVertexShader);
	gltLoadShaderSrc(fragmentShader, hFragmentShader);
	
	// Compile them
	glCompileShader(hVertexShader);
	glCompileShader(hFragmentShader);

	// Check for errors
	glGetShaderiv(hVertexShader, GL_COMPILE_STATUS, &testVal);
	if(testVal == GL_FALSE)
	{
			 char temp[256] = "";
			glGetShaderInfoLog( hVertexShader, 256, NULL, temp);
			fprintf( stderr, "Compile failed:\n%s\n", temp);
			assert(0);
			exit(0);
		glDeleteShader(hVertexShader);
		glDeleteShader(hFragmentShader);
		return (GLuint)NULL;
	}

	glGetShaderiv(hFragmentShader, GL_COMPILE_STATUS, &testVal);
	if(testVal == GL_FALSE)
	{
		 char temp[256] = "";
			glGetShaderInfoLog( hFragmentShader, 256, NULL, temp);
			fprintf( stderr, "Compile failed:\n%s\n", temp);
			assert(0);
			exit(0);
		glDeleteShader(hVertexShader);
		glDeleteShader(hFragmentShader);
		return (GLuint)NULL;
	}

	// Link them - assuming it works...
	hReturn = glCreateProgram();
	glAttachShader(hReturn, hVertexShader);
	glAttachShader(hReturn, hFragmentShader);

	glLinkProgram(hReturn);

	// These are no longer needed
	glDeleteShader(hVertexShader);
	glDeleteShader(hFragmentShader);  

	// Make sure link worked too
	glGetProgramiv(hReturn, GL_LINK_STATUS, &testVal);
	if(testVal == GL_FALSE)
	{
		glDeleteProgram(hReturn);
		return (GLuint)NULL;
	}

	return hReturn;  
}   


void GLInstancingRenderer::writeTransforms()
{
	glBindBuffer(GL_ARRAY_BUFFER, cube_vbo);
	glFlush();
	
	char* orgBase =  (char*)glMapBuffer( GL_ARRAY_BUFFER,GL_READ_WRITE);

	int totalNumInstances= 0;

	for (int k=0;k<m_graphicsInstances.size();k++)
	{
		btGraphicsInstance* gfxObj = m_graphicsInstances[k];
		totalNumInstances+=gfxObj->m_numGraphicsInstances;
	}



	for (int k=0;k<m_graphicsInstances.size();k++)
	{
		//int k=0;
		btGraphicsInstance* gfxObj = m_graphicsInstances[k];

	

		int POSITION_BUFFER_SIZE = (totalNumInstances*sizeof(float)*4);
		int ORIENTATION_BUFFER_SIZE = (totalNumInstances*sizeof(float)*4);
		int COLOR_BUFFER_SIZE = (totalNumInstances*sizeof(float)*4);
		int SCALE_BUFFER_SIZE = (totalNumInstances*sizeof(float)*3);

		char* base = orgBase;

		float* positions = (float*)(base+SHAPE_BUFFER_SIZE);
		float* orientations = (float*)(base+SHAPE_BUFFER_SIZE + POSITION_BUFFER_SIZE);
		float* colors= (float*)(base+SHAPE_BUFFER_SIZE + POSITION_BUFFER_SIZE+ORIENTATION_BUFFER_SIZE);
		float* scaling= (float*)(base+SHAPE_BUFFER_SIZE + POSITION_BUFFER_SIZE+ORIENTATION_BUFFER_SIZE+COLOR_BUFFER_SIZE);

		static int offset=0;
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

	glUnmapBuffer( GL_ARRAY_BUFFER);
	//if this glFinish is removed, the animation is not always working/blocks
	//@todo: figure out why
	glFlush();
}

int GLInstancingRenderer::registerGraphicsInstance(int shapeIndex, const float* position, const float* quaternion, const float* color, const float* scaling)
{
	btGraphicsInstance* gfxObj = m_graphicsInstances[shapeIndex];

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


int GLInstancingRenderer::registerShape(const float* vertices, int numvertices, const int* indices, int numIndices)
{
	btGraphicsInstance* gfxObj = new btGraphicsInstance;
	
	if (m_graphicsInstances.size())
	{
		btGraphicsInstance* prevObj = m_graphicsInstances[m_graphicsInstances.size()-1];
		gfxObj->m_instanceOffset = prevObj->m_instanceOffset + prevObj->m_numGraphicsInstances;
		gfxObj->m_vertexArrayOffset = prevObj->m_vertexArrayOffset + prevObj->m_numVertices;
	} else
	{
		gfxObj->m_instanceOffset = 0;
	}

	m_graphicsInstances.push_back(gfxObj);
	gfxObj->m_numIndices = numIndices;
	gfxObj->m_numVertices = numvertices;
	
	
	glBindBuffer(GL_ARRAY_BUFFER, cube_vbo);
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
	glBindBuffer(GL_ARRAY_BUFFER, cube_vbo);
	glBindVertexArray(0);
	glBindBuffer(GL_ARRAY_BUFFER,0);
	glBindVertexArray(0);

	
	return m_graphicsInstances.size()-1;
}




void GLInstancingRenderer::InitShaders()
{
	
	int POSITION_BUFFER_SIZE = (MAX_CONVEX_BODIES_CL*sizeof(float)*4);
	int ORIENTATION_BUFFER_SIZE = (MAX_CONVEX_BODIES_CL*sizeof(float)*4);
	int COLOR_BUFFER_SIZE = (MAX_CONVEX_BODIES_CL*sizeof(float)*4);
	int SCALE_BUFFER_SIZE = (MAX_CONVEX_BODIES_CL*sizeof(float)*3);


	instancingShader = gltLoadShaderPair(vertexShader,fragmentShader);

	glLinkProgram(instancingShader);
	glUseProgram(instancingShader);
	angle_loc = glGetUniformLocation(instancingShader, "angle");
	ModelViewMatrix = glGetUniformLocation(instancingShader, "ModelViewMatrix");
	ProjectionMatrix = glGetUniformLocation(instancingShader, "ProjectionMatrix");
	uniform_texture_diffuse = glGetUniformLocation(instancingShader, "Diffuse");

	GLuint offset = 0;


	glGenBuffers(1, &cube_vbo);
	glBindBuffer(GL_ARRAY_BUFFER, cube_vbo);


	int size = SHAPE_BUFFER_SIZE  + POSITION_BUFFER_SIZE+ORIENTATION_BUFFER_SIZE+COLOR_BUFFER_SIZE+SCALE_BUFFER_SIZE;
	VBOsize = size;

	glBufferData(GL_ARRAY_BUFFER, size, 0, GL_DYNAMIC_DRAW);//GL_STATIC_DRAW);

	glBindBuffer(GL_ARRAY_BUFFER,0);
	glBindVertexArray(0);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	

}


void myinit()
{
	GLint err = glGetError();

	//	GLfloat light_ambient[] = { btScalar(0.2), btScalar(0.2), btScalar(0.2), btScalar(1.0) };
	GLfloat light_ambient[] = { btScalar(1.0), btScalar(1.2), btScalar(0.2), btScalar(1.0) };

	GLfloat light_diffuse[] = { btScalar(1.0), btScalar(1.0), btScalar(1.0), btScalar(1.0) };
	GLfloat light_specular[] = { btScalar(1.0), btScalar(1.0), btScalar(1.0), btScalar(1.0 )};
	/*	light_position is NOT default value	*/
	GLfloat light_position0[] = { btScalar(10000.0), btScalar(10000.0), btScalar(10000.0), btScalar(0.0 )};
	GLfloat light_position1[] = { btScalar(-1.0), btScalar(-10.0), btScalar(-1.0), btScalar(0.0) };

	glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
	glLightfv(GL_LIGHT0, GL_POSITION, light_position0);

	glLightfv(GL_LIGHT1, GL_AMBIENT, light_ambient);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, light_diffuse);
	glLightfv(GL_LIGHT1, GL_SPECULAR, light_specular);
	glLightfv(GL_LIGHT1, GL_POSITION, light_position1);

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_LIGHT1);


	//	glShadeModel(GL_FLAT);//GL_SMOOTH);
	glShadeModel(GL_SMOOTH);

	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LESS);

	glClearColor(float(0.7),float(0.7),float(0.7),float(0));
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);


	static bool m_textureenabled = true;
	static bool m_textureinitialized = false;


	if(m_textureenabled)
	{
		if(!m_textureinitialized)
		{
			glActiveTexture(GL_TEXTURE0);

			GLubyte*	image=new GLubyte[256*256*3];
			for(int y=0;y<256;++y)
			{
				const int	t=y>>5;
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

			glGenTextures(1,(GLuint*)&m_texturehandle);
			glBindTexture(GL_TEXTURE_2D,m_texturehandle);
			glTexEnvf(GL_TEXTURE_ENV,GL_TEXTURE_ENV_MODE,GL_MODULATE);
			glTexParameterf(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR_MIPMAP_LINEAR);
			glTexParameterf(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
			glTexParameterf(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_REPEAT);
			glTexParameterf(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_REPEAT);
			gluBuild2DMipmaps(GL_TEXTURE_2D,3,256,256,GL_RGB,GL_UNSIGNED_BYTE,image);
			delete[] image;
			m_textureinitialized=true;
		}
		//		glMatrixMode(GL_TEXTURE);
		//		glLoadIdentity();
		//		glMatrixMode(GL_MODELVIEW);

		glEnable(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D,m_texturehandle);

	} else
	{
		glDisable(GL_TEXTURE_2D);
	}

	glEnable(GL_COLOR_MATERIAL);
	 
	err = glGetError();
	assert(err==GL_NO_ERROR);

	//	  glEnable(GL_CULL_FACE);
	//	  glCullFace(GL_BACK);
}

void updateCamera() 
{


	
	btVector3 m_cameraUp(0,1,0);
	int m_forwardAxis=2;
	

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();


	//m_azi+=0.0f;

	btScalar rele = m_ele * btScalar(0.01745329251994329547);// rads per deg
	btScalar razi = m_azi * btScalar(0.01745329251994329547);// rads per deg


		btQuaternion rot(m_cameraUp,razi);


	btVector3 eyePos(0,0,0);
	eyePos[m_forwardAxis] = -m_cameraDistance;

	btVector3 forward(eyePos[0],eyePos[1],eyePos[2]);
	if (forward.length2() < SIMD_EPSILON)
	{
		forward.setValue(1.f,0.f,0.f);
	}
	btVector3 right = m_cameraUp.cross(forward);
	btQuaternion roll(right,-rele);

	eyePos = btMatrix3x3(rot) * btMatrix3x3(roll) * eyePos;

	m_cameraPosition[0] = eyePos.getX();
	m_cameraPosition[1] = eyePos.getY();
	m_cameraPosition[2] = eyePos.getZ();
	m_cameraPosition += m_cameraTargetPosition;


	float m_frustumZNear=1;
	float m_frustumZFar=1000;

	if (m_glutScreenWidth == 0 && m_glutScreenHeight == 0)
		return;

	float aspect;
	btVector3 extents;

	if (m_glutScreenWidth > m_glutScreenHeight) 
	{
		aspect = m_glutScreenWidth / (float)m_glutScreenHeight;
		extents.setValue(aspect * 1.0f, 1.0f,0);
	} else 
	{
		aspect = m_glutScreenHeight / (float)m_glutScreenWidth;
		extents.setValue(1.0f, aspect*1.f,0);
	}


	if (m_ortho)
	{
		// reset matrix
		glLoadIdentity();
		extents *= m_cameraDistance;
		btVector3 lower = m_cameraTargetPosition - extents;
		btVector3 upper = m_cameraTargetPosition + extents;
		glOrtho(lower.getX(), upper.getX(), lower.getY(), upper.getY(),-1000,1000);

		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
	} else
	{
		if (m_glutScreenWidth > m_glutScreenHeight) 
		{
			glFrustum (-aspect * m_frustumZNear, aspect * m_frustumZNear, -m_frustumZNear, m_frustumZNear, m_frustumZNear, m_frustumZFar);
		} else 
		{
			glFrustum (-aspect * m_frustumZNear, aspect * m_frustumZNear, -m_frustumZNear, m_frustumZNear, m_frustumZNear, m_frustumZFar);
		}
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		gluLookAt(m_cameraPosition[0], m_cameraPosition[1], m_cameraPosition[2], 
			m_cameraTargetPosition[0], m_cameraTargetPosition[1], m_cameraTargetPosition[2], 
			m_cameraUp.getX(),m_cameraUp.getY(),m_cameraUp.getZ());
	}

}


void GLInstancingRenderer::RenderScene(void)
{
	 BT_PROFILE("GlutDisplayFunc");

	myinit();

	updateCamera();

	//render coordinate system
	glBegin(GL_LINES);
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

	//do a finish, to make sure timings are clean
	//	glFinish();



	//	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ARRAY_BUFFER, cube_vbo);
	glFlush();

	//updatePos();

//	simulationLoop();

	//useCPU = true;

	int totalNumInstances = 0;

	for (int i=0;i<m_graphicsInstances.size();i++)
	{
		totalNumInstances+=m_graphicsInstances[i]->m_numGraphicsInstances;
	}

	int curOffset = 0;

	for (int i=0;i<m_graphicsInstances.size();i++)
	{
		
		btGraphicsInstance* gfxObj = m_graphicsInstances[i];
		int myOffset = gfxObj->m_instanceOffset*4*sizeof(float);

		int POSITION_BUFFER_SIZE = (totalNumInstances*sizeof(float)*4);
		int ORIENTATION_BUFFER_SIZE = (totalNumInstances*sizeof(float)*4);
		int COLOR_BUFFER_SIZE = (totalNumInstances*sizeof(float)*4);
		int SCALE_BUFFER_SIZE = (totalNumInstances*sizeof(float)*3);

		glBindVertexArray(gfxObj->m_cube_vao);

		
		int vertexStride = 9*sizeof(float);
		int vertexBase = gfxObj->m_vertexArrayOffset*vertexStride;

		glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 9*sizeof(float), (GLvoid*)vertexBase);
		glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 0, (GLvoid *)(curOffset*4*sizeof(float)+SHAPE_BUFFER_SIZE));
		glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, 0, (GLvoid *)(curOffset*4*sizeof(float)+SHAPE_BUFFER_SIZE+POSITION_BUFFER_SIZE));
		int uvoffset = 7*sizeof(float)+vertexBase;
		int normaloffset = 4*sizeof(float)+vertexBase;

		glVertexAttribPointer(3, 2, GL_FLOAT, GL_FALSE, 9*sizeof(float), (GLvoid *)uvoffset);
		glVertexAttribPointer(4, 3, GL_FLOAT, GL_FALSE, 9*sizeof(float), (GLvoid *)normaloffset);
		glVertexAttribPointer(5, 4, GL_FLOAT, GL_FALSE, 0, (GLvoid *)(curOffset*4*sizeof(float)+SHAPE_BUFFER_SIZE+POSITION_BUFFER_SIZE+ORIENTATION_BUFFER_SIZE));
		glVertexAttribPointer(6, 3, GL_FLOAT, GL_FALSE, 0, (GLvoid *)(curOffset*3*sizeof(float)+SHAPE_BUFFER_SIZE+POSITION_BUFFER_SIZE+ORIENTATION_BUFFER_SIZE+COLOR_BUFFER_SIZE));

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
	
		glUseProgram(instancingShader);
		glUniform1f(angle_loc, 0);
		GLfloat pm[16];
		glGetFloatv(GL_PROJECTION_MATRIX, pm);
		glUniformMatrix4fv(ProjectionMatrix, 1, false, &pm[0]);

		GLfloat mvm[16];
		glGetFloatv(GL_MODELVIEW_MATRIX, mvm);
		glUniformMatrix4fv(ModelViewMatrix, 1, false, &mvm[0]);

		glUniform1i(uniform_texture_diffuse, 0);

		glFlush();

		if (gfxObj->m_numGraphicsInstances)
		{
			int indexCount = gfxObj->m_numIndices;
			int indexOffset = 0;

			glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, gfxObj->m_index_vbo);
			{
				BT_PROFILE("glDrawElementsInstanced");
				glDrawElementsInstanced(GL_TRIANGLES, indexCount, GL_UNSIGNED_INT, (void*)indexOffset, gfxObj->m_numGraphicsInstances);
			}
		}
		curOffset+= gfxObj->m_numGraphicsInstances;
	}
	glUseProgram(0);
	glBindBuffer(GL_ARRAY_BUFFER,0);
	glBindVertexArray(0);

	
	GLint err = glGetError();
	assert(err==GL_NO_ERROR);
}


void GLInstancingRenderer::CleanupShaders()
{
	
	delete []m_data->m_instance_positions_ptr;
	delete []m_data->m_instance_quaternion_ptr;
	delete []m_data->m_instance_colors_ptr;
	delete []m_data->m_instance_scale_ptr;
}