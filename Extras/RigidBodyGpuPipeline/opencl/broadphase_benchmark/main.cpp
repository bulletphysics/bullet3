
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

//starts crashing when more than 32700 objects on my Geforce 260, unless _USE_SUB_DATA is defined (still unstable though)
//runs fine with fewer objects

#define NUM_OBJECTS_X 42
//327
#define NUM_OBJECTS_Y 42
#define NUM_OBJECTS_Z 42
//#define NUM_OBJECTS_Z 20

//#define _USE_SUB_DATA

//#define NUM_OBJECTS_X 100
//#define NUM_OBJECTS_Y 100
//#define NUM_OBJECTS_Z 100

///RECREATE_CL_AND_SHADERS_ON_RESIZE will delete and re-create OpenCL and GLSL shaders/buffers at each resize
//#define RECREATE_CL_AND_SHADERS_ON_RESIZE

///
/// OpenCL - OpenGL interop example. Updating transforms of many cubes on GPU, without going through main memory/using the PCIe bus
/// Create all OpenGL resources AFTER create OpenCL context!
///


#include <GL/glew.h>
#include <stdio.h>

#include "btGlutInclude.h"
#include "../opengl_interop/btStopwatch.h"


#include "LinearMath/btVector3.h"
#include "LinearMath/btQuaternion.h"
#include "LinearMath/btMatrix3x3.h"
static float sAngle(0);

#include <assert.h>

#ifdef _WIN32
#include <windows.h>
#endif

#include <stdlib.h>
#include <math.h>
#include "../3dGridBroadphase/Shared/btGpu3DGridBroadphase.h"
#include "../3dGridBroadphase/Shared/bt3dGridBroadphaseOCL.h"
#include "btGridBroadphaseCl.h"

#define USE_NEW
#ifdef USE_NEW
btGridBroadphaseCl* sBroadphase=0;
#else
btGpu3DGridBroadphase* sBroadphase=0;
#endif

btAlignedObjectArray<btBroadphaseProxy*> proxyArray;


#define RS_SCALE (1.0 / (1.0 + RAND_MAX))


int randbiased (double x) {
    for (;;) {
        double p = rand () * RS_SCALE;
        if (p >= x) return 0;
        if (p+RS_SCALE <= x) return 1;
        /* p < x < p+RS_SCALE */
        x = (x - p) * (1.0 + RAND_MAX);
    }
}

size_t randrange (size_t n) 
{
    double xhi;
    double resolution = n * RS_SCALE;
    double x = resolution * rand (); /* x in [0,n) */
    size_t lo = (size_t) floor (x);

    xhi = x + resolution;

    for (;;) {
        lo++;
        if (lo >= xhi || randbiased ((lo - x) / (xhi - x))) return lo-1;
        x = lo;
    }
}

//OpenCL stuff
#include "../basic_initialize/btOpenCLUtils.h"
#include "../opengl_interop/btOpenCLGLInteropBuffer.h"
#include "findPairsOpenCL.h"

btFindPairsIO gFpIO;

cl_context			g_cxMainContext;
cl_command_queue	g_cqCommandQue;
cl_device_id		g_device;
static const size_t workGroupSize = 64;
cl_mem				gLinVelMem;
cl_mem				gAngVelMem;
cl_mem				gBodyTimes;

btVector3 m_cameraPosition(142,220,142);
btVector3 m_cameraTargetPosition(0,-30,0);
btScalar m_cameraDistance = 200;
btVector3 m_cameraUp(0,1,0);
float m_azi=-50.f;
float m_ele=0.f;




btOpenCLGLInteropBuffer* g_interopBuffer = 0;
cl_kernel g_sineWaveKernel;



////for Adl
#include <Adl/Adl.h>

adl::DeviceCL* g_deviceCL=0;



bool useCPU = false;
bool printStats = false;
bool runOpenCLKernels = true;

#define MSTRINGIFY(A) #A
static char* interopKernelString = 
#include "integrateKernel.cl"


btStopwatch gStopwatch;
int m_glutScreenWidth = 640;
int m_glutScreenHeight= 480;

bool m_ortho = false;

static GLuint               instancingShader;        // The instancing renderer
static GLuint               cube_vao;
static GLuint               cube_vbo;
static GLuint               index_vbo;
static GLuint				m_texturehandle;

static bool                 done = false;
static GLint                angle_loc = 0;
static GLint ModelViewMatrix;
static GLint ProjectionMatrix;

void writeTransforms();

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
"	ambient = vec3(0.2,0.2,0.2);\n"
"		\n"
"		\n"
"	vec4 local_normal = (quatRotate3( vertexnormal,q));\n"
"	vec3 light_pos = vec3(10000,10000,10000);\n"
"	normal = normalize(ModelViewMatrix * local_normal).xyz;\n"
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
"	intensity = max(dot(lightDir,normalize(normal)),0.5);\n"
"	cf = intensity*vec3(1.0,1.0,1.0);//intensity * (gl_FrontMaterial.diffuse).rgb+ambient;//gl_FrontMaterial.ambient.rgb;\n"
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


////////////////////////////////////////////////////////////////
// Load the shader from the specified file. Returns false if the
// shader could not be loaded
bool gltLoadShaderFile(const char *szFile, GLuint shader)
{
	GLint shaderLength = 0;
	FILE *fp;

	// Open the shader file
	fp = fopen(szFile, "r");
	if(fp != NULL)
	{
		// See how long the file is
		while (fgetc(fp) != EOF)
			shaderLength++;

		// Allocate a block of memory to send in the shader
		assert(shaderLength < MAX_SHADER_LENGTH);   // make me bigger!
		if(shaderLength > MAX_SHADER_LENGTH)
		{
			fclose(fp);
			return false;
		}

		// Go back to beginning of file
		rewind(fp);

		// Read the whole file in
		if (shaderText != NULL)
			fread(shaderText, 1, shaderLength, fp);

		// Make sure it is null terminated and close the file
		shaderText[shaderLength] = '\0';
		fclose(fp);
	}
	else
		return false;    

	//	printf(shaderText);
	// Load the string
	gltLoadShaderSrc((const char *)shaderText, shader);

	return true;
}   


/////////////////////////////////////////////////////////////////
// Load a pair of shaders, compile, and link together. Specify the complete
// file path for each shader. Note, there is no support for
// just loading say a vertex program... you have to do both.
GLuint gltLoadShaderPair(const char *szVertexProg, const char *szFragmentProg, bool loadFromFile)
{
	// Temporary Shader objects
	GLuint hVertexShader;
	GLuint hFragmentShader; 
	GLuint hReturn = 0;   
	GLint testVal;

	// Create shader objects
	hVertexShader = glCreateShader(GL_VERTEX_SHADER);
	hFragmentShader = glCreateShader(GL_FRAGMENT_SHADER);

	if (loadFromFile)
	{

		if(gltLoadShaderFile(szVertexProg, hVertexShader) == false)
		{
			glDeleteShader(hVertexShader);
			glDeleteShader(hFragmentShader);
			return (GLuint)NULL;
		}

		if(gltLoadShaderFile(szFragmentProg, hFragmentShader) == false)
		{
			glDeleteShader(hVertexShader);
			glDeleteShader(hFragmentShader);
			return (GLuint)NULL;
		}
	} else
	{
		gltLoadShaderSrc(vertexShader, hVertexShader);
		gltLoadShaderSrc(fragmentShader, hFragmentShader);
	}
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

///position xyz, unused w, normal, uv
static const GLfloat cube_vertices[] =
{
	-1.0f, -1.0f, 1.0f, 0.0f,	0,0,1,	0,0,//0
	1.0f, -1.0f, 1.0f, 0.0f,	0,0,1,	1,0,//1
	1.0f,  1.0f, 1.0f, 0.0f,	0,0,1,	1,1,//2
	-1.0f,  1.0f, 1.0f, 0.0f,	0,0,1,	0,1	,//3

	-1.0f, -1.0f, -1.0f, 1.0f,	0,0,-1,	0,0,//4
	1.0f, -1.0f, -1.0f, 1.0f,	0,0,-1,	1,0,//5
	1.0f,  1.0f, -1.0f, 1.0f,	0,0,-1,	1,1,//6
	-1.0f,  1.0f, -1.0f, 1.0f,	0,0,-1,	0,1,//7

	-1.0f, -1.0f, -1.0f, 1.0f,	-1,0,0,	0,0,
	-1.0f, 1.0f, -1.0f, 1.0f,	-1,0,0,	1,0,
	-1.0f,  1.0f, 1.0f, 1.0f,	-1,0,0,	1,1,
	-1.0f,  -1.0f, 1.0f, 1.0f,	-1,0,0,	0,1,

	1.0f, -1.0f, -1.0f, 1.0f,	1,0,0,	0,0,
	1.0f, 1.0f, -1.0f, 1.0f,	1,0,0,	1,0,
	1.0f,  1.0f, 1.0f, 1.0f,	1,0,0,	1,1,
	1.0f,  -1.0f, 1.0f, 1.0f,	1,0,0,	0,1,

	-1.0f, -1.0f,  -1.0f, 1.0f,	0,-1,0,	0,0,
	-1.0f, -1.0f, 1.0f, 1.0f,	0,-1,0,	1,0,
	1.0f, -1.0f,  1.0f, 1.0f,	0,-1,0,	1,1,
	1.0f,-1.0f,  -1.0f,  1.0f,	0,-1,0,	0,1,

	-1.0f, 1.0f,  -1.0f, 1.0f,	0,1,0,	0,0,
	-1.0f, 1.0f, 1.0f, 1.0f,	0,1,0,	1,0,
	1.0f, 1.0f,  1.0f, 1.0f,	0,1,0,	1,1,
	1.0f,1.0f,  -1.0f,  1.0f,	0,1,0,	0,1,
};

static const int cube_indices[]=
{
	0,1,2,0,2,3,//ground face
	4,5,6,4,6,7,//top face
	8,9,10,8,10,11,
	12,13,14,12,14,15,
	16,17,18,16,18,19,
	20,21,22,20,22,23
};

int m_mouseOldX = -1;
int m_mouseOldY = -1;
int m_mouseButtons = 0;


void mouseFunc(int button, int state, int x, int y)
{
	if (state == 0) 
	{
        m_mouseButtons |= 1<<button;
    } else
	{
        m_mouseButtons = 0;
    }

	m_mouseOldX = x;
    m_mouseOldY = y;
}


btVector3	getRayTo(int x,int y)
{

	

	if (m_ortho)
	{

		btScalar aspect;
		btVector3 extents;
		aspect = m_glutScreenWidth / (btScalar)m_glutScreenHeight;
		extents.setValue(aspect * 1.0f, 1.0f,0);
		
		extents *= m_cameraDistance;
		btVector3 lower = m_cameraTargetPosition - extents;
		btVector3 upper = m_cameraTargetPosition + extents;

		btScalar u = x / btScalar(m_glutScreenWidth);
		btScalar v = (m_glutScreenHeight - y) / btScalar(m_glutScreenHeight);
		
		btVector3	p(0,0,0);
		p.setValue((1.0f - u) * lower.getX() + u * upper.getX(),(1.0f - v) * lower.getY() + v * upper.getY(),m_cameraTargetPosition.getZ());
		return p;
	}

	float top = 1.f;
	float bottom = -1.f;
	float nearPlane = 1.f;
	float tanFov = (top-bottom)*0.5f / nearPlane;
	float fov = btScalar(2.0) * btAtan(tanFov);

	btVector3	rayFrom = m_cameraPosition;
	btVector3 rayForward = (m_cameraTargetPosition-m_cameraPosition);
	rayForward.normalize();
	float farPlane = 10000.f;
	rayForward*= farPlane;

	btVector3 rightOffset;
	btVector3 vertical = m_cameraUp;

	btVector3 hor;
	hor = rayForward.cross(vertical);
	hor.normalize();
	vertical = hor.cross(rayForward);
	vertical.normalize();

	float tanfov = tanf(0.5f*fov);


	hor *= 2.f * farPlane * tanfov;
	vertical *= 2.f * farPlane * tanfov;

	btScalar aspect;
	
	aspect = m_glutScreenWidth / (btScalar)m_glutScreenHeight;
	
	hor*=aspect;


	btVector3 rayToCenter = rayFrom + rayForward;
	btVector3 dHor = hor * 1.f/float(m_glutScreenWidth);
	btVector3 dVert = vertical * 1.f/float(m_glutScreenHeight);


	btVector3 rayTo = rayToCenter - 0.5f * hor + 0.5f * vertical;
	rayTo += btScalar(x) * dHor;
	rayTo -= btScalar(y) * dVert;
	return rayTo;
}


void mouseMotionFunc(int x, int y)
{
	float dx, dy;
    dx = btScalar(x) - m_mouseOldX;
    dy = btScalar(y) - m_mouseOldY;


	///only if ALT key is pressed (Maya style)
	{
		if(m_mouseButtons & 2)
		{
			btVector3 hor = getRayTo(0,0)-getRayTo(1,0);
			btVector3 vert = getRayTo(0,0)-getRayTo(0,1);
			btScalar multiplierX = btScalar(0.001);
			btScalar multiplierY = btScalar(0.001);
			if (m_ortho)
			{
				multiplierX = 1;
				multiplierY = 1;
			}
			m_cameraTargetPosition += hor* dx * multiplierX;
			m_cameraTargetPosition += vert* dy * multiplierY;
		}

		if(m_mouseButtons & (2 << 2) && m_mouseButtons & 1)
		{
		}
		else if(m_mouseButtons & 1) 
		{
			m_azi += dx * btScalar(0.2);
			m_azi = fmodf(m_azi, btScalar(360.f));
			m_ele += dy * btScalar(0.2);
			m_ele = fmodf(m_ele, btScalar(180.f));
		} 
		else if(m_mouseButtons & 4) 
		{
			m_cameraDistance -= dy * btScalar(0.02f);
			if (m_cameraDistance<btScalar(0.1))
				m_cameraDistance = btScalar(0.1);

			
		} 
	}


	m_mouseOldX = x;
    m_mouseOldY = y;

}


void DeleteCL()
{
	clReleaseContext(g_cxMainContext);
	clReleaseCommandQueue(g_cqCommandQue);
}

void InitCL()
{
	void* glCtx=0;
	void* glDC = 0;

#ifdef _WIN32
	glCtx = wglGetCurrentContext();
#else //!_WIN32
	GLXContext glCtx = glXGetCurrentContext();
#endif //!_WIN32
	glDC = wglGetCurrentDC();

	int ciErrNum = 0;
#ifdef CL_PLATFORM_INTEL
	cl_device_type deviceType = CL_DEVICE_TYPE_ALL;
#else
	cl_device_type deviceType = CL_DEVICE_TYPE_GPU;
#endif


	g_cxMainContext = btOpenCLUtils::createContextFromType(deviceType, &ciErrNum, glCtx, glDC);
	oclCHECKERROR(ciErrNum, CL_SUCCESS);

	int numDev = btOpenCLUtils::getNumDevices(g_cxMainContext);

	if (numDev>0)
	{
		g_device= btOpenCLUtils::getDevice(g_cxMainContext,0);
		btOpenCLDeviceInfo clInfo;
		btOpenCLUtils::getDeviceInfo(g_device,clInfo);
		btOpenCLUtils::printDeviceInfo(g_device);
		// create a command-queue
		g_cqCommandQue = clCreateCommandQueue(g_cxMainContext, g_device, 0, &ciErrNum);
		oclCHECKERROR(ciErrNum, CL_SUCCESS);
		//normally you would create and execute kernels using this command queue

	}


}

#define NUM_OBJECTS (NUM_OBJECTS_X*NUM_OBJECTS_Y*NUM_OBJECTS_Z)
#define POSITION_BUFFER_SIZE (NUM_OBJECTS*sizeof(float)*4)
#define ORIENTATION_BUFFER_SIZE (NUM_OBJECTS*sizeof(float)*4)
#define COLOR_BUFFER_SIZE (NUM_OBJECTS*sizeof(float)*4)
#define SCALE_BUFFER_SIZE (NUM_OBJECTS*sizeof(float)*3)


GLfloat* instance_positions_ptr = 0;
GLfloat* instance_quaternion_ptr = 0;
GLfloat* instance_colors_ptr = 0;
GLfloat* instance_scale_ptr= 0;


void DeleteShaders()
{
	glDeleteVertexArrays(1, &cube_vao);
	glDeleteBuffers(1,&index_vbo);
	glDeleteBuffers(1,&cube_vbo);
	glDeleteProgram(instancingShader);
}


void InitShaders()
{
	
	btOverlappingPairCache* overlappingPairCache=0;
#ifdef	USE_NEW
	sBroadphase = new btGridBroadphaseCl(overlappingPairCache,btVector3(3.f, 3.f, 3.f), 32, 32, 32,NUM_OBJECTS, NUM_OBJECTS, 64, 100.f, 16,
		g_cxMainContext ,g_device,g_cqCommandQue);
#else
	sBroadphase = new btGpu3DGridBroadphase(btVector3(10.f, 10.f, 10.f), 32, 32, 32,NUM_OBJECTS, NUM_OBJECTS, 64, 100.f, 16);
#endif



//	sBroadphase = new bt3dGridBroadphaseOCL(overlappingPairCache,btVector3(10.f, 10.f, 10.f), 32, 32, 32,NUM_OBJECTS, NUM_OBJECTS, 64, 100.f, 16,
//		g_cxMainContext ,g_device,g_cqCommandQue);



	bool loadFromFile = false;
	instancingShader = gltLoadShaderPair("instancing.vs","instancing.fs", loadFromFile);

	glLinkProgram(instancingShader);
	glUseProgram(instancingShader);
	angle_loc = glGetUniformLocation(instancingShader, "angle");
	ModelViewMatrix = glGetUniformLocation(instancingShader, "ModelViewMatrix");
	ProjectionMatrix = glGetUniformLocation(instancingShader, "ProjectionMatrix");
	uniform_texture_diffuse = glGetUniformLocation(instancingShader, "Diffuse");

	GLuint offset = 0;


	glGenBuffers(1, &cube_vbo);
	glBindBuffer(GL_ARRAY_BUFFER, cube_vbo);

	instance_positions_ptr = (GLfloat*)new float[NUM_OBJECTS*4];
	instance_quaternion_ptr = (GLfloat*)new float[NUM_OBJECTS*4];
	instance_colors_ptr = (GLfloat*)new float[NUM_OBJECTS*4];
	instance_scale_ptr = (GLfloat*)new float[NUM_OBJECTS*3];

	int index=0;
	for (int i=0;i<NUM_OBJECTS_X;i++)
	{
		for (int j=0;j<NUM_OBJECTS_Y;j++)
		{
			for (int k=0;k<NUM_OBJECTS_Z;k++)
			{
				instance_positions_ptr[index*4]=-(i-NUM_OBJECTS_X/2)*10;
				instance_positions_ptr[index*4+1]=-(j-NUM_OBJECTS_Y/2)*10;
				instance_positions_ptr[index*4+2]=-(k-NUM_OBJECTS_Z/2)*10;
				instance_positions_ptr[index*4+3]=1;

				int shapeType =0;
				void* userPtr = 0;
				btVector3 aabbMin(
					instance_positions_ptr[index*4],
					instance_positions_ptr[index*4+1],
					instance_positions_ptr[index*4+2]);
				btVector3 aabbMax = aabbMin;
				aabbMin -= btVector3(1,1,1);
				aabbMax += btVector3(1,1,1);

				void* myptr = (void*)index;//0;//&mBoxes[i]
				btBroadphaseProxy* proxy = sBroadphase->createProxy(aabbMin,aabbMax,shapeType,myptr,1,1,0,0);//m_dispatcher);
				proxyArray.push_back(proxy);

				instance_quaternion_ptr[index*4]=0;
				instance_quaternion_ptr[index*4+1]=0;
				instance_quaternion_ptr[index*4+2]=0;
				instance_quaternion_ptr[index*4+3]=1;

				instance_colors_ptr[index*4]=j<NUM_OBJECTS_Y/2? 0.5f : 1.f;
				instance_colors_ptr[index*4+1]=k<NUM_OBJECTS_Y/2? 0.5f : 1.f;
				instance_colors_ptr[index*4+2]=i<NUM_OBJECTS_Y/2? 0.5f : 1.f;
				instance_colors_ptr[index*4+3]=1.f;

				instance_scale_ptr[index*3] = 1;
				instance_scale_ptr[index*3+1] = 1;
				instance_scale_ptr[index*3+2] = 1;
				

				index++;
			}
		}
	}

	int size = sizeof(cube_vertices)  + POSITION_BUFFER_SIZE+ORIENTATION_BUFFER_SIZE+COLOR_BUFFER_SIZE+SCALE_BUFFER_SIZE;

	char* bla = (char*)malloc(size);
	int szc = sizeof(cube_vertices);
	memcpy(bla,&cube_vertices[0],szc);
	memcpy(bla+sizeof(cube_vertices),instance_positions_ptr,POSITION_BUFFER_SIZE);
	memcpy(bla+sizeof(cube_vertices)+POSITION_BUFFER_SIZE,instance_quaternion_ptr,ORIENTATION_BUFFER_SIZE);
	memcpy(bla+sizeof(cube_vertices)+POSITION_BUFFER_SIZE+ORIENTATION_BUFFER_SIZE,instance_colors_ptr, COLOR_BUFFER_SIZE);
	memcpy(bla+sizeof(cube_vertices)+POSITION_BUFFER_SIZE+ORIENTATION_BUFFER_SIZE+COLOR_BUFFER_SIZE,instance_scale_ptr, SCALE_BUFFER_SIZE);

	glBufferData(GL_ARRAY_BUFFER, size, bla, GL_DYNAMIC_DRAW);//GL_STATIC_DRAW);

	///initialize parts of the buffer
#ifdef _USE_SUB_DATA
	glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(cube_vertices)+ 16384, bla);//cube_vertices);
#endif

	char* dest=  (char*)glMapBuffer( GL_ARRAY_BUFFER,GL_WRITE_ONLY);//GL_WRITE_ONLY
	memcpy(dest,cube_vertices,sizeof(cube_vertices));
	//memcpy(dest+sizeof(cube_vertices),instance_colors,sizeof(instance_colors));
	glUnmapBuffer( GL_ARRAY_BUFFER);



	writeTransforms();

	/*
	glBufferSubData(GL_ARRAY_BUFFER, sizeof(cube_vertices) + sizeof(instance_colors), POSITION_BUFFER_SIZE, instance_positions_ptr);
	glBufferSubData(GL_ARRAY_BUFFER, sizeof(cube_vertices) + sizeof(instance_colors)+POSITION_BUFFER_SIZE,ORIENTATION_BUFFER_SIZE , instance_quaternion_ptr);
	*/

	glGenVertexArrays(1, &cube_vao);
	glBindVertexArray(cube_vao);
	glBindBuffer(GL_ARRAY_BUFFER, cube_vbo);
	glBindVertexArray(0);

	glGenBuffers(1, &index_vbo);
	int indexBufferSize = sizeof(cube_indices);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, index_vbo);

	glBufferData(GL_ELEMENT_ARRAY_BUFFER, indexBufferSize, NULL, GL_STATIC_DRAW);
	glBufferSubData(GL_ELEMENT_ARRAY_BUFFER,0,indexBufferSize,cube_indices);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ARRAY_BUFFER,0);
	glBindVertexArray(0);

}



void updateCamera() {


	
	btVector3 m_cameraUp(0,1,0);
	int m_forwardAxis=2;
	

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

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



void myinit()
{



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
					const int		s=x>>5;
					const GLubyte	b=180;					
					GLubyte			c=b+((s+t&1)&1)*(255-b);
					pi[0]=c;
					pi[1]=c;
					pi[2]=c;
					pi+=3;
				}
			}

			glGenTextures(1,(GLuint*)&m_texturehandle);
			glBindTexture(GL_TEXTURE_2D,m_texturehandle);
			glTexEnvf(GL_TEXTURE_ENV,GL_TEXTURE_ENV_MODE,GL_MODULATE);
			glTexParameterf(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR_MIPMAP_LINEAR);
			glTexParameterf(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR_MIPMAP_LINEAR);
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


	//	  glEnable(GL_CULL_FACE);
	//	  glCullFace(GL_BACK);
}

//#pragma optimize( "g", off )



void writeTransforms()
{


	glFlush();
	char* bla =  (char*)glMapBuffer( GL_ARRAY_BUFFER,GL_READ_WRITE);//GL_WRITE_ONLY

	float* positions = (float*)(bla+sizeof(cube_vertices));
	float* orientations = (float*)(bla+sizeof(cube_vertices) + POSITION_BUFFER_SIZE);
	float* colors= (float*)(bla+sizeof(cube_vertices) + POSITION_BUFFER_SIZE+ORIENTATION_BUFFER_SIZE);
	float* scaling= (float*)(bla+sizeof(cube_vertices) + POSITION_BUFFER_SIZE+ORIENTATION_BUFFER_SIZE+COLOR_BUFFER_SIZE);

	//	positions[0]+=0.001f;

	static int offset=0;
	//offset++;

	static btVector3 axis(1,0,0);
	sAngle += 0.01f;
	int index=0;
	btQuaternion orn(axis,sAngle);
	for (int i=0;i<NUM_OBJECTS_X;i++)
	{
		for (int j=0;j<NUM_OBJECTS_Y;j++)
		{
			for (int k=0;k<NUM_OBJECTS_Z;k++)
			{
				//if (!((index+offset)%15))
				{
					instance_positions_ptr[index*4+1]-=0.01f;
					positions[index*4]=instance_positions_ptr[index*4];
					positions[index*4+1]=instance_positions_ptr[index*4+1];
					positions[index*4+2]=instance_positions_ptr[index*4+2];
					positions[index*4+3]=instance_positions_ptr[index*4+3];

					orientations[index*4] = orn[0];
					orientations[index*4+1] = orn[1];
					orientations[index*4+2] = orn[2];
					orientations[index*4+3] = orn[3];
				}
				//				memcpy((void*)&orientations[index*4],orn,sizeof(btQuaternion));
				index++;
			}
		}
	}

	glUnmapBuffer( GL_ARRAY_BUFFER);
	//if this glFinish is removed, the animation is not always working/blocks
	//@todo: figure out why
	glFlush();
}


void updatePos()
{


	if (useCPU)
	{
#if 0
		int index=0;
		for (int i=0;i<NUM_OBJECTS_X;i++)
		{
			for (int j=0;j<NUM_OBJECTS_Y;j++)
			{
				for (int k=0;k<NUM_OBJECTS_Z;k++)
				{
					//if (!((index+offset)%15))
					{
						instance_positions_ptr[index*4+1]-=0.01f;
					}
				}
			}
		}
		writeTransforms();
#endif

	} 
	else
	{

		glFinish();

		cl_mem clBuffer = g_interopBuffer->getCLBUffer();
		cl_int ciErrNum = CL_SUCCESS;
		ciErrNum = clEnqueueAcquireGLObjects(g_cqCommandQue, 1, &clBuffer, 0, 0, NULL);
		oclCHECKERROR(ciErrNum, CL_SUCCESS);
		if (runOpenCLKernels)
		{
			int numObjects = NUM_OBJECTS;
			int offset = (sizeof(cube_vertices) )/4;

			ciErrNum = clSetKernelArg(g_sineWaveKernel, 0, sizeof(int), &offset);
			ciErrNum = clSetKernelArg(g_sineWaveKernel, 1, sizeof(int), &numObjects);
			ciErrNum = clSetKernelArg(g_sineWaveKernel, 2, sizeof(cl_mem), (void*)&clBuffer );

			ciErrNum = clSetKernelArg(g_sineWaveKernel, 3, sizeof(cl_mem), (void*)&gLinVelMem);
			ciErrNum = clSetKernelArg(g_sineWaveKernel, 4, sizeof(cl_mem), (void*)&gAngVelMem);
			ciErrNum = clSetKernelArg(g_sineWaveKernel, 5, sizeof(cl_mem), (void*)&gBodyTimes);
			
			
			

		
			size_t	numWorkItems = workGroupSize*((NUM_OBJECTS + (workGroupSize)) / workGroupSize);
			ciErrNum = clEnqueueNDRangeKernel(g_cqCommandQue, g_sineWaveKernel, 1, NULL, &numWorkItems, &workGroupSize,0 ,0 ,0);
			oclCHECKERROR(ciErrNum, CL_SUCCESS);
		}
	
		ciErrNum = clEnqueueReleaseGLObjects(g_cqCommandQue, 1, &clBuffer, 0, 0, 0);
		oclCHECKERROR(ciErrNum, CL_SUCCESS);
		clFinish(g_cqCommandQue);

	}

}


void cpuBroadphase()
{
	glFlush();
	char* bla =  (char*)glMapBuffer( GL_ARRAY_BUFFER,GL_READ_WRITE);//GL_WRITE_ONLY

	float* positions = (float*)(bla+sizeof(cube_vertices));
	float* orientations = (float*)(bla+sizeof(cube_vertices) + POSITION_BUFFER_SIZE);
	float* colors= (float*)(bla+sizeof(cube_vertices) + POSITION_BUFFER_SIZE+ORIENTATION_BUFFER_SIZE);
	float* scaling= (float*)(bla+sizeof(cube_vertices) + POSITION_BUFFER_SIZE+ORIENTATION_BUFFER_SIZE+COLOR_BUFFER_SIZE);

	int index=0;

	for (int i=0;i<NUM_OBJECTS_X;i++)
	{
		for (int j=0;j<NUM_OBJECTS_Y;j++)
		{
			for (int k=0;k<NUM_OBJECTS_Z;k++)
			{
				colors[index*4] = 0.f;
				colors[index*4+1] = 1.f;
				colors[index*4+2] = 0.f;
				colors[index*4+3] = 1.f;


//				instance_positions_ptr[index*4+1]-=0.01f;
				btVector3 aabbMin(
					positions[index*4],
					positions[index*4+1],
					positions[index*4+2]);
				btVector3 aabbMax = aabbMin;
				aabbMin -= btVector3(1,1,1);
				aabbMax += btVector3(1,1,1);

				//sBroadphase->setAabb(proxyArray[index],aabbMin,aabbMax,0);

				index++;
			}
		}
	}

#ifdef USE_NEW
	

#else
	sBroadphase->calculateOverlappingPairs(0);
	int overlap = sBroadphase->getOverlappingPairCache()->getNumOverlappingPairs();
	for (int i=0;i<overlap;i++)
	{
		const btBroadphasePair& pair = sBroadphase->getOverlappingPairCache()->getOverlappingPairArray()[i];
		int indexA = (int)pair.m_pProxy0->m_clientObject;
		int indexB = (int)pair.m_pProxy1->m_clientObject;
				colors[indexA*4] = 1.f;
				colors[indexA*4+1] = 0.f;
				colors[indexA*4+2] = 0.f;
				colors[indexA*4+3] = 1.f;

				colors[indexB*4] = 1.f;
				colors[indexB*4+1] = 0.f;
				colors[indexB*4+2] = 0.f;
				colors[indexB*4+3] = 1.f;
	}
#endif

	//now color the overlap

	

	glUnmapBuffer( GL_ARRAY_BUFFER);
	//if this glFinish is removed, the animation is not always working/blocks
	//@todo: figure out why
	glFlush();
}

void	broadphase()
{
	if (useCPU)
	{
		cpuBroadphase();

	} 
	else
	{

		glFinish();

		cl_mem clBuffer = g_interopBuffer->getCLBUffer();
		cl_int ciErrNum = CL_SUCCESS;
		ciErrNum = clEnqueueAcquireGLObjects(g_cqCommandQue, 1, &clBuffer, 0, 0, NULL);
		oclCHECKERROR(ciErrNum, CL_SUCCESS);
		if (runOpenCLKernels)
		{

			gFpIO.m_numObjects = NUM_OBJECTS;
			gFpIO.m_positionOffset = (sizeof(cube_vertices) )/4;
			gFpIO.m_clObjectsBuffer = clBuffer;
			gFpIO.m_dAABB = sBroadphase->m_dAABB;
			setupGpuAabbsSimple(gFpIO);

			sBroadphase->calculateOverlappingPairs(0, NUM_OBJECTS);


			gFpIO.m_dAllOverlappingPairs = sBroadphase->m_dAllOverlappingPairs;
			gFpIO.m_numOverlap = sBroadphase->m_numPrefixSum;

			colorPairsOpenCL(gFpIO);

		}
	
		ciErrNum = clEnqueueReleaseGLObjects(g_cqCommandQue, 1, &clBuffer, 0, 0, 0);
		oclCHECKERROR(ciErrNum, CL_SUCCESS);
		clFinish(g_cqCommandQue);



	}
}


//#pragma optimize( "g", on )

void RenderScene(void)
{

#if 0
	float modelview[20]={0,1,2,3,4,5,6,7,8,9,0,1,2,3,4,5,6,7,8,9};
	// get the current modelview matrix
	glGetFloatv(GL_MODELVIEW_MATRIX , modelview);
	float projection[20]={0,1,2,3,4,5,6,7,8,9,0,1,2,3,4,5,6,7,8,9};
	glGetFloatv(GL_PROJECTION_MATRIX, projection);
#endif

	myinit();

	updateCamera();

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

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

	float start = gStopwatch.getTimeMilliseconds();

	//	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ARRAY_BUFFER, cube_vbo);
	glFlush();

	updatePos();

	broadphase();

	//useCPU = true;

	float stop = gStopwatch.getTimeMilliseconds();
	gStopwatch.reset();

	if (printStats)
	{
		printf("updatePos=%f ms on ",stop-start);

		if (useCPU)
		{
			printf("CPU \n");
		} else
		{
			printf("OpenCL ");
			if (runOpenCLKernels)
				printf("running the kernels");
			else
				printf("without running the kernels");
			printf("\n");
		}
	}

	glBindVertexArray(cube_vao);

	glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 9*sizeof(float), 0);
	glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 0, (GLvoid *)(sizeof(cube_vertices)));
	glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, 0, (GLvoid *)(sizeof(cube_vertices)+POSITION_BUFFER_SIZE));
	int uvoffset = 7*sizeof(float);
	int normaloffset = 4*sizeof(float);

	glVertexAttribPointer(3, 2, GL_FLOAT, GL_FALSE, 9*sizeof(float), (GLvoid *)uvoffset);
	glVertexAttribPointer(4, 3, GL_FLOAT, GL_FALSE, 9*sizeof(float), (GLvoid *)normaloffset);
	glVertexAttribPointer(5, 4, GL_FLOAT, GL_FALSE, 0, (GLvoid *)(sizeof(cube_vertices)+POSITION_BUFFER_SIZE+ORIENTATION_BUFFER_SIZE));
	glVertexAttribPointer(6, 3, GL_FLOAT, GL_FALSE, 0, (GLvoid *)(sizeof(cube_vertices)+POSITION_BUFFER_SIZE+ORIENTATION_BUFFER_SIZE+COLOR_BUFFER_SIZE));

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
	int numInstances = NUM_OBJECTS;
	int indexCount = sizeof(cube_indices)/sizeof(int);
	int indexOffset = 0;

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, index_vbo);
	glDrawElementsInstanced(GL_TRIANGLES, indexCount, GL_UNSIGNED_INT, (void*)indexOffset, numInstances);

	glUseProgram(0);
	glBindBuffer(GL_ARRAY_BUFFER,0);
	glBindVertexArray(0);

	glutSwapBuffers();
	glutPostRedisplay();

	GLint err = glGetError();
	assert(err==GL_NO_ERROR);
}


void ChangeSize(int w, int h)
{
	m_glutScreenWidth = w;
	m_glutScreenHeight = h;

#ifdef RECREATE_CL_AND_SHADERS_ON_RESIZE
	delete g_interopBuffer;
	clReleaseKernel(g_sineWaveKernel);
	releaseFindPairs(fpio);
	DeleteCL();
	DeleteShaders();
#endif //RECREATE_CL_AND_SHADERS_ON_RESIZE

	// Set Viewport to window dimensions
	glViewport(0, 0, w, h);

#ifdef RECREATE_CL_AND_SHADERS_ON_RESIZE
	InitCL();
	InitShaders();
	
	g_interopBuffer = new btOpenCLGLInteropBuffer(g_cxMainContext,g_cqCommandQue,cube_vbo);
	clFinish(g_cqCommandQue);
	g_sineWaveKernel = btOpenCLUtils::compileCLKernelFromString(g_cxMainContext, interopKernelString, "interopKernel" );
	initFindPairs(...);
#endif //RECREATE_CL_AND_SHADERS_ON_RESIZE

}

void Keyboard(unsigned char key, int x, int y)
{
	switch (key)
	{
	case 27:
		done = true;
		break;
	case 'O':
	case 'o':
		{
			m_ortho = !m_ortho;
			break;
		}
	case 'c':
	case 'C':
		{
			useCPU = !useCPU;
			if (useCPU)
				printf("using CPU\n");
			else
				printf("using OpenCL\n");
			break;
		}
	case 's':
	case 'S':
		{
			printStats = !printStats;
			break;
		}
	case 'k':
	case 'K':
		{
			runOpenCLKernels=!runOpenCLKernels;
			break;
		}
	case 'q':
	case 'Q':
		exit(0);
	default:
		break;
	}
}

// Cleanup
void ShutdownRC(void)
{
	glDeleteBuffers(1, &cube_vbo);
	glDeleteVertexArrays(1, &cube_vao);
}

int main(int argc, char* argv[])
{
	srand(0);
	//	printf("vertexShader = \n%s\n",vertexShader);
	//	printf("fragmentShader = \n%s\n",fragmentShader);

	glutInit(&argc, argv);

	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA);


	glutInitWindowSize(m_glutScreenWidth, m_glutScreenHeight);
	char buf[1024];
	sprintf(buf,"OpenCL broadphase benchmark, %d cubes on the GPU", NUM_OBJECTS);
	glutCreateWindow(buf);

	glutReshapeFunc(ChangeSize);

	glutMouseFunc(mouseFunc);
	glutMotionFunc(mouseMotionFunc);

	glutKeyboardFunc(Keyboard);
	glutDisplayFunc(RenderScene);

	GLenum err = glewInit();
	if (GLEW_OK != err)
	{
		/* Problem: glewInit failed, something is seriously wrong. */
		fprintf(stderr, "Error: %s\n", glewGetErrorString(err));
	}

	//ChangeSize(m_glutScreenWidth,m_glutScreenHeight);

	InitCL();
	

#define CUSTOM_CL_INITIALIZATION
#ifdef CUSTOM_CL_INITIALIZATION
	g_deviceCL = new adl::DeviceCL();
	g_deviceCL->m_deviceIdx = g_device;
	g_deviceCL->m_context = g_cxMainContext;
	g_deviceCL->m_commandQueue = g_cqCommandQue;

#else
	DeviceUtils::Config cfg;
	cfg.m_type = DeviceUtils::Config::DEVICE_CPU;
	g_deviceCL = DeviceUtils::allocate( TYPE_CL, cfg );
#endif

	int size = NUM_OBJECTS;
	adl::Buffer<btVector3> linvelBuf( g_deviceCL, size );
	adl::Buffer<btVector3> angvelBuf( g_deviceCL, size );
	adl::Buffer<float>		bodyTimes(g_deviceCL,size);

	gLinVelMem = (cl_mem)linvelBuf.m_ptr;
	gAngVelMem = (cl_mem)angvelBuf.m_ptr;
	gBodyTimes = (cl_mem)bodyTimes.m_ptr;

	btVector3* linVelHost= new btVector3[size];
	btVector3* angVelHost = new btVector3[size];
	float*		bodyTimesHost = new float[size];

	for (int i=0;i<NUM_OBJECTS;i++)
	{
		linVelHost[i].setValue(0,0,0);
		angVelHost[i].setValue(1,0,0);
		bodyTimesHost[i] = i*(1024.f/NUM_OBJECTS);//double(randrange(0x2fffffff))/double(0x2fffffff)*float(1024);//NUM_OBJECTS);
	}

	linvelBuf.write(linVelHost,NUM_OBJECTS);
	angvelBuf.write(angVelHost,NUM_OBJECTS);
	bodyTimes.write(bodyTimesHost,NUM_OBJECTS);

	adl::DeviceUtils::waitForCompletion( g_deviceCL );


	InitShaders();
	
	g_interopBuffer = new btOpenCLGLInteropBuffer(g_cxMainContext,g_cqCommandQue,cube_vbo);
	clFinish(g_cqCommandQue);


	g_sineWaveKernel = btOpenCLUtils::compileCLKernelFromString(g_cxMainContext, g_device,interopKernelString, "sineWaveKernel" );
	initFindPairs(gFpIO, g_cxMainContext, g_device, g_cqCommandQue, NUM_OBJECTS);

	


	glutMainLoop();
	ShutdownRC();

	return 0;
}
