
#include "GpuSoftBodyDemo.h"
#define USE_BARREL_VERTICES
#include "OpenGLWindow/ShapeData.h"
#include "OpenGLWindow/GLInstancingRenderer.h"
#include "Bullet3Common/b3Quaternion.h"
#include "OpenGLWindow/b3gWindowInterface.h"
#include "Bullet3OpenCL/BroadphaseCollision/b3GpuSapBroadphase.h"
#include "../GpuDemoInternalData.h"
#include "Bullet3OpenCL/Initialize/b3OpenCLUtils.h"
#include "OpenGLWindow/OpenGLInclude.h"
#include "OpenGLWindow/GLInstanceRendererInternalData.h"
#include "Bullet3OpenCL/ParallelPrimitives/b3LauncherCL.h"
#include "Bullet3OpenCL/RigidBody/b3GpuRigidBodyPipeline.h"
#include "Bullet3OpenCL/RigidBody/b3GpuNarrowPhase.h"
#include "Bullet3Collision/NarrowPhaseCollision/b3Config.h"
#include "GpuSoftBodyDemoInternalData.h"
#include "Bullet3Collision/BroadPhaseCollision/b3DynamicBvhBroadphase.h"


static b3KeyboardCallback oldCallback = 0;
extern bool gReset;

#define MSTRINGIFY(A) #A

static const char* s_rigidBodyKernelString = MSTRINGIFY(

typedef struct
{
	float4 m_pos;
	float4 m_quat;
	float4 m_linVel;
	float4 m_angVel;
	unsigned int m_collidableIdx;
	float m_invMass;
	float m_restituitionCoeff;
	float m_frictionCoeff;
} Body;

__kernel void 
	copyTransformsToVBOKernel( __global Body* gBodies, __global float4* posOrnColor, const int numNodes)
{
	int nodeID = get_global_id(0);
	if( nodeID < numNodes )
	{
		posOrnColor[nodeID] = (float4) (gBodies[nodeID].m_pos.xyz,1.0);
		posOrnColor[nodeID + numNodes] = gBodies[nodeID].m_quat;
	}
}
);





GpuSoftBodyDemo::GpuSoftBodyDemo()
:m_instancingRenderer(0),
m_window(0)
{
	m_data = new GpuSoftBodyDemoInternalData;
}
GpuSoftBodyDemo::~GpuSoftBodyDemo()
{
	
	delete m_data;
}







static void PairKeyboardCallback(int key, int state)
{
	if (key=='R' && state)
	{
		gReset = true;
	}
	
	//b3DefaultKeyboardCallback(key,state);
	oldCallback(key,state);
}

void	GpuSoftBodyDemo::setupScene(const ConstructionInfo& ci)
{

}

void	GpuSoftBodyDemo::initPhysics(const ConstructionInfo& ci)
{

	GLint err = glGetError();
	assert(err==GL_NO_ERROR);

	if (ci.m_window)
	{
		m_window = ci.m_window;
		oldCallback = ci.m_window->getKeyboardCallback();
		ci.m_window->setKeyboardCallback(PairKeyboardCallback);

	}

	m_instancingRenderer = ci.m_instancingRenderer;

	initCL(ci.preferredOpenCLDeviceIndex,ci.preferredOpenCLPlatformIndex);

	if (m_clData->m_clContext)
	{
		int errNum=0;

		cl_program rbProg=0;
		m_data->m_copyTransformsToVBOKernel = b3OpenCLUtils::compileCLKernelFromString(m_clData->m_clContext,m_clData->m_clDevice,s_rigidBodyKernelString,"copyTransformsToVBOKernel",&errNum,rbProg);
		
		b3Config config;
		b3GpuNarrowPhase* np = new b3GpuNarrowPhase(m_clData->m_clContext,m_clData->m_clDevice,m_clData->m_clQueue,config);
		b3GpuSapBroadphase* bp = new b3GpuSapBroadphase(m_clData->m_clContext,m_clData->m_clDevice,m_clData->m_clQueue);
		m_data->m_np = np;
		m_data->m_bp = bp;
		b3DynamicBvhBroadphase* broadphaseDbvt = new b3DynamicBvhBroadphase(config.m_maxConvexBodies);

		m_data->m_rigidBodyPipeline = new b3GpuRigidBodyPipeline(m_clData->m_clContext,m_clData->m_clDevice,m_clData->m_clQueue, np, bp,broadphaseDbvt,config);

		err = glGetError();
		assert(err==GL_NO_ERROR);


		setupScene(ci);

		err = glGetError();
		assert(err==GL_NO_ERROR);

		m_data->m_rigidBodyPipeline->writeAllInstancesToGpu();
		np->writeAllBodiesToGpu();
		bp->writeAabbsToGpu();

	}


	m_instancingRenderer->writeTransforms();
	
	

}

void	GpuSoftBodyDemo::exitPhysics()
{
	delete m_data->m_instancePosOrnColor;
	delete m_data->m_rigidBodyPipeline;

	m_window->setKeyboardCallback(oldCallback);
	
	delete m_data->m_np;
	m_data->m_np = 0;
	delete m_data->m_bp;
	m_data->m_bp = 0;

	exitCL();
}

	

struct GraphicsVertex
{
	float pos[4];
	float normal[3];
	float texcoord[2];
};

void GpuSoftClothDemo::renderScene()
{
	if (m_data->m_clothShapeIndex>=0 && m_data->m_clothVertices)
	{
		GraphicsVertex* cpu_buffer = (GraphicsVertex*)m_data->m_clothVertices;
		int width = 256;
		int height=256;
		static float shift = 0.f;
		shift+=0.01;
		if (shift>B3_2_PI)
			shift-=B3_2_PI;

		int numVertices = 0;
		// Initial test data for rendering
		for(int y = 0; y < height; y++)
		{
			for(int x = 0; x < width; x++)
			{
				
				double coord = b3Sin(x/5.0+shift)*0.01;
				//coord = sin(y/);

				cpu_buffer[y*width+x].pos[0]      = (x/((float)(width-1)))*1;
				cpu_buffer[y*width+x].pos[1]      = coord;
				cpu_buffer[y*width+x].pos[2]      = (y/((float)(height-1)))*1; 
				cpu_buffer[y*width+x].pos[3]      = 0.f;

				cpu_buffer[y*width+x].normal[0]   = 1;
				cpu_buffer[y*width+x].normal[1]   = 0;
				cpu_buffer[y*width+x].normal[2]   = 0;
				cpu_buffer[y*width+x].texcoord[0] = 1*x/((float)(width-1));
				cpu_buffer[y*width+x].texcoord[1] = (1.f-4*y/((float)(height-1)));
				numVertices++;
			}
		}

		m_instancingRenderer->updateShape(m_data->m_clothShapeIndex,m_data->m_clothVertices);
	}
	m_instancingRenderer->renderScene();

}
void GpuSoftBodyDemo::renderScene()
{
	m_instancingRenderer->renderScene();
}

void GpuSoftBodyDemo::clientMoveAndDisplay()
{
	GLint err = glGetError();
	assert(err==GL_NO_ERROR);

	bool animate=true;
	int numObjects= m_data->m_rigidBodyPipeline->getNumBodies();
//m_instancingRenderer->getInternalData()->m_totalNumInstances;
	b3Vector4* positions = 0;
	if (animate && numObjects)
	{
		B3_PROFILE("gl2cl");
		
		if (!m_data->m_instancePosOrnColor)
		{
			GLuint vbo = m_instancingRenderer->getInternalData()->m_vbo;
			int arraySizeInBytes  = numObjects * (3)*sizeof(b3Vector4);
			glBindBuffer(GL_ARRAY_BUFFER, vbo);
			cl_bool blocking=  CL_TRUE;
			positions=  (b3Vector4*)glMapBufferRange( GL_ARRAY_BUFFER,m_instancingRenderer->getMaxShapeCapacity(),arraySizeInBytes, GL_MAP_READ_BIT );//GL_READ_WRITE);//GL_WRITE_ONLY
			GLint err = glGetError();
			assert(err==GL_NO_ERROR);
			m_data->m_instancePosOrnColor = new b3OpenCLArray<b3Vector4>(m_clData->m_clContext,m_clData->m_clQueue);
			m_data->m_instancePosOrnColor->resize(3*numObjects);
			m_data->m_instancePosOrnColor->copyFromHostPointer(positions,3*numObjects,0);
			glUnmapBuffer( GL_ARRAY_BUFFER);
			err = glGetError();
			assert(err==GL_NO_ERROR);
		}
	}
	
	err = glGetError();
	assert(err==GL_NO_ERROR);

	{
		B3_PROFILE("stepSimulation");
		m_data->m_rigidBodyPipeline->stepSimulation(1./60.f);
	}

	if (numObjects)
	{
		B3_PROFILE("cl2gl_convert");
		int ciErrNum = 0;
		cl_mem bodies = m_data->m_rigidBodyPipeline->getBodyBuffer();
		b3LauncherCL launch(m_clData->m_clQueue,m_data->m_copyTransformsToVBOKernel,"m_copyTransformsToVBOKernel");
		launch.setBuffer(bodies);
		launch.setBuffer(m_data->m_instancePosOrnColor->getBufferCL());
		launch.setConst(numObjects);
		launch.launch1D(numObjects);
		oclCHECKERROR(ciErrNum, CL_SUCCESS);
	}

	err = glGetError();
	assert(err==GL_NO_ERROR);

	if (animate && numObjects)
	{
		B3_PROFILE("cl2gl_upload");
		GLint err = glGetError();
		assert(err==GL_NO_ERROR);
		GLuint vbo = m_instancingRenderer->getInternalData()->m_vbo;
		int arraySizeInBytes  = numObjects * (3)*sizeof(b3Vector4);
		glBindBuffer(GL_ARRAY_BUFFER, vbo);
		cl_bool blocking=  CL_TRUE;
		positions=  (b3Vector4*)glMapBufferRange( GL_ARRAY_BUFFER,m_instancingRenderer->getMaxShapeCapacity(),arraySizeInBytes, GL_MAP_WRITE_BIT );//GL_READ_WRITE);//GL_WRITE_ONLY
		err = glGetError();
		assert(err==GL_NO_ERROR);
		m_data->m_instancePosOrnColor->copyToHostPointer(positions,3*numObjects,0);
		glUnmapBuffer( GL_ARRAY_BUFFER);
		err = glGetError();
		assert(err==GL_NO_ERROR);
	}

	err = glGetError();
	assert(err==GL_NO_ERROR);

}


GpuSoftClothDemo::GpuSoftClothDemo()
{
}
GpuSoftClothDemo::~GpuSoftClothDemo()
{
}
	


void	GpuSoftClothDemo::setupScene(const ConstructionInfo& ci)
{
	GLint err = glGetError();
	assert(err==GL_NO_ERROR);

	
	int width = 256;
	int height = 256;

	GraphicsVertex* cpu_buffer = new GraphicsVertex[width*height];
	memset(cpu_buffer, 0, width*height*sizeof(GraphicsVertex));


	int numVertices = 0;
	// Initial test data for rendering
	for(int y = 0; y < height; y++)
	{
		for(int x = 0; x < width; x++)
		{
			double coord = b3Sin(x/5.0)*0.01;
			//coord = sin(y/);

			cpu_buffer[y*width+x].pos[0]      = (x/((float)(width-1)))*1;
			cpu_buffer[y*width+x].pos[1]      = coord;
			cpu_buffer[y*width+x].pos[2]      = (y/((float)(height-1)))*1; 
			cpu_buffer[y*width+x].pos[3]      = 0.f;

			cpu_buffer[y*width+x].normal[0]   = 1;
			cpu_buffer[y*width+x].normal[1]   = 0;
			cpu_buffer[y*width+x].normal[2]   = 0;
			cpu_buffer[y*width+x].texcoord[0] = 1*x/((float)(width-1));
			cpu_buffer[y*width+x].texcoord[1] = (1.f-4*y/((float)(height-1)));
			numVertices++;
		}
	}

	err = glGetError();
	assert(err==GL_NO_ERROR);

	int numIndices = 0;

	// Generate and fill index array for rendering
	 int* indices = new  int[width*3*2+2 + height*width*3*2];

	for(int y = 0; y < height-1; y++)
	{
		for(int x = 0; x < width-1; x++)
		{
			// *3 indices/triangle, *2 triangles/quad
			int baseIndex = (x + y*(width-1))*3*2;
			indices[baseIndex] = x + y*width;
			indices[baseIndex+1] = x+1 + y*width;
			indices[baseIndex+2] = x+width + y*width;


			indices[baseIndex+3] = x + 1 +  y*width;
			indices[baseIndex+4] = x+(width+1) + y*width;
			indices[baseIndex+5] = x+width + y*width;
			numIndices++;
		}
	}


	int num_barrel_vertices = sizeof(barrel_vertices)/(9*sizeof(float));
	int num_barrel_indices = sizeof(barrel_indices)/sizeof(int);

	int textureIndex  = -1;
	{
		int width,height,n;
		
		const char* filename = "data/bullet_logo.png";
		const unsigned char* image=0;
		
		const char* prefix[]={"./","../","../../","../../../","../../../../"};
		int numprefix = sizeof(prefix)/sizeof(const char*);
		
		for (int i=0;!image && i<numprefix;i++)
		{
			char relativeFileName[1024];
			sprintf(relativeFileName,"%s%s",prefix[i],filename);
			image = loadImage(relativeFileName,width,height,n);
		}
		
		b3Assert(image);
		if (image)
		{
			textureIndex = ci.m_instancingRenderer->registerTexture(image,width,height);
		}
	}
//	int shapeIndex = ci.m_instancingRenderer->registerShape(barrel_vertices,num_barrel_vertices,barrel_indices,num_barrel_indices);

	err = glGetError();
	assert(err==GL_NO_ERROR);

	m_data->m_clothVertices = &cpu_buffer[0].pos[0];

	int shapeIndex = ci.m_instancingRenderer->registerShape(&cpu_buffer[0].pos[0],numVertices,indices,numIndices,B3_GL_TRIANGLES,textureIndex);
	m_data->m_clothShapeIndex = shapeIndex;

	err = glGetError();
	assert(err==GL_NO_ERROR);

	
	float pos[4] = {0,0,0,0};
	float orn[4] = {0,0,0,1};
	float color[4] = {1,1,1,1};
	float scaling[4] = {10,10,10,1};

	ci.m_instancingRenderer->registerGraphicsInstance(shapeIndex,pos,orn,color,scaling);
	ci.m_instancingRenderer->setCameraDistance(4);
	ci.m_instancingRenderer->setCameraTargetPosition(pos);

	err = glGetError();
	assert(err==GL_NO_ERROR);

}	
