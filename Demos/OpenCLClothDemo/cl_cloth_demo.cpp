/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2008 Advanced Micro Devices

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifdef _WIN32
#include <GL/glew.h>
#endif


#ifndef USE_MINICL
#define USE_SIMDAWARE_SOLVER
#ifndef __APPLE__
#define USE_GPU_SOLVER
#if defined (_WIN32)
	#define USE_GPU_COPY //only tested on Windows
#endif //_WIN32
#endif //__APPLE__
#endif //USE_MINICL




#include "clstuff.h"
#include "gl_win.h"
#include "cloth.h"

#include "../OpenGL/GLDebugDrawer.h"

GLDebugDrawer debugDraw;

const int numFlags = 5;
const int clothWidth = 40;
const int clothHeight = 60;//60;
float _windAngle = 1.0;//0.4;
float _windStrength = 10.;









#include "btBulletDynamicsCommon.h"
#include "LinearMath/btHashMap.h"
#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"
#include "vectormath/vmInclude.h"
#include "BulletMultiThreaded/GpuSoftBodySolvers/CPU/btSoftBodySolver_CPU.h"
#include "BulletMultiThreaded/GpuSoftBodySolvers/OpenCL/btSoftBodySolver_OpenCL.h"
#include "BulletMultiThreaded/GpuSoftBodySolvers/OpenCL/btSoftBodySolver_OpenCLSIMDAware.h"
#include "BulletMultiThreaded/GpuSoftBodySolvers/OpenCL/btSoftBodySolverVertexBuffer_OpenGL.h"
#include "BulletMultiThreaded/GpuSoftBodySolvers/OpenCL/btSoftBodySolverOutputCLtoGL.h"


btRigidBody *capCollider;


using Vectormath::Aos::Vector3;

class piece_of_cloth;
class btBroadphaseInterface;
class btCollisionShape;
class btOverlappingPairCache;
class btCollisionDispatcher;
class btConstraintSolver;
struct btCollisionAlgorithmCreateFunc;
class btDefaultCollisionConfiguration;

#include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"

namespace Vectormath
{
	namespace Aos
	{
		class Transform3;
	}
}


btAlignedObjectArray<btCollisionShape*>	m_collisionShapes;
btBroadphaseInterface*	m_broadphase;
btCollisionDispatcher*	m_dispatcher;
btConstraintSolver*	m_solver;
btDefaultCollisionConfiguration* m_collisionConfiguration;

btCPUSoftBodySolver *g_cpuSolver = NULL;
btOpenCLSoftBodySolver *g_openCLSolver = NULL;
btOpenCLSoftBodySolverSIMDAware *g_openCLSIMDSolver = NULL;

btSoftBodySolver *g_solver = NULL;

btSoftBodySolverOutput *g_softBodyOutput = NULL;

btAlignedObjectArray<btSoftBody *> m_flags;
btSoftRigidDynamicsWorld* m_dynamicsWorld;
btAlignedObjectArray<piece_of_cloth> cloths;

extern cl_context			g_cxMainContext;
extern cl_device_id			g_cdDevice;
extern cl_command_queue		g_cqCommandQue;


const float flagSpacing = 30.f;


// Helper to test and add links correctly.
// Records links that have already been generated
static bool testAndAddLink( btAlignedObjectArray<int> &trianglesForLinks, btSoftBody *softBody, int triangle, int *triangleVertexIndexArray, int numVertices, int vertex0, int vertex1, int nonLinkVertex, btSoftBody::Material *structuralMaterial, bool createBendLinks, btSoftBody::Material *bendMaterial )
{		
	if( trianglesForLinks[ numVertices * vertex0 + vertex1 ] >= 0 && createBendLinks)
	{
		// Already have link so find other triangle and generate cross link

		int otherTriangle = trianglesForLinks[numVertices * vertex0 + vertex1];
		int otherIndices[3] = {triangleVertexIndexArray[otherTriangle * 3], triangleVertexIndexArray[otherTriangle * 3 + 1], triangleVertexIndexArray[otherTriangle * 3 + 2]};

		int nodeA;
		// Test all links of the other triangle against this link. The one that's not part of it is what we want.
		if( otherIndices[0] != vertex0 && otherIndices[0] != vertex1 )
			nodeA = otherIndices[0];
		if( otherIndices[1] != vertex0 && otherIndices[1] != vertex1 )
			nodeA = otherIndices[1];
		if( otherIndices[2] != vertex0 && otherIndices[2] != vertex1 )
			nodeA = otherIndices[2];

		softBody->appendLink( nodeA, nonLinkVertex, bendMaterial );
	} else {
		// Don't yet have link so create it
		softBody->appendLink( vertex0, vertex1, structuralMaterial );

		// If we added a new link, set the triangle array
		trianglesForLinks[numVertices * vertex0 + vertex1] = triangle;
		trianglesForLinks[numVertices * vertex1 + vertex0] = triangle;

	}

	return true;
}

btSoftBody *createFromIndexedMesh( btVector3 *vertexArray, int numVertices, int *triangleVertexIndexArray, int numTriangles, bool createBendLinks )
{
	btSoftBody* softBody = new btSoftBody(&(m_dynamicsWorld->getWorldInfo()), numVertices, vertexArray, 0);
	btSoftBody::Material * structuralMaterial = softBody->appendMaterial();
	btSoftBody::Material * bendMaterial;
	if( createBendLinks )
	{
		bendMaterial = softBody->appendMaterial();
		bendMaterial->m_kLST = 0.7;
	} else {
		bendMaterial = NULL;
	}
	structuralMaterial->m_kLST = 1.0;
	

	// List of values for each link saying which triangle is associated with that link
	// -1 to start. Once a value is entered we know the "other" triangle
	// and can add a link across the link
	btAlignedObjectArray<int> triangleForLinks;
	triangleForLinks.resize( numVertices * numVertices, -1 );
	int numLinks = 0;
	for( int triangle = 0; triangle < numTriangles; ++triangle )
	{
		int index[3] = {triangleVertexIndexArray[triangle * 3], triangleVertexIndexArray[triangle * 3 + 1], triangleVertexIndexArray[triangle * 3 + 2]};
		softBody->appendFace( index[0], index[1], index[2] );
		
		// Generate the structural links directly from the triangles
		testAndAddLink( triangleForLinks, softBody, triangle, triangleVertexIndexArray, numVertices, index[0], index[1], index[2], structuralMaterial, createBendLinks, bendMaterial );
		testAndAddLink( triangleForLinks, softBody, triangle, triangleVertexIndexArray, numVertices, index[1], index[2], index[0], structuralMaterial, createBendLinks, bendMaterial );
		testAndAddLink( triangleForLinks, softBody, triangle, triangleVertexIndexArray, numVertices, index[2], index[0], index[1], structuralMaterial, createBendLinks, bendMaterial);
	}

	return softBody;
}

/**
 * Create a sequence of flag objects and add them to the world.
 */
void createFlag( btSoftBodySolver &solver, int width, int height, btAlignedObjectArray<btSoftBody *> &flags )
{
	// First create a triangle mesh to represent a flag

	using Vectormath::Aos::Matrix3;
	using Vectormath::Aos::Vector3;

	// Allocate a simple mesh consisting of a vertex array and a triangle index array
	btIndexedMesh mesh;
	mesh.m_numVertices = width*height;
	mesh.m_numTriangles = 2*(width-1)*(height-1);

	btVector3 *vertexArray = new btVector3[mesh.m_numVertices];

	mesh.m_vertexBase = reinterpret_cast<const unsigned char*>(vertexArray);
	int *triangleVertexIndexArray = new int[3*mesh.m_numTriangles];	
	mesh.m_triangleIndexBase = reinterpret_cast<const unsigned char*>(triangleVertexIndexArray);
	mesh.m_triangleIndexStride = sizeof(int)*3;
	mesh.m_vertexStride = sizeof(Vector3);

	// Generate normalised object space vertex coordinates for a rectangular flag
	float zCoordinate = 0.0f;
	
	Matrix3 defaultScale(Vector3(5.f, 0.f, 0.f), Vector3(0.f, 20.f, 0.f), Vector3(0.f, 0.f, 1.f));
	for( int y = 0; y < height; ++y )
	{
		float yCoordinate = y*2.0f/float(height) - 1.0f;
		for( int x = 0; x < width; ++x )
		{			
			float xCoordinate = x*2.0f/float(width) - 1.0f;

			Vector3 vertex(xCoordinate, yCoordinate, zCoordinate);
			Vector3 transformedVertex = defaultScale*vertex;

			vertexArray[y*width + x] = btVector3(transformedVertex.getX(), transformedVertex.getY(), transformedVertex.getZ() );

		}
	}

	// Generate vertex indices for triangles
	for( int y = 0; y < (height-1); ++y )
	{
		for( int x = 0; x < (width-1); ++x )
		{	
			// Triangle 0
			// Top left of square on mesh
			{
				int vertex0 = y*width + x;
				int vertex1 = vertex0 + 1;
				int vertex2 = vertex0 + width;
				int triangleIndex = 2*y*(width-1) + 2*x;
				triangleVertexIndexArray[(mesh.m_triangleIndexStride*triangleIndex)/sizeof(int)] = vertex0;
				triangleVertexIndexArray[(mesh.m_triangleIndexStride*triangleIndex+1)/sizeof(int)+1] = vertex1;
				triangleVertexIndexArray[(mesh.m_triangleIndexStride*triangleIndex+2)/sizeof(int)+2] = vertex2;
			}

			// Triangle 1
			// Bottom right of square on mesh
			{
				int vertex0 = y*width + x + 1;
				int vertex1 = vertex0 + width;
				int vertex2 = vertex1 - 1;
				int triangleIndex = 2*y*(width-1) + 2*x + 1;
				triangleVertexIndexArray[(mesh.m_triangleIndexStride*triangleIndex)/sizeof(int)] = vertex0;
				triangleVertexIndexArray[(mesh.m_triangleIndexStride*triangleIndex)/sizeof(int)+1] = vertex1;
				triangleVertexIndexArray[(mesh.m_triangleIndexStride*triangleIndex)/sizeof(int)+2] = vertex2;
			}
		}
	}

	
	float rotateAngleRoundZ = 0.0;
	float rotateAngleRoundX = 0.5;
	btMatrix3x3 defaultRotate;
	defaultRotate[0] = btVector3(cos(rotateAngleRoundZ), sin(rotateAngleRoundZ), 0.f); 
	defaultRotate[1] = btVector3(-sin(rotateAngleRoundZ), cos(rotateAngleRoundZ), 0.f);
	defaultRotate[2] = btVector3(0.f, 0.f, 1.f);
	btMatrix3x3 defaultRotateX;
	defaultRotateX[0] = btVector3(1.f, 0.f, 0.f);
	defaultRotateX[1] = btVector3( 0.f, cos(rotateAngleRoundX), sin(rotateAngleRoundX));
	defaultRotateX[2] = btVector3(0.f, -sin(rotateAngleRoundX), cos(rotateAngleRoundX));

	btMatrix3x3 defaultRotateAndScale( (defaultRotateX*defaultRotate) );


	// Construct the sequence flags applying a slightly different translation to each one to arrange them
	// appropriately in the scene.
	for( int i = 0; i < numFlags; ++i )
	{
		float zTranslate = flagSpacing * (i-numFlags/2);

		btVector3 defaultTranslate(0.f, 20.f, zTranslate);

		btTransform transform( defaultRotateAndScale, defaultTranslate );
		transform.setOrigin(defaultTranslate);


		btSoftBody *softBody = createFromIndexedMesh( vertexArray, mesh.m_numVertices, triangleVertexIndexArray, mesh.m_numTriangles, true );


		for( int i = 0; i < mesh.m_numVertices; ++i )
		{
			softBody->setMass(i, 10.f/mesh.m_numVertices);
		}
		softBody->setMass((height-1)*(width), 0.f);
		softBody->setMass((height-1)*(width) + width - 1, 0.f);
		softBody->setMass((height-1)*width + width/2, 0.f);
		softBody->m_cfg.collisions = btSoftBody::fCollision::CL_SS+btSoftBody::fCollision::CL_RS;	
		
		softBody->m_cfg.kLF = 0.0005f;
		softBody->m_cfg.kVCF = 0.001f;
		softBody->m_cfg.kDP = 0.f;
		softBody->m_cfg.kDG = 0.f;

		
		flags.push_back( softBody );

		softBody->transform( transform );
		
		m_dynamicsWorld->addSoftBody( softBody );
	}

	delete [] vertexArray;
	delete [] triangleVertexIndexArray;
}


void updatePhysicsWorld()
{
	static int counter = 1;

	// Change wind velocity a bit based on a frame counter
	if( (counter % 400) == 0 )
	{
		_windAngle = (_windAngle + 0.05f);
		if( _windAngle > (2*3.141) )
			_windAngle = 0;

		for( int flagIndex = 0; flagIndex < m_flags.size(); ++flagIndex )
		{		
			btSoftBody *cloth = 0;

			cloth = m_flags[flagIndex];

			float localWind = _windAngle + 0.5*(((float(rand())/RAND_MAX))-0.1);
			float xCoordinate = cos(localWind)*_windStrength;
			float zCoordinate = sin(localWind)*_windStrength;

			cloth->setWindVelocity( btVector3(xCoordinate, 0, zCoordinate) );
		}
	}

	//btVector3 origin( capCollider->getWorldTransform().getOrigin() );
	//origin.setX( origin.getX() + 0.05 );
	//capCollider->getWorldTransform().setOrigin( origin );
	
	counter++;
}

void initBullet(void)
{

#ifdef USE_GPU_SOLVER
#ifdef USE_SIMDAWARE_SOLVER
	g_openCLSIMDSolver = new btOpenCLSoftBodySolverSIMDAware( g_cqCommandQue, g_cxMainContext);
	g_solver = g_openCLSIMDSolver;
#ifdef USE_GPU_COPY
	g_softBodyOutput = new btSoftBodySolverOutputCLtoGL(g_cqCommandQue, g_cxMainContext);
#else // #ifdef USE_GPU_COPY
	g_softBodyOutput = new btSoftBodySolverOutputCLtoCPU;
#endif // #ifdef USE_GPU_COPY
#else
	g_openCLSolver = new btOpenCLSoftBodySolver( g_cqCommandQue, g_cxMainContext );
	g_solver = g_openCLSolver;
#ifdef USE_GPU_COPY
	g_softBodyOutput = new btSoftBodySolverOutputCLtoGL(g_cqCommandQue, g_cxMainContext);
#else // #ifdef USE_GPU_COPY
	g_softBodyOutput = new btSoftBodySolverOutputCLtoCPU;
#endif // #ifdef USE_GPU_COPY
#endif
#else
	g_cpuSolver = new btCPUSoftBodySolver;
	g_solver = g_cpuSolver;
	g_softBodyOutput = new btSoftBodySolverOutputCPUtoCPU;
#endif

	//m_collisionConfiguration = new btDefaultCollisionConfiguration();
	m_collisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration();
	

	m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);
	m_broadphase = new btDbvtBroadphase();
	btSequentialImpulseConstraintSolver* sol = new btSequentialImpulseConstraintSolver;
	m_solver = sol;

	m_dynamicsWorld = new btSoftRigidDynamicsWorld(m_dispatcher, m_broadphase, m_solver, m_collisionConfiguration, g_solver);	

	m_dynamicsWorld->setGravity(btVector3(0,-10,0));	
	btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(50.),btScalar(50.),btScalar(50.)));	
	m_collisionShapes.push_back(groundShape);
	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0,-50,0));

	




	m_dynamicsWorld->getWorldInfo().air_density			=	(btScalar)1.2;
	m_dynamicsWorld->getWorldInfo().water_density		=	0;
	m_dynamicsWorld->getWorldInfo().water_offset		=	0;
	m_dynamicsWorld->getWorldInfo().water_normal		=	btVector3(0,0,0);
	m_dynamicsWorld->getWorldInfo().m_gravity.setValue(0,-10,0);



#if 0
	{
		btScalar mass(0.);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0,0,0);
		if (isDynamic)
			groundShape->calculateLocalInertia(mass,localInertia);

		//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
		btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,groundShape,localInertia);
		btRigidBody* body = new btRigidBody(rbInfo);

		//add the body to the dynamics world
		m_dynamicsWorld->addRigidBody(body);
	}
 
#endif

	#if 1
	{		
		btScalar mass(0.);

		//btScalar mass(1.);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);
		
		btCollisionShape *capsuleShape = new btCapsuleShape(5, 10);
		capsuleShape->setMargin( 0.5 );
		
		


		btVector3 localInertia(0,0,0);
		if (isDynamic)
			capsuleShape->calculateLocalInertia(mass,localInertia);

		m_collisionShapes.push_back(capsuleShape);
		btTransform capsuleTransform;
		capsuleTransform.setIdentity();
#ifdef TABLETEST
		capsuleTransform.setOrigin(btVector3(0, 10, -11));
		const btScalar pi = 3.141592654;
		capsuleTransform.setRotation(btQuaternion(0, 0, pi/2));
#else
		capsuleTransform.setOrigin(btVector3(0, 0, 0));
		
		const btScalar pi = 3.141592654;
		//capsuleTransform.setRotation(btQuaternion(0, 0, pi/2));
		capsuleTransform.setRotation(btQuaternion(0, 0, 0));
#endif
		btDefaultMotionState* myMotionState = new btDefaultMotionState(capsuleTransform);
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,capsuleShape,localInertia);
		btRigidBody* body = new btRigidBody(rbInfo);
		body->setFriction( 0.8f );

		m_dynamicsWorld->addRigidBody(body);
		//cap_1.collisionShape = body;
		capCollider = body;
	}
#endif


#ifdef USE_GPU_SOLVER
	createFlag( *g_openCLSolver, clothWidth, clothHeight, m_flags );
#else
	createFlag( *g_cpuSolver, clothWidth, clothHeight, m_flags );
#endif

	// Create output buffer descriptions for ecah flag
	// These describe where the simulation should send output data to
	for( int flagIndex = 0; flagIndex < m_flags.size(); ++flagIndex )
	{		
//		m_flags[flagIndex]->setWindVelocity( Vectormath::Aos::Vector3( 0.f, 0.f, 15.f ) );
		
		// In this case we have a DX11 output buffer with a vertex at index 0, 8, 16 and so on as well as a normal at 3, 11, 19 etc.
		// Copies will be performed GPU-side directly into the output buffer

#ifdef USE_GPU_COPY
		GLuint targetVBO = cloths[flagIndex].getVBO();
		btOpenGLInteropVertexBufferDescriptor *vertexBufferDescriptor = new btOpenGLInteropVertexBufferDescriptor(g_cqCommandQue, g_cxMainContext, targetVBO, 0, 8, 3, 8);
#else
		btCPUVertexBufferDescriptor *vertexBufferDescriptor = new btCPUVertexBufferDescriptor(reinterpret_cast< float* >(cloths[flagIndex].cpu_buffer), 0, 8, 3, 8);
#endif
		cloths[flagIndex].m_vertexBufferDescriptor = vertexBufferDescriptor;
	}


	g_solver->optimize( m_dynamicsWorld->getSoftBodyArray() );
	
	if (!g_solver->checkInitialized())
	{
		printf("OpenCL kernel initialization ?failed\n");
		btAssert(0);
		exit(0);
	}

}




btClock m_clock;

void doFlags()
{
	//float ms = getDeltaTimeMicroseconds();
	btScalar dt = (btScalar)m_clock.getTimeMicroseconds();
	m_clock.reset();

	///step the simulation
	if( m_dynamicsWorld )
	{
		m_dynamicsWorld->stepSimulation(dt/1000000.);

		static int frameCount = 0;
		frameCount++;
		if (frameCount==100)
		{
 			m_dynamicsWorld->stepSimulation(1./60.,0);
			
		// Option to save a .bullet file
		//	btDefaultSerializer*	serializer = new btDefaultSerializer();
		//	m_dynamicsWorld->serialize(serializer);
		//	FILE* file = fopen("testFile.bullet","wb");
		//	fwrite(serializer->getBufferPointer(),serializer->getCurrentBufferSize(),1, file);
		//	fclose(file);

			CProfileManager::dumpAll();
		}
		updatePhysicsWorld();

		//m_dynamicsWorld->setDebugDrawer(&debugDraw);
		//debugDraw.setDebugMode(btIDebugDraw::DBG_DrawWireframe);
		//g_solver->copyBackToSoftBodies();

		//m_dynamicsWorld->debugDrawWorld();
		
	}
	

	for( int flagIndex = 0; flagIndex < m_flags.size(); ++flagIndex )
	{
		g_softBodyOutput->copySoftBodyToVertexBuffer( m_flags[flagIndex], cloths[flagIndex].m_vertexBufferDescriptor );
		cloths[flagIndex].draw();
	}
}


int main(int argc, char *argv[])
{

	
	preInitGL(argc, argv);
#ifdef _WIN32
	glewInit();
#endif

#ifdef USE_GPU_COPY
#ifdef _WIN32
    HGLRC glCtx = wglGetCurrentContext();
#else //!_WIN32
    GLXContext glCtx = glXGetCurrentContext();
#endif //!_WIN32
	HDC glDC = wglGetCurrentDC();
	
	initCL(glCtx, glDC);
#else

	initCL();

#endif

	cloths.resize(numFlags);

	for( int flagIndex =  0; flagIndex < numFlags; ++flagIndex )
	{
		cloths[flagIndex].create_buffers(clothWidth, clothHeight);
	}

	initBullet();
	m_dynamicsWorld->stepSimulation(1./60.,0);

	std::string flagTexs[] = {
		"bullet_logo.png",
		"bullet_logo.png",
	};
	int numFlagTexs = 2;

	for( int flagIndex =  0; flagIndex < numFlags; ++flagIndex )
	{
		cloths[flagIndex].create_texture(flagTexs[flagIndex % numFlagTexs]);
		cloths[flagIndex].x_offset = 0; 
		cloths[flagIndex].y_offset = 0; 
		cloths[flagIndex].z_offset = 0;
	}

	goGL();

	if( g_cpuSolver )
		delete g_cpuSolver;
	if( g_openCLSolver  )
		delete g_openCLSolver;
	if( g_openCLSIMDSolver  )
		delete g_openCLSIMDSolver;
	if( g_softBodyOutput )
		delete g_softBodyOutput;

 	return 0;
}

