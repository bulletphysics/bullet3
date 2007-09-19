/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "btBulletDynamicsCommon.h"
#include "LinearMath/btIDebugDraw.h"
#include "GLDebugDrawer.h"
#include "ConcaveDemo.h"
#include "GL_ShapeDrawer.h"
#include "GlutStuff.h"

//#define USE_PARALLEL_DISPATCHER 1
#ifdef USE_PARALLEL_DISPATCHER
#include "../../Extras/BulletMultiThreaded/SpuGatheringCollisionDispatcher.h"
#include "../../Extras/BulletMultiThreaded/Win32ThreadSupport.h"
#include "../../Extras/BulletMultiThreaded/SpuNarrowPhaseCollisionTask/SpuGatheringCollisionTask.h"
#endif//USE_PARALLEL_DISPATCHER



GLDebugDrawer	debugDrawer;
class btIDebugDraw* debugDrawerPtr=0;

btVector3*	gVertices=0;
int*	gIndices=0;
btBvhTriangleMeshShape* trimeshShape =0;
btRigidBody* staticBody = 0;
static float waveheight = 5.f;

const float TRIANGLE_SIZE=8.f;



///User can override this material combiner by implementing gContactAddedCallback and setting body0->m_collisionFlags |= btCollisionObject::customMaterialCallback;
inline btScalar	calculateCombinedFriction(float friction0,float friction1)
{
	btScalar friction = friction0 * friction1;

	const btScalar MAX_FRICTION  = 10.f;
	if (friction < -MAX_FRICTION)
		friction = -MAX_FRICTION;
	if (friction > MAX_FRICTION)
		friction = MAX_FRICTION;
	return friction;

}

inline btScalar	calculateCombinedRestitution(float restitution0,float restitution1)
{
	return restitution0 * restitution1;
}



bool CustomMaterialCombinerCallback(btManifoldPoint& cp,	const btCollisionObject* colObj0,int partId0,int index0,const btCollisionObject* colObj1,int partId1,int index1)
{

	float friction0 = colObj0->getFriction();
	float friction1 = colObj1->getFriction();
	float restitution0 = colObj0->getRestitution();
	float restitution1 = colObj1->getRestitution();

	if (colObj0->getCollisionFlags() & btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK)
	{
		friction0 = 1.0;//partId0,index0
		restitution0 = 0.f;
	}
	if (colObj1->getCollisionFlags() & btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK)
	{
		if (index1&1)
		{
			friction1 = 1.0f;//partId1,index1
		} else
		{
			friction1 = 0.f;
		}
		restitution1 = 0.f;
	}

	cp.m_combinedFriction = calculateCombinedFriction(friction0,friction1);
	cp.m_combinedRestitution = calculateCombinedRestitution(restitution0,restitution1);

	//this return value is currently ignored, but to be on the safe side: return false if you don't calculate friction
	return true;
}

extern ContactAddedCallback		gContactAddedCallback;



int main(int argc,char** argv)
{
	gContactAddedCallback = CustomMaterialCombinerCallback;

	ConcaveDemo* concaveDemo = new ConcaveDemo();
	concaveDemo->initPhysics();
	concaveDemo->setCameraDistance(30.f);

	return glutmain(argc, argv,640,480,"Static Concave Mesh Demo",concaveDemo);
}


	const int NUM_VERTS_X = 30;
	const int NUM_VERTS_Y = 30;
	const int totalVerts = NUM_VERTS_X*NUM_VERTS_Y;

void	ConcaveDemo::setVertexPositions(float waveheight, float offset)
{
	int i;
	int j;

	for ( i=0;i<NUM_VERTS_X;i++)
	{
		for (j=0;j<NUM_VERTS_Y;j++)
		{
			gVertices[i+j*NUM_VERTS_X].setValue((i-NUM_VERTS_X*0.5f)*TRIANGLE_SIZE,
				//0.f,
				waveheight*sinf((float)i+offset)*cosf((float)j+offset),
				(j-NUM_VERTS_Y*0.5f)*TRIANGLE_SIZE);
		}
	}
}

void ConcaveDemo::keyboardCallback(unsigned char key, int x, int y)
{
	if (key == 'g')
	{
		m_animatedMesh = !m_animatedMesh;
		if (m_animatedMesh)
		{
			staticBody->setCollisionFlags( staticBody->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
			staticBody->setActivationState(DISABLE_DEACTIVATION);
		} else
		{
			staticBody->setCollisionFlags( staticBody->getCollisionFlags() & ~btCollisionObject::CF_KINEMATIC_OBJECT);
			staticBody->forceActivationState(ACTIVE_TAG);
		}
	}

	DemoApplication::keyboardCallback(key,x,y);

}

void	ConcaveDemo::initPhysics()
{
	#define TRISIZE 10.f

	int vertStride = sizeof(btVector3);
	int indexStride = 3*sizeof(int);

	
	const int totalTriangles = 2*(NUM_VERTS_X-1)*(NUM_VERTS_Y-1);

	gVertices = new btVector3[totalVerts];
	gIndices = new int[totalTriangles*3];

	int i;


	setVertexPositions(waveheight,0.f);

	int index=0;
	for ( i=0;i<NUM_VERTS_X-1;i++)
	{
		for (int j=0;j<NUM_VERTS_Y-1;j++)
		{
			gIndices[index++] = j*NUM_VERTS_X+i;
			gIndices[index++] = j*NUM_VERTS_X+i+1;
			gIndices[index++] = (j+1)*NUM_VERTS_X+i+1;

			gIndices[index++] = j*NUM_VERTS_X+i;
			gIndices[index++] = (j+1)*NUM_VERTS_X+i+1;
			gIndices[index++] = (j+1)*NUM_VERTS_X+i;
		}
	}
	
	btTriangleIndexVertexArray* indexVertexArrays = new btTriangleIndexVertexArray(totalTriangles,
		gIndices,
		indexStride,
		totalVerts,(btScalar*) &gVertices[0].x(),vertStride);

	bool useQuantizedAabbCompression = true;

//comment out the next line to read the BVH from disk (first run the demo once to create the BVH)
#define SERIALIZE_TO_DISK 1
#ifdef SERIALIZE_TO_DISK
	trimeshShape  = new btBvhTriangleMeshShape(indexVertexArrays,useQuantizedAabbCompression);
	
	///we can serialize the BVH data 
	void* buffer = 0;
	int numBytes = trimeshShape->getOptimizedBvh()->calculateSerializeBufferSize();
	buffer = btAlignedAlloc(numBytes,16);
	bool swapEndian = false;
	trimeshShape->getOptimizedBvh()->serialize(buffer,numBytes,swapEndian);
	FILE* file = fopen("bvh.bin","wb");
	fwrite(buffer,1,numBytes,file);
	fclose(file);

#else

	trimeshShape  = new btBvhTriangleMeshShape(indexVertexArrays,useQuantizedAabbCompression,false);

	char* fileName = "bvh.bin";

	FILE* file = fopen(fileName,"rb");
	int size=0;
	btOptimizedBvh* bvh = 0;

	if (fseek(file, 0, SEEK_END) || (size = ftell(file)) == EOF || fseek(file, 0, SEEK_SET)) {        /* File operations denied? ok, just close and return failure */
		printf("Error: cannot get filesize from %s\n", fileName);
		exit(0);
	} else
	{

		fseek(file, 0, SEEK_SET);

		int buffersize = size+btOptimizedBvh::getAlignmentSerializationPadding();

		void* buffer = btAlignedAlloc(buffersize,16);
		//memset(buffer,0xcc,size);
		int read = fread(buffer,1,size,file);
		fclose(file);
		bool swapEndian = false;
		bvh = btOptimizedBvh::deSerializeInPlace(buffer,buffersize,swapEndian);
	}

	trimeshShape->setOptimizedBvh(bvh);

#endif

//	btCollisionShape* groundShape = new btBoxShape(btVector3(50,3,50));

btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();

#ifdef USE_PARALLEL_DISPATCHER

#ifdef USE_WIN32_THREADING

	int maxNumOutstandingTasks = 4;//number of maximum outstanding tasks
	Win32ThreadSupport* threadSupport = new Win32ThreadSupport(Win32ThreadSupport::Win32ThreadConstructionInfo(
								"collision",
								processCollisionTask,
								createCollisionLocalStoreMemory,
								maxNumOutstandingTasks));
#else
///todo other platform threading
///Playstation 3 SPU (SPURS)  version is available through PS3 Devnet
///Libspe2 SPU support will be available soon
///pthreads version
///you can hook it up to your custom task scheduler by deriving from btThreadSupportInterface
#endif

	btCollisionDispatcher* dispatcher = new	SpuGatheringCollisionDispatcher(threadSupport,maxNumOutstandingTasks,collisionConfiguration);
#else
		btCollisionDispatcher* dispatcher = new	btCollisionDispatcher(collisionConfiguration);
#endif//USE_PARALLEL_DISPATCHER


	btVector3 worldMin(-1000,-1000,-1000);
	btVector3 worldMax(1000,1000,1000);
	btBroadphaseInterface* pairCache = new btAxisSweep3(worldMin,worldMax);
	btConstraintSolver* constraintSolver = new btSequentialImpulseConstraintSolver();
	m_dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher,pairCache,constraintSolver);
#ifdef USE_PARALLEL_DISPATCHER
	m_dynamicsWorld->getDispatchInfo().m_enableSPU=true;
#endif //USE_PARALLEL_DISPATCHER
	m_dynamicsWorld->setDebugDrawer(&debugDrawer);
	debugDrawerPtr = &debugDrawer;
	
	float mass = 0.f;
	btTransform	startTransform;
	startTransform.setIdentity();
	startTransform.setOrigin(btVector3(0,-2,0));

	{
		for (int i=0;i<10;i++)
		{
			//btCollisionShape* colShape = new btBoxShape(btVector3(1,1,1));
			btCollisionShape* colShape = new btCapsuleShape(0.5,2.0);//boxShape = new btSphereShape(1.f);
			startTransform.setOrigin(btVector3(2*i,10,1));
			localCreateRigidBody(1, startTransform,colShape);
		}
	}

	startTransform.setIdentity();
	staticBody = localCreateRigidBody(mass, startTransform,trimeshShape);

	staticBody->setCollisionFlags(staticBody->getCollisionFlags() | btCollisionObject::CF_STATIC_OBJECT);

	//enable custom material callback
	staticBody->setCollisionFlags(staticBody->getCollisionFlags()  | btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);

	
	
}

void ConcaveDemo::clientMoveAndDisplay()
{
	 glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	float dt = m_clock.getTimeMicroseconds() * 0.000001f;
	m_clock.reset();

	if (m_animatedMesh)
	{
		static float offset=0.f;
		offset+=0.01f;

		setVertexPositions(waveheight,offset);

		trimeshShape->refitTree();

		//clear all contact points involving mesh proxy. Note: this is a slow/unoptimized operation.
		m_dynamicsWorld->getBroadphase()->getOverlappingPairCache()->cleanProxyFromPairs(staticBody->getBroadphaseHandle(),getDynamicsWorld()->getDispatcher());
	}

	m_dynamicsWorld->stepSimulation(dt);
	
	renderme();

    glFlush();
    glutSwapBuffers();

}




void ConcaveDemo::displayCallback(void) {

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	renderme();

    glFlush();
    glutSwapBuffers();
}


