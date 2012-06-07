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
#include "GLDebugFont.h"


//#define SHIFT_INDICES 1
#define SWAP_WINDING 1
//#define ROTATE_GROUND 1
bool enable=true;

#if defined (SHIFT_INDICES) && !defined (SWAP_WINDING)
//#define TEST_INCONSISTENT_WINDING
#endif



#include "btBulletDynamicsCommon.h"
#include "BulletCollision/CollisionDispatch/btInternalEdgeUtility.h"
#include "Taru.mdl"

#include "LinearMath/btIDebugDraw.h"
#include "GLDebugDrawer.h"
#include "InternalEdgeDemo.h"
#include "GL_ShapeDrawer.h"
#include "GlutStuff.h"
#include "BulletCollision/CollisionShapes/btTriangleShape.h"
#include "GLDebugDrawer.h"
GLDebugDrawer	gDebugDrawer;

static btVector3*	gVertices=0;
static int*	gIndices=0;
static btBvhTriangleMeshShape* trimeshShape =0;
static btRigidBody* staticBody = 0;
static float waveheight = 0.f;

const float TRIANGLE_SIZE=20.f;



///User can override this material combiner by implementing gContactAddedCallback and setting body0->m_collisionFlags |= btCollisionObject::customMaterialCallback;
inline btScalar	calculateCombinedFriction(float friction0,float friction1)
{
	return 0.f;
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


///////////////////////////////////////////////////////////////


static bool CustomMaterialCombinerCallback(btManifoldPoint& cp,	const btCollisionObjectWrapper* colObj0Wrap,int partId0,int index0,const btCollisionObjectWrapper* colObj1Wrap,int partId1,int index1)
{

	if (enable)
	{
		btAdjustInternalEdgeContacts(cp,colObj1Wrap,colObj0Wrap, partId1,index1);
		//btAdjustInternalEdgeContacts(cp,colObj1,colObj0, partId1,index1, BT_TRIANGLE_CONVEX_BACKFACE_MODE);
		//btAdjustInternalEdgeContacts(cp,colObj1,colObj0, partId1,index1, BT_TRIANGLE_CONVEX_DOUBLE_SIDED+BT_TRIANGLE_CONCAVE_DOUBLE_SIDED);
	}

	float friction0 = colObj0Wrap->getCollisionObject()->getFriction();
	float friction1 = colObj1Wrap->getCollisionObject()->getFriction();
	float restitution0 = colObj0Wrap->getCollisionObject()->getRestitution();
	float restitution1 = colObj1Wrap->getCollisionObject()->getRestitution();

	if (colObj0Wrap->getCollisionObject()->getCollisionFlags() & btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK)
	{
		friction0 = 1.0;//partId0,index0
		restitution0 = 0.f;
	}
	if (colObj1Wrap->getCollisionObject()->getCollisionFlags() & btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK)
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

	const int NUM_VERTS_X = 2;
	const int NUM_VERTS_Y = 2;
	const int totalVerts = NUM_VERTS_X*NUM_VERTS_Y;

void	InternalEdgeDemo::setVertexPositions(float waveheight, float offset)
{
	int i;
	int j;

	for ( i=0;i<NUM_VERTS_X;i++)
	{
		for (j=0;j<NUM_VERTS_Y;j++)
		{
			gVertices[i+j*NUM_VERTS_X].setValue(
				(i-NUM_VERTS_X*0.5f)*TRIANGLE_SIZE,
				//0.f,
				waveheight*sinf((float)i+offset)*cosf((float)j+offset),
				(j-NUM_VERTS_Y*0.5f)*TRIANGLE_SIZE);
		}
	}
}

void InternalEdgeDemo::keyboardCallback(unsigned char key, int x, int y)
{
	if (key=='n')
	{
		enable = !enable;
	}

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



void	InternalEdgeDemo::initPhysics()
{
	
	setTexturing(true);
	setShadows(false);//true);

	#define TRISIZE 10.f

     gContactAddedCallback = CustomMaterialCombinerCallback;

#define USE_TRIMESH_SHAPE 1
#ifdef USE_TRIMESH_SHAPE

	int vertStride = sizeof(btVector3);
	int indexStride = 3*sizeof(int);

	
	const int totalTriangles = 2*(NUM_VERTS_X-1)*(NUM_VERTS_Y-1);

	gVertices = new btVector3[totalVerts];
	gIndices = new int[totalTriangles*3];

	int i;


	setVertexPositions(waveheight,0.f);
	
	
	//gVertices[1].setY(21.1);
	//gVertices[1].setY(121.1);
	gVertices[1].setY(.1f);

#ifdef ROTATE_GROUND
	//gVertices[1].setY(-1.1);
#else
	//gVertices[1].setY(0.1);
	//gVertices[1].setY(-0.1);
	//gVertices[1].setY(-20.1);
	//gVertices[1].setY(-20);
#endif
	
	int index=0;
	for ( i=0;i<NUM_VERTS_X-1;i++)
	{
		for (int j=0;j<NUM_VERTS_Y-1;j++)
		{

#ifdef SWAP_WINDING
#ifdef SHIFT_INDICES
			gIndices[index++] = j*NUM_VERTS_X+i;
			gIndices[index++] = (j+1)*NUM_VERTS_X+i+1;
			gIndices[index++] = j*NUM_VERTS_X+i+1;
			
			gIndices[index++] = j*NUM_VERTS_X+i;
			gIndices[index++] = (j+1)*NUM_VERTS_X+i;
			gIndices[index++] = (j+1)*NUM_VERTS_X+i+1;
			
#else
			gIndices[index++] = (j+1)*NUM_VERTS_X+i+1;
			gIndices[index++] = j*NUM_VERTS_X+i+1;
			gIndices[index++] = j*NUM_VERTS_X+i;

			gIndices[index++] = (j+1)*NUM_VERTS_X+i;
			gIndices[index++] = (j+1)*NUM_VERTS_X+i+1;
			gIndices[index++] = j*NUM_VERTS_X+i;
#endif //SHIFT_INDICES
#else //SWAP_WINDING

#ifdef SHIFT_INDICES
			gIndices[index++] = (j+1)*NUM_VERTS_X+i+1;
			gIndices[index++] = j*NUM_VERTS_X+i;
			gIndices[index++] = j*NUM_VERTS_X+i+1;

#ifdef TEST_INCONSISTENT_WINDING
			gIndices[index++] = j*NUM_VERTS_X+i;
			gIndices[index++] = (j+1)*NUM_VERTS_X+i;
			gIndices[index++] = (j+1)*NUM_VERTS_X+i+1;

#else //TEST_INCONSISTENT_WINDING
			gIndices[index++] = (j+1)*NUM_VERTS_X+i;
			gIndices[index++] = j*NUM_VERTS_X+i;
			gIndices[index++] = (j+1)*NUM_VERTS_X+i+1;
#endif //TEST_INCONSISTENT_WINDING
			
			
			
#else //SHIFT_INDICES
			gIndices[index++] = j*NUM_VERTS_X+i;
			gIndices[index++] = j*NUM_VERTS_X+i+1;
			gIndices[index++] = (j+1)*NUM_VERTS_X+i+1;

			gIndices[index++] = j*NUM_VERTS_X+i;
			gIndices[index++] = (j+1)*NUM_VERTS_X+i+1;
			gIndices[index++] = (j+1)*NUM_VERTS_X+i;
#endif //SHIFT_INDICES

#endif //SWAP_WINDING

			
		}
	}

	m_indexVertexArrays = new btTriangleIndexVertexArray(totalTriangles,
		gIndices,
		indexStride,
		totalVerts,(btScalar*) &gVertices[0].x(),vertStride);

	
	bool useQuantizedAabbCompression = true;

//comment out the next line to read the BVH from disk (first run the demo once to create the BVH)
#define SERIALIZE_TO_DISK 1
#ifdef SERIALIZE_TO_DISK
	btVector3 aabbMin(-1000,-1000,-1000),aabbMax(1000,1000,1000);
	
	trimeshShape  = new btBvhTriangleMeshShape(m_indexVertexArrays,useQuantizedAabbCompression,aabbMin,aabbMax);
	m_collisionShapes.push_back(trimeshShape);
	
	
	///we can serialize the BVH data 
	void* buffer = 0;
	int numBytes = trimeshShape->getOptimizedBvh()->calculateSerializeBufferSize();
	buffer = btAlignedAlloc(numBytes,16);
	bool swapEndian = false;
	trimeshShape->getOptimizedBvh()->serialize(buffer,numBytes,swapEndian);
	FILE* file = fopen("bvh.bin","wb");
	fwrite(buffer,1,numBytes,file);
	fclose(file);
	btAlignedFree(buffer);
	


#else

	trimeshShape  = new btBvhTriangleMeshShape(m_indexVertexArrays,useQuantizedAabbCompression,false);

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
		int read = fread(buffer,1,size,file);
		fclose(file);
		bool swapEndian = false;
		bvh = btOptimizedBvh::deSerializeInPlace(buffer,buffersize,swapEndian);
	}

	trimeshShape->setOptimizedBvh(bvh);

#endif

	btCollisionShape* groundShape = trimeshShape;

	btTriangleInfoMap* triangleInfoMap = new btTriangleInfoMap();
	

	btGenerateInternalEdgeInfo(trimeshShape,triangleInfoMap);
	


#else
	btCollisionShape* groundShape = new btBoxShape(btVector3(50,3,50));

	m_collisionShapes.push_back(groundShape);

#endif //USE_TRIMESH_SHAPE

	m_collisionConfiguration = new btDefaultCollisionConfiguration();
	

	m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);


	
	m_broadphase = new btDbvtBroadphase();
	m_solver = new btSequentialImpulseConstraintSolver();
	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);
/*
m_dynamicsWorld->getSolverInfo().m_splitImpulse = true;
	m_dynamicsWorld->getSolverInfo().m_splitImpulsePenetrationThreshold = 1e30f;
	m_dynamicsWorld->getSolverInfo().m_maxErrorReduction = 1e30f;
	m_dynamicsWorld->getSolverInfo().m_erp  =1.f;
	m_dynamicsWorld->getSolverInfo().m_erp2 = 1.f;
*/

	m_dynamicsWorld->setGravity(btVector3(0,-10,0));

	
	float mass = 0.f;
	btTransform	startTransform;
	startTransform.setIdentity();
	startTransform.setOrigin(btVector3(0,-2,0));


	btConvexHullShape* colShape = new btConvexHullShape();
	for (int i=0;i<TaruVtxCount;i++)
	{
		btVector3 vtx(TaruVtx[i*3],TaruVtx[i*3+1],TaruVtx[i*3+2]);
		colShape->addPoint(vtx);
	}
	//this will enable polyhedral contact clipping, better quality, slightly slower
	colShape->initializePolyhedralFeatures();

	//the polyhedral contact clipping can use either GJK or SAT test to find the separating axis
	m_dynamicsWorld->getDispatchInfo().m_enableSatConvex=false;

	m_collisionShapes.push_back(colShape);

	{
		for (int i=0;i<1;i++)
		{
			startTransform.setOrigin(btVector3(-10.f+i*3.f,2.2f+btScalar(i)*0.1f,-1.3f));
			btRigidBody* body = localCreateRigidBody(10, startTransform,colShape);
			body->setActivationState(DISABLE_DEACTIVATION);
			body->setLinearVelocity(btVector3(0,0,-1));
			//body->setContactProcessingThreshold(0.f);
		}
	}
	{
		btBoxShape* colShape = new btBoxShape(btVector3(1,1,1));
		colShape->initializePolyhedralFeatures();
		m_collisionShapes.push_back(colShape);
		startTransform.setOrigin(btVector3(-16.f+i*3.f,1.f+btScalar(i)*0.1f,-1.3f));
		btRigidBody* body = localCreateRigidBody(10, startTransform,colShape);
		body->setActivationState(DISABLE_DEACTIVATION);
		body->setLinearVelocity(btVector3(0,0,-1));
	}

	startTransform.setIdentity();
#ifdef ROTATE_GROUND
	btQuaternion orn(btVector3(0,0,1),SIMD_PI);
	startTransform.setOrigin(btVector3(-20,0,0));
	startTransform.setRotation(orn);
#endif //ROTATE_GROUND

	staticBody = localCreateRigidBody(mass, startTransform,groundShape);
	//staticBody->setContactProcessingThreshold(-0.031f);
	staticBody->setCollisionFlags(staticBody->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);//STATIC_OBJECT);

	//enable custom material callback
	staticBody->setCollisionFlags(staticBody->getCollisionFlags()  | btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);

	getDynamicsWorld()->setDebugDrawer(&gDebugDrawer);
	setDebugMode(btIDebugDraw::DBG_DrawText|btIDebugDraw::DBG_NoHelpText+btIDebugDraw::DBG_DrawWireframe+btIDebugDraw::DBG_DrawContactPoints);


#ifdef BT_INTERNAL_EDGE_DEBUG_DRAW
	btSetDebugDrawer(&gDebugDrawer);
#endif //BT_INTERNAL_EDGE_DEBUG_DRAW

	
}


void	InternalEdgeDemo::clientResetScene()
{
	DemoApplication::clientResetScene();
	for (int i=0;i<m_dynamicsWorld->getNumCollisionObjects();i++)
	{
		btCollisionObject* colobj = m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(colobj);
		if (body && body->getInvMass() != 0.f)
		{

			body->setLinearVelocity(btVector3(0,0,-1));
		}

	}
}

void InternalEdgeDemo::clientMoveAndDisplay()
{
	 glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	float dt = getDeltaTimeMicroseconds() * 0.000001f;

	if (m_animatedMesh)
	{
		static float offset=0.f;
		offset+=0.01f;

	//	setVertexPositions(waveheight,offset);
#if 0 ///not currently supported, we need to update the btInternalTriangleInfoMap
		int i;
		int j;
		btVector3 aabbMin(BT_LARGE_FLOAT,BT_LARGE_FLOAT,BT_LARGE_FLOAT);
		btVector3 aabbMax(-BT_LARGE_FLOAT,-BT_LARGE_FLOAT,-BT_LARGE_FLOAT);

		for ( i=NUM_VERTS_X/2-3;i<NUM_VERTS_X/2+2;i++)
		{
			for (j=NUM_VERTS_X/2-3;j<NUM_VERTS_Y/2+2;j++)
			{
			
			aabbMax.setMax(gVertices[i+j*NUM_VERTS_X]);
			aabbMin.setMin(gVertices[i+j*NUM_VERTS_X]);
			
				gVertices[i+j*NUM_VERTS_X].setValue((i-NUM_VERTS_X*0.5f)*TRIANGLE_SIZE,
					0.f,
					//waveheight*sinf((float)i+offset)*cosf((float)j+offset),
					(j-NUM_VERTS_Y*0.5f)*TRIANGLE_SIZE);
					
			aabbMin.setMin(gVertices[i+j*NUM_VERTS_X]);
			aabbMax.setMax(gVertices[i+j*NUM_VERTS_X]);

			}
		}
		trimeshShape->partialRefitTree(aabbMin,aabbMax);
#else
		btVector3 aabbMin,aabbMax;
		trimeshShape->getMeshInterface()->calculateAabbBruteForce(aabbMin,aabbMax);
		trimeshShape->refitTree(aabbMin,aabbMax);
	
#endif

		
		//for debugging: clear all contact points involving mesh proxy. Note: this is a slow/unoptimized operation.
		//m_dynamicsWorld->getBroadphase()->getOverlappingPairCache()->cleanProxyFromPairs(staticBody->getBroadphaseHandle(),getDynamicsWorld()->getDispatcher());
	}



	m_dynamicsWorld->stepSimulation(dt);
	///enable one of the following to debug (render debug lines each frame)
	//m_dynamicsWorld->stepSimulation(1./800.,0);
	//m_dynamicsWorld->stepSimulation(1./60.,100,1./800.);
	//m_dynamicsWorld->stepSimulation(1./60.,0);

	
	int lineWidth=450;
	int xStart = m_glutScreenWidth - lineWidth;
	int yStart = 20;

	if((getDebugMode() & btIDebugDraw::DBG_DrawText)!=0)
	{
		setOrthographicProjection();
		glDisable(GL_LIGHTING);
		glColor3f(0, 0, 0);
		char buf[124];
		
		glRasterPos3f(xStart, yStart, 0);
		if (enable)
		{
			sprintf(buf,"InternalEdgeUtility enabled");
		} else
		{
			sprintf(buf,"InternalEdgeUtility disabled");
		}
		GLDebugDrawString(xStart,20,buf);
		yStart+=20;
		glRasterPos3f(xStart, yStart, 0);
		sprintf(buf,"Press 'n' to toggle InternalEdgeUtility");
		yStart+=20;
		GLDebugDrawString(xStart,yStart,buf);
		glRasterPos3f(xStart, yStart, 0);
		
		resetPerspectiveProjection();
		glEnable(GL_LIGHTING);
	}

	
	renderme();

	//optional but useful: debug drawing
	m_dynamicsWorld->debugDrawWorld();


    glFlush();
    swapBuffers();

}




void InternalEdgeDemo::displayCallback(void) {

	clientMoveAndDisplay();
	/*
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	renderme();

	//optional but useful: debug drawing
	if (m_dynamicsWorld)
		m_dynamicsWorld->debugDrawWorld();


    glFlush();
    glutSwapBuffers();
	*/

}



void	InternalEdgeDemo::exitPhysics()
{



	//cleanup in the reverse order of creation/initialization

	//remove the rigidbodies from the dynamics world and delete them
	int i;
	for (i=m_dynamicsWorld->getNumCollisionObjects()-1; i>=0 ;i--)
	{
		btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
			delete body->getMotionState();
		}
		m_dynamicsWorld->removeCollisionObject( obj );
		delete obj;
	}

	//delete collision shapes
	for (int j=0;j<m_collisionShapes.size();j++)
	{
		btCollisionShape* shape = m_collisionShapes[j];
		delete shape;
	}

	//delete dynamics world
	delete m_dynamicsWorld;

	if (m_indexVertexArrays)
		delete m_indexVertexArrays;

	//delete solver
	delete m_solver;

	//delete broadphase
	delete m_broadphase;

	//delete dispatcher
	delete m_dispatcher;

	delete m_collisionConfiguration;

	
}




