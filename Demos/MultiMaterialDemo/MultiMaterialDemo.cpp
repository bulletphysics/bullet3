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
#include "MultiMaterialDemo.h"
#include "GL_ShapeDrawer.h"
#include "GlutStuff.h"

#include "BulletCollision/CollisionShapes/btTriangleShape.h"
#include "BulletCollision/CollisionShapes/btTriangleIndexVertexMaterialArray.h"
#include "BulletCollision/CollisionShapes/btMultimaterialTriangleMeshShape.h"
#include "BulletCollision/CollisionShapes/btMaterial.h"

// Create a custom material, just because we can
class CustomMaterial : public btMaterial
{
public:
    int foo1;
    int foo2;
    CustomMaterial(){}
    CustomMaterial(int a, int b) {foo1 = a; foo2 = b;}
};

// Storage for the vertex data
static btVector3*	gVertices = 0;
// Storage for the face data
static int*	gIndices = 0;
// Storage for the material data
static CustomMaterial* gMaterials = 0;
// Storage for the face -> material index data
static int* gFaceMaterialIndices = 0;

static btBvhTriangleMeshShape* trimeshShape =0;
static btRigidBody* staticBody = 0;
static float waveheight = 0.f;

const float TRIANGLE_SIZE=1.f;


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



static bool CustomMaterialCombinerCallback(btManifoldPoint& cp,	const btCollisionObject* colObj0,int partId0,int index0,const btCollisionObject* colObj1,int partId1,int index1)
{
    
    // Apply material properties
    if (colObj0->getCollisionShape()->getShapeType() == TRIANGLE_SHAPE_PROXYTYPE)
    {
        const btCollisionShape* parent0 = colObj0->getRootCollisionShape();
        if(parent0 != 0 && parent0->getShapeType() == MULTIMATERIAL_TRIANGLE_MESH_PROXYTYPE)
        {
            btMultimaterialTriangleMeshShape* shape = (btMultimaterialTriangleMeshShape*)parent0;
            const btMaterial * props = shape->getMaterialProperties(partId0, index0);
            cp.m_combinedFriction = calculateCombinedFriction(props->m_friction, colObj1->getFriction());
            cp.m_combinedRestitution = props->m_restitution * colObj1->getRestitution();
        }
    }
    else if (colObj1->getCollisionShape()->getShapeType() == TRIANGLE_SHAPE_PROXYTYPE)
    {
        const btCollisionShape* parent1 = colObj1->getRootCollisionShape();
        if(parent1 != 0 && parent1->getShapeType() == MULTIMATERIAL_TRIANGLE_MESH_PROXYTYPE)
        {
            btMultimaterialTriangleMeshShape* shape = (btMultimaterialTriangleMeshShape*)parent1;
            const btMaterial * props = shape->getMaterialProperties(partId1, index1);
            cp.m_combinedFriction = calculateCombinedFriction(props->m_friction, colObj0->getFriction());
            cp.m_combinedRestitution = props->m_restitution * colObj0->getRestitution();
        }
    }

    //this return value is currently ignored, but to be on the safe side: return false if you don't calculate friction
    return true;
}

extern ContactAddedCallback		gContactAddedCallback;

const int NUM_VERTS_X = 20;
const int NUM_VERTS_Y = 50;
const int totalVerts = NUM_VERTS_X*NUM_VERTS_Y;

void MultiMaterialDemo::setVertexPositions(float waveheight, float offset)
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

void MultiMaterialDemo::keyboardCallback(unsigned char key, int x, int y)
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

void	MultiMaterialDemo::initPhysics()
{
#define TRISIZE 50.f

    gContactAddedCallback = CustomMaterialCombinerCallback;

    // The number of triangles
    const int totalTriangles = 2*(NUM_VERTS_X-1)*(NUM_VERTS_Y-1);
    // The number of materials
    const int totalMaterials = 2;

    int vertStride = sizeof(btVector3);
    int indexStride = 3*sizeof(int);
    int materialStride = sizeof(CustomMaterial);
    int triangleMaterialStride = sizeof(int);

    gVertices = new btVector3[totalVerts];
    gIndices = new int[totalTriangles*3];
    gMaterials = new CustomMaterial[totalMaterials];
    gFaceMaterialIndices = new int[totalTriangles];

    // Explicitly set up the materials.  It's a small array so let's do it bit by bit.
    gMaterials[0].m_friction = 0;
    gMaterials[0].m_restitution = 0.9;
    gMaterials[0].foo1 = 5;
    gMaterials[0].foo2 = 7;
    gMaterials[1].m_friction = 0.9;
    gMaterials[1].m_restitution = 0.1;
    gMaterials[1].foo1 = 53;
    gMaterials[1].foo2 = 15;

    int i;
    // Set up the vertex data
    setVertexPositions(waveheight,0.f);
    int index=0;
    // Set up the face data
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

    // Set up the face->material index data
    for(int a = 0; a < totalTriangles; a++)
    {
        // This will give the first half of the faces low friction and high restitution
        // and the second half of the faces high friction and low restitution
        if(a > totalTriangles*0.5f)
            gFaceMaterialIndices[a] = 0;
        else
            gFaceMaterialIndices[a] = 1;
    }

    // Create the array structure
    m_indexVertexArrays = new btTriangleIndexVertexMaterialArray(
        totalTriangles, gIndices, indexStride,
        totalVerts,(btScalar*) &gVertices[0].x(),vertStride,
        totalMaterials, (unsigned char *)gMaterials, sizeof(CustomMaterial),
        gFaceMaterialIndices, sizeof(int));

    bool useQuantizedAabbCompression = true;
    // Create the multimaterial mesh shape
    trimeshShape  = new btMultimaterialTriangleMeshShape((btTriangleIndexVertexMaterialArray*)m_indexVertexArrays,useQuantizedAabbCompression);
    m_collisionShapes.push_back(trimeshShape);

    btCollisionShape* groundShape = trimeshShape;

    m_collisionConfiguration = new btDefaultCollisionConfiguration();

    m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);

    btVector3 worldMin(-1000,-1000,-1000);
    btVector3 worldMax(1000,1000,1000);
    m_broadphase = new btAxisSweep3(worldMin,worldMax);
    m_solver = new btSequentialImpulseConstraintSolver();
    m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);

    float mass = 0.f;
    btTransform	startTransform;
    startTransform.setIdentity();
    startTransform.setOrigin(btVector3(0,-2,0));

    btCollisionShape* colShape = new btBoxShape(btVector3(0.5f,0.5f,0.5f));
    m_collisionShapes.push_back(colShape);

    {
        for (int i=0;i<1;i++)
        {
            startTransform.setOrigin(btVector3(10,10,-20));
            btRigidBody* body = localCreateRigidBody(1, startTransform,colShape);
            body->setCollisionFlags(body->getCollisionFlags()  | btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);
            body->setFriction(0.9f);
            body->setGravity(btVector3(0,-20.f,0));
            body->applyCentralImpulse(btVector3(-7.7f,0,0));
        }
    }

    startTransform.setIdentity();
    staticBody = localCreateRigidBody(mass, startTransform,groundShape);

    staticBody->setCollisionFlags(staticBody->getCollisionFlags() | btCollisionObject::CF_STATIC_OBJECT);

    //enable custom material callback
    staticBody->setCollisionFlags(staticBody->getCollisionFlags()  | btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);
}

void MultiMaterialDemo::clientMoveAndDisplay()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

    float dt = getDeltaTimeMicroseconds() * 0.000001f;

    if (m_animatedMesh)
    {
        static float offset=0.f;
        offset+=0.01f;

        //	setVertexPositions(waveheight,offset);

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
                    (j-NUM_VERTS_Y*0.5f)*TRIANGLE_SIZE);

                aabbMin.setMin(gVertices[i+j*NUM_VERTS_X]);
                aabbMax.setMax(gVertices[i+j*NUM_VERTS_X]);

            }
        }

        trimeshShape->partialRefitTree(aabbMin,aabbMax);

        //clear all contact points involving mesh proxy. Note: this is a slow/unoptimized operation.
        m_dynamicsWorld->getBroadphase()->getOverlappingPairCache()->cleanProxyFromPairs(staticBody->getBroadphaseHandle(),getDynamicsWorld()->getDispatcher());
    }

    m_dynamicsWorld->stepSimulation(dt);

    //optional but useful: debug drawing
    m_dynamicsWorld->debugDrawWorld();

    renderme();

    glFlush();
    glutSwapBuffers();

}




void MultiMaterialDemo::displayCallback(void) {

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

    renderme();

    glFlush();
    glutSwapBuffers();
}



void	MultiMaterialDemo::exitPhysics()
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




