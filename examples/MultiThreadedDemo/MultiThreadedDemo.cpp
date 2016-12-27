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
#include "LinearMath/btQuickprof.h"
#include "LinearMath/btIDebugDraw.h"
#include "../CommonInterfaces/CommonParameterInterface.h"
#include <stdio.h> //printf debugging
#include <algorithm>

class btCollisionShape;

#include "CommonRigidBodyMTBase.h"
#include "MultiThreadedDemo.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "btBulletCollisionCommon.h"


#define BT_OVERRIDE

static btScalar gSliderStackRows = 8.0f;
static btScalar gSliderStackColumns = 6.0f;
static btScalar gSliderStackHeight = 15.0f;
static btScalar gSliderGroundHorizontalAmplitude = 0.0f;
static btScalar gSliderGroundVerticalAmplitude = 0.0f;


/// MultiThreadedDemo shows how to setup and use multithreading
class MultiThreadedDemo  : public CommonRigidBodyMTBase
{
    static const int kUpAxis = 1;

	btRigidBody* localCreateRigidBody(btScalar mass, const btTransform& worldTransform, btCollisionShape* colSape);

    btVector3 m_cameraTargetPos;
    float m_cameraPitch;
    float m_cameraYaw;
    float m_cameraDist;
    btRigidBody* m_groundBody;
    btTransform m_groundStartXf;
    float m_groundMovePhase;

    void createStack( const btVector3& pos, btCollisionShape* boxShape, const btVector3& halfBoxSize, int size );
    void createSceneObjects();
    void destroySceneObjects();

public:
    BT_DECLARE_ALIGNED_ALLOCATOR();

    MultiThreadedDemo( struct GUIHelperInterface* helper );

	virtual ~MultiThreadedDemo() {}

    virtual void stepSimulation( float deltaTime ) BT_OVERRIDE
    {
        if ( m_dynamicsWorld )
        {
            if (m_groundBody)
            {
                // update ground
                const float cyclesPerSecond = 1.0f;
                m_groundMovePhase += cyclesPerSecond * deltaTime;
                m_groundMovePhase -= floor( m_groundMovePhase );  // keep phase between 0 and 1
                btTransform xf = m_groundStartXf;
                float gndHOffset = btSin(m_groundMovePhase * SIMD_2_PI) * gSliderGroundHorizontalAmplitude;
                float gndHVel    = btCos(m_groundMovePhase * SIMD_2_PI) * gSliderGroundHorizontalAmplitude * cyclesPerSecond * SIMD_2_PI; // d(gndHOffset)/dt
                float gndVOffset = btSin(m_groundMovePhase * SIMD_2_PI) * gSliderGroundVerticalAmplitude;
                float gndVVel    = btCos(m_groundMovePhase * SIMD_2_PI) * gSliderGroundVerticalAmplitude * cyclesPerSecond * SIMD_2_PI; // d(gndVOffset)/dt
                btVector3 offset(0,0,0);
                btVector3 vel(0,0,0);
                int horizAxis = 2;
                offset[horizAxis] = gndHOffset;
                vel[horizAxis] = gndHVel;
                offset[kUpAxis] = gndVOffset;
                vel[kUpAxis] = gndVVel;
                xf.setOrigin(xf.getOrigin() + offset);
                m_groundBody->setWorldTransform( xf );
                m_groundBody->setLinearVelocity( vel );
            }
            // always step by 1/60 for benchmarking
            m_dynamicsWorld->stepSimulation( 1.0f / 60.0f, 0 );
        }
    }

    virtual void initPhysics() BT_OVERRIDE;
    virtual void resetCamera() BT_OVERRIDE
	{
        m_guiHelper->resetCamera( m_cameraDist,
                                  m_cameraPitch,
                                  m_cameraYaw,
                                  m_cameraTargetPos.x(),
                                  m_cameraTargetPos.y(),
                                  m_cameraTargetPos.z()
                                  );
	}

};


MultiThreadedDemo::MultiThreadedDemo(struct GUIHelperInterface* helper)
    : CommonRigidBodyMTBase( helper )
{
    m_groundBody = NULL;
    m_groundMovePhase = 0.0f;
    m_cameraTargetPos = btVector3( 0.0f, 0.0f, 0.0f );
    m_cameraPitch = 90.0f;
    m_cameraYaw = 30.0f;
    m_cameraDist = 48.0f;
    helper->setUpAxis( kUpAxis );
}


void MultiThreadedDemo::initPhysics()
{
    createEmptyDynamicsWorld();

    m_dynamicsWorld->setGravity( btVector3( 0, -10, 0 ) );

    {
        SliderParams slider( "Stack height", &gSliderStackHeight );
        slider.m_minVal = 1.0f;
        slider.m_maxVal = 30.0f;
        slider.m_clampToIntegers = true;
        m_guiHelper->getParameterInterface()->registerSliderFloatParameter( slider );
    }
    {
        SliderParams slider( "Stack rows", &gSliderStackRows );
        slider.m_minVal = 1.0f;
        slider.m_maxVal = 20.0f;
        slider.m_clampToIntegers = true;
        m_guiHelper->getParameterInterface()->registerSliderFloatParameter( slider );
    }
    {
        SliderParams slider( "Stack columns", &gSliderStackColumns );
        slider.m_minVal = 1.0f;
        slider.m_maxVal = 20.0f;
        slider.m_clampToIntegers = true;
        m_guiHelper->getParameterInterface()->registerSliderFloatParameter( slider );
    }
    {
        // horizontal ground shake
        SliderParams slider( "Ground horiz amp", &gSliderGroundHorizontalAmplitude );
        slider.m_minVal = 0.0f;
        slider.m_maxVal = 1.0f;
        slider.m_clampToNotches = false;
        m_guiHelper->getParameterInterface()->registerSliderFloatParameter( slider );
    }
    {
        // vertical ground shake
        SliderParams slider( "Ground vert amp", &gSliderGroundVerticalAmplitude );
        slider.m_minVal = 0.0f;
        slider.m_maxVal = 1.0f;
        slider.m_clampToNotches = false;
        m_guiHelper->getParameterInterface()->registerSliderFloatParameter( slider );
    }
	
    createSceneObjects();

    m_guiHelper->createPhysicsDebugDrawer( m_dynamicsWorld );
}



btRigidBody* MultiThreadedDemo::localCreateRigidBody(btScalar mass, const btTransform& startTransform, btCollisionShape* shape)
{
    btRigidBody* body = createRigidBody(mass, startTransform, shape);
    if ( mass > 0.0f )
    {
        // prevent bodies from sleeping to make profiling/benchmarking easier
        body->forceActivationState( DISABLE_DEACTIVATION );
    }
    return body;
}


void MultiThreadedDemo::createStack( const btVector3& center, btCollisionShape* boxShape, const btVector3& halfBoxSize, int size )
{
    btTransform trans;
    trans.setIdentity();
    float halfBoxHeight = halfBoxSize.y();
    float halfBoxWidth = halfBoxSize.x();

    for ( int i = 0; i<size; i++ )
    {
        // This constructs a row, from left to right
        int rowSize = size - i;
        for ( int j = 0; j< rowSize; j++ )
        {
            btVector3 pos = center + btVector3( halfBoxWidth*( 1 + j * 2 - rowSize ),
                halfBoxHeight * ( 1 + i * 2),
                0.0f
                );

            trans.setOrigin( pos );
            btScalar mass = 1.f;

            btRigidBody* body = localCreateRigidBody( mass, trans, boxShape );
            body->setFriction(1.0f);
        }
    }
}


void MultiThreadedDemo::createSceneObjects()
{
    {
        // create ground box
        btTransform tr;
        tr.setIdentity();
        tr.setOrigin( btVector3( 0.f, -3.f, 0.f ) );
        m_groundStartXf = tr;

        //either use heightfield or triangle mesh

        btVector3 groundExtents( 400, 400, 400 );
        groundExtents[ kUpAxis ] = 3;
        btCollisionShape* groundShape = new btBoxShape( groundExtents );
        m_collisionShapes.push_back( groundShape );

        //create ground object
        m_groundBody = createKinematicBody( m_groundStartXf, groundShape );
        m_groundBody->forceActivationState( DISABLE_DEACTIVATION );
        m_groundBody->setFriction(1.0f);
    }

    {
        // create walls of cubes
        const btVector3 halfExtents = btVector3( 0.5f, 0.25f, 0.5f );
        int numStackRows = btMax(1, int(gSliderStackRows));
        int numStackCols = btMax(1, int(gSliderStackColumns));
        int stackHeight = 15;
        float stackZSpacing = 3.0f;
        float stackXSpacing = 20.0f;

        btBoxShape* boxShape = new btBoxShape( halfExtents );
        m_collisionShapes.push_back( boxShape );

        for ( int iX = 0; iX < numStackCols; ++iX )
        {
            for ( int iZ = 0; iZ < numStackRows; ++iZ )
            {
                btVector3 center = btVector3( iX * stackXSpacing, 0.0f, ( iZ - numStackRows / 2 ) * stackZSpacing );
                createStack( center, boxShape, halfExtents, stackHeight );
            }
        }
    }
#if 0
    if ( false )
    {
        // destroyer ball
        btTransform sphereTrans;
        sphereTrans.setIdentity();
        sphereTrans.setOrigin( btVector3( 0, 2, 40 ) );
        btSphereShape* ball = new btSphereShape( 2.f );
        m_collisionShapes.push_back( ball );
        btRigidBody* ballBody = localCreateRigidBody( 10000.f, sphereTrans, ball );
        ballBody->setLinearVelocity( btVector3( 0, 0, -10 ) );
    }
#endif
    m_guiHelper->autogenerateGraphicsObjects( m_dynamicsWorld );

}


CommonExampleInterface*    MultiThreadedDemoCreateFunc( struct CommonExampleOptions& options )
{
	return new MultiThreadedDemo(options.m_guiHelper);
}

