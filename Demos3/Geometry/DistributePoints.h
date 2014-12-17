#ifndef DISTRIBUTE_POINTS_H
#define DISTRIBUTE_POINTS_H

#include "Bullet3AppSupport/BulletDemoInterface.h"
#include "OpenGLWindow/CommonGraphicsApp.h"
#include "BulletCollision/CollisionShapes/btConvexHullShape.h"
#include "BulletCollision/CollisionShapes/btConvexPolyhedron.h"
#include "btBulletDynamicsCommon.h"

inline btScalar randRange(btScalar minRange, btScalar maxRange)
{
	return (rand() / (btScalar(RAND_MAX) + btScalar(1.0))) * (maxRange - minRange) + minRange;
}

void myCallback(btBroadphasePair& collisionPair, btCollisionDispatcher& dispatcher, const btDispatcherInfo& dispatchInfo)
{
	if (1)
	{
		btCollisionObject* colObj0 = (btCollisionObject*)collisionPair.m_pProxy0->m_clientObject;
		btCollisionObject* colObj1 = (btCollisionObject*)collisionPair.m_pProxy1->m_clientObject;
		btRigidBody* body0 = btRigidBody::upcast(colObj0);
		btRigidBody* body1 = btRigidBody::upcast(colObj1);
		if (body0 && body1)
		{
			btVector3 vec = body1->getWorldTransform().getOrigin()-body0->getWorldTransform().getOrigin();

			vec.safeNormalize();
			//add a small 'random' direction to avoid getting stuck in a plane
			//vec += 0.00001*btVector3(randRange(0,1),randRange(0,1),randRange(0,1));
			//magnitude: 1./vec.length1();
			btScalar l = vec.length();
			btScalar mag = 1.1/(l*l*l);
			
			body0->applyImpulse(-mag*vec,btVector3(0,0,0));
			body1->applyImpulse(mag*vec,btVector3(0,0,0));
			
		}

	} else
	{
		btCollisionDispatcher::defaultNearCallback(collisionPair,dispatcher,dispatchInfo);
	}
}

class DistributePoints : public BulletDemoInterface
{
    CommonGraphicsApp* m_app;
    float m_x;
    float m_y;
    
	btDefaultCollisionConfiguration* m_collisionConfiguration;
	btCollisionDispatcher* m_dispatcher;
	btDiscreteDynamicsWorld* m_dynamicsWorld;
	btDbvtBroadphase*	m_broadphase;
	btSequentialImpulseConstraintSolver* m_solver;
	
public:
    
    DistributePoints(CommonGraphicsApp* app)
    :m_app(app),
    m_x(0),
    m_y(0)
    {
		m_app->setUpAxis(1);
    	m_collisionConfiguration = new btDefaultCollisionConfiguration();
		m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);
		m_broadphase = new btDbvtBroadphase();
		btSequentialImpulseConstraintSolver* sol = new btSequentialImpulseConstraintSolver;
		m_solver = sol;
		m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher, m_broadphase, m_solver, m_collisionConfiguration);
		m_dispatcher->setNearCallback(myCallback);

		
    }
    virtual ~DistributePoints()
    {
		delete m_dynamicsWorld;
		delete m_solver;
		delete m_broadphase;
		delete m_dispatcher;
		delete m_collisionConfiguration;
    }
    static BulletDemoInterface*    CreateFunc(CommonGraphicsApp* app)
    {
        return new DistributePoints(app);
    }
    
	inline btScalar randRange(btScalar minRange, btScalar maxRange)
	{
		return (rand() / (btScalar(RAND_MAX) + btScalar(1.0))) * (maxRange - minRange) + minRange;
	}
	
    virtual void    initPhysics()
    {
		//create spheres, attached with point to point constraint
		//use a collision callback, apply forces
	

		btScalar radius = 0.01;
		btSphereShape* sphere = new btSphereShape(1);
		int sphereId = m_app->registerGraphicsSphereShape(radius,false);

		for (int i=0;i<256;i++)
		{
			
			btScalar mass =1.f;
			btVector3 localInertia;
			sphere->calculateLocalInertia(mass,localInertia);
			btRigidBody::btRigidBodyConstructionInfo ci(mass,0,sphere,localInertia);
			ci.m_startWorldTransform.setIdentity();
			btVector3 center(randRange(-1,1),randRange(-1,1),randRange(-1,1));
			center.normalize();
			ci.m_startWorldTransform.setOrigin(center);
			btRigidBody* body = new btRigidBody(ci);
			const btVector3& pos = body->getWorldTransform().getOrigin();
			btQuaternion orn = body->getWorldTransform().getRotation();
			btVector4 color(1,0,0,1);
			btVector3 scaling(radius,radius,radius);
			
			int instanceId = m_app->m_renderer->registerGraphicsInstance(sphereId,pos,orn,color,scaling);
			body->setUserIndex(instanceId);
			m_dynamicsWorld->addRigidBody(body);
			btVector3 pivotInA = -body->getWorldTransform().getOrigin();
			btVector3 pivotInB(0,0,0);
	//		btPoint2PointConstraint* p2p = new btPoint2PointConstraint(*body, btTypedConstraint::getFixedBody(),pivotInA,pivotInB);
	//		m_dynamicsWorld->addConstraint(p2p);
			body->setActivationState(DISABLE_DEACTIVATION);
		}
		m_dynamicsWorld->setGravity(btVector3(0,0,0));
	
		m_app->m_renderer->writeTransforms();
		
		
    }
    virtual void    exitPhysics()
    {
        
    }
	
	
	virtual void	stepSimulation(float deltaTime)
    {
		m_dynamicsWorld->stepSimulation(1./60.,0);
		for (int i=0;i<m_dynamicsWorld->getNumCollisionObjects();i++)
		{
			btRigidBody* body = btRigidBody::upcast(m_dynamicsWorld->getCollisionObjectArray()[i]);
			if (body && body->getUserIndex()>=0)
			{
				btTransform pos = body->getWorldTransform();
				pos.setOrigin(pos.getOrigin().normalized());
				body->setWorldTransform(pos);
				body->setAngularVelocity(btVector3(0,0,0));
				body->setLinearVelocity(btVector3(0,0,0));
				
			}
		}
		
		

    }
    virtual void	renderScene()
    {
		//sync transforms
		for (int i=0;i<m_dynamicsWorld->getNumCollisionObjects();i++)
		{
			btRigidBody* body = btRigidBody::upcast(m_dynamicsWorld->getCollisionObjectArray()[i]);
			if (body && body->getUserIndex()>=0)
			{
				const btVector3& pos = body->getWorldTransform().getOrigin();
				btQuaternion orn = body->getWorldTransform().getRotation();
				
				m_app->m_renderer->writeSingleInstanceTransformToCPU(pos,orn,body->getUserIndex());
			}
		}
		
		
		
		m_app->m_renderer->writeTransforms();
		
        m_app->m_renderer->renderScene();
		
        
    }
    virtual void	physicsDebugDraw()
    {
   	
		int lineWidth = 1;
		int pointSize = 2;

		btAlignedObjectArray<btVector3> m_vertices;
		for (int i=0;i<m_dynamicsWorld->getNumCollisionObjects();i++)
		{
			btRigidBody* body = btRigidBody::upcast(m_dynamicsWorld->getCollisionObjectArray()[i]);
			if (body && body->getUserIndex()>=0)
			{
				btTransform pos = body->getWorldTransform();
				m_vertices.push_back(pos.getOrigin());
			}
		}
		
		btConvexHullShape* m_convexHull = new btConvexHullShape(&m_vertices[0].x(),m_vertices.size(),sizeof(btVector3));
		m_convexHull->initializePolyhedralFeatures();
		const btConvexPolyhedron*	poly = m_convexHull->getConvexPolyhedron();
		
		btScalar averageEdgeLength = 0.f;
		btScalar numLengths=0;
		
		for (int p=0;p<poly->m_faces.size();p++)
		{
			for (int f=2;f<poly->m_faces[p].m_indices.size();f++)
			{
				btVector4 color0(0,0,1,1);
				btVector4 color1(0,0,1,1);
				btVector4 color2(0,0,1,1);
				
				int index0=poly->m_faces[p].m_indices[f-2];
				int index1=poly->m_faces[p].m_indices[f-1];
				int index2=poly->m_faces[p].m_indices[f];
				
				btVector3 v0 = poly->m_vertices[index0];
				btVector3 v1 = poly->m_vertices[index1];
				btVector3 v2 = poly->m_vertices[index2];
				btVector3 e0 = v1-v0;
				btVector3 e1 = v2-v1;
				btVector3 e2 = v0-v2;
				btScalar e0Length = e0.length();
				btScalar e1Length = e1.length();
				btScalar e2Length = e2.length();
				averageEdgeLength+=e0Length;
				averageEdgeLength+=e1Length;
				averageEdgeLength+=e2Length;
				numLengths+=3;
			}
		}
		averageEdgeLength/=numLengths;
		btScalar maxLengthDiff = 0.f;
		
		for (int p=0;p<poly->m_faces.size();p++)
		{
			for (int f=2;f<poly->m_faces[p].m_indices.size();f++)
			{
				btVector4 color0(0,0,1,1);
				btVector4 color1(0,0,1,1);
				btVector4 color2(0,0,1,1);
				
				int index0=poly->m_faces[p].m_indices[f-2];
				int index1=poly->m_faces[p].m_indices[f-1];
				int index2=poly->m_faces[p].m_indices[f];
				
				btVector3 v0 = poly->m_vertices[index0];
				btVector3 v1 = poly->m_vertices[index1];
				btVector3 v2 = poly->m_vertices[index2];
				btVector3 e0 = v1-v0;
				btVector3 e1 = v2-v1;
				btVector3 e2 = v0-v2;
				btScalar e0LengthDiff = btFabs(averageEdgeLength-e0.length());
				btScalar e1LengthDiff = btFabs(averageEdgeLength-e1.length());
				btScalar e2LengthDiff = btFabs(averageEdgeLength-e2.length());

				btSetMax(maxLengthDiff,e0LengthDiff);
				btSetMax(maxLengthDiff,e1LengthDiff);
				btSetMax(maxLengthDiff,e2LengthDiff);
				
				m_app->m_renderer->drawLine(&v0.x(),
											&v1.x(),
											color0,lineWidth);
				m_app->m_renderer->drawLine(&v1.x(),
											&v2.x(),
											color1,lineWidth);
				m_app->m_renderer->drawLine(&v2.x(),
											&v0.x(),
											color2,lineWidth);
				
			}
			//printf("maxEdgeLength=%f\n",maxEdgeLength);
		}
		for (int i=0;i<poly->m_vertices.size();i++)
		{
			btVector4 color(1,0,0,1);
			
			m_app->m_renderer->drawPoint(poly->m_vertices[i],color,pointSize);
			
		}
		
		delete m_convexHull;
		
		char msg[1024];
		btScalar targetDist = 2.0f*sqrtf(4.0f/(btScalar)m_vertices.size());
		sprintf(msg,"average Edge Length = %f, maxLengthDiff = %f",averageEdgeLength,maxLengthDiff);		
		b3Printf(msg);
               
    }
    virtual bool	mouseMoveCallback(float x,float y)
    {
		return false;   
    }
    virtual bool	mouseButtonCallback(int button, int state, float x, float y)
    {
        return false;   
    }
    virtual bool	keyboardCallback(int key, int state)
    {
        return false;   
    }
    
};
#endif //DISTRIBUTE_POINTS_H

