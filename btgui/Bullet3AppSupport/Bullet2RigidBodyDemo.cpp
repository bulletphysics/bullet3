#include "Bullet2RigidBodyDemo.h"
#include "btBulletDynamicsCommon.h"
#include "OpenGLWindow/SimpleOpenGL3App.h"
#include "BulletCollision/CollisionShapes/btShapeHull.h"//to create a tesselation of a generic btConvexShape
#include "MyDebugDrawer.h"
struct GraphicsVertex
{
	float pos[4];
	float normal[3];
	float texcoord[2];
};


struct MyGraphicsPhysicsBridge : public GraphicsPhysicsBridge
{
	SimpleOpenGL3App* m_glApp;
	MyDebugDrawer* m_debugDraw;

	MyGraphicsPhysicsBridge(SimpleOpenGL3App* glApp)
		:m_glApp(glApp), m_debugDraw(0)
	{
	}
	virtual void createRigidBodyGraphicsObject(btRigidBody* body, const btVector3& color)
	{
	    createCollisionObjectGraphicsObject(body,color);
	}
	virtual void createCollisionObjectGraphicsObject(btCollisionObject* body, const btVector3& color)
	{
		btCollisionShape* shape = body->getCollisionShape();
		btTransform startTransform = body->getWorldTransform();
		int graphicsShapeId = shape->getUserIndex();
		btAssert(graphicsShapeId >= 0);
		btVector3 localScaling = shape->getLocalScaling();
		int graphicsInstanceId = m_glApp->m_instancingRenderer->registerGraphicsInstance(graphicsShapeId, startTransform.getOrigin(), startTransform.getRotation(), color, localScaling);
		body->setUserIndex(graphicsInstanceId);
	}
	virtual void createCollisionShapeGraphicsObject(btCollisionShape* collisionShape)
	{
		//already has a graphics object?
		if (collisionShape->getUserIndex()>=0)
			return;

		//todo: support all collision shape types
		switch (collisionShape->getShapeType())
		{
		case BOX_SHAPE_PROXYTYPE:
		{
			btBoxShape* box = (btBoxShape*)collisionShape;
			btVector3 halfExtents = box->getHalfExtentsWithMargin();
			int cubeShapeId = m_glApp->registerCubeShape(halfExtents.x(), halfExtents.y(), halfExtents.z());
			box->setUserIndex(cubeShapeId);
			break;
		}
		case TRIANGLE_MESH_SHAPE_PROXYTYPE:
		{

			break;
		}
		default:
		{
			if (collisionShape->isConvex())
			{
				btConvexShape* convex = (btConvexShape*)collisionShape;
				{
					btShapeHull* hull = new btShapeHull(convex);
					hull->buildHull(0.0);

					{
						//int strideInBytes = 9*sizeof(float);
						//int numVertices = hull->numVertices();
						//int numIndices =hull->numIndices();

						btAlignedObjectArray<GraphicsVertex> gvertices;
						btAlignedObjectArray<int> indices;

						for (int t=0;t<hull->numTriangles();t++)
						{

							btVector3 triNormal;

							int index0 = hull->getIndexPointer()[t*3+0];
							int index1 = hull->getIndexPointer()[t*3+1];
							int index2 = hull->getIndexPointer()[t*3+2];
							btVector3 pos0 =hull->getVertexPointer()[index0];
							btVector3 pos1 =hull->getVertexPointer()[index1];
							btVector3 pos2 =hull->getVertexPointer()[index2];
							triNormal = (pos1-pos0).cross(pos2-pos0);
							triNormal.normalize();

							for (int v=0;v<3;v++)
							{
								int index = hull->getIndexPointer()[t*3+v];
								GraphicsVertex vtx;
								btVector3 pos =hull->getVertexPointer()[index];
								vtx.pos[0] = pos.x();
								vtx.pos[1] = pos.y();
								vtx.pos[2] = pos.z();
								vtx.pos[3] = 0.f;

								vtx.normal[0] =triNormal.x();
								vtx.normal[1] =triNormal.y();
								vtx.normal[2] =triNormal.z();

								vtx.texcoord[0] = 0.5f;
								vtx.texcoord[1] = 0.5f;

								indices.push_back(gvertices.size());
								gvertices.push_back(vtx);
							}
						}


						int shapeId = m_glApp->m_instancingRenderer->registerShape(&gvertices[0].pos[0],gvertices.size(),&indices[0],indices.size());
						convex->setUserIndex(shapeId);
					}
				}
			} else
			{
				btAssert(0);
			}
		}
		};
	}
	virtual void syncPhysicsToGraphics(const btDiscreteDynamicsWorld* rbWorld)
	{
		int numCollisionObjects = rbWorld->getNumCollisionObjects();
		for (int i = 0; i<numCollisionObjects; i++)
		{
			btCollisionObject* colObj = rbWorld->getCollisionObjectArray()[i];
			btVector3 pos = colObj->getWorldTransform().getOrigin();
			btQuaternion orn = colObj->getWorldTransform().getRotation();
			int index = colObj->getUserIndex();
			if (index >= 0)
			{
				m_glApp->m_instancingRenderer->writeSingleInstanceTransformToCPU(pos, orn, index);
			}
		}
		m_glApp->m_instancingRenderer->writeTransforms();
	}

	virtual void createPhysicsDebugDrawer(btDiscreteDynamicsWorld* rbWorld)
	{
	    btAssert(rbWorld);
        m_debugDraw = new MyDebugDrawer(m_glApp);
        rbWorld->setDebugDrawer(m_debugDraw );


        m_debugDraw->setDebugMode(
            btIDebugDraw::DBG_DrawWireframe
            +btIDebugDraw::DBG_DrawAabb
            //btIDebugDraw::DBG_DrawContactPoints
            );

	}

	virtual CommonParameterInterface* getParameterInterface()
	{
		return m_glApp->m_parameterInterface;
	}

	virtual void setUpAxis(int axis)
	{
		m_glApp->setUpAxis(axis);
	}
};

Bullet2RigidBodyDemo::Bullet2RigidBodyDemo(SimpleOpenGL3App* app, CommonPhysicsSetup* physicsSetup)
	:	m_physicsSetup(physicsSetup),
	m_controlPressed(false),
	m_altPressed(false),
	m_glApp(app)
{

}
void Bullet2RigidBodyDemo::initPhysics()
{
	MyGraphicsPhysicsBridge glBridge(m_glApp);
	glBridge.setUpAxis(1);
	m_physicsSetup->initPhysics(glBridge);
	m_glApp->m_instancingRenderer->writeTransforms();

}

void Bullet2RigidBodyDemo::exitPhysics()
{

	m_physicsSetup->exitPhysics();

}

void	Bullet2RigidBodyDemo::stepSimulation(float deltaTime)
{
	m_physicsSetup->stepSimulation(deltaTime);

}

void Bullet2RigidBodyDemo::renderScene()
{
	//sync graphics -> physics world transforms

	MyGraphicsPhysicsBridge glBridge(m_glApp);
	m_physicsSetup->syncPhysicsToGraphics(glBridge);

	m_glApp->m_instancingRenderer->renderScene();

}

void	Bullet2RigidBodyDemo::physicsDebugDraw()
{
    m_physicsSetup->debugDraw();
}

Bullet2RigidBodyDemo::~Bullet2RigidBodyDemo()
{
}

btVector3	Bullet2RigidBodyDemo::getRayTo(int x,int y)
{
	if (!m_glApp->m_instancingRenderer)
	{
		btAssert(0);
		return btVector3(0,0,0);
	}

	float top = 1.f;
	float bottom = -1.f;
	float nearPlane = 1.f;
	float tanFov = (top-bottom)*0.5f / nearPlane;
	float fov = b3Scalar(2.0) * b3Atan(tanFov);

	btVector3 camPos,camTarget;
	m_glApp->m_instancingRenderer->getCameraPosition(camPos);
	m_glApp->m_instancingRenderer->getCameraTargetPosition(camTarget);

	btVector3	rayFrom = camPos;
	btVector3 rayForward = (camTarget-camPos);
	rayForward.normalize();
	float farPlane = 10000.f;
	rayForward*= farPlane;

	btVector3 rightOffset;
	btVector3 cameraUp=btVector3(0,0,0);
	cameraUp[m_glApp->getUpAxis()]=1;

	btVector3 vertical = cameraUp;

	btVector3 hor;
	hor = rayForward.cross(vertical);
	hor.normalize();
	vertical = hor.cross(rayForward);
	vertical.normalize();

	float tanfov = tanf(0.5f*fov);


	hor *= 2.f * farPlane * tanfov;
	vertical *= 2.f * farPlane * tanfov;

	b3Scalar aspect;
	float width = m_glApp->m_instancingRenderer->getScreenWidth();
	float height = m_glApp->m_instancingRenderer->getScreenHeight();

	aspect =  width / height;

	hor*=aspect;


	btVector3 rayToCenter = rayFrom + rayForward;
	btVector3 dHor = hor * 1.f/width;
	btVector3 dVert = vertical * 1.f/height;


	btVector3 rayTo = rayToCenter - 0.5f * hor + 0.5f * vertical;
	rayTo += btScalar(x) * dHor;
	rayTo -= btScalar(y) * dVert;
	return rayTo;
}


bool	Bullet2RigidBodyDemo::mouseMoveCallback(float x,float y)
{
	btVector3 rayTo = getRayTo(x, y);
	btVector3 rayFrom;
	m_glApp->m_instancingRenderer->getCameraPosition(rayFrom);
	m_physicsSetup->movePickedBody(rayFrom,rayTo);

	return false;
}

bool	Bullet2RigidBodyDemo::mouseButtonCallback(int button, int state, float x, float y)
{

	if (state==1)
	{
		if(button==0 && (!m_altPressed && !m_controlPressed))
		{
			btVector3 camPos;
			m_glApp->m_instancingRenderer->getCameraPosition(camPos);

			btVector3 rayFrom = camPos;
			btVector3 rayTo = getRayTo(x,y);

			m_physicsSetup->pickBody(rayFrom, rayTo);


		}
	} else
	{
		if (button==0)
		{
			m_physicsSetup->removePickingConstraint();
			//remove p2p
		}
	}

	//printf("button=%d, state=%d\n",button,state);
	return false;
}
