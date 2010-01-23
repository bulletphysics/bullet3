
#include "btBulletFileLoader.h"
#include "btBulletFile.h"

#include "btBulletDynamicsCommon.h"

btBulletFileLoader::btBulletFileLoader(btDynamicsWorld* world)
:m_dynamicsWorld(world),
m_verboseDumpAllTypes(false)
{
}

bool	btBulletFileLoader::loadFileFromMemory( char* fileName)
{
	bParse::btBulletFile* bulletFile2 = new bParse::btBulletFile("testFile.bullet");

	bool result = loadFileFromMemory(bulletFile2);

	delete bulletFile2;
	
	return result;

}



bool	btBulletFileLoader::loadFileFromMemory( char* memoryBuffer, int len)
{
	bParse::btBulletFile* bulletFile2 = new bParse::btBulletFile(memoryBuffer,len);

	bool result = loadFileFromMemory(bulletFile2);

	delete bulletFile2;

	return result;
}


bool	btBulletFileLoader::loadFileFromMemory(  bParse::btBulletFile* bulletFile2)
{
	

	bool ok = (bulletFile2->getFlags()& bParse::FD_OK)!=0;
	
	if (ok)
		bulletFile2->parse(m_verboseDumpAllTypes);
	else 
		return false;
	
	if (m_verboseDumpAllTypes)
	{
		bulletFile2->dumpChunks(bulletFile2->getFileDNA());
	}

	int i;
	btHashMap<btHashPtr,btCollisionShape*>	shapeMap;

	for (i=0;i<bulletFile2->m_collisionShapes.size();i++)
	{
		btCollisionShapeData* shapeData = (btCollisionShapeData*)bulletFile2->m_collisionShapes[i];
		switch (shapeData->m_shapeType)
		{
		case CYLINDER_SHAPE_PROXYTYPE:
		case CAPSULE_SHAPE_PROXYTYPE:
		case BOX_SHAPE_PROXYTYPE:
		case SPHERE_SHAPE_PROXYTYPE:
		case MULTI_SPHERE_SHAPE_PROXYTYPE:
			{
				btConvexInternalShapeData* bsd = (btConvexInternalShapeData*)shapeData;
				btVector3 implicitShapeDimensions;
				implicitShapeDimensions.deSerialize(bsd->m_implicitShapeDimensions);
				btVector3 margin(bsd->m_collisionMargin,bsd->m_collisionMargin,bsd->m_collisionMargin);
				btCollisionShape* shape = 0;
				switch (shapeData->m_shapeType)
				{
					case BOX_SHAPE_PROXYTYPE:
						{
							shape = createBoxShape(implicitShapeDimensions+margin);
							break;
						}
					case SPHERE_SHAPE_PROXYTYPE:
						{
							shape = createSphereShape(implicitShapeDimensions.getX());
							break;
						}
					case CAPSULE_SHAPE_PROXYTYPE:
						{
							shape = createCapsuleShape(implicitShapeDimensions.getX(),implicitShapeDimensions.getY());
							break;
						}
					case CYLINDER_SHAPE_PROXYTYPE:
						{
							btVector3 halfExtents = implicitShapeDimensions+margin;

							shape = createCylinderShapeY(halfExtents.getX(),halfExtents.getY());
							break;
						}
					case MULTI_SPHERE_SHAPE_PROXYTYPE:
						{
#if 1
							btMultiSphereShapeData* mss = (btMultiSphereShapeData*)bsd;
							int numSpheres = mss->m_localPositionArraySize;

							btAlignedObjectArray<btVector3> tmpPos;
							btAlignedObjectArray<btScalar> radii;
							radii.resize(numSpheres);
							tmpPos.resize(numSpheres);

							for (int i=0;i<numSpheres;i++)
							{
								tmpPos[i].deSerialize(mss->m_localPositionArrayPtr[i].m_pos);
								radii[i] = mss->m_localPositionArrayPtr[i].m_radius;
							}

							
							shape = new btMultiSphereShape(&tmpPos[0],&radii[0],numSpheres);
#endif 
							break;
						}
					default:
						{
							printf("error: cannot create shape type (%d)\n",shapeData->m_shapeType);
						}
				}

				if (shape)
				{
					shape->setMargin(bsd->m_collisionMargin);
					btVector3 localScaling;
					localScaling.deSerialize(bsd->m_localScaling);
					shape->setLocalScaling(localScaling);
					
					shapeMap.insert(shapeData,shape);
				}
				break;
			}
		default:
			{
				printf("unsupported shape type (%d)\n",shapeData->m_shapeType);
			}
		}
		
	}
	for (i=0;i<bulletFile2->m_rigidBodies.size();i++)
	{
		btRigidBodyData* colObjData = (btRigidBodyData*)bulletFile2->m_rigidBodies[i];
		btScalar mass = colObjData->m_inverseMass? 1.f/colObjData->m_inverseMass : 0.f;
		btVector3 localInertia;
		localInertia.setZero();
		btCollisionShape** shapePtr = shapeMap.find(colObjData->m_collisionObjectData.m_collisionShape);
		if (shapePtr && *shapePtr)
		{
			btTransform startTransform;
			startTransform.deSerialize(colObjData->m_collisionObjectData.m_worldTransform);

			btCollisionShape* shape = (btCollisionShape*)*shapePtr;
			
			if (mass)
			{
				shape->calculateLocalInertia(mass,localInertia);
			}
			
			bool isDynamic = mass!=0.f;
			createRigidBody(isDynamic,mass,startTransform,shape);
			
		} else
		{
			printf("error: no shape found\n");
		}
	}

	for (i=0;i<bulletFile2->m_collisionObjects.size();i++)
	{
		btCollisionObjectData* colObjData = (btCollisionObjectData*)bulletFile2->m_collisionObjects[i];
		printf("bla");
	}
	return false;
}

btTypedConstraint*			btBulletFileLoader::createUniversalD6Constraint(class btRigidBody* body0,class btRigidBody* otherBody,
			btTransform& localAttachmentFrameRef,
			btTransform& localAttachmentOther,
			const btVector3& linearMinLimits,
			const btVector3& linearMaxLimits,
			const btVector3& angularMinLimits,
			const btVector3& angularMaxLimits,
			bool disableCollisionsBetweenLinkedBodies)
{
	return 0;
}
	
btRigidBody*  btBulletFileLoader::createRigidBody(bool isDynamic, float mass, const btTransform& startTransform,btCollisionShape* shape)
{
	btVector3 localInertia;

	if (mass)
		shape->calculateLocalInertia(mass,localInertia);
	
	btRigidBody* body = new btRigidBody(mass,0,shape,localInertia);	
	body->setWorldTransform(startTransform);

	m_dynamicsWorld->addRigidBody(body);
	
	return body;
}

btCollisionShape* btBulletFileLoader::createPlaneShape(const btVector3& planeNormal,btScalar planeConstant)
{
	return 0;
}
btCollisionShape* btBulletFileLoader::createBoxShape(const btVector3& halfExtents)
{
	return new btBoxShape(halfExtents);
}
btCollisionShape* btBulletFileLoader::createSphereShape(btScalar radius)
{
	return new btSphereShape(radius);
}

btCollisionShape* btBulletFileLoader::createCapsuleShape(btScalar radius, btScalar height)
{
	return new btCapsuleShape(radius,height);
}



btCollisionShape* btBulletFileLoader::createCylinderShapeY(btScalar radius,btScalar height)
{
	return new btCylinderShape(btVector3(radius,height,radius));
}
btTriangleMesh*	btBulletFileLoader::createTriangleMeshContainer()
{
	return 0;
}
btCollisionShape* btBulletFileLoader::createBvhTriangleMeshShape(btTriangleMesh* trimesh)
{
	return 0;
}
btCollisionShape* btBulletFileLoader::createConvexTriangleMeshShape(btTriangleMesh* trimesh)
{
	return 0;
}
btCollisionShape* btBulletFileLoader::createGimpactShape(btTriangleMesh* trimesh)
{
	return 0;
}
btConvexHullShape* btBulletFileLoader::createConvexHullShape()
{
	return 0;
}

btCompoundShape* btBulletFileLoader::createCompoundShape()
{
	return 0;
}