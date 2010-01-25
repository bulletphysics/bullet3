
#include "btBulletWorldImporter.h"
#include "btBulletFile.h"

#include "btBulletDynamicsCommon.h"


btBulletWorldImporter::btBulletWorldImporter(btDynamicsWorld* world)
:m_dynamicsWorld(world),
m_verboseDumpAllTypes(false)
{
}

bool	btBulletWorldImporter::loadFileFromMemory( const char* fileName)
{
	bParse::btBulletFile* bulletFile2 = new bParse::btBulletFile(fileName);

	bool result = loadFileFromMemory(bulletFile2);

	delete bulletFile2;
	
	return result;

}



bool	btBulletWorldImporter::loadFileFromMemory( char* memoryBuffer, int len)
{
	bParse::btBulletFile* bulletFile2 = new bParse::btBulletFile(memoryBuffer,len);

	bool result = loadFileFromMemory(bulletFile2);

	delete bulletFile2;

	return result;
}


bool	btBulletWorldImporter::loadFileFromMemory(  bParse::btBulletFile* bulletFile2)
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
		case CONVEX_HULL_SHAPE_PROXYTYPE:
			{
				btConvexInternalShapeData* bsd = (btConvexInternalShapeData*)shapeData;
				btVector3 implicitShapeDimensions;
				implicitShapeDimensions.deSerializeFloat(bsd->m_implicitShapeDimensions);
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
							btMultiSphereShapeData* mss = (btMultiSphereShapeData*)bsd;
							int numSpheres = mss->m_localPositionArraySize;

							btAlignedObjectArray<btVector3> tmpPos;
							btAlignedObjectArray<btScalar> radii;
							radii.resize(numSpheres);
							tmpPos.resize(numSpheres);
							for (int i=0;i<numSpheres;i++)
							{
								tmpPos[i].deSerializeFloat(mss->m_localPositionArrayPtr[i].m_pos);
								radii[i] = mss->m_localPositionArrayPtr[i].m_radius;
							}
							shape = new btMultiSphereShape(&tmpPos[0],&radii[0],numSpheres);
							break;
						}
					case CONVEX_HULL_SHAPE_PROXYTYPE:
						{
							int sz = sizeof(btConvexHullShapeData);
							int sz2 = sizeof(btConvexInternalShapeData);
							int sz3 = sizeof(btCollisionShapeData);
							btConvexHullShapeData* convexData = (btConvexHullShapeData*)bsd;
							int numPoints = convexData->m_numUnscaledPoints;

							btAlignedObjectArray<btVector3> tmpPoints;
							tmpPoints.resize(numPoints);
							for (int i=0;i<numPoints;i++)
							{
#ifdef BT_USE_DOUBLE_PRECISION
							if (convexData->m_unscaledPointsDoublePtr)
								tmpPoints[i].deSerialize(convexData->m_unscaledPointsDoublePtr[i]);
							if (convexData->m_unscaledPointsFloatPtr)
								tmpPoints[i].deSerializeFloat(convexData->m_unscaledPointsFloatPtr[i]);
#else
							if (convexData->m_unscaledPointsFloatPtr)
								tmpPoints[i].deSerialize(convexData->m_unscaledPointsFloatPtr[i]);
							if (convexData->m_unscaledPointsDoublePtr)
								tmpPoints[i].deSerializeDouble(convexData->m_unscaledPointsDoublePtr[i]);
#endif //BT_USE_DOUBLE_PRECISION
							}
							shape = new btConvexHullShape(&tmpPoints[0].getX(),numPoints,sizeof(btVector3));
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
					localScaling.deSerializeFloat(bsd->m_localScaling);
					shape->setLocalScaling(localScaling);
					
					shapeMap.insert(shapeData,shape);
				}
				break;
			}
		case TRIANGLE_MESH_SHAPE_PROXYTYPE:
		{
			btTriangleMeshShapeData* trimesh = (btTriangleMeshShapeData*)shapeData;
			btTriangleIndexVertexArray* meshInterface = new btTriangleIndexVertexArray();
			for (int i=0;i<trimesh->m_meshInterface.m_numMeshParts;i++)
			{
				btIndexedMesh meshPart;
				if (trimesh->m_meshInterface.m_meshPartsPtr[i].m_indices32)
				{
					meshPart.m_indexType = PHY_INTEGER;
					meshPart.m_triangleIndexStride = 3*sizeof(int);
					meshPart.m_triangleIndexBase = (const unsigned char*)trimesh->m_meshInterface.m_meshPartsPtr[i].m_indices32;
				} else
				{
					meshPart.m_indexType = PHY_SHORT;
					meshPart.m_triangleIndexStride = 3*sizeof(short int);
					meshPart.m_triangleIndexBase = (const unsigned char*)trimesh->m_meshInterface.m_meshPartsPtr[i].m_indices16;
				}

				if (trimesh->m_meshInterface.m_meshPartsPtr[i].m_vertices3f)
				{
					meshPart.m_vertexType = PHY_FLOAT;
					meshPart.m_vertexStride = sizeof(btVector3FloatData);
					meshPart.m_vertexBase = (const unsigned char*)trimesh->m_meshInterface.m_meshPartsPtr[i].m_vertices3f;
				} else
				{
					meshPart.m_vertexType = PHY_DOUBLE;
					meshPart.m_vertexStride = sizeof(btVector3DoubleData);
					meshPart.m_vertexBase = (const unsigned char*)trimesh->m_meshInterface.m_meshPartsPtr[i].m_vertices3d;
				}
				meshPart.m_numTriangles = trimesh->m_meshInterface.m_meshPartsPtr[i].m_numTriangles;
				meshPart.m_numVertices = trimesh->m_meshInterface.m_meshPartsPtr[i].m_numVertices;
				
				meshInterface->addIndexedMesh(meshPart);
			}
			btVector3 scaling; scaling.deSerializeFloat(trimesh->m_meshInterface.m_scaling);
			meshInterface->setScaling(scaling);

			btBvhTriangleMeshShape* trimeshShape = new btBvhTriangleMeshShape(meshInterface,true);
			trimeshShape->setMargin(trimesh->m_collisionMargin);
			shapeMap.insert(shapeData,trimeshShape);

			//printf("trimesh->m_collisionMargin=%f\n",trimesh->m_collisionMargin);
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
		if (bulletFile2->getFlags() & bParse::FD_DOUBLE_PRECISION)
		{
			btRigidBodyDoubleData* colObjData = (btRigidBodyDoubleData*)bulletFile2->m_rigidBodies[i];
			btScalar mass = btScalar(colObjData->m_inverseMass? 1.f/colObjData->m_inverseMass : 0.f);
			btVector3 localInertia;
			localInertia.setZero();
			btCollisionShape** shapePtr = shapeMap.find(colObjData->m_collisionObjectData.m_collisionShape);
			if (shapePtr && *shapePtr)
			{
				btTransform startTransform;
				startTransform.deSerializeDouble(colObjData->m_collisionObjectData.m_worldTransform);
			//	startTransform.setBasis(btMatrix3x3::getIdentity());
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
	
		} else
		{
			btRigidBodyFloatData* colObjData = (btRigidBodyFloatData*)bulletFile2->m_rigidBodies[i];
			btScalar mass = btScalar(colObjData->m_inverseMass? 1.f/colObjData->m_inverseMass : 0.f);
			btVector3 localInertia;
			localInertia.setZero();
			btCollisionShape** shapePtr = shapeMap.find(colObjData->m_collisionObjectData.m_collisionShape);
			if (shapePtr && *shapePtr)
			{
				btTransform startTransform;
				startTransform.deSerializeFloat(colObjData->m_collisionObjectData.m_worldTransform);
			//	startTransform.setBasis(btMatrix3x3::getIdentity());
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
	}

	for (i=0;i<bulletFile2->m_collisionObjects.size();i++)
	{
		btCollisionObjectData* colObjData = (btCollisionObjectData*)bulletFile2->m_collisionObjects[i];
		printf("bla");
	}
	return false;
}

btTypedConstraint*			btBulletWorldImporter::createUniversalD6Constraint(class btRigidBody* body0,class btRigidBody* otherBody,
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
	
btRigidBody*  btBulletWorldImporter::createRigidBody(bool isDynamic, btScalar mass, const btTransform& startTransform,btCollisionShape* shape)
{
	btVector3 localInertia;
	localInertia.setZero();

	if (mass)
		shape->calculateLocalInertia(mass,localInertia);
	
	btRigidBody* body = new btRigidBody(mass,0,shape,localInertia);	
	body->setWorldTransform(startTransform);

	m_dynamicsWorld->addRigidBody(body);
	
	return body;
}

btCollisionShape* btBulletWorldImporter::createPlaneShape(const btVector3& planeNormal,btScalar planeConstant)
{
	return 0;
}
btCollisionShape* btBulletWorldImporter::createBoxShape(const btVector3& halfExtents)
{
	return new btBoxShape(halfExtents);
}
btCollisionShape* btBulletWorldImporter::createSphereShape(btScalar radius)
{
	return new btSphereShape(radius);
}

btCollisionShape* btBulletWorldImporter::createCapsuleShape(btScalar radius, btScalar height)
{
	return new btCapsuleShape(radius,height);
}



btCollisionShape* btBulletWorldImporter::createCylinderShapeY(btScalar radius,btScalar height)
{
	return new btCylinderShape(btVector3(radius,height,radius));
}
btTriangleMesh*	btBulletWorldImporter::createTriangleMeshContainer()
{
	return 0;
}
btCollisionShape* btBulletWorldImporter::createBvhTriangleMeshShape(btTriangleMesh* trimesh)
{
	return 0;
}
btCollisionShape* btBulletWorldImporter::createConvexTriangleMeshShape(btTriangleMesh* trimesh)
{
	return 0;
}
btCollisionShape* btBulletWorldImporter::createGimpactShape(btTriangleMesh* trimesh)
{
	return 0;
}
btConvexHullShape* btBulletWorldImporter::createConvexHullShape()
{
	return 0;
}

btCompoundShape* btBulletWorldImporter::createCompoundShape()
{
	return 0;
}

