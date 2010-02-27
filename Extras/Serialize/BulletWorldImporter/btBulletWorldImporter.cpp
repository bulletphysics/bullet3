
#include "btBulletWorldImporter.h"
#include "btBulletFile.h"

#include "btBulletDynamicsCommon.h"
#include "BulletCollision/Gimpact/btGImpactShape.h"

//#define USE_INTERNAL_EDGE_UTILITY
#ifdef USE_INTERNAL_EDGE_UTILITY
#include "BulletCollision/CollisionDispatch/btInternalEdgeUtility.h"
#endif //USE_INTERNAL_EDGE_UTILITY

btBulletWorldImporter::btBulletWorldImporter(btDynamicsWorld* world)
:m_dynamicsWorld(world),
m_verboseDumpAllTypes(false)
{
}

btBulletWorldImporter::~btBulletWorldImporter()
{
}

void btBulletWorldImporter::deleteAllData()
{
	int i;
	for (i=0;i<m_allocatedCollisionShapes.size();i++)
	{
		delete m_allocatedCollisionShapes[i];
	}
	m_allocatedCollisionShapes.clear();

	for (i=0;i<m_allocatedRigidBodies.size();i++)
	{
		delete m_allocatedRigidBodies[i];
	}
	m_allocatedRigidBodies.clear();

	for (i=0;i<m_allocatedConstraints.size();i++)
	{
		delete m_allocatedConstraints[i];
	}
	m_allocatedConstraints.clear();

	for (i=0;i<m_allocatedBvhs.size();i++)
	{
		delete m_allocatedBvhs[i];
	}
	m_allocatedBvhs.clear();
	
	for (i=0;i<m_allocatedTriangleInfoMaps.size();i++)
	{
		delete m_allocatedTriangleInfoMaps[i];
	}
	m_allocatedTriangleInfoMaps.clear();
	for (i=0;i<m_allocatedTriangleIndexArrays.size();i++)
	{
		delete m_allocatedTriangleIndexArrays[i];
	}
	m_allocatedTriangleIndexArrays.clear();
	for (i=0;i<m_allocatedNames.size();i++)
	{
		delete m_allocatedNames[i];
	}
	m_allocatedNames.clear();

}


bool	btBulletWorldImporter::loadFile( const char* fileName)
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

btTriangleIndexVertexArray* btBulletWorldImporter::createMeshInterface(btStridingMeshInterfaceData&  meshData)
{
	btTriangleIndexVertexArray* meshInterface = createTriangleMeshContainer();

	for (int i=0;i<meshData.m_numMeshParts;i++)
	{
		btIndexedMesh meshPart;
		if (meshData.m_meshPartsPtr[i].m_indices32)
		{
			meshPart.m_indexType = PHY_INTEGER;
			meshPart.m_triangleIndexStride = 3*sizeof(int);
			meshPart.m_triangleIndexBase = (const unsigned char*)meshData.m_meshPartsPtr[i].m_indices32;
		} else
		{
			meshPart.m_indexType = PHY_SHORT;
			meshPart.m_triangleIndexStride = sizeof(btShortIntIndexTripletData);
			meshPart.m_triangleIndexBase = (const unsigned char*)meshData.m_meshPartsPtr[i].m_3indices16;
		}

		if (meshData.m_meshPartsPtr[i].m_vertices3f)
		{
			meshPart.m_vertexType = PHY_FLOAT;
			meshPart.m_vertexStride = sizeof(btVector3FloatData);
			meshPart.m_vertexBase = (const unsigned char*)meshData.m_meshPartsPtr[i].m_vertices3f;
		} else
		{
			meshPart.m_vertexType = PHY_DOUBLE;
			meshPart.m_vertexStride = sizeof(btVector3DoubleData);
			meshPart.m_vertexBase = (const unsigned char*)meshData.m_meshPartsPtr[i].m_vertices3d;
		}
		meshPart.m_numTriangles = meshData.m_meshPartsPtr[i].m_numTriangles;
		meshPart.m_numVertices = meshData.m_meshPartsPtr[i].m_numVertices;
		

		meshInterface->addIndexedMesh(meshPart,meshPart.m_indexType);
	}

	return meshInterface;
}

#ifdef USE_INTERNAL_EDGE_UTILITY
extern ContactAddedCallback		gContactAddedCallback;

static bool btAdjustInternalEdgeContactsCallback(btManifoldPoint& cp,	const btCollisionObject* colObj0,int partId0,int index0,const btCollisionObject* colObj1,int partId1,int index1)
{

	btAdjustInternalEdgeContacts(cp,colObj1,colObj0, partId1,index1);
		//btAdjustInternalEdgeContacts(cp,colObj1,colObj0, partId1,index1, BT_TRIANGLE_CONVEX_BACKFACE_MODE);
		//btAdjustInternalEdgeContacts(cp,colObj1,colObj0, partId1,index1, BT_TRIANGLE_CONVEX_DOUBLE_SIDED+BT_TRIANGLE_CONCAVE_DOUBLE_SIDED);
	return true;
}
#endif //USE_INTERNAL_EDGE_UTILITY

btCollisionShape* btBulletWorldImporter::convertCollisionShape(  btCollisionShapeData* shapeData  )
{
	btCollisionShape* shape = 0;

	switch (shapeData->m_shapeType)
		{
	case STATIC_PLANE_PROXYTYPE:
		{
			btStaticPlaneShapeData* planeData = (btStaticPlaneShapeData*)shapeData;
			btVector3 planeNormal,localScaling;
			planeNormal.deSerializeFloat(planeData->m_planeNormal);
			localScaling.deSerializeFloat(planeData->m_localScaling);
			shape = createPlaneShape(planeNormal,planeData->m_planeConstant);
			shape->setLocalScaling(localScaling);

			break;
		}
	case GIMPACT_SHAPE_PROXYTYPE:
		{
			btGImpactMeshShapeData* gimpactData = (btGImpactMeshShapeData*) shapeData;
			if (gimpactData->m_gimpactSubType == CONST_GIMPACT_TRIMESH_SHAPE)
			{
				btTriangleIndexVertexArray* meshInterface = createMeshInterface(gimpactData->m_meshInterface);
				btGImpactMeshShape* gimpactShape = createGimpactShape(meshInterface);
				btVector3 localScaling;
				localScaling.deSerializeFloat(gimpactData->m_localScaling);
				gimpactShape->setLocalScaling(localScaling);
				gimpactShape->setMargin(btScalar(gimpactData->m_collisionMargin));
				gimpactShape->updateBound();
				shape = gimpactShape;
			} else
			{
				printf("unsupported gimpact sub type\n");
			}
			break;
		}

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
				btVector3 localScaling;
				localScaling.deSerializeFloat(bsd->m_localScaling);
				btVector3 margin(bsd->m_collisionMargin,bsd->m_collisionMargin,bsd->m_collisionMargin);
				switch (shapeData->m_shapeType)
				{
					case BOX_SHAPE_PROXYTYPE:
						{
							shape = createBoxShape(implicitShapeDimensions/localScaling+margin);
							break;
						}
					case SPHERE_SHAPE_PROXYTYPE:
						{
							shape = createSphereShape(implicitShapeDimensions.getX());
							break;
						}
					case CAPSULE_SHAPE_PROXYTYPE:
						{
							btCapsuleShapeData* capData = (btCapsuleShapeData*)shapeData;
							switch (capData->m_upAxis)
							{
							case 0:
								{
									shape = createCapsuleShapeX(implicitShapeDimensions.getY(),2*implicitShapeDimensions.getX());
									break;
								}
							case 1:
								{
									shape = createCapsuleShapeY(implicitShapeDimensions.getX(),2*implicitShapeDimensions.getY());
									break;
								}
							case 2:
								{
									shape = createCapsuleShapeZ(implicitShapeDimensions.getX(),2*implicitShapeDimensions.getZ());
									break;
								}
							default:
								{
									printf("error: wrong up axis for btCapsuleShape\n");
								}

							};
							
							break;
						}
					case CYLINDER_SHAPE_PROXYTYPE:
						{
							btCylinderShapeData* cylData = (btCylinderShapeData*) shapeData;
							btVector3 halfExtents = implicitShapeDimensions+margin;
							switch (cylData->m_upAxis)
							{
							case 0:
								{
									shape = createCylinderShapeX(halfExtents.getY(),halfExtents.getX());
									break;
								}
							case 1:
								{
									shape = createCylinderShapeY(halfExtents.getX(),halfExtents.getY());
									break;
								}
							case 2:
								{
									shape = createCylinderShapeZ(halfExtents.getX(),halfExtents.getZ());
									break;
								}
							default:
								{
									printf("unknown Cylinder up axis\n");
								}

							};
							

							
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
							int i;
							for ( i=0;i<numSpheres;i++)
							{
								tmpPos[i].deSerializeFloat(mss->m_localPositionArrayPtr[i].m_pos);
								radii[i] = mss->m_localPositionArrayPtr[i].m_radius;
							}
							shape = new btMultiSphereShape(&tmpPos[0],&radii[0],numSpheres);
							break;
						}
					case CONVEX_HULL_SHAPE_PROXYTYPE:
						{
						//	int sz = sizeof(btConvexHullShapeData);
						//	int sz2 = sizeof(btConvexInternalShapeData);
						//	int sz3 = sizeof(btCollisionShapeData);
							btConvexHullShapeData* convexData = (btConvexHullShapeData*)bsd;
							int numPoints = convexData->m_numUnscaledPoints;

							btAlignedObjectArray<btVector3> tmpPoints;
							tmpPoints.resize(numPoints);
							int i;
							for ( i=0;i<numPoints;i++)
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
							btConvexHullShape* hullShape = createConvexHullShape();
							for (i=0;i<numPoints;i++)
							{
								hullShape->addPoint(tmpPoints[i]);
							}
							shape = hullShape;
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
					
				}
				break;
			}
		case TRIANGLE_MESH_SHAPE_PROXYTYPE:
		{
			btTriangleMeshShapeData* trimesh = (btTriangleMeshShapeData*)shapeData;
			btTriangleIndexVertexArray* meshInterface = createMeshInterface(trimesh->m_meshInterface);

			btVector3 scaling; scaling.deSerializeFloat(trimesh->m_meshInterface.m_scaling);
			meshInterface->setScaling(scaling);


			btOptimizedBvh* bvh = 0;
#if 1
			if (trimesh->m_quantizedFloatBvh)
			{
				btOptimizedBvh** bvhPtr = m_bvhMap.find(trimesh->m_quantizedFloatBvh);
				if (bvhPtr && *bvhPtr)
				{
					bvh = *bvhPtr;
				} else
				{
					bvh = createOptimizedBvh();
					bvh->deSerializeFloat(*trimesh->m_quantizedFloatBvh);
				}
			}
			if (trimesh->m_quantizedDoubleBvh)
			{
				btOptimizedBvh** bvhPtr = m_bvhMap.find(trimesh->m_quantizedDoubleBvh);
				if (bvhPtr && *bvhPtr)
				{
					bvh = *bvhPtr;
				} else
				{
					bvh = createOptimizedBvh();
					bvh->deSerializeDouble(*trimesh->m_quantizedDoubleBvh);
				}
			}
#endif


			btBvhTriangleMeshShape* trimeshShape = createBvhTriangleMeshShape(meshInterface,bvh);
			trimeshShape->setMargin(trimesh->m_collisionMargin);
			shape = trimeshShape;

			if (trimesh->m_triangleInfoMap)
			{
				btTriangleInfoMap* map = createTriangleInfoMap();
				map->deSerialize(*trimesh->m_triangleInfoMap);
				trimeshShape->setTriangleInfoMap(map);

#ifdef USE_INTERNAL_EDGE_UTILITY
				gContactAddedCallback = btAdjustInternalEdgeContactsCallback;
#endif //USE_INTERNAL_EDGE_UTILITY

			}

			//printf("trimesh->m_collisionMargin=%f\n",trimesh->m_collisionMargin);
			break;
		}
		case COMPOUND_SHAPE_PROXYTYPE:
			{
				btCompoundShapeData* compoundData = (btCompoundShapeData*)shapeData;
				btCompoundShape* compoundShape = createCompoundShape();


				btAlignedObjectArray<btCollisionShape*> childShapes;
				for (int i=0;i<compoundData->m_numChildShapes;i++)
				{
					btCollisionShape* childShape = convertCollisionShape(compoundData->m_childShapePtr[i].m_childShape);
					if (childShape)
					{
						btTransform localTransform;
						localTransform.deSerializeFloat(compoundData->m_childShapePtr[i].m_transform);
						compoundShape->addChildShape(localTransform,childShape);
					} else
					{
						printf("error: couldn't create childShape for compoundShape\n");
					}
					
				}
				shape = compoundShape;

				break;
			}
		default:
			{
				printf("unsupported shape type (%d)\n",shapeData->m_shapeType);
			}
		}

		return shape;
	
}

char* btBulletWorldImporter::duplicateName(const char* name)
{
	if (name)
	{
		int l = strlen(name);
		char* newName = new char[l+1];
		memcpy(newName,name,l);
		newName[l] = 0;
		m_allocatedNames.push_back(newName);
		return newName;
	}
	return 0;
}


bool	btBulletWorldImporter::loadFileFromMemory(  bParse::btBulletFile* bulletFile2)
{
	
	int i;
	bool ok = (bulletFile2->getFlags()& bParse::FD_OK)!=0;
	
	if (ok)
		bulletFile2->parse(m_verboseDumpAllTypes);
	else 
		return false;
	
	if (m_verboseDumpAllTypes)
	{
		bulletFile2->dumpChunks(bulletFile2->getFileDNA());
	}

	for (i=0;i<bulletFile2->m_bvhs.size();i++)
	{
		btOptimizedBvh* bvh = createOptimizedBvh();

		if (bulletFile2->getFlags() & bParse::FD_DOUBLE_PRECISION)
		{
			btQuantizedBvhDoubleData* bvhData = (btQuantizedBvhDoubleData*)bulletFile2->m_bvhs[i];
			bvh->deSerializeDouble(*bvhData);
		} else
		{
			btQuantizedBvhFloatData* bvhData = (btQuantizedBvhFloatData*)bulletFile2->m_bvhs[i];
			bvh->deSerializeFloat(*bvhData);
		}
		m_bvhMap.insert(bulletFile2->m_bvhs[i],bvh);
	}



	btHashMap<btHashPtr,btCollisionShape*>	shapeMap;

	for (i=0;i<bulletFile2->m_collisionShapes.size();i++)
	{
		btCollisionShapeData* shapeData = (btCollisionShapeData*)bulletFile2->m_collisionShapes[i];
		btCollisionShape* shape = convertCollisionShape(shapeData);
		if (shape)
		{
	//		printf("shapeMap.insert(%x,%x)\n",shapeData,shape);
			shapeMap.insert(shapeData,shape);
		}

		if (shape&& shapeData->m_name)
		{
			char* newname = duplicateName(shapeData->m_name);
			m_objectNameMap.insert(shape,newname);
			m_nameShapeMap.insert(newname,shape);
		}
	}

	btHashMap<btHashPtr,btCollisionObject*>	bodyMap;

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
				
				btRigidBody* body = createRigidBody(isDynamic,mass,startTransform,shape,colObjData->m_collisionObjectData.m_name);
#ifdef USE_INTERNAL_EDGE_UTILITY
				if (shape->getShapeType() == TRIANGLE_MESH_SHAPE_PROXYTYPE)
				{
					btBvhTriangleMeshShape* trimesh = (btBvhTriangleMeshShape*)shape;
					if (trimesh->getTriangleInfoMap())
					{
						body->setCollisionFlags(body->getCollisionFlags()  | btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);
					}
				}
#endif //USE_INTERNAL_EDGE_UTILITY
				bodyMap.insert(colObjData,body);
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
				btRigidBody* body = createRigidBody(isDynamic,mass,startTransform,shape,colObjData->m_collisionObjectData.m_name);
#ifdef USE_INTERNAL_EDGE_UTILITY
				if (shape->getShapeType() == TRIANGLE_MESH_SHAPE_PROXYTYPE)
				{
					btBvhTriangleMeshShape* trimesh = (btBvhTriangleMeshShape*)shape;
					if (trimesh->getTriangleInfoMap())
					{
						body->setCollisionFlags(body->getCollisionFlags()  | btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);
					}
				}
#endif //USE_INTERNAL_EDGE_UTILITY
				bodyMap.insert(colObjData,body);
			} else
			{
				printf("error: no shape found\n");
			}
		}
	}

	for (i=0;i<bulletFile2->m_collisionObjects.size();i++)
	{
		if (bulletFile2->getFlags() & bParse::FD_DOUBLE_PRECISION)
		{
			btCollisionObjectDoubleData* colObjData = (btCollisionObjectDoubleData*)bulletFile2->m_collisionObjects[i];
			btCollisionShape** shapePtr = shapeMap.find(colObjData->m_collisionShape);
			if (shapePtr && *shapePtr)
			{
				btTransform startTransform;
				startTransform.deSerializeDouble(colObjData->m_worldTransform);
				btCollisionShape* shape = (btCollisionShape*)*shapePtr;
				btCollisionObject* body = createCollisionObject(startTransform,shape,colObjData->m_name);

#ifdef USE_INTERNAL_EDGE_UTILITY
				if (shape->getShapeType() == TRIANGLE_MESH_SHAPE_PROXYTYPE)
				{
					btBvhTriangleMeshShape* trimesh = (btBvhTriangleMeshShape*)shape;
					if (trimesh->getTriangleInfoMap())
					{
						body->setCollisionFlags(body->getCollisionFlags()  | btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);
					}
				}
#endif //USE_INTERNAL_EDGE_UTILITY
				bodyMap.insert(colObjData,body);
			} else
			{
				printf("error: no shape found\n");
			}
	
		} else
		{
			btCollisionObjectFloatData* colObjData = (btCollisionObjectFloatData*)bulletFile2->m_collisionObjects[i];
			btCollisionShape** shapePtr = shapeMap.find(colObjData->m_collisionShape);
			if (shapePtr && *shapePtr)
			{
				btTransform startTransform;
				startTransform.deSerializeFloat(colObjData->m_worldTransform);
				btCollisionShape* shape = (btCollisionShape*)*shapePtr;
				btCollisionObject* body = createCollisionObject(startTransform,shape,colObjData->m_name);

#ifdef USE_INTERNAL_EDGE_UTILITY
				if (shape->getShapeType() == TRIANGLE_MESH_SHAPE_PROXYTYPE)
				{
					btBvhTriangleMeshShape* trimesh = (btBvhTriangleMeshShape*)shape;
					if (trimesh->getTriangleInfoMap())
					{
						body->setCollisionFlags(body->getCollisionFlags()  | btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);
					}
				}
#endif //USE_INTERNAL_EDGE_UTILITY
				bodyMap.insert(colObjData,body);
			} else
			{
				printf("error: no shape found\n");
			}
		}
		
		printf("bla");
	}

	
	for (i=0;i<bulletFile2->m_constraints.size();i++)
	{
		btTypedConstraintData* constraintData = (btTypedConstraintData*)bulletFile2->m_constraints[i];
		btCollisionObject** colAptr = bodyMap.find(constraintData->m_rbA);
		btCollisionObject** colBptr = bodyMap.find(constraintData->m_rbB);

		btRigidBody* rbA = 0;
		btRigidBody* rbB = 0;

		if (colAptr)
		{
			rbA = btRigidBody::upcast(*colAptr);
			if (!rbA)
				rbA = &getFixedBody();
		}
		if (colBptr)
		{
			rbB = btRigidBody::upcast(*colBptr);
			if (!rbB)
				rbB = &getFixedBody();
		}
				
		btTypedConstraint* constraint = 0;

		switch (constraintData->m_objectType)
		{
		case POINT2POINT_CONSTRAINT_TYPE:
			{
				if (bulletFile2->getFlags() & bParse::FD_DOUBLE_PRECISION)
				{
					btPoint2PointConstraintDoubleData* p2pData = (btPoint2PointConstraintDoubleData*)constraintData;
					if (rbA && rbB)
					{					
						btVector3 pivotInA,pivotInB;
						pivotInA.deSerializeDouble(p2pData->m_pivotInA);
						pivotInB.deSerializeDouble(p2pData->m_pivotInB);
						constraint = createPoint2PointConstraint(*rbA,*rbB,pivotInA,pivotInB);
					} else
					{
						btVector3 pivotInA;
						pivotInA.deSerializeDouble(p2pData->m_pivotInA);
						constraint = createPoint2PointConstraint(*rbA,pivotInA);
					}
				} else
				{
					btPoint2PointConstraintFloatData* p2pData = (btPoint2PointConstraintFloatData*)constraintData;
					if (rbA&& rbB)
					{					
						btVector3 pivotInA,pivotInB;
						pivotInA.deSerializeFloat(p2pData->m_pivotInA);
						pivotInB.deSerializeFloat(p2pData->m_pivotInB);
						constraint = createPoint2PointConstraint(*rbA,*rbB,pivotInA,pivotInB);
					
					} else
					{
						btVector3 pivotInA;
						pivotInA.deSerializeFloat(p2pData->m_pivotInA);
						constraint = createPoint2PointConstraint(*rbA,pivotInA);
					}

				}

				break;
			}
		case HINGE_CONSTRAINT_TYPE:
			{
				btHingeConstraint* hinge = 0;

				if (bulletFile2->getFlags() & bParse::FD_DOUBLE_PRECISION)
				{
					btHingeConstraintDoubleData* hingeData = (btHingeConstraintDoubleData*)constraintData;
					if (rbA&& rbB)
					{
						btTransform rbAFrame,rbBFrame;
						rbAFrame.deSerializeDouble(hingeData->m_rbAFrame);
						rbBFrame.deSerializeDouble(hingeData->m_rbBFrame);
						hinge = createHingeConstraint(*rbA,*rbB,rbAFrame,rbBFrame,hingeData->m_useReferenceFrameA!=0);
					} else
					{
						btTransform rbAFrame;
						rbAFrame.deSerializeDouble(hingeData->m_rbAFrame);
						hinge = createHingeConstraint(*rbA,rbAFrame,hingeData->m_useReferenceFrameA!=0);
					}
					if (hingeData->m_enableAngularMotor)
					{
						hinge->enableAngularMotor(true,hingeData->m_motorTargetVelocity,hingeData->m_maxMotorImpulse);
					}
					hinge->setAngularOnly(hingeData->m_angularOnly!=0);
					hinge->setLimit(btScalar(hingeData->m_lowerLimit),btScalar(hingeData->m_upperLimit),btScalar(hingeData->m_limitSoftness),btScalar(hingeData->m_biasFactor),btScalar(hingeData->m_relaxationFactor));
				} else
				{
					btHingeConstraintFloatData* hingeData = (btHingeConstraintFloatData*)constraintData;
					if (rbA&& rbB)
					{
						btTransform rbAFrame,rbBFrame;
						rbAFrame.deSerializeFloat(hingeData->m_rbAFrame);
						rbBFrame.deSerializeFloat(hingeData->m_rbBFrame);
						hinge = createHingeConstraint(*rbA,*rbB,rbAFrame,rbBFrame,hingeData->m_useReferenceFrameA!=0);
					} else
					{
						btTransform rbAFrame;
						rbAFrame.deSerializeFloat(hingeData->m_rbAFrame);
						hinge = createHingeConstraint(*rbA,rbAFrame,hingeData->m_useReferenceFrameA!=0);
					}
					if (hingeData->m_enableAngularMotor)
					{
						hinge->enableAngularMotor(true,hingeData->m_motorTargetVelocity,hingeData->m_maxMotorImpulse);
					}
					hinge->setAngularOnly(hingeData->m_angularOnly!=0);
					hinge->setLimit(btScalar(hingeData->m_lowerLimit),btScalar(hingeData->m_upperLimit),btScalar(hingeData->m_limitSoftness),btScalar(hingeData->m_biasFactor),btScalar(hingeData->m_relaxationFactor));
				}

				constraint = hinge;
				break;

			}
		case CONETWIST_CONSTRAINT_TYPE:
			{
				btConeTwistConstraintData* coneData = (btConeTwistConstraintData*)constraintData;
				btConeTwistConstraint* coneTwist = 0;
				
				if (rbA&& rbB)
				{
					btTransform rbAFrame,rbBFrame;
					rbAFrame.deSerializeFloat(coneData->m_rbAFrame);
					rbBFrame.deSerializeFloat(coneData->m_rbBFrame);
					coneTwist = createConeTwistConstraint(*rbA,*rbB,rbAFrame,rbBFrame);
				} else
				{
					btTransform rbAFrame;
					rbAFrame.deSerializeFloat(coneData->m_rbAFrame);
					coneTwist = createConeTwistConstraint(*rbA,rbAFrame);
				}
				coneTwist->setLimit(coneData->m_swingSpan1,coneData->m_swingSpan2,coneData->m_twistSpan,coneData->m_limitSoftness,coneData->m_biasFactor,coneData->m_relaxationFactor);
				coneTwist->setDamping(coneData->m_damping);
				
				constraint = coneTwist;
				break;
			}

		case D6_CONSTRAINT_TYPE:
			{
				btGeneric6DofConstraintData* dofData = (btGeneric6DofConstraintData*)constraintData;
				btGeneric6DofConstraint* dof = 0;

				if (rbA&& rbB)
				{
					btTransform rbAFrame,rbBFrame;
					rbAFrame.deSerializeFloat(dofData->m_rbAFrame);
					rbBFrame.deSerializeFloat(dofData->m_rbBFrame);
					dof = createGeneric6DofConstraint(*rbA,*rbB,rbAFrame,rbBFrame,dofData->m_useLinearReferenceFrameA!=0);
				} else
				{
					btTransform rbBFrame;
					rbBFrame.deSerializeFloat(dofData->m_rbBFrame);
					dof = createGeneric6DofConstraint(*rbB,rbBFrame,dofData->m_useLinearReferenceFrameA!=0);
				}
				btVector3 angLowerLimit,angUpperLimit, linLowerLimit,linUpperlimit;
				angLowerLimit.deSerializeFloat(dofData->m_angularLowerLimit);
				angUpperLimit.deSerializeFloat(dofData->m_angularUpperLimit);
				linLowerLimit.deSerializeFloat(dofData->m_linearLowerLimit);
				linUpperlimit.deSerializeFloat(dofData->m_linearUpperLimit);
				
				dof->setAngularLowerLimit(angLowerLimit);
				dof->setAngularUpperLimit(angUpperLimit);
				dof->setLinearLowerLimit(linLowerLimit);
				dof->setLinearUpperLimit(linUpperlimit);

				constraint = dof;
				break;
			}
		case SLIDER_CONSTRAINT_TYPE:
			{
				btSliderConstraintData* sliderData = (btSliderConstraintData*)constraintData;
				btSliderConstraint* slider = 0;
				if (rbA&& rbB)
				{
					btTransform rbAFrame,rbBFrame;
					rbAFrame.deSerializeFloat(sliderData->m_rbAFrame);
					rbBFrame.deSerializeFloat(sliderData->m_rbBFrame);
					slider = createSliderConstraint(*rbA,*rbB,rbAFrame,rbBFrame,sliderData->m_useLinearReferenceFrameA!=0);
				} else
				{
					btTransform rbBFrame;
					rbBFrame.deSerializeFloat(sliderData->m_rbBFrame);
					slider = createSliderConstraint(*rbB,rbBFrame,sliderData->m_useLinearReferenceFrameA!=0);
				}
				slider->setLowerLinLimit(sliderData->m_linearLowerLimit);
				slider->setUpperLinLimit(sliderData->m_linearUpperLimit);
				slider->setLowerAngLimit(sliderData->m_angularLowerLimit);
				slider->setUpperAngLimit(sliderData->m_angularUpperLimit);
				slider->setUseFrameOffset(sliderData->m_useOffsetForConstraintFrame!=0);
				constraint = slider;
				break;
			}
		
		default:
			{
				printf("unknown constraint type\n");
			}
		};

		if (constraint)
		{
			constraint->setDbgDrawSize(constraintData->m_dbgDrawSize);
			if (constraintData->m_name)
			{
				char* newname = duplicateName(constraintData->m_name);
				m_nameConstraintMap.insert(newname,constraint);
				m_objectNameMap.insert(constraint,newname);
			}
			if(m_dynamicsWorld)
				m_dynamicsWorld->addConstraint(constraint,constraintData->m_disableCollisionsBetweenLinkedBodies!=0);
		}
		
	}

	return true;
}



btCollisionObject* btBulletWorldImporter::createCollisionObject(const btTransform& startTransform,btCollisionShape* shape, const char* bodyName)
{
	return createRigidBody(false,0,startTransform,shape,bodyName);
}



btRigidBody*  btBulletWorldImporter::createRigidBody(bool isDynamic, btScalar mass, const btTransform& startTransform,btCollisionShape* shape,const char* bodyName)
{
	btVector3 localInertia;
	localInertia.setZero();

	if (mass)
		shape->calculateLocalInertia(mass,localInertia);
	
	btRigidBody* body = new btRigidBody(mass,0,shape,localInertia);	
	body->setWorldTransform(startTransform);

	if (m_dynamicsWorld)
		m_dynamicsWorld->addRigidBody(body);
	
	if (bodyName)
	{
		char* newname = duplicateName(bodyName);
		m_objectNameMap.insert(body,newname);
		m_nameBodyMap.insert(newname,body);
	}
	m_allocatedRigidBodies.push_back(body);
	return body;

}

btCollisionShape* btBulletWorldImporter::createPlaneShape(const btVector3& planeNormal,btScalar planeConstant)
{
	btStaticPlaneShape* shape = new btStaticPlaneShape(planeNormal,planeConstant);
	m_allocatedCollisionShapes.push_back(shape);
	return shape;
}
btCollisionShape* btBulletWorldImporter::createBoxShape(const btVector3& halfExtents)
{
	btBoxShape* shape = new btBoxShape(halfExtents);
	m_allocatedCollisionShapes.push_back(shape);
	return shape;
}
btCollisionShape* btBulletWorldImporter::createSphereShape(btScalar radius)
{
	btSphereShape* shape = new btSphereShape(radius);
	m_allocatedCollisionShapes.push_back(shape);
	return shape;
}


btCollisionShape* btBulletWorldImporter::createCapsuleShapeX(btScalar radius, btScalar height)
{
	btCapsuleShapeX* shape = new btCapsuleShapeX(radius,height);
	m_allocatedCollisionShapes.push_back(shape);
	return shape;
}

btCollisionShape* btBulletWorldImporter::createCapsuleShapeY(btScalar radius, btScalar height)
{
	btCapsuleShape* shape = new btCapsuleShape(radius,height);
	m_allocatedCollisionShapes.push_back(shape);
	return shape;
}

btCollisionShape* btBulletWorldImporter::createCapsuleShapeZ(btScalar radius, btScalar height)
{
	btCapsuleShapeZ* shape = new btCapsuleShapeZ(radius,height);
	m_allocatedCollisionShapes.push_back(shape);
	return shape;
}

btCollisionShape* btBulletWorldImporter::createCylinderShapeX(btScalar radius,btScalar height)
{
	btCylinderShapeX* shape = new btCylinderShapeX(btVector3(height,radius,radius));
	m_allocatedCollisionShapes.push_back(shape);
	return shape;
}

btCollisionShape* btBulletWorldImporter::createCylinderShapeY(btScalar radius,btScalar height)
{
	btCylinderShape* shape = new btCylinderShape(btVector3(radius,height,radius));
	m_allocatedCollisionShapes.push_back(shape);
	return shape;
}

btCollisionShape* btBulletWorldImporter::createCylinderShapeZ(btScalar radius,btScalar height)
{
	btCylinderShapeZ* shape = new btCylinderShapeZ(btVector3(radius,radius,height));
	m_allocatedCollisionShapes.push_back(shape);
	return shape;
}

btTriangleIndexVertexArray*	btBulletWorldImporter::createTriangleMeshContainer()
{
	btTriangleIndexVertexArray* in = new btTriangleIndexVertexArray();
	m_allocatedTriangleIndexArrays.push_back(in);
	return in;
}

btOptimizedBvh*	btBulletWorldImporter::createOptimizedBvh()
{
	btOptimizedBvh* bvh = new btOptimizedBvh();
	m_allocatedBvhs.push_back(bvh);
	return bvh;
}


btTriangleInfoMap* btBulletWorldImporter::createTriangleInfoMap()
{
	btTriangleInfoMap* tim = new btTriangleInfoMap();
	m_allocatedTriangleInfoMaps.push_back(tim);
	return tim;
}

btBvhTriangleMeshShape* btBulletWorldImporter::createBvhTriangleMeshShape(btStridingMeshInterface* trimesh, btOptimizedBvh* bvh)
{
	if (bvh)
	{
		btBvhTriangleMeshShape* bvhTriMesh = new btBvhTriangleMeshShape(trimesh,bvh->isQuantized(), false);
		bvhTriMesh->setOptimizedBvh(bvh);
		m_allocatedCollisionShapes.push_back(bvhTriMesh);
		return bvhTriMesh;
	}

	btBvhTriangleMeshShape* ts = new btBvhTriangleMeshShape(trimesh,true);
	m_allocatedCollisionShapes.push_back(ts);
	return ts;

}
btCollisionShape* btBulletWorldImporter::createConvexTriangleMeshShape(btStridingMeshInterface* trimesh)
{
	return 0;
}
btGImpactMeshShape* btBulletWorldImporter::createGimpactShape(btStridingMeshInterface* trimesh)
{
	btGImpactMeshShape* shape = new btGImpactMeshShape(trimesh);
	m_allocatedCollisionShapes.push_back(shape);
	return shape;
	
}
btConvexHullShape* btBulletWorldImporter::createConvexHullShape()
{
	btConvexHullShape* shape = new btConvexHullShape();
	m_allocatedCollisionShapes.push_back(shape);
	return shape;
}

btCompoundShape* btBulletWorldImporter::createCompoundShape()
{
	btCompoundShape* shape = new btCompoundShape();
	m_allocatedCollisionShapes.push_back(shape);
	return shape;
}

btRigidBody& btBulletWorldImporter::getFixedBody()
{
	static btRigidBody s_fixed(0, 0,0);
	s_fixed.setMassProps(btScalar(0.),btVector3(btScalar(0.),btScalar(0.),btScalar(0.)));
	return s_fixed;
}

btPoint2PointConstraint* btBulletWorldImporter::createPoint2PointConstraint(btRigidBody& rbA,btRigidBody& rbB, const btVector3& pivotInA,const btVector3& pivotInB)
{
	btPoint2PointConstraint* p2p = new btPoint2PointConstraint(rbA,rbB,pivotInA,pivotInB);
	m_allocatedConstraints.push_back(p2p);
	return p2p;
}

btPoint2PointConstraint* btBulletWorldImporter::createPoint2PointConstraint(btRigidBody& rbA,const btVector3& pivotInA)
{
	btPoint2PointConstraint* p2p = new btPoint2PointConstraint(rbA,pivotInA);
	m_allocatedConstraints.push_back(p2p);
	return p2p;
}


btHingeConstraint* btBulletWorldImporter::createHingeConstraint(btRigidBody& rbA,btRigidBody& rbB, const btTransform& rbAFrame, const btTransform& rbBFrame, bool useReferenceFrameA)
{
	btHingeConstraint* hinge = new btHingeConstraint(rbA,rbB,rbAFrame,rbBFrame,useReferenceFrameA);
	m_allocatedConstraints.push_back(hinge);
	return hinge;
}

btHingeConstraint* btBulletWorldImporter::createHingeConstraint(btRigidBody& rbA,const btTransform& rbAFrame, bool useReferenceFrameA)
{
	btHingeConstraint* hinge = new btHingeConstraint(rbA,rbAFrame,useReferenceFrameA);
	m_allocatedConstraints.push_back(hinge);
	return hinge;
}

btConeTwistConstraint* btBulletWorldImporter::createConeTwistConstraint(btRigidBody& rbA,btRigidBody& rbB,const btTransform& rbAFrame, const btTransform& rbBFrame)
{
	btConeTwistConstraint* cone = new btConeTwistConstraint(rbA,rbB,rbAFrame,rbBFrame);
	m_allocatedConstraints.push_back(cone);
	return cone;
}

btConeTwistConstraint* btBulletWorldImporter::createConeTwistConstraint(btRigidBody& rbA,const btTransform& rbAFrame)
{
	btConeTwistConstraint* cone = new btConeTwistConstraint(rbA,rbAFrame);
	m_allocatedConstraints.push_back(cone);
	return cone;
}


btGeneric6DofConstraint* btBulletWorldImporter::createGeneric6DofConstraint(btRigidBody& rbA, btRigidBody& rbB, const btTransform& frameInA, const btTransform& frameInB ,bool useLinearReferenceFrameA)
{
	btGeneric6DofConstraint* dof = new btGeneric6DofConstraint(rbA,rbB,frameInA,frameInB,useLinearReferenceFrameA);
	m_allocatedConstraints.push_back(dof);
	return dof;
}

btGeneric6DofConstraint* btBulletWorldImporter::createGeneric6DofConstraint(btRigidBody& rbB, const btTransform& frameInB, bool useLinearReferenceFrameB)
{
	btGeneric6DofConstraint* dof =  new btGeneric6DofConstraint(rbB,frameInB,useLinearReferenceFrameB);
	m_allocatedConstraints.push_back(dof);
	return dof;
}
btSliderConstraint* btBulletWorldImporter::createSliderConstraint(btRigidBody& rbA, btRigidBody& rbB, const btTransform& frameInA, const btTransform& frameInB ,bool useLinearReferenceFrameA)
{
	btSliderConstraint* slider = new btSliderConstraint(rbA,rbB,frameInA,frameInB,useLinearReferenceFrameA);
	m_allocatedConstraints.push_back(slider);
	return slider;
}

btSliderConstraint* btBulletWorldImporter::createSliderConstraint(btRigidBody& rbB, const btTransform& frameInB, bool useLinearReferenceFrameA)
{
	btSliderConstraint* slider = new btSliderConstraint(rbB,frameInB,useLinearReferenceFrameA);
	m_allocatedConstraints.push_back(slider);
	return slider;
}


	// query for data
int	btBulletWorldImporter::getNumCollisionShapes() const
{
	return m_allocatedCollisionShapes.size();
}

btCollisionShape* btBulletWorldImporter::getCollisionShapeByIndex(int index)
{
	return m_allocatedCollisionShapes[index];
}

btCollisionShape* btBulletWorldImporter::getCollisionShapeByName(const char* name)
{
	btCollisionShape** shapePtr = m_nameShapeMap.find(name);
	if (shapePtr&& *shapePtr)
	{
		return *shapePtr;
	}
	return 0;
}

btRigidBody* btBulletWorldImporter::getRigidBodyByName(const char* name)
{
	btRigidBody** bodyPtr = m_nameBodyMap.find(name);
	if (bodyPtr && *bodyPtr)
	{
		return *bodyPtr;
	}
	return 0;
}

btTypedConstraint* btBulletWorldImporter::getConstraintByName(const char* name)
{
	btTypedConstraint** constraintPtr = m_nameConstraintMap.find(name);
	if (constraintPtr && *constraintPtr)
	{
		return *constraintPtr;
	}
	return 0;
}

const char*	btBulletWorldImporter::getNameForPointer(const void* ptr) const
{
	const char*const * namePtr = m_objectNameMap.find(ptr);
	if (namePtr && *namePtr)
		return *namePtr;
	return 0;
}


int btBulletWorldImporter::getNumRigidBodies() const
{
	return m_allocatedRigidBodies.size();
}

btCollisionObject* btBulletWorldImporter::getRigidBodyByIndex(int index) const
{
	return m_allocatedRigidBodies[index];
}
int btBulletWorldImporter::getNumConstraints() const
{
	return m_allocatedConstraints.size();
}

btTypedConstraint* btBulletWorldImporter::getConstraintByIndex(int index) const
{
	return m_allocatedConstraints[index];
}

int btBulletWorldImporter::getNumBvhs() const
{
	return m_allocatedBvhs.size();
}
 btOptimizedBvh* btBulletWorldImporter::getBvhByIndex(int index) const
{
	return m_allocatedBvhs[index];
}

int btBulletWorldImporter::getNumTriangleInfoMaps() const
{
	return m_allocatedTriangleInfoMaps.size();
}

btTriangleInfoMap* btBulletWorldImporter::getTriangleInfoMapByIndex(int index) const
{
	return m_allocatedTriangleInfoMaps[index];
}

