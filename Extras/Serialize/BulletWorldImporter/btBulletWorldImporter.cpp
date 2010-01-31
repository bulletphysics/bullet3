
#include "btBulletWorldImporter.h"
#include "btBulletFile.h"

#include "btBulletDynamicsCommon.h"
#include "BulletCollision/Gimpact/btGImpactShape.h"

btBulletWorldImporter::btBulletWorldImporter(btDynamicsWorld* world)
:m_dynamicsWorld(world),
m_verboseDumpAllTypes(false)
{
}

btBulletWorldImporter::~btBulletWorldImporter()
{
	for (int i=0;i<m_allocatedCollisionShapes.size();i++)
	{
		delete m_allocatedCollisionShapes[i];
	}
	m_allocatedCollisionShapes.clear();
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
			meshPart.m_triangleIndexStride = 3*sizeof(short int);
			meshPart.m_triangleIndexBase = (const unsigned char*)meshData.m_meshPartsPtr[i].m_indices16;
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
		
		meshInterface->addIndexedMesh(meshPart);
	}

	return meshInterface;
}

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
			shape = new btStaticPlaneShape(planeNormal,btScalar(planeData->m_planeConstant));
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

			btCollisionShape* trimeshShape = createBvhTriangleMeshShape(meshInterface);
			trimeshShape->setMargin(trimesh->m_collisionMargin);
			shape = trimeshShape;

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
		btCollisionShape* shape = convertCollisionShape(shapeData);
		if (shape)
			shapeMap.insert(shapeData,shape);
	}

	btHashMap<btHashPtr,btRigidBody*>	bodyMap;

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
				btRigidBody* body = createRigidBody(isDynamic,mass,startTransform,shape);
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
				btRigidBody* body = createRigidBody(isDynamic,mass,startTransform,shape);
				bodyMap.insert(colObjData,body);
			} else
			{
				printf("error: no shape found\n");
			}
		}
	}

//	for (i=0;i<bulletFile2->m_collisionObjects.size();i++)
//	{
//		btCollisionObjectData* colObjData = (btCollisionObjectData*)bulletFile2->m_collisionObjects[i];
//		printf("bla");
//	}

	
	for (i=0;i<bulletFile2->m_constraints.size();i++)
	{
		btTypedConstraintData* constraintData = (btTypedConstraintData*)bulletFile2->m_constraints[i];
		btRigidBody** rbAptr = bodyMap.find(constraintData->m_rbA);
		btRigidBody** rbBptr = bodyMap.find(constraintData->m_rbB);
				
		switch (constraintData->m_objectType)
		{
		case POINT2POINT_CONSTRAINT_TYPE:
			{
				btPoint2PointConstraint* constraint = 0;

				if (bulletFile2->getFlags() & bParse::FD_DOUBLE_PRECISION)
				{
					btPoint2PointConstraintDoubleData* p2pData = (btPoint2PointConstraintDoubleData*)constraintData;
					if (rbAptr && rbBptr)
					{					
						btVector3 pivotInA,pivotInB;
						pivotInA.deSerializeDouble(p2pData->m_pivotInA);
						pivotInB.deSerializeDouble(p2pData->m_pivotInB);
						constraint = new btPoint2PointConstraint(**rbAptr,**rbBptr,pivotInA,pivotInB);
					} else
					{
						btVector3 pivotInA;
						pivotInA.deSerializeDouble(p2pData->m_pivotInA);
						constraint = new btPoint2PointConstraint(**rbAptr,pivotInA);
					}
				} else
				{
					btPoint2PointConstraintFloatData* p2pData = (btPoint2PointConstraintFloatData*)constraintData;
					if (rbAptr && rbBptr)
					{					
						btVector3 pivotInA,pivotInB;
						pivotInA.deSerializeFloat(p2pData->m_pivotInA);
						pivotInB.deSerializeFloat(p2pData->m_pivotInB);
						constraint = new btPoint2PointConstraint(**rbAptr,**rbBptr,pivotInA,pivotInB);
					
					} else
					{
						btVector3 pivotInA;
						pivotInA.deSerializeFloat(p2pData->m_pivotInA);
						constraint = new btPoint2PointConstraint(**rbAptr,pivotInA);
					}

				}

				m_dynamicsWorld->addConstraint(constraint,constraintData->m_disableCollisionsBetweenLinkedBodies!=0);
				constraint->setDbgDrawSize(constraintData->m_dbgDrawSize);
				break;
			}
		case HINGE_CONSTRAINT_TYPE:
			{
				btHingeConstraint* hinge = 0;

				if (bulletFile2->getFlags() & bParse::FD_DOUBLE_PRECISION)
				{
					btHingeConstraintDoubleData* hingeData = (btHingeConstraintDoubleData*)constraintData;
					if (rbAptr && rbBptr)
					{
						btTransform rbAFrame,rbBFrame;
						rbAFrame.deSerializeDouble(hingeData->m_rbAFrame);
						rbBFrame.deSerializeDouble(hingeData->m_rbBFrame);
						hinge = new btHingeConstraint(**rbAptr,**rbBptr,rbAFrame,rbBFrame,hingeData->m_useReferenceFrameA!=0);
					} else
					{
						btTransform rbAFrame;
						rbAFrame.deSerializeDouble(hingeData->m_rbAFrame);
						hinge = new btHingeConstraint(**rbAptr,rbAFrame,hingeData->m_useReferenceFrameA!=0);
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
					if (rbAptr && rbBptr)
					{
						btTransform rbAFrame,rbBFrame;
						rbAFrame.deSerializeFloat(hingeData->m_rbAFrame);
						rbBFrame.deSerializeFloat(hingeData->m_rbBFrame);
						hinge = new btHingeConstraint(**rbAptr,**rbBptr,rbAFrame,rbBFrame,hingeData->m_useReferenceFrameA!=0);
					} else
					{
						btTransform rbAFrame;
						rbAFrame.deSerializeFloat(hingeData->m_rbAFrame);
						hinge = new btHingeConstraint(**rbAptr,rbAFrame,hingeData->m_useReferenceFrameA!=0);
					}
					if (hingeData->m_enableAngularMotor)
					{
						hinge->enableAngularMotor(true,hingeData->m_motorTargetVelocity,hingeData->m_maxMotorImpulse);
					}
					hinge->setAngularOnly(hingeData->m_angularOnly!=0);
					hinge->setLimit(btScalar(hingeData->m_lowerLimit),btScalar(hingeData->m_upperLimit),btScalar(hingeData->m_limitSoftness),btScalar(hingeData->m_biasFactor),btScalar(hingeData->m_relaxationFactor));
				}
				m_dynamicsWorld->addConstraint(hinge,constraintData->m_disableCollisionsBetweenLinkedBodies!=0);
				hinge->setDbgDrawSize(constraintData->m_dbgDrawSize);
				
				
				break;

			}
		case CONETWIST_CONSTRAINT_TYPE:
			{
				btConeTwistConstraintData* coneData = (btConeTwistConstraintData*)constraintData;
				btConeTwistConstraint* coneTwist = 0;
				
				if (rbAptr && rbBptr)
				{
					btTransform rbAFrame,rbBFrame;
					rbAFrame.deSerializeFloat(coneData->m_rbAFrame);
					rbBFrame.deSerializeFloat(coneData->m_rbBFrame);
					coneTwist = new btConeTwistConstraint(**rbAptr,**rbBptr,rbAFrame,rbBFrame);
				} else
				{
					btTransform rbAFrame;
					rbAFrame.deSerializeFloat(coneData->m_rbAFrame);
					coneTwist = new btConeTwistConstraint(**rbAptr,rbAFrame);
				}
				coneTwist->setLimit(coneData->m_swingSpan1,coneData->m_swingSpan2,coneData->m_twistSpan,coneData->m_limitSoftness,coneData->m_biasFactor,coneData->m_relaxationFactor);
				coneTwist->setDamping(coneData->m_damping);
				m_dynamicsWorld->addConstraint(coneTwist,constraintData->m_disableCollisionsBetweenLinkedBodies!=0);
				coneTwist->setDbgDrawSize(constraintData->m_dbgDrawSize);


				break;
			}

		case D6_CONSTRAINT_TYPE:
			{
				btGeneric6DofConstraintData* dofData = (btGeneric6DofConstraintData*)constraintData;
				btGeneric6DofConstraint* dof = 0;

				if (rbAptr && rbBptr)
				{
					btTransform rbAFrame,rbBFrame;
					rbAFrame.deSerializeFloat(dofData->m_rbAFrame);
					rbBFrame.deSerializeFloat(dofData->m_rbBFrame);
					dof = new btGeneric6DofConstraint(**rbAptr,**rbBptr,rbAFrame,rbBFrame,dofData->m_useLinearReferenceFrameA!=0);
				} else
				{
					btTransform rbBFrame;
					rbBFrame.deSerializeFloat(dofData->m_rbBFrame);
					dof = new btGeneric6DofConstraint(**rbBptr,rbBFrame,dofData->m_useLinearReferenceFrameA!=0);
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

				m_dynamicsWorld->addConstraint(dof,constraintData->m_disableCollisionsBetweenLinkedBodies!=0);
				dof->setDbgDrawSize(constraintData->m_dbgDrawSize);
				break;
			}
		case SLIDER_CONSTRAINT_TYPE:
			{
				btSliderConstraintData* sliderData = (btSliderConstraintData*)constraintData;
				btSliderConstraint* slider = 0;
				if (rbAptr && rbBptr)
				{
					btTransform rbAFrame,rbBFrame;
					rbAFrame.deSerializeFloat(sliderData->m_rbAFrame);
					rbBFrame.deSerializeFloat(sliderData->m_rbBFrame);
					slider = new btSliderConstraint(**rbAptr,**rbBptr,rbAFrame,rbBFrame,sliderData->m_useLinearReferenceFrameA!=0);
				} else
				{
					btTransform rbBFrame;
					rbBFrame.deSerializeFloat(sliderData->m_rbBFrame);
					slider = new btSliderConstraint(**rbBptr,rbBFrame,sliderData->m_useLinearReferenceFrameA!=0);
				}
				slider->setLowerLinLimit(sliderData->m_linearLowerLimit);
				slider->setUpperLinLimit(sliderData->m_linearUpperLimit);
				slider->setLowerAngLimit(sliderData->m_angularLowerLimit);
				slider->setUpperAngLimit(sliderData->m_angularUpperLimit);
				slider->setUseFrameOffset(sliderData->m_useOffsetForConstraintFrame!=0);

				m_dynamicsWorld->addConstraint(slider,constraintData->m_disableCollisionsBetweenLinkedBodies!=0);
				slider->setDbgDrawSize(constraintData->m_dbgDrawSize);

				break;
			}
		
		default:
			{
				printf("unknown constraint type\n");
			}
		};
		
	}

	return true;
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


btCollisionShape* btBulletWorldImporter::createCapsuleShapeX(btScalar radius, btScalar height)
{
	return new btCapsuleShapeX(radius,height);
}

btCollisionShape* btBulletWorldImporter::createCapsuleShapeY(btScalar radius, btScalar height)
{
	return new btCapsuleShape(radius,height);
}

btCollisionShape* btBulletWorldImporter::createCapsuleShapeZ(btScalar radius, btScalar height)
{
	return new btCapsuleShapeZ(radius,height);
}

btCollisionShape* btBulletWorldImporter::createCylinderShapeX(btScalar radius,btScalar height)
{
	return new btCylinderShapeX(btVector3(height,radius,radius));
}

btCollisionShape* btBulletWorldImporter::createCylinderShapeY(btScalar radius,btScalar height)
{
	return new btCylinderShape(btVector3(radius,height,radius));
}

btCollisionShape* btBulletWorldImporter::createCylinderShapeZ(btScalar radius,btScalar height)
{
	return new btCylinderShapeZ(btVector3(radius,radius,height));
}

btTriangleIndexVertexArray*	btBulletWorldImporter::createTriangleMeshContainer()
{
	return new btTriangleIndexVertexArray();
}
btCollisionShape* btBulletWorldImporter::createBvhTriangleMeshShape(btStridingMeshInterface* trimesh)
{
	return new btBvhTriangleMeshShape(trimesh,true);
}
btCollisionShape* btBulletWorldImporter::createConvexTriangleMeshShape(btStridingMeshInterface* trimesh)
{
	return 0;
}
btGImpactMeshShape* btBulletWorldImporter::createGimpactShape(btStridingMeshInterface* trimesh)
{
	return new btGImpactMeshShape(trimesh);
}
btConvexHullShape* btBulletWorldImporter::createConvexHullShape()
{
	return new btConvexHullShape();
}

btCompoundShape* btBulletWorldImporter::createCompoundShape()
{
	return new btCompoundShape();
}

