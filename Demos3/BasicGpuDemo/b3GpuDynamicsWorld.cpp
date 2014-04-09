#include "b3GpuDynamicsWorld.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"

//#include "../../../opencl/gpu_rigidbody_pipeline2/CLPhysicsDemo.h"
//#include "../../../opencl/gpu_rigidbody_pipeline/b3GpuNarrowPhaseAndSolver.h"
#include "BulletCollision/CollisionShapes/btPolyhedralConvexShape.h"
#include "BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h"
#include "BulletCollision/CollisionShapes/btCompoundShape.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"
#include "BulletCollision/CollisionShapes/btStaticPlaneShape.h"
#include "BulletCollision/CollisionShapes/btConvexHullShape.h"
#include "BulletDynamics/ConstraintSolver/btPoint2PointConstraint.h"
#include "BulletDynamics/ConstraintSolver/btGeneric6DofConstraint.h"

#include "LinearMath/btQuickprof.h"
//#include "Bullet3Common/b3Logging.h"
#include "Bullet3OpenCL/BroadphaseCollision/b3GpuSapBroadphase.h"
#include "Bullet3OpenCL/RigidBody/b3GpuNarrowPhase.h"
#include "Bullet3OpenCL/RigidBody/b3GpuRigidBodyPipeline.h"
#include "Bullet3Dynamics/ConstraintSolver/b3Point2PointConstraint.h"
#include "Bullet3Dynamics/ConstraintSolver/b3Generic6DofConstraint.h"

#include "Bullet3Collision/NarrowPhaseCollision/b3RigidBodyCL.h"


#ifdef _WIN32
	#include <windows.h>
#endif


//#if (BT_BULLET_VERSION >= 282)
//#define BT_USE_BODY_UPDATE_REVISION
//#endif


b3GpuDynamicsWorld::b3GpuDynamicsWorld(class b3GpuSapBroadphase* bp,class b3GpuNarrowPhase* np, class b3GpuRigidBodyPipeline* rigidBodyPipeline)
	:btDynamicsWorld(0,0,0),
	m_gravity(0,-10,0),
m_cpuGpuSync(true),
m_bp(bp),
m_np(np),
m_rigidBodyPipeline(rigidBodyPipeline),
m_localTime(0.f),
m_staticBody(0)
{
	btConvexHullShape* nullShape = new btConvexHullShape();
	m_staticBody = new btRigidBody(0,0,nullShape);
	addRigidBody(m_staticBody,0,0);
}

b3GpuDynamicsWorld::~b3GpuDynamicsWorld()
{
	
}


#include <windows.h>

int		b3GpuDynamicsWorld::stepSimulation( btScalar timeStepUnused, int maxSubStepsUnused, btScalar fixedTimeStep)
{
	///Don't use more than 1 simulation step, it destroys the performance having to copy the data between CPU and GPU multiple times per frame
	///Please use the CPU version in btDiscreteDynamicsWorld if you don't like this
		

	CProfileManager::Reset();

	BT_PROFILE("stepSimulation");

	{
		BT_PROFILE("sync constraints CPU");
		//todo: determine what data has changed, or perform copy on GPU?
		for (int i=0;i<m_constraints.size();i++)
		{
			btTypedConstraint* constraint = m_constraints[i];
			b3TypedConstraint* c = (b3TypedConstraint*) constraint->getUserConstraintPtr();
			if (c)
			{
				switch (constraint->getConstraintType())
				{
					case POINT2POINT_CONSTRAINT_TYPE:
					{
						btPoint2PointConstraint* p2 = (btPoint2PointConstraint*) constraint;
						b3Point2PointConstraint* p3 = (b3Point2PointConstraint*) c;
						p3->setPivotA((const b3Vector3&)p2->getPivotInA());
						p3->setPivotB((const b3Vector3&)p2->getPivotInB());
						p3->m_setting.m_damping = p2->m_setting.m_damping;
						p3->m_setting.m_impulseClamp = p2->m_setting.m_impulseClamp;
						p3->m_setting.m_tau = p2->m_setting.m_tau;

						break;
					};

					case D6_CONSTRAINT_TYPE:
					{
						btGeneric6DofConstraint* dof2  = (btGeneric6DofConstraint*) constraint;
						b3Generic6DofConstraint* dof3  = (b3Generic6DofConstraint*) c;
						const b3RigidBodyData* bodiesCL = m_np->getBodiesCpu();

						b3Transform frameInA = (b3Transform&) dof2->getFrameOffsetA();
						b3Transform frameInB = (b3Transform&) dof2->getFrameOffsetB();
						dof3->setFrames(frameInA,frameInB,bodiesCL);
						break;
					}

					default:
						{
						}
				};
			}
		}
	}
	
	// detect any change (very simple)
	{
		B3_PROFILE("body update revision detection (CPU)");
#ifdef BT_USE_BODY_UPDATE_REVISION
		b3Assert(m_bodyUpdateRevisions.size() == m_collisionObjects.size());
		b3Assert(m_np->getNumRigidBodies() == m_bodyUpdateRevisions.size());
#endif //BT_USE_BODY_UPDATE_REVISION

		 b3RigidBodyData* bodiesCL = (b3RigidBodyData*)m_np->getBodiesCpu();
		for (int i=0;i<this->m_collisionObjects.size();i++)
		{
			if (i>=m_np->getNumRigidBodies())
			{
				b3Error("bodiesCL out-of-range\n");
				continue;
			}

#ifdef BT_USE_BODY_UPDATE_REVISION
			if (m_bodyUpdateRevisions[i] != m_collisionObjects[i]->getUpdateRevisionInternal())
#endif//BT_USE_BODY_UPDATE_REVISION
			{
				m_cpuGpuSync = true;

#ifdef BT_USE_BODY_UPDATE_REVISION
				m_bodyUpdateRevisions[i] = m_collisionObjects[i]->getUpdateRevisionInternal();
#endif

				
				btRigidBody* body = btRigidBody::upcast(m_collisionObjects[i]);
				if (body)
				{
					b3Vector3 pos = (const b3Vector3&)m_collisionObjects[i]->getWorldTransform().getOrigin();
					btQuaternion orn2 = m_collisionObjects[i]->getWorldTransform().getRotation();
					b3Quaternion orn(orn2[0],orn2[1],orn2[2],orn2[3]); 
					body->integrateVelocities(fixedTimeStep);
					m_np->setObjectTransformCpu(&pos[0],&orn[0],i);
					b3Vector3 linVel = (const b3Vector3&)body->getLinearVelocity();
					b3Vector3 angVel = (const b3Vector3&)body->getAngularVelocity();
					m_np->setObjectVelocityCpu(&linVel[0],&angVel[0],i);

					
				}

			}
		}
	}

	

	
	if (m_cpuGpuSync)
	{
		BT_PROFILE("cpuGpuSync");
		m_cpuGpuSync = false;
		m_np->writeAllBodiesToGpu();
		m_bp->writeAabbsToGpu();
		m_rigidBodyPipeline->writeAllInstancesToGpu();
	}



	//internalSingleStepSimulation(fixedTimeStep);
	// dispatch preTick callback
	if(0 != m_internalPreTickCallback) 
	{
		BT_PROFILE("internalPreTickCallback");
		(*m_internalPreTickCallback)(this, fixedTimeStep);
	}	

	{
		BT_PROFILE("m_rigidBodyPipeline->stepSimulation");
		m_rigidBodyPipeline->stepSimulation(fixedTimeStep);
	}
			
	{
		BT_PROFILE("readbackBodiesToCpu");
		//now copy info back to rigid bodies....
		m_np->readbackAllBodiesToCpu();
	}

	{
		BT_PROFILE("scatter transforms into rigidbody (CPU)");
			
		const b3RigidBodyData* bodiesCL = m_np->getBodiesCpu();

		for (int i=0;i<this->m_collisionObjects.size();i++)
		{
			btVector3 pos;
			btQuaternion orn;
			if (m_np->getObjectTransformFromCpu(&pos[0],&orn[0],i))
			{
				btTransform newTrans;
				newTrans.setOrigin(pos);
				newTrans.setRotation(orn);

				btCollisionObject* colObj = this->m_collisionObjects[i];
				colObj->setWorldTransform(newTrans);

				btRigidBody* body = btRigidBody::upcast(m_collisionObjects[i]);
				if (body)
				{
					body->setLinearVelocity((btVector3&)bodiesCL[i].m_linVel);
					body->setAngularVelocity((btVector3&)bodiesCL[i].m_angVel);
				}
			}

				
#ifdef BT_USE_BODY_UPDATE_REVISION
			//ignore this revision update
			m_bodyUpdateRevisions[i] = m_collisionObjects[i]->getUpdateRevisionInternal();
#endif //BT_USE_BODY_UPDATE_REVISION

		}

		{
			BT_PROFILE("synchronizeMotionStates");
			synchronizeMotionStates();
		}
	}

	{
		B3_PROFILE("clearForces");
		clearForces();
	}

#ifndef BT_NO_PROFILE
	CProfileManager::Increment_Frame_Counter();
//	CProfileManager::dumpAll();
#endif //BT_NO_PROFILE
	

	return 1;
}


void	b3GpuDynamicsWorld::clearForces()
{
	///@todo: iterate over awake simulation islands!
	for ( int i=0;i<m_collisionObjects.size();i++)
	{
		btRigidBody* body = btRigidBody::upcast(m_collisionObjects[i]);
		//need to check if next line is ok
		//it might break backward compatibility (people applying forces on sleeping objects get never cleared and accumulate on wake-up
		if (body)
			body->clearForces();
	}
}	

void	b3GpuDynamicsWorld::setGravity(const btVector3& gravity)
{
	m_gravity = gravity;
	m_rigidBodyPipeline->setGravity(gravity);
}

int b3GpuDynamicsWorld::findOrRegisterCollisionShape(const btCollisionShape* colShape)
{
	int index = m_uniqueShapes.findLinearSearch(colShape);
	if (index==m_uniqueShapes.size())
	{
		if (colShape->isPolyhedral())
		{
			m_uniqueShapes.push_back(colShape);
			
			btPolyhedralConvexShape* convex = (btPolyhedralConvexShape*)colShape;
			int numVertices=convex->getNumVertices();
			
			int strideInBytes=sizeof(btVector3);
			btAlignedObjectArray<btVector3> tmpVertices;
			tmpVertices.resize(numVertices);
			for (int i=0;i<numVertices;i++)
				convex->getVertex(i,tmpVertices[i]);
			const float scaling[4]={1,1,1,1};
			//bool noHeightField=true;
			
			//int gpuShapeIndex = m_np->registerConvexHullShape(&tmpVertices[0].getX(), strideInBytes, numVertices, scaling);
			const float* verts = numVertices? &tmpVertices[0].getX() : 0;
			
			int gpuShapeIndex = m_np->registerConvexHullShape(verts,strideInBytes, numVertices, scaling);
			m_uniqueShapeMapping.push_back(gpuShapeIndex);
		} else
		{
			if (colShape->getShapeType()==TRIANGLE_MESH_SHAPE_PROXYTYPE)
			{
				m_uniqueShapes.push_back(colShape);
				
				btBvhTriangleMeshShape* trimesh = (btBvhTriangleMeshShape*) colShape;
				btStridingMeshInterface* meshInterface = trimesh->getMeshInterface();
				b3AlignedObjectArray<b3Vector3> vertices;
				b3AlignedObjectArray<int> indices;
				
				btVector3 trimeshScaling(1,1,1);
				for (int partId=0;partId<meshInterface->getNumSubParts();partId++)
				{
					
					const unsigned char *vertexbase = 0;
					int numverts = 0;
					PHY_ScalarType type = PHY_INTEGER;
					int stride = 0;
					const unsigned char *indexbase = 0;
					int indexstride = 0;
					int numfaces = 0;
					PHY_ScalarType indicestype = PHY_INTEGER;
					//PHY_ScalarType indexType=0;
					
					b3Vector3 triangleVerts[3];
					meshInterface->getLockedReadOnlyVertexIndexBase(&vertexbase,numverts,	type,stride,&indexbase,indexstride,numfaces,indicestype,partId);
					btVector3 aabbMin,aabbMax;
					
					for (int triangleIndex = 0 ; triangleIndex < numfaces;triangleIndex++)
					{
						unsigned int* gfxbase = (unsigned int*)(indexbase+triangleIndex*indexstride);
						
						for (int j=2;j>=0;j--)
						{
							
							int graphicsindex = indicestype==PHY_SHORT?((unsigned short*)gfxbase)[j]:gfxbase[j];
							if (type == PHY_FLOAT)
							{
								float* graphicsbase = (float*)(vertexbase+graphicsindex*stride);
								triangleVerts[j] = b3MakeVector3(
															 graphicsbase[0]*trimeshScaling.getX(),
															 graphicsbase[1]*trimeshScaling.getY(),
															 graphicsbase[2]*trimeshScaling.getZ());
							}
							else
							{
								double* graphicsbase = (double*)(vertexbase+graphicsindex*stride);
								triangleVerts[j] = b3MakeVector3( btScalar(graphicsbase[0]*trimeshScaling.getX()),
															 btScalar(graphicsbase[1]*trimeshScaling.getY()),
															 btScalar(graphicsbase[2]*trimeshScaling.getZ()));
							}
						}
						vertices.push_back(triangleVerts[0]);
						vertices.push_back(triangleVerts[1]);
						vertices.push_back(triangleVerts[2]);
						indices.push_back(indices.size());
						indices.push_back(indices.size());
						indices.push_back(indices.size());
					}
				}
				//GraphicsShape* gfxShape = 0;//b3BulletDataExtractor::createGraphicsShapeFromWavefrontObj(objData);
				
				//GraphicsShape* gfxShape = b3BulletDataExtractor::createGraphicsShapeFromConvexHull(&sUnitSpherePoints[0],MY_UNITSPHERE_POINTS);
				float meshScaling[4] = {1,1,1,1};
				//int shapeIndex = renderer.registerShape(gfxShape->m_vertices,gfxShape->m_numvertices,gfxShape->m_indices,gfxShape->m_numIndices);
				//float groundPos[4] = {0,0,0,0};
				
				//renderer.registerGraphicsInstance(shapeIndex,groundPos,rotOrn,color,meshScaling);
				if (vertices.size() && indices.size())
				{
					int gpuShapeIndex = m_np->registerConcaveMesh(&vertices,&indices, meshScaling);
					m_uniqueShapeMapping.push_back(gpuShapeIndex);
				} else
				{
					printf("Error: no vertices in mesh in b3GpuDynamicsWorld::addRigidBody\n");
					index = -1;
					b3Assert(0);
				}
				
				
			} else
			{
				if (colShape->getShapeType()==COMPOUND_SHAPE_PROXYTYPE)
				{
					
					btCompoundShape* compound = (btCompoundShape*) colShape;
					b3AlignedObjectArray<b3GpuChildShape> childShapes;
					
					for (int i=0;i<compound->getNumChildShapes();i++)
					{
						//for now, only support polyhedral child shapes
						b3Assert(compound->getChildShape(i)->isPolyhedral());
						b3GpuChildShape child;
						child.m_shapeIndex = findOrRegisterCollisionShape(compound->getChildShape(i));
						btVector3 pos = compound->getChildTransform(i).getOrigin();
						btQuaternion orn = compound->getChildTransform(i).getRotation();
						for (int v=0;v<4;v++)
						{
							child.m_childPosition[v] = pos[v];
							child.m_childOrientation[v] = orn[v];
						}
						childShapes.push_back(child);
					}
					index = m_uniqueShapes.size();
					m_uniqueShapes.push_back(colShape);
					
					int gpuShapeIndex = m_np->registerCompoundShape(&childShapes);
					m_uniqueShapeMapping.push_back(gpuShapeIndex);

					
					
					
					/*printf("Error: unsupported compound type (%d) in b3GpuDynamicsWorld::addRigidBody\n",colShape->getShapeType());
					index = -1;
					b3Assert(0);
					 */
				} else
				{
					if (colShape->getShapeType()==SPHERE_SHAPE_PROXYTYPE)
					{
						m_uniqueShapes.push_back(colShape);
						btSphereShape* sphere = (btSphereShape*)colShape;
						
						int gpuShapeIndex = m_np->registerSphereShape(sphere->getRadius());
						m_uniqueShapeMapping.push_back(gpuShapeIndex);
					} else
					{
						if (colShape->getShapeType()==STATIC_PLANE_PROXYTYPE)
						{
							m_uniqueShapes.push_back(colShape);
							btStaticPlaneShape* plane = (btStaticPlaneShape*)colShape;
						
							int gpuShapeIndex = m_np->registerPlaneShape((b3Vector3&)plane->getPlaneNormal(),plane->getPlaneConstant());
							m_uniqueShapeMapping.push_back(gpuShapeIndex);
						} else
						{
							printf("Error: unsupported shape type (%d) in b3GpuDynamicsWorld::addRigidBody\n",colShape->getShapeType());
							index = -1;
							b3Assert(0);
						}
					}
				}
			}
		}
		
	}
	
	return index;
}

void	b3GpuDynamicsWorld::addRigidBody(btRigidBody* body)
{
	m_cpuGpuSync = true;
	//body->setMotionState(0);
	

	int index = findOrRegisterCollisionShape(body->getCollisionShape());

	if (index>=0)
	{
		int gpuShapeIndex= m_uniqueShapeMapping[index];
		float mass = body->getInvMass() ? 1.f/body->getInvMass() : 0.f;
		btVector3 pos = body->getWorldTransform().getOrigin();
		btQuaternion orn = body->getWorldTransform().getRotation();
	
		int bodyIndex = m_rigidBodyPipeline->registerPhysicsInstance(mass,&pos.getX(),&orn.getX(),gpuShapeIndex,m_collisionObjects.size(),false);
		
		body->setUserIndex(bodyIndex);

		m_collisionObjects.push_back(body);
		m_bodyUpdateRevisions.push_back(-1);
	}
}

void	b3GpuDynamicsWorld::removeRigidBody(btRigidBody* colObj)
{
	m_cpuGpuSync = true;
	btDynamicsWorld::removeCollisionObject(colObj);
	m_bodyUpdateRevisions.resize(this->m_collisionObjects.size());
	for (int i=0;i<m_bodyUpdateRevisions.size();i++)
	{
		m_bodyUpdateRevisions[i] = -1;
	}

	int bodyIndex = colObj->getUserIndex();
	
	if (getNumCollisionObjects()==0)
	{
		m_uniqueShapes.resize(0);
		m_uniqueShapeMapping.resize(0);
		m_np->reset();
		m_bp->reset();
		m_rigidBodyPipeline->reset();
#ifdef BT_USE_BODY_UPDATE_REVISION
		m_bodyUpdateRevisions.resize(0);
#endif //BT_USE_BODY_UPDATE_REVISION
	}

}


void b3GpuDynamicsWorld::reset()
{
	m_staticBody = 0;

	if (m_collisionObjects.size())
		b3Warning("m_collisionObjects should be empty before calling b3GpuDynamicsWorld::reset");
	m_collisionObjects.clear();
	if (m_bodyUpdateRevisions.size())
		b3Warning("world (m_bodyUpdateRevisions) should be empty before calling b3GpuDynamicsWorld::reset");
	m_bodyUpdateRevisions.clear();
	
	if (m_constraints.size())
		b3Warning("m_constraints should be empty before calling b3GpuDynamicsWorld::reset");
	m_constraints.clear();


	m_uniqueShapes.resize(0);
	m_uniqueShapeMapping.resize(0);
	m_np->reset();
	m_bp->reset();
	m_rigidBodyPipeline->reset();
#ifdef BT_USE_BODY_UPDATE_REVISION
	m_bodyUpdateRevisions.resize(0);
#endif

	btConvexHullShape* nullShape = new btConvexHullShape();
	m_staticBody = new btRigidBody(0,0,nullShape);
	addRigidBody(m_staticBody,0,0);
}

void	b3GpuDynamicsWorld::removeCollisionObject(btCollisionObject* colObj)
{
	m_cpuGpuSync = true;
	btDynamicsWorld::removeCollisionObject(colObj);
	m_bodyUpdateRevisions.resize(this->m_collisionObjects.size());
	for (int i=0;i<m_bodyUpdateRevisions.size();i++)
	{
		m_bodyUpdateRevisions[i] = -1;
	}
	if (getNumCollisionObjects()==0)
	{
		reset();
	}

}

void	b3GpuDynamicsWorld::rayTest(const btVector3& rayFromWorld, const btVector3& rayToWorld, RayResultCallback& resultCallback) const
{
	b3AlignedObjectArray<b3RayInfo> rays;
	b3RayInfo ray;
	ray.m_from = (const b3Vector3&)rayFromWorld;
	ray.m_to = (const b3Vector3&)rayToWorld;
	rays.push_back(ray);

	b3AlignedObjectArray<b3RayHit> hitResults;
	b3RayHit hit;
	hit.m_hitFraction = 1.f;

	hitResults.push_back(hit);

	m_rigidBodyPipeline->castRays(rays,hitResults);
	b3Printf("hit = %f\n", hitResults[0].m_hitFraction);
	if (hitResults[0].m_hitFraction<1.f)
	{
		b3Assert(hitResults[0].m_hitBody >=0);
		b3Assert(hitResults[0].m_hitBody < m_collisionObjects.size());
		b3Vector3 hitNormalLocal = hitResults[0].m_hitNormal;
		btCollisionObject* colObj = m_collisionObjects[hitResults[0].m_hitBody];
		LocalRayResult rayResult(colObj,0,(btVector3&)hitNormalLocal,hitResults[0].m_hitFraction);
		rayResult.m_hitFraction = hitResults[0].m_hitFraction;

		resultCallback.addSingleResult(rayResult,true);
	}
	

}

void	b3GpuDynamicsWorld::synchronizeMotionStates()
{
	//iterate  over all collision objects
	for ( int i=0;i<m_collisionObjects.size();i++)
	{
		btCollisionObject* colObj = m_collisionObjects[i];
		btRigidBody* body = btRigidBody::upcast(colObj);
		if (body)
			synchronizeSingleMotionState(body);
	}
}


void	b3GpuDynamicsWorld::synchronizeSingleMotionState(btRigidBody* body)
{
	btAssert(body);

	if (body->getMotionState() && !body->isStaticOrKinematicObject())
	{
		//we need to call the update at least once, even for sleeping objects
		//otherwise the 'graphics' transform never updates properly
		const btTransform& centerOfMassWorldTrans = body->getWorldTransform();
		body->getMotionState()->setWorldTransform(centerOfMassWorldTrans);
	}
}

void	b3GpuDynamicsWorld::debugDrawWorld()
{
	BT_PROFILE("debugDrawWorld");

	btCollisionWorld::debugDrawWorld();
}

void	b3GpuDynamicsWorld::addConstraint(btTypedConstraint* constraint, bool disableCollisionsBetweenLinkedBodies)
{
	constraint->setUserConstraintPtr(0);

	m_constraints.push_back(constraint);
	
	switch (constraint->getConstraintType())
	{
	case POINT2POINT_CONSTRAINT_TYPE:
		{
			btPoint2PointConstraint* p = (btPoint2PointConstraint*) constraint;
			int rbA = p->getRigidBodyA().getUserIndex();
			int rbB = p->getRigidBodyB().getUserIndex();
			btVector3 pivotInB = p->getPivotInB();
			if (rbB<=0)
			{
				pivotInB = p->getRigidBodyA().getWorldTransform()*p->getPivotInA();
				rbB = m_staticBody->getUserIndex();
			}
			if (rbA>=0 && rbB>=0)
			{
				b3Point2PointConstraint* p2p = new b3Point2PointConstraint(rbA,rbB, (const b3Vector3&)p->getPivotInA(),(const b3Vector3&)pivotInB);
				p2p->setBreakingImpulseThreshold(p->getBreakingImpulseThreshold());
				constraint->setUserConstraintPtr(p2p);
				m_rigidBodyPipeline->addConstraint(p2p);
			} else
			{
				b3Error("invalid body index in addConstraint,b3Point2PointConstraint\n");
			}
			break;
		}
	case D6_CONSTRAINT_TYPE:
		{
			btGeneric6DofConstraint* dof2  = (btGeneric6DofConstraint*) constraint;
			const b3RigidBodyData* bodiesCL = m_np->getBodiesCpu();

			int rbA = dof2->getRigidBodyA().getUserIndex();
			int rbB = dof2->getRigidBodyB().getUserIndex();

			btTransform frameInA = dof2->getFrameOffsetB();
			btTransform frameInB = dof2->getFrameOffsetB();

			if (rbA<=0)
			{
				frameInA = dof2->getRigidBodyB().getWorldTransform()*dof2->getFrameOffsetB();
				rbA = m_staticBody->getUserIndex();
			}

			if (rbB<=0)
			{
				frameInB = dof2->getRigidBodyA().getWorldTransform()*dof2->getFrameOffsetA();
				rbB = m_staticBody->getUserIndex();
			}
			if (rbA>=0 && rbB>=0)
			{
				b3Generic6DofConstraint* dof3 = new b3Generic6DofConstraint(rbA,rbB,(b3Transform&)frameInA,(b3Transform&)frameInB,false,bodiesCL);//(();//(rbA,rbB, (const b3Vector3&)p->getPivotInA(),(const b3Vector3&)pivotInB);
				{
					btVector3 limit(0,0,0);
					dof2->getLinearLowerLimit(limit);
					dof3->setLinearLowerLimit((const b3Vector3&)limit);

					dof2->setLinearUpperLimit(limit);
					dof3->setLinearUpperLimit((const b3Vector3&)limit);

					dof2->setAngularLowerLimit(limit);
					dof3->setAngularLowerLimit((const b3Vector3&)limit);

					dof2->setAngularUpperLimit(limit);
					dof3->setAngularUpperLimit((const b3Vector3&)limit);
				/*	for (int i=0;i<6;i++)
					{
						dof3->setParam(BT_CONSTRAINT_STOP_CFM,dof2->getParam(BT_CONSTRAINT_STOP_CFM,i),i);
						dof3->setParam(BT_CONSTRAINT_STOP_ERP,dof2->getParam(BT_CONSTRAINT_STOP_ERP,i),i);
					}
					*/
					dof3->setBreakingImpulseThreshold(dof2->getBreakingImpulseThreshold());
				}
//				p2p->setBreakingImpulseThreshold(p->getBreakingImpulseThreshold());
				constraint->setUserConstraintPtr(dof3);
				m_rigidBodyPipeline->addConstraint(dof3);
			} else
			{
				b3Error("invalid body index in addConstraint, btGeneric6DofConstraint.\n");
			}

//			b3Generic6DofConstraint
			break;
		}
	default:
		b3Warning("Warning: b3GpuDynamicsWorld::addConstraint with unsupported constraint type\n");
	};


}

void	b3GpuDynamicsWorld::removeConstraint(btTypedConstraint* constraint)
{
	b3TypedConstraint* c = (b3TypedConstraint*) constraint->getUserConstraintPtr();

	if (c)
	{
		this->m_rigidBodyPipeline->removeConstraint(c);
		delete c;
	}
	m_constraints.remove(constraint);
}

