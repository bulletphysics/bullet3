/*
Physics Effects Copyright(C) 2011 Sony Computer Entertainment Inc.
All rights reserved.

Physics Effects is open software; you can redistribute it and/or
modify it under the terms of the BSD License.

Physics Effects is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the BSD License for more details.

A copy of the BSD License is distributed with
Physics Effects under the filename: physics_effects_license.txt
*/

#include "btBulletPhysicsEffectsWorld.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "BulletDynamics/ConstraintSolver/btTypedConstraint.h"
#include "BulletDynamics/ConstraintSolver/btContactConstraint.h"
#include "LinearMath/btAabbUtil2.h"
#include "BulletMultiThreaded/SequentialThreadSupport.h"
#include "BulletCollision/CollisionShapes/btCollisionShape.h"
#include "LinearMath/btIDebugDraw.h"
#include "LinearMath/btQuickprof.h"

#include "BulletDynamics/ConstraintSolver/btContactSolverInfo.h"

#include "BulletDynamics/ConstraintSolver/btConstraintSolver.h"


#include "BulletPhysicsEffects/btLowLevelData.h"
#include <physics_effects/low_level/pfx_low_level_include.h>
#include <physics_effects/util/pfx_util_include.h>
#include "btLowLevelBroadphase.h"


#include "BulletCollision/CollisionShapes/btCompoundShape.h"//required for shape conversion
#include "BulletCollision/CollisionShapes/btConvexHullShape.h"//required for shape conversion
#include "BulletCollision/CollisionShapes/btCylinderShape.h"//required for shape conversion
#include "BulletCollision/CollisionShapes/btSphereShape.h"//required for shape conversion
#include "BulletCollision/CollisionShapes/btCapsuleShape.h"//required for shape conversion
#include "BulletCollision/CollisionShapes/btBoxShape.h"//required for shape conversion
#include "BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h"
#include "BulletMultiThreaded/vectormath2bullet.h"

#ifdef _WIN32
#include "BulletMultiThreaded/Win32ThreadSupport.h"
#endif


#ifdef USE_PE_GATHER_SCATTER_SPURS_TASK
#include "SpuDispatch/BulletPEGatherScatterSpursSupport.h"
#include "SpuDispatch/SpuPEGatherScatterTaskProcess.h"
#endif


btBulletPhysicsEffectsWorld::btBulletPhysicsEffectsWorld(btLowLevelData* lowLevelData, btDispatcher* dispatcher,btLowLevelBroadphase* broadphase,btConstraintSolver* constraintSolver,btCollisionConfiguration* collisionConfiguration, btThreadSupportInterface* threadSupport)
:btDiscreteDynamicsWorld(dispatcher,broadphase,constraintSolver,collisionConfiguration),
m_lowLevelData(lowLevelData)
{

#ifdef USE_PE_GATHER_SCATTER_SPURS_TASK
	int numGatherScatterSpus = threadSupport->getNumTasks();
	m_PEGatherScatterProcess = new SpuPEGatherScatterTaskProcess(threadSupport,numGatherScatterSpus);
#endif //USE_PE_GATHER_SCATTER_SPURS_TASK

	m_lowLevelStates.resize(broadphase->m_maxHandles);//or use expandNonInitializing?
	m_lowLevelBodies.resize(broadphase->m_maxHandles);
	m_lowLevelSolverBodies.resize(broadphase->m_maxHandles);
	m_lowLevelCollidables.resize(broadphase->m_maxHandles);

	m_lowLevelData->m_states = &m_lowLevelStates[0];
	m_lowLevelData->m_collidables = &m_lowLevelCollidables[0];
	m_lowLevelData->m_bodies = &m_lowLevelBodies[0];
	m_lowLevelData->m_solverBodies = &m_lowLevelSolverBodies[0];
	m_lowLevelData->m_numRigidBodies = broadphase->m_maxHandles;

}
		
btBulletPhysicsEffectsWorld::~btBulletPhysicsEffectsWorld()
{
#ifdef USE_PE_GATHER_SCATTER_SPURS_TASK
	delete	m_PEGatherScatterProcess;
#endif

}



void	btBulletPhysicsEffectsWorld::integrateTransforms(btScalar timeStep)
{
			///integrate transforms
#ifdef USE_PE_GATHER_SCATTER_SPURS_TASK
	if (getDispatchInfo().m_enableSPU)
	{
		BT_PROFILE("integrateTransformsSPU");
		int numRemainingObjects = m_nonStaticRigidBodies.size();

		int batchSize = PARALLEL_BATCH_SIZE;
		int startIndex = 0;
		while (numRemainingObjects>0)
		{
			int currentBatchSize = numRemainingObjects > batchSize? batchSize : numRemainingObjects;

			//issue and flush is likely to be called every frame, move the construction and deletion out of the inner loop (at construction/init etc)
			m_PEGatherScatterProcess->issueTask(
													CMD_SAMPLE_INTEGRATE_BODIES,
													&m_nonStaticRigidBodies[0],
													0,
													0,
													startIndex,
													currentBatchSize);
			numRemainingObjects -= currentBatchSize;
			startIndex += currentBatchSize;
		}
		
		m_PEGatherScatterProcess->flush();
	} else
#endif //USE_PE_GATHER_SCATTER_SPURS_TASK
	{
//		BT_PROFILE("integrateTransforms");
		btDiscreteDynamicsWorld::integrateTransforms(timeStep);
	}
}


void	btBulletPhysicsEffectsWorld::predictUnconstraintMotion(btScalar timeStep)
{
#ifdef USE_PE_GATHER_SCATTER_SPURS_TASK
	if (getDispatchInfo().m_enableSPU)
	{
		BT_PROFILE("predictUnconstraintMotionSPU");
		int numRemainingObjects = m_nonStaticRigidBodies.size();

		int batchSize = PARALLEL_BATCH_SIZE;
		int startIndex=0;
		while (numRemainingObjects>0)
		{
			int currentBatchSize = numRemainingObjects > batchSize? batchSize : numRemainingObjects;

			//issue and flush is likely to be called every frame, move the construction and deletion out of the inner loop (at construction/init etc)
			m_PEGatherScatterProcess->issueTask(
													CMD_SAMPLE_PREDICT_MOTION_BODIES,
													&m_nonStaticRigidBodies[0],
													0,
													0,
													startIndex,
													currentBatchSize);
			numRemainingObjects -= currentBatchSize;
			startIndex += currentBatchSize;
		}
		
		m_PEGatherScatterProcess->flush();
	}
	else
#endif //USE_PE_GATHER_SCATTER_SPURS_TASK
	{
		btDiscreteDynamicsWorld::predictUnconstraintMotion( timeStep);
	}

	for ( int i=0;i<m_nonStaticRigidBodies.size();i++)
	{
		btRigidBody* body = m_nonStaticRigidBodies[i];
		body->setHitFraction(1.f);

		if (body->isActive() && (!body->isStaticOrKinematicObject()))
		{
			syncRigidBodyState(body);
		}
	}
}



void	btBulletPhysicsEffectsWorld::solveConstraints(btContactSolverInfo& solverInfo)
{
	BT_PROFILE("solveConstraints");

	btCollisionDispatcher* disp = (btCollisionDispatcher*) getDispatcher();
	int numBodies = getNumCollisionObjects();
	
	btPersistentManifold** manifolds = disp->getInternalManifoldPointer();
	
	int numManifolds = disp->getNumManifolds();
	
	
	if ((getNumCollisionObjects()>0) && (numManifolds + m_constraints.size()>0))
	{

		btCollisionObject** bodies = numBodies ? &getCollisionObjectArray()[0] : 0;
		btTypedConstraint** constraints = m_constraints.size() ? &m_constraints[0] : 0;

		getConstraintSolver()->solveGroup( bodies,numBodies, disp->getInternalManifoldPointer(),numManifolds, constraints, m_constraints.size() ,m_solverInfo,m_debugDrawer,m_stackAlloc,disp);
	}
}


void	btBulletPhysicsEffectsWorld::calculateSimulationIslands()
{

}

static void	convertShape(btCollisionShape* bulletShape, btAlignedObjectArray<sce::PhysicsEffects::PfxShape>& shapes)
{
	switch (bulletShape->getShapeType())
	{
	case BOX_SHAPE_PROXYTYPE:
		{
			btBoxShape* boxshape = (btBoxShape*)bulletShape;
			sce::PhysicsEffects::PfxBox box(boxshape->getHalfExtentsWithMargin().getX(),boxshape->getHalfExtentsWithMargin().getY(),boxshape->getHalfExtentsWithMargin().getZ());
			sce::PhysicsEffects::PfxShape& shape = shapes.expand();
			shape.reset();
			shape.setBox(box);
			break;
		}

	case TRIANGLE_MESH_SHAPE_PROXYTYPE:
		{
			btBvhTriangleMeshShape* trimesh = (btBvhTriangleMeshShape*) bulletShape;

			int numSubParts = trimesh->getMeshInterface()->getNumSubParts();
			btAssert(numSubParts>0);
			
			for (int i=0;i<numSubParts;i++)
			{
				unsigned char* vertexBase=0;
				int numVerts = 0;
				PHY_ScalarType vertexType;
				int vertexStride=0;
				unsigned char* indexBase=0;
				int indexStride=0;
				int numFaces=0;
				PHY_ScalarType indexType;

				trimesh->getMeshInterface()->getLockedVertexIndexBase(&vertexBase,numVerts,vertexType,vertexStride,&indexBase,indexStride,numFaces,indexType,i);

				sce::PhysicsEffects::PfxCreateLargeTriMeshParam param;
				btAssert(param.flag&SCE_PFX_MESH_FLAG_16BIT_INDEX);
				unsigned short int* copyIndices = new unsigned short int[numFaces*3];
				switch (indexType)
				{
				case PHY_UCHAR:	
					{
						for (int p=0;p<numFaces;p++)
						{
							copyIndices[p*3]=indexBase[p*indexStride];
							copyIndices[p*3+1]=indexBase[p*indexStride+1];
							copyIndices[p*3+2]=indexBase[p*indexStride+2];
						}
						break;
					}
					//PHY_INTEGER:
					//PHY_SHORT:
				default:
					{
						btAssert(0);
					}
				};
				
				param.verts = (float*)vertexBase;
				param.numVerts = numVerts;
				param.vertexStrideBytes = vertexStride;

				param.triangles = copyIndices;
				param.numTriangles = numFaces;
				param.triangleStrideBytes = sizeof(unsigned short int)*3;
				
				sce::PhysicsEffects::PfxLargeTriMesh* largeMesh = new sce::PhysicsEffects::PfxLargeTriMesh();

				sce::PhysicsEffects::PfxInt32 ret = pfxCreateLargeTriMesh(*largeMesh,param);
				if(ret != SCE_PFX_OK) {
					SCE_PFX_PRINTF("Can't create large mesh.\n");
				}

				sce::PhysicsEffects::PfxShape& shape = shapes.expand();
				shape.reset();
				shape.setLargeTriMesh(largeMesh);
			}

			break;
		}
	case SPHERE_SHAPE_PROXYTYPE:
		{
			btSphereShape* sphereshape = (btSphereShape*)bulletShape;
			sce::PhysicsEffects::PfxSphere sphere(sphereshape->getRadius());
			sce::PhysicsEffects::PfxShape& shape = shapes.expand();
			shape.reset();
			shape.setSphere(sphere);
			break;
		}
	case CAPSULE_SHAPE_PROXYTYPE:
		{
			btCapsuleShape* capsuleshape= (btCapsuleShape*)bulletShape;//assume btCapsuleShapeX for now
			sce::PhysicsEffects::PfxCapsule capsule(capsuleshape->getHalfHeight(),capsuleshape->getRadius());
			sce::PhysicsEffects::PfxShape& shape = shapes.expand();
			shape.reset();
			shape.setCapsule(capsule);
			break;
		}
	case CYLINDER_SHAPE_PROXYTYPE:
		{
			btCylinderShape* cylindershape= (btCylinderShape*)bulletShape;//assume btCylinderShapeX for now
			sce::PhysicsEffects::PfxCylinder cylinder(cylindershape->getHalfExtentsWithMargin()[0],cylindershape->getRadius());
			sce::PhysicsEffects::PfxShape& shape = shapes.expand();
			shape.reset();
			shape.setCylinder(cylinder);
			break;
		}
	case CONVEX_HULL_SHAPE_PROXYTYPE:
		{
			btConvexHullShape* convexHullShape = (btConvexHullShape*)bulletShape;
			
			sce::PhysicsEffects::PfxConvexMesh* convex = new sce::PhysicsEffects::PfxConvexMesh();
			convex->m_numVerts = convexHullShape->getNumPoints();
			convex->m_numIndices = 0;//todo for ray intersection test support
			
			for (int i=0;i<convex->m_numVerts;i++)
			{
				convex->m_verts[i].setX(convexHullShape->getPoints()[i].getX());
				convex->m_verts[i].setY(convexHullShape->getPoints()[i].getY());
				convex->m_verts[i].setZ(convexHullShape->getPoints()[i].getZ());
			}

			convex->updateAABB();
			sce::PhysicsEffects::PfxShape& shape = shapes.expand();
			shape.reset();
			shape.setConvexMesh(convex);
			break;
		}
	case COMPOUND_SHAPE_PROXYTYPE:
		{
			btCompoundShape* compound = (btCompoundShape*) bulletShape;
			
			for (int s=0;s<compound->getNumChildShapes();s++)
			{
				convertShape(compound->getChildShape(s),shapes);

				sce::PhysicsEffects::PfxMatrix3 rotMat = getVmMatrix3(compound->getChildTransform(s).getBasis());
				sce::PhysicsEffects::PfxVector3 translate = getVmVector3(compound->getChildTransform(s).getOrigin());
				sce::PhysicsEffects::PfxTransform3 childtransform(rotMat,translate);
				shapes[shapes.size()-1].setOffsetTransform(childtransform);
			}

			break;
		}


	default:
		{
			btAssert(0);
		}
	};

}

void	btBulletPhysicsEffectsWorld::createStateAndCollidable(btRigidBody* body)
{
	
	int objectIndex = body->getBroadphaseProxy()->m_uniqueId;
	btAssert(objectIndex>=0);
	//btAssert(objectIndex<m_maxHandles);

	//initialize it
	sce::PhysicsEffects::PfxRigidBody* pfxbody = &m_lowLevelBodies[objectIndex];
	sce::PhysicsEffects::PfxRigidState* pfxstate = &m_lowLevelStates[objectIndex];
	sce::PhysicsEffects::PfxCollidable* pfxcollidable = &m_lowLevelCollidables[objectIndex];

	///convert/initialize body, shape, state and collidable
	pfxstate->reset();
	pfxbody->reset();
	pfxcollidable->reset();
	
	
	
	pfxbody->setFriction(body->getFriction());
	pfxbody->setRestitution(body->getRestitution());

	if (body->getInvMass())
	{
		btScalar mass = 1.f/body->getInvMass();
		pfxbody->setMass(mass);
		Vectormath::Aos::Matrix3 inertiaInv = inertiaInv.identity();
		inertiaInv.setElem(0,0,body->getInvInertiaDiagLocal().getX());
		inertiaInv.setElem(1,1,body->getInvInertiaDiagLocal().getY());
		inertiaInv.setElem(2,2,body->getInvInertiaDiagLocal().getZ());
		pfxbody->setInertiaInv(inertiaInv);
		pfxstate->setMotionType(sce::PhysicsEffects::kPfxMotionTypeActive);
	} else
	{
		pfxstate->setMotionType(sce::PhysicsEffects::kPfxMotionTypeFixed);
	}

	btAlignedObjectArray<sce::PhysicsEffects::PfxShape> shapes;
	
	convertShape(body->getCollisionShape(), shapes);

	btAssert(shapes.size()>0);

	if (shapes.size()==1)
	{
		pfxcollidable->addShape(shapes[0]);
		pfxcollidable->finish();
	} else
	{
		if (shapes.size()>1)
		{
			sce::PhysicsEffects::PfxUInt16* ints=new sce::PhysicsEffects::PfxUInt16[shapes.size()];
			sce::PhysicsEffects::PfxShape* pfxshapes = new sce::PhysicsEffects::PfxShape[shapes.size()];
			int p;
			for (p=0;p<shapes.size();p++)
			{
				ints[p]=p;
				pfxshapes[p] = shapes[p];
			}
			pfxcollidable->reset(pfxshapes,ints,shapes.size());
			
			for (p=0;p<shapes.size();p++)
			{
				pfxcollidable->addShape(pfxshapes[p]);
			}
			
			pfxcollidable->finish();
		}
	}
	pfxstate->setRigidBodyId(objectIndex);

	syncRigidBodyState(body);
		
	
}



void	btBulletPhysicsEffectsWorld::syncRigidBodyState(btRigidBody* body)
{
	int objectIndex = body->getBroadphaseProxy()->m_uniqueId;
	sce::PhysicsEffects::PfxRigidState* pfxstate = &m_lowLevelStates[objectIndex];

	pfxstate->setPosition(sce::PhysicsEffects::PfxVector3(body->getWorldTransform().getOrigin()[0],body->getWorldTransform().getOrigin()[1],body->getWorldTransform().getOrigin()[2]));
	sce::PhysicsEffects::PfxQuat rot(body->getOrientation().getX(),body->getOrientation().getY(),body->getOrientation().getZ(),body->getOrientation().getW());
	pfxstate->setOrientation(rot);

}
	


void	btBulletPhysicsEffectsWorld::addRigidBody(btRigidBody* body)
{

	btDiscreteDynamicsWorld::addRigidBody(body);

	//create a state and collidable
		
	createStateAndCollidable(body);


	


//	m_lowLevelData->m_numRigidBodies++;
//	btAssert(m_lowLevelData->m_numRigidBodies<m_lowLevelData->m_maxNumRigidBodies);
	
}

void	btBulletPhysicsEffectsWorld::addRigidBody(btRigidBody* body, short group, short mask)
{
	btDiscreteDynamicsWorld::addRigidBody(body,group,mask);

}

void	btBulletPhysicsEffectsWorld::removeRigidBody(btRigidBody* body)
{
	btDiscreteDynamicsWorld::removeRigidBody(body);

//	m_lowLevelData->m_numRigidBodies--;
//	btAssert(m_lowLevelData->m_numRigidBodies>=0);
}

