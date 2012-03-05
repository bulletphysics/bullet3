/*
Copyright (c) 2012 Advanced Micro Devices, Inc.  

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
//Originally written by Erwin Coumans

#include "CustomCollisionDispatcher.h"
#include "BulletCollision/BroadphaseCollision/btCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "CustomConvexShape.h"
#include "CustomConvexPairCollision.h"
#include "LinearMath/btQuickprof.h"



#ifdef CL_PLATFORM_AMD

#include "Adl/Adl.h"
#include "Stubs/AdlMath.h"
#include "Stubs/AdlContact4.h"
#include "Stubs/AdlQuaternion.h"
#include "Stubs/ChNarrowPhase.h"

#include "Stubs/Solver.h"


struct	CustomDispatchData
{
	adl::DeviceCL* m_ddcl;
	adl::Device* m_deviceHost;
	ShapeDataType m_ShapeBuffer;
	
	adl::HostBuffer<int2>* m_pBufPairsCPU;
	adl::Buffer<int2>* m_pBufPairsGPU;
	adl::Buffer<Contact4>* m_pBufContactOutGPU;
	adl::HostBuffer<Contact4>* m_pBufContactOutCPU;
	adl::ChNarrowphase<adl::TYPE_CL>::Data* m_Data;

	adl::HostBuffer<RigidBodyBase::Body>* m_pBufRBodiesCPU;
	adl::Buffer<RigidBodyBase::Body>* m_pBufRBodiesGPU;

	adl::Buffer<RigidBodyBase::Shape>*	m_bodyInfoBufferCPU;
	adl::Buffer<RigidBodyBase::Shape>*	m_bodyInfoBufferGPU;

	adl::Solver<adl::TYPE_CL>::Data* m_solverDataGPU;
	SolverData		m_contactCGPU;
	void*			m_frictionCGPU;

	int m_numAcceleratedShapes;
};
#endif //CL_PLATFORM_AMD

CustomCollisionDispatcher::CustomCollisionDispatcher(btCollisionConfiguration* collisionConfiguration
#ifdef CL_PLATFORM_AMD
		, cl_context context,cl_device_id device,cl_command_queue queue
#endif //CL_PLATFORM_AMD
):btCollisionDispatcher(collisionConfiguration),
m_internalData(0)
{
#ifdef CL_PLATFORM_AMD

	if (context && queue)
	{
		m_internalData = new CustomDispatchData();
		memset(m_internalData,0,sizeof(CustomDispatchData));

		adl::DeviceUtils::Config cfg;
		m_internalData->m_ddcl = new adl::DeviceCL();
		m_internalData->m_ddcl->m_deviceIdx = device;
		m_internalData->m_ddcl->m_context = context;
		m_internalData->m_ddcl->m_commandQueue = queue;
		m_internalData->m_ddcl->m_kernelManager = new adl::KernelManager;


		m_internalData->m_deviceHost = adl::DeviceUtils::allocate( adl::TYPE_HOST, cfg );
		m_internalData->m_pBufPairsCPU = new adl::HostBuffer<int2>(m_internalData->m_deviceHost, MAX_BROADPHASE_COLLISION_CL);
		m_internalData->m_pBufContactOutCPU = new adl::HostBuffer<Contact4>(m_internalData->m_deviceHost, MAX_BROADPHASE_COLLISION_CL);
		m_internalData->m_pBufRBodiesCPU = new adl::HostBuffer<RigidBodyBase::Body>(m_internalData->m_deviceHost, MAX_CONVEX_BODIES_CL);
		
		m_internalData->m_bodyInfoBufferCPU = new adl::Buffer<RigidBodyBase::Shape>(m_internalData->m_deviceHost,MAX_CONVEX_BODIES_CL);
		m_internalData->m_pBufContactOutGPU = new adl::Buffer<Contact4>(m_internalData->m_ddcl, MAX_BROADPHASE_COLLISION_CL);
		m_internalData->m_bodyInfoBufferGPU = new adl::Buffer<RigidBodyBase::Shape>(m_internalData->m_ddcl,MAX_CONVEX_BODIES_CL);
		m_internalData->m_pBufPairsGPU = new adl::Buffer<int2>(m_internalData->m_ddcl, MAX_BROADPHASE_COLLISION_CL);
		m_internalData->m_solverDataGPU = adl::Solver<adl::TYPE_CL>::allocate( m_internalData->m_ddcl, MAX_BROADPHASE_COLLISION_CL);
		m_internalData->m_pBufRBodiesGPU = new adl::Buffer<RigidBodyBase::Body>(m_internalData->m_ddcl, MAX_CONVEX_BODIES_CL);
		m_internalData->m_Data = adl::ChNarrowphase<adl::TYPE_CL>::allocate(m_internalData->m_ddcl);
		m_internalData->m_ShapeBuffer = adl::ChNarrowphase<adl::TYPE_CL>::allocateShapeBuffer(m_internalData->m_ddcl, MAX_CONVEX_SHAPES_CL);	
		m_internalData->m_numAcceleratedShapes = 0;

		m_internalData->m_contactCGPU = adl::Solver<adl::TYPE_CL>::allocateConstraint4( m_internalData->m_ddcl, MAX_BROADPHASE_COLLISION_CL);
		m_internalData->m_frictionCGPU = adl::Solver<adl::TYPE_CL>::allocateFrictionConstraint( m_internalData->m_ddcl, MAX_BROADPHASE_COLLISION_CL);

	}



#endif //CL_PLATFORM_AMD
}

CustomCollisionDispatcher::~CustomCollisionDispatcher(void)
{
#ifdef CL_PLATFORM_AMD
	if (m_internalData)
	{
		delete m_internalData->m_pBufPairsCPU;
		delete m_internalData->m_pBufPairsGPU;
		delete m_internalData->m_pBufContactOutGPU;
		delete m_internalData->m_pBufContactOutCPU;

		adl::Solver<adl::TYPE_CL>::deallocateConstraint4( m_internalData->m_contactCGPU );
		adl::Solver<adl::TYPE_CL>::deallocateFrictionConstraint( m_internalData->m_frictionCGPU );


		adl::Solver<adl::TYPE_CL>::deallocate(m_internalData->m_solverDataGPU);

		adl::DeviceUtils::deallocate(m_internalData->m_deviceHost);
		delete m_internalData->m_ddcl;		
		delete m_internalData;
	}
	
#endif //CL_PLATFORM_AMD

}


#ifdef CL_PLATFORM_AMD
#include "BulletDynamics/Dynamics/btRigidBody.h"

RigidBodyBase::Shape CreateBodyInfo(const btCollisionObject& colObj)
{
	RigidBodyBase::Shape shape;
	const btRigidBody* bulletBody = btRigidBody::upcast(&colObj);
	if( colObj.isStaticOrKinematicObject() || !bulletBody)
	{

		//body.m_quat = qtGetIdentity();
		//body.m_invMass = 0.f;
		shape.m_initInvInertia = mtZero();
		shape.m_invInertia = mtZero();
	}
	else
	{

		btVector3 invLocalInertia = bulletBody->getInvInertiaDiagLocal();
		shape.m_initInvInertia = mtZero();
		shape.m_initInvInertia.m_row[0].x = invLocalInertia.x();
		shape.m_initInvInertia.m_row[1].y = invLocalInertia.y();
		shape.m_initInvInertia.m_row[2].z = invLocalInertia.z();

		btQuaternion q = colObj.getWorldTransform().getRotation();
		Quaternion qBody;	
		qBody.x = q.getX();
		qBody.y = q.getY();
		qBody.z = q.getZ();
		qBody.w = q.getW();

		Matrix3x3 m = qtGetRotationMatrix( qBody);
		Matrix3x3 mT = mtTranspose( m );
		shape.m_invInertia = mtMul( mtMul( m, shape.m_initInvInertia ), mT );
		//bulletBody->getInvInertiaTensorWorld();




	//	shape.m_initInvInertia = mtInvert( localInertia );
	}
	return shape;
}

RigidBodyBase::Body CreateRBodyCL(const btCollisionObject& colObj, int shapeIdx)
{
	RigidBodyBase::Body bodyCL;


	// position
	const btVector3& p = colObj.getWorldTransform().getOrigin();
	bodyCL.m_pos.x = p.getX();
	bodyCL.m_pos.y = p.getY();
	bodyCL.m_pos.z = p.getZ();
	bodyCL.m_pos.w = 0.0f;

	// quaternion
	btQuaternion q = colObj.getWorldTransform().getRotation();
	bodyCL.m_quat.x = q.getX();
	bodyCL.m_quat.y = q.getY();
	bodyCL.m_quat.z = q.getZ();
	bodyCL.m_quat.w = q.getW();

	const btRigidBody* bulletBody = btRigidBody::upcast(&colObj);
	if( colObj.isStaticOrKinematicObject() || !bulletBody)
	{
		// linear velocity
		bodyCL.m_linVel = make_float4(0.0f, 0.0f, 0.0f);

		// angular velocity
		bodyCL.m_angVel = make_float4(0.0f, 0.0f, 0.0f);
		bodyCL.m_invMass = 0.f;
	} else
	{
		// linear velocity
		const btVector3& lv = bulletBody->getLinearVelocity();
		const btVector3& av = bulletBody->getAngularVelocity();

		bodyCL.m_linVel = make_float4(lv.x(),lv.y(),lv.z(),0.0f);
		// angular velocity
		bodyCL.m_angVel = make_float4(av.x(),av.y(),av.z(),0.0f);
		bodyCL.m_invMass = bulletBody->getInvMass();
	}
	// shape index
	bodyCL.m_shapeIdx = shapeIdx; 


	// restituition coefficient
	bodyCL.m_restituitionCoeff = colObj.getRestitution();

	// friction coefficient
	bodyCL.m_frictionCoeff = colObj.getFriction();

	return bodyCL;
}
#endif //CL_PLATFORM_AMD

void CustomCollisionDispatcher::dispatchAllCollisionPairs(btOverlappingPairCache* pairCache,const btDispatcherInfo& dispatchInfo,btDispatcher* dispatcher) 
{
	BT_PROFILE("CustomCollisionDispatcher::dispatchAllCollisionPairs");
	{
	btBroadphasePairArray& overlappingPairArray = pairCache->getOverlappingPairArray();
	bool bGPU = (m_internalData != 0);
#ifdef CL_PLATFORM_AMD
	if ( !bGPU )
#endif //CL_PLATFORM_AMD
	{
		BT_PROFILE("btCollisionDispatcher::dispatchAllCollisionPairs");
		btCollisionDispatcher::dispatchAllCollisionPairs(pairCache,dispatchInfo,dispatcher);
	}
#ifdef CL_PLATFORM_AMD

	else
	{
		{
			BT_PROFILE("refreshContactPoints");
			//----------------------------------------------------------------
			// GPU version of convex heightmap narrowphase collision detection
			//----------------------------------------------------------------
			for ( int i = 0; i < getNumManifolds(); i++ )
			{
				btPersistentManifold* manifold = getManifoldByIndexInternal(i);


				btCollisionObject* body0 = (btCollisionObject*)manifold->getBody0();
				btCollisionObject* body1 = (btCollisionObject*)manifold->getBody1();

				manifold->refreshContactPoints(body0->getWorldTransform(),body1->getWorldTransform());
			}
		}

		// OpenCL 
		int nColPairsFromBP = overlappingPairArray.size();
		btAssert(MAX_BROADPHASE_COLLISION_CL >= nColPairsFromBP);

		int maxBodyIndex = -1;

		{
			BT_PROFILE("CreateRBodyCL and GPU pairs");
			for ( int i=0; i<overlappingPairArray.size(); i++)
			{
				btAssert(i<MAX_BROADPHASE_COLLISION_CL);

				btBroadphasePair* pair = &overlappingPairArray[i];

				btCollisionObject* colObj0 = (btCollisionObject*)pair->m_pProxy0->m_clientObject;
				btCollisionObject* colObj1 = (btCollisionObject*)pair->m_pProxy1->m_clientObject;

				int bodyIndex0 = colObj0->getCompanionId();
				int bodyIndex1 = colObj1->getCompanionId();

				//keep a one-to-one mapping between Bullet and Adl broadphase pairs
				(*m_internalData->m_pBufPairsCPU)[i].x = bodyIndex0;
				(*m_internalData->m_pBufPairsCPU)[i].y = bodyIndex1;

				if (bodyIndex0>=0 && bodyIndex1>=0)
				{
					//create companion shapes (if necessary)

					btAssert(colObj0->getCollisionShape()->getShapeType() == CUSTOM_POLYHEDRAL_SHAPE_TYPE);
					btAssert(colObj1->getCollisionShape()->getShapeType() == CUSTOM_POLYHEDRAL_SHAPE_TYPE);

					CustomConvexShape* convexShape0 = (CustomConvexShape*)colObj0->getCollisionShape();
					CustomConvexShape* convexShape1 = (CustomConvexShape*)colObj1->getCollisionShape();

					if (convexShape0->m_acceleratedCompanionShapeIndex<0)
					{
						convexShape0->m_acceleratedCompanionShapeIndex = m_internalData->m_numAcceleratedShapes;
						adl::ChNarrowphase<adl::TYPE_CL>::setShape(m_internalData->m_ShapeBuffer, convexShape0->m_ConvexHeightField, convexShape0->m_acceleratedCompanionShapeIndex, 0.0f);
						m_internalData->m_numAcceleratedShapes++;
					}
					if (convexShape1->m_acceleratedCompanionShapeIndex<0)
					{
						convexShape1->m_acceleratedCompanionShapeIndex = m_internalData->m_numAcceleratedShapes;
						adl::ChNarrowphase<adl::TYPE_CL>::setShape(m_internalData->m_ShapeBuffer, convexShape1->m_ConvexHeightField, convexShape1->m_acceleratedCompanionShapeIndex, 0.0f);
						m_internalData->m_numAcceleratedShapes++;
					}

					btAssert(m_internalData->m_numAcceleratedShapes<MAX_CONVEX_SHAPES_CL);

					if (bodyIndex0>maxBodyIndex)
						maxBodyIndex = bodyIndex0;
					if (bodyIndex1>maxBodyIndex)
						maxBodyIndex = bodyIndex1;

					btAssert(maxBodyIndex<MAX_CONVEX_BODIES_CL);
					if (maxBodyIndex>=MAX_CONVEX_BODIES_CL)
					{
						printf("error: maxBodyIndex(%d)>MAX_CONVEX_BODIES_CL(%d)\n",maxBodyIndex,MAX_CONVEX_BODIES_CL);
					}

					(*m_internalData->m_pBufRBodiesCPU)[bodyIndex0] = CreateRBodyCL(*colObj0, convexShape0->m_acceleratedCompanionShapeIndex);
					m_internalData->m_bodyInfoBufferCPU->m_ptr[bodyIndex0] = CreateBodyInfo(*colObj0);
					(*m_internalData->m_pBufRBodiesCPU)[bodyIndex1] = CreateRBodyCL(*colObj1, convexShape0->m_acceleratedCompanionShapeIndex);
					m_internalData->m_bodyInfoBufferCPU->m_ptr[bodyIndex1] = CreateBodyInfo(*colObj1);
				} else
				{
					//TODO: dispatch using default dispatcher
					btAssert(0);
				}
			}
		}


		if (maxBodyIndex>=0)
		{
			
			int numOfConvexRBodies = maxBodyIndex+1;

			

			adl::ChNarrowphaseBase::Config cfgNP;
			cfgNP.m_collisionMargin = 0.01f;
			int nContactOut = 0;

			{
				BT_PROFILE("ChNarrowphase::execute");
				adl::ChNarrowphase<adl::TYPE_CL>::execute(m_internalData->m_Data, m_internalData->m_pBufPairsGPU, nColPairsFromBP, m_internalData->m_pBufRBodiesGPU, m_internalData->m_ShapeBuffer, m_internalData->m_pBufContactOutGPU, nContactOut, cfgNP);
				adl::DeviceUtils::waitForCompletion(m_internalData->m_ddcl);
			}


			bool useCpu = false;//true;
			bool useSolver = true;//true;//false;
			
			if (useSolver)
			{
				float dt=1./60.;
				adl::SolverBase::ConstraintCfg csCfg( dt );
				csCfg.m_enableParallelSolve = true;
				csCfg.m_averageExtent = 0.2f;//@TODO m_averageObjExtent;
				csCfg.m_staticIdx = -1;//numOfConvexRBodies-1;//m_nBodies-1;

			
			if (useCpu)
			{

				{
					BT_PROFILE("read m_pBufContactOutGPU");
					m_internalData->m_pBufContactOutGPU->read(m_internalData->m_pBufContactOutCPU->m_ptr, nContactOut);//MAX_BROADPHASE_COLLISION_CL);
					adl::DeviceUtils::waitForCompletion(m_internalData->m_ddcl);
				}

				BT_PROFILE("CPU stuff");
				adl::Solver<adl::TYPE_HOST>::Data* solverData = adl::Solver<adl::TYPE_HOST>::allocate( m_internalData->m_deviceHost, nContactOut);

				SolverData contactCPU = adl::Solver<adl::TYPE_HOST>::allocateConstraint4( 
					m_internalData->m_deviceHost, 
					numOfConvexRBodies*MAX_PAIRS_PER_BODY_CL );

				void* frictionCPU = adl::Solver<adl::TYPE_HOST>::allocateFrictionConstraint( 
					m_internalData->m_deviceHost, 
					numOfConvexRBodies*MAX_PAIRS_PER_BODY_CL );

				//write body with current linear/angluar velocities to GPU
				m_internalData->m_bodyInfoBufferGPU->write(m_internalData->m_bodyInfoBufferCPU->m_ptr,numOfConvexRBodies);
				adl::DeviceUtils::waitForCompletion(m_internalData->m_ddcl);


				if (nContactOut)
				{
					reorderConvertToConstraints2( 
						solverData, 
						m_internalData->m_pBufRBodiesCPU, 
						m_internalData->m_bodyInfoBufferCPU, 
						m_internalData->m_pBufContactOutCPU,
						contactCPU, 
						frictionCPU, 
						nContactOut, 
						csCfg );

					bool forceGPU = true;

					if (forceGPU)
					{

						SolverData contactCPUcopy = adl::Solver<adl::TYPE_HOST>::allocateConstraint4( 
							m_internalData->m_deviceHost, 
							numOfConvexRBodies*MAX_PAIRS_PER_BODY_CL );

							adl::Solver<adl::TYPE_CL>::reorderConvertToConstraints( 
						m_internalData->m_solverDataGPU, 
						m_internalData->m_pBufRBodiesGPU, 
						m_internalData->m_bodyInfoBufferGPU, 
						m_internalData->m_pBufContactOutGPU,
						m_internalData->m_contactCGPU, 
						m_internalData->m_frictionCGPU, 
						nContactOut, 
						csCfg );

						adl::DeviceUtils::waitForCompletion(m_internalData->m_ddcl);
						m_internalData->m_contactCGPU->read(contactCPUcopy->m_ptr,nContactOut);
						adl::DeviceUtils::waitForCompletion(m_internalData->m_ddcl);

						
						//m_internalData->m_contactCGPU->write(contactCPU->m_ptr,nContactOut);
						adl::DeviceUtils::waitForCompletion(m_internalData->m_ddcl);
						m_internalData->m_solverDataGPU->m_nIterations = 4;
					
						adl::Solver<adl::TYPE_CL>::solveContactConstraint( m_internalData->m_solverDataGPU, 
							m_internalData->m_pBufRBodiesGPU, 
							m_internalData->m_bodyInfoBufferGPU, 
							m_internalData->m_contactCGPU,
							0, 
							nContactOut );

							adl::DeviceUtils::waitForCompletion( m_internalData->m_ddcl );

						//read body updated linear/angular velocities back to CPU
						m_internalData->m_pBufRBodiesGPU->read(
							m_internalData->m_pBufRBodiesCPU->m_ptr,numOfConvexRBodies);
							adl::DeviceUtils::waitForCompletion( m_internalData->m_ddcl );

					} else
					{
					solverData->m_nIterations = 4;
					adl::Solver<adl::TYPE_HOST>::solveContactConstraint( solverData, 
						m_internalData->m_pBufRBodiesCPU, 
						m_internalData->m_bodyInfoBufferCPU, 
						contactCPU,
						0, 
						nContactOut );
					}



					}

				adl::Solver<adl::TYPE_HOST>::deallocateConstraint4( contactCPU );
				adl::Solver<adl::TYPE_HOST>::deallocateFrictionConstraint( frictionCPU );
				adl::Solver<adl::TYPE_HOST>::deallocate( solverData );

				

			}
			else
			{
				
				{
					BT_PROFILE("rigid body data to GPU buffer");
					// Transfer rigid body data from CPU buffer to GPU buffer
					m_internalData->m_pBufRBodiesGPU->write(m_internalData->m_pBufRBodiesCPU->m_ptr, numOfConvexRBodies);
					m_internalData->m_pBufPairsGPU->write(m_internalData->m_pBufPairsCPU->m_ptr, MAX_BROADPHASE_COLLISION_CL);
					//write body with current linear/angluar velocities to GPU
					m_internalData->m_bodyInfoBufferGPU->write(m_internalData->m_bodyInfoBufferCPU->m_ptr,numOfConvexRBodies);
					adl::DeviceUtils::waitForCompletion(m_internalData->m_ddcl);
				}
				{
					BT_PROFILE("GPU reorderConvertToConstraints");
					adl::Solver<adl::TYPE_CL>::reorderConvertToConstraints( 
						m_internalData->m_solverDataGPU, 
						m_internalData->m_pBufRBodiesGPU, 
						m_internalData->m_bodyInfoBufferGPU, 
						m_internalData->m_pBufContactOutGPU,
						m_internalData->m_contactCGPU, 
						m_internalData->m_frictionCGPU, 
						nContactOut, 
						csCfg );
				}

				{
					BT_PROFILE("GPU solveContactConstraint");
				m_internalData->m_solverDataGPU->m_nIterations = 4;
					
					adl::Solver<adl::TYPE_CL>::solveContactConstraint( m_internalData->m_solverDataGPU, 
						m_internalData->m_pBufRBodiesGPU, 
						m_internalData->m_bodyInfoBufferGPU, 
						m_internalData->m_contactCGPU,
						0, 
						nContactOut );
	
					adl::DeviceUtils::waitForCompletion( m_internalData->m_ddcl );
				}
				{
					BT_PROFILE("read body velocities back to CPU");
					//read body updated linear/angular velocities back to CPU
					m_internalData->m_pBufRBodiesGPU->read(
						m_internalData->m_pBufRBodiesCPU->m_ptr,numOfConvexRBodies);
						adl::DeviceUtils::waitForCompletion( m_internalData->m_ddcl );
				}

				
			}

#if 0
				if( !m_useGPUPipeline )
				{	//	CPU
						BT_PROFILE("CPU solve");
						{
							BT_PROFILE("CPU reorderConvertToConstraints");

					SOLVER_CLASS<TYPE_HOST>::reorderConvertToConstraints( solver, m_bodyBuffer, m_bodyInfoBufferCPU, (Buffer<Contact4>*)m_contactBuffer, 
						contactC, frictionC, m_numContacts, csCfg );
						}
						{
							BT_PROFILE("CPU solveContactConstraint");

					solver->m_nIterations = 4;
					SOLVER_CLASS<TYPE_HOST>::solveContactConstraint( solver, m_bodyBuffer, m_bodyInfoBufferCPU, contactC, 0, m_numContacts );
						}
				}
				else
				{
						BT_PROFILE("GPU solve");
					{	//	GPU using host buffers
						{
							BT_PROFILE("GPU reorderConvertToConstraints");

						Solver<TYPE_CL>::reorderConvertToConstraints( m_solver, m_bodyBuffer, m_bodyInfoBufferCPU, (Buffer<Contact4>*)m_contactBuffer, 
							contactC, frictionC, m_numContacts, csCfg );
						}
						timerEnd();

						timerStart(0);
						//for(int iter=0; iter<4; iter++)
						{
							BT_PROFILE("GPU solveContactConstraint");

							Solver<TYPE_CL>::solveContactConstraint( m_solver, m_bodyBuffer, m_bodyInfoBufferCPU, contactC, frictionC, m_numContacts );
						}
						DeviceUtils::waitForCompletion( m_device );
					}
				}
				timerEnd();
#endif


			}

			//if we ran the solver, it will overwrite the batchIdx so we cannot write back the results
			//try to make it work by writing velocity back to rigid body

			if (useSolver)
			{
				
				BT_PROFILE("writing velocity back to btRigidBody");

				for ( int i=0; i<overlappingPairArray.size(); i++)
				{
					btAssert(i<MAX_BROADPHASE_COLLISION_CL);

					btBroadphasePair* pair = &overlappingPairArray[i];

					btCollisionObject* colObj0 = (btCollisionObject*)pair->m_pProxy0->m_clientObject;
					btCollisionObject* colObj1 = (btCollisionObject*)pair->m_pProxy1->m_clientObject;

					int bodyIndex0 = colObj0->getCompanionId();
					int bodyIndex1 = colObj1->getCompanionId();

					RigidBodyBase::Body* bA = &m_internalData->m_pBufRBodiesCPU->m_ptr[bodyIndex0];
					RigidBodyBase::Body* bB = &m_internalData->m_pBufRBodiesCPU->m_ptr[bodyIndex1];
					btRigidBody* bodyA = btRigidBody::upcast(colObj0);
					if (bodyA && !bodyA->isStaticOrKinematicObject())
					{
						bodyA->setLinearVelocity(btVector3(
										bA->m_linVel.x,
										bA->m_linVel.y,
										bA->m_linVel.z));

						bodyA->setAngularVelocity(btVector3(
										bA->m_angVel.x,
										bA->m_angVel.y,
										bA->m_angVel.z));
					}
					btRigidBody* bodyB = btRigidBody::upcast(colObj1);
					if (bodyB && !bodyB->isStaticOrKinematicObject())
					{
						bodyB->setLinearVelocity(btVector3(
							bB->m_linVel.x,
							bB->m_linVel.y,
							bB->m_linVel.z));
						bodyB->setAngularVelocity(btVector3(
										bB->m_angVel.x,
										bB->m_angVel.y,
										bB->m_angVel.z));

					}




				}
			} else
			{
				BT_PROFILE("copy Contact4 to btPersistentManifold");
				// Now we got the narrowphase info from GPU and need to update rigid bodies with the info and go back to the original pipeline in Bullet physics. 
				for ( int i = 0; i < nContactOut; i++ )
				{
					Contact4 contact = (*m_internalData->m_pBufContactOutCPU)[i];

					int idxBodyA = contact.m_bodyAPtr;
					int idxBodyB = contact.m_bodyBPtr;

					btAssert(contact.m_batchIdx>=0);
					btAssert(contact.m_batchIdx<overlappingPairArray.size());

					btBroadphasePair* pair = &overlappingPairArray[contact.m_batchIdx];

					btCollisionObject* colObj0 = (btCollisionObject*)pair->m_pProxy0->m_clientObject;
					btCollisionObject* colObj1 = (btCollisionObject*)pair->m_pProxy1->m_clientObject;

					if (!pair->m_algorithm)
					{
						pair->m_algorithm = findAlgorithm(colObj0,colObj1,0);
					}

					btManifoldResult contactPointResult(colObj0, colObj1);


					CustomConvexConvexPairCollision* pairAlgo = (CustomConvexConvexPairCollision*) pair->m_algorithm;

					if (!pairAlgo->getManifoldPtr())
					{
						pairAlgo->createManifoldPtr(colObj0,colObj1,dispatchInfo);
					}
					
					contactPointResult.setPersistentManifold(pairAlgo->getManifoldPtr());
					
					contactPointResult.getPersistentManifold()->refreshContactPoints(colObj0->getWorldTransform(),colObj1->getWorldTransform());

					const btTransform& transA = colObj0->getWorldTransform();
					const btTransform& transB = colObj1->getWorldTransform();

					int numPoints = contact.getNPoints();

					for ( int k=0; k < numPoints; k++ )
					{
						btVector3 normalOnBInWorld(
							contact.m_worldNormal.x,
							contact.m_worldNormal.y,
							contact.m_worldNormal.z);
						btVector3 pointInWorldOnB(
							contact.m_worldPos[k].x,
							contact.m_worldPos[k].y,
							contact.m_worldPos[k].z);

						btScalar depth = contact.m_worldPos[k].w;

						if (depth<0)
						{
							const btVector3 deltaC = transB.getOrigin() - transA.getOrigin();

							normalOnBInWorld.normalize();

							if((deltaC.dot(normalOnBInWorld))>0.0f)
							{
								normalOnBInWorld= -normalOnBInWorld;

								contactPointResult.addContactPoint(normalOnBInWorld, pointInWorldOnB, depth);
							}
							else
							{
								contactPointResult.addContactPoint(normalOnBInWorld, pointInWorldOnB-normalOnBInWorld*depth, depth);
							}
						}
					}
				}
			}
		}
	}
#endif //CL_PLATFORM_AMD
	}

}

