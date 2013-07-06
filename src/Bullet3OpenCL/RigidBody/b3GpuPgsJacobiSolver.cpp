#include "b3GpuPgsJacobiSolver.h"

#include "Bullet3Collision/NarrowPhaseCollision/b3RigidBodyCL.h"

#include "Bullet3Dynamics/ConstraintSolver/b3TypedConstraint.h"
#include <new>
#include "Bullet3Common/b3AlignedObjectArray.h"
#include <string.h> //for memset
#include "Bullet3Collision/NarrowPhaseCollision/b3Contact4.h"
#include "Bullet3OpenCL/ParallelPrimitives/b3OpenCLArray.h"
#include "Bullet3OpenCL/ParallelPrimitives/b3LauncherCL.h"



#include "Bullet3OpenCL/RigidBody/kernels/jointSolver.h" //solveConstraintRowsCL
#include "Bullet3OpenCL/Initialize/b3OpenCLUtils.h"

#define B3_JOINT_SOLVER_PATH "src/Bullet3OpenCL/RigidBody/kernels/jointSolver.cl"


struct b3GpuPgsJacobiSolverInternalData
{

	cl_context m_context;
	cl_device_id m_device;
	cl_command_queue m_queue;
	cl_kernel m_solveJointConstraintRowsKernels;


};
b3GpuPgsJacobiSolver::b3GpuPgsJacobiSolver (cl_context ctx, cl_device_id device, cl_command_queue queue,bool usePgs)
	:b3PgsJacobiSolver (usePgs)
{
	m_gpuData = new b3GpuPgsJacobiSolverInternalData();
	m_gpuData->m_context = ctx;
	m_gpuData->m_device = device;
	m_gpuData->m_queue = queue;

	cl_int errNum=0;

	{
		cl_program prog = b3OpenCLUtils::compileCLProgramFromString(m_gpuData->m_context,m_gpuData->m_device,solveConstraintRowsCL,&errNum,"",B3_JOINT_SOLVER_PATH);
		b3Assert(errNum==CL_SUCCESS);
		m_gpuData->m_solveJointConstraintRowsKernels = b3OpenCLUtils::compileCLKernelFromString(m_gpuData->m_context, m_gpuData->m_device,solveConstraintRowsCL, "solveJointConstraintRows",&errNum,prog);
		b3Assert(errNum==CL_SUCCESS);
		clReleaseProgram(prog);
	}


}

b3GpuPgsJacobiSolver::~b3GpuPgsJacobiSolver ()
{
	clReleaseKernel(m_gpuData->m_solveJointConstraintRowsKernels);

	delete m_gpuData;
}

struct b3BatchConstraint
{
	int m_bodyAPtrAndSignBit;
	int m_bodyBPtrAndSignBit;
	int	m_constraintRowOffset;
	short int	m_numConstraintRows;
	short int m_batchId;

	short int& getBatchIdx()
	{
		return m_batchId;
	}
};

static b3AlignedObjectArray<b3BatchConstraint> batchConstraints;
static b3AlignedObjectArray<int> batches;






b3Scalar b3GpuPgsJacobiSolver::solveGroupCacheFriendlySetup(b3RigidBodyCL* bodies, b3InertiaCL* inertias, int numBodies, b3Contact4* manifoldPtr, int numManifolds,b3TypedConstraint** constraints,int numConstraints,const b3ContactSolverInfo& infoGlobal)
{
	B3_PROFILE("GPU solveGroupCacheFriendlySetup");
	batchConstraints.resize(numConstraints);
	m_staticIdx = -1;
	m_maxOverrideNumSolverIterations = 0;



	m_tmpSolverBodyPool.resize(0);
	
	
	m_bodyCount.resize(0);
	m_bodyCount.resize(numBodies,0);
	m_bodyCountCheck.resize(0);
	m_bodyCountCheck.resize(numBodies,0);
	
	m_deltaLinearVelocities.resize(0);
	m_deltaLinearVelocities.resize(numBodies,b3Vector3(0,0,0));
	m_deltaAngularVelocities.resize(0);
	m_deltaAngularVelocities.resize(numBodies,b3Vector3(0,0,0));
	
	int totalBodies = 0;

	for (int i=0;i<numConstraints;i++)
	{
		int bodyIndexA = constraints[i]->getRigidBodyA();
		int bodyIndexB = constraints[i]->getRigidBodyB();
		if (m_usePgs)
		{
			m_bodyCount[bodyIndexA]=-1;
			m_bodyCount[bodyIndexB]=-1;
		} else
		{
			//didn't implement joints with Jacobi version yet
			b3Assert(0);
		}

	}
	for (int i=0;i<numManifolds;i++)
	{
		int bodyIndexA = manifoldPtr[i].getBodyA();
		int bodyIndexB = manifoldPtr[i].getBodyB();
		if (m_usePgs)
		{
			m_bodyCount[bodyIndexA]=-1;
			m_bodyCount[bodyIndexB]=-1;
		} else
		{
			if (bodies[bodyIndexA].getInvMass())
			{
				//m_bodyCount[bodyIndexA]+=manifoldPtr[i].getNPoints();
				m_bodyCount[bodyIndexA]++;
			}
			else
				m_bodyCount[bodyIndexA]=-1;

			if (bodies[bodyIndexB].getInvMass())
			//	m_bodyCount[bodyIndexB]+=manifoldPtr[i].getNPoints();
				m_bodyCount[bodyIndexB]++;
			else
				m_bodyCount[bodyIndexB]=-1;
		}

	}


	
	if (1)
	{
		int j;
		for (j=0;j<numConstraints;j++)
		{
			b3TypedConstraint* constraint = constraints[j];
			
			constraint->internalSetAppliedImpulse(0.0f);
		}
	}

	//b3RigidBody* rb0=0,*rb1=0;
	//if (1)
	{
		{

			int totalNumRows = 0;
			int i;
			
			m_tmpConstraintSizesPool.resizeNoInitialize(numConstraints);
			//calculate the total number of contraint rows
			for (i=0;i<numConstraints;i++)
			{
				b3TypedConstraint::b3ConstraintInfo1& info1 = m_tmpConstraintSizesPool[i];

				b3JointFeedback* fb = constraints[i]->getJointFeedback();
				if (fb)
				{
					fb->m_appliedForceBodyA.setZero();
					fb->m_appliedTorqueBodyA.setZero();
					fb->m_appliedForceBodyB.setZero();
					fb->m_appliedTorqueBodyB.setZero();
				}

				if (constraints[i]->isEnabled())
				{
				}
				if (constraints[i]->isEnabled())
				{
					constraints[i]->getInfo1(&info1,bodies);
				} else
				{
					info1.m_numConstraintRows = 0;
					info1.nub = 0;
				}
				
				batchConstraints[i].m_numConstraintRows = info1.m_numConstraintRows;
				batchConstraints[i].m_constraintRowOffset = totalNumRows;
				totalNumRows += info1.m_numConstraintRows;
			}
			m_tmpSolverNonContactConstraintPool.resizeNoInitialize(totalNumRows);

			
#ifndef DISABLE_JOINTS
			///setup the b3SolverConstraints
			int currentRow = 0;

			for (i=0;i<numConstraints;i++)
			{
				const b3TypedConstraint::b3ConstraintInfo1& info1 = m_tmpConstraintSizesPool[i];
				
				if (info1.m_numConstraintRows)
				{
					b3Assert(currentRow<totalNumRows);

					b3SolverConstraint* currentConstraintRow = &m_tmpSolverNonContactConstraintPool[currentRow];
					b3TypedConstraint* constraint = constraints[i];

					b3RigidBodyCL& rbA = bodies[ constraint->getRigidBodyA()];
					//b3RigidBody& rbA = constraint->getRigidBodyA();
	//				b3RigidBody& rbB = constraint->getRigidBodyB();
					b3RigidBodyCL& rbB = bodies[ constraint->getRigidBodyB()];
					
					

                    int solverBodyIdA = getOrInitSolverBody(constraint->getRigidBodyA(),bodies,inertias);
                    int solverBodyIdB = getOrInitSolverBody(constraint->getRigidBodyB(),bodies,inertias);

                    b3SolverBody* bodyAPtr = &m_tmpSolverBodyPool[solverBodyIdA];
                    b3SolverBody* bodyBPtr = &m_tmpSolverBodyPool[solverBodyIdB];

					if (rbA.getInvMass())
					{
						batchConstraints[i].m_bodyAPtrAndSignBit = solverBodyIdA;
					} else
					{
						if (!solverBodyIdA)
							m_staticIdx = 0;
						batchConstraints[i].m_bodyAPtrAndSignBit = -solverBodyIdA;
					}

					if (rbB.getInvMass())
					{
						batchConstraints[i].m_bodyBPtrAndSignBit = solverBodyIdB;
					} else
					{
						if (!solverBodyIdB)
							m_staticIdx = 0;
						batchConstraints[i].m_bodyBPtrAndSignBit = -solverBodyIdB;
					}


					int overrideNumSolverIterations = constraint->getOverrideNumSolverIterations() > 0 ? constraint->getOverrideNumSolverIterations() : infoGlobal.m_numIterations;
					if (overrideNumSolverIterations>m_maxOverrideNumSolverIterations)
						m_maxOverrideNumSolverIterations = overrideNumSolverIterations;


					int j;
					for ( j=0;j<info1.m_numConstraintRows;j++)
					{
						memset(&currentConstraintRow[j],0,sizeof(b3SolverConstraint));
						currentConstraintRow[j].m_lowerLimit = -B3_INFINITY;
						currentConstraintRow[j].m_upperLimit = B3_INFINITY;
						currentConstraintRow[j].m_appliedImpulse = 0.f;
						currentConstraintRow[j].m_appliedPushImpulse = 0.f;
						currentConstraintRow[j].m_solverBodyIdA = solverBodyIdA;
						currentConstraintRow[j].m_solverBodyIdB = solverBodyIdB;
						currentConstraintRow[j].m_overrideNumSolverIterations = overrideNumSolverIterations;
					}

					bodyAPtr->internalGetDeltaLinearVelocity().setValue(0.f,0.f,0.f);
					bodyAPtr->internalGetDeltaAngularVelocity().setValue(0.f,0.f,0.f);
					bodyAPtr->internalGetPushVelocity().setValue(0.f,0.f,0.f);
					bodyAPtr->internalGetTurnVelocity().setValue(0.f,0.f,0.f);
					bodyBPtr->internalGetDeltaLinearVelocity().setValue(0.f,0.f,0.f);
					bodyBPtr->internalGetDeltaAngularVelocity().setValue(0.f,0.f,0.f);
					bodyBPtr->internalGetPushVelocity().setValue(0.f,0.f,0.f);
					bodyBPtr->internalGetTurnVelocity().setValue(0.f,0.f,0.f);


					b3TypedConstraint::b3ConstraintInfo2 info2;
					info2.fps = 1.f/infoGlobal.m_timeStep;
					info2.erp = infoGlobal.m_erp;
					info2.m_J1linearAxis = currentConstraintRow->m_contactNormal;
					info2.m_J1angularAxis = currentConstraintRow->m_relpos1CrossNormal;
					info2.m_J2linearAxis = 0;
					info2.m_J2angularAxis = currentConstraintRow->m_relpos2CrossNormal;
					info2.rowskip = sizeof(b3SolverConstraint)/sizeof(b3Scalar);//check this
					///the size of b3SolverConstraint needs be a multiple of b3Scalar
		            b3Assert(info2.rowskip*sizeof(b3Scalar)== sizeof(b3SolverConstraint));
					info2.m_constraintError = &currentConstraintRow->m_rhs;
					currentConstraintRow->m_cfm = infoGlobal.m_globalCfm;
					info2.m_damping = infoGlobal.m_damping;
					info2.cfm = &currentConstraintRow->m_cfm;
					info2.m_lowerLimit = &currentConstraintRow->m_lowerLimit;
					info2.m_upperLimit = &currentConstraintRow->m_upperLimit;
					info2.m_numIterations = infoGlobal.m_numIterations;
					constraints[i]->getInfo2(&info2,bodies);

					///finalize the constraint setup
					for ( j=0;j<info1.m_numConstraintRows;j++)
					{
						b3SolverConstraint& solverConstraint = currentConstraintRow[j];

						if (solverConstraint.m_upperLimit>=constraints[i]->getBreakingImpulseThreshold())
						{
							solverConstraint.m_upperLimit = constraints[i]->getBreakingImpulseThreshold();
						}

						if (solverConstraint.m_lowerLimit<=-constraints[i]->getBreakingImpulseThreshold())
						{
							solverConstraint.m_lowerLimit = -constraints[i]->getBreakingImpulseThreshold();
						}

						solverConstraint.m_originalContactPoint = constraint;
							
						b3Matrix3x3& invInertiaWorldA= inertias[constraint->getRigidBodyA()].m_invInertiaWorld;
						{

							//b3Vector3 angularFactorA(1,1,1);
							const b3Vector3& ftorqueAxis1 = solverConstraint.m_relpos1CrossNormal;
							solverConstraint.m_angularComponentA = invInertiaWorldA*ftorqueAxis1;//*angularFactorA;
						}
						
						b3Matrix3x3& invInertiaWorldB= inertias[constraint->getRigidBodyB()].m_invInertiaWorld;
						{

							const b3Vector3& ftorqueAxis2 = solverConstraint.m_relpos2CrossNormal;
							solverConstraint.m_angularComponentB = invInertiaWorldB*ftorqueAxis2;//*constraint->getRigidBodyB().getAngularFactor();
						}

						{
							//it is ok to use solverConstraint.m_contactNormal instead of -solverConstraint.m_contactNormal
							//because it gets multiplied iMJlB
							b3Vector3 iMJlA = solverConstraint.m_contactNormal*rbA.getInvMass();
							b3Vector3 iMJaA = invInertiaWorldA*solverConstraint.m_relpos1CrossNormal;
							b3Vector3 iMJlB = solverConstraint.m_contactNormal*rbB.getInvMass();//sign of normal?
							b3Vector3 iMJaB = invInertiaWorldB*solverConstraint.m_relpos2CrossNormal;

							b3Scalar sum = iMJlA.dot(solverConstraint.m_contactNormal);
							sum += iMJaA.dot(solverConstraint.m_relpos1CrossNormal);
							sum += iMJlB.dot(solverConstraint.m_contactNormal);
							sum += iMJaB.dot(solverConstraint.m_relpos2CrossNormal);
							b3Scalar fsum = b3Fabs(sum);
							b3Assert(fsum > B3_EPSILON);
							solverConstraint.m_jacDiagABInv = fsum>B3_EPSILON?b3Scalar(1.)/sum : 0.f;
						}


						///fix rhs
						///todo: add force/torque accelerators
						{
							b3Scalar rel_vel;
							b3Scalar vel1Dotn = solverConstraint.m_contactNormal.dot(rbA.m_linVel) + solverConstraint.m_relpos1CrossNormal.dot(rbA.m_angVel);
							b3Scalar vel2Dotn = -solverConstraint.m_contactNormal.dot(rbB.m_linVel) + solverConstraint.m_relpos2CrossNormal.dot(rbB.m_angVel);

							rel_vel = vel1Dotn+vel2Dotn;

							b3Scalar restitution = 0.f;
							b3Scalar positionalError = solverConstraint.m_rhs;//already filled in by getConstraintInfo2
							b3Scalar	velocityError = restitution - rel_vel * info2.m_damping;
							b3Scalar	penetrationImpulse = positionalError*solverConstraint.m_jacDiagABInv;
							b3Scalar	velocityImpulse = velocityError *solverConstraint.m_jacDiagABInv;
							solverConstraint.m_rhs = penetrationImpulse+velocityImpulse;
							solverConstraint.m_appliedImpulse = 0.f;

						}
					}
				}
				currentRow+=m_tmpConstraintSizesPool[i].m_numConstraintRows;
			}
#endif //DISABLE_JOINTS
		}


		{
			int i;

			for (i=0;i<numManifolds;i++)
			{
				b3Contact4& manifold = manifoldPtr[i];
				convertContact(bodies,inertias,&manifold,infoGlobal);
			}
		}
	}

//	b3ContactSolverInfo info = infoGlobal;


	int numNonContactPool = m_tmpSolverNonContactConstraintPool.size();
	int numConstraintPool = m_tmpSolverContactConstraintPool.size();
	int numFrictionPool = m_tmpSolverContactFrictionConstraintPool.size();

	///@todo: use stack allocator for such temporarily memory, same for solver bodies/constraints
	m_orderNonContactConstraintPool.resizeNoInitialize(numNonContactPool);
	if ((infoGlobal.m_solverMode & B3_SOLVER_USE_2_FRICTION_DIRECTIONS))
		m_orderTmpConstraintPool.resizeNoInitialize(numConstraintPool*2);
	else
		m_orderTmpConstraintPool.resizeNoInitialize(numConstraintPool);

	m_orderFrictionConstraintPool.resizeNoInitialize(numFrictionPool);
	{
		int i;
		for (i=0;i<numNonContactPool;i++)
		{
			m_orderNonContactConstraintPool[i] = i;
		}
		for (i=0;i<numConstraintPool;i++)
		{
			m_orderTmpConstraintPool[i] = i;
		}
		for (i=0;i<numFrictionPool;i++)
		{
			m_orderFrictionConstraintPool[i] = i;
		}
	}

	return 0.f;

}








b3Scalar b3GpuPgsJacobiSolver::solveGroupCacheFriendlyIterations(b3TypedConstraint** cpuConstraints,int numConstraints,const b3ContactSolverInfo& infoGlobal)
{
	bool useCpu = false;
	bool createBatches = batches.size()==0;
	if (useCpu)
	{
		return b3PgsJacobiSolver::solveGroupCacheFriendlyIterations(cpuConstraints,numConstraints,infoGlobal);
	} else
	{
		B3_PROFILE("GpuSolveGroupCacheFriendlyIterations");
		if (createBatches)
		{
			
			batches.resize(0);

			{
				B3_PROFILE("batch joints");
				b3Assert(batchConstraints.size()==numConstraints);
				int simdWidth =numConstraints;
				int numBodies = m_tmpSolverBodyPool.size();
				sortConstraintByBatch3( &batchConstraints[0], numConstraints, simdWidth , m_staticIdx,  numBodies);
			}
		}
		int maxIterations = infoGlobal.m_numIterations;
		bool useBatching = true;
		if (useBatching )
		{
			b3OpenCLArray<b3SolverConstraint> gpuSolverConstraintRows(m_gpuData->m_context,m_gpuData->m_queue);
			gpuSolverConstraintRows.copyFromHost(m_tmpSolverNonContactConstraintPool);

			b3OpenCLArray<b3SolverBody> gpuSolverBodies(m_gpuData->m_context,m_gpuData->m_queue);
			gpuSolverBodies.copyFromHost(m_tmpSolverBodyPool);
//			gpuSolverBodies.copyToHost(m_tmpSolverBodyPool);
			
			b3OpenCLArray<b3BatchConstraint> gpuBatchConstraints(m_gpuData->m_context,m_gpuData->m_queue);
			gpuBatchConstraints.copyFromHost(batchConstraints);
			
			b3OpenCLArray<b3SolverConstraint> gpuConstraintRows(m_gpuData->m_context,m_gpuData->m_queue);
			gpuConstraintRows.copyFromHost(m_tmpSolverNonContactConstraintPool);
			

			for ( int iteration = 0 ; iteration< maxIterations ; iteration++)
			{
				
				int batchOffset = 0;
				int constraintOffset=0;
				int numBatches = batches.size();
				for (int bb=0;bb<numBatches;bb++)
				{
					int numConstraintsInBatch = batches[bb];

					bool useGpu=false;
					if (useGpu)
					{
						b3LauncherCL launcher(m_gpuData->m_queue,m_gpuData->m_solveJointConstraintRowsKernels);
						launcher.setBuffer(gpuSolverBodies.getBufferCL());
						launcher.setBuffer(gpuBatchConstraints.getBufferCL());
						launcher.setBuffer(gpuConstraintRows.getBufferCL());
						launcher.setConst(batchOffset);
						launcher.setConst(constraintOffset);
						launcher.setConst(numConstraintsInBatch);

						launcher.launch1D(numConstraintsInBatch);

					} else
					{
						
						for (int b=0;b<numConstraintsInBatch;b++)
						{
							const b3BatchConstraint& c = batchConstraints[batchOffset+b];
							b3Assert(c.m_batchId==bb);
						


							//can be done in parallel...
							for (int jj=0;jj<c.m_numConstraintRows;jj++)
							{
//							
								b3SolverConstraint& constraint = m_tmpSolverNonContactConstraintPool[c.m_constraintRowOffset+jj];
//								resolveSingleConstraintRowGenericSIMD(m_tmpSolverBodyPool[constraint.m_solverBodyIdA],m_tmpSolverBodyPool[constraint.m_solverBodyIdB],constraint);
								resolveSingleConstraintRowGeneric(m_tmpSolverBodyPool[constraint.m_solverBodyIdA],m_tmpSolverBodyPool[constraint.m_solverBodyIdB],constraint);

							}
						}
					}
					batchOffset+=numConstraintsInBatch;
					constraintOffset+=numConstraintsInBatch;
				}
			}//for (int iteration...

			gpuSolverBodies.copyToHost(m_tmpSolverBodyPool);
			clFinish(m_gpuData->m_queue);
			printf(",,\n");


		} else
		{
			for ( int iteration = 0 ; iteration< maxIterations ; iteration++)
			{			
				int numJoints =			m_tmpSolverNonContactConstraintPool.size();
				for (int j=0;j<numJoints;j++)
				{
					b3SolverConstraint& constraint = m_tmpSolverNonContactConstraintPool[j];
					resolveSingleConstraintRowGenericSIMD(m_tmpSolverBodyPool[constraint.m_solverBodyIdA],m_tmpSolverBodyPool[constraint.m_solverBodyIdB],constraint);
				}

				if (!m_usePgs)
				{
					averageVelocities();
				}
			}
		}
		
	}
	return 0.f;
}




static b3AlignedObjectArray<int> bodyUsed;
static b3AlignedObjectArray<int> curUsed;



inline int b3GpuPgsJacobiSolver::sortConstraintByBatch3( b3BatchConstraint* cs, int numConstraints, int simdWidth , int staticIdx, int numBodies)
{
	int sz = sizeof(b3BatchConstraint);

	B3_PROFILE("sortConstraintByBatch3");
	
	static int maxSwaps = 0;
	int numSwaps = 0;

	curUsed.resize(2*simdWidth);

	static int maxNumConstraints = 0;
	if (maxNumConstraints<numConstraints)
	{
		maxNumConstraints = numConstraints;
		//printf("maxNumConstraints  = %d\n",maxNumConstraints );
	}

	int numUsedArray = numBodies/32+1;
	bodyUsed.resize(numUsedArray);

	for (int q=0;q<numUsedArray;q++)
		bodyUsed[q]=0;

	
	int curBodyUsed = 0;

	int numIter = 0;
    
		
#if defined(_DEBUG)
	for(int i=0; i<numConstraints; i++)
		cs[i].getBatchIdx() = -1;
#endif
	
	int numValidConstraints = 0;
	int unprocessedConstraintIndex = 0;

	int batchIdx = 0;
    

	{
		B3_PROFILE("cpu batch innerloop");
		
		while( numValidConstraints < numConstraints)
		{
			numIter++;
			int nCurrentBatch = 0;
			//	clear flag
			for(int i=0; i<curBodyUsed; i++) 
				bodyUsed[curUsed[i]/32] = 0;

            curBodyUsed = 0;

			for(int i=numValidConstraints; i<numConstraints; i++)
			{
				int idx = i;
				b3Assert( idx < numConstraints );
				//	check if it can go
				int bodyAS = cs[idx].m_bodyAPtrAndSignBit;
				int bodyBS = cs[idx].m_bodyBPtrAndSignBit;
				int bodyA = abs(bodyAS);
				int bodyB = abs(bodyBS);
				bool aIsStatic = (bodyAS<0) || bodyAS==staticIdx;
				bool bIsStatic = (bodyBS<0) || bodyBS==staticIdx;
				int aUnavailable = 0;
				int bUnavailable = 0;
				if (!aIsStatic)
				{
					aUnavailable = bodyUsed[ bodyA/32 ] & (1<<(bodyA&31));
				}
				if (!aUnavailable)
				if (!bIsStatic)
				{
					bUnavailable = bodyUsed[ bodyB/32 ] & (1<<(bodyB&31));
				}
                
				if( aUnavailable==0 && bUnavailable==0 ) // ok
				{
					if (!aIsStatic)
					{
						bodyUsed[ bodyA/32 ] |= (1<<(bodyA&31));
						curUsed[curBodyUsed++]=bodyA;
					}
					if (!bIsStatic)
					{
						bodyUsed[ bodyB/32 ] |= (1<<(bodyB&31));
						curUsed[curBodyUsed++]=bodyB;
					}

					cs[idx].getBatchIdx() = batchIdx;

					if (i!=numValidConstraints)
					{
						b3Swap(cs[i],cs[numValidConstraints]);
						numSwaps++;
					}

					numValidConstraints++;
					{
						nCurrentBatch++;
						if( nCurrentBatch == simdWidth )
						{
							nCurrentBatch = 0;
							for(int i=0; i<curBodyUsed; i++) 
								bodyUsed[curUsed[i]/32] = 0;
							curBodyUsed = 0;
						}
					}
				}
			}
			batches.push_back(nCurrentBatch);
			batchIdx ++;
		}
	}
	
#if defined(_DEBUG)
    //		debugPrintf( "nBatches: %d\n", batchIdx );
	for(int i=0; i<numConstraints; i++)
    {
        b3Assert( cs[i].getBatchIdx() != -1 );
    }
#endif

	if (maxSwaps<numSwaps)
	{
		maxSwaps = numSwaps;
		//printf("maxSwaps = %d\n", maxSwaps);
	}
	
	return batchIdx;
}