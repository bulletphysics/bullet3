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
//Originally written by Takahiro Harada


#include "b3Solver.h"

///useNewBatchingKernel  is a rewritten kernel using just a single thread of the warp, for experiments
bool useNewBatchingKernel = true;

#define B3_SOLVER_SETUP_KERNEL_PATH "src/Bullet3OpenCL/RigidBody/kernels/solverSetup.cl"
#define B3_SOLVER_SETUP2_KERNEL_PATH "src/Bullet3OpenCL/RigidBody/kernels/solverSetup2.cl"
#define B3_SOLVER_CONTACT_KERNEL_PATH "src/Bullet3OpenCL/RigidBody/kernels/solveContact.cl"
#define B3_SOLVER_FRICTION_KERNEL_PATH "src/Bullet3OpenCL/RigidBody/kernels/solveFriction.cl"
#define B3_BATCHING_PATH "src/Bullet3OpenCL/RigidBody/kernels/batchingKernels.cl"
#define B3_BATCHING_NEW_PATH "src/Bullet3OpenCL/RigidBody/kernels/batchingKernelsNew.cl"


#include "kernels/solverSetup.h"
#include "kernels/solverSetup2.h"

#include "kernels/solveContact.h"
#include "kernels/solveFriction.h"

#include "kernels/batchingKernels.h"
#include "kernels/batchingKernelsNew.h"


#include "Bullet3Common/b3Quickprof.h"
#include "Bullet3OpenCL/ParallelPrimitives/b3LauncherCL.h"
#include "Bullet3Common/b3Vector3.h"

struct SolverDebugInfo
{
	int m_valInt0;
	int m_valInt1;
	int m_valInt2;
	int m_valInt3;
	
	int m_valInt4;
	int m_valInt5;
	int m_valInt6;
	int m_valInt7;

	int m_valInt8;
	int m_valInt9;
	int m_valInt10;
	int m_valInt11;

	int	m_valInt12;
	int	m_valInt13;
	int	m_valInt14;
	int	m_valInt15;


	float m_val0;
	float m_val1;
	float m_val2;
	float m_val3;
};




class SolverDeviceInl
{
public:
	struct ParallelSolveData
	{
		b3OpenCLArray<unsigned int>* m_numConstraints;
		b3OpenCLArray<unsigned int>* m_offsets;
	};
};



b3Solver::b3Solver(cl_context ctx, cl_device_id device, cl_command_queue queue, int pairCapacity)
			:m_nIterations(4),
			m_context(ctx),
			m_device(device),
			m_queue(queue)
{
	m_sort32 = new b3RadixSort32CL(ctx,device,queue);
	m_scan = new b3PrefixScanCL(ctx,device,queue,N_SPLIT*N_SPLIT);
	m_search = new b3BoundSearchCL(ctx,device,queue,N_SPLIT*N_SPLIT);

	const int sortSize = B3NEXTMULTIPLEOF( pairCapacity, 512 );

	m_sortDataBuffer = new b3OpenCLArray<b3SortData>(ctx,queue,sortSize);
	m_contactBuffer2 = new b3OpenCLArray<b3Contact4>(ctx,queue);

	m_numConstraints = new b3OpenCLArray<unsigned int>(ctx,queue,N_SPLIT*N_SPLIT );
	m_numConstraints->resize(N_SPLIT*N_SPLIT);

	m_offsets = new b3OpenCLArray<unsigned int>( ctx,queue, N_SPLIT*N_SPLIT );
	m_offsets->resize(N_SPLIT*N_SPLIT);
	const char* additionalMacros = "";
	const char* srcFileNameForCaching="";



	cl_int pErrNum;
	const char* batchKernelSource = batchingKernelsCL;
	const char* batchKernelNewSource = batchingKernelsNewCL;
	
	const char* solverSetupSource = solverSetupCL;
	const char* solverSetup2Source = solverSetup2CL;
	const char* solveContactSource = solveContactCL;
	const char* solveFrictionSource = solveFrictionCL;
	
	
	
	{
		
		cl_program solveContactProg= b3OpenCLUtils::compileCLProgramFromString( ctx, device, solveContactSource, &pErrNum,additionalMacros, B3_SOLVER_CONTACT_KERNEL_PATH);
		b3Assert(solveContactProg);
		
		cl_program solveFrictionProg= b3OpenCLUtils::compileCLProgramFromString( ctx, device, solveFrictionSource, &pErrNum,additionalMacros, B3_SOLVER_FRICTION_KERNEL_PATH);
		b3Assert(solveFrictionProg);

		cl_program solverSetup2Prog= b3OpenCLUtils::compileCLProgramFromString( ctx, device, solverSetup2Source, &pErrNum,additionalMacros, B3_SOLVER_SETUP2_KERNEL_PATH);
		b3Assert(solverSetup2Prog);

		
		cl_program solverSetupProg= b3OpenCLUtils::compileCLProgramFromString( ctx, device, solverSetupSource, &pErrNum,additionalMacros, B3_SOLVER_SETUP_KERNEL_PATH);
		b3Assert(solverSetupProg);
		
		
		m_solveFrictionKernel= b3OpenCLUtils::compileCLKernelFromString( ctx, device, solveFrictionSource, "BatchSolveKernelFriction", &pErrNum, solveFrictionProg,additionalMacros );
		b3Assert(m_solveFrictionKernel);

		m_solveContactKernel= b3OpenCLUtils::compileCLKernelFromString( ctx, device, solveContactSource, "BatchSolveKernelContact", &pErrNum, solveContactProg,additionalMacros );
		b3Assert(m_solveContactKernel);
		
		m_contactToConstraintKernel = b3OpenCLUtils::compileCLKernelFromString( ctx, device, solverSetupSource, "ContactToConstraintKernel", &pErrNum, solverSetupProg,additionalMacros );
		b3Assert(m_contactToConstraintKernel);
			
		m_setSortDataKernel =  b3OpenCLUtils::compileCLKernelFromString( ctx, device, solverSetup2Source, "SetSortDataKernel", &pErrNum, solverSetup2Prog,additionalMacros );
		b3Assert(m_setSortDataKernel);
				
		m_reorderContactKernel = b3OpenCLUtils::compileCLKernelFromString( ctx, device, solverSetup2Source, "ReorderContactKernel", &pErrNum, solverSetup2Prog,additionalMacros );
		b3Assert(m_reorderContactKernel);
		

		m_copyConstraintKernel = b3OpenCLUtils::compileCLKernelFromString( ctx, device, solverSetup2Source, "CopyConstraintKernel", &pErrNum, solverSetup2Prog,additionalMacros );
		b3Assert(m_copyConstraintKernel);
		
	}

	{
		cl_program batchingProg = b3OpenCLUtils::compileCLProgramFromString( ctx, device, batchKernelSource, &pErrNum,additionalMacros, B3_BATCHING_PATH);
		b3Assert(batchingProg);
		
		m_batchingKernel = b3OpenCLUtils::compileCLKernelFromString( ctx, device, batchKernelSource, "CreateBatches", &pErrNum, batchingProg,additionalMacros );
		b3Assert(m_batchingKernel);
	}
	{
		cl_program batchingNewProg = b3OpenCLUtils::compileCLProgramFromString( ctx, device, batchKernelNewSource, &pErrNum,additionalMacros, B3_BATCHING_NEW_PATH);
		b3Assert(batchingNewProg);
	
		m_batchingKernelNew = b3OpenCLUtils::compileCLKernelFromString( ctx, device, batchKernelNewSource, "CreateBatchesNew", &pErrNum, batchingNewProg,additionalMacros );
		//m_batchingKernelNew = b3OpenCLUtils::compileCLKernelFromString( ctx, device, batchKernelNewSource, "CreateBatchesBruteForce", &pErrNum, batchingNewProg,additionalMacros );
		b3Assert(m_batchingKernelNew);
	}
}
		
b3Solver::~b3Solver()
{
	delete m_sortDataBuffer;
	delete m_contactBuffer2;

	delete m_sort32;
	delete m_scan;
	delete m_search;


	clReleaseKernel(m_batchingKernel);
	clReleaseKernel(m_batchingKernelNew);
	
	clReleaseKernel( m_solveContactKernel);
	clReleaseKernel( m_solveFrictionKernel);

	clReleaseKernel( m_contactToConstraintKernel);
	clReleaseKernel( m_setSortDataKernel);
	clReleaseKernel( m_reorderContactKernel);
	clReleaseKernel( m_copyConstraintKernel);
			
}


 


/*void b3Solver::reorderConvertToConstraints( const b3OpenCLArray<b3RigidBodyCL>* bodyBuf, 
	const b3OpenCLArray<b3InertiaCL>* shapeBuf,
	b3OpenCLArray<b3Contact4>* contactsIn, b3OpenCLArray<b3GpuConstraint4>* contactCOut, void* additionalData, 
	int nContacts, const b3Solver::ConstraintCfg& cfg )
{
	if( m_contactBuffer )
	{
		m_contactBuffer->resize(nContacts);
	}
	if( m_contactBuffer == 0 )
	{
		B3_PROFILE("new m_contactBuffer;");
		m_contactBuffer = new b3OpenCLArray<b3Contact4>(m_context,m_queue,nContacts );
		m_contactBuffer->resize(nContacts);
	}
	

	

	//DeviceUtils::Config dhCfg;
	//Device* deviceHost = DeviceUtils::allocate( TYPE_HOST, dhCfg );
	if( cfg.m_enableParallelSolve )
	{
		

		clFinish(m_queue);
		
		//	contactsIn -> m_contactBuffer
		{
			B3_PROFILE("sortContacts");
			sortContacts( bodyBuf, contactsIn, additionalData, nContacts, cfg );
			clFinish(m_queue);
		}
		
		
		{
			B3_PROFILE("m_copyConstraintKernel");

			

			b3Int4 cdata; cdata.x = nContacts;
			b3BufferInfoCL bInfo[] = { b3BufferInfoCL( m_contactBuffer->getBufferCL() ), b3BufferInfoCL( contactsIn->getBufferCL() ) };
//			b3LauncherCL launcher( m_queue, data->m_device->getKernel( PATH, "CopyConstraintKernel",  "-I ..\\..\\ -Wf,--c++", 0 ) );
			b3LauncherCL launcher( m_queue, m_copyConstraintKernel );
			launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(b3BufferInfoCL) );
			launcher.setConst(  cdata );
			launcher.launch1D( nContacts, 64 );
			clFinish(m_queue);
		}

		{
			B3_PROFILE("batchContacts");
			b3Solver::batchContacts( contactsIn, nContacts, m_numConstraints, m_offsets, cfg.m_staticIdx );

		}
	}
	{
			B3_PROFILE("waitForCompletion (batchContacts)");
			clFinish(m_queue);
	}
	
	//================
	
	{
		B3_PROFILE("convertToConstraints");
		b3Solver::convertToConstraints(  bodyBuf, shapeBuf, contactsIn, contactCOut, additionalData, nContacts, cfg );
	}

	{
		B3_PROFILE("convertToConstraints waitForCompletion");
		clFinish(m_queue);
	}
	
}
*/

	static
	inline
	float calcRelVel(const b3Vector3& l0, const b3Vector3& l1, const b3Vector3& a0, const b3Vector3& a1, 
					 const b3Vector3& linVel0, const b3Vector3& angVel0, const b3Vector3& linVel1, const b3Vector3& angVel1)
	{
		return b3Dot(l0, linVel0) + b3Dot(a0, angVel0) + b3Dot(l1, linVel1) + b3Dot(a1, angVel1);
	}


	static
	inline
	void setLinearAndAngular(const b3Vector3& n, const b3Vector3& r0, const b3Vector3& r1,
							 b3Vector3& linear, b3Vector3& angular0, b3Vector3& angular1)
	{
		linear = -n;
		angular0 = -b3Cross(r0, n);
		angular1 = b3Cross(r1, n);
	}


template<bool JACOBI>
static
__inline
void solveContact(b3GpuConstraint4& cs, 
	const b3Vector3& posA, b3Vector3& linVelA, b3Vector3& angVelA, float invMassA, const b3Matrix3x3& invInertiaA,
	const b3Vector3& posB, b3Vector3& linVelB, b3Vector3& angVelB, float invMassB, const b3Matrix3x3& invInertiaB, 
	float maxRambdaDt[4], float minRambdaDt[4])
{

	b3Vector3 dLinVelA; dLinVelA.setZero();
	b3Vector3 dAngVelA; dAngVelA.setZero();
	b3Vector3 dLinVelB; dLinVelB.setZero();
	b3Vector3 dAngVelB; dAngVelB.setZero();

	for(int ic=0; ic<4; ic++)
	{
		//	dont necessary because this makes change to 0
		if( cs.m_jacCoeffInv[ic] == 0.f ) continue;

		{
			b3Vector3 angular0, angular1, linear;
			b3Vector3 r0 = cs.m_worldPos[ic] - (b3Vector3&)posA;
			b3Vector3 r1 = cs.m_worldPos[ic] - (b3Vector3&)posB;
			setLinearAndAngular( (const b3Vector3 &)-cs.m_linear, (const b3Vector3 &)r0, (const b3Vector3 &)r1, linear, angular0, angular1 );

			float rambdaDt = calcRelVel((const b3Vector3 &)cs.m_linear,(const b3Vector3 &) -cs.m_linear, angular0, angular1,
				linVelA, angVelA, linVelB, angVelB ) + cs.m_b[ic];
			rambdaDt *= cs.m_jacCoeffInv[ic];

			{
				float prevSum = cs.m_appliedRambdaDt[ic];
				float updated = prevSum;
				updated += rambdaDt;
				updated = b3Max( updated, minRambdaDt[ic] );
				updated = b3Min( updated, maxRambdaDt[ic] );
				rambdaDt = updated - prevSum;
				cs.m_appliedRambdaDt[ic] = updated;
			}

			b3Vector3 linImp0 = invMassA*linear*rambdaDt;
			b3Vector3 linImp1 = invMassB*(-linear)*rambdaDt;
			b3Vector3 angImp0 = (invInertiaA* angular0)*rambdaDt;
			b3Vector3 angImp1 = (invInertiaB* angular1)*rambdaDt;
#ifdef _WIN32
            b3Assert(_finite(linImp0.getX()));
			b3Assert(_finite(linImp1.getX()));
#endif
			if( JACOBI )
			{
				dLinVelA += linImp0;
				dAngVelA += angImp0;
				dLinVelB += linImp1;
				dAngVelB += angImp1;
			}
			else
			{
				linVelA += linImp0;
				angVelA += angImp0;
				linVelB += linImp1;
				angVelB += angImp1;
			}
		}
	}

	if( JACOBI )
	{
		linVelA += dLinVelA;
		angVelA += dAngVelA;
		linVelB += dLinVelB;
		angVelB += dAngVelB;
	}

}





	static
	__inline
	void solveFriction(b3GpuConstraint4& cs, 
		const b3Vector3& posA, b3Vector3& linVelA, b3Vector3& angVelA, float invMassA, const b3Matrix3x3& invInertiaA,
		const b3Vector3& posB, b3Vector3& linVelB, b3Vector3& angVelB, float invMassB, const b3Matrix3x3& invInertiaB, 
		float maxRambdaDt[4], float minRambdaDt[4])
	{

		if( cs.m_fJacCoeffInv[0] == 0 && cs.m_fJacCoeffInv[0] == 0 ) return;
		const b3Vector3& center = (const b3Vector3&)cs.m_center;

		b3Vector3 n = -(const b3Vector3&)cs.m_linear;

		b3Vector3 tangent[2];
#if 1		
		b3PlaneSpace1 (n, tangent[0],tangent[1]);
#else
		b3Vector3 r = cs.m_worldPos[0]-center;
		tangent[0] = cross3( n, r );
		tangent[1] = cross3( tangent[0], n );
		tangent[0] = normalize3( tangent[0] );
		tangent[1] = normalize3( tangent[1] );
#endif

		b3Vector3 angular0, angular1, linear;
		b3Vector3 r0 = center - posA;
		b3Vector3 r1 = center - posB;
		for(int i=0; i<2; i++)
		{
			setLinearAndAngular( tangent[i], r0, r1, linear, angular0, angular1 );
			float rambdaDt = calcRelVel(linear, -linear, angular0, angular1,
				linVelA, angVelA, linVelB, angVelB );
			rambdaDt *= cs.m_fJacCoeffInv[i];

				{
					float prevSum = cs.m_fAppliedRambdaDt[i];
					float updated = prevSum;
					updated += rambdaDt;
					updated = b3Max( updated, minRambdaDt[i] );
					updated = b3Min( updated, maxRambdaDt[i] );
					rambdaDt = updated - prevSum;
					cs.m_fAppliedRambdaDt[i] = updated;
				}

			b3Vector3 linImp0 = invMassA*linear*rambdaDt;
			b3Vector3 linImp1 = invMassB*(-linear)*rambdaDt;
			b3Vector3 angImp0 = (invInertiaA* angular0)*rambdaDt;
			b3Vector3 angImp1 = (invInertiaB* angular1)*rambdaDt;
#ifdef _WIN32
			b3Assert(_finite(linImp0.getX()));
			b3Assert(_finite(linImp1.getX()));
#endif
			linVelA += linImp0;
			angVelA += angImp0;
			linVelB += linImp1;
			angVelB += angImp1;
		}

		{	//	angular damping for point constraint
			b3Vector3 ab = ( posB - posA ).normalized();
			b3Vector3 ac = ( center - posA ).normalized();
			if( b3Dot( ab, ac ) > 0.95f || (invMassA == 0.f || invMassB == 0.f))
			{
				float angNA = b3Dot( n, angVelA );
				float angNB = b3Dot( n, angVelB );

				angVelA -= (angNA*0.1f)*n;
				angVelB -= (angNB*0.1f)*n;
			}
		}

	}


struct SolveTask// : public ThreadPool::Task
{
	SolveTask(b3AlignedObjectArray<b3RigidBodyCL>& bodies,  b3AlignedObjectArray<b3InertiaCL>& shapes, b3AlignedObjectArray<b3GpuConstraint4>& constraints,
		int start, int nConstraints)
		: m_bodies( bodies ), m_shapes( shapes ), m_constraints( constraints ), m_start( start ), m_nConstraints( nConstraints ),
		m_solveFriction( true ){}

	unsigned short int getType(){ return 0; }

	void run(int tIdx)
	{
		

		for(int ic=0; ic<m_nConstraints; ic++)
		{
			int i = m_start + ic;

			float frictionCoeff = m_constraints[i].getFrictionCoeff();
			int aIdx = (int)m_constraints[i].m_bodyA;
			int bIdx = (int)m_constraints[i].m_bodyB;
			b3RigidBodyCL& bodyA = m_bodies[aIdx];
			b3RigidBodyCL& bodyB = m_bodies[bIdx];

			if( !m_solveFriction )
			{
				float maxRambdaDt[4] = {FLT_MAX,FLT_MAX,FLT_MAX,FLT_MAX};
				float minRambdaDt[4] = {0.f,0.f,0.f,0.f};

				solveContact<false>( m_constraints[i], (b3Vector3&)bodyA.m_pos, (b3Vector3&)bodyA.m_linVel, (b3Vector3&)bodyA.m_angVel, bodyA.m_invMass, (const b3Matrix3x3 &)m_shapes[aIdx].m_invInertiaWorld, 
					 (b3Vector3&)bodyB.m_pos, (b3Vector3&)bodyB.m_linVel, (b3Vector3&)bodyB.m_angVel, bodyB.m_invMass, (const b3Matrix3x3 &)m_shapes[bIdx].m_invInertiaWorld,
					maxRambdaDt, minRambdaDt );

			}
			else
			{
				float maxRambdaDt[4] = {FLT_MAX,FLT_MAX,FLT_MAX,FLT_MAX};
				float minRambdaDt[4] = {0.f,0.f,0.f,0.f};

				float sum = 0;
				for(int j=0; j<4; j++)
				{
					sum +=m_constraints[i].m_appliedRambdaDt[j];
				}
				frictionCoeff = 0.7f;
				for(int j=0; j<4; j++)
				{
					maxRambdaDt[j] = frictionCoeff*sum;
					minRambdaDt[j] = -maxRambdaDt[j];
				}

			solveFriction( m_constraints[i], (b3Vector3&)bodyA.m_pos, (b3Vector3&)bodyA.m_linVel, (b3Vector3&)bodyA.m_angVel, bodyA.m_invMass,(const b3Matrix3x3 &) m_shapes[aIdx].m_invInertiaWorld, 
					(b3Vector3&)bodyB.m_pos, (b3Vector3&)bodyB.m_linVel, (b3Vector3&)bodyB.m_angVel, bodyB.m_invMass,(const b3Matrix3x3 &) m_shapes[bIdx].m_invInertiaWorld,
					maxRambdaDt, minRambdaDt );
			
			}
		}

		
	}

	b3AlignedObjectArray<b3RigidBodyCL>& m_bodies;
	b3AlignedObjectArray<b3InertiaCL>& m_shapes;
	b3AlignedObjectArray<b3GpuConstraint4>& m_constraints;
	int m_start;
	int m_nConstraints;
	bool m_solveFriction;
};


void b3Solver::solveContactConstraintHost(  b3OpenCLArray<b3RigidBodyCL>* bodyBuf, b3OpenCLArray<b3InertiaCL>* shapeBuf, 
			b3OpenCLArray<b3GpuConstraint4>* constraint, void* additionalData, int n ,int maxNumBatches)
{

	b3AlignedObjectArray<b3RigidBodyCL> bodyNative;
	bodyBuf->copyToHost(bodyNative);
	b3AlignedObjectArray<b3InertiaCL> shapeNative;
	shapeBuf->copyToHost(shapeNative);
	b3AlignedObjectArray<b3GpuConstraint4> constraintNative;
	constraint->copyToHost(constraintNative);

	for(int iter=0; iter<m_nIterations; iter++)
	{
		SolveTask task( bodyNative, shapeNative, constraintNative, 0, n );
		task.m_solveFriction = false;
		task.run(0);
	}

	for(int iter=0; iter<m_nIterations; iter++)
	{
		SolveTask task( bodyNative, shapeNative, constraintNative, 0, n );
		task.m_solveFriction = true;
		task.run(0);
	}

	bodyBuf->copyFromHost(bodyNative);
	shapeBuf->copyFromHost(shapeNative);
	constraint->copyFromHost(constraintNative);

	
}

void checkConstraintBatch(const b3OpenCLArray<b3RigidBodyCL>* bodyBuf,
					const b3OpenCLArray<b3InertiaCL>* shapeBuf,
					b3OpenCLArray<b3GpuConstraint4>* constraint, 
					b3OpenCLArray<unsigned int>* m_numConstraints,
					b3OpenCLArray<unsigned int>* m_offsets,
					int batchId
					)
{
//						b3BufferInfoCL( m_numConstraints->getBufferCL() ), 
//						b3BufferInfoCL( m_offsets->getBufferCL() ) 
	
	const int nn = b3SolverBase::N_SPLIT*b3SolverBase::N_SPLIT;
	int numWorkItems = 64*nn/b3SolverBase::N_BATCHES;

	b3AlignedObjectArray<unsigned int> gN;
	m_numConstraints->copyToHost(gN);
	b3AlignedObjectArray<unsigned int> gOffsets;
	m_offsets->copyToHost(gOffsets);
	int nSplit = b3SolverBase::N_SPLIT;
	int bIdx = batchId;

	b3AlignedObjectArray<b3GpuConstraint4> cpuConstraints;
	constraint->copyToHost(cpuConstraints);

	printf("batch = %d\n", batchId);

	int numWorkgroups = nn/b3SolverBase::N_BATCHES;
	b3AlignedObjectArray<int> usedBodies;


	for (int wgIdx=0;wgIdx<numWorkgroups;wgIdx++)
	{
		printf("wgIdx = %d           ", wgIdx);
		int xIdx = (wgIdx/(nSplit/2))*2 + (bIdx&1);
		int yIdx = (wgIdx%(nSplit/2))*2 + (bIdx>>1);
		int cellIdx = xIdx+yIdx*nSplit;
		printf("cellIdx=%d\n",cellIdx);
		if( gN[cellIdx] == 0 ) 
			continue;

		const int start = gOffsets[cellIdx];
		const int end = start + gN[cellIdx];

		for (int c=start;c<end;c++)
		{
			b3GpuConstraint4& constraint = cpuConstraints[c];
			//printf("constraint (%d,%d)\n", constraint.m_bodyA,constraint.m_bodyB);
			if (usedBodies.findLinearSearch(constraint.m_bodyA)< usedBodies.size())
			{
				printf("error?\n");
			}
			if (usedBodies.findLinearSearch(constraint.m_bodyB)< usedBodies.size())
			{
				printf("error?\n");
			}
		}

		for (int c=start;c<end;c++)
		{
			b3GpuConstraint4& constraint = cpuConstraints[c];
			usedBodies.push_back(constraint.m_bodyA);
			usedBodies.push_back(constraint.m_bodyB);
		}

	}
}

static bool verify=false;

void b3Solver::solveContactConstraint(  const b3OpenCLArray<b3RigidBodyCL>* bodyBuf, const b3OpenCLArray<b3InertiaCL>* shapeBuf, 
			b3OpenCLArray<b3GpuConstraint4>* constraint, void* additionalData, int n ,int maxNumBatches)
{
	
	
	b3Int4 cdata = b3MakeInt4( n, 0, 0, 0 );
	{
		
		const int nn = N_SPLIT*N_SPLIT;

		cdata.x = 0;
		cdata.y = maxNumBatches;//250;


		int numWorkItems = 64*nn/N_BATCHES;
#ifdef DEBUG_ME
		SolverDebugInfo* debugInfo = new  SolverDebugInfo[numWorkItems];
		adl::b3OpenCLArray<SolverDebugInfo> gpuDebugInfo(data->m_device,numWorkItems);
#endif



		{

			B3_PROFILE("m_batchSolveKernel iterations");
			for(int iter=0; iter<m_nIterations; iter++)
			{
				for(int ib=0; ib<N_BATCHES; ib++)
				{
					
					if (verify)
					{
						checkConstraintBatch(bodyBuf,shapeBuf,constraint,m_numConstraints,m_offsets,ib);
					}

#ifdef DEBUG_ME
					memset(debugInfo,0,sizeof(SolverDebugInfo)*numWorkItems);
					gpuDebugInfo.write(debugInfo,numWorkItems);
#endif


					cdata.z = ib;
					cdata.w = N_SPLIT;

				b3LauncherCL launcher( m_queue, m_solveContactKernel );
#if 1
                    
					b3BufferInfoCL bInfo[] = { 

						b3BufferInfoCL( bodyBuf->getBufferCL() ), 
						b3BufferInfoCL( shapeBuf->getBufferCL() ), 
						b3BufferInfoCL( constraint->getBufferCL() ),
						b3BufferInfoCL( m_numConstraints->getBufferCL() ), 
						b3BufferInfoCL( m_offsets->getBufferCL() ) 
#ifdef DEBUG_ME
						,	b3BufferInfoCL(&gpuDebugInfo)
#endif
						};

					

                    launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(b3BufferInfoCL) );
					//launcher.setConst(  cdata.x );
                    launcher.setConst(  cdata.y );
                    launcher.setConst(  cdata.z );
                    launcher.setConst(  cdata.w );
                    launcher.launch1D( numWorkItems, 64 );

                    
#else
                    const char* fileName = "m_batchSolveKernel.bin";
                    FILE* f = fopen(fileName,"rb");
                    if (f)
                    {
                        int sizeInBytes=0;
                        if (fseek(f, 0, SEEK_END) || (sizeInBytes = ftell(f)) == EOF || fseek(f, 0, SEEK_SET))
                        {
                            printf("error, cannot get file size\n");
                            exit(0);
                        }
                        
                        unsigned char* buf = (unsigned char*) malloc(sizeInBytes);
                        fread(buf,sizeInBytes,1,f);
                        int serializedBytes = launcher.deserializeArgs(buf, sizeInBytes,m_context);
                        int num = *(int*)&buf[serializedBytes];
                        
                        launcher.launch1D( num);

                        //this clFinish is for testing on errors
                        clFinish(m_queue);
                    }

#endif
					

#ifdef DEBUG_ME
					clFinish(m_queue);
					gpuDebugInfo.read(debugInfo,numWorkItems);
					clFinish(m_queue);
					for (int i=0;i<numWorkItems;i++)
					{
						if (debugInfo[i].m_valInt2>0)
						{
							printf("debugInfo[i].m_valInt2 = %d\n",i,debugInfo[i].m_valInt2);
						}

						if (debugInfo[i].m_valInt3>0)
						{
							printf("debugInfo[i].m_valInt3 = %d\n",i,debugInfo[i].m_valInt3);
						}
					}
#endif //DEBUG_ME


				}
			}
		
			clFinish(m_queue);


		}

		cdata.x = 1;
		bool applyFriction=true;
		if (applyFriction)
    	{
			B3_PROFILE("m_batchSolveKernel iterations2");
			for(int iter=0; iter<m_nIterations; iter++)
			{
				for(int ib=0; ib<N_BATCHES; ib++)
				{
					cdata.z = ib;
					cdata.w = N_SPLIT;

					b3BufferInfoCL bInfo[] = { 
						b3BufferInfoCL( bodyBuf->getBufferCL() ), 
						b3BufferInfoCL( shapeBuf->getBufferCL() ), 
						b3BufferInfoCL( constraint->getBufferCL() ),
						b3BufferInfoCL( m_numConstraints->getBufferCL() ), 
						b3BufferInfoCL( m_offsets->getBufferCL() )
#ifdef DEBUG_ME
						,b3BufferInfoCL(&gpuDebugInfo)
#endif //DEBUG_ME
					};
					b3LauncherCL launcher( m_queue, m_solveFrictionKernel );
					launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(b3BufferInfoCL) );
					//launcher.setConst(  cdata.x );
                    launcher.setConst(  cdata.y );
                    launcher.setConst(  cdata.z );
                    launcher.setConst(  cdata.w );
                    
					launcher.launch1D( 64*nn/N_BATCHES, 64 );
				}
			}
			clFinish(m_queue);
			
		}
#ifdef DEBUG_ME
		delete[] debugInfo;
#endif //DEBUG_ME
	}

	
}

void b3Solver::convertToConstraints( const b3OpenCLArray<b3RigidBodyCL>* bodyBuf, 
	const b3OpenCLArray<b3InertiaCL>* shapeBuf, 
	b3OpenCLArray<b3Contact4>* contactsIn, b3OpenCLArray<b3GpuConstraint4>* contactCOut, void* additionalData, 
	int nContacts, const ConstraintCfg& cfg )
{
	b3OpenCLArray<b3GpuConstraint4>* constraintNative =0;

	struct CB
	{
		int m_nContacts;
		float m_dt;
		float m_positionDrift;
		float m_positionConstraintCoeff;
	};

	{
		B3_PROFILE("m_contactToConstraintKernel");
		CB cdata;
		cdata.m_nContacts = nContacts;
		cdata.m_dt = cfg.m_dt;
		cdata.m_positionDrift = cfg.m_positionDrift;
		cdata.m_positionConstraintCoeff = cfg.m_positionConstraintCoeff;

		
		b3BufferInfoCL bInfo[] = { b3BufferInfoCL( contactsIn->getBufferCL() ), b3BufferInfoCL( bodyBuf->getBufferCL() ), b3BufferInfoCL( shapeBuf->getBufferCL()),
			b3BufferInfoCL( contactCOut->getBufferCL() )};
		b3LauncherCL launcher( m_queue, m_contactToConstraintKernel );
		launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(b3BufferInfoCL) );
		//launcher.setConst(  cdata );
        
        launcher.setConst(cdata.m_nContacts);
		launcher.setConst(cdata.m_dt);
		launcher.setConst(cdata.m_positionDrift);
		launcher.setConst(cdata.m_positionConstraintCoeff);
        
		launcher.launch1D( nContacts, 64 );	
		clFinish(m_queue);

	}

	contactCOut->resize(nContacts);
}

/*
void b3Solver::sortContacts(  const b3OpenCLArray<b3RigidBodyCL>* bodyBuf, 
			b3OpenCLArray<b3Contact4>* contactsIn, void* additionalData, 
			int nContacts, const b3Solver::ConstraintCfg& cfg )
{
	
	

	const int sortAlignment = 512; // todo. get this out of sort
	if( cfg.m_enableParallelSolve )
	{
		

		int sortSize = NEXTMULTIPLEOF( nContacts, sortAlignment );

		b3OpenCLArray<unsigned int>* countsNative = m_numConstraints;//BufferUtils::map<TYPE_CL, false>( data->m_device, &countsHost );
		b3OpenCLArray<unsigned int>* offsetsNative = m_offsets;//BufferUtils::map<TYPE_CL, false>( data->m_device, &offsetsHost );

		{	//	2. set cell idx
			struct CB
			{
				int m_nContacts;
				int m_staticIdx;
				float m_scale;
				int m_nSplit;
			};

			b3Assert( sortSize%64 == 0 );
			CB cdata;
			cdata.m_nContacts = nContacts;
			cdata.m_staticIdx = cfg.m_staticIdx;
			cdata.m_scale = 1.f/(N_OBJ_PER_SPLIT*cfg.m_averageExtent);
			cdata.m_nSplit = N_SPLIT;

			
			b3BufferInfoCL bInfo[] = { b3BufferInfoCL( contactsIn->getBufferCL() ), b3BufferInfoCL( bodyBuf->getBufferCL() ), b3BufferInfoCL( m_sortDataBuffer->getBufferCL() ) };
			b3LauncherCL launcher( m_queue, m_setSortDataKernel );
			launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(b3BufferInfoCL) );
			launcher.setConst(  cdata );
			launcher.launch1D( sortSize, 64 );
		}

		{	//	3. sort by cell idx
			int n = N_SPLIT*N_SPLIT;
			int sortBit = 32;
			//if( n <= 0xffff ) sortBit = 16;
			//if( n <= 0xff ) sortBit = 8;
			m_sort32->execute(*m_sortDataBuffer,sortSize);
		}
		{	//	4. find entries
			m_search->execute( *m_sortDataBuffer, nContacts, *countsNative, N_SPLIT*N_SPLIT, b3BoundSearchCL::COUNT);

			m_scan->execute( *countsNative, *offsetsNative, N_SPLIT*N_SPLIT );
		}

		{	//	5. sort constraints by cellIdx
			//	todo. preallocate this
//			b3Assert( contactsIn->getType() == TYPE_HOST );
//			b3OpenCLArray<b3Contact4>* out = BufferUtils::map<TYPE_CL, false>( data->m_device, contactsIn );	//	copying contacts to this buffer

			{
				

				b3Int4 cdata; cdata.x = nContacts;
				b3BufferInfoCL bInfo[] = { b3BufferInfoCL( contactsIn->getBufferCL() ), b3BufferInfoCL( m_contactBuffer->getBufferCL() ), b3BufferInfoCL( m_sortDataBuffer->getBufferCL() ) };
				b3LauncherCL launcher( m_queue, m_reorderContactKernel );
				launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(b3BufferInfoCL) );
				launcher.setConst(  cdata );
				launcher.launch1D( nContacts, 64 );
			}
//			BufferUtils::unmap<true>( out, contactsIn, nContacts );
		}
	}

	
}

*/

void	b3Solver::batchContacts(  b3OpenCLArray<b3Contact4>* contacts, int nContacts, b3OpenCLArray<unsigned int>* nNative, b3OpenCLArray<unsigned int>* offsetsNative, int staticIdx )
{
	
	int numWorkItems = 64*N_SPLIT*N_SPLIT;
	{
		B3_PROFILE("batch generation");
		
		b3Int4 cdata;
		cdata.x = nContacts;
		cdata.y = 0;
		cdata.z = staticIdx;

		
#ifdef BATCH_DEBUG
		SolverDebugInfo* debugInfo = new  SolverDebugInfo[numWorkItems];
		adl::b3OpenCLArray<SolverDebugInfo> gpuDebugInfo(data->m_device,numWorkItems);
		memset(debugInfo,0,sizeof(SolverDebugInfo)*numWorkItems);
		gpuDebugInfo.write(debugInfo,numWorkItems);
#endif

		


		b3BufferInfoCL bInfo[] = { 
			b3BufferInfoCL( contacts->getBufferCL() ), 
			b3BufferInfoCL(  m_contactBuffer2->getBufferCL()),
			b3BufferInfoCL( nNative->getBufferCL() ), 
			b3BufferInfoCL( offsetsNative->getBufferCL() ),
#ifdef BATCH_DEBUG
			,	b3BufferInfoCL(&gpuDebugInfo)
#endif
		};

		
		

		{
			B3_PROFILE("batchingKernel");
			//b3LauncherCL launcher( m_queue, m_batchingKernel);
			cl_kernel k = useNewBatchingKernel ? m_batchingKernelNew : m_batchingKernel;

			b3LauncherCL launcher( m_queue, k);
			if (!useNewBatchingKernel )
			{
				launcher.setBuffer( contacts->getBufferCL() );
			}
			launcher.setBuffer( m_contactBuffer2->getBufferCL() );
			launcher.setBuffer( nNative->getBufferCL());
			launcher.setBuffer( offsetsNative->getBufferCL());

			//launcher.setConst(  cdata );
            launcher.setConst(staticIdx);
            
			launcher.launch1D( numWorkItems, 64 );
			clFinish(m_queue);
		}

#ifdef BATCH_DEBUG
	aaaa
		b3Contact4* hostContacts = new b3Contact4[nContacts];
		m_contactBuffer->read(hostContacts,nContacts);
		clFinish(m_queue);

		gpuDebugInfo.read(debugInfo,numWorkItems);
		clFinish(m_queue);

		for (int i=0;i<numWorkItems;i++)
		{
			if (debugInfo[i].m_valInt1>0)
			{
				printf("catch\n");
			}
			if (debugInfo[i].m_valInt2>0)
			{
				printf("catch22\n");
			}

			if (debugInfo[i].m_valInt3>0)
			{
				printf("catch666\n");
			}

			if (debugInfo[i].m_valInt4>0)
			{
				printf("catch777\n");
			}
		}
		delete[] debugInfo;
#endif //BATCH_DEBUG

	}

//	copy buffer to buffer
	//b3Assert(m_contactBuffer->size()==nContacts);
	//contacts->copyFromOpenCLArray( *m_contactBuffer);
	//clFinish(m_queue);//needed?
	
	
	
}

