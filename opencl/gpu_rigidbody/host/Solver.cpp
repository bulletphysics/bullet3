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


#include "Solver.h"
#include "../Stubs/AdlMatrix3x3.h"

#define SOLVER_SETUP_KERNEL_PATH "opencl/gpu_rigidbody/kernels/solverSetup.cl"
#define SOLVER_SETUP2_KERNEL_PATH "opencl/gpu_rigidbody/kernels/solverSetup2.cl"

#define SOLVER_CONTACT_KERNEL_PATH "opencl/gpu_rigidbody/kernels/solveContact.cl"
#define SOLVER_FRICTION_KERNEL_PATH "opencl/gpu_rigidbody/kernels/solveFriction.cl"

#define BATCHING_PATH "opencl/gpu_rigidbody/kernels/batchingKernels.cl"


#include "../kernels/solverSetup.h"
#include "../kernels/solverSetup2.h"

#include "../kernels/solveContact.h"
#include "../kernels/solveFriction.h"

#include "../kernels/batchingKernels.h"
#include "BulletCommon/btQuickprof.h"
#include "../../parallel_primitives/host/btLauncherCL.h"


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
		btOpenCLArray<unsigned int>* m_numConstraints;
		btOpenCLArray<unsigned int>* m_offsets;
	};
};



Solver::Solver(cl_context ctx, cl_device_id device, cl_command_queue queue, int pairCapacity)
			:m_nIterations(4),
			m_context(ctx),
			m_device(device),
			m_queue(queue)
{
	m_sort32 = new btRadixSort32CL(ctx,device,queue);
	m_scan = new btPrefixScanCL(ctx,device,queue,N_SPLIT*N_SPLIT);
	m_search = new btBoundSearchCL(ctx,device,queue,N_SPLIT*N_SPLIT);

	const int sortSize = NEXTMULTIPLEOF( pairCapacity, 512 );

	m_sortDataBuffer = new btOpenCLArray<btSortData>(ctx,queue,sortSize);
	m_contactBuffer = new btOpenCLArray<btContact4>(ctx,queue);

	m_numConstraints = new btOpenCLArray<unsigned int>(ctx,queue,N_SPLIT*N_SPLIT );
	m_numConstraints->resize(N_SPLIT*N_SPLIT);

	m_offsets = new btOpenCLArray<unsigned int>( ctx,queue, N_SPLIT*N_SPLIT );
	m_offsets->resize(N_SPLIT*N_SPLIT);
	const char* additionalMacros = "";
	const char* srcFileNameForCaching="";



	cl_int pErrNum;
	const char* batchKernelSource = batchingKernelsCL;
	const char* solverSetupSource = solverSetupCL;
	const char* solverSetup2Source = solverSetup2CL;
	const char* solveContactSource = solveContactCL;
	const char* solveFrictionSource = solveFrictionCL;
	
	
	
	{
		
		cl_program solveContactProg= btOpenCLUtils::compileCLProgramFromString( ctx, device, solveContactSource, &pErrNum,additionalMacros, SOLVER_CONTACT_KERNEL_PATH);
		btAssert(solveContactProg);
		
		cl_program solveFrictionProg= btOpenCLUtils::compileCLProgramFromString( ctx, device, solveFrictionSource, &pErrNum,additionalMacros, SOLVER_FRICTION_KERNEL_PATH);
		btAssert(solveFrictionProg);

		cl_program solverSetup2Prog= btOpenCLUtils::compileCLProgramFromString( ctx, device, solverSetup2Source, &pErrNum,additionalMacros, SOLVER_SETUP2_KERNEL_PATH);
		btAssert(solverSetup2Prog);

		
		cl_program solverSetupProg= btOpenCLUtils::compileCLProgramFromString( ctx, device, solverSetupSource, &pErrNum,additionalMacros, SOLVER_SETUP_KERNEL_PATH);
		btAssert(solverSetupProg);
		
		
		m_solveFrictionKernel= btOpenCLUtils::compileCLKernelFromString( ctx, device, solveFrictionSource, "BatchSolveKernelFriction", &pErrNum, solveFrictionProg,additionalMacros );
		btAssert(m_solveFrictionKernel);

		m_solveContactKernel= btOpenCLUtils::compileCLKernelFromString( ctx, device, solveContactSource, "BatchSolveKernelContact", &pErrNum, solveContactProg,additionalMacros );
		btAssert(m_solveContactKernel);
		
		m_contactToConstraintKernel = btOpenCLUtils::compileCLKernelFromString( ctx, device, solverSetupSource, "ContactToConstraintKernel", &pErrNum, solverSetupProg,additionalMacros );
		btAssert(m_contactToConstraintKernel);
			
		m_setSortDataKernel =  btOpenCLUtils::compileCLKernelFromString( ctx, device, solverSetup2Source, "SetSortDataKernel", &pErrNum, solverSetup2Prog,additionalMacros );
		btAssert(m_setSortDataKernel);
				
		m_reorderContactKernel = btOpenCLUtils::compileCLKernelFromString( ctx, device, solverSetup2Source, "ReorderContactKernel", &pErrNum, solverSetup2Prog,additionalMacros );
		btAssert(m_reorderContactKernel);
		

		m_copyConstraintKernel = btOpenCLUtils::compileCLKernelFromString( ctx, device, solverSetup2Source, "CopyConstraintKernel", &pErrNum, solverSetup2Prog,additionalMacros );
		btAssert(m_copyConstraintKernel);
		
	}

	{
		cl_program batchingProg = btOpenCLUtils::compileCLProgramFromString( ctx, device, batchKernelSource, &pErrNum,additionalMacros, BATCHING_PATH);
		btAssert(batchingProg);
		
		m_batchingKernel = btOpenCLUtils::compileCLKernelFromString( ctx, device, batchKernelSource, "CreateBatches", &pErrNum, batchingProg,additionalMacros );
		btAssert(m_batchingKernel);
	}
			
}
		
Solver::~Solver()
{
	delete m_sortDataBuffer;
	delete m_contactBuffer;

	delete m_sort32;
	delete m_scan;
	delete m_search;


	clReleaseKernel(m_batchingKernel);
	
	clReleaseKernel( m_solveContactKernel);
	clReleaseKernel( m_solveFrictionKernel);

	clReleaseKernel( m_contactToConstraintKernel);
	clReleaseKernel( m_setSortDataKernel);
	clReleaseKernel( m_reorderContactKernel);
	clReleaseKernel( m_copyConstraintKernel);
			
}


 


/*void Solver::reorderConvertToConstraints( const btOpenCLArray<btRigidBodyCL>* bodyBuf, 
	const btOpenCLArray<btInertiaCL>* shapeBuf,
	btOpenCLArray<btContact4>* contactsIn, btOpenCLArray<btGpuConstraint4>* contactCOut, void* additionalData, 
	int nContacts, const Solver::ConstraintCfg& cfg )
{
	if( m_contactBuffer )
	{
		m_contactBuffer->resize(nContacts);
	}
	if( m_contactBuffer == 0 )
	{
		BT_PROFILE("new m_contactBuffer;");
		m_contactBuffer = new btOpenCLArray<btContact4>(m_context,m_queue,nContacts );
		m_contactBuffer->resize(nContacts);
	}
	

	

	//DeviceUtils::Config dhCfg;
	//Device* deviceHost = DeviceUtils::allocate( TYPE_HOST, dhCfg );
	if( cfg.m_enableParallelSolve )
	{
		

		clFinish(m_queue);
		
		//	contactsIn -> m_contactBuffer
		{
			BT_PROFILE("sortContacts");
			sortContacts( bodyBuf, contactsIn, additionalData, nContacts, cfg );
			clFinish(m_queue);
		}
		
		
		{
			BT_PROFILE("m_copyConstraintKernel");

			

			btInt4 cdata; cdata.x = nContacts;
			btBufferInfoCL bInfo[] = { btBufferInfoCL( m_contactBuffer->getBufferCL() ), btBufferInfoCL( contactsIn->getBufferCL() ) };
//			btLauncherCL launcher( m_queue, data->m_device->getKernel( PATH, "CopyConstraintKernel",  "-I ..\\..\\ -Wf,--c++", 0 ) );
			btLauncherCL launcher( m_queue, m_copyConstraintKernel );
			launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(btBufferInfoCL) );
			launcher.setConst(  cdata );
			launcher.launch1D( nContacts, 64 );
			clFinish(m_queue);
		}

		{
			BT_PROFILE("batchContacts");
			Solver::batchContacts( contactsIn, nContacts, m_numConstraints, m_offsets, cfg.m_staticIdx );

		}
	}
	{
			BT_PROFILE("waitForCompletion (batchContacts)");
			clFinish(m_queue);
	}
	
	//================
	
	{
		BT_PROFILE("convertToConstraints");
		Solver::convertToConstraints(  bodyBuf, shapeBuf, contactsIn, contactCOut, additionalData, nContacts, cfg );
	}

	{
		BT_PROFILE("convertToConstraints waitForCompletion");
		clFinish(m_queue);
	}
	
}
*/

	static
	inline
	float calcRelVel(const float4& l0, const float4& l1, const float4& a0, const float4& a1, 
					 const float4& linVel0, const float4& angVel0, const float4& linVel1, const float4& angVel1)
	{
		return dot3F4(l0, linVel0) + dot3F4(a0, angVel0) + dot3F4(l1, linVel1) + dot3F4(a1, angVel1);
	}


	static
	inline
	void setLinearAndAngular(const float4& n, const float4& r0, const float4& r1,
							 float4& linear, float4& angular0, float4& angular1)
	{
		linear = -n;
		angular0 = -cross3(r0, n);
		angular1 = cross3(r1, n);
	}


template<bool JACOBI>
static
__inline
void solveContact(btGpuConstraint4& cs, 
	const float4& posA, float4& linVelA, float4& angVelA, float invMassA, const Matrix3x3& invInertiaA,
	const float4& posB, float4& linVelB, float4& angVelB, float invMassB, const Matrix3x3& invInertiaB, 
	float maxRambdaDt[4], float minRambdaDt[4])
{
	float4 dLinVelA = make_float4(0.f);
	float4 dAngVelA = make_float4(0.f);
	float4 dLinVelB = make_float4(0.f);
	float4 dAngVelB = make_float4(0.f);

	for(int ic=0; ic<4; ic++)
	{
		//	dont necessary because this makes change to 0
		if( cs.m_jacCoeffInv[ic] == 0.f ) continue;

		{
			float4 angular0, angular1, linear;
			btVector3 r0 = cs.m_worldPos[ic] - (btVector3&)posA;
			btVector3 r1 = cs.m_worldPos[ic] - (btVector3&)posB;
			setLinearAndAngular( (const float4 &)-cs.m_linear, (const float4 &)r0, (const float4 &)r1, linear, angular0, angular1 );

			float rambdaDt = calcRelVel((const float4 &)cs.m_linear,(const float4 &) -cs.m_linear, angular0, angular1,
				linVelA, angVelA, linVelB, angVelB ) + cs.m_b[ic];
			rambdaDt *= cs.m_jacCoeffInv[ic];

			{
				float prevSum = cs.m_appliedRambdaDt[ic];
				float updated = prevSum;
				updated += rambdaDt;
				updated = max2( updated, minRambdaDt[ic] );
				updated = min2( updated, maxRambdaDt[ic] );
				rambdaDt = updated - prevSum;
				cs.m_appliedRambdaDt[ic] = updated;
			}

			float4 linImp0 = invMassA*linear*rambdaDt;
			float4 linImp1 = invMassB*(-linear)*rambdaDt;
			float4 angImp0 = mtMul1(invInertiaA, angular0)*rambdaDt;
			float4 angImp1 = mtMul1(invInertiaB, angular1)*rambdaDt;
#ifdef _WIN32
            btAssert(_finite(linImp0.x));
			btAssert(_finite(linImp1.x));
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


 void btPlaneSpace1 (const float4* n, float4* p, float4* q)
{
  if (btFabs(n->z) > SIMDSQRT12) {
    // choose p in y-z plane
    btScalar a = n->y*n->y + n->z*n->z;
    btScalar k = btRecipSqrt (a);
    p->x = 0;
	p->y = -n->z*k;
	p->z = n->y*k;
    // set q = n x p
    q->x = a*k;
	q->y = -n->x*p->z;
	q->z = n->x*p->y;
  }
  else {
    // choose p in x-y plane
    btScalar a = n->x*n->x + n->y*n->y;
    btScalar k = btRecipSqrt (a);
    p->x = -n->y*k;
	p->y = n->x*k;
	p->z = 0;
    // set q = n x p
    q->x = -n->z*p->y;
	q->y = n->z*p->x;
	q->z = a*k;
  }
}


	static
	__inline
	void solveFriction(btGpuConstraint4& cs, 
		const float4& posA, float4& linVelA, float4& angVelA, float invMassA, const Matrix3x3& invInertiaA,
		const float4& posB, float4& linVelB, float4& angVelB, float invMassB, const Matrix3x3& invInertiaB, 
		float maxRambdaDt[4], float minRambdaDt[4])
	{
		if( cs.m_fJacCoeffInv[0] == 0 && cs.m_fJacCoeffInv[0] == 0 ) return;
		const float4& center = (const float4&)cs.m_center;

		float4 n = -(const float4&)cs.m_linear;

		float4 tangent[2];
#if 1		
		btPlaneSpace1 (&n, &tangent[0],&tangent[1]);
#else
		float4 r = cs.m_worldPos[0]-center;
		tangent[0] = cross3( n, r );
		tangent[1] = cross3( tangent[0], n );
		tangent[0] = normalize3( tangent[0] );
		tangent[1] = normalize3( tangent[1] );
#endif

		float4 angular0, angular1, linear;
		float4 r0 = center - posA;
		float4 r1 = center - posB;
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
					updated = max2( updated, minRambdaDt[i] );
					updated = min2( updated, maxRambdaDt[i] );
					rambdaDt = updated - prevSum;
					cs.m_fAppliedRambdaDt[i] = updated;
				}

			float4 linImp0 = invMassA*linear*rambdaDt;
			float4 linImp1 = invMassB*(-linear)*rambdaDt;
			float4 angImp0 = mtMul1(invInertiaA, angular0)*rambdaDt;
			float4 angImp1 = mtMul1(invInertiaB, angular1)*rambdaDt;
#ifdef _WIN32
			btAssert(_finite(linImp0.x));
			btAssert(_finite(linImp1.x));
#endif
			linVelA += linImp0;
			angVelA += angImp0;
			linVelB += linImp1;
			angVelB += angImp1;
		}

		{	//	angular damping for point constraint
			float4 ab = normalize3( posB - posA );
			float4 ac = normalize3( center - posA );
			if( dot3F4( ab, ac ) > 0.95f || (invMassA == 0.f || invMassB == 0.f))
			{
				float angNA = dot3F4( n, angVelA );
				float angNB = dot3F4( n, angVelB );

				angVelA -= (angNA*0.1f)*n;
				angVelB -= (angNB*0.1f)*n;
			}
		}
	}


struct SolveTask// : public ThreadPool::Task
{
	SolveTask(btAlignedObjectArray<btRigidBodyCL>& bodies,  btAlignedObjectArray<btInertiaCL>& shapes, btAlignedObjectArray<btGpuConstraint4>& constraints,
		int start, int nConstraints)
		: m_bodies( bodies ), m_shapes( shapes ), m_constraints( constraints ), m_start( start ), m_nConstraints( nConstraints ),
		m_solveFriction( true ){}

	u16 getType(){ return 0; }

	void run(int tIdx)
	{
		

		for(int ic=0; ic<m_nConstraints; ic++)
		{
			int i = m_start + ic;

			float frictionCoeff = m_constraints[i].getFrictionCoeff();
			int aIdx = (int)m_constraints[i].m_bodyA;
			int bIdx = (int)m_constraints[i].m_bodyB;
			btRigidBodyCL& bodyA = m_bodies[aIdx];
			btRigidBodyCL& bodyB = m_bodies[bIdx];

			if( !m_solveFriction )
			{
				float maxRambdaDt[4] = {FLT_MAX,FLT_MAX,FLT_MAX,FLT_MAX};
				float minRambdaDt[4] = {0.f,0.f,0.f,0.f};

				solveContact<false>( m_constraints[i], (float4&)bodyA.m_pos, (float4&)bodyA.m_linVel, (float4&)bodyA.m_angVel, bodyA.m_invMass, (const Matrix3x3 &)m_shapes[aIdx].m_invInertiaWorld, 
					 (float4&)bodyB.m_pos, (float4&)bodyB.m_linVel, (float4&)bodyB.m_angVel, bodyB.m_invMass, (const Matrix3x3 &)m_shapes[bIdx].m_invInertiaWorld,
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

			solveFriction( m_constraints[i], (float4&)bodyA.m_pos, (float4&)bodyA.m_linVel, (float4&)bodyA.m_angVel, bodyA.m_invMass,(const Matrix3x3 &) m_shapes[aIdx].m_invInertiaWorld, 
					(float4&)bodyB.m_pos, (float4&)bodyB.m_linVel, (float4&)bodyB.m_angVel, bodyB.m_invMass,(const Matrix3x3 &) m_shapes[bIdx].m_invInertiaWorld,
					maxRambdaDt, minRambdaDt );
			
			}
		}

		
	}

	btAlignedObjectArray<btRigidBodyCL>& m_bodies;
	btAlignedObjectArray<btInertiaCL>& m_shapes;
	btAlignedObjectArray<btGpuConstraint4>& m_constraints;
	int m_start;
	int m_nConstraints;
	bool m_solveFriction;
};


void Solver::solveContactConstraintHost(  btOpenCLArray<btRigidBodyCL>* bodyBuf, btOpenCLArray<btInertiaCL>* shapeBuf, 
			btOpenCLArray<btGpuConstraint4>* constraint, void* additionalData, int n ,int maxNumBatches)
{

	btAlignedObjectArray<btRigidBodyCL> bodyNative;
	bodyBuf->copyToHost(bodyNative);
	btAlignedObjectArray<btInertiaCL> shapeNative;
	shapeBuf->copyToHost(shapeNative);
	btAlignedObjectArray<btGpuConstraint4> constraintNative;
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

void Solver::solveContactConstraint(  const btOpenCLArray<btRigidBodyCL>* bodyBuf, const btOpenCLArray<btInertiaCL>* shapeBuf, 
			btOpenCLArray<btGpuConstraint4>* constraint, void* additionalData, int n ,int maxNumBatches)
{
	
	
	btInt4 cdata = btMakeInt4( n, 0, 0, 0 );
	{
		
		const int nn = N_SPLIT*N_SPLIT;

		cdata.x = 0;
		cdata.y = maxNumBatches;//250;


		int numWorkItems = 64*nn/N_BATCHES;
#ifdef DEBUG_ME
		SolverDebugInfo* debugInfo = new  SolverDebugInfo[numWorkItems];
		adl::btOpenCLArray<SolverDebugInfo> gpuDebugInfo(data->m_device,numWorkItems);
#endif



		{

			BT_PROFILE("m_batchSolveKernel iterations");
			for(int iter=0; iter<m_nIterations; iter++)
			{
				for(int ib=0; ib<N_BATCHES; ib++)
				{
#ifdef DEBUG_ME
					memset(debugInfo,0,sizeof(SolverDebugInfo)*numWorkItems);
					gpuDebugInfo.write(debugInfo,numWorkItems);
#endif


					cdata.z = ib;
					cdata.w = N_SPLIT;

				btLauncherCL launcher( m_queue, m_solveContactKernel );
#if 1
                    
					btBufferInfoCL bInfo[] = { 

						btBufferInfoCL( bodyBuf->getBufferCL() ), 
						btBufferInfoCL( shapeBuf->getBufferCL() ), 
						btBufferInfoCL( constraint->getBufferCL() ),
						btBufferInfoCL( m_numConstraints->getBufferCL() ), 
						btBufferInfoCL( m_offsets->getBufferCL() ) 
#ifdef DEBUG_ME
						,	btBufferInfoCL(&gpuDebugInfo)
#endif
						};

					

                    launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(btBufferInfoCL) );
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
			BT_PROFILE("m_batchSolveKernel iterations2");
			for(int iter=0; iter<m_nIterations; iter++)
			{
				for(int ib=0; ib<N_BATCHES; ib++)
				{
					cdata.z = ib;
					cdata.w = N_SPLIT;

					btBufferInfoCL bInfo[] = { 
						btBufferInfoCL( bodyBuf->getBufferCL() ), 
						btBufferInfoCL( shapeBuf->getBufferCL() ), 
						btBufferInfoCL( constraint->getBufferCL() ),
						btBufferInfoCL( m_numConstraints->getBufferCL() ), 
						btBufferInfoCL( m_offsets->getBufferCL() )
#ifdef DEBUG_ME
						,btBufferInfoCL(&gpuDebugInfo)
#endif //DEBUG_ME
					};
					btLauncherCL launcher( m_queue, m_solveFrictionKernel );
					launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(btBufferInfoCL) );
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

void Solver::convertToConstraints( const btOpenCLArray<btRigidBodyCL>* bodyBuf, 
	const btOpenCLArray<btInertiaCL>* shapeBuf, 
	btOpenCLArray<btContact4>* contactsIn, btOpenCLArray<btGpuConstraint4>* contactCOut, void* additionalData, 
	int nContacts, const ConstraintCfg& cfg )
{
	btOpenCLArray<btGpuConstraint4>* constraintNative =0;

	struct CB
	{
		int m_nContacts;
		float m_dt;
		float m_positionDrift;
		float m_positionConstraintCoeff;
	};

	{
		BT_PROFILE("m_contactToConstraintKernel");
		CB cdata;
		cdata.m_nContacts = nContacts;
		cdata.m_dt = cfg.m_dt;
		cdata.m_positionDrift = cfg.m_positionDrift;
		cdata.m_positionConstraintCoeff = cfg.m_positionConstraintCoeff;

		
		btBufferInfoCL bInfo[] = { btBufferInfoCL( contactsIn->getBufferCL() ), btBufferInfoCL( bodyBuf->getBufferCL() ), btBufferInfoCL( shapeBuf->getBufferCL()),
			btBufferInfoCL( contactCOut->getBufferCL() )};
		btLauncherCL launcher( m_queue, m_contactToConstraintKernel );
		launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(btBufferInfoCL) );
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
void Solver::sortContacts(  const btOpenCLArray<btRigidBodyCL>* bodyBuf, 
			btOpenCLArray<btContact4>* contactsIn, void* additionalData, 
			int nContacts, const Solver::ConstraintCfg& cfg )
{
	
	

	const int sortAlignment = 512; // todo. get this out of sort
	if( cfg.m_enableParallelSolve )
	{
		

		int sortSize = NEXTMULTIPLEOF( nContacts, sortAlignment );

		btOpenCLArray<unsigned int>* countsNative = m_numConstraints;//BufferUtils::map<TYPE_CL, false>( data->m_device, &countsHost );
		btOpenCLArray<unsigned int>* offsetsNative = m_offsets;//BufferUtils::map<TYPE_CL, false>( data->m_device, &offsetsHost );

		{	//	2. set cell idx
			struct CB
			{
				int m_nContacts;
				int m_staticIdx;
				float m_scale;
				int m_nSplit;
			};

			btAssert( sortSize%64 == 0 );
			CB cdata;
			cdata.m_nContacts = nContacts;
			cdata.m_staticIdx = cfg.m_staticIdx;
			cdata.m_scale = 1.f/(N_OBJ_PER_SPLIT*cfg.m_averageExtent);
			cdata.m_nSplit = N_SPLIT;

			
			btBufferInfoCL bInfo[] = { btBufferInfoCL( contactsIn->getBufferCL() ), btBufferInfoCL( bodyBuf->getBufferCL() ), btBufferInfoCL( m_sortDataBuffer->getBufferCL() ) };
			btLauncherCL launcher( m_queue, m_setSortDataKernel );
			launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(btBufferInfoCL) );
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
			m_search->execute( *m_sortDataBuffer, nContacts, *countsNative, N_SPLIT*N_SPLIT, btBoundSearchCL::COUNT);

			m_scan->execute( *countsNative, *offsetsNative, N_SPLIT*N_SPLIT );
		}

		{	//	5. sort constraints by cellIdx
			//	todo. preallocate this
//			btAssert( contactsIn->getType() == TYPE_HOST );
//			btOpenCLArray<btContact4>* out = BufferUtils::map<TYPE_CL, false>( data->m_device, contactsIn );	//	copying contacts to this buffer

			{
				

				btInt4 cdata; cdata.x = nContacts;
				btBufferInfoCL bInfo[] = { btBufferInfoCL( contactsIn->getBufferCL() ), btBufferInfoCL( m_contactBuffer->getBufferCL() ), btBufferInfoCL( m_sortDataBuffer->getBufferCL() ) };
				btLauncherCL launcher( m_queue, m_reorderContactKernel );
				launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(btBufferInfoCL) );
				launcher.setConst(  cdata );
				launcher.launch1D( nContacts, 64 );
			}
//			BufferUtils::unmap<true>( out, contactsIn, nContacts );
		}
	}

	
}

*/

void Solver::batchContacts(  btOpenCLArray<btContact4>* contacts, int nContacts, btOpenCLArray<unsigned int>* nNative, btOpenCLArray<unsigned int>* offsetsNative, int staticIdx )
{

	{
		BT_PROFILE("batch generation");
		
		btInt4 cdata;
		cdata.x = nContacts;
		cdata.y = 0;
		cdata.z = staticIdx;

		int numWorkItems = 64*N_SPLIT*N_SPLIT;
#ifdef BATCH_DEBUG
		SolverDebugInfo* debugInfo = new  SolverDebugInfo[numWorkItems];
		adl::btOpenCLArray<SolverDebugInfo> gpuDebugInfo(data->m_device,numWorkItems);
		memset(debugInfo,0,sizeof(SolverDebugInfo)*numWorkItems);
		gpuDebugInfo.write(debugInfo,numWorkItems);
#endif


		btBufferInfoCL bInfo[] = { 
			btBufferInfoCL( contacts->getBufferCL() ), 
			btBufferInfoCL( m_contactBuffer->getBufferCL() ), 
			btBufferInfoCL( nNative->getBufferCL() ), 
			btBufferInfoCL( offsetsNative->getBufferCL() ) 
#ifdef BATCH_DEBUG
			,	btBufferInfoCL(&gpuDebugInfo)
#endif
		};

		
		{
			BT_PROFILE("batchingKernel");
			btLauncherCL launcher( m_queue, m_batchingKernel);
			launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(btBufferInfoCL) );
			//launcher.setConst(  cdata );
            launcher.setConst(staticIdx);
            
			launcher.launch1D( numWorkItems, 64 );
			clFinish(m_queue);
		}

#ifdef BATCH_DEBUG
	aaaa
		btContact4* hostContacts = new btContact4[nContacts];
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
	btAssert(m_contactBuffer->size()==nContacts);
	//contacts->copyFromOpenCLArray( *m_contactBuffer);
	//clFinish(m_queue);//needed?

	
}

