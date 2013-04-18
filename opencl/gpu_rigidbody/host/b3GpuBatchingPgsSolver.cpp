

#include "b3GpuBatchingPgsSolver.h"
#include "../../parallel_primitives/host/btRadixSort32CL.h"
#include "Bullet3Common/b3Quickprof.h"
#include "../../parallel_primitives/host/btLauncherCL.h"
#include "../../parallel_primitives/host/btBoundSearchCL.h"
#include "../../parallel_primitives/host/btPrefixScanCL.h"
#include <string.h>
#include "../../basic_initialize/b3OpenCLUtils.h"
#include "../host/b3Config.h"
#include "b3Solver.h"


#define SOLVER_SETUP_KERNEL_PATH "opencl/gpu_rigidbody/kernels/solverSetup.cl"
#define SOLVER_SETUP2_KERNEL_PATH "opencl/gpu_rigidbody/kernels/solverSetup2.cl"
#define SOLVER_CONTACT_KERNEL_PATH "opencl/gpu_rigidbody/kernels/solveContact.cl"
#define SOLVER_FRICTION_KERNEL_PATH "opencl/gpu_rigidbody/kernels/solveFriction.cl"
#define BATCHING_PATH "opencl/gpu_rigidbody/kernels/batchingKernels.cl"
#define BATCHING_NEW_PATH "opencl/gpu_rigidbody/kernels/batchingKernelsNew.cl"

#include "../kernels/solverSetup.h"
#include "../kernels/solverSetup2.h"
#include "../kernels/solveContact.h"
#include "../kernels/solveFriction.h"
#include "../kernels/batchingKernels.h"
#include "../kernels/batchingKernelsNew.h"



enum
{
	BT_SOLVER_N_SPLIT = 16,
	BT_SOLVER_N_BATCHES = 4,
	BT_SOLVER_N_OBJ_PER_SPLIT = 10,
	BT_SOLVER_N_TASKS_PER_BATCH = BT_SOLVER_N_SPLIT*BT_SOLVER_N_SPLIT,
};


bool gpuBatchContacts = true;//true;
bool gpuSolveConstraint = true;//true;


struct	btGpuBatchingPgsSolverInternalData
{
	cl_context m_context;
	cl_device_id m_device;
	cl_command_queue m_queue;
	int m_pairCapacity;
	int m_nIterations;

	btOpenCLArray<b3GpuConstraint4>* m_contactCGPU;
	btOpenCLArray<unsigned int>* m_numConstraints;
	btOpenCLArray<unsigned int>* m_offsets;
		
	b3Solver*		m_solverGPU;		
	
	cl_kernel m_batchingKernel;
	cl_kernel m_batchingKernelNew;
	cl_kernel m_solveContactKernel;
	cl_kernel m_solveFrictionKernel;
	cl_kernel m_contactToConstraintKernel;
	cl_kernel m_setSortDataKernel;
	cl_kernel m_reorderContactKernel;
	cl_kernel m_copyConstraintKernel;

	class btRadixSort32CL*	m_sort32;
	class btBoundSearchCL*	m_search;
	class btPrefixScanCL*	m_scan;

	btOpenCLArray<btSortData>* m_sortDataBuffer;
	btOpenCLArray<b3Contact4>* m_contactBuffer;

	btOpenCLArray<b3RigidBodyCL>* m_bodyBufferGPU;
	btOpenCLArray<btInertiaCL>* m_inertiaBufferGPU;
	btOpenCLArray<b3Contact4>* m_pBufContactOutGPU;


	b3AlignedObjectArray<unsigned int> m_idxBuffer;
	b3AlignedObjectArray<btSortData> m_sortData;
	b3AlignedObjectArray<b3Contact4> m_old;
};



b3GpuBatchingPgsSolver::b3GpuBatchingPgsSolver(cl_context ctx,cl_device_id device, cl_command_queue  q,int pairCapacity)
{
	m_data = new btGpuBatchingPgsSolverInternalData;
	m_data->m_context = ctx;
	m_data->m_device = device;
	m_data->m_queue = q;
	m_data->m_pairCapacity = pairCapacity;
	m_data->m_nIterations = 4;

	m_data->m_bodyBufferGPU = new btOpenCLArray<b3RigidBodyCL>(ctx,q);
	m_data->m_inertiaBufferGPU = new btOpenCLArray<btInertiaCL>(ctx,q);
	m_data->m_pBufContactOutGPU = new btOpenCLArray<b3Contact4>(ctx,q);

	m_data->m_solverGPU = new b3Solver(ctx,device,q,512*1024);

	m_data->m_sort32 = new btRadixSort32CL(ctx,device,m_data->m_queue);
	m_data->m_scan = new btPrefixScanCL(ctx,device,m_data->m_queue,BT_SOLVER_N_SPLIT*BT_SOLVER_N_SPLIT);
	m_data->m_search = new btBoundSearchCL(ctx,device,m_data->m_queue,BT_SOLVER_N_SPLIT*BT_SOLVER_N_SPLIT);

	const int sortSize = BTNEXTMULTIPLEOF( pairCapacity, 512 );

	m_data->m_sortDataBuffer = new btOpenCLArray<btSortData>(ctx,m_data->m_queue,sortSize);
	m_data->m_contactBuffer = new btOpenCLArray<b3Contact4>(ctx,m_data->m_queue);

	m_data->m_numConstraints = new btOpenCLArray<unsigned int>(ctx,m_data->m_queue,BT_SOLVER_N_SPLIT*BT_SOLVER_N_SPLIT );
	m_data->m_numConstraints->resize(BT_SOLVER_N_SPLIT*BT_SOLVER_N_SPLIT);

	m_data->m_contactCGPU = new btOpenCLArray<b3GpuConstraint4>(ctx,q,pairCapacity);

	m_data->m_offsets = new btOpenCLArray<unsigned int>( ctx,m_data->m_queue, BT_SOLVER_N_SPLIT*BT_SOLVER_N_SPLIT );
	m_data->m_offsets->resize(BT_SOLVER_N_SPLIT*BT_SOLVER_N_SPLIT);
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
		
		cl_program solveContactProg= b3OpenCLUtils::compileCLProgramFromString( ctx, device, solveContactSource, &pErrNum,additionalMacros, SOLVER_CONTACT_KERNEL_PATH);
		btAssert(solveContactProg);
		
		cl_program solveFrictionProg= b3OpenCLUtils::compileCLProgramFromString( ctx, device, solveFrictionSource, &pErrNum,additionalMacros, SOLVER_FRICTION_KERNEL_PATH);
		btAssert(solveFrictionProg);

		cl_program solverSetup2Prog= b3OpenCLUtils::compileCLProgramFromString( ctx, device, solverSetup2Source, &pErrNum,additionalMacros, SOLVER_SETUP2_KERNEL_PATH);
		btAssert(solverSetup2Prog);

		
		cl_program solverSetupProg= b3OpenCLUtils::compileCLProgramFromString( ctx, device, solverSetupSource, &pErrNum,additionalMacros, SOLVER_SETUP_KERNEL_PATH);
		btAssert(solverSetupProg);
		
		
		m_data->m_solveFrictionKernel= b3OpenCLUtils::compileCLKernelFromString( ctx, device, solveFrictionSource, "BatchSolveKernelFriction", &pErrNum, solveFrictionProg,additionalMacros );
		btAssert(m_data->m_solveFrictionKernel);

		m_data->m_solveContactKernel= b3OpenCLUtils::compileCLKernelFromString( ctx, device, solveContactSource, "BatchSolveKernelContact", &pErrNum, solveContactProg,additionalMacros );
		btAssert(m_data->m_solveContactKernel);
		
		m_data->m_contactToConstraintKernel = b3OpenCLUtils::compileCLKernelFromString( ctx, device, solverSetupSource, "ContactToConstraintKernel", &pErrNum, solverSetupProg,additionalMacros );
		btAssert(m_data->m_contactToConstraintKernel);
			
		m_data->m_setSortDataKernel =  b3OpenCLUtils::compileCLKernelFromString( ctx, device, solverSetup2Source, "SetSortDataKernel", &pErrNum, solverSetup2Prog,additionalMacros );
		btAssert(m_data->m_setSortDataKernel);
				
		m_data->m_reorderContactKernel = b3OpenCLUtils::compileCLKernelFromString( ctx, device, solverSetup2Source, "ReorderContactKernel", &pErrNum, solverSetup2Prog,additionalMacros );
		btAssert(m_data->m_reorderContactKernel);
		

		m_data->m_copyConstraintKernel = b3OpenCLUtils::compileCLKernelFromString( ctx, device, solverSetup2Source, "CopyConstraintKernel", &pErrNum, solverSetup2Prog,additionalMacros );
		btAssert(m_data->m_copyConstraintKernel);
		
	}

	{
		cl_program batchingProg = b3OpenCLUtils::compileCLProgramFromString( ctx, device, batchKernelSource, &pErrNum,additionalMacros, BATCHING_PATH);
		btAssert(batchingProg);
		
		m_data->m_batchingKernel = b3OpenCLUtils::compileCLKernelFromString( ctx, device, batchKernelSource, "CreateBatches", &pErrNum, batchingProg,additionalMacros );
		btAssert(m_data->m_batchingKernel);
	}
			
	{
		cl_program batchingNewProg = b3OpenCLUtils::compileCLProgramFromString( ctx, device, batchKernelNewSource, &pErrNum,additionalMacros, BATCHING_NEW_PATH);
		btAssert(batchingNewProg);
		
		m_data->m_batchingKernelNew = b3OpenCLUtils::compileCLKernelFromString( ctx, device, batchKernelNewSource, "CreateBatchesNew", &pErrNum, batchingNewProg,additionalMacros );
		btAssert(m_data->m_batchingKernelNew);
	}
		






}

b3GpuBatchingPgsSolver::~b3GpuBatchingPgsSolver()
{
	delete m_data->m_sortDataBuffer;
	delete m_data->m_contactBuffer;

	delete m_data->m_sort32;
	delete m_data->m_scan;
	delete m_data->m_search;


	clReleaseKernel(m_data->m_batchingKernel);
	clReleaseKernel(m_data->m_batchingKernelNew);
	
	clReleaseKernel( m_data->m_solveContactKernel);
	clReleaseKernel( m_data->m_solveFrictionKernel);

	clReleaseKernel( m_data->m_contactToConstraintKernel);
	clReleaseKernel( m_data->m_setSortDataKernel);
	clReleaseKernel( m_data->m_reorderContactKernel);
	clReleaseKernel( m_data->m_copyConstraintKernel);

	delete m_data;
}



struct btConstraintCfg
{
	btConstraintCfg( float dt = 0.f ): m_positionDrift( 0.005f ), m_positionConstraintCoeff( 0.2f ), m_dt(dt), m_staticIdx(0) {}

	float m_positionDrift;
	float m_positionConstraintCoeff;
	float m_dt;
	bool m_enableParallelSolve;
	float m_averageExtent;
	int m_staticIdx;
};





void b3GpuBatchingPgsSolver::solveContactConstraint(  const btOpenCLArray<b3RigidBodyCL>* bodyBuf, const btOpenCLArray<btInertiaCL>* shapeBuf, 
			btOpenCLArray<b3GpuConstraint4>* constraint, void* additionalData, int n ,int maxNumBatches,int numIterations)
{
	
	
	btInt4 cdata = btMakeInt4( n, 0, 0, 0 );
	{
		
		const int nn = BT_SOLVER_N_SPLIT*BT_SOLVER_N_SPLIT;

		cdata.x = 0;
		cdata.y = maxNumBatches;//250;


		int numWorkItems = 64*nn/BT_SOLVER_N_BATCHES;
#ifdef DEBUG_ME
		SolverDebugInfo* debugInfo = new  SolverDebugInfo[numWorkItems];
		adl::btOpenCLArray<SolverDebugInfo> gpuDebugInfo(data->m_device,numWorkItems);
#endif



		{

			BT_PROFILE("m_batchSolveKernel iterations");
			for(int iter=0; iter<numIterations; iter++)
			{
				for(int ib=0; ib<BT_SOLVER_N_BATCHES; ib++)
				{
#ifdef DEBUG_ME
					memset(debugInfo,0,sizeof(SolverDebugInfo)*numWorkItems);
					gpuDebugInfo.write(debugInfo,numWorkItems);
#endif


					cdata.z = ib;
					cdata.w = BT_SOLVER_N_SPLIT;

				btLauncherCL launcher( m_data->m_queue, m_data->m_solveContactKernel );
#if 1
                    
					btBufferInfoCL bInfo[] = { 

						btBufferInfoCL( bodyBuf->getBufferCL() ), 
						btBufferInfoCL( shapeBuf->getBufferCL() ), 
						btBufferInfoCL( constraint->getBufferCL() ),
						btBufferInfoCL( m_data->m_numConstraints->getBufferCL() ), 
						btBufferInfoCL( m_data->m_offsets->getBufferCL() ) 
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
		
			clFinish(m_data->m_queue);


		}

		cdata.x = 1;
		bool applyFriction=true;
		if (applyFriction)
    	{
			BT_PROFILE("m_batchSolveKernel iterations2");
			for(int iter=0; iter<numIterations; iter++)
			{
				for(int ib=0; ib<BT_SOLVER_N_BATCHES; ib++)
				{
					cdata.z = ib;
					cdata.w = BT_SOLVER_N_SPLIT;

					btBufferInfoCL bInfo[] = { 
						btBufferInfoCL( bodyBuf->getBufferCL() ), 
						btBufferInfoCL( shapeBuf->getBufferCL() ), 
						btBufferInfoCL( constraint->getBufferCL() ),
						btBufferInfoCL( m_data->m_numConstraints->getBufferCL() ), 
						btBufferInfoCL( m_data->m_offsets->getBufferCL() )
#ifdef DEBUG_ME
						,btBufferInfoCL(&gpuDebugInfo)
#endif //DEBUG_ME
					};
					btLauncherCL launcher( m_data->m_queue, m_data->m_solveFrictionKernel );
					launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(btBufferInfoCL) );
					//launcher.setConst(  cdata.x );
                    launcher.setConst(  cdata.y );
                    launcher.setConst(  cdata.z );
                    launcher.setConst(  cdata.w );
                    
					launcher.launch1D( 64*nn/BT_SOLVER_N_BATCHES, 64 );
				}
			}
			clFinish(m_data->m_queue);
			
		}
#ifdef DEBUG_ME
		delete[] debugInfo;
#endif //DEBUG_ME
	}

	
}














void b3GpuBatchingPgsSolver::solveContacts(int numBodies, cl_mem bodyBuf, cl_mem inertiaBuf, int numContacts, cl_mem contactBuf, const b3Config& config)
{
	m_data->m_bodyBufferGPU->setFromOpenCLBuffer(bodyBuf,numBodies);
	m_data->m_inertiaBufferGPU->setFromOpenCLBuffer(inertiaBuf,numBodies);
	m_data->m_pBufContactOutGPU->setFromOpenCLBuffer(contactBuf,numContacts);

	int nContactOut = m_data->m_pBufContactOutGPU->size();

	bool useSolver = true;
    
    if (useSolver)
    {
        float dt=1./60.;
        btConstraintCfg csCfg( dt );
        csCfg.m_enableParallelSolve = true;
        csCfg.m_averageExtent = .2f;//@TODO m_averageObjExtent;
        csCfg.m_staticIdx = 0;//m_static0Index;//m_planeBodyIndex;
        
        
        btOpenCLArray<b3RigidBodyCL>* bodyBuf = m_data->m_bodyBufferGPU;

        void* additionalData = 0;//m_data->m_frictionCGPU;
        const btOpenCLArray<btInertiaCL>* shapeBuf = m_data->m_inertiaBufferGPU;
        btOpenCLArray<b3GpuConstraint4>* contactConstraintOut = m_data->m_contactCGPU;
        int nContacts = nContactOut;
        
        
		int maxNumBatches = 0;
 
        {
            
            if( m_data->m_solverGPU->m_contactBuffer2)
            {
                m_data->m_solverGPU->m_contactBuffer2->resize(nContacts);
            }
            
            if( m_data->m_solverGPU->m_contactBuffer2 == 0 )
            {
				m_data->m_solverGPU->m_contactBuffer2 = new btOpenCLArray<b3Contact4>(m_data->m_context,m_data->m_queue, nContacts );
                m_data->m_solverGPU->m_contactBuffer2->resize(nContacts);
            }
			
            clFinish(m_data->m_queue);
            
            
            
            {
                BT_PROFILE("batching");
                //@todo: just reserve it, without copy of original contact (unless we use warmstarting)
                
                
                
                const btOpenCLArray<b3RigidBodyCL>* bodyNative = bodyBuf;
                
                
                {
                    
                    //btOpenCLArray<b3RigidBodyCL>* bodyNative = btOpenCLArrayUtils::map<adl::TYPE_CL, true>( data->m_device, bodyBuf );
                    //btOpenCLArray<b3Contact4>* contactNative = btOpenCLArrayUtils::map<adl::TYPE_CL, true>( data->m_device, contactsIn );
                    
                    const int sortAlignment = 512; // todo. get this out of sort
                    if( csCfg.m_enableParallelSolve )
                    {
                        
                        
                        int sortSize = BTNEXTMULTIPLEOF( nContacts, sortAlignment );
                        
                        btOpenCLArray<unsigned int>* countsNative = m_data->m_solverGPU->m_numConstraints;
                        btOpenCLArray<unsigned int>* offsetsNative = m_data->m_solverGPU->m_offsets;
                        
                        {	//	2. set cell idx
                            BT_PROFILE("GPU set cell idx");
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
                            cdata.m_staticIdx = csCfg.m_staticIdx;
                            cdata.m_scale = 1.f/(BT_SOLVER_N_OBJ_PER_SPLIT*csCfg.m_averageExtent);
                            cdata.m_nSplit = BT_SOLVER_N_SPLIT;
                            
                            m_data->m_solverGPU->m_sortDataBuffer->resize(nContacts);
                            
                            
                            btBufferInfoCL bInfo[] = { btBufferInfoCL( m_data->m_pBufContactOutGPU->getBufferCL() ), btBufferInfoCL( bodyBuf->getBufferCL()), btBufferInfoCL( m_data->m_solverGPU->m_sortDataBuffer->getBufferCL()) };
                            btLauncherCL launcher(m_data->m_queue, m_data->m_solverGPU->m_setSortDataKernel );
                            launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(btBufferInfoCL) );
                            launcher.setConst( cdata.m_nContacts );
                            launcher.setConst( cdata.m_scale );
                            launcher.setConst(cdata.m_nSplit);
                            
                            
                            launcher.launch1D( sortSize, 64 );
                        }
                        
                        
                        bool gpuRadixSort=true;
                        if (gpuRadixSort)
                        {	//	3. sort by cell idx
                            BT_PROFILE("gpuRadixSort");
                            int n = BT_SOLVER_N_SPLIT*BT_SOLVER_N_SPLIT;
                            int sortBit = 32;
                            //if( n <= 0xffff ) sortBit = 16;
                            //if( n <= 0xff ) sortBit = 8;
                            //adl::RadixSort<adl::TYPE_CL>::execute( data->m_sort, *data->m_sortDataBuffer, sortSize );
                            //adl::RadixSort32<adl::TYPE_CL>::execute( data->m_sort32, *data->m_sortDataBuffer, sortSize );
                            btOpenCLArray<btSortData>& keyValuesInOut = *(m_data->m_solverGPU->m_sortDataBuffer);
                            this->m_data->m_solverGPU->m_sort32->execute(keyValuesInOut);
                            
                            /*b3AlignedObjectArray<btSortData> hostValues;
                             keyValuesInOut.copyToHost(hostValues);
                             printf("hostValues.size=%d\n",hostValues.size());
                             */
                            
                        }
                        
                        {
                            //	4. find entries
                            BT_PROFILE("gpuBoundSearch");
                            
                            m_data->m_solverGPU->m_search->execute(*m_data->m_solverGPU->m_sortDataBuffer,nContacts,*countsNative,
                                                                           BT_SOLVER_N_SPLIT*BT_SOLVER_N_SPLIT,btBoundSearchCL::COUNT);
                            
                            
                            //adl::BoundSearch<adl::TYPE_CL>::execute( data->m_search, *data->m_sortDataBuffer, nContacts, *countsNative,
                            //	BT_SOLVER_N_SPLIT*BT_SOLVER_N_SPLIT, adl::BoundSearchBase::COUNT );
                            
                            //unsigned int sum;
                            m_data->m_solverGPU->m_scan->execute(*countsNative,*offsetsNative, BT_SOLVER_N_SPLIT*BT_SOLVER_N_SPLIT);//,&sum );
                            //printf("sum = %d\n",sum);
                        }
                        
                        
                        
                        
                        if (nContacts)
                        {	//	5. sort constraints by cellIdx
                            {
                                BT_PROFILE("gpu m_reorderContactKernel");
                                
                                btInt4 cdata;
                                cdata.x = nContacts;
                                
								btBufferInfoCL bInfo[] = { btBufferInfoCL( m_data->m_pBufContactOutGPU->getBufferCL() ), btBufferInfoCL( m_data->m_solverGPU->m_contactBuffer2->getBufferCL())
                                    , btBufferInfoCL( m_data->m_solverGPU->m_sortDataBuffer->getBufferCL()) };
                                btLauncherCL launcher(m_data->m_queue,m_data->m_solverGPU->m_reorderContactKernel);
                                launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(btBufferInfoCL) );
                                launcher.setConst( cdata );
                                launcher.launch1D( nContacts, 64 );
                            }
                        }
                        
                        
                        
                        
                    }
                    
                }
                
                clFinish(m_data->m_queue);
                
				
				if (nContacts)
				{
					BT_PROFILE("gpu m_copyConstraintKernel");
					btInt4 cdata; cdata.x = nContacts;
					btBufferInfoCL bInfo[] = { btBufferInfoCL(  m_data->m_solverGPU->m_contactBuffer2->getBufferCL() ), btBufferInfoCL( m_data->m_pBufContactOutGPU->getBufferCL() ) };
					btLauncherCL launcher(m_data->m_queue, m_data->m_solverGPU->m_copyConstraintKernel );
					launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(btBufferInfoCL) );
					launcher.setConst(  cdata );
					launcher.launch1D( nContacts, 64 );
					clFinish(m_data->m_queue);
				}
                
                
                bool compareGPU = false;
				if (nContacts)
				{
					if (gpuBatchContacts)
					{
						BT_PROFILE("gpu batchContacts");
						maxNumBatches = 25;//250;
						m_data->m_solverGPU->batchContacts( m_data->m_pBufContactOutGPU, nContacts, m_data->m_solverGPU->m_numConstraints, m_data->m_solverGPU->m_offsets, csCfg.m_staticIdx );
					} else
					{
						BT_PROFILE("cpu batchContacts");
						b3AlignedObjectArray<b3Contact4> cpuContacts;
						btOpenCLArray<b3Contact4>* contactsIn = m_data->m_solverGPU->m_contactBuffer2;
						contactsIn->copyToHost(cpuContacts);
                    
						btOpenCLArray<unsigned int>* countsNative = m_data->m_solverGPU->m_numConstraints;
						btOpenCLArray<unsigned int>* offsetsNative = m_data->m_solverGPU->m_offsets;
                    
						b3AlignedObjectArray<unsigned int> nNativeHost;
						b3AlignedObjectArray<unsigned int> offsetsNativeHost;
                    
						{
							BT_PROFILE("countsNative/offsetsNative copyToHost");
							countsNative->copyToHost(nNativeHost);
							offsetsNative->copyToHost(offsetsNativeHost);
						}
                    
                    
						int numNonzeroGrid=0;
                    
						{
							BT_PROFILE("batch grid");
							for(int i=0; i<BT_SOLVER_N_SPLIT*BT_SOLVER_N_SPLIT; i++)
							{
								int n = (nNativeHost)[i];
								int offset = (offsetsNativeHost)[i];
                            
								if( n )
								{
									numNonzeroGrid++;
									//printf("cpu batch\n");
                                
									
									int simdWidth =64;//-1;//32;
									int numBatches = sortConstraintByBatch3( &cpuContacts[0]+offset, n, simdWidth,csCfg.m_staticIdx ,numBodies);	//	on GPU
									

									maxNumBatches = btMax(numBatches,maxNumBatches);
                                
									clFinish(m_data->m_queue);
                                
								}
							}
						}
						{
							BT_PROFILE("m_contactBuffer->copyFromHost");
							m_data->m_solverGPU->m_contactBuffer2->copyFromHost((b3AlignedObjectArray<b3Contact4>&)cpuContacts);
						}
						
					} 
					
                }
                
                
                //printf("maxNumBatches = %d\n", maxNumBatches);
                
                if (nContacts)
                {
                    //BT_PROFILE("gpu convertToConstraints");
					m_data->m_solverGPU->convertToConstraints( bodyBuf, 
						shapeBuf, m_data->m_solverGPU->m_contactBuffer2,
						contactConstraintOut, 
						additionalData, nContacts, 
						(b3SolverBase::ConstraintCfg&) csCfg );
                    clFinish(m_data->m_queue);
                }
                
                
                
            } 
            
            
        } 
        
        
        if (1)
        {
            m_data->m_solverGPU->m_nIterations = 4;//10
			if (gpuSolveConstraint)
			{
				BT_PROFILE("GPU solveContactConstraint");

				m_data->m_solverGPU->solveContactConstraint(
					m_data->m_bodyBufferGPU, 
					m_data->m_inertiaBufferGPU,
					m_data->m_contactCGPU,0,
					nContactOut ,
					maxNumBatches);
			}
			else
			{
				BT_PROFILE("Host solveContactConstraint");

				m_data->m_solverGPU->solveContactConstraintHost(m_data->m_bodyBufferGPU, m_data->m_inertiaBufferGPU, m_data->m_contactCGPU,0, nContactOut ,maxNumBatches);
			}
            
            clFinish(m_data->m_queue);
        }
        
        
#if 0
        if (0)
        {
            BT_PROFILE("read body velocities back to CPU");
            //read body updated linear/angular velocities back to CPU
            m_data->m_bodyBufferGPU->read(
                                                  m_data->m_bodyBufferCPU->m_ptr,numOfConvexRBodies);
            adl::DeviceUtils::waitForCompletion( m_data->m_deviceCL );
        }
#endif
        
    }

}


void b3GpuBatchingPgsSolver::batchContacts( btOpenCLArray<b3Contact4>* contacts, int nContacts, btOpenCLArray<unsigned int>* n, btOpenCLArray<unsigned int>* offsets, int staticIdx )
{
}



static bool sortfnc(const btSortData& a,const btSortData& b)
{
	return (a.m_key<b.m_key);
}



b3AlignedObjectArray<int> bodyUsed;




b3AlignedObjectArray<unsigned int> idxBuffer;
b3AlignedObjectArray<btSortData> sortData;
b3AlignedObjectArray<b3Contact4> old;


inline int b3GpuBatchingPgsSolver::sortConstraintByBatch( b3Contact4* cs, int n, int simdWidth , int staticIdx, int numBodies)
{
	b3AlignedObjectArray<int> bodyUsed;
	bodyUsed.resize(numBodies);
	for (int q=0;q<numBodies;q++)
		bodyUsed[q]=0;

	BT_PROFILE("sortConstraintByBatch");
	int numIter = 0;
    
	sortData.resize(n);
	idxBuffer.resize(n);
	old.resize(n);
	
	unsigned int* idxSrc = &idxBuffer[0];
	unsigned int* idxDst = &idxBuffer[0];
	int nIdxSrc, nIdxDst;
    
	const int N_FLG = 256;
	const int FLG_MASK = N_FLG-1;
	unsigned int flg[N_FLG/32];
#if defined(_DEBUG)
	for(int i=0; i<n; i++)
		cs[i].getBatchIdx() = -1;
#endif
	for(int i=0; i<n; i++) idxSrc[i] = i;
	nIdxSrc = n;
    
	int batchIdx = 0;
    
	{
		BT_PROFILE("cpu batch innerloop");
		while( nIdxSrc )
		{
			numIter++;
			nIdxDst = 0;
			int nCurrentBatch = 0;
            
			//	clear flag
			for(int i=0; i<N_FLG/32; i++) flg[i] = 0;
            
			for(int i=0; i<nIdxSrc; i++)
			{
				int idx = idxSrc[i];
				btAssert( idx < n );
				//	check if it can go
				int bodyAS = cs[idx].m_bodyAPtrAndSignBit;
				int bodyBS = cs[idx].m_bodyBPtrAndSignBit;
                
				
                
				int bodyA = abs(bodyAS);
				int bodyB = abs(bodyBS);
                
				int aIdx = bodyA & FLG_MASK;
				int bIdx = bodyB & FLG_MASK;
                
				unsigned int aUnavailable = flg[ aIdx/32 ] & (1<<(aIdx&31));
				unsigned int bUnavailable = flg[ bIdx/32 ] & (1<<(bIdx&31));
                
				bool aIsStatic = (bodyAS<0) || bodyAS==staticIdx;
				bool bIsStatic = (bodyBS<0) || bodyBS==staticIdx;

                //use inv_mass!
				aUnavailable = !aIsStatic? aUnavailable:0;//
				bUnavailable = !bIsStatic? bUnavailable:0;
                
				if( aUnavailable==0 && bUnavailable==0 ) // ok
				{
					if (!!aIsStatic)
						flg[ aIdx/32 ] |= (1<<(aIdx&31));
					if (!bIsStatic)
						flg[ bIdx/32 ] |= (1<<(bIdx&31));

					cs[idx].getBatchIdx() = batchIdx;
					sortData[idx].m_key = batchIdx;
					sortData[idx].m_value = idx;
                    
					{
						nCurrentBatch++;
						if( nCurrentBatch == simdWidth )
						{
							nCurrentBatch = 0;
							for(int i=0; i<N_FLG/32; i++) flg[i] = 0;
						}
					}
				}
				else
				{
					idxDst[nIdxDst++] = idx;
				}
			}
			btSwap( idxSrc, idxDst );
			btSwap( nIdxSrc, nIdxDst );
			batchIdx ++;
		}
	}
	{
		BT_PROFILE("quickSort");
		sortData.quickSort(sortfnc);
	}
	
	
	{
        BT_PROFILE("reorder");
		//	reorder
		
		memcpy( &old[0], cs, sizeof(b3Contact4)*n);
		for(int i=0; i<n; i++)
		{
			int idx = sortData[i].m_value;
			cs[i] = old[idx];
		}
	}
    
	
#if defined(_DEBUG)
    //		debugPrintf( "nBatches: %d\n", batchIdx );
	for(int i=0; i<n; i++)
    {
        btAssert( cs[i].getBatchIdx() != -1 );
    }
#endif
	return batchIdx;
}


inline int b3GpuBatchingPgsSolver::sortConstraintByBatch2( b3Contact4* cs, int numConstraints, int simdWidth , int staticIdx, int numBodies)
{
	
	BT_PROFILE("sortConstraintByBatch");
	


	bodyUsed.resize(2*simdWidth);

	for (int q=0;q<2*simdWidth;q++)
		bodyUsed[q]=0;

	int curBodyUsed = 0;

	int numIter = 0;
    
	m_data->m_sortData.resize(numConstraints);
	m_data->m_idxBuffer.resize(numConstraints);
	m_data->m_old.resize(numConstraints);
	
	unsigned int* idxSrc = &m_data->m_idxBuffer[0];
		
#if defined(_DEBUG)
	for(int i=0; i<numConstraints; i++)
		cs[i].getBatchIdx() = -1;
#endif
	for(int i=0; i<numConstraints; i++) 
		idxSrc[i] = i;
    
	int numValidConstraints = 0;
	int unprocessedConstraintIndex = 0;

	int batchIdx = 0;
    

	{
		BT_PROFILE("cpu batch innerloop");
		
		while( numValidConstraints < numConstraints)
		{
			numIter++;
			int nCurrentBatch = 0;
			//	clear flag
			for(int i=0; i<curBodyUsed; i++) 
				bodyUsed[i] = 0;
            curBodyUsed = 0;

			for(int i=numValidConstraints; i<numConstraints; i++)
			{
				int idx = idxSrc[i];
				btAssert( idx < numConstraints );
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
					for (int j=0;j<curBodyUsed;j++)
					{
						if (bodyA == bodyUsed[j])
						{
							aUnavailable=1;
							break;
						}
					}
				}
				if (!aUnavailable)
				if (!bIsStatic)
				{
					for (int j=0;j<curBodyUsed;j++)
					{
						if (bodyB == bodyUsed[j])
						{
							bUnavailable=1;
							break;
						}
					}
				}
                
				if( aUnavailable==0 && bUnavailable==0 ) // ok
				{
					if (!aIsStatic)
					{
						bodyUsed[curBodyUsed++] = bodyA;
					}
					if (!bIsStatic)
					{
						bodyUsed[curBodyUsed++] = bodyB;
					}

					cs[idx].getBatchIdx() = batchIdx;
					m_data->m_sortData[idx].m_key = batchIdx;
					m_data->m_sortData[idx].m_value = idx;

					if (i!=numValidConstraints)
					{
						btSwap(idxSrc[i], idxSrc[numValidConstraints]);
					}

					numValidConstraints++;
					{
						nCurrentBatch++;
						if( nCurrentBatch == simdWidth )
						{
							nCurrentBatch = 0;
							for(int i=0; i<curBodyUsed; i++) 
								bodyUsed[i] = 0;

							
							curBodyUsed = 0;
						}
					}
				}
			}
			
			batchIdx ++;
		}
	}
	{
		BT_PROFILE("quickSort");
		//m_data->m_sortData.quickSort(sortfnc);
	}

	{
        BT_PROFILE("reorder");
		//	reorder
		
		memcpy( &m_data->m_old[0], cs, sizeof(b3Contact4)*numConstraints);

		for(int i=0; i<numConstraints; i++)
		{
			btAssert(m_data->m_sortData[idxSrc[i]].m_value == idxSrc[i]);
			int idx = m_data->m_sortData[idxSrc[i]].m_value;
			cs[i] = m_data->m_old[idx];
		}
	}
	
#if defined(_DEBUG)
    //		debugPrintf( "nBatches: %d\n", batchIdx );
	for(int i=0; i<numConstraints; i++)
    {
        btAssert( cs[i].getBatchIdx() != -1 );
    }
#endif

	
	return batchIdx;
}


inline int b3GpuBatchingPgsSolver::sortConstraintByBatch3( b3Contact4* cs, int numConstraints, int simdWidth , int staticIdx, int numBodies)
{
	
	BT_PROFILE("sortConstraintByBatch");
	
	static int maxSwaps = 0;
	int numSwaps = 0;

	static int maxNumConstraints = 0;
	if (maxNumConstraints<numConstraints)
	{
		maxNumConstraints = numConstraints;
		printf("maxNumConstraints  = %d\n",maxNumConstraints );
	}

	bodyUsed.resize(2*simdWidth);

	for (int q=0;q<2*simdWidth;q++)
		bodyUsed[q]=0;

	int curBodyUsed = 0;

	int numIter = 0;
    
	m_data->m_sortData.resize(0);
	m_data->m_idxBuffer.resize(0);
	m_data->m_old.resize(0);
	
		
#if defined(_DEBUG)
	for(int i=0; i<numConstraints; i++)
		cs[i].getBatchIdx() = -1;
#endif
	
	int numValidConstraints = 0;
	int unprocessedConstraintIndex = 0;

	int batchIdx = 0;
    

	{
		BT_PROFILE("cpu batch innerloop");
		
		while( numValidConstraints < numConstraints)
		{
			numIter++;
			int nCurrentBatch = 0;
			//	clear flag
			for(int i=0; i<curBodyUsed; i++) 
				bodyUsed[i] = 0;
            curBodyUsed = 0;

			for(int i=numValidConstraints; i<numConstraints; i++)
			{
				int idx = i;
				btAssert( idx < numConstraints );
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
					for (int j=0;j<curBodyUsed;j++)
					{
						if (bodyA == bodyUsed[j])
						{
							aUnavailable=1;
							break;
						}
					}
				}
				if (!aUnavailable)
				if (!bIsStatic)
				{
					for (int j=0;j<curBodyUsed;j++)
					{
						if (bodyB == bodyUsed[j])
						{
							bUnavailable=1;
							break;
						}
					}
				}
                
				if( aUnavailable==0 && bUnavailable==0 ) // ok
				{
					if (!aIsStatic)
					{
						bodyUsed[curBodyUsed++] = bodyA;
					}
					if (!bIsStatic)
					{
						bodyUsed[curBodyUsed++] = bodyB;
					}

					cs[idx].getBatchIdx() = batchIdx;

					if (i!=numValidConstraints)
					{
						btSwap(cs[i],cs[numValidConstraints]);
						numSwaps++;
					}

					numValidConstraints++;
					{
						nCurrentBatch++;
						if( nCurrentBatch == simdWidth )
						{
							nCurrentBatch = 0;
							for(int i=0; i<curBodyUsed; i++) 
								bodyUsed[i] = 0;
							curBodyUsed = 0;
						}
					}
				}
			}
			batchIdx ++;
		}
	}
	
#if defined(_DEBUG)
    //		debugPrintf( "nBatches: %d\n", batchIdx );
	for(int i=0; i<numConstraints; i++)
    {
        btAssert( cs[i].getBatchIdx() != -1 );
    }
#endif

	if (maxSwaps<numSwaps)
	{
		maxSwaps = numSwaps;
		printf("maxSwaps = %d\n", maxSwaps);
	}
	
	return batchIdx;
}
