

#include "btGpuBatchingPgsSolver.h"
#include "../../parallel_primitives/host/btRadixSort32CL.h"
#include "BulletCommon/btQuickprof.h"
#include "../../parallel_primitives/host/btLauncherCL.h"
#include "../../parallel_primitives/host/btBoundSearchCL.h"
#include "../../parallel_primitives/host/btPrefixScanCL.h"
#include <string.h>
#include "../../basic_initialize/btOpenCLUtils.h"
#include "../host/btConfig.h"
#include "../Stubs/Solver.h"


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


#define BTNEXTMULTIPLEOF(num, alignment) (((num)/(alignment) + (((num)%(alignment)==0)?0:1))*(alignment))
enum
{
	BT_SOLVER_N_SPLIT = 16,
	BT_SOLVER_N_BATCHES = 4,
	BT_SOLVER_N_OBJ_PER_SPLIT = 10,
	BT_SOLVER_N_TASKS_PER_BATCH = BT_SOLVER_N_SPLIT*BT_SOLVER_N_SPLIT,
};


bool gpuBatchContacts = true;
bool gpuSolveConstraint = true;


struct	btGpuBatchingPgsSolverInternalData
{
	cl_context m_context;
	cl_device_id m_device;
	cl_command_queue m_queue;
	int m_pairCapacity;
	int m_nIterations;

	btOpenCLArray<btGpuConstraint4>* m_contactCGPU;

	btOpenCLArray<unsigned int>* m_numConstraints;
	btOpenCLArray<unsigned int>* m_offsets;
		
	Solver*		m_solverGPU;		
	
	cl_kernel m_batchingKernel;
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
	btOpenCLArray<btContact4>* m_contactBuffer;

	btOpenCLArray<btRigidBodyCL>* m_bodyBufferGPU;
	btOpenCLArray<btInertiaCL>* m_inertiaBufferGPU;
	btOpenCLArray<btContact4>* m_pBufContactOutGPU;
};



btGpuBatchingPgsSolver::btGpuBatchingPgsSolver(cl_context ctx,cl_device_id device, cl_command_queue  q,int pairCapacity)
{
	m_data = new btGpuBatchingPgsSolverInternalData;
	m_data->m_context = ctx;
	m_data->m_device = device;
	m_data->m_queue = q;
	m_data->m_pairCapacity = pairCapacity;
	m_data->m_nIterations = 4;

	m_data->m_bodyBufferGPU = new btOpenCLArray<btRigidBodyCL>(ctx,q);
	m_data->m_inertiaBufferGPU = new btOpenCLArray<btInertiaCL>(ctx,q);
	m_data->m_pBufContactOutGPU = new btOpenCLArray<btContact4>(ctx,q);

	m_data->m_solverGPU = new Solver(ctx,device,q,512*1024);

	m_data->m_sort32 = new btRadixSort32CL(ctx,device,m_data->m_queue);
	m_data->m_scan = new btPrefixScanCL(ctx,device,m_data->m_queue,BT_SOLVER_N_SPLIT*BT_SOLVER_N_SPLIT);
	m_data->m_search = new btBoundSearchCL(ctx,device,m_data->m_queue,BT_SOLVER_N_SPLIT*BT_SOLVER_N_SPLIT);

	const int sortSize = BTNEXTMULTIPLEOF( pairCapacity, 512 );

	m_data->m_sortDataBuffer = new btOpenCLArray<btSortData>(ctx,m_data->m_queue,sortSize);
	m_data->m_contactBuffer = new btOpenCLArray<btContact4>(ctx,m_data->m_queue);

	m_data->m_numConstraints = new btOpenCLArray<unsigned int>(ctx,m_data->m_queue,BT_SOLVER_N_SPLIT*BT_SOLVER_N_SPLIT );
	m_data->m_numConstraints->resize(BT_SOLVER_N_SPLIT*BT_SOLVER_N_SPLIT);

	m_data->m_contactCGPU = new btOpenCLArray<btGpuConstraint4>(ctx,q,pairCapacity);

	m_data->m_offsets = new btOpenCLArray<unsigned int>( ctx,m_data->m_queue, BT_SOLVER_N_SPLIT*BT_SOLVER_N_SPLIT );
	m_data->m_offsets->resize(BT_SOLVER_N_SPLIT*BT_SOLVER_N_SPLIT);
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
		
		
		m_data->m_solveFrictionKernel= btOpenCLUtils::compileCLKernelFromString( ctx, device, solveFrictionSource, "BatchSolveKernelFriction", &pErrNum, solveFrictionProg,additionalMacros );
		btAssert(m_data->m_solveFrictionKernel);

		m_data->m_solveContactKernel= btOpenCLUtils::compileCLKernelFromString( ctx, device, solveContactSource, "BatchSolveKernelContact", &pErrNum, solveContactProg,additionalMacros );
		btAssert(m_data->m_solveContactKernel);
		
		m_data->m_contactToConstraintKernel = btOpenCLUtils::compileCLKernelFromString( ctx, device, solverSetupSource, "ContactToConstraintKernel", &pErrNum, solverSetupProg,additionalMacros );
		btAssert(m_data->m_contactToConstraintKernel);
			
		m_data->m_setSortDataKernel =  btOpenCLUtils::compileCLKernelFromString( ctx, device, solverSetup2Source, "SetSortDataKernel", &pErrNum, solverSetup2Prog,additionalMacros );
		btAssert(m_data->m_setSortDataKernel);
				
		m_data->m_reorderContactKernel = btOpenCLUtils::compileCLKernelFromString( ctx, device, solverSetup2Source, "ReorderContactKernel", &pErrNum, solverSetup2Prog,additionalMacros );
		btAssert(m_data->m_reorderContactKernel);
		

		m_data->m_copyConstraintKernel = btOpenCLUtils::compileCLKernelFromString( ctx, device, solverSetup2Source, "CopyConstraintKernel", &pErrNum, solverSetup2Prog,additionalMacros );
		btAssert(m_data->m_copyConstraintKernel);
		
	}

	{
		cl_program batchingProg = btOpenCLUtils::compileCLProgramFromString( ctx, device, batchKernelSource, &pErrNum,additionalMacros, BATCHING_PATH);
		btAssert(batchingProg);
		
		m_data->m_batchingKernel = btOpenCLUtils::compileCLKernelFromString( ctx, device, batchKernelSource, "CreateBatches", &pErrNum, batchingProg,additionalMacros );
		btAssert(m_data->m_batchingKernel);
	}
			







}

btGpuBatchingPgsSolver::~btGpuBatchingPgsSolver()
{
	delete m_data->m_sortDataBuffer;
	delete m_data->m_contactBuffer;

	delete m_data->m_sort32;
	delete m_data->m_scan;
	delete m_data->m_search;


	clReleaseKernel(m_data->m_batchingKernel);
	
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
	btConstraintCfg( float dt = 0.f ): m_positionDrift( 0.005f ), m_positionConstraintCoeff( 0.2f ), m_dt(dt), m_staticIdx(-1) {}

	float m_positionDrift;
	float m_positionConstraintCoeff;
	float m_dt;
	bool m_enableParallelSolve;
	float m_averageExtent;
	int m_staticIdx;
};





void btGpuBatchingPgsSolver::solveContactConstraint(  const btOpenCLArray<btRigidBodyCL>* bodyBuf, const btOpenCLArray<btInertiaCL>* shapeBuf, 
			btOpenCLArray<btGpuConstraint4>* constraint, void* additionalData, int n ,int maxNumBatches,int numIterations)
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














void btGpuBatchingPgsSolver::solveContacts(int numBodies, cl_mem bodyBuf, cl_mem inertiaBuf, int numContacts, cl_mem contactBuf, const btConfig& config)
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
        csCfg.m_averageExtent = 0.2f;//@TODO m_averageObjExtent;
        csCfg.m_staticIdx = -1;//m_static0Index;//m_planeBodyIndex;
        
        btOpenCLArray<btContact4>* contactsIn = m_data->m_pBufContactOutGPU;
        btOpenCLArray<btRigidBodyCL>* bodyBuf = m_data->m_bodyBufferGPU;

        void* additionalData = 0;//m_data->m_frictionCGPU;
        const btOpenCLArray<btInertiaCL>* shapeBuf = m_data->m_inertiaBufferGPU;
        btOpenCLArray<btGpuConstraint4>* contactConstraintOut = m_data->m_contactCGPU;
        int nContacts = nContactOut;
        
        
		int maxNumBatches = 0;
 
        {
            
            if( m_data->m_solverGPU->m_contactBuffer)
            {
                m_data->m_solverGPU->m_contactBuffer->resize(nContacts);
            }
            
            if( m_data->m_solverGPU->m_contactBuffer == 0 )
            {
				m_data->m_solverGPU->m_contactBuffer = new btOpenCLArray<btContact4>(m_data->m_context,m_data->m_queue, nContacts );
                m_data->m_solverGPU->m_contactBuffer->resize(nContacts);
            }
            clFinish(m_data->m_queue);
            
            
            
            {
                BT_PROFILE("batching");
                //@todo: just reserve it, without copy of original contact (unless we use warmstarting)
                
                
                btOpenCLArray<btContact4>* contactNative  = contactsIn;
                const btOpenCLArray<btRigidBodyCL>* bodyNative = bodyBuf;
                
                
                {
                    
                    //btOpenCLArray<btRigidBodyCL>* bodyNative = btOpenCLArrayUtils::map<adl::TYPE_CL, true>( data->m_device, bodyBuf );
                    //btOpenCLArray<btContact4>* contactNative = btOpenCLArrayUtils::map<adl::TYPE_CL, true>( data->m_device, contactsIn );
                    
                    const int sortAlignment = 512; // todo. get this out of sort
                    if( csCfg.m_enableParallelSolve )
                    {
                        
                        
                        int sortSize = NEXTMULTIPLEOF( nContacts, sortAlignment );
                        
                        btOpenCLArray<u32>* countsNative = m_data->m_solverGPU->m_numConstraints;
                        btOpenCLArray<u32>* offsetsNative = m_data->m_solverGPU->m_offsets;
                        
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
                            
                            
                            btBufferInfoCL bInfo[] = { btBufferInfoCL( contactNative->getBufferCL() ), btBufferInfoCL( bodyBuf->getBufferCL()), btBufferInfoCL( m_data->m_solverGPU->m_sortDataBuffer->getBufferCL()) };
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
                            
                            /*btAlignedObjectArray<btSortData> hostValues;
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
                                
                                btBufferInfoCL bInfo[] = { btBufferInfoCL( contactNative->getBufferCL() ), btBufferInfoCL( m_data->m_solverGPU->m_contactBuffer->getBufferCL())
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
                    btBufferInfoCL bInfo[] = { btBufferInfoCL(  m_data->m_solverGPU->m_contactBuffer->getBufferCL() ), btBufferInfoCL( contactNative->getBufferCL() ) };
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
					maxNumBatches=250;//for now
                    BT_PROFILE("gpu batchContacts");
                    m_data->m_solverGPU->batchContacts( (btOpenCLArray<btContact4>*)contactNative, nContacts, m_data->m_solverGPU->m_numConstraints, m_data->m_solverGPU->m_offsets, csCfg.m_staticIdx );
                } else
                {
                    BT_PROFILE("cpu batchContacts");
                    btAlignedObjectArray<btContact4> cpuContacts;
                    btOpenCLArray<btContact4>* contactsIn = m_data->m_pBufContactOutGPU;
                    contactsIn->copyToHost(cpuContacts);
                    
                    btOpenCLArray<u32>* countsNative = m_data->m_solverGPU->m_numConstraints;
                    btOpenCLArray<u32>* offsetsNative = m_data->m_solverGPU->m_offsets;
                    
                    btAlignedObjectArray<u32> nNativeHost;
                    btAlignedObjectArray<u32> offsetsNativeHost;
                    
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
                                
                                int simdWidth = -1;
                                int numBatches = sortConstraintByBatch( &cpuContacts[0]+offset, n, simdWidth,csCfg.m_staticIdx );	//	on GPU
                                maxNumBatches = btMax(numBatches,maxNumBatches);
                                
                                clFinish(m_data->m_queue);
                                
                            }
                        }
                    }
                    {
                        BT_PROFILE("m_contactBuffer->copyFromHost");
                        m_data->m_solverGPU->m_contactBuffer->copyFromHost((btAlignedObjectArray<btContact4>&)cpuContacts);
                    }
                  //  printf("maxNumBatches = %d\n", maxNumBatches);
				} 
                }
                
                
                
                
                if (nContacts)
                {
                    //BT_PROFILE("gpu convertToConstraints");
					m_data->m_solverGPU->convertToConstraints( bodyBuf, 
						shapeBuf, m_data->m_solverGPU->m_contactBuffer /*contactNative*/, 
						contactConstraintOut, 
						additionalData, nContacts, 
						(SolverBase::ConstraintCfg&) csCfg );
                    clFinish(m_data->m_queue);
                }
                
                
                
            } 
            
            
        } 
        
        
        if (1)
        {
            BT_PROFILE("GPU solveContactConstraint");
            m_data->m_solverGPU->m_nIterations = 4;//10
			if (gpuSolveConstraint)
			{
				m_data->m_solverGPU->solveContactConstraint(
					m_data->m_bodyBufferGPU, 
					m_data->m_inertiaBufferGPU,
					m_data->m_contactCGPU,0,
					nContactOut ,
					maxNumBatches);
			}
			else
			{
				//m_data->m_solverGPU->solveContactConstraintHost(m_data->m_bodyBufferGPU, m_data->m_inertiaBufferGPU, m_data->m_contactCGPU,0, nContactOut ,maxNumBatches);
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


void btGpuBatchingPgsSolver::batchContacts( btOpenCLArray<btContact4>* contacts, int nContacts, btOpenCLArray<unsigned int>* n, btOpenCLArray<unsigned int>* offsets, int staticIdx )
{
}



static bool sortfnc(const btSortData& a,const btSortData& b)
{
	return (a.m_key<b.m_key);
}

btAlignedObjectArray<unsigned int> idxBuffer;
btAlignedObjectArray<btSortData> sortData;
btAlignedObjectArray<btContact4> old;


inline int btGpuBatchingPgsSolver::sortConstraintByBatch( btContact4* cs, int n, int simdWidth , int staticIdx)
{
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
                
				/*if (bodyAS<0)
					printf("A static\n");
                
				if (bodyBS<0)
					printf("B static\n");
                */
                
				int bodyA = abs(bodyAS);
				int bodyB = abs(bodyBS);
                
				int aIdx = bodyA & FLG_MASK;
				int bIdx = bodyB & FLG_MASK;
                
				unsigned int aUnavailable = flg[ aIdx/32 ] & (1<<(aIdx&31));
				unsigned int bUnavailable = flg[ bIdx/32 ] & (1<<(bIdx&31));
                
                //use inv_mass!
				aUnavailable = (bodyAS>=0)&&bodyAS!=staticIdx? aUnavailable:0;//
				bUnavailable = (bodyBS>=0)&&bodyBS!=staticIdx? bUnavailable:0;
                
				if( aUnavailable==0 && bUnavailable==0 ) // ok
				{
					flg[ aIdx/32 ] |= (1<<(aIdx&31));
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
		
		memcpy( &old[0], cs, sizeof(btContact4)*n);
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
