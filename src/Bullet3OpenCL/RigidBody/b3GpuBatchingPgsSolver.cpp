
bool b3GpuBatchContacts = true;
bool b3GpuSolveConstraint = true;


#include "b3GpuBatchingPgsSolver.h"
#include "Bullet3OpenCL/ParallelPrimitives/b3RadixSort32CL.h"

#include "Bullet3OpenCL/ParallelPrimitives/b3LauncherCL.h"
#include "Bullet3OpenCL/ParallelPrimitives/b3BoundSearchCL.h"
#include "Bullet3OpenCL/ParallelPrimitives/b3PrefixScanCL.h"
#include <string.h>
#include "Bullet3OpenCL/Initialize/b3OpenCLUtils.h"
#include "b3Config.h"
#include "b3Solver.h"


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



enum
{
	B3_SOLVER_N_SPLIT = 16,
	B3_SOLVER_N_BATCHES = 4,
	B3_SOLVER_N_OBJ_PER_SPLIT = 10,
	B3_SOLVER_N_TASKS_PER_BATCH = B3_SOLVER_N_SPLIT*B3_SOLVER_N_SPLIT,
};




struct	b3GpuBatchingPgsSolverInternalData
{
	cl_context m_context;
	cl_device_id m_device;
	cl_command_queue m_queue;
	int m_pairCapacity;
	int m_nIterations;

	b3OpenCLArray<b3GpuConstraint4>* m_contactCGPU;
	b3OpenCLArray<unsigned int>* m_numConstraints;
	b3OpenCLArray<unsigned int>* m_offsets;
		
	b3Solver*		m_solverGPU;		
	
	cl_kernel m_batchingKernel;
	cl_kernel m_batchingKernelNew;
	cl_kernel m_solveContactKernel;
	cl_kernel m_solveFrictionKernel;
	cl_kernel m_contactToConstraintKernel;
	cl_kernel m_setSortDataKernel;
	cl_kernel m_reorderContactKernel;
	cl_kernel m_copyConstraintKernel;

	class b3RadixSort32CL*	m_sort32;
	class b3BoundSearchCL*	m_search;
	class b3PrefixScanCL*	m_scan;

	b3OpenCLArray<b3SortData>* m_sortDataBuffer;
	b3OpenCLArray<b3Contact4>* m_contactBuffer;

	b3OpenCLArray<b3RigidBodyCL>* m_bodyBufferGPU;
	b3OpenCLArray<b3InertiaCL>* m_inertiaBufferGPU;
	b3OpenCLArray<b3Contact4>* m_pBufContactOutGPU;


	b3AlignedObjectArray<unsigned int> m_idxBuffer;
	b3AlignedObjectArray<b3SortData> m_sortData;
	b3AlignedObjectArray<b3Contact4> m_old;
};



b3GpuBatchingPgsSolver::b3GpuBatchingPgsSolver(cl_context ctx,cl_device_id device, cl_command_queue  q,int pairCapacity)
{
	m_data = new b3GpuBatchingPgsSolverInternalData;
	m_data->m_context = ctx;
	m_data->m_device = device;
	m_data->m_queue = q;
	m_data->m_pairCapacity = pairCapacity;
	m_data->m_nIterations = 4;

	m_data->m_bodyBufferGPU = new b3OpenCLArray<b3RigidBodyCL>(ctx,q);
	m_data->m_inertiaBufferGPU = new b3OpenCLArray<b3InertiaCL>(ctx,q);
	m_data->m_pBufContactOutGPU = new b3OpenCLArray<b3Contact4>(ctx,q);

	m_data->m_solverGPU = new b3Solver(ctx,device,q,512*1024);

	m_data->m_sort32 = new b3RadixSort32CL(ctx,device,m_data->m_queue);
	m_data->m_scan = new b3PrefixScanCL(ctx,device,m_data->m_queue,B3_SOLVER_N_SPLIT*B3_SOLVER_N_SPLIT);
	m_data->m_search = new b3BoundSearchCL(ctx,device,m_data->m_queue,B3_SOLVER_N_SPLIT*B3_SOLVER_N_SPLIT);

	const int sortSize = B3NEXTMULTIPLEOF( pairCapacity, 512 );

	m_data->m_sortDataBuffer = new b3OpenCLArray<b3SortData>(ctx,m_data->m_queue,sortSize);
	m_data->m_contactBuffer = new b3OpenCLArray<b3Contact4>(ctx,m_data->m_queue);

	m_data->m_numConstraints = new b3OpenCLArray<unsigned int>(ctx,m_data->m_queue,B3_SOLVER_N_SPLIT*B3_SOLVER_N_SPLIT );
	m_data->m_numConstraints->resize(B3_SOLVER_N_SPLIT*B3_SOLVER_N_SPLIT);

	m_data->m_contactCGPU = new b3OpenCLArray<b3GpuConstraint4>(ctx,q,pairCapacity);

	m_data->m_offsets = new b3OpenCLArray<unsigned int>( ctx,m_data->m_queue, B3_SOLVER_N_SPLIT*B3_SOLVER_N_SPLIT );
	m_data->m_offsets->resize(B3_SOLVER_N_SPLIT*B3_SOLVER_N_SPLIT);
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
		
		
		m_data->m_solveFrictionKernel= b3OpenCLUtils::compileCLKernelFromString( ctx, device, solveFrictionSource, "BatchSolveKernelFriction", &pErrNum, solveFrictionProg,additionalMacros );
		b3Assert(m_data->m_solveFrictionKernel);

		m_data->m_solveContactKernel= b3OpenCLUtils::compileCLKernelFromString( ctx, device, solveContactSource, "BatchSolveKernelContact", &pErrNum, solveContactProg,additionalMacros );
		b3Assert(m_data->m_solveContactKernel);
		
		m_data->m_contactToConstraintKernel = b3OpenCLUtils::compileCLKernelFromString( ctx, device, solverSetupSource, "ContactToConstraintKernel", &pErrNum, solverSetupProg,additionalMacros );
		b3Assert(m_data->m_contactToConstraintKernel);
			
		m_data->m_setSortDataKernel =  b3OpenCLUtils::compileCLKernelFromString( ctx, device, solverSetup2Source, "SetSortDataKernel", &pErrNum, solverSetup2Prog,additionalMacros );
		b3Assert(m_data->m_setSortDataKernel);
				
		m_data->m_reorderContactKernel = b3OpenCLUtils::compileCLKernelFromString( ctx, device, solverSetup2Source, "ReorderContactKernel", &pErrNum, solverSetup2Prog,additionalMacros );
		b3Assert(m_data->m_reorderContactKernel);
		

		m_data->m_copyConstraintKernel = b3OpenCLUtils::compileCLKernelFromString( ctx, device, solverSetup2Source, "CopyConstraintKernel", &pErrNum, solverSetup2Prog,additionalMacros );
		b3Assert(m_data->m_copyConstraintKernel);
		
	}

	{
		cl_program batchingProg = b3OpenCLUtils::compileCLProgramFromString( ctx, device, batchKernelSource, &pErrNum,additionalMacros, B3_BATCHING_PATH);
		b3Assert(batchingProg);
		
		m_data->m_batchingKernel = b3OpenCLUtils::compileCLKernelFromString( ctx, device, batchKernelSource, "CreateBatches", &pErrNum, batchingProg,additionalMacros );
		b3Assert(m_data->m_batchingKernel);
	}
			
	{
		cl_program batchingNewProg = b3OpenCLUtils::compileCLProgramFromString( ctx, device, batchKernelNewSource, &pErrNum,additionalMacros, B3_BATCHING_NEW_PATH);
		b3Assert(batchingNewProg);
		
		m_data->m_batchingKernelNew = b3OpenCLUtils::compileCLKernelFromString( ctx, device, batchKernelNewSource, "CreateBatchesNew", &pErrNum, batchingNewProg,additionalMacros );
		b3Assert(m_data->m_batchingKernelNew);
	}
		






}

b3GpuBatchingPgsSolver::~b3GpuBatchingPgsSolver()
{
	delete m_data->m_bodyBufferGPU;
	delete m_data->m_inertiaBufferGPU;
	delete m_data->m_pBufContactOutGPU;

	delete m_data->m_contactCGPU;
	delete m_data->m_numConstraints;
	delete m_data->m_offsets;
	delete m_data->m_sortDataBuffer;
	delete m_data->m_contactBuffer;

	delete m_data->m_sort32;
	delete m_data->m_scan;
	delete m_data->m_search;
	delete m_data->m_solverGPU;

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



struct b3ConstraintCfg
{
	b3ConstraintCfg( float dt = 0.f ): m_positionDrift( 0.005f ), m_positionConstraintCoeff( 0.2f ), m_dt(dt), m_staticIdx(0) {}

	float m_positionDrift;
	float m_positionConstraintCoeff;
	float m_dt;
	bool m_enableParallelSolve;
	float m_averageExtent;
	int m_staticIdx;
};





void b3GpuBatchingPgsSolver::solveContactConstraint(  const b3OpenCLArray<b3RigidBodyCL>* bodyBuf, const b3OpenCLArray<b3InertiaCL>* shapeBuf, 
			b3OpenCLArray<b3GpuConstraint4>* constraint, void* additionalData, int n ,int maxNumBatches,int numIterations)
{
	
	
	b3Int4 cdata = b3MakeInt4( n, 0, 0, 0 );
	{
		
		const int nn = B3_SOLVER_N_SPLIT*B3_SOLVER_N_SPLIT;

		cdata.x = 0;
		cdata.y = maxNumBatches;//250;


		int numWorkItems = 64*nn/B3_SOLVER_N_BATCHES;
#ifdef DEBUG_ME
		SolverDebugInfo* debugInfo = new  SolverDebugInfo[numWorkItems];
		adl::b3OpenCLArray<SolverDebugInfo> gpuDebugInfo(data->m_device,numWorkItems);
#endif



		{

			B3_PROFILE("m_batchSolveKernel iterations");
			for(int iter=0; iter<numIterations; iter++)
			{
				for(int ib=0; ib<B3_SOLVER_N_BATCHES; ib++)
				{
#ifdef DEBUG_ME
					memset(debugInfo,0,sizeof(SolverDebugInfo)*numWorkItems);
					gpuDebugInfo.write(debugInfo,numWorkItems);
#endif


					cdata.z = ib;
					cdata.w = B3_SOLVER_N_SPLIT;

				b3LauncherCL launcher( m_data->m_queue, m_data->m_solveContactKernel );
#if 1
                    
					b3BufferInfoCL bInfo[] = { 

						b3BufferInfoCL( bodyBuf->getBufferCL() ), 
						b3BufferInfoCL( shapeBuf->getBufferCL() ), 
						b3BufferInfoCL( constraint->getBufferCL() ),
						b3BufferInfoCL( m_data->m_numConstraints->getBufferCL() ), 
						b3BufferInfoCL( m_data->m_offsets->getBufferCL() ) 
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
		
			clFinish(m_data->m_queue);


		}

		cdata.x = 1;
		bool applyFriction=true;
		if (applyFriction)
    	{
			B3_PROFILE("m_batchSolveKernel iterations2");
			for(int iter=0; iter<numIterations; iter++)
			{
				for(int ib=0; ib<B3_SOLVER_N_BATCHES; ib++)
				{
					cdata.z = ib;
					cdata.w = B3_SOLVER_N_SPLIT;

					b3BufferInfoCL bInfo[] = { 
						b3BufferInfoCL( bodyBuf->getBufferCL() ), 
						b3BufferInfoCL( shapeBuf->getBufferCL() ), 
						b3BufferInfoCL( constraint->getBufferCL() ),
						b3BufferInfoCL( m_data->m_numConstraints->getBufferCL() ), 
						b3BufferInfoCL( m_data->m_offsets->getBufferCL() )
#ifdef DEBUG_ME
						,b3BufferInfoCL(&gpuDebugInfo)
#endif //DEBUG_ME
					};
					b3LauncherCL launcher( m_data->m_queue, m_data->m_solveFrictionKernel );
					launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(b3BufferInfoCL) );
					//launcher.setConst(  cdata.x );
                    launcher.setConst(  cdata.y );
                    launcher.setConst(  cdata.z );
                    launcher.setConst(  cdata.w );
                    
					launcher.launch1D( 64*nn/B3_SOLVER_N_BATCHES, 64 );
				}
			}
			clFinish(m_data->m_queue);
			
		}
#ifdef DEBUG_ME
		delete[] debugInfo;
#endif //DEBUG_ME
	}

	
}











static bool sortfnc(const b3SortData& a,const b3SortData& b)
{
	return (a.m_key<b.m_key);
}



void b3GpuBatchingPgsSolver::solveContacts(int numBodies, cl_mem bodyBuf, cl_mem inertiaBuf, int numContacts, cl_mem contactBuf, const b3Config& config, int static0Index)
{
	m_data->m_bodyBufferGPU->setFromOpenCLBuffer(bodyBuf,numBodies);
	m_data->m_inertiaBufferGPU->setFromOpenCLBuffer(inertiaBuf,numBodies);
	m_data->m_pBufContactOutGPU->setFromOpenCLBuffer(contactBuf,numContacts);

	int nContactOut = m_data->m_pBufContactOutGPU->size();

	bool useSolver = true;
    
    if (useSolver)
    {
        float dt=1./60.;
        b3ConstraintCfg csCfg( dt );
        csCfg.m_enableParallelSolve = true;
        csCfg.m_averageExtent = 0.3;//0.1;//2;//.2f;//@TODO m_averageObjExtent;
        csCfg.m_staticIdx = static0Index;
        
        
        b3OpenCLArray<b3RigidBodyCL>* bodyBuf = m_data->m_bodyBufferGPU;

        void* additionalData = 0;//m_data->m_frictionCGPU;
        const b3OpenCLArray<b3InertiaCL>* shapeBuf = m_data->m_inertiaBufferGPU;
        b3OpenCLArray<b3GpuConstraint4>* contactConstraintOut = m_data->m_contactCGPU;
        int nContacts = nContactOut;
        
        
		int maxNumBatches = 0;
 
        {
            
            if( m_data->m_solverGPU->m_contactBuffer2)
            {
                m_data->m_solverGPU->m_contactBuffer2->resize(nContacts);
            }
            
            if( m_data->m_solverGPU->m_contactBuffer2 == 0 )
            {
				m_data->m_solverGPU->m_contactBuffer2 = new b3OpenCLArray<b3Contact4>(m_data->m_context,m_data->m_queue, nContacts );
                m_data->m_solverGPU->m_contactBuffer2->resize(nContacts);
            }
			
            clFinish(m_data->m_queue);
            
            
            
            {
                B3_PROFILE("batching");
                //@todo: just reserve it, without copy of original contact (unless we use warmstarting)
                
                
                
                const b3OpenCLArray<b3RigidBodyCL>* bodyNative = bodyBuf;
                
                
                {
                    
                    //b3OpenCLArray<b3RigidBodyCL>* bodyNative = b3OpenCLArrayUtils::map<adl::TYPE_CL, true>( data->m_device, bodyBuf );
                    //b3OpenCLArray<b3Contact4>* contactNative = b3OpenCLArrayUtils::map<adl::TYPE_CL, true>( data->m_device, contactsIn );
                    
                    const int sortAlignment = 512; // todo. get this out of sort
                    if( csCfg.m_enableParallelSolve )
                    {
                        
                        
                        int sortSize = B3NEXTMULTIPLEOF( nContacts, sortAlignment );
                        
                        b3OpenCLArray<unsigned int>* countsNative = m_data->m_solverGPU->m_numConstraints;
                        b3OpenCLArray<unsigned int>* offsetsNative = m_data->m_solverGPU->m_offsets;
                        
                        {	//	2. set cell idx
                            B3_PROFILE("GPU set cell idx");
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
                            cdata.m_staticIdx = csCfg.m_staticIdx;
                            cdata.m_scale = 1.f/(B3_SOLVER_N_OBJ_PER_SPLIT*csCfg.m_averageExtent);
                            cdata.m_nSplit = B3_SOLVER_N_SPLIT;
                            
                            m_data->m_solverGPU->m_sortDataBuffer->resize(nContacts);
                            
                            
                            b3BufferInfoCL bInfo[] = { b3BufferInfoCL( m_data->m_pBufContactOutGPU->getBufferCL() ), b3BufferInfoCL( bodyBuf->getBufferCL()), b3BufferInfoCL( m_data->m_solverGPU->m_sortDataBuffer->getBufferCL()) };
                            b3LauncherCL launcher(m_data->m_queue, m_data->m_solverGPU->m_setSortDataKernel );
                            launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(b3BufferInfoCL) );
                            launcher.setConst( cdata.m_nContacts );
                            launcher.setConst( cdata.m_scale );
                            launcher.setConst(cdata.m_nSplit);
							launcher.setConst(cdata.m_staticIdx);
                            
                            
                            launcher.launch1D( sortSize, 64 );
                        }
                        
                        
                        bool gpuRadixSort=true;
                        if (gpuRadixSort)
                        {	//	3. sort by cell idx
                            B3_PROFILE("gpuRadixSort");
                            //int n = B3_SOLVER_N_SPLIT*B3_SOLVER_N_SPLIT;
                            //int sortBit = 32;
                            //if( n <= 0xffff ) sortBit = 16;
                            //if( n <= 0xff ) sortBit = 8;
                            //adl::RadixSort<adl::TYPE_CL>::execute( data->m_sort, *data->m_sortDataBuffer, sortSize );
                            //adl::RadixSort32<adl::TYPE_CL>::execute( data->m_sort32, *data->m_sortDataBuffer, sortSize );
                            b3OpenCLArray<b3SortData>& keyValuesInOut = *(m_data->m_solverGPU->m_sortDataBuffer);
                            this->m_data->m_solverGPU->m_sort32->execute(keyValuesInOut);
                            
                           
                            
                        } else
						{
							b3OpenCLArray<b3SortData>& keyValuesInOut = *(m_data->m_solverGPU->m_sortDataBuffer);
							 b3AlignedObjectArray<b3SortData> hostValues;
                             keyValuesInOut.copyToHost(hostValues);
                             hostValues.quickSort(sortfnc);
							 keyValuesInOut.copyFromHost(hostValues);
						}
                        
                        {
                            //	4. find entries
                            B3_PROFILE("gpuBoundSearch");
                            
                            m_data->m_solverGPU->m_search->execute(*m_data->m_solverGPU->m_sortDataBuffer,nContacts,*countsNative,
                                                                           B3_SOLVER_N_SPLIT*B3_SOLVER_N_SPLIT,b3BoundSearchCL::COUNT);
                            
                            
                            //adl::BoundSearch<adl::TYPE_CL>::execute( data->m_search, *data->m_sortDataBuffer, nContacts, *countsNative,
                            //	B3_SOLVER_N_SPLIT*B3_SOLVER_N_SPLIT, adl::BoundSearchBase::COUNT );
                            
                            //unsigned int sum;
                            m_data->m_solverGPU->m_scan->execute(*countsNative,*offsetsNative, B3_SOLVER_N_SPLIT*B3_SOLVER_N_SPLIT);//,&sum );
                            //printf("sum = %d\n",sum);
                        }
                        
                        
                        
                        
                        if (nContacts)
                        {	//	5. sort constraints by cellIdx
                            {
                                B3_PROFILE("gpu m_reorderContactKernel");
                                
                                b3Int4 cdata;
                                cdata.x = nContacts;
                                
								b3BufferInfoCL bInfo[] = { b3BufferInfoCL( m_data->m_pBufContactOutGPU->getBufferCL() ), b3BufferInfoCL( m_data->m_solverGPU->m_contactBuffer2->getBufferCL())
                                    , b3BufferInfoCL( m_data->m_solverGPU->m_sortDataBuffer->getBufferCL()) };
                                b3LauncherCL launcher(m_data->m_queue,m_data->m_solverGPU->m_reorderContactKernel);
                                launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(b3BufferInfoCL) );
                                launcher.setConst( cdata );
                                launcher.launch1D( nContacts, 64 );
                            }
                        }
                        
                        
                        
                        
                    }
                    
                }
                
                clFinish(m_data->m_queue);
                
//				{
//				b3AlignedObjectArray<unsigned int> histogram;
//				m_data->m_solverGPU->m_numConstraints->copyToHost(histogram);
//				printf(",,,\n");
//				}
							
				
				if (nContacts)
				{
					B3_PROFILE("gpu m_copyConstraintKernel");
					b3Int4 cdata; cdata.x = nContacts;
					b3BufferInfoCL bInfo[] = { b3BufferInfoCL(  m_data->m_solverGPU->m_contactBuffer2->getBufferCL() ), b3BufferInfoCL( m_data->m_pBufContactOutGPU->getBufferCL() ) };
					b3LauncherCL launcher(m_data->m_queue, m_data->m_solverGPU->m_copyConstraintKernel );
					launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(b3BufferInfoCL) );
					launcher.setConst(  cdata );
					launcher.launch1D( nContacts, 64 );
					clFinish(m_data->m_queue);
				}
                
                
                bool compareGPU = false;
				if (nContacts)
				{
					if (b3GpuBatchContacts)
					{
						B3_PROFILE("gpu batchContacts");
						maxNumBatches = 50;//250;
						m_data->m_solverGPU->batchContacts( m_data->m_pBufContactOutGPU, nContacts, m_data->m_solverGPU->m_numConstraints, m_data->m_solverGPU->m_offsets, csCfg.m_staticIdx );
					} else
					{
						B3_PROFILE("cpu batchContacts");
						b3AlignedObjectArray<b3Contact4> cpuContacts;
						b3OpenCLArray<b3Contact4>* contactsIn = m_data->m_solverGPU->m_contactBuffer2;
						contactsIn->copyToHost(cpuContacts);
                    
						b3OpenCLArray<unsigned int>* countsNative = m_data->m_solverGPU->m_numConstraints;
						b3OpenCLArray<unsigned int>* offsetsNative = m_data->m_solverGPU->m_offsets;
                    
						b3AlignedObjectArray<unsigned int> nNativeHost;
						b3AlignedObjectArray<unsigned int> offsetsNativeHost;
                    
						{
							B3_PROFILE("countsNative/offsetsNative copyToHost");
							countsNative->copyToHost(nNativeHost);
							offsetsNative->copyToHost(offsetsNativeHost);
						}
                    
                    
						int numNonzeroGrid=0;
                    
						{
							B3_PROFILE("batch grid");
							for(int i=0; i<B3_SOLVER_N_SPLIT*B3_SOLVER_N_SPLIT; i++)
							{
								int n = (nNativeHost)[i];
								int offset = (offsetsNativeHost)[i];
                            
								if( n )
								{
									numNonzeroGrid++;
									//printf("cpu batch\n");
                                
									
									int simdWidth =64;//-1;//32;
									//int numBatches = sortConstraintByBatch( &cpuContacts[0]+offset, n, simdWidth,csCfg.m_staticIdx ,numBodies);	//	on GPU
									int numBatches = sortConstraintByBatch2( &cpuContacts[0]+offset, n, simdWidth,csCfg.m_staticIdx ,numBodies);	//	on GPU
									//int numBatches = sortConstraintByBatch3( &cpuContacts[0]+offset, n, simdWidth,csCfg.m_staticIdx ,numBodies);	//	on GPU
									
									

									maxNumBatches = b3Max(numBatches,maxNumBatches);
									static int globalMaxBatch = 0;
									if (maxNumBatches>globalMaxBatch )
									{
										globalMaxBatch  = maxNumBatches;
										b3Printf("maxNumBatches = %d\n",maxNumBatches);
									}
                                
									clFinish(m_data->m_queue);
                                
								}
							}
						}
						{
							B3_PROFILE("m_contactBuffer->copyFromHost");
							m_data->m_solverGPU->m_contactBuffer2->copyFromHost((b3AlignedObjectArray<b3Contact4>&)cpuContacts);
						}
						
					} 
					
                }
                
                
                //printf("maxNumBatches = %d\n", maxNumBatches);
                
                if (nContacts)
                {
                    //B3_PROFILE("gpu convertToConstraints");
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
			if (b3GpuSolveConstraint)
			{
				B3_PROFILE("GPU solveContactConstraint");

				m_data->m_solverGPU->solveContactConstraint(
					m_data->m_bodyBufferGPU, 
					m_data->m_inertiaBufferGPU,
					m_data->m_contactCGPU,0,
					nContactOut ,
					maxNumBatches);
			}
			else
			{
				B3_PROFILE("Host solveContactConstraint");

				m_data->m_solverGPU->solveContactConstraintHost(m_data->m_bodyBufferGPU, m_data->m_inertiaBufferGPU, m_data->m_contactCGPU,0, nContactOut ,maxNumBatches);
			}
            
            clFinish(m_data->m_queue);
        }
        
        
#if 0
        if (0)
        {
            B3_PROFILE("read body velocities back to CPU");
            //read body updated linear/angular velocities back to CPU
            m_data->m_bodyBufferGPU->read(
                                                  m_data->m_bodyBufferCPU->m_ptr,numOfConvexRBodies);
            adl::DeviceUtils::waitForCompletion( m_data->m_deviceCL );
        }
#endif
        
    }

}


void b3GpuBatchingPgsSolver::batchContacts( b3OpenCLArray<b3Contact4>* contacts, int nContacts, b3OpenCLArray<unsigned int>* n, b3OpenCLArray<unsigned int>* offsets, int staticIdx )
{
}











b3AlignedObjectArray<unsigned int> idxBuffer;
b3AlignedObjectArray<b3SortData> sortData;
b3AlignedObjectArray<b3Contact4> old;


inline int b3GpuBatchingPgsSolver::sortConstraintByBatch( b3Contact4* cs, int n, int simdWidth , int staticIdx, int numBodies)
{
	
	B3_PROFILE("sortConstraintByBatch");
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
	for(int i=0; i<n; i++) 
		idxSrc[i] = i;
	nIdxSrc = n;
    
	int batchIdx = 0;
    
	{
		B3_PROFILE("cpu batch innerloop");
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
				b3Assert( idx < n );
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
					if (!aIsStatic)
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
			b3Swap( idxSrc, idxDst );
			b3Swap( nIdxSrc, nIdxDst );
			batchIdx ++;
		}
	}
	{
		B3_PROFILE("quickSort");
		sortData.quickSort(sortfnc);
	}
	
	
	{
        B3_PROFILE("reorder");
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
        b3Assert( cs[i].getBatchIdx() != -1 );
    }
#endif
	return batchIdx;
}


b3AlignedObjectArray<int> bodyUsed2;

inline int b3GpuBatchingPgsSolver::sortConstraintByBatch2( b3Contact4* cs, int numConstraints, int simdWidth , int staticIdx, int numBodies)
{
	
	B3_PROFILE("sortConstraintByBatch2");
	

	
	bodyUsed2.resize(2*simdWidth);

	for (int q=0;q<2*simdWidth;q++)
		bodyUsed2[q]=0;

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
		B3_PROFILE("cpu batch innerloop");
		
		while( numValidConstraints < numConstraints)
		{
			numIter++;
			int nCurrentBatch = 0;
			//	clear flag
			for(int i=0; i<curBodyUsed; i++) 
				bodyUsed2[i] = 0;
            curBodyUsed = 0;

			for(int i=numValidConstraints; i<numConstraints; i++)
			{
				int idx = idxSrc[i];
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
					for (int j=0;j<curBodyUsed;j++)
					{
						if (bodyA == bodyUsed2[j])
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
						if (bodyB == bodyUsed2[j])
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
						bodyUsed2[curBodyUsed++] = bodyA;
					}
					if (!bIsStatic)
					{
						bodyUsed2[curBodyUsed++] = bodyB;
					}

					cs[idx].getBatchIdx() = batchIdx;
					m_data->m_sortData[idx].m_key = batchIdx;
					m_data->m_sortData[idx].m_value = idx;

					if (i!=numValidConstraints)
					{
						b3Swap(idxSrc[i], idxSrc[numValidConstraints]);
					}

					numValidConstraints++;
					{
						nCurrentBatch++;
						if( nCurrentBatch == simdWidth )
						{
							nCurrentBatch = 0;
							for(int i=0; i<curBodyUsed; i++) 
								bodyUsed2[i] = 0;

							
							curBodyUsed = 0;
						}
					}
				}
			}
			
			batchIdx ++;
		}
	}
	{
		B3_PROFILE("quickSort");
		//m_data->m_sortData.quickSort(sortfnc);
	}

	{
        B3_PROFILE("reorder");
		//	reorder
		
		memcpy( &m_data->m_old[0], cs, sizeof(b3Contact4)*numConstraints);

		for(int i=0; i<numConstraints; i++)
		{
			b3Assert(m_data->m_sortData[idxSrc[i]].m_value == idxSrc[i]);
			int idx = m_data->m_sortData[idxSrc[i]].m_value;
			cs[i] = m_data->m_old[idx];
		}
	}
	
#if defined(_DEBUG)
    //		debugPrintf( "nBatches: %d\n", batchIdx );
	for(int i=0; i<numConstraints; i++)
    {
        b3Assert( cs[i].getBatchIdx() != -1 );
    }
#endif

	
	return batchIdx;
}


b3AlignedObjectArray<int> bodyUsed;
b3AlignedObjectArray<int> curUsed;


inline int b3GpuBatchingPgsSolver::sortConstraintByBatch3( b3Contact4* cs, int numConstraints, int simdWidth , int staticIdx, int numBodies)
{
	
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
