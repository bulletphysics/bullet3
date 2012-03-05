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


#define PATH "..\\..\\dynamics\\basic_demo\\Stubs\\SolverKernels"
#define BATCHING_PATH "..\\..\\dynamics\\basic_demo\\Stubs\\batchingKernels"

#define KERNEL1 "SingleBatchSolveKernel"
#define KERNEL2 "BatchSolveKernel"

#define KERNEL3 "ContactToConstraintKernel"
#define KERNEL4 "SetSortDataKernel"
#define KERNEL5 "ReorderContactKernel"
#include "SolverKernels.h"

#include "batchingKernels.h"


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
		Buffer<u32>* m_numConstraints;
		Buffer<u32>* m_offsets;
	};
};

template<DeviceType TYPE>
typename Solver<TYPE>::Data* Solver<TYPE>::allocate( const Device* device, int pairCapacity )
{
		const char* src[] = 
#if defined(ADL_LOAD_KERNEL_FROM_STRING)
		{solverKernelsCL, 0};
#else
		{0,0};
#endif

		const char* src2[] = 
#if defined(ADL_LOAD_KERNEL_FROM_STRING)
		{batchingKernelsCL, 0};
#else
		{0,0};
#endif


	

	Data* data = new Data;
	data->m_device = device;
	bool cacheBatchingKernel = true;
	data->m_batchingKernel = device->getKernel( BATCHING_PATH, "CreateBatches",  "-I ..\\..\\ ", src2[TYPE],cacheBatchingKernel);
	//data->m_batchingKernel = device->getKernel( BATCHING_PATH, "CreateBatches",  "-I ..\\..\\ ", 0,cacheBatchingKernel);
	bool cacheSolverKernel  = true;

	data->m_batchSolveKernel = device->getKernel( PATH, KERNEL2, "-I ..\\..\\ ", src[TYPE],cacheSolverKernel );
	data->m_contactToConstraintKernel = device->getKernel( PATH, KERNEL3, 
		"-I ..\\..\\ ", src[TYPE] );
	data->m_setSortDataKernel = device->getKernel( PATH, KERNEL4, 
		"-I ..\\..\\ ", src[TYPE] );
	data->m_reorderContactKernel = device->getKernel( PATH, KERNEL5, 
		"-I ..\\..\\ ", src[TYPE] );

	data->m_copyConstraintKernel = device->getKernel( PATH, "CopyConstraintKernel", 
		"-I ..\\..\\ ", src[TYPE] );

	data->m_parallelSolveData = new SolverDeviceInl::ParallelSolveData;
	{
		SolverDeviceInl::ParallelSolveData* solveData = (SolverDeviceInl::ParallelSolveData*)data->m_parallelSolveData;
		solveData->m_numConstraints = new Buffer<u32>( device, N_SPLIT*N_SPLIT );
		solveData->m_offsets = new Buffer<u32>( device, N_SPLIT*N_SPLIT );
	}
	const int sortSize = NEXTMULTIPLEOF( pairCapacity, 512 );


	//data->m_sort = RadixSort<TYPE>::allocate( data->m_device, sortSize );//todo. remove hardcode this
	data->m_sort32 = RadixSort32<TYPE>::allocate( data->m_device, sortSize );//todo. remove hardcode this
	
	data->m_search = BoundSearch<TYPE>::allocate( data->m_device, N_SPLIT*N_SPLIT );
	data->m_scan = PrefixScan<TYPE>::allocate( data->m_device, N_SPLIT*N_SPLIT );

	data->m_sortDataBuffer = new Buffer<SortData>( data->m_device, sortSize );

	if( pairCapacity < DYNAMIC_CONTACT_ALLOCATION_THRESHOLD )
		data->m_contactBuffer = new Buffer<Contact4>( data->m_device, pairCapacity );
	else
		data->m_contactBuffer = 0;

	return data;
}

template<DeviceType TYPE>
void Solver<TYPE>::deallocate( Data* data )
{
	{
		SolverDeviceInl::ParallelSolveData* solveData = (SolverDeviceInl::ParallelSolveData*)data->m_parallelSolveData;
		delete solveData->m_numConstraints;
		delete solveData->m_offsets;
		delete solveData;
	}

//	RadixSort<TYPE>::deallocate( data->m_sort );
	RadixSort32<TYPE>::deallocate(data->m_sort32);
	BoundSearch<TYPE>::deallocate( data->m_search );
	PrefixScan<TYPE>::deallocate( data->m_scan );

	delete data->m_sortDataBuffer;
	if( data->m_contactBuffer ) delete data->m_contactBuffer;

	delete data;
}

template<DeviceType TYPE>
void Solver<TYPE>::reorderConvertToConstraints( typename Solver<TYPE>::Data* data, const Buffer<RigidBodyBase::Body>* bodyBuf, 
	const Buffer<RigidBodyBase::Inertia>* shapeBuf,
	Buffer<Contact4>* contactsIn, SolverData contactCOut, void* additionalData, 
	int nContacts, const typename Solver<TYPE>::ConstraintCfg& cfg )
{
	if( data->m_contactBuffer )
	{
		if( data->m_contactBuffer->getSize() < nContacts )
		{
			BT_PROFILE("delete data->m_contactBuffer;");
			delete data->m_contactBuffer;
			data->m_contactBuffer = 0;
		}
	}
	if( data->m_contactBuffer == 0 )
	{
		BT_PROFILE("new data->m_contactBuffer;");

		data->m_contactBuffer = new Buffer<Contact4>( data->m_device, nContacts );
	}
	Stopwatch sw;

	Buffer<Contact4>* contactNative = BufferUtils::map<TYPE_CL, true>( data->m_device, contactsIn, nContacts );

	//DeviceUtils::Config dhCfg;
	//Device* deviceHost = DeviceUtils::allocate( TYPE_HOST, dhCfg );
	if( cfg.m_enableParallelSolve )
	{
		SolverDeviceInl::ParallelSolveData* nativeSolveData = (SolverDeviceInl::ParallelSolveData*)data->m_parallelSolveData;

		DeviceUtils::waitForCompletion( data->m_device );
		sw.start();
		//	contactsIn -> data->m_contactBuffer
		{
			BT_PROFILE("sortContacts");
			Solver<TYPE>::sortContacts( data, bodyBuf, contactNative, additionalData, nContacts, cfg );
			DeviceUtils::waitForCompletion( data->m_device );
		}
		sw.split();
		if(0)
		{
			Contact4* tmp = new Contact4[nContacts];
			data->m_contactBuffer->read( tmp, nContacts );
			DeviceUtils::waitForCompletion( data->m_contactBuffer->m_device );
			contactNative->write( tmp, nContacts );
			DeviceUtils::waitForCompletion( contactNative->m_device );
			delete [] tmp;
		}
		else
		{
			BT_PROFILE("m_copyConstraintKernel");

			Buffer<int4> constBuffer( data->m_device, 1, BufferBase::BUFFER_CONST );

			int4 cdata; cdata.x = nContacts;
			BufferInfo bInfo[] = { BufferInfo( data->m_contactBuffer ), BufferInfo( contactNative ) };
//			Launcher launcher( data->m_device, data->m_device->getKernel( PATH, "CopyConstraintKernel",  "-I ..\\..\\ -Wf,--c++", 0 ) );
			Launcher launcher( data->m_device, data->m_copyConstraintKernel );
			launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(Launcher::BufferInfo) );
			launcher.setConst( constBuffer, cdata );
			launcher.launch1D( nContacts, 64 );
			DeviceUtils::waitForCompletion( data->m_device );
		}
		{
			BT_PROFILE("batchContacts");
			Solver<TYPE>::batchContacts( data, contactNative, nContacts, nativeSolveData->m_numConstraints, nativeSolveData->m_offsets, cfg.m_staticIdx );

		}
	}
	{
			BT_PROFILE("waitForCompletion (batchContacts)");
			DeviceUtils::waitForCompletion( data->m_device );
	}
	sw.split();
	//================
	if(0)
	{
//		Solver<TYPE_HOST>::Data* solverHost = Solver<TYPE_HOST>::allocate( deviceHost, nContacts );
//		Solver<TYPE_HOST>::convertToConstraints( solverHost, bodyBuf, shapeBuf, contactNative, contactCOut, additionalData, nContacts, cfg );
//		Solver<TYPE_HOST>::deallocate( solverHost );
	}
	else
	{
		BT_PROFILE("convertToConstraints");
		Solver<TYPE>::convertToConstraints( data, bodyBuf, shapeBuf, contactNative, contactCOut, additionalData, nContacts, cfg );
	}
	{
		BT_PROFILE("convertToConstraints waitForCompletion");
		DeviceUtils::waitForCompletion( data->m_device );
	}
	sw.stop();

	{
		BT_PROFILE("printf");

		float t[5];
		sw.getMs( t, 3 );
//		printf("%3.2f, %3.2f, %3.2f, ", t[0], t[1], t[2]);
	}

	{
		BT_PROFILE("deallocate and unmap");

		//DeviceUtils::deallocate( deviceHost );

		BufferUtils::unmap<true>( contactNative, contactsIn, nContacts );
	}
}


template<DeviceType TYPE>
void Solver<TYPE>::solveContactConstraint( typename Solver<TYPE>::Data* data, const Buffer<RigidBodyBase::Body>* bodyBuf, const Buffer<RigidBodyBase::Inertia>* shapeBuf, 
			SolverData constraint, void* additionalData, int n )
{
	if(0)
	{
		DeviceUtils::Config dhCfg;
		Device* deviceHost = DeviceUtils::allocate( TYPE_HOST, dhCfg );
		{
			Solver<TYPE_HOST>::Data* hostData = Solver<TYPE_HOST>::allocate( deviceHost, 0 );
			Solver<TYPE_HOST>::solveContactConstraint( hostData, bodyBuf, shapeBuf, constraint, additionalData, n );
			Solver<TYPE_HOST>::deallocate( hostData );
		}
		DeviceUtils::deallocate( deviceHost );
		return;
	}

	ADLASSERT( data );

	Buffer<Constraint4>* cBuffer =0;
	
	Buffer<RigidBodyBase::Body>* gBodyNative=0; 
	Buffer<RigidBodyBase::Inertia>* gShapeNative =0;
	Buffer<Constraint4>* gConstraintNative =0;
	

	{
		BT_PROFILE("map");
	cBuffer = (Buffer<Constraint4>*)constraint;

		gBodyNative= BufferUtils::map<TYPE, true>( data->m_device, bodyBuf );
		gShapeNative= BufferUtils::map<TYPE, true>( data->m_device, shapeBuf );
		gConstraintNative = BufferUtils::map<TYPE, true>( data->m_device, cBuffer );
		DeviceUtils::waitForCompletion( data->m_device );
	}

	Buffer<int4> constBuffer;
	int4 cdata = make_int4( n, 0, 0, 0 );
	{
		SolverDeviceInl::ParallelSolveData* solveData = (SolverDeviceInl::ParallelSolveData*)data->m_parallelSolveData;
		const int nn = N_SPLIT*N_SPLIT;

		cdata.x = 0;
		cdata.y = 250;

#if 0
//check how the cells are filled
		unsigned int* hostCounts = new unsigned int[N_SPLIT*N_SPLIT];
		solveData->m_numConstraints->read(hostCounts,N_SPLIT*N_SPLIT);
		DeviceUtils::waitForCompletion( data->m_device );
		for (int i=0;i<N_SPLIT*N_SPLIT;i++)
		{
			if (hostCounts[i])
			{
				printf("hostCounts[%d]=%d\n",i,hostCounts[i]);
			}
		}
		delete[] hostCounts;
#endif

		int numWorkItems = 64*nn/N_BATCHES;
#ifdef DEBUG_ME
		SolverDebugInfo* debugInfo = new  SolverDebugInfo[numWorkItems];
		adl::Buffer<SolverDebugInfo> gpuDebugInfo(data->m_device,numWorkItems);
#endif



		{

			BT_PROFILE("m_batchSolveKernel iterations");
			for(int iter=0; iter<data->m_nIterations; iter++)
			{
				for(int ib=0; ib<N_BATCHES; ib++)
				{
#ifdef DEBUG_ME
					memset(debugInfo,0,sizeof(SolverDebugInfo)*numWorkItems);
					gpuDebugInfo.write(debugInfo,numWorkItems);
#endif


					cdata.z = ib;
					cdata.w = N_SPLIT;

				

					BufferInfo bInfo[] = { 

						BufferInfo( gBodyNative ), 
						BufferInfo( gShapeNative ), 
						BufferInfo( gConstraintNative ),
						BufferInfo( solveData->m_numConstraints ), 
						BufferInfo( solveData->m_offsets ) 
#ifdef DEBUG_ME
						,	BufferInfo(&gpuDebugInfo)
#endif
						};

					Launcher launcher( data->m_device, data->m_batchSolveKernel );
					launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(Launcher::BufferInfo) );
					launcher.setConst( constBuffer, cdata );
					
					launcher.launch1D( numWorkItems, 64 );

#ifdef DEBUG_ME
					DeviceUtils::waitForCompletion( data->m_device );
					gpuDebugInfo.read(debugInfo,numWorkItems);
					DeviceUtils::waitForCompletion( data->m_device );
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
		
			DeviceUtils::waitForCompletion( data->m_device );


		}

		cdata.x = 1;
		{
			BT_PROFILE("m_batchSolveKernel iterations2");
			for(int iter=0; iter<data->m_nIterations; iter++)
			{
				for(int ib=0; ib<N_BATCHES; ib++)
				{
					cdata.z = ib;
					cdata.w = N_SPLIT;

					BufferInfo bInfo[] = { 
						BufferInfo( gBodyNative ), 
						BufferInfo( gShapeNative ), 
						BufferInfo( gConstraintNative ),
						BufferInfo( solveData->m_numConstraints ), 
						BufferInfo( solveData->m_offsets )
#ifdef DEBUG_ME
						,BufferInfo(&gpuDebugInfo)
#endif //DEBUG_ME
					};
					Launcher launcher( data->m_device, data->m_batchSolveKernel );
					launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(Launcher::BufferInfo) );
					launcher.setConst( constBuffer, cdata );
					launcher.launch1D( 64*nn/N_BATCHES, 64 );
				}
			}
			DeviceUtils::waitForCompletion( data->m_device );
			
		}
#ifdef DEBUG_ME
		delete[] debugInfo;
#endif //DEBUG_ME
	}

	{
		BT_PROFILE("unmap");
	BufferUtils::unmap<true>( gBodyNative, bodyBuf );
	BufferUtils::unmap<false>( gShapeNative, shapeBuf );
	BufferUtils::unmap<true>( gConstraintNative, cBuffer );
	DeviceUtils::waitForCompletion( data->m_device );
	}
}

template<DeviceType TYPE>
void Solver<TYPE>::convertToConstraints( typename Solver<TYPE>::Data* data, const Buffer<RigidBodyBase::Body>* bodyBuf, 
	const Buffer<RigidBodyBase::Inertia>* shapeBuf, 
	Buffer<Contact4>* contactsIn, SolverData contactCOut, void* additionalData, 
	int nContacts, const ConstraintCfg& cfg )
{
	ADLASSERT( data->m_device->m_type == TYPE_CL );

	Buffer<RigidBodyBase::Body>* bodyNative =0;
	Buffer<RigidBodyBase::Inertia>* shapeNative =0;
	Buffer<Contact4>* contactNative =0;
	Buffer<Constraint4>* constraintNative =0;

	{
		BT_PROFILE("map buffers");

		bodyNative = BufferUtils::map<TYPE, true>( data->m_device, bodyBuf );
		shapeNative  = BufferUtils::map<TYPE, true>( data->m_device, shapeBuf );
		contactNative= BufferUtils::map<TYPE, true>( data->m_device, contactsIn );
		constraintNative = BufferUtils::map<TYPE, false>( data->m_device, (Buffer<Constraint4>*)contactCOut );
	}
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

		Buffer<CB> constBuffer( data->m_device, 1, BufferBase::BUFFER_CONST );
		BufferInfo bInfo[] = { BufferInfo( contactNative ), BufferInfo( bodyNative ), BufferInfo( shapeNative ),
			BufferInfo( constraintNative )};
		Launcher launcher( data->m_device, data->m_contactToConstraintKernel );
		launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(Launcher::BufferInfo) );
		launcher.setConst( constBuffer, cdata );
		launcher.launch1D( nContacts, 64 );	
		DeviceUtils::waitForCompletion( data->m_device );

	}

	{
		BT_PROFILE("unmap");
		BufferUtils::unmap<false>( bodyNative, bodyBuf );
		BufferUtils::unmap<false>( shapeNative, shapeBuf );
		BufferUtils::unmap<false>( contactNative, contactsIn );
		BufferUtils::unmap<true>( constraintNative, (Buffer<Constraint4>*)contactCOut );
	}
}

template<DeviceType TYPE>
void Solver<TYPE>::sortContacts( typename Solver<TYPE>::Data* data, const Buffer<RigidBodyBase::Body>* bodyBuf, 
			Buffer<Contact4>* contactsIn, void* additionalData, 
			int nContacts, const typename Solver<TYPE>::ConstraintCfg& cfg )
{
	ADLASSERT( data->m_device->m_type == TYPE_CL );
	Buffer<RigidBodyBase::Body>* bodyNative 
		= BufferUtils::map<TYPE_CL, true>( data->m_device, bodyBuf );
	Buffer<Contact4>* contactNative 
		= BufferUtils::map<TYPE_CL, true>( data->m_device, contactsIn );

	const int sortAlignment = 512; // todo. get this out of sort
	if( cfg.m_enableParallelSolve )
	{
		SolverDeviceInl::ParallelSolveData* nativeSolveData = (SolverDeviceInl::ParallelSolveData*)data->m_parallelSolveData;

		int sortSize = NEXTMULTIPLEOF( nContacts, sortAlignment );

		Buffer<u32>* countsNative = nativeSolveData->m_numConstraints;//BufferUtils::map<TYPE_CL, false>( data->m_device, &countsHost );
		Buffer<u32>* offsetsNative = nativeSolveData->m_offsets;//BufferUtils::map<TYPE_CL, false>( data->m_device, &offsetsHost );

		{	//	2. set cell idx
			struct CB
			{
				int m_nContacts;
				int m_staticIdx;
				float m_scale;
				int m_nSplit;
			};

			ADLASSERT( sortSize%64 == 0 );
			CB cdata;
			cdata.m_nContacts = nContacts;
			cdata.m_staticIdx = cfg.m_staticIdx;
			cdata.m_scale = 1.f/(N_OBJ_PER_SPLIT*cfg.m_averageExtent);
			cdata.m_nSplit = N_SPLIT;

			Buffer<CB> constBuffer( data->m_device, 1, BufferBase::BUFFER_CONST );
			BufferInfo bInfo[] = { BufferInfo( contactNative ), BufferInfo( bodyNative ), BufferInfo( data->m_sortDataBuffer ) };
			Launcher launcher( data->m_device, data->m_setSortDataKernel );
			launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(Launcher::BufferInfo) );
			launcher.setConst( constBuffer, cdata );
			launcher.launch1D( sortSize, 64 );
		}

		{	//	3. sort by cell idx
			int n = N_SPLIT*N_SPLIT;
			int sortBit = 32;
			//if( n <= 0xffff ) sortBit = 16;
			//if( n <= 0xff ) sortBit = 8;
			RadixSort32<TYPE>::execute( data->m_sort32, *data->m_sortDataBuffer,sortSize);
		}
		{	//	4. find entries
			BoundSearch<TYPE>::execute( data->m_search, *data->m_sortDataBuffer, nContacts, *countsNative, N_SPLIT*N_SPLIT, BoundSearchBase::COUNT );

			PrefixScan<TYPE>::execute( data->m_scan, *countsNative, *offsetsNative, N_SPLIT*N_SPLIT );
		}

		{	//	5. sort constraints by cellIdx
			//	todo. preallocate this
//			ADLASSERT( contactsIn->getType() == TYPE_HOST );
//			Buffer<Contact4>* out = BufferUtils::map<TYPE_CL, false>( data->m_device, contactsIn );	//	copying contacts to this buffer

			{
				Buffer<int4> constBuffer( data->m_device, 1, BufferBase::BUFFER_CONST );

				int4 cdata; cdata.x = nContacts;
				BufferInfo bInfo[] = { BufferInfo( contactNative ), BufferInfo( data->m_contactBuffer ), BufferInfo( data->m_sortDataBuffer ) };
				Launcher launcher( data->m_device, data->m_reorderContactKernel );
				launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(Launcher::BufferInfo) );
				launcher.setConst( constBuffer, cdata );
				launcher.launch1D( nContacts, 64 );
			}
//			BufferUtils::unmap<true>( out, contactsIn, nContacts );
		}
	}

	BufferUtils::unmap<false>( bodyNative, bodyBuf );
	BufferUtils::unmap<false>( contactNative, contactsIn );
}

template<DeviceType TYPE>
void Solver<TYPE>::batchContacts( typename Solver<TYPE>::Data* data, Buffer<Contact4>* contacts, int nContacts, Buffer<u32>* n, Buffer<u32>* offsets, int staticIdx )
{
	ADLASSERT( data->m_device->m_type == TYPE_CL );

	if(0)
	{
		BT_PROFILE("CPU classTestKernel/Kernel (batch generation?)");

		DeviceUtils::Config dhCfg;
		Device* deviceHost = DeviceUtils::allocate( TYPE_HOST, dhCfg );
		{
			Solver<TYPE_HOST>::Data* hostData = Solver<TYPE_HOST>::allocate( deviceHost, 0 );
			Solver<TYPE_HOST>::batchContacts( hostData, contacts, nContacts, n, offsets, staticIdx );
			Solver<TYPE_HOST>::deallocate( hostData );
		}
		DeviceUtils::deallocate( deviceHost );
		return;
	}

	Buffer<Contact4>* contactNative 
		= BufferUtils::map<TYPE_CL, true>( data->m_device, contacts, nContacts );
	Buffer<u32>* nNative
		= BufferUtils::map<TYPE_CL, true>( data->m_device, n );
	Buffer<u32>* offsetsNative
		= BufferUtils::map<TYPE_CL, true>( data->m_device, offsets );

	{
		BT_PROFILE("GPU classTestKernel/Kernel (batch generation?)");
		Buffer<int4> constBuffer( data->m_device, 1, BufferBase::BUFFER_CONST );
		int4 cdata;
		cdata.x = nContacts;
		cdata.y = 0;
		cdata.z = staticIdx;

		int numWorkItems = 64*N_SPLIT*N_SPLIT;
#ifdef BATCH_DEBUG
		SolverDebugInfo* debugInfo = new  SolverDebugInfo[numWorkItems];
		adl::Buffer<SolverDebugInfo> gpuDebugInfo(data->m_device,numWorkItems);
		memset(debugInfo,0,sizeof(SolverDebugInfo)*numWorkItems);
		gpuDebugInfo.write(debugInfo,numWorkItems);
#endif


		BufferInfo bInfo[] = { 
			BufferInfo( contactNative ), 
			BufferInfo( data->m_contactBuffer ), 
			BufferInfo( nNative ), 
			BufferInfo( offsetsNative ) 
#ifdef BATCH_DEBUG
			,	BufferInfo(&gpuDebugInfo)
#endif
		};

		
		
		Launcher launcher( data->m_device, data->m_batchingKernel);
		launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(Launcher::BufferInfo) );
		launcher.setConst( constBuffer, cdata );
		launcher.launch1D( numWorkItems, 64 );
		DeviceUtils::waitForCompletion( data->m_device );

#ifdef BATCH_DEBUG
	aaaa
		Contact4* hostContacts = new Contact4[nContacts];
		data->m_contactBuffer->read(hostContacts,nContacts);
		DeviceUtils::waitForCompletion( data->m_device );

		gpuDebugInfo.read(debugInfo,numWorkItems);
		DeviceUtils::waitForCompletion( data->m_device );

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

	if(0)
	{
		u32* nhost = new u32[N_SPLIT*N_SPLIT];

		nNative->read( nhost, N_SPLIT*N_SPLIT );

		Contact4* chost = new Contact4[nContacts];
		data->m_contactBuffer->read( chost, nContacts );
		DeviceUtils::waitForCompletion( data->m_device );
		printf(">>");
		int nonzero = 0;
		u32 maxn = 0;
		for(int i=0; i<N_SPLIT*N_SPLIT; i++)
		{
			printf("%d-", nhost[i]);
			nonzero += (nhost[i]==0)? 0:1;
			maxn = max2( nhost[i], maxn );
		}
		printf("\nnonzero:zero = %d:%d (%d)\n", nonzero, N_SPLIT*N_SPLIT-nonzero, maxn);
		printf("\n\n");

		int prev = 0;
		int prevIdx = 0;
		int maxNBatches = 0;
		for(int i=0; i<nContacts; i++)
		{
//			printf("(%d, %d:%d),", chost[i].m_batchIdx, chost[i].m_bodyAPtr, chost[i].m_bodyBPtr);
			if( prev != 0 && chost[i].m_batchIdx == 0 )
			{
				maxNBatches = max2( maxNBatches, prev );
				printf("\n[%d]", prev);

				//for(int j=prevIdx; j<i; j++)
				//{
				//	printf("(%d:%d),", chost[j].m_bodyAPtr, chost[j].m_bodyBPtr);
				//}

				//printf("\n");

				prevIdx = i;
			}

			printf("%d,", chost[i].m_batchIdx);

			prev = chost[i].m_batchIdx;
		}
		printf("\n");
		printf("Max: %d\n", maxNBatches);

		delete [] chost;
		delete [] nhost;
	}
//	copy buffer to buffer
	contactNative->write( *data->m_contactBuffer, nContacts );
	DeviceUtils::waitForCompletion( data->m_device );

	if(0)
	{
		DeviceUtils::Config dhCfg;
		Device* deviceHost = DeviceUtils::allocate( TYPE_HOST, dhCfg );
		{
			HostBuffer<Contact4> host( deviceHost, nContacts );
			contactNative->read( host.m_ptr, nContacts );
			DeviceUtils::waitForCompletion( data->m_device );

			for(int i=0; i<nContacts; i++)
			{
				ADLASSERT( host[i].m_bodyAPtr <= (u32)staticIdx );
				ADLASSERT( host[i].m_bodyBPtr <= (u32)staticIdx );
			}
		}
		DeviceUtils::deallocate( deviceHost );
	}

	BufferUtils::unmap<true>( contactNative, contacts );
	BufferUtils::unmap<false>( nNative, n );
	BufferUtils::unmap<false>( offsetsNative, offsets );
}

#undef PATH
#undef KERNEL1
#undef KERNEL2

#undef KERNEL3
#undef KERNEL4
#undef KERNEL5
