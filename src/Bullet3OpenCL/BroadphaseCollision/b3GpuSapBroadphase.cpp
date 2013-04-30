
#include "b3GpuSapBroadphase.h"
#include "Bullet3Common/b3Vector3.h"
#include "Bullet3OpenCL/ParallelPrimitives/b3LauncherCL.h"
#include "Bullet3Common/b3Quickprof.h"
#include "Bullet3OpenCL/Initialize/b3OpenCLUtils.h"
#include "kernels/sapKernels.h"
#include "kernels/sapFastKernels.h"
#include "Bullet3Common/b3MinMax.h"

#define B3_BROADPHASE_SAP_PATH "src/Bullet3OpenCL/BroadphaseCollision/kernels/sap.cl"
#define B3_BROADPHASE_SAPFAST_PATH "src/Bullet3OpenCL/BroadphaseCollision/kernels/sapFast.cl"

b3GpuSapBroadphase::b3GpuSapBroadphase(cl_context ctx,cl_device_id device, cl_command_queue  q )
:m_context(ctx),
m_device(device),
m_queue(q),
m_allAabbsGPU(ctx,q),
m_smallAabbsGPU(ctx,q),
m_largeAabbsGPU(ctx,q),
m_overlappingPairs(ctx,q),
m_gpuSmallSortData(ctx,q),
m_gpuSmallSortedAabbs(ctx,q),
m_currentBuffer(-1)
{
	const char* sapSrc = sapCL;
    const char* sapFastSrc = sapFastCL;
    
	cl_int errNum=0;

	cl_program sapProg = b3OpenCLUtils::compileCLProgramFromString(m_context,m_device,sapSrc,&errNum,"",B3_BROADPHASE_SAP_PATH);
	b3Assert(errNum==CL_SUCCESS);
	cl_program sapFastProg = b3OpenCLUtils::compileCLProgramFromString(m_context,m_device,sapFastSrc,&errNum,"",B3_BROADPHASE_SAPFAST_PATH);
	b3Assert(errNum==CL_SUCCESS);

	
	//m_sapKernel = b3OpenCLUtils::compileCLKernelFromString(m_context, m_device,sapSrc, "computePairsKernelOriginal",&errNum,sapProg );
	//m_sapKernel = b3OpenCLUtils::compileCLKernelFromString(m_context, m_device,sapSrc, "computePairsKernelBarrier",&errNum,sapProg );
	//m_sapKernel = b3OpenCLUtils::compileCLKernelFromString(m_context, m_device,sapSrc, "computePairsKernelLocalSharedMemory",&errNum,sapProg );

	
	m_sap2Kernel = b3OpenCLUtils::compileCLKernelFromString(m_context, m_device,sapSrc, "computePairsKernelTwoArrays",&errNum,sapProg );
	b3Assert(errNum==CL_SUCCESS);

#if 0

	m_sapKernel = b3OpenCLUtils::compileCLKernelFromString(m_context, m_device,sapSrc, "computePairsKernelOriginal",&errNum,sapProg );
	b3Assert(errNum==CL_SUCCESS);
#else
#ifndef __APPLE__
	m_sapKernel = b3OpenCLUtils::compileCLKernelFromString(m_context, m_device,sapFastSrc, "computePairsKernel",&errNum,sapFastProg );
	b3Assert(errNum==CL_SUCCESS);
#else
	m_sapKernel = b3OpenCLUtils::compileCLKernelFromString(m_context, m_device,sapSrc, "computePairsKernelLocalSharedMemory",&errNum,sapProg );
	b3Assert(errNum==CL_SUCCESS);
#endif
#endif

	m_flipFloatKernel = b3OpenCLUtils::compileCLKernelFromString(m_context, m_device,sapSrc, "flipFloatKernel",&errNum,sapProg );

	m_copyAabbsKernel= b3OpenCLUtils::compileCLKernelFromString(m_context, m_device,sapSrc, "copyAabbsKernel",&errNum,sapProg );

	m_scatterKernel = b3OpenCLUtils::compileCLKernelFromString(m_context, m_device,sapSrc, "scatterKernel",&errNum,sapProg );

	m_sorter = new b3RadixSort32CL(m_context,m_device,m_queue);
}

b3GpuSapBroadphase::~b3GpuSapBroadphase()
{
	delete m_sorter;
	clReleaseKernel(m_scatterKernel);
	clReleaseKernel(m_flipFloatKernel);
	clReleaseKernel(m_copyAabbsKernel);
	clReleaseKernel(m_sapKernel);
	clReleaseKernel(m_sap2Kernel);

}

/// conservative test for overlap between two aabbs
static bool TestAabbAgainstAabb2(const b3Vector3 &aabbMin1, const b3Vector3 &aabbMax1,
								const b3Vector3 &aabbMin2, const b3Vector3 &aabbMax2)
{
	bool overlap = true;
	overlap = (aabbMin1.getX() > aabbMax2.getX() || aabbMax1.getX() < aabbMin2.getX()) ? false : overlap;
	overlap = (aabbMin1.getZ() > aabbMax2.getZ() || aabbMax1.getZ() < aabbMin2.getZ()) ? false : overlap;
	overlap = (aabbMin1.getY() > aabbMax2.getY() || aabbMax1.getY() < aabbMin2.getY()) ? false : overlap;
	return overlap;
}



//http://stereopsis.com/radix.html
static unsigned int FloatFlip(float fl)
{
	unsigned int f = *(unsigned int*)&fl;
	unsigned int mask = -(int)(f >> 31) | 0x80000000;
	return f ^ mask;
};

void  b3GpuSapBroadphase::init3dSap()
{
	if (m_currentBuffer<0)
	{
		m_allAabbsGPU.copyToHost(m_allAabbsCPU);

		m_currentBuffer = 0;
		for (int axis=0;axis<3;axis++)
		{
			for (int buf=0;buf<2;buf++)
			{
				int totalNumAabbs = m_allAabbsCPU.size();
				m_sortedAxisCPU[axis][buf].resize(totalNumAabbs);

				if (buf==m_currentBuffer)
				{
					for (int i=0;i<totalNumAabbs;i++)
					{
						m_sortedAxisCPU[axis][buf][i].m_key = FloatFlip(m_allAabbsCPU[i].m_minIndices[axis]);
						m_sortedAxisCPU[axis][buf][i].m_value = i;
					}
				}
			}
		}
	}
}
void  b3GpuSapBroadphase::calculateOverlappingPairsHostIncremental3Sap()
{
	b3Assert(m_currentBuffer>=0);
	if (m_currentBuffer<0)
		return;
	
	m_allAabbsGPU.copyToHost(m_allAabbsCPU);

	for (int axis=0;axis<3;axis++)
	{
		for (int buf=0;buf<2;buf++)
		{
			b3Assert(m_sortedAxisCPU[axis][buf].size() == m_allAabbsCPU.size());
		}
	}


	m_currentBuffer = 1-m_currentBuffer;
	
	for (int axis=0;axis<3;axis++)
	{
		int totalNumAabbs = m_allAabbsCPU.size();
		for (int i=0;i<totalNumAabbs;i++)
		{
			m_sortedAxisCPU[axis][m_currentBuffer][i].m_key = FloatFlip(m_allAabbsCPU[i].m_minIndices[axis]);
			m_sortedAxisCPU[axis][m_currentBuffer][i].m_value = i;
		}
	}

	
}

void  b3GpuSapBroadphase::calculateOverlappingPairsHost()
{
	//test
	//if (m_currentBuffer>=0)
	//	calculateOverlappingPairsHostIncremental3Sap();

	int axis=0;

	b3Assert(m_allAabbsCPU.size() == m_allAabbsGPU.size());
	

	
	m_allAabbsGPU.copyToHost(m_allAabbsCPU);
	
	{
		int numSmallAabbs = m_smallAabbsCPU.size();
		for (int j=0;j<numSmallAabbs;j++)
		{
			//sync aabb
			int aabbIndex = m_smallAabbsCPU[j].m_signedMaxIndices[3];
			m_smallAabbsCPU[j] = m_allAabbsCPU[aabbIndex];
			m_smallAabbsCPU[j].m_signedMaxIndices[3] = aabbIndex;
		}
	}

	{
		int numLargeAabbs = m_largeAabbsCPU.size();
		for (int j=0;j<numLargeAabbs;j++)
		{
			//sync aabb
			int aabbIndex = m_largeAabbsCPU[j].m_signedMaxIndices[3];
			m_largeAabbsCPU[j] = m_allAabbsCPU[aabbIndex];
			m_largeAabbsCPU[j].m_signedMaxIndices[3] = aabbIndex;

		}
	}

	b3AlignedObjectArray<b3Int2> hostPairs;

	{
		int numSmallAabbs = m_smallAabbsCPU.size();
		for (int i=0;i<numSmallAabbs;i++)
		{
			float reference = m_smallAabbsCPU[i].m_max[axis];

			for (int j=i+1;j<numSmallAabbs;j++)
			{
				if (TestAabbAgainstAabb2((b3Vector3&)m_smallAabbsCPU[i].m_min, (b3Vector3&)m_smallAabbsCPU[i].m_max,
					(b3Vector3&)m_smallAabbsCPU[j].m_min,(b3Vector3&)m_smallAabbsCPU[j].m_max))
				{
					b3Int2 pair;
					pair.x = m_smallAabbsCPU[i].m_minIndices[3];//store the original index in the unsorted aabb array
					pair.y = m_smallAabbsCPU[j].m_minIndices[3];
					hostPairs.push_back(pair);
				}
			}
		}
	}

	
	{
		int numSmallAabbs = m_smallAabbsCPU.size();
		for (int i=0;i<numSmallAabbs;i++)
		{
			float reference = m_smallAabbsCPU[i].m_max[axis];
			int numLargeAabbs = m_largeAabbsCPU.size();

			for (int j=0;j<numLargeAabbs;j++)
			{
				if (TestAabbAgainstAabb2((b3Vector3&)m_smallAabbsCPU[i].m_min, (b3Vector3&)m_smallAabbsCPU[i].m_max,
					(b3Vector3&)m_largeAabbsCPU[j].m_min,(b3Vector3&)m_largeAabbsCPU[j].m_max))
				{
					b3Int2 pair;
					pair.x = m_largeAabbsCPU[j].m_minIndices[3];
					pair.y = m_smallAabbsCPU[i].m_minIndices[3];//store the original index in the unsorted aabb array
					hostPairs.push_back(pair);
				}
			}
		}
	}


	if (hostPairs.size())
	{
		m_overlappingPairs.copyFromHost(hostPairs);
	} else
	{
		m_overlappingPairs.resize(0);
	}

	//init3dSap();

}

void  b3GpuSapBroadphase::calculateOverlappingPairs()
{
	int axis = 0;//todo on GPU for now hardcode



	{

	bool syncOnHost = false;

	if (syncOnHost)
	{
		B3_PROFILE("Synchronize m_smallAabbsGPU (CPU/slow)");
		
		m_allAabbsGPU.copyToHost(m_allAabbsCPU);

		m_smallAabbsGPU.copyToHost(m_smallAabbsCPU);
		{
			int numSmallAabbs = m_smallAabbsCPU.size();
			for (int j=0;j<numSmallAabbs;j++)
			{
				//sync aabb
				int aabbIndex = m_smallAabbsCPU[j].m_signedMaxIndices[3];
				m_smallAabbsCPU[j] = m_allAabbsCPU[aabbIndex];
				m_smallAabbsCPU[j].m_signedMaxIndices[3] = aabbIndex;
			}
		}
		m_smallAabbsGPU.copyFromHost(m_smallAabbsCPU);
	
	} else
	{
		{
			int numSmallAabbs = m_smallAabbsGPU.size();
			if (numSmallAabbs)
			{
				B3_PROFILE("copyAabbsKernelSmall");
				b3BufferInfoCL bInfo[] = { 
					b3BufferInfoCL( m_allAabbsGPU.getBufferCL(), true ), 
					b3BufferInfoCL( m_smallAabbsGPU.getBufferCL()),
				};

				b3LauncherCL launcher(m_queue, m_copyAabbsKernel );
				launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(b3BufferInfoCL) );
				launcher.setConst( numSmallAabbs  );
				int num = numSmallAabbs;
				launcher.launch1D( num);
				clFinish(m_queue);
			}
		}
	}

	if (syncOnHost)
	{
		B3_PROFILE("Synchronize m_largeAabbsGPU (CPU/slow)");
		
		m_allAabbsGPU.copyToHost(m_allAabbsCPU);

		m_largeAabbsGPU.copyToHost(m_largeAabbsCPU);
		{
			int numLargeAabbs = m_largeAabbsCPU.size();
			for (int j=0;j<numLargeAabbs;j++)
			{
				//sync aabb
				int aabbIndex = m_largeAabbsCPU[j].m_signedMaxIndices[3];
				m_largeAabbsCPU[j] = m_allAabbsCPU[aabbIndex];
				m_largeAabbsCPU[j].m_signedMaxIndices[3] = aabbIndex;
			}
		}
		m_largeAabbsGPU.copyFromHost(m_largeAabbsCPU);
	
	} else
	{
		int numLargeAabbs = m_largeAabbsGPU.size();
		
		if (numLargeAabbs)
		{
			B3_PROFILE("copyAabbsKernelLarge");
			b3BufferInfoCL bInfo[] = { 
				b3BufferInfoCL( m_allAabbsGPU.getBufferCL(), true ), 
				b3BufferInfoCL( m_largeAabbsGPU.getBufferCL()),
			};

			b3LauncherCL launcher(m_queue, m_copyAabbsKernel );
			launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(b3BufferInfoCL) );
			launcher.setConst( numLargeAabbs  );
			int num = numLargeAabbs;
			launcher.launch1D( num);
			clFinish(m_queue);
		}
	}




		B3_PROFILE("GPU SAP");
		
		int numSmallAabbs = m_smallAabbsGPU.size();
		m_gpuSmallSortData.resize(numSmallAabbs);
		int numLargeAabbs = m_smallAabbsGPU.size();

#if 1
		if (m_smallAabbsGPU.size())
		{
			B3_PROFILE("flipFloatKernel");
			b3BufferInfoCL bInfo[] = { b3BufferInfoCL( m_smallAabbsGPU.getBufferCL(), true ), b3BufferInfoCL( m_gpuSmallSortData.getBufferCL())};
			b3LauncherCL launcher(m_queue, m_flipFloatKernel );
			launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(b3BufferInfoCL) );
			launcher.setConst( numSmallAabbs  );
			launcher.setConst( axis  );
			
			int num = numSmallAabbs;
			launcher.launch1D( num);
			clFinish(m_queue);
		}

		{
			B3_PROFILE("gpu radix sort\n");
			m_sorter->execute(m_gpuSmallSortData);
			clFinish(m_queue);
		}

		m_gpuSmallSortedAabbs.resize(numSmallAabbs);
		if (numSmallAabbs)
		{
			B3_PROFILE("scatterKernel");
			b3BufferInfoCL bInfo[] = { b3BufferInfoCL( m_smallAabbsGPU.getBufferCL(), true ), b3BufferInfoCL( m_gpuSmallSortData.getBufferCL(),true),b3BufferInfoCL(m_gpuSmallSortedAabbs.getBufferCL())};
			b3LauncherCL launcher(m_queue, m_scatterKernel );
			launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(b3BufferInfoCL) );
			launcher.setConst( numSmallAabbs);
			int num = numSmallAabbs;
			launcher.launch1D( num);
			clFinish(m_queue);
			
		}
        

			int maxPairsPerBody = 64;
			int maxPairs = maxPairsPerBody * numSmallAabbs;//todo
			m_overlappingPairs.resize(maxPairs);

			b3OpenCLArray<int> pairCount(m_context, m_queue);
			pairCount.push_back(0);
            int numPairs=0;

			{
				int numLargeAabbs = m_largeAabbsGPU.size();
				if (numLargeAabbs && numSmallAabbs)
				{
					B3_PROFILE("sap2Kernel");
					b3BufferInfoCL bInfo[] = { b3BufferInfoCL( m_largeAabbsGPU.getBufferCL() ),b3BufferInfoCL( m_gpuSmallSortedAabbs.getBufferCL() ), b3BufferInfoCL( m_overlappingPairs.getBufferCL() ), b3BufferInfoCL(pairCount.getBufferCL())};
					b3LauncherCL launcher(m_queue, m_sap2Kernel);
					launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(b3BufferInfoCL) );
					launcher.setConst(   numLargeAabbs  );
					launcher.setConst( numSmallAabbs);
					launcher.setConst( axis  );
					launcher.setConst( maxPairs  );
//@todo: use actual maximum work item sizes of the device instead of hardcoded values
					launcher.launch2D( numLargeAabbs, numSmallAabbs,4,64);
                
					numPairs = pairCount.at(0);
					if (numPairs >maxPairs)
						numPairs =maxPairs;
					
				}
			}
			if (m_gpuSmallSortedAabbs.size())
			{
				B3_PROFILE("sapKernel");
				b3BufferInfoCL bInfo[] = { b3BufferInfoCL( m_gpuSmallSortedAabbs.getBufferCL() ), b3BufferInfoCL( m_overlappingPairs.getBufferCL() ), b3BufferInfoCL(pairCount.getBufferCL())};
				b3LauncherCL launcher(m_queue, m_sapKernel);
				launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(b3BufferInfoCL) );
				launcher.setConst( numSmallAabbs  );
				launcher.setConst( axis  );
				launcher.setConst( maxPairs  );

			
				int num = numSmallAabbs;
#if 0                
                int buffSize = launcher.getSerializationBufferSize();
                unsigned char* buf = new unsigned char[buffSize+sizeof(int)];
                for (int i=0;i<buffSize+1;i++)
                {
                    unsigned char* ptr = (unsigned char*)&buf[i];
                    *ptr = 0xff;
                }
                int actualWrite = launcher.serializeArguments(buf,buffSize);
                
                unsigned char* cptr = (unsigned char*)&buf[buffSize];
    //            printf("buf[buffSize] = %d\n",*cptr);
                
                assert(buf[buffSize]==0xff);//check for buffer overrun
                int* ptr = (int*)&buf[buffSize];
                
                *ptr = num;
                
                FILE* f = fopen("m_sapKernelArgs.bin","wb");
                fwrite(buf,buffSize+sizeof(int),1,f);
                fclose(f);
#endif//

                launcher.launch1D( num);
				clFinish(m_queue);
                
                numPairs = pairCount.at(0);
                if (numPairs>maxPairs)
					numPairs = maxPairs;
			}
			
#else
        int numPairs = 0;
        
        
        b3LauncherCL launcher(m_queue, m_sapKernel);

        const char* fileName = "m_sapKernelArgs.bin";
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
            
            b3OpenCLArray<int> pairCount(m_context, m_queue);
            int numElements = launcher.m_arrays[2]->size()/sizeof(int);
            pairCount.setFromOpenCLBuffer(launcher.m_arrays[2]->getBufferCL(),numElements);
            numPairs = pairCount.at(0);
            //printf("overlapping pairs = %d\n",numPairs);
            b3AlignedObjectArray<b3Int2>		hostOoverlappingPairs;
            b3OpenCLArray<b3Int2> tmpGpuPairs(m_context,m_queue);
            tmpGpuPairs.setFromOpenCLBuffer(launcher.m_arrays[1]->getBufferCL(),numPairs );
   
            tmpGpuPairs.copyToHost(hostOoverlappingPairs);
            m_overlappingPairs.copyFromHost(hostOoverlappingPairs);
            //printf("hello %d\n", m_overlappingPairs.size());
            free(buf);
            fclose(f);
            
        } else {
            printf("error: cannot find file %s\n",fileName);
        }
        
        clFinish(m_queue);

        
#endif

			
        m_overlappingPairs.resize(numPairs);
		
	}//B3_PROFILE("GPU_RADIX SORT");

	
}

void b3GpuSapBroadphase::writeAabbsToGpu()
{
	m_allAabbsGPU.copyFromHost(m_allAabbsCPU);//might not be necessary, the 'setupGpuAabbsFull' already takes care of this
	m_smallAabbsGPU.copyFromHost(m_smallAabbsCPU);
	m_largeAabbsGPU.copyFromHost(m_largeAabbsCPU);

}

void b3GpuSapBroadphase::createLargeProxy(const b3Vector3& aabbMin,  const b3Vector3& aabbMax, int userPtr ,short int collisionFilterGroup,short int collisionFilterMask)
{
	int index = userPtr;
	b3SapAabb aabb;
	for (int i=0;i<4;i++)
	{
		aabb.m_min[i] = aabbMin[i];
		aabb.m_max[i] = aabbMax[i];
	}
	aabb.m_minIndices[3] = index;
	aabb.m_signedMaxIndices[3] = m_allAabbsCPU.size();
	m_largeAabbsCPU.push_back(aabb);
	m_allAabbsCPU.push_back(aabb);
}

void b3GpuSapBroadphase::createProxy(const b3Vector3& aabbMin,  const b3Vector3& aabbMax, int userPtr ,short int collisionFilterGroup,short int collisionFilterMask)
{
	int index = userPtr;
	b3SapAabb aabb;
	for (int i=0;i<4;i++)
	{
		aabb.m_min[i] = aabbMin[i];
		aabb.m_max[i] = aabbMax[i];
	}
	aabb.m_minIndices[3] = index;
	aabb.m_signedMaxIndices[3] = m_allAabbsCPU.size();
	m_smallAabbsCPU.push_back(aabb);
	m_allAabbsCPU.push_back(aabb);
}

cl_mem	b3GpuSapBroadphase::getAabbBufferWS()
{
	return m_allAabbsGPU.getBufferCL();
}

int	b3GpuSapBroadphase::getNumOverlap()
{
	return m_overlappingPairs.size();
}
cl_mem	b3GpuSapBroadphase::getOverlappingPairBuffer()
{
	return m_overlappingPairs.getBufferCL();
}