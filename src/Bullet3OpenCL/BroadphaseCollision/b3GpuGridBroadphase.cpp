
#include "b3GpuGridBroadphase.h"
#include "Bullet3Geometry/b3AabbUtil.h"
#include "kernels/gridBroadphaseKernels.h"

//#include "kernels/gridBroadphase.cl"


#include "Bullet3OpenCL/Initialize/b3OpenCLUtils.h"
#include "Bullet3OpenCL/ParallelPrimitives/b3LauncherCL.h"




#define B3_GRID_BROADPHASE_PATH "src/Bullet3OpenCL/BroadphaseCollision/kernels/gridBroadphase.cl"

cl_kernel kCalcHashAABB;
cl_kernel kClearCellStart;
cl_kernel kFindCellStart;
cl_kernel kFindOverlappingPairs;


cl_kernel kFindPairsLarge;
cl_kernel kComputePairCacheChanges;
cl_kernel kSqueezeOverlappingPairBuff;


int maxPairsPerBody = 32;
int maxBodiesPerCell = 1024;//??

b3GpuGridBroadphase::b3GpuGridBroadphase(cl_context ctx,cl_device_id device, cl_command_queue  q )
:m_context(ctx),
m_device(device),
m_queue(q),
m_allAabbsGPU(ctx,q),
m_gpuPairs(ctx,q),
m_hashGpu(ctx,q),
m_paramsGPU(ctx,q),
m_cellStartGpu(ctx,q)
{

	
	b3Vector3 gridSize = b3MakeVector3(3,3,3);
	b3Vector3 invGridSize = b3MakeVector3(1.f/gridSize[0],1.f/gridSize[1],1.f/gridSize[2]);

	m_paramsCPU.m_gridSize[0] = 128;
	m_paramsCPU.m_gridSize[1] = 128;
	m_paramsCPU.m_gridSize[2] = 128;
	m_paramsCPU.m_gridSize[3] = maxBodiesPerCell;
	m_paramsCPU.setMaxBodiesPerCell(maxBodiesPerCell);
	m_paramsCPU.m_invCellSize[0] = invGridSize[0];
	m_paramsCPU.m_invCellSize[1] = invGridSize[1];
	m_paramsCPU.m_invCellSize[2] = invGridSize[2];
	m_paramsCPU.m_invCellSize[3] = 0.f;
	m_paramsGPU.push_back(m_paramsCPU);

	cl_int errNum=0;
	cl_program gridProg = b3OpenCLUtils::compileCLProgramFromString(m_context,m_device,0,&errNum,"",B3_GRID_BROADPHASE_PATH,true);
	b3Assert(errNum==CL_SUCCESS);

	kCalcHashAABB = b3OpenCLUtils::compileCLKernelFromString(m_context, m_device,gridBroadphaseCL, "kCalcHashAABB",&errNum,gridProg);
	b3Assert(errNum==CL_SUCCESS);
	
	kClearCellStart = b3OpenCLUtils::compileCLKernelFromString(m_context, m_device,gridBroadphaseCL, "kClearCellStart",&errNum,gridProg);
	b3Assert(errNum==CL_SUCCESS);

	kFindCellStart = b3OpenCLUtils::compileCLKernelFromString(m_context, m_device,gridBroadphaseCL, "kFindCellStart",&errNum,gridProg);
	b3Assert(errNum==CL_SUCCESS);

	
	kFindOverlappingPairs = b3OpenCLUtils::compileCLKernelFromString(m_context, m_device,gridBroadphaseCL, "kFindOverlappingPairs",&errNum,gridProg);
	b3Assert(errNum==CL_SUCCESS);

	kFindPairsLarge = b3OpenCLUtils::compileCLKernelFromString(m_context, m_device,gridBroadphaseCL, "kFindPairsLarge",&errNum,gridProg);
	b3Assert(errNum==CL_SUCCESS);

	kComputePairCacheChanges = b3OpenCLUtils::compileCLKernelFromString(m_context, m_device,gridBroadphaseCL, "kComputePairCacheChanges",&errNum,gridProg);
	b3Assert(errNum==CL_SUCCESS);

	kSqueezeOverlappingPairBuff = b3OpenCLUtils::compileCLKernelFromString(m_context, m_device,gridBroadphaseCL, "kSqueezeOverlappingPairBuff",&errNum,gridProg);
	b3Assert(errNum==CL_SUCCESS);

	m_sorter = new b3RadixSort32CL(m_context,m_device,m_queue);

}
b3GpuGridBroadphase::~b3GpuGridBroadphase()
{
	clReleaseKernel( kCalcHashAABB);
	clReleaseKernel( kClearCellStart);
	clReleaseKernel( kFindCellStart);
	clReleaseKernel( kFindOverlappingPairs);
	
	clReleaseKernel( kFindPairsLarge);
	clReleaseKernel( kComputePairCacheChanges);
	clReleaseKernel( kSqueezeOverlappingPairBuff);
	delete m_sorter;
}



void b3GpuGridBroadphase::createProxy(const b3Vector3& aabbMin,  const b3Vector3& aabbMax, int userPtr ,short int collisionFilterGroup,short int collisionFilterMask)
{
	b3SapAabb aabb;
	aabb.m_minVec = aabbMin;
	aabb.m_maxVec = aabbMax;
	aabb.m_minIndices[3] = userPtr;
	aabb.m_signedMaxIndices[3] = userPtr;
	m_allAabbsCPU.push_back(aabb);
}
void b3GpuGridBroadphase::createLargeProxy(const b3Vector3& aabbMin,  const b3Vector3& aabbMax, int userPtr ,short int collisionFilterGroup,short int collisionFilterMask)
{
	createProxy(aabbMin,aabbMax,userPtr,collisionFilterGroup,collisionFilterMask);
	
}

void  b3GpuGridBroadphase::calculateOverlappingPairs(int maxPairs)
{
	B3_PROFILE("b3GpuGridBroadphase::calculateOverlappingPairs");
	/*
	calculateOverlappingPairsHost(maxPairs);
	{

		b3AlignedObjectArray<b3Int4> cpuPairs;
		m_gpuPairs.copyToHost(cpuPairs);
		printf("host m_gpuPairs.size()=%d\n",m_gpuPairs.size());
		for (int i=0;i<m_gpuPairs.size();i++)
		{
			printf("host pair %d = %d,%d\n",i,cpuPairs[i].x,cpuPairs[i].y);
		}
	}
	*/
	//return;
	int numAabbs = m_allAabbsGPU.size();
	if (numAabbs)
	{
		m_hashGpu.resize(numAabbs);
		{
			b3LauncherCL launch(m_queue,kCalcHashAABB);
			launch.setConst(numAabbs);
			launch.setBuffer(m_allAabbsGPU.getBufferCL());
			launch.setBuffer(m_hashGpu.getBufferCL());
			launch.setBuffer(this->m_paramsGPU.getBufferCL());
			launch.launch1D(numAabbs);
		}

		m_sorter->execute(m_hashGpu);
		
		int numCells = this->m_paramsCPU.m_gridSize[0]*this->m_paramsCPU.m_gridSize[1]*this->m_paramsCPU.m_gridSize[2];
		m_cellStartGpu.resize(numCells);
		//b3AlignedObjectArray<int >			cellStartCpu;
				
		
		{
			b3LauncherCL launch(m_queue,kClearCellStart);
			launch.setConst(numCells);
			launch.setBuffer(m_cellStartGpu.getBufferCL());
			launch.launch1D(numCells);
			//m_cellStartGpu.copyToHost(cellStartCpu);
			//printf("??\n");

		}


		{

			b3LauncherCL launch(m_queue,kFindCellStart);
			launch.setConst(numAabbs);
			launch.setBuffer(m_hashGpu.getBufferCL());
			launch.setBuffer(m_cellStartGpu.getBufferCL());
			launch.launch1D(numAabbs);
			//m_cellStartGpu.copyToHost(cellStartCpu);
			//printf("??\n");

		}
		
		{


			b3OpenCLArray<b3Int2> pairsGpu2(m_context,m_queue);
			b3OpenCLArray<unsigned int> pairsGpu(m_context,m_queue);
			b3OpenCLArray<unsigned int> pairStartCurGpu(m_context,m_queue);
			b3AlignedObjectArray<unsigned int> pairStartCpu;

			m_gpuPairs.resize(numAabbs*maxPairsPerBody);
			pairsGpu2.resize(numAabbs*maxPairsPerBody);
			pairsGpu.resize(numAabbs*maxPairsPerBody);
			pairStartCurGpu.resize(numAabbs*2+2);

			pairStartCpu.resize(numAabbs*2+2);

			pairStartCpu[0] = 0;
			pairStartCpu[1] = 0;
			for(int i = 1; i <= numAabbs; i++) 
			{
				pairStartCpu[i * 2] = pairStartCpu[(i-1) * 2] + maxPairsPerBody;
				pairStartCpu[i * 2 + 1] = 0;
			}
			pairStartCurGpu.copyFromHost(pairStartCpu);
			
			b3OpenCLArray<int> pairCount(m_context,m_queue);
			pairCount.push_back(0);

			b3LauncherCL launch(m_queue,kFindOverlappingPairs);
			launch.setConst(numAabbs);
			launch.setBuffer(m_allAabbsGPU.getBufferCL());
			launch.setBuffer(m_hashGpu.getBufferCL());
			launch.setBuffer(m_cellStartGpu.getBufferCL());
			launch.setBuffer(pairsGpu.getBufferCL());
			launch.setBuffer(pairStartCurGpu.getBufferCL());
			launch.setBuffer(m_paramsGPU.getBufferCL());
			//launch.setBuffer(0);
			launch.setBuffer(pairCount.getBufferCL());
			launch.setBuffer(m_gpuPairs.getBufferCL());
			
			launch.launch1D(numAabbs);
			

			
			int actualCount = pairCount.at(0);
			m_gpuPairs.resize(actualCount);
			/*
			b3AlignedObjectArray<b3Int4> pairsCpu;
			m_gpuPairs.copyToHost(pairsCpu);
			
			printf("m_gpuPairs.size()=%d\n",m_gpuPairs.size());
			for (int i=0;i<m_gpuPairs.size();i++)
			{
				printf("pair %d = %d,%d\n",i,pairsCpu[i].x,pairsCpu[i].y);
			}

			printf("?!?\n");
			*/
		}
	

	}

	



	//calculateOverlappingPairsHost(maxPairs);
}
void  b3GpuGridBroadphase::calculateOverlappingPairsHost(int maxPairs)
{
	m_hostPairs.resize(0);
		
	for (int i=0;i<m_allAabbsCPU.size();i++)
	{
		for (int j=i+1;j<m_allAabbsCPU.size();j++)
		{
			if (b3TestAabbAgainstAabb2(m_allAabbsCPU[i].m_minVec, m_allAabbsCPU[i].m_maxVec,
				m_allAabbsCPU[j].m_minVec,m_allAabbsCPU[j].m_maxVec))
			{
				b3Int4 pair;
				int a = m_allAabbsCPU[j].m_minIndices[3];
				int b = m_allAabbsCPU[i].m_minIndices[3];
				if (a<=b)
				{
					pair.x = a; 
					pair.y = b;//store the original index in the unsorted aabb array
				} else
				{
					pair.x = b;
					pair.y = a;//store the original index in the unsorted aabb array
				}
					
				m_hostPairs.push_back(pair);
			}
		}
	}


	m_gpuPairs.copyFromHost(m_hostPairs);
}

	//call writeAabbsToGpu after done making all changes (createProxy etc)
void b3GpuGridBroadphase::writeAabbsToGpu()
{
	m_allAabbsGPU.copyFromHost(m_allAabbsCPU);
}

cl_mem	b3GpuGridBroadphase::getAabbBufferWS()
{
	return this->m_allAabbsGPU.getBufferCL();
}
int	b3GpuGridBroadphase::getNumOverlap()
{
	return m_gpuPairs.size();
}
cl_mem	b3GpuGridBroadphase::getOverlappingPairBuffer()
{
	return m_gpuPairs.getBufferCL();
}

b3OpenCLArray<b3SapAabb>&	b3GpuGridBroadphase::getAllAabbsGPU()
{
	return m_allAabbsGPU;
}

b3AlignedObjectArray<b3SapAabb>&	b3GpuGridBroadphase::getAllAabbsCPU()
{
	return m_allAabbsCPU;
}