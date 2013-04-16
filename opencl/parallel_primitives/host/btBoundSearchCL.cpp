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
//Host-code rewritten by Erwin Coumans

#define BOUNDSEARCH_PATH "opencl/parallel_primitives/kernels/BoundSearchKernels.cl"
#define KERNEL0 "SearchSortDataLowerKernel"
#define KERNEL1 "SearchSortDataUpperKernel"
#define KERNEL2 "SubtractKernel"


#include "btBoundSearchCL.h"
#include "../../basic_initialize/b3OpenCLUtils.h"
#include "btLauncherCL.h"
#include "../kernels/BoundSearchKernelsCL.h"

btBoundSearchCL::btBoundSearchCL(cl_context ctx, cl_device_id device, cl_command_queue queue, int maxSize)
	:m_context(ctx),
	m_device(device),
	m_queue(queue)
{

	const char* additionalMacros = "";
	const char* srcFileNameForCaching="";

	cl_int pErrNum;
	const char* kernelSource = boundSearchKernelsCL;

	cl_program boundSearchProg = b3OpenCLUtils::compileCLProgramFromString( ctx, device, kernelSource, &pErrNum,additionalMacros, BOUNDSEARCH_PATH);
	btAssert(boundSearchProg);

	m_lowerSortDataKernel = b3OpenCLUtils::compileCLKernelFromString( ctx, device, kernelSource, "SearchSortDataLowerKernel", &pErrNum, boundSearchProg,additionalMacros );
	btAssert(m_lowerSortDataKernel );

	m_upperSortDataKernel= b3OpenCLUtils::compileCLKernelFromString( ctx, device, kernelSource, "SearchSortDataUpperKernel", &pErrNum, boundSearchProg,additionalMacros );
	btAssert(m_upperSortDataKernel);

	m_subtractKernel = 0;

	if( maxSize )
	{
		m_subtractKernel= b3OpenCLUtils::compileCLKernelFromString( ctx, device, kernelSource, "SubtractKernel", &pErrNum, boundSearchProg,additionalMacros );
		btAssert(m_subtractKernel);
	}

	//m_constBuffer = new btOpenCLArray<btInt4>( device, 1, BufferBase::BUFFER_CONST );
	
	m_lower = (maxSize == 0)? 0: new btOpenCLArray<unsigned int>(ctx,queue,maxSize );
	m_upper = (maxSize == 0)? 0: new btOpenCLArray<unsigned int>(ctx,queue, maxSize );

	m_filler = new btFillCL(ctx,device,queue);
}

btBoundSearchCL::~btBoundSearchCL()
{
	
	delete m_lower;
	delete m_upper;
	delete m_filler;
			
	clReleaseKernel(m_lowerSortDataKernel);
	clReleaseKernel(m_upperSortDataKernel);
	clReleaseKernel(m_subtractKernel);
	

}


void btBoundSearchCL::execute(btOpenCLArray<btSortData>& src, int nSrc, btOpenCLArray<unsigned int>& dst, int nDst, Option option )
{
	btInt4 constBuffer;
	constBuffer.x = nSrc;
	constBuffer.y = nDst;

	if( option == BOUND_LOWER )
	{
		btBufferInfoCL bInfo[] = { btBufferInfoCL( src.getBufferCL(), true ), btBufferInfoCL( dst.getBufferCL()) };

		btLauncherCL launcher( m_queue, m_lowerSortDataKernel );
		launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(btBufferInfoCL) );
		launcher.setConst( nSrc );
        launcher.setConst( nDst );
        
		launcher.launch1D( nSrc, 64 );
	}
	else if( option == BOUND_UPPER )
	{
		btBufferInfoCL bInfo[] = { btBufferInfoCL( src.getBufferCL(), true ), btBufferInfoCL( dst.getBufferCL() ) };

		btLauncherCL launcher(m_queue, m_upperSortDataKernel );
		launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(btBufferInfoCL) );
        launcher.setConst( nSrc );
        launcher.setConst( nDst );

		launcher.launch1D( nSrc, 64 );
	}
	else if( option == COUNT )
	{
		btAssert( m_lower );
		btAssert( m_upper );
		btAssert( m_lower->capacity() <= (int)nDst );
		btAssert( m_upper->capacity() <= (int)nDst );

		int zero = 0;
		m_filler->execute( *m_lower, zero, nDst );
		m_filler->execute( *m_upper, zero, nDst );

		execute( src, nSrc, *m_lower, nDst, BOUND_LOWER );
		execute( src, nSrc, *m_upper, nDst, BOUND_UPPER );

		{
			btBufferInfoCL bInfo[] = { btBufferInfoCL( m_upper->getBufferCL(), true ), btBufferInfoCL( m_lower->getBufferCL(), true ), btBufferInfoCL( dst.getBufferCL() ) };

			btLauncherCL  launcher( m_queue, m_subtractKernel );
			launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(btBufferInfoCL) );
            launcher.setConst( nSrc );
            launcher.setConst( nDst );

			launcher.launch1D( nDst, 64 );
		}
	}
	else
	{
		btAssert( 0 );
	}

}


void btBoundSearchCL::executeHost( btAlignedObjectArray<btSortData>& src, int nSrc, 
	btAlignedObjectArray<unsigned int>& dst,  int nDst, Option option )
{


	for(int i=0; i<nSrc-1; i++) 
		btAssert( src[i].m_key <= src[i+1].m_key );

	btSortData minData,zeroData,maxData;
	minData.m_key = -1;
	minData.m_value = -1;
	zeroData.m_key=0;
	zeroData.m_value=0;
	maxData.m_key = nDst;
	maxData.m_value = nDst;

	if( option == BOUND_LOWER )
	{
		for(int i=0; i<nSrc; i++)
		{
			btSortData& iData = (i==0)? minData: src[i-1];
			btSortData& jData = (i==nSrc)? maxData: src[i];

			if( iData.m_key != jData.m_key )
			{
				int k = jData.m_key;
				{
					dst[k] = i;
				}
			}
		}
	}
	else if( option == BOUND_UPPER )
	{
		for(int i=1; i<nSrc+1; i++)
		{
			btSortData& iData = src[i-1];
			btSortData& jData = (i==nSrc)? maxData: src[i];

			if( iData.m_key != jData.m_key )
			{
				int k = iData.m_key;
				{
					dst[k] = i;
				}
			}
		}
	}
	else if( option == COUNT )
	{
		btAlignedObjectArray<unsigned int> lower;
		lower.resize(nDst );
		btAlignedObjectArray<unsigned int> upper;
		upper.resize(nDst );

		for(int i=0; i<nDst; i++) 
		{ 
			lower[i] = upper[i] = 0; 
		}

		executeHost( src, nSrc, lower, nDst, BOUND_LOWER );
		executeHost( src, nSrc, upper, nDst, BOUND_UPPER );

		for( int i=0; i<nDst; i++) 
		{ 
			dst[i] = upper[i] - lower[i]; 
		}
	}
	else
	{
		btAssert( 0 );
	}
}
