#include "btFillCL.h"
#include "../../basic_initialize/btOpenCLUtils.h"
#include "btBufferInfoCL.h"
#include "btLauncherCL.h"

#define FILL_CL_PROGRAM_PATH "opencl/parallel_primitives/kernels/FillKernels.cl"

#include "../kernels/FillKernelsCL.h"

btFillCL::btFillCL(cl_context ctx, cl_device_id device, cl_command_queue queue)
:m_commandQueue(queue)
{
	const char* kernelSource = fillKernelsCL;
	cl_int pErrNum;
	const char* additionalMacros = "";

	cl_program fillProg = btOpenCLUtils::compileCLProgramFromString( ctx, device, kernelSource, &pErrNum,additionalMacros, FILL_CL_PROGRAM_PATH);
	btAssert(fillProg);

	m_fillIntKernel = btOpenCLUtils::compileCLKernelFromString( ctx, device, kernelSource, "FillIntKernel", &pErrNum, fillProg,additionalMacros );
	btAssert(m_fillIntKernel);

	m_fillUnsignedIntKernel = btOpenCLUtils::compileCLKernelFromString( ctx, device, kernelSource, "FillUnsignedIntKernel", &pErrNum, fillProg,additionalMacros );
	btAssert(m_fillIntKernel);

	m_fillFloatKernel = btOpenCLUtils::compileCLKernelFromString( ctx, device, kernelSource, "FillFloatKernel", &pErrNum, fillProg,additionalMacros );
	btAssert(m_fillFloatKernel);

	

	m_fillKernelInt2 = btOpenCLUtils::compileCLKernelFromString( ctx, device, kernelSource, "FillInt2Kernel", &pErrNum, fillProg,additionalMacros );
	btAssert(m_fillKernelInt2);
	
}

btFillCL::~btFillCL()
{
	clReleaseKernel(m_fillKernelInt2);
	clReleaseKernel(m_fillIntKernel);
	clReleaseKernel(m_fillUnsignedIntKernel);
	clReleaseKernel(m_fillFloatKernel);

}

void btFillCL::execute(btOpenCLArray<float>& src, const float value, int n, int offset)
{
	btAssert( n>0 );

	{
		btLauncherCL launcher( m_commandQueue, m_fillFloatKernel );
		launcher.setBuffer( src.getBufferCL());
		launcher.setConst( n );
		launcher.setConst( value );
		launcher.setConst( offset);

		launcher.launch1D( n );
	}
}

void btFillCL::execute(btOpenCLArray<int>& src, const int value, int n, int offset)
{
	btAssert( n>0 );
	

	{
		btLauncherCL launcher( m_commandQueue, m_fillIntKernel );
		launcher.setBuffer(src.getBufferCL());
		launcher.setConst( n);
		launcher.setConst( value);
		launcher.setConst( offset);
		launcher.launch1D( n );
	}
}


void btFillCL::execute(btOpenCLArray<unsigned int>& src, const unsigned int value, int n, int offset)
{
	btAssert( n>0 );

	{
		btBufferInfoCL bInfo[] = { btBufferInfoCL( src.getBufferCL() ) };

		btLauncherCL launcher( m_commandQueue, m_fillUnsignedIntKernel );
		launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(btBufferInfoCL) );
		launcher.setConst( n );
        launcher.setConst(value);
		launcher.setConst(offset);

		launcher.launch1D( n );
	}
}

void btFillCL::executeHost(btAlignedObjectArray<btInt2> &src, const btInt2 &value, int n, int offset)
{
	for (int i=0;i<n;i++)
	{
		src[i+offset]=value;
	}
}

void btFillCL::executeHost(btAlignedObjectArray<int> &src, const int value, int n, int offset)
{
	for (int i=0;i<n;i++)
	{
		src[i+offset]=value;
	}
}

void btFillCL::execute(btOpenCLArray<btInt2> &src, const btInt2 &value, int n, int offset)
{
	btAssert( n>0 );
	

	{
		btBufferInfoCL bInfo[] = { btBufferInfoCL( src.getBufferCL() ) };

		btLauncherCL launcher(m_commandQueue, m_fillKernelInt2);
		launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(btBufferInfoCL) );
		launcher.setConst(n);
		launcher.setConst(value);
		launcher.setConst(offset);

		//( constBuffer );
		launcher.launch1D( n );
	}
}
