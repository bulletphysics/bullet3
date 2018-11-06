
#include <gtest/gtest.h>
#include "Bullet3Common/b3Logging.h"
#include "Bullet3Common/b3CommandLineArgs.h"
#include "Bullet3OpenCL/Initialize/b3OpenCLUtils.h"
#include "Bullet3OpenCL/BroadphaseCollision/kernels/sapKernels.h"
#include "Bullet3OpenCL/BroadphaseCollision/kernels/gridBroadphaseKernels.h"

extern int gArgc;
extern char** gArgv;

namespace
{
struct CompileBullet3BroadphaseKernels : public ::testing::Test
{
	cl_context m_clContext;
	cl_device_id m_clDevice;
	cl_command_queue m_clQueue;
	char* m_clDeviceName;
	cl_platform_id m_platformId;

	CompileBullet3BroadphaseKernels()
		: m_clDeviceName(0),
		  m_clContext(0),
		  m_clDevice(0),
		  m_clQueue(0),
		  m_platformId(0)
	{
		// You can do set-up work for each test here.
		b3CommandLineArgs args(gArgc, gArgv);
		int preferredDeviceIndex = -1;
		int preferredPlatformIndex = -1;
		bool allowCpuOpenCL = false;

		initCL();
	}

	virtual ~CompileBullet3BroadphaseKernels()
	{
		// You can do clean-up work that doesn't throw exceptions here.
		exitCL();
	}

	// If the constructor and destructor are not enough for setting up
	// and cleaning up each test, you can define the following methods:

#include "initCL.h"

	virtual void SetUp()
	{
		// Code here will be called immediately after the constructor (right
		// before each test).
	}

	virtual void TearDown()
	{
		// Code here will be called immediately after each test (right
		// before the destructor).
	}
};

TEST_F(CompileBullet3BroadphaseKernels, sapKernels)
{
	cl_int errNum = 0;
	cl_program sapProg = b3OpenCLUtils::compileCLProgramFromString(m_clContext, m_clDevice, sapCL, &errNum, "", 0, true);
	{
		ASSERT_EQ(CL_SUCCESS, errNum);
		cl_kernel copyAabbsKernel = b3OpenCLUtils::compileCLKernelFromString(m_clContext, m_clDevice, sapCL, "copyAabbsKernel", &errNum, sapProg);
		ASSERT_EQ(CL_SUCCESS, errNum);
		ASSERT_FALSE(copyAabbsKernel == 0);
		clReleaseKernel(copyAabbsKernel);
	}
	{
		cl_kernel sap2Kernel = b3OpenCLUtils::compileCLKernelFromString(m_clContext, m_clDevice, sapCL, "computePairsKernelTwoArrays", &errNum, sapProg);
		ASSERT_EQ(CL_SUCCESS, errNum);
		ASSERT_FALSE(sap2Kernel == 0);
		clReleaseKernel(sap2Kernel);
	}
	{
		cl_kernel sapKernelBruteForce = b3OpenCLUtils::compileCLKernelFromString(m_clContext, m_clDevice, sapCL, "computePairsKernelBruteForce", &errNum, sapProg);
		ASSERT_EQ(CL_SUCCESS, errNum);
		ASSERT_FALSE(sapKernelBruteForce == 0);
		clReleaseKernel(sapKernelBruteForce);
	}
	{
		cl_kernel sapKernelOriginal = b3OpenCLUtils::compileCLKernelFromString(m_clContext, m_clDevice, sapCL, "computePairsKernelOriginal", &errNum, sapProg);
		ASSERT_EQ(CL_SUCCESS, errNum);
		ASSERT_FALSE(sapKernelOriginal == 0);
		clReleaseKernel(sapKernelOriginal);
	}

	{
		cl_kernel sapKernelBarrier = b3OpenCLUtils::compileCLKernelFromString(m_clContext, m_clDevice, sapCL, "computePairsKernelBarrier", &errNum, sapProg);
		ASSERT_EQ(CL_SUCCESS, errNum);
		ASSERT_FALSE(sapKernelBarrier == 0);
		clReleaseKernel(sapKernelBarrier);
	}
	{
		cl_kernel sapKernelLocalShared = b3OpenCLUtils::compileCLKernelFromString(m_clContext, m_clDevice, sapCL, "computePairsKernelLocalSharedMemory", &errNum, sapProg);
		ASSERT_EQ(CL_SUCCESS, errNum);
		ASSERT_FALSE(sapKernelLocalShared == 0);
		clReleaseKernel(sapKernelLocalShared);
	}
	{
		cl_kernel prepareSumVarianceKernel = b3OpenCLUtils::compileCLKernelFromString(m_clContext, m_clDevice, sapCL, "prepareSumVarianceKernel", &errNum, sapProg);
		ASSERT_EQ(CL_SUCCESS, errNum);
		ASSERT_FALSE(prepareSumVarianceKernel == 0);
		clReleaseKernel(prepareSumVarianceKernel);
	}
	{
		cl_kernel k = b3OpenCLUtils::compileCLKernelFromString(m_clContext, m_clDevice, sapCL, "flipFloatKernel", &errNum, sapProg);
		ASSERT_EQ(CL_SUCCESS, errNum);
		ASSERT_FALSE(k == 0);
		clReleaseKernel(k);
	}
	{
		cl_kernel k = b3OpenCLUtils::compileCLKernelFromString(m_clContext, m_clDevice, sapCL, "scatterKernel", &errNum, sapProg);
		ASSERT_EQ(CL_SUCCESS, errNum);
		ASSERT_FALSE(k == 0);
		clReleaseKernel(k);
	}

	clReleaseProgram(sapProg);
};

TEST_F(CompileBullet3BroadphaseKernels, gridBroadphaseKernels)
{
	cl_int errNum = 0;
	cl_program gridProg = b3OpenCLUtils::compileCLProgramFromString(m_clContext, m_clDevice, gridBroadphaseCL, &errNum, "", 0, true);
	ASSERT_EQ(CL_SUCCESS, errNum);

	{
		cl_kernel k = b3OpenCLUtils::compileCLKernelFromString(m_clContext, m_clDevice, gridBroadphaseCL, "kCalcHashAABB", &errNum, gridProg);
		ASSERT_EQ(CL_SUCCESS, errNum);
		clReleaseKernel(k);
	}
	{
		cl_kernel k = b3OpenCLUtils::compileCLKernelFromString(m_clContext, m_clDevice, gridBroadphaseCL, "kClearCellStart", &errNum, gridProg);
		ASSERT_EQ(CL_SUCCESS, errNum);
		clReleaseKernel(k);
	}

	{
		cl_kernel k = b3OpenCLUtils::compileCLKernelFromString(m_clContext, m_clDevice, gridBroadphaseCL, "kFindCellStart", &errNum, gridProg);
		ASSERT_EQ(CL_SUCCESS, errNum);
		clReleaseKernel(k);
	}

	{
		cl_kernel k = b3OpenCLUtils::compileCLKernelFromString(m_clContext, m_clDevice, gridBroadphaseCL, "kFindOverlappingPairs", &errNum, gridProg);
		ASSERT_EQ(CL_SUCCESS, errNum);
		clReleaseKernel(k);
	}

	clReleaseProgram(gridProg);
}
};  // namespace
