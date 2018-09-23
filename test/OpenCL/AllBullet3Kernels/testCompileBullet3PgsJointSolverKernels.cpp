
#include <gtest/gtest.h>
#include "Bullet3Common/b3Logging.h"
#include "Bullet3Common/b3CommandLineArgs.h"
#include "Bullet3OpenCL/Initialize/b3OpenCLUtils.h"

#include "Bullet3OpenCL/RigidBody/kernels/jointSolver.h"

extern int gArgc;
extern char** gArgv;

namespace
{
struct testCompileBullet3PgsJointSolverKernels : public ::testing::Test
{
	cl_context m_clContext;
	cl_device_id m_clDevice;
	cl_command_queue m_clQueue;
	char* m_clDeviceName;
	cl_platform_id m_platformId;

	testCompileBullet3PgsJointSolverKernels()
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

	virtual ~testCompileBullet3PgsJointSolverKernels()
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

TEST_F(testCompileBullet3PgsJointSolverKernels, solveConstraintRowsCL)
{
	cl_int errNum = 0;

	cl_program prog = b3OpenCLUtils::compileCLProgramFromString(m_clContext, m_clDevice, solveConstraintRowsCL, &errNum, "", 0, true);
	ASSERT_EQ(CL_SUCCESS, errNum);

	{
		cl_kernel k = b3OpenCLUtils::compileCLKernelFromString(m_clContext, m_clDevice, solveConstraintRowsCL, "solveJointConstraintRows", &errNum, prog);
		ASSERT_EQ(CL_SUCCESS, errNum);
		ASSERT_FALSE(k == 0);
		clReleaseKernel(k);
	}

	{
		cl_kernel k = b3OpenCLUtils::compileCLKernelFromString(m_clContext, m_clDevice, solveConstraintRowsCL, "initSolverBodies", &errNum, prog);
		ASSERT_EQ(CL_SUCCESS, errNum);
		ASSERT_FALSE(k == 0);
		clReleaseKernel(k);
	}
	{
		cl_kernel k = b3OpenCLUtils::compileCLKernelFromString(m_clContext, m_clDevice, solveConstraintRowsCL, "getInfo1Kernel", &errNum, prog);
		ASSERT_EQ(CL_SUCCESS, errNum);
		ASSERT_FALSE(k == 0);
		clReleaseKernel(k);
	}
	{
		cl_kernel k = b3OpenCLUtils::compileCLKernelFromString(m_clContext, m_clDevice, solveConstraintRowsCL, "initBatchConstraintsKernel", &errNum, prog);
		ASSERT_EQ(CL_SUCCESS, errNum);
		ASSERT_FALSE(k == 0);
		clReleaseKernel(k);
	}
	{
		cl_kernel k = b3OpenCLUtils::compileCLKernelFromString(m_clContext, m_clDevice, solveConstraintRowsCL, "getInfo2Kernel", &errNum, prog);
		ASSERT_EQ(CL_SUCCESS, errNum);
		ASSERT_FALSE(k == 0);
		clReleaseKernel(k);
	}
	{
		cl_kernel k = b3OpenCLUtils::compileCLKernelFromString(m_clContext, m_clDevice, solveConstraintRowsCL, "writeBackVelocitiesKernel", &errNum, prog);
		ASSERT_EQ(CL_SUCCESS, errNum);
		ASSERT_FALSE(k == 0);
		clReleaseKernel(k);
	}
	{
		cl_kernel k = b3OpenCLUtils::compileCLKernelFromString(m_clContext, m_clDevice, solveConstraintRowsCL, "breakViolatedConstraintsKernel", &errNum, prog);
		ASSERT_EQ(CL_SUCCESS, errNum);
		ASSERT_FALSE(k == 0);
		clReleaseKernel(k);
	}

	clReleaseProgram(prog);
}
};  // namespace
