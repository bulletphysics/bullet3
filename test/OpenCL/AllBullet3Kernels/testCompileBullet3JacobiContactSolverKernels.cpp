
#include <gtest/gtest.h>
#include "Bullet3Common/b3Logging.h"
#include "Bullet3Common/b3CommandLineArgs.h"
#include "Bullet3OpenCL/Initialize/b3OpenCLUtils.h"

#include "Bullet3OpenCL/RigidBody/kernels/solverUtils.h"

extern int gArgc;
extern char** gArgv;

namespace
{
struct CompileBullet3JacobiContactSolverKernels : public ::testing::Test
{
	cl_context m_clContext;
	cl_device_id m_clDevice;
	cl_command_queue m_clQueue;
	char* m_clDeviceName;
	cl_platform_id m_platformId;

	CompileBullet3JacobiContactSolverKernels()
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

	virtual ~CompileBullet3JacobiContactSolverKernels()
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

TEST_F(CompileBullet3JacobiContactSolverKernels, jacobiContactKernels)
{
	cl_int errNum = 0;
	const char* additionalMacros = "";

	cl_program solverUtilsProg = b3OpenCLUtils::compileCLProgramFromString(m_clContext, m_clDevice, solverUtilsCL, &errNum, additionalMacros, 0, true);
	ASSERT_EQ(CL_SUCCESS, errNum);

	{
		cl_kernel k = b3OpenCLUtils::compileCLKernelFromString(m_clContext, m_clDevice, solverUtilsCL, "CountBodiesKernel", &errNum, solverUtilsProg, additionalMacros);
		ASSERT_EQ(CL_SUCCESS, errNum);
		ASSERT_FALSE(k == 0);
		clReleaseKernel(k);
	}

	{
		cl_kernel k = b3OpenCLUtils::compileCLKernelFromString(m_clContext, m_clDevice, solverUtilsCL, "ContactToConstraintSplitKernel", &errNum, solverUtilsProg, additionalMacros);
		ASSERT_EQ(CL_SUCCESS, errNum);
		ASSERT_FALSE(k == 0);
		clReleaseKernel(k);
	}

	{
		cl_kernel k = b3OpenCLUtils::compileCLKernelFromString(m_clContext, m_clDevice, solverUtilsCL, "ClearVelocitiesKernel", &errNum, solverUtilsProg, additionalMacros);
		ASSERT_EQ(CL_SUCCESS, errNum);
		ASSERT_FALSE(k == 0);
		clReleaseKernel(k);
	}

	{
		cl_kernel k = b3OpenCLUtils::compileCLKernelFromString(m_clContext, m_clDevice, solverUtilsCL, "AverageVelocitiesKernel", &errNum, solverUtilsProg, additionalMacros);
		ASSERT_EQ(CL_SUCCESS, errNum);
		ASSERT_FALSE(k == 0);
		clReleaseKernel(k);
	}

	{
		cl_kernel k = b3OpenCLUtils::compileCLKernelFromString(m_clContext, m_clDevice, solverUtilsCL, "UpdateBodyVelocitiesKernel", &errNum, solverUtilsProg, additionalMacros);
		ASSERT_EQ(CL_SUCCESS, errNum);
		ASSERT_FALSE(k == 0);
		clReleaseKernel(k);
	}

	{
		cl_kernel k = b3OpenCLUtils::compileCLKernelFromString(m_clContext, m_clDevice, solverUtilsCL, "SolveContactJacobiKernel", &errNum, solverUtilsProg, additionalMacros);
		ASSERT_EQ(CL_SUCCESS, errNum);
		ASSERT_FALSE(k == 0);
		clReleaseKernel(k);
	}

	{
		cl_kernel k = b3OpenCLUtils::compileCLKernelFromString(m_clContext, m_clDevice, solverUtilsCL, "SolveFrictionJacobiKernel", &errNum, solverUtilsProg, additionalMacros);
		ASSERT_EQ(CL_SUCCESS, errNum);
		ASSERT_FALSE(k == 0);
		clReleaseKernel(k);
	}
	clReleaseProgram(solverUtilsProg);
}
};  // namespace
