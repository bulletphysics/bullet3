
#include <gtest/gtest.h>
#include "Bullet3Common/b3Logging.h"
#include "Bullet3Common/b3CommandLineArgs.h"
#include "Bullet3OpenCL/Initialize/b3OpenCLUtils.h"

#include "Bullet3OpenCL/NarrowphaseCollision/kernels/satKernels.h"
#include "Bullet3OpenCL/NarrowphaseCollision/kernels/mprKernels.h"
#include "Bullet3OpenCL/NarrowphaseCollision/kernels/satConcaveKernels.h"
#include "Bullet3OpenCL/NarrowphaseCollision/kernels/satClipHullContacts.h"
#include "Bullet3OpenCL/NarrowphaseCollision/kernels/bvhTraversal.h"
#include "Bullet3OpenCL/NarrowphaseCollision/kernels/primitiveContacts.h"

extern int gArgc;
extern char** gArgv;

namespace
{
struct CompileBullet3NarrowphaseKernels : public ::testing::Test
{
	cl_context m_clContext;
	cl_device_id m_clDevice;
	cl_command_queue m_clQueue;
	char* m_clDeviceName;
	cl_platform_id m_platformId;

	CompileBullet3NarrowphaseKernels()
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

	virtual ~CompileBullet3NarrowphaseKernels()
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

TEST_F(CompileBullet3NarrowphaseKernels, satKernelsCL)
{
	cl_int errNum = 0;

	char flags[1024] = {0};

	cl_program satProg = b3OpenCLUtils::compileCLProgramFromString(m_clContext, m_clDevice, satKernelsCL, &errNum, flags, 0, true);
	ASSERT_EQ(CL_SUCCESS, errNum);

	{
		cl_kernel m_findSeparatingAxisKernel = b3OpenCLUtils::compileCLKernelFromString(m_clContext, m_clDevice, satKernelsCL, "findSeparatingAxisKernel", &errNum, satProg);
		ASSERT_EQ(CL_SUCCESS, errNum);
		clReleaseKernel(m_findSeparatingAxisKernel);
	}

	{
		cl_kernel m_findSeparatingAxisVertexFaceKernel = b3OpenCLUtils::compileCLKernelFromString(m_clContext, m_clDevice, satKernelsCL, "findSeparatingAxisVertexFaceKernel", &errNum, satProg);
		ASSERT_EQ(CL_SUCCESS, errNum);
		clReleaseKernel(m_findSeparatingAxisVertexFaceKernel);
	}

	{
		cl_kernel m_findSeparatingAxisEdgeEdgeKernel = b3OpenCLUtils::compileCLKernelFromString(m_clContext, m_clDevice, satKernelsCL, "findSeparatingAxisEdgeEdgeKernel", &errNum, satProg);
		ASSERT_EQ(CL_SUCCESS, errNum);
		clReleaseKernel(m_findSeparatingAxisEdgeEdgeKernel);
	}

	{
		cl_kernel m_findConcaveSeparatingAxisKernel = b3OpenCLUtils::compileCLKernelFromString(m_clContext, m_clDevice, satKernelsCL, "findConcaveSeparatingAxisKernel", &errNum, satProg);
		ASSERT_EQ(CL_SUCCESS, errNum);
		clReleaseKernel(m_findConcaveSeparatingAxisKernel);
	}

	{
		cl_kernel m_findCompoundPairsKernel = b3OpenCLUtils::compileCLKernelFromString(m_clContext, m_clDevice, satKernelsCL, "findCompoundPairsKernel", &errNum, satProg);
		ASSERT_EQ(CL_SUCCESS, errNum);
		clReleaseKernel(m_findCompoundPairsKernel);
	}

	{
		cl_kernel m_processCompoundPairsKernel = b3OpenCLUtils::compileCLKernelFromString(m_clContext, m_clDevice, satKernelsCL, "processCompoundPairsKernel", &errNum, satProg);
		ASSERT_EQ(CL_SUCCESS, errNum);
		clReleaseKernel(m_processCompoundPairsKernel);
	}

	clReleaseProgram(satProg);
}

TEST_F(CompileBullet3NarrowphaseKernels, satConcaveKernelsCL)
{
	cl_int errNum = 0;

	char flags[1024] = {0};

	cl_program satConcaveProg = b3OpenCLUtils::compileCLProgramFromString(m_clContext, m_clDevice, satConcaveKernelsCL, &errNum, flags, 0, true);
	ASSERT_EQ(CL_SUCCESS, errNum);

	{
		cl_kernel m_findConcaveSeparatingAxisVertexFaceKernel = b3OpenCLUtils::compileCLKernelFromString(m_clContext, m_clDevice, satConcaveKernelsCL, "findConcaveSeparatingAxisVertexFaceKernel", &errNum, satConcaveProg);
		ASSERT_EQ(CL_SUCCESS, errNum);
		clReleaseKernel(m_findConcaveSeparatingAxisVertexFaceKernel);
	}

	{
		cl_kernel m_findConcaveSeparatingAxisEdgeEdgeKernel = b3OpenCLUtils::compileCLKernelFromString(m_clContext, m_clDevice, satConcaveKernelsCL, "findConcaveSeparatingAxisEdgeEdgeKernel", &errNum, satConcaveProg);
		ASSERT_EQ(CL_SUCCESS, errNum);
		clReleaseKernel(m_findConcaveSeparatingAxisEdgeEdgeKernel);
	}

	clReleaseProgram(satConcaveProg);
}

TEST_F(CompileBullet3NarrowphaseKernels, satClipKernelsCL)
{
	char flags[1024] = {0};
	cl_int errNum = 0;
	//#ifdef CL_PLATFORM_INTEL
	//		sprintf(flags,"-g -s \"%s\"","C:/develop/bullet3_experiments2/opencl/gpu_narrowphase/kernels/satClipHullContacts.cl");
	//#endif

	cl_program satClipContactsProg = b3OpenCLUtils::compileCLProgramFromString(m_clContext, m_clDevice, satClipKernelsCL, &errNum, flags, 0, true);
	ASSERT_EQ(CL_SUCCESS, errNum);

	{
		cl_kernel m_clipHullHullKernel = b3OpenCLUtils::compileCLKernelFromString(m_clContext, m_clDevice, satClipKernelsCL, "clipHullHullKernel", &errNum, satClipContactsProg);
		ASSERT_EQ(CL_SUCCESS, errNum);
		clReleaseKernel(m_clipHullHullKernel);
	}

	{
		cl_kernel m_clipCompoundsHullHullKernel = b3OpenCLUtils::compileCLKernelFromString(m_clContext, m_clDevice, satClipKernelsCL, "clipCompoundsHullHullKernel", &errNum, satClipContactsProg);
		ASSERT_EQ(CL_SUCCESS, errNum);
		clReleaseKernel(m_clipCompoundsHullHullKernel);
	}

	{
		cl_kernel k = b3OpenCLUtils::compileCLKernelFromString(m_clContext, m_clDevice, satClipKernelsCL, "findClippingFacesKernel", &errNum, satClipContactsProg);
		ASSERT_EQ(CL_SUCCESS, errNum);
		clReleaseKernel(k);
	}

	{
		cl_kernel k = b3OpenCLUtils::compileCLKernelFromString(m_clContext, m_clDevice, satClipKernelsCL, "clipFacesAndFindContactsKernel", &errNum, satClipContactsProg);
		ASSERT_EQ(CL_SUCCESS, errNum);
		clReleaseKernel(k);
	}

	{
		cl_kernel k = b3OpenCLUtils::compileCLKernelFromString(m_clContext, m_clDevice, satClipKernelsCL, "clipHullHullConcaveConvexKernel", &errNum, satClipContactsProg);
		ASSERT_EQ(CL_SUCCESS, errNum);
		clReleaseKernel(k);
	}

	{
		cl_kernel k = b3OpenCLUtils::compileCLKernelFromString(m_clContext, m_clDevice, satClipKernelsCL,
															   "newContactReductionKernel", &errNum, satClipContactsProg);
		ASSERT_EQ(CL_SUCCESS, errNum);
		clReleaseKernel(k);
	}

	clReleaseProgram(satClipContactsProg);
}

TEST_F(CompileBullet3NarrowphaseKernels, bvhTraversalKernels)
{
	cl_int errNum = 0;
	cl_program bvhTraversalProg = b3OpenCLUtils::compileCLProgramFromString(m_clContext, m_clDevice, bvhTraversalKernelCL, &errNum, "", 0, true);
	ASSERT_EQ(CL_SUCCESS, errNum);

	{
		cl_kernel k = b3OpenCLUtils::compileCLKernelFromString(m_clContext, m_clDevice, bvhTraversalKernelCL, "bvhTraversalKernel", &errNum, bvhTraversalProg, "");
		ASSERT_EQ(CL_SUCCESS, errNum);
		clReleaseKernel(k);
	}
	clReleaseProgram(bvhTraversalProg);
}

TEST_F(CompileBullet3NarrowphaseKernels, primitiveContactsKernelsCL)
{
	cl_int errNum = 0;
	cl_program primitiveContactsProg = b3OpenCLUtils::compileCLProgramFromString(m_clContext, m_clDevice, primitiveContactsKernelsCL, &errNum, "", 0, true);
	ASSERT_EQ(CL_SUCCESS, errNum);

	{
		cl_kernel k = b3OpenCLUtils::compileCLKernelFromString(m_clContext, m_clDevice, primitiveContactsKernelsCL, "primitiveContactsKernel", &errNum, primitiveContactsProg, "");
		ASSERT_EQ(CL_SUCCESS, errNum);
		clReleaseKernel(k);
	}

	{
		cl_kernel k = b3OpenCLUtils::compileCLKernelFromString(m_clContext, m_clDevice, primitiveContactsKernelsCL, "findConcaveSphereContactsKernel", &errNum, primitiveContactsProg);
		ASSERT_EQ(CL_SUCCESS, errNum);
		clReleaseKernel(k);
	}

	{
		cl_kernel k = b3OpenCLUtils::compileCLKernelFromString(m_clContext, m_clDevice, primitiveContactsKernelsCL, "processCompoundPairsPrimitivesKernel", &errNum, primitiveContactsProg, "");
		ASSERT_EQ(CL_SUCCESS, errNum);
		clReleaseKernel(k);
	}

	clReleaseProgram(primitiveContactsProg);
}

TEST_F(CompileBullet3NarrowphaseKernels, mprKernelsCL)
{
	cl_int errNum = 0;
	const char* srcConcave = satConcaveKernelsCL;
	char flags[1024] = {0};
	cl_program mprProg = b3OpenCLUtils::compileCLProgramFromString(m_clContext, m_clDevice, mprKernelsCL, &errNum, flags, 0, true);
	ASSERT_EQ(CL_SUCCESS, errNum);

	{
		cl_kernel k = b3OpenCLUtils::compileCLKernelFromString(m_clContext, m_clDevice, mprKernelsCL, "mprPenetrationKernel", &errNum, mprProg);
		ASSERT_EQ(CL_SUCCESS, errNum);
		clReleaseKernel(k);
	}

	{
		cl_kernel k = b3OpenCLUtils::compileCLKernelFromString(m_clContext, m_clDevice, mprKernelsCL, "findSeparatingAxisUnitSphereKernel", &errNum, mprProg);
		ASSERT_EQ(CL_SUCCESS, errNum);
		clReleaseKernel(k);
	}

	clReleaseProgram(mprProg);
}

};  // namespace
