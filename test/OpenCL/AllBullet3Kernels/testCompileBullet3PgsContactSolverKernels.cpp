
#include <gtest/gtest.h>
#include "Bullet3Common/b3Logging.h"
#include "Bullet3Common/b3CommandLineArgs.h"
#include "Bullet3OpenCL/Initialize/b3OpenCLUtils.h"

#include "Bullet3OpenCL/RigidBody/kernels/solverSetup.h"
#include "Bullet3OpenCL/RigidBody/kernels/solverSetup2.h"
#include "Bullet3OpenCL/RigidBody/kernels/solveContact.h"
#include "Bullet3OpenCL/RigidBody/kernels/solveFriction.h"
#include "Bullet3OpenCL/RigidBody/kernels/batchingKernels.h"
#include "Bullet3OpenCL/RigidBody/kernels/batchingKernelsNew.h"


extern int gArgc;
extern char** gArgv;

namespace
{
	struct CompileBullet3PgsContactSolverKernels : public ::testing::Test 
	{
		cl_context			m_clContext;
		cl_device_id		m_clDevice;
		cl_command_queue	m_clQueue;
		char*				m_clDeviceName;
		cl_platform_id		m_platformId;

		CompileBullet3PgsContactSolverKernels()
			:m_clDeviceName(0),
			m_clContext(0),
			m_clDevice(0),
			m_clQueue(0),
			m_platformId(0)
		{
				// You can do set-up work for each test here.
			b3CommandLineArgs args(gArgc,gArgv);
			int preferredDeviceIndex=-1;
			int preferredPlatformIndex = -1;
			bool allowCpuOpenCL = false;


			initCL();
		}

		virtual ~CompileBullet3PgsContactSolverKernels() 
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

	TEST_F(CompileBullet3PgsContactSolverKernels,solveFrictionCL)
	{
		const char* additionalMacros="";
		cl_int errNum=0;
		cl_program solveFrictionProg= b3OpenCLUtils::compileCLProgramFromString( m_clContext, m_clDevice, solveFrictionCL, &errNum,additionalMacros, 0,true);
		ASSERT_EQ(CL_SUCCESS,errNum);

		{
			cl_kernel k = b3OpenCLUtils::compileCLKernelFromString( m_clContext, m_clDevice, solveFrictionCL, "BatchSolveKernelFriction", &errNum, solveFrictionProg,additionalMacros );
			ASSERT_EQ(CL_SUCCESS,errNum);
			ASSERT_FALSE(k==0);
			clReleaseKernel(k);
		}

		{
			cl_kernel k =b3OpenCLUtils::compileCLKernelFromString( m_clContext, m_clDevice, solveFrictionCL, "solveSingleFrictionKernel", &errNum, solveFrictionProg,additionalMacros );
			ASSERT_EQ(CL_SUCCESS,errNum);
			ASSERT_FALSE(k==0);
			clReleaseKernel(k);
		}
		clReleaseProgram(solveFrictionProg);
	}

	TEST_F(CompileBullet3PgsContactSolverKernels,solverSetupCL)
	{
		const char* additionalMacros="";
		cl_int errNum=0;
			cl_program solverSetupProg= b3OpenCLUtils::compileCLProgramFromString( m_clContext, m_clDevice, solverSetupCL, &errNum,additionalMacros, 0,true);
		ASSERT_EQ(CL_SUCCESS,errNum);

		{
			cl_kernel k = b3OpenCLUtils::compileCLKernelFromString( m_clContext, m_clDevice, solverSetupCL, "ContactToConstraintKernel", &errNum, solverSetupProg,additionalMacros );
			ASSERT_EQ(CL_SUCCESS,errNum);
			ASSERT_FALSE(k==0);
			clReleaseKernel(k);
		}

		clReleaseProgram(solverSetupProg);
	}

	TEST_F(CompileBullet3PgsContactSolverKernels,solverSetup2CL)
	{
		const char* additionalMacros="";
		cl_int errNum=0;

		cl_program solverSetup2Prog= b3OpenCLUtils::compileCLProgramFromString( m_clContext, m_clDevice, solverSetup2CL, &errNum,additionalMacros, 0,true);
		ASSERT_EQ(CL_SUCCESS,errNum);

			
			
		{
			cl_kernel k =  b3OpenCLUtils::compileCLKernelFromString( m_clContext, m_clDevice, solverSetup2CL, "SetSortDataKernel", &errNum, solverSetup2Prog,additionalMacros );
			ASSERT_EQ(CL_SUCCESS,errNum);
			ASSERT_FALSE(k==0);
			clReleaseKernel(k);
		}

		{
			cl_kernel k =  b3OpenCLUtils::compileCLKernelFromString( m_clContext, m_clDevice, solverSetup2CL, "SetDeterminismSortDataBodyA", &errNum, solverSetup2Prog,additionalMacros );
			ASSERT_EQ(CL_SUCCESS,errNum);
			ASSERT_FALSE(k==0);
			clReleaseKernel(k);
		}

		{
			cl_kernel k =  b3OpenCLUtils::compileCLKernelFromString( m_clContext, m_clDevice, solverSetup2CL, "SetDeterminismSortDataBodyB", &errNum, solverSetup2Prog,additionalMacros );
			ASSERT_EQ(CL_SUCCESS,errNum);
			ASSERT_FALSE(k==0);
			clReleaseKernel(k);
		}

		{
			cl_kernel k = b3OpenCLUtils::compileCLKernelFromString( m_clContext, m_clDevice, solverSetup2CL, "SetDeterminismSortDataChildShapeA", &errNum, solverSetup2Prog,additionalMacros );
			ASSERT_EQ(CL_SUCCESS,errNum);
			ASSERT_FALSE(k==0);
			clReleaseKernel(k);
		}

		{
			cl_kernel k = b3OpenCLUtils::compileCLKernelFromString( m_clContext, m_clDevice, solverSetup2CL, "SetDeterminismSortDataChildShapeB", &errNum, solverSetup2Prog,additionalMacros );
			ASSERT_EQ(CL_SUCCESS,errNum);
			ASSERT_FALSE(k==0);
			clReleaseKernel(k);
		}

		
		{
			cl_kernel k = b3OpenCLUtils::compileCLKernelFromString( m_clContext, m_clDevice, solverSetup2CL, "ReorderContactKernel", &errNum, solverSetup2Prog,additionalMacros );
			ASSERT_EQ(CL_SUCCESS,errNum);
			ASSERT_FALSE(k==0);
			clReleaseKernel(k);
		}
		

		{
			cl_kernel k = b3OpenCLUtils::compileCLKernelFromString( m_clContext, m_clDevice, solverSetup2CL, "CopyConstraintKernel", &errNum, solverSetup2Prog,additionalMacros );
			ASSERT_EQ(CL_SUCCESS,errNum);
			ASSERT_FALSE(k==0);
			clReleaseKernel(k);
		}

		clReleaseProgram(solverSetup2Prog);
	}

	TEST_F(CompileBullet3PgsContactSolverKernels,solveContactCL)
	{
		const char* additionalMacros="";
		cl_int errNum=0;

		cl_program solveContactProg= b3OpenCLUtils::compileCLProgramFromString( m_clContext, m_clDevice, solveContactCL, &errNum,additionalMacros, 0,true);
		ASSERT_EQ(CL_SUCCESS,errNum);
		
	

		{
			cl_kernel k = b3OpenCLUtils::compileCLKernelFromString( m_clContext, m_clDevice, solveContactCL, "BatchSolveKernelContact", &errNum, solveContactProg,additionalMacros );
			ASSERT_EQ(CL_SUCCESS,errNum);
			ASSERT_FALSE(k==0);
			clReleaseKernel(k);
		}

		{
			cl_kernel k  = b3OpenCLUtils::compileCLKernelFromString( m_clContext, m_clDevice, solveContactCL, "solveSingleContactKernel", &errNum, solveContactProg,additionalMacros );
			ASSERT_EQ(CL_SUCCESS,errNum);
			ASSERT_FALSE(k==0);
			clReleaseKernel(k);
		}

		clReleaseProgram(solveContactProg);
	}


	TEST_F(CompileBullet3PgsContactSolverKernels,batchingKernelsCL)
	{
		const char* additionalMacros="";
		cl_int errNum=0;

		cl_program batchingProg = b3OpenCLUtils::compileCLProgramFromString( m_clContext, m_clDevice, batchingKernelsCL, &errNum,additionalMacros, 0,true);
		ASSERT_EQ(CL_SUCCESS,errNum);
		
		{
			cl_kernel k = b3OpenCLUtils::compileCLKernelFromString( m_clContext, m_clDevice, batchingKernelsCL, "CreateBatches", &errNum, batchingProg,additionalMacros );
			ASSERT_EQ(CL_SUCCESS,errNum);
			ASSERT_FALSE(k==0);
			clReleaseKernel(k);
		}

		clReleaseProgram(batchingProg);
	}
		
	TEST_F(CompileBullet3PgsContactSolverKernels,batchingKernelsNewCL)
	{
		const char* additionalMacros="";
		cl_int errNum=0;

		cl_program batchingNewProg = b3OpenCLUtils::compileCLProgramFromString( m_clContext, m_clDevice, batchingKernelsNewCL, &errNum,additionalMacros, 0,true);
		ASSERT_EQ(CL_SUCCESS,errNum);
		
		{
			cl_kernel k  = b3OpenCLUtils::compileCLKernelFromString( m_clContext, m_clDevice, batchingKernelsNewCL, "CreateBatchesNew", &errNum, batchingNewProg,additionalMacros );
			ASSERT_EQ(CL_SUCCESS,errNum);
			ASSERT_FALSE(k==0);
			clReleaseKernel(k);
		}
		clReleaseProgram(batchingNewProg);
	}
	
};
