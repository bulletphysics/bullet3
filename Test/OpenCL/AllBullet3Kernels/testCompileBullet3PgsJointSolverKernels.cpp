
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
		cl_context			m_clContext;
		cl_device_id		m_clDevice;
		cl_command_queue	m_clQueue;
		char*				m_clDeviceName;
		cl_platform_id		m_platformId;

		testCompileBullet3PgsJointSolverKernels()
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


			initCL(preferredDeviceIndex, preferredPlatformIndex, allowCpuOpenCL);
		}

		virtual ~testCompileBullet3PgsJointSolverKernels() 
		{
			// You can do clean-up work that doesn't throw exceptions here.
			exitCL();
		}

		// If the constructor and destructor are not enough for setting up
		// and cleaning up each test, you can define the following methods:

		void initCL(int preferredDeviceIndex, int preferredPlatformIndex, bool allowCpuOpenCL)
		{
			void* glCtx=0;
			void* glDC = 0;
	
	
    
			int ciErrNum = 0;

			cl_device_type deviceType = CL_DEVICE_TYPE_GPU;
			if (allowCpuOpenCL)
				deviceType = CL_DEVICE_TYPE_ALL;

	
	
			//	if (useInterop)
			//	{
			//		m_data->m_clContext = b3OpenCLUtils::createContextFromType(deviceType, &ciErrNum, glCtx, glDC);
			//	} else
			{
				m_clContext = b3OpenCLUtils::createContextFromType(deviceType, &ciErrNum, 0,0,preferredDeviceIndex, preferredPlatformIndex,&m_platformId);
				ASSERT_FALSE(m_clContext==0);
			}
	
	
			ASSERT_EQ(ciErrNum, CL_SUCCESS);
	
			int numDev = b3OpenCLUtils::getNumDevices(m_clContext);
			EXPECT_GT(numDev,0);

			if (numDev>0)
			{
				m_clDevice= b3OpenCLUtils::getDevice(m_clContext,0);
				ASSERT_FALSE(m_clDevice==0);

				m_clQueue = clCreateCommandQueue(m_clContext, m_clDevice, 0, &ciErrNum);
				ASSERT_FALSE(m_clQueue==0);
				
				ASSERT_EQ(ciErrNum, CL_SUCCESS);
        
        
				b3OpenCLDeviceInfo info;
				b3OpenCLUtils::getDeviceInfo(m_clDevice,&info);
				m_clDeviceName = info.m_deviceName;
			}
		}

		void	exitCL()
		{
			clReleaseCommandQueue(m_clQueue);
			clReleaseContext(m_clContext);
		}

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

	TEST_F(testCompileBullet3PgsJointSolverKernels,solveConstraintRowsCL)
	{
		cl_int errNum=0;

		cl_program prog = b3OpenCLUtils::compileCLProgramFromString(m_clContext,m_clDevice,solveConstraintRowsCL,&errNum,"",0,true);
		ASSERT_EQ(errNum,CL_SUCCESS);
				
		{
			cl_kernel k = b3OpenCLUtils::compileCLKernelFromString(m_clContext, m_clDevice,solveConstraintRowsCL, "solveJointConstraintRows",&errNum,prog);
			ASSERT_EQ(errNum,CL_SUCCESS);
			ASSERT_FALSE(k==0);
			clReleaseKernel(k);
		}

		{
			cl_kernel k = b3OpenCLUtils::compileCLKernelFromString(m_clContext,m_clDevice,solveConstraintRowsCL,"initSolverBodies",&errNum,prog);
			ASSERT_EQ(errNum,CL_SUCCESS);
			ASSERT_FALSE(k==0);
			clReleaseKernel(k);
		}
		{
			cl_kernel k = b3OpenCLUtils::compileCLKernelFromString(m_clContext,m_clDevice,solveConstraintRowsCL,"getInfo1Kernel",&errNum,prog);
			ASSERT_EQ(errNum,CL_SUCCESS);
			ASSERT_FALSE(k==0);
			clReleaseKernel(k);
		}
		{
			cl_kernel k = b3OpenCLUtils::compileCLKernelFromString(m_clContext,m_clDevice,solveConstraintRowsCL,"initBatchConstraintsKernel",&errNum,prog);
			ASSERT_EQ(errNum,CL_SUCCESS);
			ASSERT_FALSE(k==0);
			clReleaseKernel(k);
		}
		{
			cl_kernel k= b3OpenCLUtils::compileCLKernelFromString(m_clContext,m_clDevice,solveConstraintRowsCL,"getInfo2Kernel",&errNum,prog);
			ASSERT_EQ(errNum,CL_SUCCESS);
			ASSERT_FALSE(k==0);
			clReleaseKernel(k);
		}
		{
			cl_kernel k = b3OpenCLUtils::compileCLKernelFromString(m_clContext,m_clDevice,solveConstraintRowsCL,"writeBackVelocitiesKernel",&errNum,prog);
			ASSERT_EQ(errNum,CL_SUCCESS);
			ASSERT_FALSE(k==0);
			clReleaseKernel(k);
		}
		{
			cl_kernel k = b3OpenCLUtils::compileCLKernelFromString(m_clContext,m_clDevice,solveConstraintRowsCL,"breakViolatedConstraintsKernel",&errNum,prog);
			ASSERT_EQ(errNum,CL_SUCCESS);
			ASSERT_FALSE(k==0);
			clReleaseKernel(k);
		}

		
		

		clReleaseProgram(prog);
	
	}	
};
