#include "GpuDemo.h"
#include "GpuDemoInternalData.h"
#include "BulletCommon/btScalar.h"
#include "basic_initialize/btOpenCLUtils.h"


GpuDemo::GpuDemo()
:m_clData(0)
{
	m_clData = new GpuDemoInternalData();
}

GpuDemo::~GpuDemo()
{
	if (m_clData)
	{
		btAssert(m_clData->m_clInitialized==false);
		
		delete m_clData;
	}
}

void GpuDemo::exitCL()
{
	if (m_clData->m_clInitialized)
	{
		clReleaseCommandQueue(m_clData->m_clQueue);
		clReleaseContext(m_clData->m_clContext);
		m_clData->m_clInitialized = false;
	}
	
}

void GpuDemo::initCL(int preferredDeviceIndex, int preferredPlatformIndex)
{
	void* glCtx=0;
	void* glDC = 0;
	
	
    
	int ciErrNum = 0;
	//#ifdef CL_PLATFORM_INTEL
		cl_device_type deviceType = CL_DEVICE_TYPE_ALL;
	//#else
	//cl_device_type deviceType = CL_DEVICE_TYPE_CPU;
	//#endif
	
	cl_platform_id platformId;
	
	//	if (useInterop)
	//	{
	//		m_data->m_clContext = btOpenCLUtils::createContextFromType(deviceType, &ciErrNum, glCtx, glDC);
	//	} else
	{
		m_clData->m_clContext = btOpenCLUtils::createContextFromType(deviceType, &ciErrNum, 0,0,preferredDeviceIndex, preferredPlatformIndex,&platformId);
		btOpenCLUtils::printPlatformInfo(platformId);
	}
	
	
	oclCHECKERROR(ciErrNum, CL_SUCCESS);
	
	int numDev = btOpenCLUtils::getNumDevices(m_clData->m_clContext);
	
	if (numDev>0)
	{
		m_clData->m_clDevice= btOpenCLUtils::getDevice(m_clData->m_clContext,0);
		m_clData->m_clQueue = clCreateCommandQueue(m_clData->m_clContext, m_clData->m_clDevice, 0, &ciErrNum);
		oclCHECKERROR(ciErrNum, CL_SUCCESS);
        
        btOpenCLUtils::printDeviceInfo(m_clData->m_clDevice);
		btOpenCLDeviceInfo info;
		btOpenCLUtils::getDeviceInfo(m_clData->m_clDevice,&info);
		m_clData->m_clDeviceName = info.m_deviceName;
		m_clData->m_clInitialized = true;
		
	}
	
}


