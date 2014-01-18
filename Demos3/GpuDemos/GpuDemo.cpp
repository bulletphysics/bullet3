#include "GpuDemo.h"
#include "GpuDemoInternalData.h"
#include "Bullet3Common/b3Scalar.h"
#include "Bullet3OpenCL/Initialize/b3OpenCLUtils.h"
#include "OpenGLWindow/ShapeData.h"
#include "OpenGLWindow/GLInstancingRenderer.h"
#include "OpenGLWindow/OpenGLInclude.h"
bool gAllowCpuOpenCL = false;
#include "stb_image/stb_image.h"
GpuDemo::GpuDemo()
:m_clData(0)
{
	m_clData = new GpuDemoInternalData();
}

GpuDemo::~GpuDemo()
{
	if (m_clData)
	{
		b3Assert(m_clData->m_clInitialized==false);
		
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

	cl_device_type deviceType = CL_DEVICE_TYPE_GPU;
	if (gAllowCpuOpenCL)
		deviceType = CL_DEVICE_TYPE_ALL;

	
	
	//	if (useInterop)
	//	{
	//		m_data->m_clContext = b3OpenCLUtils::createContextFromType(deviceType, &ciErrNum, glCtx, glDC);
	//	} else
	{
		m_clData->m_clContext = b3OpenCLUtils::createContextFromType(deviceType, &ciErrNum, 0,0,preferredDeviceIndex, preferredPlatformIndex,&m_clData->m_platformId);
	}
	
	
	oclCHECKERROR(ciErrNum, CL_SUCCESS);
	
	int numDev = b3OpenCLUtils::getNumDevices(m_clData->m_clContext);
	
	if (numDev>0)
	{
		m_clData->m_clDevice= b3OpenCLUtils::getDevice(m_clData->m_clContext,0);
		m_clData->m_clQueue = clCreateCommandQueue(m_clData->m_clContext, m_clData->m_clDevice, 0, &ciErrNum);
		oclCHECKERROR(ciErrNum, CL_SUCCESS);
        
        
		b3OpenCLDeviceInfo info;
		b3OpenCLUtils::getDeviceInfo(m_clData->m_clDevice,&info);
		m_clData->m_clDeviceName = info.m_deviceName;
		m_clData->m_clInitialized = true;
		
	}
	
}

GpuDemoInternalData*	GpuDemo::getInternalData()
{
	return m_clData;
}

int	GpuDemo::registerGraphicsSphereShape(const ConstructionInfo& ci, float radius, bool usePointSprites, int largeSphereThreshold, int mediumSphereThreshold)
{
	
	int strideInBytes = 9*sizeof(float);
	
	int graphicsShapeIndex = -1;
	
	if (radius>=largeSphereThreshold)
	{
		int numVertices = sizeof(detailed_sphere_vertices)/strideInBytes;
		int numIndices = sizeof(detailed_sphere_indices)/sizeof(int);
		graphicsShapeIndex = ci.m_instancingRenderer->registerShape(&detailed_sphere_vertices[0],numVertices,detailed_sphere_indices,numIndices);
	} else
	{
		
		if (usePointSprites)
		{
			int numVertices = sizeof(point_sphere_vertices)/strideInBytes;
			int numIndices = sizeof(point_sphere_indices)/sizeof(int);
			graphicsShapeIndex = ci.m_instancingRenderer->registerShape(&point_sphere_vertices[0],numVertices,point_sphere_indices,numIndices,B3_GL_POINTS);
		} else
		{
			if (radius>=mediumSphereThreshold)
			{
				int numVertices = sizeof(medium_sphere_vertices)/strideInBytes;
				int numIndices = sizeof(medium_sphere_indices)/sizeof(int);
				graphicsShapeIndex = ci.m_instancingRenderer->registerShape(&medium_sphere_vertices[0],numVertices,medium_sphere_indices,numIndices);
			} else
			{
				int numVertices = sizeof(low_sphere_vertices)/strideInBytes;
				int numIndices = sizeof(low_sphere_indices)/sizeof(int);
				graphicsShapeIndex = ci.m_instancingRenderer->registerShape(&low_sphere_vertices[0],numVertices,low_sphere_indices,numIndices);
			}
		}
	}
	return graphicsShapeIndex;
}


unsigned char* GpuDemo::loadImage(const char* fileName, int& width, int& height, int& n)
{
		unsigned char *data = stbi_load(fileName, &width, &height, &n, 0);
		return data;
}
