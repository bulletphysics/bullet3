#ifndef COMMON_MULTI_BODY_SETUP_H
#define COMMON_MULTI_BODY_SETUP_H

#include "../CommonInterfaces/CommonExampleInterface.h"
#include "../CommonInterfaces/CommonGUIHelperInterface.h"
#include "../CommonInterfaces/CommonRenderInterface.h"
#include "../CommonInterfaces/CommonGraphicsAppInterface.h"
#include "../CommonInterfaces/CommonWindowInterface.h"
#include "../CommonInterfaces/CommonCameraInterface.h"

#include "GpuDemoInternalData.h"
#include "Bullet3Common/b3Scalar.h"
#include "Bullet3OpenCL/Initialize/b3OpenCLUtils.h"

struct CommonOpenCLBase : public CommonExampleInterface
{
	struct GUIHelperInterface* m_guiHelper;
	struct GpuDemoInternalData* m_clData;

	CommonOpenCLBase(GUIHelperInterface* helper)
		: m_guiHelper(helper),
		  m_clData(0)
	{
		m_clData = new GpuDemoInternalData();
	}

	virtual ~CommonOpenCLBase()
	{
		delete m_clData;
		m_clData = 0;
	}

	virtual void stepSimulation(float deltaTime)
	{
	}

	virtual void initCL(int preferredDeviceIndex, int preferredPlatformIndex)
	{
		//	void* glCtx=0;
		//	void* glDC = 0;

		int ciErrNum = 0;

		cl_device_type deviceType = CL_DEVICE_TYPE_GPU;
		//if (gAllowCpuOpenCL)
		//	deviceType = CL_DEVICE_TYPE_ALL;

		//	if (useInterop)
		//	{
		//		m_data->m_clContext = b3OpenCLUtils::createContextFromType(deviceType, &ciErrNum, glCtx, glDC);
		//	} else
		{
			m_clData->m_clContext = b3OpenCLUtils::createContextFromType(deviceType, &ciErrNum, 0, 0, preferredDeviceIndex, preferredPlatformIndex, &m_clData->m_platformId);
		}

		oclCHECKERROR(ciErrNum, CL_SUCCESS);

		int numDev = b3OpenCLUtils::getNumDevices(m_clData->m_clContext);

		if (numDev > 0)
		{
			m_clData->m_clDevice = b3OpenCLUtils::getDevice(m_clData->m_clContext, 0);
			m_clData->m_clQueue = clCreateCommandQueue(m_clData->m_clContext, m_clData->m_clDevice, 0, &ciErrNum);
			oclCHECKERROR(ciErrNum, CL_SUCCESS);

			b3OpenCLDeviceInfo info;
			b3OpenCLUtils::getDeviceInfo(m_clData->m_clDevice, &info);
			m_clData->m_clDeviceName = info.m_deviceName;
			m_clData->m_clInitialized = true;
		}
	}

	virtual void exitCL()
	{
		if (m_clData && m_clData->m_clInitialized)
		{
			clReleaseCommandQueue(m_clData->m_clQueue);
			clReleaseContext(m_clData->m_clContext);
			m_clData->m_clInitialized = false;
		}
	}

	virtual void renderScene()
	{
		if (m_guiHelper->getRenderInterface())
		{
			m_guiHelper->getRenderInterface()->renderScene();
		}
	}

	virtual void physicsDebugDraw(int debugDrawFlags)
	{
	}

	virtual bool keyboardCallback(int key, int state)
	{
		return false;  //don't handle this key
	}

	virtual bool mouseMoveCallback(float x, float y)
	{
		return false;
	}

	virtual bool mouseButtonCallback(int button, int state, float x, float y)
	{
		CommonRenderInterface* renderer = m_guiHelper->getRenderInterface();

		if (!renderer)
		{
			b3Assert(0);
			return false;
		}

		CommonWindowInterface* window = m_guiHelper->getAppInterface()->m_window;

		if (state == 1)
		{
			if (button == 0 && (!window->isModifierKeyPressed(B3G_ALT) && !window->isModifierKeyPressed(B3G_CONTROL)))
			{
				/*btVector3 camPos;
				renderer->getActiveCamera()->getCameraPosition(camPos);

				btVector3 rayFrom = camPos;
				btVector3 rayTo = getRayTo(int(x),int(y));

				pickBody(rayFrom, rayTo);
				*/
			}
		}
		else
		{
			if (button == 0)
			{
				//				removePickingConstraint();
				//remove p2p
			}
		}

		//printf("button=%d, state=%d\n",button,state);
		return false;
	}
};

#endif  //COMMON_MULTI_BODY_SETUP_H
