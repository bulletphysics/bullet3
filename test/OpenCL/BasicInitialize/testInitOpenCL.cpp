
#include <gtest/gtest.h>
#include "Bullet3Common/b3Logging.h"

#include "Bullet3OpenCL/Initialize/b3OpenCLUtils.h"
TEST(b3OpenCLUtils, getNumPlatforms)
{
	int numPlatforms = b3OpenCLUtils::getNumPlatforms();
	ASSERT_GT(numPlatforms, 0);
}

TEST(b3OpenCLUtils, getSdkVendorName)
{
	const char* vendorSDK = b3OpenCLUtils::getSdkVendorName();
	b3Printf("getSdkVendorName=%s\n", vendorSDK);
	ASSERT_FALSE(vendorSDK == NULL);
}

TEST(b3OpenCLUtils, getPlatformInfo)
{
	int numPlatforms = b3OpenCLUtils::getNumPlatforms();
	ASSERT_GT(numPlatforms, 0);

	b3Printf("Num Platforms = %d\n", numPlatforms);
	for (int i = 0; i < numPlatforms; i++)
	{
		cl_platform_id platform = b3OpenCLUtils::getPlatform(i);
		ASSERT_FALSE(platform == NULL);

		b3OpenCLPlatformInfo platformInfo;
		b3OpenCLUtils::getPlatformInfo(platform, &platformInfo);
		ASSERT_FALSE(platformInfo.m_platformName == NULL);
		ASSERT_FALSE(platformInfo.m_platformVendor == NULL);
		ASSERT_FALSE(platformInfo.m_platformVersion == NULL);
	}
}

TEST(b3OpenCLUtils, createContextFromPlatform)
{
	int numPlatforms = b3OpenCLUtils::getNumPlatforms();
	b3Printf("Num Platforms = %d\n", numPlatforms);

	cl_device_type deviceType = CL_DEVICE_TYPE_ALL;
	int ciErrNum = 0;

	for (int i = 0; i < numPlatforms; i++)
	{
		cl_platform_id platform = b3OpenCLUtils::getPlatform(i);
		b3OpenCLPlatformInfo platformInfo;
		b3OpenCLUtils::getPlatformInfo(platform, &platformInfo);
		b3Printf("--------------------------------\n");
		b3Printf("Platform info for platform nr %d:\n", i);
		b3Printf("  CL_PLATFORM_VENDOR: \t\t\t%s\n", platformInfo.m_platformVendor);
		b3Printf("  CL_PLATFORM_NAME: \t\t\t%s\n", platformInfo.m_platformName);
		b3Printf("  CL_PLATFORM_VERSION: \t\t\t%s\n", platformInfo.m_platformVersion);

		cl_context ctx = b3OpenCLUtils::createContextFromPlatform(platform, deviceType, &ciErrNum);
		ASSERT_FALSE(ctx == 0);
		ASSERT_EQ(CL_SUCCESS, ciErrNum);
		clReleaseContext(ctx);
	}
}

TEST(b3OpenCLUtils, getDeviceAndQueue)
{
	int numPlatforms = b3OpenCLUtils::getNumPlatforms();
	b3Printf("Num Platforms = %d\n", numPlatforms);
	cl_device_type deviceType = CL_DEVICE_TYPE_ALL;
	int ciErrNum = 0;
	for (int i = 0; i < numPlatforms; i++)
	{
		cl_platform_id platform = b3OpenCLUtils::getPlatform(i);
		b3OpenCLPlatformInfo platformInfo;
		b3OpenCLUtils::getPlatformInfo(platform, &platformInfo);
		b3Printf("--------------------------------\n");
		b3Printf("Platform info for platform nr %d:\n", i);
		b3Printf("  CL_PLATFORM_VENDOR: \t\t\t%s\n", platformInfo.m_platformVendor);
		b3Printf("  CL_PLATFORM_NAME: \t\t\t%s\n", platformInfo.m_platformName);
		b3Printf("  CL_PLATFORM_VERSION: \t\t\t%s\n", platformInfo.m_platformVersion);
		cl_context ctx = b3OpenCLUtils::createContextFromPlatform(platform, deviceType, &ciErrNum);
		ASSERT_FALSE(ctx == 0);
		ASSERT_EQ(CL_SUCCESS, ciErrNum);
		int numDevices = b3OpenCLUtils::getNumDevices(ctx);
		ASSERT_GT(numDevices, 0);

		b3Printf("Num Devices = %d\n", numDevices);
		for (int j = 0; j < numDevices; j++)
		{
			cl_device_id device = b3OpenCLUtils::getDevice(ctx, j);
			b3OpenCLDeviceInfo devInfo;
			b3OpenCLUtils::getDeviceInfo(device, &devInfo);
			ASSERT_GT(devInfo.m_clockFrequency, 0);
			ASSERT_GT(devInfo.m_addressBits, 0);
			ASSERT_GT(devInfo.m_computeUnits, 0);
			ASSERT_GT(devInfo.m_constantBufferSize, 0);
			ASSERT_FALSE(devInfo.m_deviceName == NULL);
			ASSERT_FALSE(devInfo.m_deviceVendor == NULL);
			ASSERT_FALSE(devInfo.m_driverVersion == NULL);
			ASSERT_GT(devInfo.m_globalMemSize, 0);

			b3OpenCLUtils::printDeviceInfo(device);

			cl_command_queue q = clCreateCommandQueue(ctx, device, 0, &ciErrNum);
			ASSERT_FALSE(q == 0);

			clReleaseCommandQueue(q);
			q = 0;
		}
		clReleaseContext(ctx);
	}
}
