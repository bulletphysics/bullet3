#ifndef BT_OPENCL_SUPPORT_HPP
#define BT_OPENCL_SUPPORT_HPP

// OpenCL support
#include <CL/cl.hpp>

namespace BTAcceleratedSoftBody
{
	class OpenCLSupportHelper
	{
	private:
		cl::Context m_context;
		std::vector<cl::Device> m_devices;
		cl::CommandQueue m_queue;
	public:
		OpenCLSupportHelper()
		{
		}

		virtual ~OpenCLSupportHelper()
		{
		}

		cl::Device getDevice()
		{
			return m_devices[0];
		}

		cl::CommandQueue getCommandQueue()
		{
			return m_queue;
		}

		cl::Context getContext()
		{
			return m_context;
		}

		bool InitOpenCLDevice()
		{
			cl_int err;

			std::vector<cl::Platform> platforms;
			err = cl::Platform::get(&platforms);
			checkErr(platforms.size() != 0 ? CL_SUCCESS : -1, "Platform::get()");

			std::string platformVendor;
			platforms[0].getInfo(CL_PLATFORM_VENDOR, &platformVendor);
			//std::cout << "Platform is by: " << platformVendor << "\n";

			intptr_t properties[] = {
				CL_CONTEXT_PLATFORM, (intptr_t)platforms[0](),
				0, 0
			};
			m_context = cl::Context(
				CL_DEVICE_TYPE_GPU, 
				properties, 
				NULL, 
				NULL, 
				&err);

			if (err != CL_SUCCESS)
			{
				btAssert( "Context::Context()" );
			}

			m_devices = m_context.getInfo<CL_CONTEXT_DEVICES>();
			if( m_devices.size() <= 0 ) 
			{
				btAssert( "devices.size() > 0" );
			}
		
			m_queue = cl::CommandQueue(m_context, m_devices[0], 0, &err);
		    if (err != CL_SUCCESS) 
			{
				btAssert( "CommandQueue::CommandQueue()");
			}
		}
	};


} // namespace BTAcceleratedSoftBody

#endif // #ifndef BT_OPENCL_SUPPORT_HPP