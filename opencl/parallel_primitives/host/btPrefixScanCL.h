
#ifndef BT_PREFIX_SCAN_CL_H
#define BT_PREFIX_SCAN_CL_H

#include "btOpenCLArray.h"
#include "btBufferInfoCL.h"
#include "BulletCommon/b3AlignedObjectArray.h"

class btPrefixScanCL
{
	enum
	{
		BLOCK_SIZE = 128
	};

//	Option m_option;

	cl_command_queue	m_commandQueue;

	cl_kernel m_localScanKernel;
	cl_kernel m_blockSumKernel;
	cl_kernel m_propagationKernel;

	btOpenCLArray<unsigned int>* m_workBuffer;


	public:
		
	btPrefixScanCL(cl_context ctx, cl_device_id device, cl_command_queue queue,int size=0);

	virtual ~btPrefixScanCL();

	void execute(btOpenCLArray<unsigned int>& src, btOpenCLArray<unsigned int>& dst, int n, unsigned int* sum = 0);
	void executeHost(b3AlignedObjectArray<unsigned int>& src, b3AlignedObjectArray<unsigned int>& dst, int n, unsigned int* sum);
};

#endif //BT_PREFIX_SCAN_CL_H
