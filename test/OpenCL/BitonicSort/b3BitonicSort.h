#ifndef B3_BITONIC_SORT_H
#define B3_BITONIC_SORT_H

#include "Bullet3OpenCL/Initialize/b3OpenCLInclude.h"

struct b3BitonicSortInfo
{
	cl_command_queue m_cqCommandQue;
	cl_device_id	dev;

	cl_kernel bitonicSortLocal;
	cl_kernel bitonicSortLocal1;
	cl_kernel bitonicSortMergeGlobal;
	cl_kernel bitonicSortMergeLocal;
	
	unsigned int dir;
	unsigned int localSizeLimit;

	b3BitonicSortInfo()
	{
		dev = 0;
		m_cqCommandQue = 0;
		bitonicSortLocal=0;
		bitonicSortLocal1=0;
		bitonicSortMergeGlobal=0;
		bitonicSortMergeLocal=0;
		dir = 1;
		localSizeLimit = 1024U;
	}
};


void bitonicSortNv(cl_mem pKey, int arrayLength, b3BitonicSortInfo& info);

#endif //B3_BITONIC_SORT_H
