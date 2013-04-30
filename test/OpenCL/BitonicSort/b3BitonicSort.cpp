
#include "b3BitonicSort.h"
#include "Bullet3Common/b3Scalar.h"


//Note: logically shared with BitonicSort OpenCL code!
// TODO : get parameter from OpenCL and pass it to kernel (needed for platforms other than NVIDIA)

void bitonicSortNv(cl_mem pKey, int arrayLength, b3BitonicSortInfo& info)
{
	
    if(arrayLength < 2)
        return;
    //Only power-of-two array lengths are supported so far
    info.dir = (info.dir != 0);
    cl_int ciErrNum;
    size_t localWorkSize, globalWorkSize;
    if(arrayLength <= info.localSizeLimit)
    {
        b3Assert( ( arrayLength) % info.localSizeLimit == 0);
        //Launch bitonicSortLocal
		ciErrNum  = clSetKernelArg(info.bitonicSortLocal, 0,   sizeof(cl_mem), (void *)&pKey);
        ciErrNum |= clSetKernelArg(info.bitonicSortLocal, 1,  sizeof(cl_uint), (void *)&arrayLength);
        ciErrNum |= clSetKernelArg(info.bitonicSortLocal, 2,  sizeof(cl_uint), (void *)&info.dir);
        oclCHECKERROR(ciErrNum, CL_SUCCESS);

        localWorkSize  = info.localSizeLimit / 2;
        globalWorkSize =  arrayLength / 2;
        ciErrNum = clEnqueueNDRangeKernel(info.m_cqCommandQue, info.bitonicSortLocal, 1, NULL, &globalWorkSize, &localWorkSize, 0, NULL, NULL);
        oclCHECKERROR(ciErrNum, CL_SUCCESS);
    }
    else
    {
        //Launch bitonicSortLocal1
        ciErrNum  = clSetKernelArg(info.bitonicSortLocal1, 0,  sizeof(cl_mem), (void *)&pKey);
        oclCHECKERROR(ciErrNum, CL_SUCCESS);

        localWorkSize  = info.localSizeLimit / 2;
        globalWorkSize =  arrayLength / 2;
        ciErrNum = clEnqueueNDRangeKernel(info.m_cqCommandQue, info.bitonicSortLocal1, 1, NULL, &globalWorkSize, &localWorkSize, 0, NULL, NULL);
        oclCHECKERROR(ciErrNum, CL_SUCCESS);

        for(unsigned int size = 2 * info.localSizeLimit; size <= arrayLength; size <<= 1)
        {
            for(unsigned stride = size / 2; stride > 0; stride >>= 1)
            {
                if(stride >= info.localSizeLimit)
                {
                    //Launch bitonicMergeGlobal
                    ciErrNum  = clSetKernelArg(info.bitonicSortMergeGlobal, 0,  sizeof(cl_mem), (void *)&pKey);
                    ciErrNum |= clSetKernelArg(info.bitonicSortMergeGlobal, 1, sizeof(cl_uint), (void *)&arrayLength);
                    ciErrNum |= clSetKernelArg(info.bitonicSortMergeGlobal, 2, sizeof(cl_uint), (void *)&size);
                    ciErrNum |= clSetKernelArg(info.bitonicSortMergeGlobal, 3, sizeof(cl_uint), (void *)&stride);
                    ciErrNum |= clSetKernelArg(info.bitonicSortMergeGlobal, 4, sizeof(cl_uint), (void *)&info.dir);
					oclCHECKERROR(ciErrNum, CL_SUCCESS);

                    localWorkSize  = info.localSizeLimit / 4;
                    globalWorkSize =  arrayLength / 2;

                    ciErrNum = clEnqueueNDRangeKernel(info.m_cqCommandQue, info.bitonicSortMergeGlobal, 1, NULL, &globalWorkSize, &localWorkSize, 0, NULL, NULL);
					oclCHECKERROR(ciErrNum, CL_SUCCESS);
                }
                else
                {
                    //Launch bitonicMergeLocal
					ciErrNum  = clSetKernelArg(info.bitonicSortMergeLocal, 0,  sizeof(cl_mem), (void *)&pKey);
                    ciErrNum |= clSetKernelArg(info.bitonicSortMergeLocal, 1, sizeof(cl_uint), (void *)&arrayLength);
                    ciErrNum |= clSetKernelArg(info.bitonicSortMergeLocal, 2, sizeof(cl_uint), (void *)&stride);
                    ciErrNum |= clSetKernelArg(info.bitonicSortMergeLocal, 3, sizeof(cl_uint), (void *)&size);
                    ciErrNum |= clSetKernelArg(info.bitonicSortMergeLocal, 4, sizeof(cl_uint), (void *)&info.dir);
					oclCHECKERROR(ciErrNum, CL_SUCCESS);

                    localWorkSize  = info.localSizeLimit / 2;
                    globalWorkSize =  arrayLength / 2;

                    ciErrNum = clEnqueueNDRangeKernel(info.m_cqCommandQue, info.bitonicSortMergeLocal, 1, NULL, &globalWorkSize, &localWorkSize, 0, NULL, NULL);
					oclCHECKERROR(ciErrNum, CL_SUCCESS);
                    break;
                }
            }
        }
    }
}