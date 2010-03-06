
///VectorAdd sample, from the NVidia JumpStart Guide
///http://developer.download.nvidia.com/OpenCL/NVIDIA_OpenCL_JumpStart_Guide.pdf

///Instead of #include <CL/cl.h> we include <MiniCL/cl.h>
///Apart from this include file, all other code should compile and work on OpenCL compliant implementation

#define USE_MINICL 1
#ifdef USE_MINICL
#include "MiniCL/cl.h"
#else //USE_MINICL
#include <CL/cl.h>
#endif//USE_MINICL

#include <stdio.h>
#include <math.h>
#include <stdlib.h>


void printDevInfo(cl_device_id device)
{
    char device_string[1024];
	
    clGetDeviceInfo(device, CL_DEVICE_NAME, sizeof(device_string), &device_string, NULL);
    printf(  " Device %s:\n", device_string);

    // CL_DEVICE_INFO
    cl_device_type type;
    clGetDeviceInfo(device, CL_DEVICE_TYPE, sizeof(type), &type, NULL);
    if( type & CL_DEVICE_TYPE_CPU )
        printf(" CL_DEVICE_TYPE:\t\t%s\n", "CL_DEVICE_TYPE_CPU");
    if( type & CL_DEVICE_TYPE_GPU )
        printf(  " CL_DEVICE_TYPE:\t\t%s\n", "CL_DEVICE_TYPE_GPU");
    if( type & CL_DEVICE_TYPE_ACCELERATOR )
        printf(  " CL_DEVICE_TYPE:\t\t%s\n", "CL_DEVICE_TYPE_ACCELERATOR");
    if( type & CL_DEVICE_TYPE_DEFAULT )
        printf(  " CL_DEVICE_TYPE:\t\t%s\n", "CL_DEVICE_TYPE_DEFAULT");
    
    // CL_DEVICE_MAX_COMPUTE_UNITS
    cl_uint compute_units;
    clGetDeviceInfo(device, CL_DEVICE_MAX_COMPUTE_UNITS, sizeof(compute_units), &compute_units, NULL);
    printf(  " CL_DEVICE_MAX_COMPUTE_UNITS:\t%d\n", compute_units);

    // CL_DEVICE_MAX_WORK_GROUP_SIZE
    size_t workitem_size[3];
    clGetDeviceInfo(device, CL_DEVICE_MAX_WORK_ITEM_SIZES, sizeof(workitem_size), &workitem_size, NULL);
    printf(  " CL_DEVICE_MAX_WORK_ITEM_SIZES:\t%d / %d / %d \n", workitem_size[0], workitem_size[1], workitem_size[2]);
    
}




// Main function 
// *********************************************************************
int main(int argc, char **argv)
{
	void *srcA, *srcB, *dst;        // Host buffers for OpenCL test
    cl_context cxGPUContext;       // OpenCL context
    cl_command_queue cqCommandQue;  // OpenCL command que
    cl_device_id* cdDevices;        // OpenCL device list    
    cl_program cpProgram;           // OpenCL program
    cl_kernel ckKernel;             // OpenCL kernel
    cl_mem cmMemObjs[3];            // OpenCL memory buffer objects:  3 for device
    size_t szGlobalWorkSize[1];     // 1D var for Total # of work items
    size_t szLocalWorkSize[1];		// 1D var for # of work items in the work group	
    size_t szParmDataBytes;			// Byte size of context information
    cl_int ciErr1, ciErr2;			// Error code var
    int iTestN = 100000 * 8;		// Size of Vectors to process

    // set Global and Local work size dimensions
    szGlobalWorkSize[0] = iTestN >> 3;  // do 8 computations per work item
    szLocalWorkSize[0]= iTestN>>3;


    // Allocate and initialize host arrays
    srcA = (void *)malloc (sizeof(cl_float) * iTestN);
    srcB = (void *)malloc (sizeof(cl_float) * iTestN);
    dst = (void *)malloc (sizeof(cl_float) * iTestN);

	int i;

	// Initialize arrays with some values
	for (i=0;i<iTestN;i++)
	{
		((cl_float*)srcA)[i] = cl_float(i);
		((cl_float*)srcB)[i] = 2;
		((cl_float*)dst)[i]=-1;
	}

    // Create OpenCL context & context
    cxGPUContext = clCreateContextFromType(0, CL_DEVICE_TYPE_CPU, NULL, NULL, &ciErr1); //could also be CL_DEVICE_TYPE_GPU
	
    // Query all devices available to the context
    ciErr1 |= clGetContextInfo(cxGPUContext, CL_CONTEXT_DEVICES, 0, NULL, &szParmDataBytes);
    cdDevices = (cl_device_id*)malloc(szParmDataBytes);
    ciErr1 |= clGetContextInfo(cxGPUContext, CL_CONTEXT_DEVICES, szParmDataBytes, cdDevices, NULL);
	if (cdDevices)
	{
		printDevInfo(cdDevices[0]);
	}

    // Create a command queue for first device the context reported
    cqCommandQue = clCreateCommandQueue(cxGPUContext, cdDevices[0], 0, &ciErr2);
    ciErr1 |= ciErr2; 

    // Allocate the OpenCL source and result buffer memory objects on the device GMEM
    cmMemObjs[0] = clCreateBuffer(cxGPUContext, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, sizeof(cl_float8) * szGlobalWorkSize[0], srcA, &ciErr2);
    ciErr1 |= ciErr2;
    cmMemObjs[1] = clCreateBuffer(cxGPUContext, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, sizeof(cl_float8) * szGlobalWorkSize[0], srcB, &ciErr2);
    ciErr1 |= ciErr2;
    cmMemObjs[2] = clCreateBuffer(cxGPUContext, CL_MEM_WRITE_ONLY, sizeof(cl_float8) * szGlobalWorkSize[0], NULL, &ciErr2);
    ciErr1 |= ciErr2;

///create kernels from binary
	int numDevices = 1;
	cl_int err;
	::size_t* lengths = (::size_t*) malloc(numDevices * sizeof(::size_t));
	const unsigned char** images = (const unsigned char**) malloc(numDevices * sizeof(const void*));

	for (i = 0; i < numDevices; ++i) {
		images[i] = 0;
		lengths[i] = 0;
	}

	cpProgram = clCreateProgramWithBinary(cxGPUContext, numDevices,cdDevices,lengths, images, 0, &err);

	// Build the executable program from a binary
	ciErr1 |= clBuildProgram(cpProgram, 0, NULL, NULL, NULL, NULL);

    // Create the kernel
    ckKernel = clCreateKernel(cpProgram, "VectorAdd", &ciErr1);
    
    // Set the Argument values
    ciErr1 |= clSetKernelArg(ckKernel, 0, sizeof(cl_mem), (void*)&cmMemObjs[0]);
    ciErr1 |= clSetKernelArg(ckKernel, 1, sizeof(cl_mem), (void*)&cmMemObjs[1]);
    ciErr1 |= clSetKernelArg(ckKernel, 2, sizeof(cl_mem), (void*)&cmMemObjs[2]);

    // Copy input data from host to GPU and launch kernel 
    ciErr1 |= clEnqueueNDRangeKernel(cqCommandQue, ckKernel, 1, NULL, szGlobalWorkSize, szLocalWorkSize, 0, NULL, NULL);

    // Read back results and check accumulated errors
    ciErr1 |= clEnqueueReadBuffer(cqCommandQue, cmMemObjs[2], CL_TRUE, 0, sizeof(cl_float8) * szGlobalWorkSize[0], dst, 0, NULL, NULL);

    // Release kernel, program, and memory objects
	// NOTE:  Most properly this should be done at any of the exit points above, but it is omitted elsewhere for clarity.
    free(cdDevices);
	clReleaseKernel(ckKernel);  
    clReleaseProgram(cpProgram);
    clReleaseCommandQueue(cqCommandQue);
    clReleaseContext(cxGPUContext);


    // print the results
    int iErrorCount = 0;
    for (i = 0; i < iTestN; i++) 
    {
		if (((float*)dst)[i] != ((float*)srcA)[i]+((float*)srcB)[i])
			iErrorCount++;
    }
	
	if (iErrorCount)
	{
		printf("MiniCL validation FAILED\n");
	} else
	{
		printf("MiniCL validation SUCCESSFULL\n");
	}
    // Free host memory, close log and return success
	for (i = 0; i < 3; i++)
    {
        clReleaseMemObject(cmMemObjs[i]);
    }

    free(srcA); 
    free(srcB);
    free (dst);
}


#ifdef USE_MINICL
#include "MiniCL/cl_MiniCL_Defs.h"
extern "C"
{
	#include "VectorAddKernels.cl"
}
MINICL_REGISTER(VectorAdd)
#endif//USE_MINICL