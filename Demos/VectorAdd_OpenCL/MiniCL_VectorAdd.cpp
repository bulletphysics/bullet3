
///VectorAdd sample, from the NVidia JumpStart Guide
///http://developer.download.nvidia.com/OpenCL/NVIDIA_OpenCL_JumpStart_Guide.pdf

///Instead of #include <CL/cl.h> we include <MiniCL/cl.h>
///Apart from this include file, all other code should compile and work on OpenCL compliant implementation


//#define LOAD_FROM_FILE

#ifdef USE_MINICL
	#include "MiniCL/cl.h"
#else //USE_MINICL
	#ifdef __APPLE__
		#include <OpenCL/OpenCL.h>
	#else
		#include <CL/cl.h>
	#endif //__APPLE__
#endif//USE_MINICL

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "LinearMath/btMinMax.h"
#define GRID3DOCL_CHECKERROR(a, b) if((a)!=(b)) { printf("3D GRID OCL Error : %d\n", (a)); btAssert((a) == (b)); }
size_t wgSize;


#ifndef USE_MINICL
#define MSTRINGIFY(A) #A
const char* stringifiedSourceCL = 
#include "VectorAddKernels.cl"
#else
const char* stringifiedSourceCL = "";
#endif




char* loadProgSource(const char* cFilename, const char* cPreamble, size_t* szFinalLength)
{
    // locals 
    FILE* pFileStream = NULL;
    size_t szSourceLength;
	
    // open the OpenCL source code file
	pFileStream = fopen(cFilename, "rb");
	if(pFileStream == 0) 
	{       
		return NULL;
	}
	
    size_t szPreambleLength = strlen(cPreamble);
	
    // get the length of the source code
    fseek(pFileStream, 0, SEEK_END); 
    szSourceLength = ftell(pFileStream);
    fseek(pFileStream, 0, SEEK_SET); 
	
    // allocate a buffer for the source code string and read it in
    char* cSourceString = (char *)malloc(szSourceLength + szPreambleLength + 1); 
    memcpy(cSourceString, cPreamble, szPreambleLength);
    fread((cSourceString) + szPreambleLength, szSourceLength, 1, pFileStream); 
	
    // close the file and return the total length of the combined (preamble + source) string
    fclose(pFileStream);
    if(szFinalLength != 0)
    {
        *szFinalLength = szSourceLength + szPreambleLength;
    }
    cSourceString[szSourceLength + szPreambleLength] = '\0';
	
    return cSourceString;
}

size_t workitem_size[3];

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
    
    clGetDeviceInfo(device, CL_DEVICE_MAX_WORK_ITEM_SIZES, sizeof(workitem_size), &workitem_size, NULL);
    printf(  " CL_DEVICE_MAX_WORK_ITEM_SIZES:\t%zu / %zu / %zu \n", workitem_size[0], workitem_size[1], workitem_size[2]);
    
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

	int actualGlobalSize = iTestN>>3;
	
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


	 cl_uint numPlatforms;
    cl_platform_id platform = NULL;
    cl_int status = clGetPlatformIDs(0, NULL, &numPlatforms);

    if (0 < numPlatforms) 
    {
        cl_platform_id* platforms = new cl_platform_id[numPlatforms];
        status = clGetPlatformIDs(numPlatforms, platforms, NULL);
        
        for (unsigned i = 0; i < numPlatforms; ++i) 
        {
            char pbuf[100];
            status = clGetPlatformInfo(platforms[i],
                                       CL_PLATFORM_VENDOR,
                                       sizeof(pbuf),
                                       pbuf,
                                       NULL);

            platform = platforms[i];
            if (!strcmp(pbuf, "Advanced Micro Devices, Inc.")) 
            {
                break;
            }
        }
        delete[] platforms;
    }

	cl_context_properties cps[3] = 
    {
        CL_CONTEXT_PLATFORM, 
        (cl_context_properties)platform, 
        0
    };

    // Create OpenCL context & context
    cxGPUContext = clCreateContextFromType(cps, CL_DEVICE_TYPE_ALL, NULL, NULL, &ciErr1); //could also be CL_DEVICE_TYPE_GPU
	
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
	::size_t* lengths = (::size_t*) malloc(numDevices * sizeof(::size_t));
	const unsigned char** images = (const unsigned char**) malloc(numDevices * sizeof(const void*));

	for (i = 0; i < numDevices; ++i) {
		images[i] = 0;
		lengths[i] = 0;
	}

	
	// Read the OpenCL kernel in from source file
	const char* cSourceFile = "VectorAddKernels.cl";
	
    printf("loadProgSource (%s)...\n", cSourceFile); 
    const char* cPathAndName = cSourceFile;
#ifdef LOAD_FROM_FILE
	size_t szKernelLength;
    const char* cSourceCL = loadProgSource(cPathAndName, "", &szKernelLength);
#else
	const char* cSourceCL = stringifiedSourceCL;
	size_t szKernelLength = strlen(stringifiedSourceCL);
#endif //LOAD_FROM_FILE


	
    // Create the program
    cpProgram = clCreateProgramWithSource(cxGPUContext, 1, (const char **)&cSourceCL, &szKernelLength, &ciErr1);
    printf("clCreateProgramWithSource...\n"); 
    if (ciErr1 != CL_SUCCESS)
    {
        printf("Error in clCreateProgramWithSource, Line %u in file %s !!!\n\n", __LINE__, __FILE__);
        exit(0);
    }
	
    // Build the program with 'mad' Optimization option
#ifdef MAC
	char* flags = "-cl-mad-enable -DMAC -DGUID_ARG";
#else
	const char* flags = "-DGUID_ARG=";
#endif
    ciErr1 = clBuildProgram(cpProgram, 0, NULL, flags, NULL, NULL);
    printf("clBuildProgram...\n"); 
    if (ciErr1 != CL_SUCCESS)
    {
        printf("Error in clBuildProgram, Line %u in file %s !!!\n\n", __LINE__, __FILE__);
        exit(0);
    }
	
    // Create the kernel
    ckKernel = clCreateKernel(cpProgram, "VectorAdd", &ciErr1);
    printf("clCreateKernel (VectorAdd)...\n"); 
    if (ciErr1 != CL_SUCCESS)
    {
        printf("Error in clCreateKernel, Line %u in file %s !!!\n\n", __LINE__, __FILE__);
		exit(0);
    }
	
	
	cl_int ciErrNum;
	
	ciErrNum = clGetKernelWorkGroupInfo(ckKernel, cdDevices[0], CL_KERNEL_WORK_GROUP_SIZE, sizeof(size_t), &wgSize, NULL);
	if (ciErrNum != CL_SUCCESS)
	{
		printf("cannot get workgroup size\n");
		exit(0);
	}

	

   
    // Set the Argument values
    ciErr1 |= clSetKernelArg(ckKernel, 0, sizeof(cl_mem), (void*)&cmMemObjs[0]);
    ciErr1 |= clSetKernelArg(ckKernel, 1, sizeof(cl_mem), (void*)&cmMemObjs[1]);
    ciErr1 |= clSetKernelArg(ckKernel, 2, sizeof(cl_mem), (void*)&cmMemObjs[2]);

	
	
	int workgroupSize = wgSize;
	if(workgroupSize <= 0)
	{ // let OpenCL library calculate workgroup size
		size_t globalWorkSize[2];
		globalWorkSize[0] = actualGlobalSize;
		globalWorkSize[1] = 1;
	
		// Copy input data from host to GPU and launch kernel 
		ciErr1 |= clEnqueueNDRangeKernel(cqCommandQue, ckKernel, 1, NULL, globalWorkSize, NULL, 0,0,0 );

	}
	else
	{
		size_t localWorkSize[2], globalWorkSize[2];
		workgroupSize = btMin(workgroupSize, actualGlobalSize);
		int num_t = actualGlobalSize / workgroupSize;
		int num_g = num_t * workgroupSize;
		if(num_g < actualGlobalSize)
		{
			num_t++;
			//this can cause problems -> processing outside of the buffer
			//make sure to check kernel
		}

		size_t globalThreads[] = {num_t * workgroupSize};
		size_t localThreads[] = {workgroupSize};


		localWorkSize[0]  = workgroupSize;
		globalWorkSize[0] = num_t * workgroupSize;
		localWorkSize[1] = 1;
		globalWorkSize[1] = 1;

		// Copy input data from host to GPU and launch kernel 
		ciErr1 |= clEnqueueNDRangeKernel(cqCommandQue, ckKernel, 1, NULL, globalThreads, localThreads, 0, NULL, NULL);

	}
	
	if (ciErrNum != CL_SUCCESS)
	{
		printf("cannot clEnqueueNDRangeKernel\n");
		exit(0);
	}
	
	clFinish(cqCommandQue);
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
	printf("Press ENTER to quit\n");
	getchar();
}


#ifdef USE_MINICL

#include "MiniCL/cl_MiniCL_Defs.h"

extern "C"
{
	///GUID_ARG is only used by MiniCL to pass in the guid used by its get_global_id implementation


	#define MSTRINGIFY(A) A
	#include "VectorAddKernels.cl"
	#undef MSTRINGIFY
}
MINICL_REGISTER(VectorAdd)
#endif//USE_MINICL
