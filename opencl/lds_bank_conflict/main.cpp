//Adapted from CUDA to OpenCL by Erwin Coumans
//See http://bitbucket.org/erwincoumans/opencl_course

// Copyright 2012 NVIDIA Corporation
// 
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// 
//     http://www.apache.org/licenses/LICENSE-2.0
// 
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include "b3OpenCLUtils.h"
#include "../parallel_primitives/host/b3OpenCLArray.h"
#include "../parallel_primitives/host/b3LauncherCL.h"
#include "Bullet3Common/b3Quickprof.h"
#include "../parallel_primitives/host/b3FillCL.h"
#include "Bullet3Common/b3CommandLineArgs.h"

#include <string.h>
#include <stdio.h>
#include <assert.h>

//make sure to update the same #define in the opencl/lds_bank_conflict/lds_kernels.cl
const int TILE_DIM = 32;
const int BLOCK_ROWS = 8;
const int NUM_REPS = 100;

// Check errors and print GB/s
void postprocess(const float *ref, const float *res, int n, float ms)
{
  bool passed = true;
  for (int i = 0; i < n; i++)
	if (res[i] != ref[i]) {
	  printf("\nError: at res[%d] got %f but expected %f\n", i, res[i], ref[i]);
	  printf("%25s\n", "*** FAILED ***");
	  passed = false;
	  break;
	}
  if (passed)
	printf("%20.2f\n", 2 * n * sizeof(float) * 1e-6 * NUM_REPS / ms );
}

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

int main(int argc, char **argv)
{
	printf("Use --deviceId=<id> or --platformId=<id> to override OpenCL device\n");
	b3CommandLineArgs args(argc,argv);

	const int nx = 1024;
	const int ny = 1024;
 
	const int mem_size = nx*ny*sizeof(float);
	const int num_elements = nx*ny;
	b3Clock clock;
	double startEvent=0.f;
	double stopEvent=0.f;

	int localSizeX = TILE_DIM;
	int localSizeY = BLOCK_ROWS;

	int numThreadsX = (nx/TILE_DIM)*TILE_DIM;
	int numThreadsY = (ny/TILE_DIM)*BLOCK_ROWS;

	int gridX = numThreadsX / localSizeX;
	int gridY = numThreadsY / localSizeY;

	int ciErrNum = 0;
	int preferred_device = -1;
	int preferred_platform = -1;
	args.GetCmdLineArgument("deviceId",preferred_device);
	args.GetCmdLineArgument("platformId",preferred_platform);


	cl_platform_id		platformId=0;
	cl_context			ctx=0;
	cl_command_queue	queue=0;
	cl_device_id		device=0;
	cl_kernel			copyKernel=0;
	cl_kernel			copySharedMemKernel=0;
	cl_kernel			transposeNaiveKernel = 0;
	cl_kernel			transposeCoalescedKernel = 0;
	cl_kernel			transposeNoBankConflictsKernel= 0;
	

	ctx = b3OpenCLUtils::createContextFromType(CL_DEVICE_TYPE_ALL, &ciErrNum,0,0,preferred_device,preferred_platform,&platformId);
	b3OpenCLUtils::printPlatformInfo(platformId);
	oclCHECKERROR(ciErrNum, CL_SUCCESS);
	device = b3OpenCLUtils::getDevice(ctx,0);
	b3OpenCLUtils::printDeviceInfo(device);
	queue = clCreateCommandQueue(ctx, device, 0, &ciErrNum);

	const char* cSourceFile = "opencl/lds_bank_conflict/lds_kernels.cl";
	
	size_t szKernelLength;

	const char* cSourceCL =0;
	char relativeFileName[1024];

	{
		const char* prefix[]={"./","../","../../","../../../","../../../../"};
		int numPrefixes = sizeof(prefix)/sizeof(char*);

		for (int i=0;!cSourceCL && i<numPrefixes;i++)
		{
			
			sprintf(relativeFileName,"%s%s",prefix[i],cSourceFile);
			cSourceCL = loadProgSource(relativeFileName, "", &szKernelLength);
			if (cSourceCL)
			{
				printf("Loaded program source: %s\n", relativeFileName); 
			}
		}
	}
	if (!cSourceCL)
	{
		printf("Couldn't find file %s, exiting\n",cSourceFile);
		exit(0);
	}

char flags[1024]={0};
#ifdef CL_PLATFORM_INTEL
///use this flag to allow for OpenCL kernel debugging on CPU using the Intel OpenCL run-time
	//sprintf(flags,"-g -s \"%s\"","C:/develop/opencl_course/opencl/lds_bank_conflict/lds_kernels.cl");
#endif//CL_PLATFORM_INTEL

	
	copyKernel  = b3OpenCLUtils::compileCLKernelFromString(ctx,device,cSourceCL,"copyKernel",&ciErrNum,0,flags);
	copySharedMemKernel  = b3OpenCLUtils::compileCLKernelFromString(ctx,device,cSourceCL,"copySharedMemKernel",&ciErrNum,0,flags);
	transposeNaiveKernel = b3OpenCLUtils::compileCLKernelFromString(ctx,device,cSourceCL,"transposeNaiveKernel",&ciErrNum,0,flags);
	transposeCoalescedKernel = b3OpenCLUtils::compileCLKernelFromString(ctx,device,cSourceCL,"transposeCoalescedKernel",&ciErrNum,0,flags);
	transposeNoBankConflictsKernel = b3OpenCLUtils::compileCLKernelFromString(ctx,device,cSourceCL,"transposeNoBankConflictsKernel",&ciErrNum,0,flags);
	
	b3FillCL clMemSet(ctx,device,queue);

	printf("\n============================================\n");

	printf("Matrix size: %d %d, Block size: %d %d, Tile size: %d %d\n", 
		 nx, ny, TILE_DIM, BLOCK_ROWS, TILE_DIM, TILE_DIM);

	float *h_idata = (float*)malloc(mem_size);
	float *h_cdata = (float*)malloc(mem_size);
	float *h_tdata = (float*)malloc(mem_size);
	float *gold    = (float*)malloc(mem_size);
  
	b3OpenCLArray<float> d_idataCL(ctx,queue);d_idataCL.resize(num_elements);
	b3OpenCLArray<float> d_cdataCL(ctx,queue);d_cdataCL.resize(num_elements);
	b3OpenCLArray<float> d_tdataCL(ctx,queue);d_tdataCL.resize(num_elements);
  

	// check parameters and calculate execution configuration
	if (nx % TILE_DIM || ny % TILE_DIM) 
	{
		printf("nx and ny must be a multiple of TILE_DIM\n");
		goto error_exit;
	}

	if (TILE_DIM % BLOCK_ROWS) 
	{
		printf("TILE_DIM must be a multiple of BLOCK_ROWS\n");
		goto error_exit;
	}
	
  // host
  for (int j = 0; j < ny; j++)
	for (int i = 0; i < nx; i++)
	  h_idata[j*nx + i] = j*nx + i;

  // correct result for error checking
  for (int j = 0; j < ny; j++)
	for (int i = 0; i < nx; i++)
	{
	  gold[j*nx + i] = h_idata[i*nx + j];
	}
  
  d_idataCL.copyFromHostPointer(h_idata,num_elements);

  // events for timing
  clock.reset();

  float ms;

  // ------------
  // time kernels
  // ------------
  printf("%25s%25s\n", "Routine", "Bandwidth (GB/s)");
  
  // ----
  // copy 
  // ----
  printf("%25s", "copy");

  clMemSet.execute(d_cdataCL,0.f,num_elements);
  
  {
	    // warm up
		b3LauncherCL launcher( queue, copyKernel);
		launcher.setBuffer( d_cdataCL.getBufferCL());
		launcher.setBuffer( d_idataCL.getBufferCL());
		launcher.launch2D(numThreadsX,numThreadsY,localSizeX,localSizeY );

		startEvent = clock.getTimeMicroseconds()/1e3;
		for (int i = 0; i < NUM_REPS; i++)
			launcher.launch2D(numThreadsX,numThreadsY,localSizeX,localSizeY );
		oclCHECKERROR(ciErrNum, CL_SUCCESS);
		clFinish(queue);
		stopEvent = clock.getTimeMicroseconds()/1e3;
	}

	ms = float(stopEvent-startEvent);

	d_cdataCL.copyToHostPointer(h_cdata,num_elements,0);
	postprocess(h_idata, h_cdata, nx*ny, ms);

  // -------------
  // copySharedMem 
  // -------------
	printf("%25s", "shared memory copy");
	clMemSet.execute(d_cdataCL,0.f,num_elements);

	{
		b3LauncherCL launcher( queue, copySharedMemKernel);
		launcher.setBuffer( d_cdataCL.getBufferCL());
		launcher.setBuffer( d_idataCL.getBufferCL());
		launcher.launch2D(numThreadsX,numThreadsY,localSizeX,localSizeY );

		startEvent = clock.getTimeMicroseconds()/1e3;
		for (int i = 0; i < NUM_REPS; i++)
			launcher.launch2D(numThreadsX,numThreadsY,localSizeX,localSizeY );
		oclCHECKERROR(ciErrNum, CL_SUCCESS);
		clFinish(queue);
		stopEvent = clock.getTimeMicroseconds()/1e3;
	}

	ms = float(stopEvent-startEvent);
	d_cdataCL.copyToHostPointer(h_cdata,num_elements,0);
	postprocess(h_idata, h_cdata, nx * ny, ms);

  // --------------
  // transposeNaive 
  // --------------
	printf("%25s", "naive transpose");
	clMemSet.execute(d_tdataCL,0.f,num_elements);
	{
		// warmup
		b3LauncherCL launcher( queue, transposeNaiveKernel);
		launcher.setBuffer( d_tdataCL.getBufferCL());
		launcher.setBuffer( d_idataCL.getBufferCL());
		launcher.launch2D(numThreadsX,numThreadsY,localSizeX,localSizeY );

		startEvent = clock.getTimeMicroseconds()/1e3;
		for (int i = 0; i < NUM_REPS; i++)
			launcher.launch2D(numThreadsX,numThreadsY,localSizeX,localSizeY );
		oclCHECKERROR(ciErrNum, CL_SUCCESS);
		clFinish(queue);
		stopEvent = clock.getTimeMicroseconds()/1e3;
	}
	ms = float(stopEvent-startEvent);
	d_tdataCL.copyToHostPointer(h_tdata,num_elements,0);
	postprocess(gold, h_tdata, nx * ny, ms);

  // ------------------
  // transposeCoalesced 
  // ------------------
	printf("%25s", "coalesced transpose");
    clMemSet.execute(d_tdataCL,0.f,num_elements);
	{
		b3LauncherCL launcher( queue, transposeCoalescedKernel);
		launcher.setBuffer( d_tdataCL.getBufferCL());
		launcher.setBuffer( d_idataCL.getBufferCL());
		launcher.launch2D(numThreadsX,numThreadsY,localSizeX,localSizeY );

		startEvent = clock.getTimeMicroseconds()/1e3;
		for (int i = 0; i < NUM_REPS; i++)
			launcher.launch2D(numThreadsX,numThreadsY,localSizeX,localSizeY );
		oclCHECKERROR(ciErrNum, CL_SUCCESS);
		clFinish(queue);
		stopEvent = clock.getTimeMicroseconds()/1e3;
	}

	ms = float(stopEvent-startEvent);
	d_tdataCL.copyToHostPointer(h_tdata,num_elements,0);
	postprocess(gold, h_tdata, nx * ny, ms);

  // ------------------------
  // transposeNoBankConflicts
  // ------------------------
	printf("%25s", "conflict-free transpose");
	clMemSet.execute(d_tdataCL,0.f,num_elements);
	{
		b3LauncherCL launcher( queue, transposeNoBankConflictsKernel);
		launcher.setBuffer( d_tdataCL.getBufferCL());
		launcher.setBuffer( d_idataCL.getBufferCL());
		launcher.launch2D(numThreadsX,numThreadsY,localSizeX,localSizeY );

		startEvent = clock.getTimeMicroseconds()/1e3;
		for (int i = 0; i < NUM_REPS; i++)
			launcher.launch2D(numThreadsX,numThreadsY,localSizeX,localSizeY );
		oclCHECKERROR(ciErrNum, CL_SUCCESS);
		clFinish(queue);
		stopEvent = clock.getTimeMicroseconds()/1e3;
	}

	ms = float(stopEvent-startEvent);
	d_tdataCL.copyToHostPointer(h_tdata,num_elements,0);
	postprocess(gold, h_tdata, nx * ny, ms);

error_exit:
  // cleanup
	clReleaseKernel(copyKernel);
	clReleaseCommandQueue(queue);
	clReleaseContext(ctx);

	free(h_idata);
	free(h_tdata);
	free(h_cdata);
	free(gold);
	printf("Press <enter>\n");
	getchar();
}
