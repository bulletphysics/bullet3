/*
  FLUIDS v.1 - SPH Fluid Simulator for CPU and GPU
  Copyright (C) 2008. Rama Hoetzlein, http://www.rchoetzlein.com

  ZLib license
  This software is provided 'as-is', without any express or implied
  warranty.  In no event will the authors be held liable for any damages
  arising from the use of this software.

  Permission is granted to anyone to use this software for any purpose,
  including commercial applications, and to alter it and redistribute it
  freely, subject to the following restrictions:

  1. The origin of this software must not be misrepresented; you must not
     claim that you wrote the original software. If you use this software
     in a product, an acknowledgment in the product documentation would be
     appreciated but is not required.
  2. Altered source versions must be plainly marked as such, and must not be
     misrepresented as being the original software.
  3. This notice may not be removed or altered from any source distribution.
*/


//#include "C:\CUDA\common\inc\cutil.h"				// cutil32.lib
#include <string.h>
#include "../CUDA/btCudaDefines.h"



#if defined(__APPLE__) || defined(MACOSX)
	#include <GLUT/glut.h>
#else
	#include <GL/glut.h>
#endif
#include <cuda_gl_interop.h>

#include "radixsort.cu"
#include "fluid_system_kern.cu"			// build kernel

FluidParams					fcuda;

__device__ char*			bufPnts;		// point data (array of Fluid structs)
__device__ char*			bufPntSort;		// point data (array of Fluid structs)
__device__ uint*			bufHash[2];		// point grid hash
__device__ int*				bufGrid;	

	

extern "C"
{
// Initialize CUDA
void cudaInit(int argc, char **argv)
{   
    //CUT_DEVICE_INIT(argc, argv);
 
	cudaDeviceProp p;
	cudaGetDeviceProperties ( &p, 0);
	
	printf ( "-- CUDA --\n" );
	printf ( "Name:       %s\n", p.name );
	printf ( "Revision:   %d.%d\n", p.major, p.minor );
	printf ( "Global Mem: %d\n", p.totalGlobalMem );
	printf ( "Shared/Blk: %d\n", p.sharedMemPerBlock );
	printf ( "Regs/Blk:   %d\n", p.regsPerBlock );
	printf ( "Warp Size:  %d\n", p.warpSize );
	printf ( "Mem Pitch:  %d\n", p.memPitch );
	printf ( "Thrds/Blk:  %d\n", p.maxThreadsPerBlock );
	printf ( "Const Mem:  %d\n", p.totalConstMem );
	printf ( "Clock Rate: %d\n", p.clockRate );	
	
	BT_GPU_SAFE_CALL ( cudaMalloc ( (void**) &bufPnts, 10 ) );	
	BT_GPU_SAFE_CALL ( cudaMalloc ( (void**) &bufPntSort, 10 ) );
	BT_GPU_SAFE_CALL ( cudaMalloc ( (void**) &bufHash, 10 ) );	
	BT_GPU_SAFE_CALL ( cudaMalloc ( (void**) &bufGrid, 10 ) );	
};
	
// Compute number of blocks to create
int iDivUp (int a, int b) {
    return (a % b != 0) ? (a / b + 1) : (a / b);
}
void computeNumBlocks (int numPnts, int maxThreads, int &numBlocks, int &numThreads)
{
    numThreads = min( maxThreads, numPnts );
    numBlocks = iDivUp ( numPnts, numThreads );
}

void FluidClearCUDA ()
{
	BT_GPU_SAFE_CALL ( cudaFree ( bufPnts ) );	
	BT_GPU_SAFE_CALL ( cudaFree ( bufPntSort ) );
	BT_GPU_SAFE_CALL ( cudaFree ( bufHash[0] ) );	
	BT_GPU_SAFE_CALL ( cudaFree ( bufHash[1] ) );	
	BT_GPU_SAFE_CALL ( cudaFree ( bufGrid ) );
}


void FluidSetupCUDA ( int num, int stride, float3 min, float3 max, float3 res, float3 size, int chk )
{	
	fcuda.min = make_float3(min.x, min.y, min.z);
	fcuda.max = make_float3(max.x, max.y, max.z);
	fcuda.res = make_float3(res.x, res.y, res.z);
	fcuda.size = make_float3(size.x, size.y, size.z);	
	fcuda.pnts = num;
	fcuda.delta.x = res.x / size.x;
	fcuda.delta.y = res.y / size.y;
	fcuda.delta.z = res.z / size.z;
	fcuda.cells = res.x*res.y*res.z;
	fcuda.chk = chk;
		
    computeNumBlocks ( fcuda.pnts, 256, fcuda.numBlocks, fcuda.numThreads);			// particles
    computeNumBlocks ( fcuda.cells, 256, fcuda.gridBlocks, fcuda.gridThreads);		// grid cell
    
    fcuda.szPnts = (fcuda.numBlocks * fcuda.numThreads) * stride;        
    fcuda.szHash = (fcuda.numBlocks * fcuda.numThreads) * sizeof(uint2);		// <cell, particle> pairs
    fcuda.szGrid = (fcuda.gridBlocks * fcuda.gridThreads) * sizeof(uint);    
    fcuda.stride = stride;
    printf ( "pnts: %d, t:%dx%d=%d, bufPnts:%d, bufHash:%d\n", fcuda.pnts, fcuda.numBlocks, fcuda.numThreads, fcuda.numBlocks*fcuda.numThreads, fcuda.szPnts, fcuda.szHash );
    printf ( "grds: %d, t:%dx%d=%d, bufGrid:%d, Res: %dx%dx%d\n", fcuda.cells, fcuda.gridBlocks, fcuda.gridThreads, fcuda.gridBlocks*fcuda.gridThreads, fcuda.szGrid, (int) fcuda.res.x, (int) fcuda.res.y, (int) fcuda.res.z );	

	BT_GPU_SAFE_CALL ( cudaMalloc ( (void**) &bufPnts, fcuda.szPnts ) );	
	BT_GPU_SAFE_CALL ( cudaMalloc ( (void**) &bufPntSort, fcuda.szPnts ) );	
	BT_GPU_SAFE_CALL ( cudaMalloc ( (void**) &bufHash[0], fcuda.szHash ) );	
	BT_GPU_SAFE_CALL ( cudaMalloc ( (void**) &bufHash[1], fcuda.szHash ) );	
	BT_GPU_SAFE_CALL ( cudaMalloc ( (void**) &bufGrid, fcuda.szGrid ) );
	
	printf ( "POINTERS\n");
	printf ( "bufPnts:    %p\n", bufPnts );
	printf ( "bufPntSort: %p\n", bufPntSort );
	printf ( "bufHash0:   %p\n", bufHash[0] );
	printf ( "bufHash1:   %p\n", bufHash[1] );
	printf ( "bufGrid:    %p\n", bufGrid );
	
	BT_GPU_SAFE_CALL ( cudaMemcpyToSymbol ( simData, &fcuda, sizeof(FluidParams) ) );
	cudaThreadSynchronize ();
}

void FluidParamCUDA ( float sim_scale, float smooth_rad, float mass, float rest, float stiff, float visc )
{
	fcuda.sim_scale = sim_scale;
	fcuda.smooth_rad = smooth_rad;
	fcuda.r2 = smooth_rad * smooth_rad;
	fcuda.pmass = mass;
	fcuda.rest_dens = rest;	
	fcuda.stiffness = stiff;
	fcuda.visc = visc;
	
	fcuda.pdist = pow ( fcuda.pmass / fcuda.rest_dens, 1/3.0f );
	fcuda.poly6kern = 315.0f / (64.0f * 3.141592 * pow( smooth_rad, 9.0f) );
	fcuda.spikykern = -45.0f / (3.141592 * pow( smooth_rad, 6.0f) );
	fcuda.lapkern = 45.0f / (3.141592 * pow( smooth_rad, 6.0f) );	

	BT_GPU_SAFE_CALL( cudaMemcpyToSymbol ( simData, &fcuda, sizeof(FluidParams) ) );
	cudaThreadSynchronize ();
}

void TransferToCUDA ( char* data, int* grid, int numPoints )
{
	BT_GPU_SAFE_CALL( cudaMemcpy ( bufPnts, data, numPoints * fcuda.stride, cudaMemcpyHostToDevice ) );
	cudaThreadSynchronize ();
}

void TransferFromCUDA ( char* data, int* grid, int numPoints )
{
	BT_GPU_SAFE_CALL( cudaMemcpy ( data, bufPntSort, numPoints * fcuda.stride, cudaMemcpyDeviceToHost ) );	
	cudaThreadSynchronize ();	
	
	BT_GPU_SAFE_CALL( cudaMemcpy ( grid, bufGrid, fcuda.cells * sizeof(uint), cudaMemcpyDeviceToHost ) );			
}

void Grid_InsertParticlesCUDA ()
{
	BT_GPU_SAFE_CALL( cudaMemset ( bufHash[0], 0, fcuda.szHash ) );
	
	hashParticles<<< fcuda.numBlocks, fcuda.numThreads>>> ( bufPnts, (uint2*) bufHash[0], fcuda.pnts );	
	BT_GPU_CHECK_ERROR( "Kernel execution failed");
	cudaThreadSynchronize ();
	
	//int buf[20000];		
	/*printf ( "HASH: %d (%d)\n", fcuda.pnts, fcuda.numBlocks*fcuda.numThreads );
	BT_GPU_SAFE_CALL( cudaMemcpy ( buf, bufHash[0], fcuda.pnts * 2*sizeof(uint), cudaMemcpyDeviceToHost ) );		
	//for (int n=0; n < fcuda.numBlocks*fcuda.numThreads; n++) {		
	for (int n=0; n < 100; n++) {
		printf ( "%d: <%d,%d>\n", n, buf[n*2], buf[n*2+1] );
	}*/
	 
	RadixSort( (KeyValuePair *) bufHash[0], (KeyValuePair *) bufHash[1], fcuda.pnts, 32);
	BT_GPU_CHECK_ERROR( "Kernel execution failed");
	cudaThreadSynchronize ();
	
	/*printf ( "HASH: %d (%d)\n", fcuda.pnts, fcuda.numBlocks*fcuda.numThreads );
	BT_GPU_SAFE_CALL( cudaMemcpy ( buf, bufHash[0], fcuda.pnts * 2*sizeof(uint), cudaMemcpyDeviceToHost ) );		
	//for (int n=0; n < fcuda.numBlocks*fcuda.numThreads; n++) {		
	for (int n=0; n < 100; n++) {
		printf ( "%d: <%d,%d>\n", n, buf[n*2], buf[n*2+1] );
	}*/
	
	// insertParticles<<< fcuda.gridBlocks, fcuda.gridThreads>>> ( bufPnts, (uint2*) bufHash[0], bufGrid, fcuda.pnts, fcuda.cells );			
	
	BT_GPU_SAFE_CALL( cudaMemset ( bufGrid, NULL_HASH, fcuda.cells * sizeof(uint) ) );
	
	insertParticlesRadix<<< fcuda.numBlocks, fcuda.numThreads>>> ( bufPnts, (uint2*) bufHash[0], bufGrid, bufPntSort, fcuda.pnts, fcuda.cells );
	BT_GPU_CHECK_ERROR( "Kernel execution failed");
	cudaThreadSynchronize ();	
    
    /*printf ( "GRID: %d\n", fcuda.cells );
	BT_GPU_SAFE_CALL( cudaMemcpy ( buf, bufGrid, fcuda.cells * sizeof(uint), cudaMemcpyDeviceToHost ) );		
	*for (int n=0; n < 100; n++) {		
		printf ( "%d: %d\n", n, buf[n]);
	}*/
}

void SPH_ComputePressureCUDA ()
{
	computePressure<<< fcuda.numBlocks, fcuda.numThreads>>> ( bufPntSort, bufGrid, (uint2*) bufHash[0], fcuda.pnts );	
    BT_GPU_CHECK_ERROR( "Kernel execution failed");
    cudaThreadSynchronize ();	
}

void SPH_ComputeForceCUDA ()
{
	//-- standard force
	//computeForce<<< fcuda.numBlocks, fcuda.numThreads>>> ( bufPntSort, bufGrid, (uint2*) bufHash[0], fcuda.pnts );	
	
	// Force using neighbor table
	computeForceNbr<<< fcuda.numBlocks, fcuda.numThreads>>> ( bufPntSort, fcuda.pnts );	
    BT_GPU_CHECK_ERROR( "Kernel execution failed");
    cudaThreadSynchronize ();	
}

void SPH_AdvanceCUDA ( float dt, float ss )
{
	advanceParticles<<< fcuda.numBlocks, fcuda.numThreads>>> ( bufPntSort, fcuda.pnts, dt, ss );
    BT_GPU_CHECK_ERROR( "Kernel execution failed");
    cudaThreadSynchronize ();
}

}	// extern C




   	//----------- Per frame: Malloc/Free, Host<->Device
	// transfer point data to device    
    /*char* pntData;
	int size = (fcuda.numBlocks*fcuda.numThreads) * stride;
	cudaMalloc( (void**) &pntData, size);
	cudaMemcpy( pntData, data, numPoints*stride, cudaMemcpyHostToDevice);  	
    insertParticles<<< fcuda.numBlocks, fcuda.numThreads >>> ( pntData, stride, numPoints );
    cudaMemcpy( data, pntData, numPoints*stride, cudaMemcpyDeviceToHost);    
    cudaFree( pntData );*/
