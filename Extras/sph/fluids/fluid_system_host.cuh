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


#include <vector_types.h>	
#include <driver_types.h>			// for cudaStream_t

typedef unsigned int		uint;		// should be 4-bytes on CUDA
typedef unsigned char		uchar;		// should be 1-bytes on CUDA

struct FluidParams {
	int				numThreads, numBlocks;
	int				gridThreads, gridBlocks;	
	int				szPnts, szHash, szGrid;
	int				stride, pnts, cells;
	int				chk;
	float			smooth_rad, r2, sim_scale, visc;
	float3			min, max, res, size, delta;
	
	float			pdist, pmass, rest_dens, stiffness;
	float			poly6kern, spikykern, lapkern;

};

extern "C"
{

void cudaInit(int argc, char **argv);

void FluidClearCUDA ();
void FluidSetupCUDA ( int num, int stride, float3 min, float3 max, float3 res, float3 size, int chk );
void FluidParamCUDA ( float sim_scale, float smooth_rad, float mass, float rest, float stiff, float visc );

void TransferToCUDA ( char* data, int* grid, int numPoints );
void TransferFromCUDA ( char* data, int* grid, int numPoints );

void Grid_InsertParticlesCUDA ();
void SPH_ComputePressureCUDA ();
void SPH_ComputeForceCUDA ();
void SPH_AdvanceCUDA ( float dt, float ss );

}


