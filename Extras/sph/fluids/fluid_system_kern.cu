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



#ifndef _PARTICLES_KERNEL_H_
	#define _PARTICLES_KERNEL_H_

	#include <stdio.h>
	#include <math.h>

	#include "fluid_system_host.cuh"

	#define TOTAL_THREADS		65536
	#define BLOCK_THREADS		256
	#define MAX_NBR				80
	
	__constant__	FluidParams		simData;		// simulation data (on device)
	
	__device__ int				bufNeighbor[ TOTAL_THREADS*MAX_NBR ];
	__device__ float			bufNdist[ TOTAL_THREADS*MAX_NBR ];	

	#define COLOR(r,g,b)	( (uint((r)*255.0f)<<24) | (uint((g)*255.0f)<<16) | (uint((b)*255.0f)<<8) )
	#define COLORA(r,g,b,a)	( (uint((r)*255.0f)<<24) | (uint((g)*255.0f)<<16) | (uint((b)*255.0f)<<8) | uint((a)*255.0f) )
	
	#define NULL_HASH		333333
	
	#define OFFSET_CLR		12
	#define OFFSET_NEXT		16
	#define OFFSET_VEL		20
	#define OFFSET_VEVAL	32
	#define OFFSET_PRESS	48
	#define OFFSET_DENS		52
	#define OFFSET_FORCE	56
	

	__global__ void hashParticles ( char* bufPnts, uint2* bufHash, int numPnt )
	{			
		uint ndx = __mul24(blockIdx.x, blockDim.x) + threadIdx.x;	// particle index
		float3* pos = (float3*) (bufPnts + __mul24(ndx, simData.stride) );
		int gz = (pos->z - simData.min.z) * simData.delta.z ;
		int gy = (pos->y - simData.min.y) * simData.delta.y ;
		int gx = (pos->x - simData.min.x) * simData.delta.x ;
		if ( ndx >= numPnt || gx < 0 || gz > simData.res.x-1 || gy < 0 || gy > simData.res.y-1 || gz < 0 || gz > simData.res.z-1 ) 
			bufHash[ndx] = make_uint2( NULL_HASH, ndx );
		else
			bufHash[ndx] = make_uint2( __mul24(__mul24(gz, (int) simData.res.y)+gy, (int) simData.res.x) + gx, ndx );		
		
		__syncthreads ();
	}
	
	__global__ void insertParticles ( char* bufPnts, uint2* bufHash, int* bufGrid, int numPnt, int numGrid )
	{
		uint grid_ndx = __mul24(blockIdx.x, blockDim.x) + threadIdx.x;	// grid cell index		
		
		bufPnts += OFFSET_NEXT;
		bufGrid[grid_ndx] = -1;
		for (int n=0; n < numPnt; n++) {
			if ( bufHash[n].x == grid_ndx ) {
				*(int*) (bufPnts + __mul24(bufHash[n].y, simData.stride)) = bufGrid[grid_ndx];
				bufGrid[grid_ndx] = bufHash[n].y;
			}
		}		
		__syncthreads ();
	}
	
	__global__ void insertParticlesRadix ( char* bufPnts, uint2* bufHash, int* bufGrid, char* bufPntSort, int numPnt, int numGrid )
	{
		uint ndx = __mul24(blockIdx.x, blockDim.x) + threadIdx.x;		// particle index		
		
		uint2 bufHashSort = bufHash[ndx];
		
		__shared__ uint sharedHash[257];	
		sharedHash[threadIdx.x+1] = bufHashSort.x;
		if ( ndx > 0 && threadIdx.x == 0 ) {
			volatile uint2 prevData = bufHash[ndx-1];
			sharedHash[0]  = prevData.x;
		}
		__syncthreads ();
		
		if ( (ndx == 0 || bufHashSort.x != sharedHash[threadIdx.x]) && bufHashSort.x != NULL_HASH ) {
			bufGrid [ bufHashSort.x ] = ndx;			
		}
		if ( ndx < numPnt ) {
			char* src = bufPnts + __mul24( bufHashSort.y, simData.stride );
			char* dest = bufPntSort + __mul24( ndx, simData.stride );
			
			*(float3*)(dest)				= *(float3*)(src);
			*(uint*)  (dest + OFFSET_CLR)	= *(uint*)  (src + OFFSET_CLR);
			*(float3*)(dest + OFFSET_VEL)	= *(float3*)(src + OFFSET_VEL);
			*(float3*)(dest + OFFSET_VEVAL)	= *(float3*)(src + OFFSET_VEVAL);				
			
			*(float*) (dest + OFFSET_DENS)	= 0.0;
			*(float*) (dest + OFFSET_PRESS)	= 0.0;				
			*(float3*) (dest + OFFSET_FORCE)= make_float3(0,0,0);		
			*(int*)   (dest + OFFSET_NEXT)	= bufHashSort.x;			
		} 
		
		__syncthreads ();
		
	}
	
	//__shared__ int ncount [ BLOCK_THREADS ];
	
	__device__ float contributePressure ( int pndx, float3* p, int qndx, int grid_ndx, char* bufPnts, uint2* bufHash )
	{			
		float3* qpos;		
		float3 dist;
		float dsq, c, sum;
		float d = simData.sim_scale;				
		int nbr = __mul24(pndx, MAX_NBR);
						
		sum = 0.0;		
		for ( ; qndx < simData.pnts; qndx++ ) {
			
			if ( bufHash[qndx].x != grid_ndx || qndx == NULL_HASH) break;
			
			if ( qndx != pndx ) {
				qpos = (float3*) ( bufPnts + __mul24(qndx, simData.stride ));	
					
				dist.x = ( p->x - qpos->x )*d;		// dist in cm
				dist.y = ( p->y - qpos->y )*d;
				dist.z = ( p->z - qpos->z )*d;			
				dsq = (dist.x*dist.x + dist.y*dist.y + dist.z*dist.z);			
				if ( dsq < simData.r2 ) {
					c = simData.r2 - dsq;
					sum += c * c * c;				
					if  ( bufNeighbor[nbr] < MAX_NBR ) {
						bufNeighbor[ nbr+bufNeighbor[nbr] ] = qndx;
						bufNdist[ nbr+bufNeighbor[nbr] ] = sqrt(dsq);
						bufNeighbor[nbr]++;
					}
				}				
			}
			//curr = *(int*) (bufPnts + __mul24(curr, simData.stride) + OFFSET_NEXT);
		}		
		return sum;
	}
	
		/*if  ( ncount[threadIdx.x]  < MAX_NBR ) {
				bufNeighbor [ nbr + ncount[threadIdx.x]  ] = curr;
				bufNdist [ nbr + ncount[threadIdx.x]  ] = sqrt(dsq);
				ncount[threadIdx.x]++;
		}*/	
		
	__global__ void computePressure ( char* bufPntSort, int* bufGrid, uint2* bufHash, int numPnt )
	{
		uint ndx = __mul24(blockIdx.x, blockDim.x) + threadIdx.x;	// particle index		

		//if ( ndx < 1024 ) {
		
		float3* pos = (float3*) (bufPntSort + __mul24(ndx, simData.stride));

		// Find 2x2x2 grid cells
		// - Use registers only, no arrays (local-memory too slow)
		int3 cell;
		int gc0, gc1, gc2, gc3, gc4, gc5, gc6, gc7;					
		float gs = simData.smooth_rad / simData.sim_scale;		

		cell.x = max(0, (int)((-gs + pos->x - simData.min.x) * simData.delta.x));
		cell.y = max(0, (int)((-gs + pos->y - simData.min.y) * simData.delta.y));
		cell.z = max(0, (int)((-gs + pos->z - simData.min.z) * simData.delta.z));		
		gc0 = __mul24(__mul24(cell.z, simData.res.y) + cell.y, simData.res.x) + cell.x;
		gc1 = gc0 + 1;
		gc2 = gc0 + simData.res.x;
		gc3 = gc2 + 1;
		if ( cell.z+1 < simData.res.z ) {
			gc4 = gc0 + __mul24(simData.res.x, simData.res.y);
			gc5 = gc4 + 1;
			gc6 = gc4 + simData.res.x;
			gc7 = gc6 + 1;
		}
		if ( cell.x+1 >= simData.res.x ) {
			gc1 = -1; gc3 = -1;
			gc5 = -1; gc7 = -1;
		}
		if ( cell.y+1 >= simData.res.y ) {
			gc2 = -1; gc3 = -1;
			gc6 = -1; gc7 = -1;
		}
		// Sum Pressure
		float sum = 0.0;		
		bufNeighbor[ __mul24(ndx, MAX_NBR) ] = 1;
		if (gc0 != -1 ) sum += contributePressure ( ndx, pos, bufGrid[gc0], gc0, bufPntSort, bufHash );
		if (gc1 != -1 ) sum += contributePressure ( ndx, pos, bufGrid[gc1], gc1, bufPntSort, bufHash );		
		if (gc2 != -1 ) sum += contributePressure ( ndx, pos, bufGrid[gc2], gc2, bufPntSort, bufHash );		
		if (gc3 != -1 ) sum += contributePressure ( ndx, pos, bufGrid[gc3], gc3, bufPntSort, bufHash );	
		if (gc4 != -1 ) sum += contributePressure ( ndx, pos, bufGrid[gc4], gc4, bufPntSort, bufHash );
		if (gc5 != -1 ) sum += contributePressure ( ndx, pos, bufGrid[gc5], gc5, bufPntSort, bufHash );		
		if (gc6 != -1 ) sum += contributePressure ( ndx, pos, bufGrid[gc6], gc6, bufPntSort, bufHash );
		if (gc7 != -1 ) sum += contributePressure ( ndx, pos, bufGrid[gc7], gc7, bufPntSort, bufHash );
		
		// Compute Density & Pressure
		sum = sum * simData.pmass * simData.poly6kern;
		if ( sum == 0.0 ) sum = 1.0;
		*(float*) ((char*)pos + OFFSET_PRESS) = ( sum - simData.rest_dens ) * simData.stiffness;
		*(float*) ((char*)pos + OFFSET_DENS) = 1.0f / sum;			
		
		//}		
		//__syncthreads ();
	}

	__device__ void contributeForce ( float3& force, int pndx, float3* p, int qndx, int grid_ndx, char* bufPnts, uint2* bufHash )
	{
		float press = *(float*) ((char*)p + OFFSET_PRESS);
		float dens = *(float*) ((char*)p + OFFSET_DENS);
		float3 veval = *(float3*) ((char*)p + OFFSET_VEVAL );
		float3 qeval, dist;				
		float c, ndistj, dsq;
		float pterm, dterm, vterm;		
		float3* qpos;				
		float d = simData.sim_scale;				
		
		vterm = simData.lapkern * simData.visc;		
						
		for ( ; qndx < simData.pnts; qndx++ ) {
			
			if ( bufHash[qndx].x != grid_ndx || qndx == NULL_HASH) break;
			
			if ( qndx != pndx ) {
				qpos = (float3*) ( bufPnts + __mul24(qndx, simData.stride ));	
					
				dist.x = ( p->x - qpos->x )*d;		// dist in cm
				dist.y = ( p->y - qpos->y )*d;
				dist.z = ( p->z - qpos->z )*d;			
				dsq = (dist.x*dist.x + dist.y*dist.y + dist.z*dist.z);			
				if ( dsq < simData.r2 ) {				
					ndistj = sqrt(dsq);
					c = ( simData.smooth_rad - ndistj ); 
					dist.x = ( p->x - qpos->x )*d;		// dist in cm
					dist.y = ( p->y - qpos->y )*d;
					dist.z = ( p->z - qpos->z )*d;			
					pterm = -0.5f * c * simData.spikykern * ( press + *(float*)((char*)qpos+OFFSET_PRESS) ) / ndistj;
					dterm = c * dens * *(float*)((char*)qpos+OFFSET_DENS);	
					qeval = *(float3*)((char*)qpos+OFFSET_VEVAL);
					force.x += ( pterm * dist.x + vterm * ( qeval.x - veval.x )) * dterm;
					force.y += ( pterm * dist.y + vterm * ( qeval.y - veval.y )) * dterm;
					force.z += ( pterm * dist.z + vterm * ( qeval.z - veval.z )) * dterm;							
				}
			}
		}				
	}
	
	
	
	__global__ void computeForce ( char* bufPntSort, int* bufGrid, uint2* bufHash, int numPnt )
	{
		uint ndx = __mul24(blockIdx.x, blockDim.x) + threadIdx.x;	// particle index		
		
		//if ( ndx < numPnt ) {
		
		float3* pos = (float3*) (bufPntSort + __mul24(ndx, simData.stride));				
		
		// Find 2x2x2 grid cells
		// - Use registers only, no arrays (local-memory too slow)
		int3 cell;
		int gc0, gc1, gc2, gc3, gc4, gc5, gc6, gc7;					
		float gs = simData.smooth_rad / simData.sim_scale;		

		cell.x = max(0, (int)((-gs + pos->x - simData.min.x) * simData.delta.x));
		cell.y = max(0, (int)((-gs + pos->y - simData.min.y) * simData.delta.y));
		cell.z = max(0, (int)((-gs + pos->z - simData.min.z) * simData.delta.z));		
		gc0 = __mul24(__mul24(cell.z, simData.res.y) + cell.y, simData.res.x) + cell.x;
		gc1 = gc0 + 1;
		gc2 = gc0 + simData.res.x;
		gc3 = gc2 + 1;
		if ( cell.z+1 < simData.res.z ) {
			gc4 = gc0 + __mul24(simData.res.x, simData.res.y);
			gc5 = gc4 + 1;
			gc6 = gc4 + simData.res.x;
			gc7 = gc6 + 1;
		}
		if ( cell.x+1 >= simData.res.x ) {
			gc1 = -1; gc3 = -1;
			gc5 = -1; gc7 = -1;
		}
		if ( cell.y+1 >= simData.res.y ) {
			gc2 = -1; gc3 = -1;
			gc6 = -1; gc7 = -1;
		}
		// Sum Pressure
		float3 force = make_float3(0,0,0);
		if (gc0 != -1 ) contributeForce ( force, ndx, pos, bufGrid[gc0], gc0, bufPntSort, bufHash );
		if (gc1 != -1 ) contributeForce ( force, ndx, pos, bufGrid[gc1], gc1, bufPntSort, bufHash );		
		if (gc2 != -1 ) contributeForce ( force, ndx, pos, bufGrid[gc2], gc2, bufPntSort, bufHash );		
		if (gc3 != -1 ) contributeForce ( force, ndx, pos, bufGrid[gc3], gc3, bufPntSort, bufHash );	
		if (gc4 != -1 ) contributeForce ( force, ndx, pos, bufGrid[gc4], gc4, bufPntSort, bufHash );
		if (gc5 != -1 ) contributeForce ( force, ndx, pos, bufGrid[gc5], gc5, bufPntSort, bufHash );		
		if (gc6 != -1 ) contributeForce ( force, ndx, pos, bufGrid[gc6], gc6, bufPntSort, bufHash );
		if (gc7 != -1 ) contributeForce ( force, ndx, pos, bufGrid[gc7], gc7, bufPntSort, bufHash );
		
		// Update Force
		*(float3*) ((char*)pos + OFFSET_FORCE ) = force;	
		
		//}
		//__syncthreads ();
	}

	
	__global__ void computeForceNbr ( char* bufPntSort, int numPnt )
	{		
		uint ndx = __mul24(blockIdx.x, blockDim.x) + threadIdx.x;	// particle index		
		
		if ( ndx < numPnt ) {
				
		float3* pos = (float3*) (bufPntSort + __mul24(ndx, simData.stride));			
		
		float3* qpos;
		float press = *(float*) ((char*)pos + OFFSET_PRESS);
		float dens = *(float*) ((char*)pos + OFFSET_DENS);
		float3 veval = *(float3*) ((char*)pos + OFFSET_VEVAL );
		float3 qeval, dist, force;		
		float d = simData.sim_scale;
		float c, ndistj;
		float pterm, dterm, vterm;
		vterm = simData.lapkern * simData.visc;
		int nbr = __mul24(ndx, MAX_NBR);
		
		int ncnt = bufNeighbor[ nbr ];		
		
		force = make_float3(0,0,0);
		for (int j=1; j < ncnt; j++) {		// base 1, n[0] = count
			ndistj = bufNdist[ nbr+j ];
			qpos = (float3*) (bufPntSort + __mul24( bufNeighbor[ nbr+j ], simData.stride) );
			c = ( simData.smooth_rad - ndistj ); 
			dist.x = ( pos->x - qpos->x )*d;		// dist in cm
			dist.y = ( pos->y - qpos->y )*d;
			dist.z = ( pos->z - qpos->z )*d;			
			pterm = -0.5f * c * simData.spikykern * ( press + *(float*)((char*)qpos+OFFSET_PRESS) ) / ndistj;
			dterm = c * dens * *(float*)((char*)qpos+OFFSET_DENS);	
			qeval = *(float3*)((char*)qpos+OFFSET_VEVAL);
			force.x += ( pterm * dist.x + vterm * ( qeval.x - veval.x )) * dterm;
			force.y += ( pterm * dist.y + vterm * ( qeval.y - veval.y )) * dterm;
			force.z += ( pterm * dist.z + vterm * ( qeval.z - veval.z )) * dterm;			
		}
		*(float3*) ((char*)pos + OFFSET_FORCE ) = force;
		
		}	
	
	}
		
	__global__ void advanceParticles ( char* bufPntSort, int numPnt, float dt, float ss )
	{		
		uint ndx = __mul24(blockIdx.x, blockDim.x) + threadIdx.x;	// particle index		
		
		if ( ndx < numPnt ) {
				
			// Get particle vars
			float3* pos = (float3*) (bufPntSort + __mul24(ndx, simData.stride));			
			float3* vel = (float3*) ((char*)pos + OFFSET_VEL );
			float3* vel_eval = (float3*) ((char*)pos + OFFSET_VEVAL );
			float3 accel = *(float3*) ((char*)pos + OFFSET_FORCE );
			float3 vcurr, vnext;			

			// Leapfrog integration						
			accel.x *= 0.00020543;			// NOTE - To do: SPH_PMASS should be passed in			
			accel.y *= 0.00020543;
			accel.z *= 0.00020543;			
			accel.z -= 9.8;	
			
			vcurr = *vel;
			vnext.x = accel.x*dt + vcurr.x;	
			vnext.y = accel.y*dt + vcurr.y;	
			vnext.z = accel.z*dt + vcurr.z;			// v(t+1/2) = v(t-1/2) + a(t) dt			
			
			accel.x = (vcurr.x + vnext.x) * 0.5;		// v(t+1) = [v(t-1/2) + v(t+1/2)] * 0.5		used to compute forces later
			accel.y = (vcurr.y + vnext.y) * 0.5;		// v(t+1) = [v(t-1/2) + v(t+1/2)] * 0.5		used to compute forces later
			accel.z = (vcurr.z + vnext.z) * 0.5;		// v(t+1) = [v(t-1/2) + v(t+1/2)] * 0.5		used to compute forces later
			
			*vel_eval = accel;			
			*vel = vnext;
			
			dt /= simData.sim_scale;
			vnext.x = pos->x + vnext.x*dt;
			vnext.y = pos->y + vnext.y*dt;
			vnext.z = pos->z + vnext.z*dt;
			*pos = vnext;						// p(t+1) = p(t) + v(t+1/2) dt			
		}	
		
		__syncthreads ();	
	}

#endif
