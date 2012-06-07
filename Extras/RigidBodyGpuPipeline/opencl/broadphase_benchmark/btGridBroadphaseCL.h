
/*
Copyright (c) 2012 Advanced Micro Devices, Inc.  

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
//Originally written by Roman Ponomarev, Erwin Coumans

#ifndef GRID_BROADPHASE_CL_H
#define GRID_BROADPHASE_CL_H

#include "../3dGridBroadphase/Shared/bt3dGridBroadphaseOCL.h"

#include "Adl/Adl.h"
#include "Adl/AdlKernel.h"


struct MyAabbConstData 
{
	int bla;
	int numElem;
};



class btGridBroadphaseCl : public bt3dGridBroadphaseOCL
{
protected:

	adl::Kernel*			m_countOverlappingPairs;
	adl::Kernel*			m_squeezePairCaches;


	adl::Buffer<MyAabbConstData>*	m_aabbConstBuffer;


	public:

		cl_mem					m_dAllOverlappingPairs;

		
		btGridBroadphaseCl(	btOverlappingPairCache* overlappingPairCache,
							const btVector3& cellSize, 
							int gridSizeX, int gridSizeY, int gridSizeZ, 
							int maxSmallProxies, int maxLargeProxies, int maxPairsPerSmallProxy,
							btScalar maxSmallProxySize,
							int maxSmallProxiesPerCell = 4,
							cl_context context = NULL,
							cl_device_id device = NULL,
							cl_command_queue queue = NULL,
							adl::DeviceCL* deviceCL=0
							);
		
		virtual void prepareAABB(float* positions, int numObjects);
		virtual void calcHashAABB();

		void calculateOverlappingPairs(float* positions, int numObjects);
		
		virtual ~btGridBroadphaseCl();							
	
};

#endif //GRID_BROADPHASE_CL_H

