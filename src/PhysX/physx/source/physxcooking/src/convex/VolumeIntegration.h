//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2008-2019 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  


#ifndef PX_FOUNDATION_NXVOLUMEINTEGRATION
#define PX_FOUNDATION_NXVOLUMEINTEGRATION
/** \addtogroup foundation
  @{
*/


#include "foundation/Px.h"
#include "foundation/PxVec3.h"
#include "foundation/PxMat33.h"
#include "CmPhysXCommon.h"

namespace physx
{

class PxSimpleTriangleMesh;
class PxConvexMeshDesc;

/**
\brief Data structure used to store mass properties.
*/
struct PxIntegrals
	{
	PxVec3 COM;					//!< Center of mass
	PxF64 mass;						//!< Total mass
	PxF64 inertiaTensor[3][3];		//!< Inertia tensor (mass matrix) relative to the origin
	PxF64 COMInertiaTensor[3][3];	//!< Inertia tensor (mass matrix) relative to the COM

	/**
	\brief Retrieve the inertia tensor relative to the center of mass.

	\param inertia Inertia tensor.
	*/
	void getInertia(PxMat33& inertia)
	{
		for(PxU32 j=0;j<3;j++)
		{
			for(PxU32 i=0;i<3;i++)
			{
				inertia(i,j) = PxF32(COMInertiaTensor[i][j]);
			}
		}
	}

	/**
	\brief Retrieve the inertia tensor relative to the origin.

	\param inertia Inertia tensor.
	*/
	void getOriginInertia(PxMat33& inertia)
	{
		for(PxU32 j=0;j<3;j++)
		{
			for(PxU32 i=0;i<3;i++)
			{
				inertia(i,j) = PxF32(inertiaTensor[i][j]);
			}
		}
	}
	};

	bool computeVolumeIntegrals(const PxSimpleTriangleMesh& mesh, PxReal density, PxIntegrals& integrals);

	// specialized method taking polygons directly, so we don't need to compute and store triangles for each polygon
	bool computeVolumeIntegralsEberly(const PxConvexMeshDesc& mesh, PxReal density, PxIntegrals& integrals, const PxVec3& origin);   // Eberly simplified method

	// specialized method taking polygons directly, so we don't need to compute and store triangles for each polygon, SIMD version
	bool computeVolumeIntegralsEberlySIMD(const PxConvexMeshDesc& mesh, PxReal density, PxIntegrals& integrals, const PxVec3& origin);   // Eberly simplified method
}

 /** @} */
#endif
