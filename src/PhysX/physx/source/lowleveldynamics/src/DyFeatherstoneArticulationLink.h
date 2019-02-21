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


#ifndef PXD_FEATHERSTONE_ARTICULATION_LINK_H
#define PXD_FEATHERSTONE_ARTICULATION_LINK_H

#include "foundation/PxVec3.h"
#include "foundation/PxQuat.h"
#include "foundation/PxTransform.h"
#include "PsVecMath.h"
#include "CmUtils.h"
#include "CmSpatialVector.h"
#include "DyVArticulation.h"
#include "DyFeatherstoneArticulationUtils.h"

namespace physx
{
	namespace Dy
	{

		class ArticulationLinkData
		{
		public:
			ArticulationLinkData()
			{
				maxPenBias = 0.f;
			}

			Cm::SpatialVectorF				Is[6];//stI is the transpose of Is
			Cm::SpatialVectorF				IsInvD[6];
			SpatialMatrix					spatialInertia;
			SpatialMatrix					spatialArticulatedInertia;
			SpatialTransform				childToParent;
			SpatialTransform				childToBase;
			PxVec3							r; //vector from parent com to child com
			PxVec3							rw; //vector from parent com to child com
			PxReal							qstZIc[6];//jointForce - stZIc
			PxReal							invStIs[6][6];
			PxReal							maxPenBias;

		};

		struct ArticSolverData
		{
			SpatialTransform				childToParent;
			PxReal							invStIs[6][6];
		};

	}//namespace Dy
}

#endif
