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


#include "PxRigidActorExt.h"
#include "PsAllocator.h"
#include "PsInlineArray.h"
#include "CmPhysXCommon.h"
#include "PxShape.h"
#include "PxGeometryQuery.h"

using namespace physx;

PxBounds3*	PxRigidActorExt::getRigidActorShapeLocalBoundsList(const PxRigidActor& actor, PxU32& numBounds)
{
	const PxU32 numShapes = actor.getNbShapes();
	if(numShapes == 0)
		return NULL;
	
	Ps::InlineArray<PxShape*, 64> shapes("PxShape*"); 
	shapes.resize(numShapes);

	actor.getShapes(shapes.begin(), shapes.size());

	PxU32 numSqShapes = 0;
	for(PxU32 i = 0; i < numShapes; i++)
	{
		if(shapes[i]->getFlags() & PxShapeFlag::eSCENE_QUERY_SHAPE)
			numSqShapes++;
	}

	PxBounds3* bounds = reinterpret_cast<PxBounds3*>(PX_ALLOC(numSqShapes * sizeof(PxBounds3), "PxBounds3"));

	numSqShapes = 0;
	for(PxU32 i = 0; i < numShapes; i++)
	{
		if(shapes[i]->getFlags() & PxShapeFlag::eSCENE_QUERY_SHAPE)
		{			
			bounds[numSqShapes++] = PxGeometryQuery::getWorldBounds(shapes[i]->getGeometry().any(), shapes[i]->getLocalPose(), 1.0f);
		}
	}

	numBounds = numSqShapes;
	return bounds;
}
