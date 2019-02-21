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

#ifndef PX_PHYSICS_NP_CAST
#define PX_PHYSICS_NP_CAST

#include "PxPhysXConfig.h"
#include "NpScene.h"
#include "NpRigidDynamic.h"
#include "NpRigidStatic.h"
#include "NpArticulation.h"
#include "NpArticulationReducedCoordinate.h"
#include "NpArticulationLink.h"
#include "NpArticulationJoint.h"
#include "NpAggregate.h"

namespace physx
{
	// PT: Scb-to-Np casts

	PX_FORCE_INLINE const NpScene* getNpScene(const Scb::Scene* scene)
	{
		const size_t scbOffset = reinterpret_cast<size_t>(&(reinterpret_cast<NpScene*>(0)->getScene()));
		return reinterpret_cast<const NpScene*>(reinterpret_cast<const char*>(scene) - scbOffset);
	}

	PX_FORCE_INLINE const NpRigidDynamic* getNpRigidDynamic(const Scb::Body* scbBody)
	{
		const size_t offset = reinterpret_cast<size_t>(&(reinterpret_cast<NpRigidDynamic*>(0)->getScbActorFast()));
		return reinterpret_cast<const NpRigidDynamic*>(reinterpret_cast<const char*>(scbBody) - offset);
	}

	PX_FORCE_INLINE const NpRigidStatic* getNpRigidStatic(const Scb::RigidStatic* scbRigidStatic)
	{
		const size_t offset = reinterpret_cast<size_t>(&(reinterpret_cast<NpRigidStatic*>(0)->getScbActorFast()));
		return reinterpret_cast<const NpRigidStatic*>(reinterpret_cast<const char*>(scbRigidStatic) - offset);
	}

	PX_FORCE_INLINE const NpShape* getNpShape(const Scb::Shape* scbShape)
	{
		const size_t offset = reinterpret_cast<size_t>(&(reinterpret_cast<NpShape*>(0)->getScbShape()));
		return reinterpret_cast<const NpShape*>(reinterpret_cast<const char*>(scbShape) - offset);
	}

	PX_FORCE_INLINE const NpArticulationLink* getNpArticulationLink(const Scb::Body* scbArticulationLink)
	{
		const size_t offset = reinterpret_cast<size_t>(&(reinterpret_cast<NpArticulationLink*>(0)->getScbActorFast()));
		return reinterpret_cast<const NpArticulationLink*>(reinterpret_cast<const char*>(scbArticulationLink) - offset);
	}

	PX_FORCE_INLINE const NpArticulation* getNpArticulation(const Scb::Articulation* scbArticulation)
	{
		const size_t offset = reinterpret_cast<size_t>(&(reinterpret_cast<NpArticulation*>(0)->mImpl.getArticulation()));
		return reinterpret_cast<const NpArticulation*>(reinterpret_cast<const char*>(scbArticulation) - offset);
	}

	PX_FORCE_INLINE const NpArticulationReducedCoordinate* getNpArticulationRC(const Scb::Articulation* scbArticulation)
	{
		const size_t offset = reinterpret_cast<size_t>(&(reinterpret_cast<NpArticulationReducedCoordinate*>(0)->mImpl.getArticulation()));
		return reinterpret_cast<const NpArticulationReducedCoordinate*>(reinterpret_cast<const char*>(scbArticulation) - offset);
	}

	PX_FORCE_INLINE const NpArticulationJoint* getNpArticulationJoint(const Scb::ArticulationJoint* scbArticulationJoint)
	{
		const size_t offset = reinterpret_cast<size_t>(&(reinterpret_cast<NpArticulationJoint*>(0)->getScbArticulationJoint()));
		return reinterpret_cast<const NpArticulationJoint*>(reinterpret_cast<const char*>(scbArticulationJoint) - offset);
	}

	PX_FORCE_INLINE const NpAggregate* getNpAggregate(const Scb::Aggregate* aggregate)
	{
		const size_t offset = reinterpret_cast<size_t>(&(reinterpret_cast<NpAggregate*>(0)->getScbAggregate()));
		return reinterpret_cast<const NpAggregate*>(reinterpret_cast<const char*>(aggregate) - offset);
	}

}

#endif
