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
// Copyright (c) 2008-2018 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#include "ScbBase.h"

using namespace physx;
using namespace Scb;

#include "ScbActor.h"
#include "ScbRigidStatic.h"
#include "ScbBody.h"

Actor::Offsets::Offsets()
{
	const size_t staticOffset	= reinterpret_cast<size_t>(&(reinterpret_cast<Scb::RigidStatic*>(0)->getScStatic()));
	const size_t bodyOffset		= reinterpret_cast<size_t>(&(reinterpret_cast<Scb::Body*>(0)->getScBody()));

	scToScb[PxActorType::eRIGID_STATIC] = staticOffset;
	scToScb[PxActorType::eRIGID_DYNAMIC] = bodyOffset;
	scToScb[PxActorType::eARTICULATION_LINK] = bodyOffset;

	scbToSc[ScbType::eRIGID_STATIC] = staticOffset;
	scbToSc[ScbType::eBODY] = bodyOffset;
	scbToSc[ScbType::eBODY_FROM_ARTICULATION_LINK] = bodyOffset;
}

const Actor::Offsets Actor::sOffsets;
