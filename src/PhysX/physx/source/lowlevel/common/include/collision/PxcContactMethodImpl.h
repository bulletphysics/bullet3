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


#ifndef PXC_CONTACTMETHODIMPL_H
#define PXC_CONTACTMETHODIMPL_H

#include "GuGeometryUnion.h"
#include "CmPhysXCommon.h"
#include "GuContactMethodImpl.h"

namespace physx
{
namespace Cm
{
	class RenderOutput;
}

namespace Gu
{
	class ContactBuffer;
	struct Cache;
	struct NarrowPhaseParams;
}

struct PxcNpCache;
class PxcNpThreadContext;
class PxsContext;
class PxsRigidBody;
struct PxsCCDShape;

namespace Cm
{
	class FastVertex2ShapeScaling;
}

/*!\file
This file contains forward declarations of all implemented contact methods.
*/


/*! Parameter list without names to avoid unused parameter warnings 
*/
#define CONTACT_METHOD_ARGS_UNUSED			\
	const Gu::GeometryUnion&,				\
	const Gu::GeometryUnion&,				\
	const PxTransform&,						\
	const PxTransform&,						\
	const Gu::NarrowPhaseParams&,			\
	Gu::Cache&,								\
	Gu::ContactBuffer&,						\
	Cm::RenderOutput*


/*!
Method prototype for contact generation routines
*/
typedef bool (*PxcContactMethod) (GU_CONTACT_METHOD_ARGS);


// Matrix of types
extern PxcContactMethod g_ContactMethodTable[][PxGeometryType::eGEOMETRY_COUNT];
extern const bool g_CanUseContactCache[][PxGeometryType::eGEOMETRY_COUNT];
extern PxcContactMethod g_PCMContactMethodTable[][PxGeometryType::eGEOMETRY_COUNT];

extern bool gEnablePCMCaching[][PxGeometryType::eGEOMETRY_COUNT];
}

#endif
