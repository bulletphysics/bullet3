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

#ifndef PXC_NPCONTACTPREPSHARED_H
#define PXC_NPCONTACTPREPSHARED_H

namespace physx
{
class PxcNpThreadContext;
struct PxsMaterialInfo;
class PxsMaterialManager;
class PxsConstraintBlockManager;
class PxcConstraintBlockStream;
struct PxsContactManagerOutput;

namespace Gu
{
	struct ContactPoint;
}

static const PxReal PXC_SAME_NORMAL = 0.999f; //Around 6 degrees

PxU32 writeCompressedContact(const Gu::ContactPoint* const PX_RESTRICT contactPoints, const PxU32 numContactPoints, PxcNpThreadContext* threadContext,
									PxU8& writtenContactCount, PxU8*& outContactPatches, PxU8*& outContactPoints, PxU16& compressedContactSize, PxReal*& contactForces, PxU32 contactForceByteSize,
									const PxsMaterialManager* materialManager, bool hasModifiableContacts, bool forceNoResponse, PxsMaterialInfo* PX_RESTRICT pMaterial, PxU8& numPatches,
									PxU32 additionalHeaderSize = 0,  PxsConstraintBlockManager* manager = NULL, PxcConstraintBlockStream* blockStream = NULL, bool insertAveragePoint = false,
									PxcDataStreamPool* pool = NULL, PxcDataStreamPool* patchStreamPool = NULL, PxcDataStreamPool* forcePool = NULL, const bool isMeshType = false);

}

#endif
