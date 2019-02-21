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

#include "PxMetaData.h"

#include "SqPruningStructure.h"

using namespace physx;
using namespace Sq;

///////////////////////////////////////////////////////////////////////////////

void PruningStructure::getBinaryMetaData(PxOutputStream& stream)
{	
	PX_DEF_BIN_METADATA_VCLASS(stream, PruningStructure)
		PX_DEF_BIN_METADATA_BASE_CLASS(stream, PruningStructure, PxBase)		

		PX_DEF_BIN_METADATA_ITEM(stream, PruningStructure, PxU32, mNbNodes[0], 0)
		PX_DEF_BIN_METADATA_ITEM(stream, PruningStructure, PxU32, mNbNodes[1], 0)
		PX_DEF_BIN_METADATA_ITEM(stream, PruningStructure, AABBTreeRuntimeNode, mAABBTreeNodes[0], PxMetaDataFlag::ePTR)
		PX_DEF_BIN_METADATA_ITEM(stream, PruningStructure, AABBTreeRuntimeNode, mAABBTreeNodes[1], PxMetaDataFlag::ePTR)
		PX_DEF_BIN_METADATA_ITEM(stream, PruningStructure, PxU32, mNbObjects[0], 0)
		PX_DEF_BIN_METADATA_ITEM(stream, PruningStructure, PxU32, mNbObjects[1], 0)
		PX_DEF_BIN_METADATA_ITEM(stream, PruningStructure, PxU32, mAABBTreeIndices[0], PxMetaDataFlag::ePTR)
		PX_DEF_BIN_METADATA_ITEM(stream, PruningStructure, PxU32, mAABBTreeIndices[1], PxMetaDataFlag::ePTR)
		PX_DEF_BIN_METADATA_ITEM(stream, PruningStructure, PxU32, mNbActors, 0)
		PX_DEF_BIN_METADATA_ITEM(stream, PruningStructure, PxActor*, mActors, PxMetaDataFlag::ePTR)
		PX_DEF_BIN_METADATA_ITEM(stream, PruningStructure, bool, mValid, 0)
}


