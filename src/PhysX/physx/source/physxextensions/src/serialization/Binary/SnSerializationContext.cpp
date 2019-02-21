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

#include "PxBase.h"
#include "SnSerializationContext.h"
#include "PsFoundation.h"

using namespace physx;
using namespace Sn;

PxBase* DeserializationContext::resolveReference(PxU32 kind, size_t reference) const
{
	const InternalRefMap::Entry* entry0 = mInternalReferencesMap.find(InternalRefKey(reference, kind));
	PX_ASSERT(entry0);
	SerialObjectIndex objIndex = entry0->second;
	bool isExternal;
	PxU32 index = objIndex.getIndex(isExternal);
	PxBase* base = NULL;
	if (isExternal)
	{
		const ImportReference& entry = mImportReferences[index];
		base = mExternalRefs->find(entry.id);	
	}
	else
	{
		const ManifestEntry& entry = mManifestTable[index];
		base = reinterpret_cast<PxBase*>(mObjectDataAddress + entry.offset);
	}
	PX_ASSERT(base);
	return base;
}
	
void SerializationContext::registerReference(PxBase& serializable, PxU32 kind, size_t reference)
{
#if PX_CHECKED
	if ((kind & PX_SERIAL_REF_KIND_PTR_TYPE_BIT) == 0 && reference > 0xffffffff)
	{
		Ps::getFoundation().error(PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__, "PxSerializationContext::registerReference: only 32 bit indices supported.");
		return;
	}
#endif

	bool isExternal = mExternalRefs && mExternalRefs->contains(serializable);
	PxU32 index;
	if (isExternal)
	{
		PxSerialObjectId id = mExternalRefs->getId(serializable);
		PX_ASSERT(id != PX_SERIAL_OBJECT_ID_INVALID);
		if (const Ps::HashMap<PxSerialObjectId, PxU32>::Entry* entry = mImportReferencesMap.find(id))
		{
			index = entry->second;
		}
		else
		{
			index = mImportReferences.size();
			mImportReferencesMap.insert(id, index);
			mImportReferences.pushBack(ImportReference(id, serializable.getConcreteType()));
		}
	}
	else
	{
		PX_ASSERT(mCollection.contains(serializable));
		index = mObjToCollectionIndexMap[&serializable];
	}

	InternalRefMap& targetMap = (kind & PX_SERIAL_REF_KIND_PTR_TYPE_BIT) ? mInternalReferencesPtrMap : mInternalReferencesIdxMap;
	targetMap[InternalRefKey(reference, kind)] = SerialObjectIndex(index, isExternal);
}
