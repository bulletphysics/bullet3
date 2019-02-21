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


#ifndef PX_PHYSICS_SN_SERIALIZATION_CONTEXT
#define PX_PHYSICS_SN_SERIALIZATION_CONTEXT

#include "foundation/PxAssert.h"
#include "foundation/PxMemory.h"
#include "CmPhysXCommon.h"
#include "PsHash.h"
#include "PsUserAllocated.h"
#include "PxSerialFramework.h"
#include "CmCollection.h"
#include "CmUtils.h"
#include "PxDefaultStreams.h"
#include "PsFoundation.h"
#include "SnConvX_Align.h"

namespace physx
{
	namespace Sn
	{

		struct ManifestEntry
		{
		//= ATTENTION! =====================================================================================
		// Changing the data layout of this class breaks the binary serialization format.  See comments for 
		// PX_BINARY_SERIAL_VERSION.  If a modification is required, please adjust the getBinaryMetaData 
		// function.  If the modification is made on a custom branch, please change PX_BINARY_SERIAL_VERSION
		// accordingly.
		//==================================================================================================

			PX_FORCE_INLINE	ManifestEntry(PxU32 _offset, PxType _type)
			{
				Cm::markSerializedMem(this, sizeof(ManifestEntry));
				offset = _offset;
				type = _type;
			}			
			PX_FORCE_INLINE	ManifestEntry() { Cm::markSerializedMem(this, sizeof(ManifestEntry)); }
			PX_FORCE_INLINE void operator =(const ManifestEntry& m)
			{
				PxMemCopy(this, &m, sizeof(ManifestEntry));				
			}
	
			PxU32 offset;
			PxType type;
		};

		struct ImportReference
		{
		//= ATTENTION! =====================================================================================
		// Changing the data layout of this class breaks the binary serialization format.  See comments for 
		// PX_BINARY_SERIAL_VERSION.  If a modification is required, please adjust the getBinaryMetaData 
		// function.  If the modification is made on a custom branch, please change PX_BINARY_SERIAL_VERSION
		// accordingly.
		//==================================================================================================

			PX_FORCE_INLINE	ImportReference(PxSerialObjectId _id, PxType _type)
			{ 
				Cm::markSerializedMem(this, sizeof(ImportReference));
				id = _id;
				type = _type;
			}
			PX_FORCE_INLINE	ImportReference() { Cm::markSerializedMem(this, sizeof(ImportReference)); }
			PX_FORCE_INLINE void operator =(const ImportReference& m)
			{
				PxMemCopy(this, &m, sizeof(ImportReference));				
			}
			PxSerialObjectId id;
			PxType type;
		};

#define SERIAL_OBJECT_INDEX_TYPE_BIT (1u<<31)
		struct SerialObjectIndex
		{
		//= ATTENTION! =====================================================================================
		// Changing the data layout of this class breaks the binary serialization format.  See comments for 
		// PX_BINARY_SERIAL_VERSION.  If a modification is required, please adjust the getBinaryMetaData 
		// function.  If the modification is made on a custom branch, please change PX_BINARY_SERIAL_VERSION
		// accordingly.
		//==================================================================================================

			PX_FORCE_INLINE	SerialObjectIndex(PxU32 index, bool external) { setIndex(index, external); }
			PX_FORCE_INLINE	SerialObjectIndex(const SerialObjectIndex& objIndex) : mObjIndex(objIndex.mObjIndex) {}
			PX_FORCE_INLINE	SerialObjectIndex() : mObjIndex(PX_INVALID_U32) {}

			PX_FORCE_INLINE void setIndex(PxU32 index, bool external)
			{
				PX_ASSERT((index & SERIAL_OBJECT_INDEX_TYPE_BIT) == 0); 
				mObjIndex = index | (external ? SERIAL_OBJECT_INDEX_TYPE_BIT : 0);
			}

			PX_FORCE_INLINE PxU32 getIndex(bool& isExternal)
			{
				PX_ASSERT(mObjIndex != PX_INVALID_U32);
				isExternal = (mObjIndex & SERIAL_OBJECT_INDEX_TYPE_BIT) > 0;
				return mObjIndex & ~SERIAL_OBJECT_INDEX_TYPE_BIT;
			}

		private:
			PxU32 mObjIndex;
		};

		struct ExportReference
		{
		//= ATTENTION! =====================================================================================
		// Changing the data layout of this class breaks the binary serialization format.  See comments for 
		// PX_BINARY_SERIAL_VERSION.  If a modification is required, please adjust the getBinaryMetaData 
		// function.  If the modification is made on a custom branch, please change PX_BINARY_SERIAL_VERSION
		// accordingly.
		//==================================================================================================

			PX_FORCE_INLINE	ExportReference(PxSerialObjectId _id, SerialObjectIndex _objIndex)
			{
				Cm::markSerializedMem(this, sizeof(ExportReference));
				id = _id;
				objIndex = _objIndex;
			}
			PX_FORCE_INLINE	ExportReference() { Cm::markSerializedMem(this, sizeof(ExportReference)); }
			PX_FORCE_INLINE void operator =(const ExportReference& m)
			{
				PxMemCopy(this, &m, sizeof(ExportReference));				
			}
			PxSerialObjectId id;
			SerialObjectIndex objIndex;
		};

		template<class ReferenceType>
		struct InternalReference
		{
		//= ATTENTION! =====================================================================================
		// Changing the data layout of this class breaks the binary serialization format.  See comments for 
		// PX_BINARY_SERIAL_VERSION.  If a modification is required, please adjust the getBinaryMetaData 
		// function.  If the modification is made on a custom branch, please change PX_BINARY_SERIAL_VERSION
		// accordingly.
		//==================================================================================================

			PX_FORCE_INLINE	InternalReference(ReferenceType _reference, PxU32 _kind, SerialObjectIndex _objIndex)
			{
				Cm::markSerializedMem(this, sizeof(InternalReference));
				reference = _reference;
				kind = _kind;
				objIndex = _objIndex;
			}
			PX_FORCE_INLINE	InternalReference() { Cm::markSerializedMem(this, sizeof(InternalReference)); }
			PX_FORCE_INLINE void operator =(const InternalReference& m)
			{
				PxMemCopy(this, &m, sizeof(InternalReference));				
			}
			ReferenceType reference;
			PxU32 kind;
			SerialObjectIndex objIndex;
		};

		typedef InternalReference<size_t> InternalReferencePtr;
		typedef InternalReference<PxU32> InternalReferenceIdx;

		typedef shdfnd::Pair<size_t, PxU32> InternalRefKey;
		typedef Cm::CollectionHashMap<InternalRefKey, SerialObjectIndex> InternalRefMap;

		class DeserializationContext : public PxDeserializationContext, public Ps::UserAllocated
		{
			PX_NOCOPY(DeserializationContext)
		
		public:
			DeserializationContext(const ManifestEntry* manifestTable, 
								   const ImportReference* importReferences,
								   PxU8* objectDataAddress, 
								   const InternalRefMap& internalReferencesMap, 
								   const Cm::Collection* externalRefs,
								   PxU8* extraData,
								   PxU32 physxVersion)
			: mManifestTable(manifestTable)
			, mImportReferences(importReferences)
			, mObjectDataAddress(objectDataAddress)
			, mInternalReferencesMap(internalReferencesMap)
			, mExternalRefs(externalRefs)
			, mPhysXVersion(physxVersion)
			{
				mExtraDataAddress = extraData;
			}

			virtual	PxBase*	resolveReference(PxU32 kind, size_t reference) const;

			PxU32 getPhysXVersion() const { return mPhysXVersion; }
		private:
			//various pointers to deserialized data
			const ManifestEntry* mManifestTable;
			const ImportReference* mImportReferences;
			PxU8* mObjectDataAddress;

			//internal references map for resolving references.
			const InternalRefMap& mInternalReferencesMap;

			//external collection for resolving import references.
			const Cm::Collection* mExternalRefs;
			const PxU32 mPhysXVersion;
		};

		class SerializationContext : public PxSerializationContext, public Ps::UserAllocated
		{
			PX_NOCOPY(SerializationContext)
		public:
			SerializationContext(const Cm::Collection& collection, const Cm::Collection* externalRefs) 
			: mCollection(collection)
			, mExternalRefs(externalRefs) 
			{
				// fill object to collection index map (same ordering as manifest)
				for (PxU32 i=0;i<mCollection.internalGetNbObjects();i++)
				{
					mObjToCollectionIndexMap[mCollection.internalGetObject(i)] = i;
				}
			}

			virtual		void		writeData(const void* buffer, PxU32 size)		{	mMemStream.write(buffer, size);	}
			virtual		PxU32		getTotalStoredSize()							{	return mMemStream.getSize(); }
			virtual		void		alignData(PxU32 alignment = PX_SERIAL_ALIGN)		
			{	
				if(!alignment)
					return;

				PxI32 bytesToPad = PxI32(getPadding(mMemStream.getSize(), alignment));
				static const PxI32 BUFSIZE = 64;
				char buf[BUFSIZE];
				PxMemSet(buf, 0, bytesToPad < BUFSIZE ? PxU32(bytesToPad) : PxU32(BUFSIZE));
				while(bytesToPad > 0)
				{
					mMemStream.write(buf, bytesToPad < BUFSIZE ? PxU32(bytesToPad) : PxU32(BUFSIZE));
					bytesToPad -= BUFSIZE;
				}
				PX_ASSERT(!getPadding(getTotalStoredSize(), alignment));
			}

			virtual void writeName(const char*)
			{
				Ps::getFoundation().error(physx::PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, 
					"Cannot export names during exportData.");
			}

			const PxCollection& getCollection() const	{	return mCollection;		}

			virtual void registerReference(PxBase& serializable, PxU32 kind, size_t reference);

			const Ps::Array<ImportReference>& getImportReferences() { return mImportReferences; }
			InternalRefMap& getInternalReferencesPtrMap() { return mInternalReferencesPtrMap; }
			InternalRefMap& getInternalReferencesIdxMap() { return mInternalReferencesIdxMap; }

			PxU32		getSize()	const	{	return mMemStream.getSize(); }
			PxU8*		getData()	const	{	return mMemStream.getData(); }



		private:
			//import reference map for unique registration of import references and corresponding buffer.
			Ps::HashMap<PxSerialObjectId, PxU32> mImportReferencesMap;
			Ps::Array<ImportReference> mImportReferences;
			
			//maps for unique registration of internal references
			InternalRefMap mInternalReferencesPtrMap;
			InternalRefMap mInternalReferencesIdxMap;

			//map for quick lookup of manifest index. 
			Ps::HashMap<const PxBase*, PxU32> mObjToCollectionIndexMap;

			//collection and externalRefs collection for assigning references.
			const Cm::Collection& mCollection;
			const Cm::Collection* mExternalRefs;

			PxDefaultMemoryOutputStream mMemStream;

		};

	} // namespace Sn
}

#endif
