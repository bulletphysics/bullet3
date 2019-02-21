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

#include "foundation/PxErrorCallback.h"
#include "SnConvX.h"
#include "serialization/SnSerialUtils.h"
#include "PsAlloca.h"
#include "CmUtils.h"
#include "PxDefaultStreams.h"
#include <assert.h>

using namespace physx;
using namespace physx::Sn;
using namespace Cm;

void Sn::ConvX::resetConvexFlags()
{
	mConvexFlags.clear();
}

void Sn::ConvX::_enumerateFields(const MetaClass* mc, ExtraDataEntry2* entries, int& nb, int baseOffset, MetaDataType type) const
{
	PxU32 nbFields = mc->mFields.size();
	int offsetCheck = baseOffset;
	for(PxU32 j=0;j<nbFields;j++)
	{
		const PxMetaDataEntry& entry = mc->mFields[j];
		if(entry.mFlags & PxMetaDataFlag::eCLASS || entry.mFlags & PxMetaDataFlag::eEXTRA_DATA)
			continue;

		assert(offsetCheck == baseOffset + entry.mOffset);

		int currentOffset = baseOffset + entry.mOffset;

		//for(int c=0;c<entry.mCount;c++)
		{
			if(entry.mFlags & PxMetaDataFlag::eUNION)
			{
				entries[nb].entry	= entry;
				entries[nb].offset	= currentOffset;
				entries[nb].cb		= 0;
				nb++;
			}
			else if(entry.mFlags & PxMetaDataFlag::ePTR)	// This also takes care of the vtable pointer
			{
				entries[nb].entry	= entry;
				entries[nb].offset	= currentOffset;
				entries[nb].cb		= &Sn::ConvX::convertPtr;
				nb++;
			}
			else
			{
				MetaClass* fieldType = getMetaClass(entry.mType, type);
				assert(fieldType);
				if(fieldType->mCallback)
				{
					entries[nb].entry	= entry;
					entries[nb].offset	= currentOffset;
					entries[nb].cb		= fieldType->mCallback;
					nb++;
				}
				else
				{
					for(int c=0;c<entry.mCount;c++)
					{
						_enumerateFields(fieldType, entries, nb, currentOffset, type);
						currentOffset += entry.mSize/entry.mCount;
					}
				}
			}
		}
		offsetCheck += entry.mSize;
	}
}

void Sn::ConvX::_enumerateExtraData(const char* address, const MetaClass* mc, ExtraDataEntry* entries,
	                                int& nb, int offset, MetaDataType type) const
{
	PxU32 nbFields = mc->mFields.size();
	for(PxU32 j=0;j<nbFields;j++)
	{
		const PxMetaDataEntry& entry = mc->mFields[j];
		if(entry.mFlags & PxMetaDataFlag::eCLASS /*|| entry.mFlags & PxMetaDataFlag::ePTR*/ || entry.mFlags & PxMetaDataFlag::eTYPEDEF)
			continue;

		const char* entryType = entry.mType;

		//
		// Insanely Twisted Shadow GeometryUnion
		//
		// Special code is needed as long as there are no meta data tags to describe our unions properly. The way it is done here is
		// not future-proof at all. There should be a tag to describe where the union type can be found and the number of bytes
		// this type id needs. Then a mapping needs to get added from each union type id to the proper meta class name.
		//
		if (entry.mFlags & PxMetaDataFlag::eUNION)
		{
			if (!mc->mClassName || strcmp(mc->mClassName, "Gu::GeometryUnion")!=0)
				continue;
			else
			{
				// ### hardcoded bit here, will only work when union type is the first int of the struct
				const int* tmp = reinterpret_cast<const int*>(address + offset);
				const int unionType = *tmp;

				ConvX* tmpConv = const_cast<ConvX*>(this);  // ... don't ask
				const char* typeName = tmpConv->getTypeName(entry.mType, unionType);
				assert(typeName);

				bool isTriMesh = (strcmp(typeName, "PxTriangleMeshGeometryLL") == 0);
				bool isHeightField = (strcmp(typeName, "PxHeightFieldGeometryLL") == 0);
				if (!isTriMesh && !isHeightField)
				{
					continue;
				}
				else
				{
					entryType = typeName;
				}
			}
		}

		//		MetaClass* extraDataType = getMetaClass(entry.mType, type);
		//		if(!extraDataType)
		//			continue;

		if(entry.mFlags & PxMetaDataFlag::eEXTRA_DATA)
		{
			entries[nb].entry = entry;
			entries[nb].offset = offset+entry.mOffset;
			nb++;
		}
		else
		{
			if(entry.mFlags & PxMetaDataFlag::ePTR)
				continue;

			MetaClass* extraDataType = getMetaClass(entryType, type);
			if(!extraDataType)
				continue;

			if(!extraDataType->mCallback)
				_enumerateExtraData(address, extraDataType, entries, nb, offset+entry.mOffset, type);
		}
	}
}

PxU64 Sn::ConvX::read64(const void*& buffer)
{
	const PxU64* buf64 = reinterpret_cast<const PxU64*>(buffer);
	buffer = reinterpret_cast<const void*>(size_t(buffer) + sizeof(PxU64));
	PxU64 value = *buf64;
	output(value);
	return value;
}

int Sn::ConvX::read32(const void*& buffer)
{
	const int* buf32 = reinterpret_cast<const int*>(buffer);
	buffer = reinterpret_cast<const void*>(size_t(buffer) + sizeof(int));
	int value = *buf32;
	output(value);
	return value;
}

short Sn::ConvX::read16(const void*& buffer)
{
	const short* buf16 = reinterpret_cast<const short*>(buffer);
	buffer = reinterpret_cast<const void*>(size_t(buffer) + sizeof(short));
	short value = *buf16;
	output(value);
	return value;
}

#if PX_CHECKED
extern const char* gVTable;
static bool compareEntries(const ExtraDataEntry2& e0, const ExtraDataEntry2& e1)
{
	if(e0.entry.isVTablePtr() && e1.entry.isVTablePtr())
		return true;

	if((e0.entry.mFlags & PxMetaDataFlag::eUNION) && (e1.entry.mFlags & PxMetaDataFlag::eUNION))
	{
		if(e0.entry.mType && e1.entry.mType)
		{
		    // We can't compare the ptrs since they index different string tables
		    if(strcmp(e0.entry.mType, e1.entry.mType)==0)
			   return true;
		}		
		return false;
	}

	if(e0.entry.mName && e1.entry.mName)
	{
	    // We can't compare the ptrs since they index different string tables
	    if(strcmp(e0.entry.mName, e1.entry.mName)==0)
		   return true;
	}
	return false;
}
#endif

// TODO: optimize this
bool Sn::ConvX::convertClass(const char* buffer, const MetaClass* mc, int offset)
{
	// ---- big convex surgery ----
	bool convexSurgery = false;
	bool foundNbVerts = false;
	bool removeBigData = false;

	// force reference
	(void)foundNbVerts;

	displayMessage(PxErrorCode::eDEBUG_INFO, "%s\n", mc->mClassName);
	displayMessage(PxErrorCode::eDEBUG_INFO, "+++++++++++++++++++++++++++++++++++++++++++++\n");

	if(strcmp(mc->mClassName, "ConvexMesh")==0)
	{
		convexSurgery = true;
	}
	// ---- big convex surgery ----

	int nbSrcEntries = 0;
	PX_ALLOCA(srcEntries, ExtraDataEntry2, 256);  // ### painful ctors here
	int nbDstEntries = 0;
	PX_ALLOCA(dstEntries, ExtraDataEntry2, 256);  // ### painful ctors here
	// Find corresponding meta-class for target platform
	const MetaClass* target_mc = getMetaClass(mc->mClassName, META_DATA_DST);
	assert(target_mc);

	if(mc->mCallback)
	{
		srcEntries[0].cb			= mc->mCallback;
		srcEntries[0].offset		= offset;
		srcEntries[0].entry.mType	= mc->mClassName;
		srcEntries[0].entry.mName	= mc->mClassName;
		srcEntries[0].entry.mOffset	= offset;
		srcEntries[0].entry.mSize	= mc->mSize;
		srcEntries[0].entry.mCount	= 1;
		srcEntries[0].entry.mFlags	= 0;
		nbSrcEntries = 1;

		assert(target_mc->mCallback);
		dstEntries[0].cb			= target_mc->mCallback;
		dstEntries[0].offset		= offset;
		dstEntries[0].entry.mType	= target_mc->mClassName;
		dstEntries[0].entry.mName	= target_mc->mClassName;
		dstEntries[0].entry.mOffset	= offset;
		dstEntries[0].entry.mSize	= target_mc->mSize;
		dstEntries[0].entry.mCount	= 1;
		dstEntries[0].entry.mFlags	= 0;
		nbDstEntries = 1;
	}
	else
	{
		nbSrcEntries = 0;
		_enumerateFields(mc, srcEntries, nbSrcEntries, 0, META_DATA_SRC);
		assert(nbSrcEntries<256);

		nbDstEntries = 0;
		_enumerateFields(target_mc, dstEntries, nbDstEntries, 0, META_DATA_DST);
		assert(nbDstEntries<256);

		//		nb = mc->mNbEntries;
		//		assert(nb>=0);
		//		memcpy(entries, mc->mEntries, nb*sizeof(ExtraDataEntry2));
	}

	int srcOffsetCheck = 0;
	int dstOffsetCheck = 0;
	int j = 0;
	// Track cases where the vtable pointer location is different for different platforms.
	// The variables indicate whether a platform has a vtable pointer entry that has not been converted yet
	// and they will remember the index of the corrssponding entry. This works because there can only
	// be one open vtable pointer entry at a time.
	int srcOpenVTablePtrEntry = -1;
	int dstOpenVTablePtrEntry = -1;

	 //if the src and dst platform place the vtable pointers at different locations some fiddling with the iteration count can be necessary.
	int addVTablePtrShiftIteration = 0; 
	const int maxNb = nbSrcEntries > nbDstEntries ? nbSrcEntries : nbDstEntries;
	for(int i=0; i < (maxNb + addVTablePtrShiftIteration); i++)
	{

		if (i < nbSrcEntries)
		{
			displayMessage(PxErrorCode::eDEBUG_INFO, "\t0x%p\t%02x\t%d\t%d\t%s", buffer + srcOffsetCheck, 
				static_cast<unsigned char>(buffer[srcOffsetCheck]), srcOffsetCheck, srcEntries[i].entry.mOffset, srcEntries[i].entry.mName);
			for (int byteCount = 1; byteCount < srcEntries[i].entry.mSize; ++byteCount) 
				displayMessage(PxErrorCode::eDEBUG_INFO, "\t0x%p\t%02x\t%d\t%d\t.", buffer + srcOffsetCheck + byteCount,
				static_cast<unsigned char>(buffer[srcOffsetCheck + byteCount]), srcOffsetCheck + byteCount, srcEntries[i].entry.mOffset + byteCount);
		}

		bool handlePadding = true;
		bool skipLoop = false;
		while(handlePadding)
		{
			const int pad0 = i<nbSrcEntries ? srcEntries[i].entry.mFlags & PxMetaDataFlag::ePADDING : 0;
			const int pad1 = j<nbDstEntries ? dstEntries[j].entry.mFlags & PxMetaDataFlag::ePADDING : 0;
			if(pad0 || pad1)
			{
				if(pad0)
				{
#if PX_CHECKED
					if (mMarkedPadding && (strcmp(srcEntries[i].entry.mType, "paddingByte")==0))
						if(!checkPaddingBytes(buffer + srcOffsetCheck, srcEntries[i].entry.mSize))
						{							
							if(i>0)
							{
							    displayMessage(PxErrorCode::eDEBUG_WARNING,
                                  "PxBinaryConverter warning: Bytes after %s::%s don't look like padding bytes. Likely mismatch between binary data and metadata.\n",
							       mc->mClassName, srcEntries[i-1].entry.mName );
							}
							else
								displayMessage(PxErrorCode::eDEBUG_WARNING,
                                     "PxBinaryConverter warning: Bytes after %s don't look like padding bytes. Likely mismatch between binary data and metadata.\n",
							          mc->mClassName);

						}
#endif
						if(pad1)
						{
							// Both have padding
							// ### check sizes, output bytes
							if(srcEntries[i].entry.mSize==dstEntries[j].entry.mSize)
							{
								// I guess we can just go on with the normal code here
								handlePadding = false;
							}
							else
							{
								// Output padding
								assert(srcEntries[i].cb);
								assert(srcEntries[i].offset == srcOffsetCheck);

								const int padSize = dstEntries[j].entry.mSize;
								char* paddingBytes = reinterpret_cast<char*>(PX_ALLOC(sizeof(char)*padSize, "paddingByte"));
								memset(paddingBytes, 0, size_t(padSize));
								assert(dstEntries[j].cb);
								(this->*dstEntries[j].cb)(paddingBytes, dstEntries[j].entry, dstEntries[j].entry);
								assert(dstOffsetCheck==dstEntries[j].offset);
								dstOffsetCheck += padSize;
								PX_FREE(paddingBytes);

								//						srcEntries[i].cb(buffer+srcOffsetCheck, srcEntries[i].entry, dstEntries[j].entry);
								//						assert(dstOffsetCheck==dstEntries[j].offset);
								//						dstOffsetCheck += dstEntries[j].entry.mSize;
								srcOffsetCheck += srcEntries[i].entry.mSize;

								// Skip dest padding field
								j++;

								//							continue;	// ### BUG, doesn't go back to the "for"
								skipLoop = true;
								handlePadding = false;
							}
						}
						else
						{
							// Src has padding, dst has not => skip conversion
							// Don't increase j
							skipLoop = true;
							handlePadding = false;
							srcOffsetCheck += srcEntries[i].entry.mSize;
						}
				}
				else
				{
					if(pad1)
					{
						// Dst has padding, src has not

						// Output padding
						const int padSize = dstEntries[j].entry.mSize;
						char* paddingBytes = reinterpret_cast<char*>(PX_ALLOC(sizeof(char)*padSize, "paddingByte"));
						memset(paddingBytes, 0, size_t(padSize));
						assert(dstEntries[j].cb);
						(this->*dstEntries[j].cb)(paddingBytes, dstEntries[j].entry, dstEntries[j].entry);
						assert(dstOffsetCheck==dstEntries[j].offset);
						dstOffsetCheck += padSize;
						PX_FREE(paddingBytes);

						// Skip dest padding field, keep same src field
						j++;
					}
					else
					{
						assert(0);
					}
				}
			}
			else handlePadding = false;
		}

		if(skipLoop)
			continue;

		int modSrcOffsetCheck = srcOffsetCheck;
		const ExtraDataEntry2* srcEntryPtr = &srcEntries[i];
		const ExtraDataEntry2* dstEntryPtr = &dstEntries[j];

		bool isSrcVTablePtr = (i < nbSrcEntries) ? srcEntryPtr->entry.isVTablePtr() : false;
		if (isSrcVTablePtr && (dstOpenVTablePtrEntry != -1))
		{
			// vtable ptr position mismatch:
			// this check is necessary to align src and dst index again when the
			// dst vtable pointer has been written already and the src vtable ptr
			// element is reached.
			//
			//                       i
			// src: | a      | b | vt-ptr | c | ...
			// dst: | vt-ptr | a | b      | c | ...
			//                              j 
			//
			// it needs special treatment because the following case fails otherwise
			//                       i
			// src: | a      | b | vt-ptr | c      | vt-ptr | ...
			// dst: | vt-ptr | a | b      | vt-ptr | c      | ...
			//                                j 

			//
			// This entry has been written already -> advance to next src entry
			//
			srcOffsetCheck += srcEntryPtr->entry.mSize;
			i++;
			isSrcVTablePtr = (i < nbSrcEntries) ? srcEntryPtr->entry.isVTablePtr() : false;
			PX_ASSERT(dstOpenVTablePtrEntry < nbDstEntries);
			PX_ASSERT(dstEntries[dstOpenVTablePtrEntry].entry.isVTablePtr());
			dstOpenVTablePtrEntry = -1;
			PX_ASSERT(addVTablePtrShiftIteration == 0);
		}
		bool isDstVTablePtr = (j < nbDstEntries) ? dstEntryPtr->entry.isVTablePtr() : false;
		if (isDstVTablePtr && (srcOpenVTablePtrEntry != -1))
		{
			//                              i
			// src: | vt-ptr | a | b      | c | ...
			// dst: | a      | b | vt-ptr | c | ...
			//                       j 

			i--;  // next iteration the current element should get processed
			isSrcVTablePtr = true;
			PX_ASSERT(srcOpenVTablePtrEntry < nbSrcEntries);
			srcEntryPtr = &srcEntries[srcOpenVTablePtrEntry];
			PX_ASSERT(srcEntryPtr->entry.isVTablePtr());
			modSrcOffsetCheck = srcEntryPtr->offset;
			srcOffsetCheck -= srcEntryPtr->entry.mSize;  // to make sure total change is 0 after this iteration
			srcOpenVTablePtrEntry = -1;
			PX_ASSERT(addVTablePtrShiftIteration == 1);
			addVTablePtrShiftIteration = 0;
		}

		if(i==nbSrcEntries && j==nbDstEntries)
		{
			PX_ASSERT((srcOpenVTablePtrEntry == -1) && (dstOpenVTablePtrEntry == -1));
			break;
		}

		if (isSrcVTablePtr || isDstVTablePtr)
		{
            if (!isSrcVTablePtr)
			{
				//          i
				// src: | a      | b | vt-ptr | c | ...
				// dst: | vt-ptr | a | b      | c | ...
				//          j 

				PX_ASSERT(dstOpenVTablePtrEntry == -1);  // the other case should be detected and treated earlier
				PX_ASSERT(srcOpenVTablePtrEntry == -1);
				PX_ASSERT(addVTablePtrShiftIteration == 0);

				int k;
				for(k=i+1; k < nbSrcEntries; k++)
				{
					if (srcEntries[k].entry.isVTablePtr())
						break;
				}
				PX_ASSERT(k < nbSrcEntries);

				srcEntryPtr = &srcEntries[k];
				modSrcOffsetCheck = srcEntryPtr->offset;
				srcOffsetCheck -= srcEntryPtr->entry.mSize;  // to make sure total change is 0 after this iteration

				dstOpenVTablePtrEntry = j;
				i--; // to make sure the original entry gets processed in the next iteration
			}
			else if (!isDstVTablePtr)
			{
				//          i ---> i
				// src: | vt-ptr | a | b      | c | ...
				// dst: | a      | b | vt-ptr | c | ...
				//          j 

				PX_ASSERT(srcOpenVTablePtrEntry == -1);  // the other case should be detected and treated earlier
				PX_ASSERT(dstOpenVTablePtrEntry == -1);
				PX_ASSERT(addVTablePtrShiftIteration == 0);

				srcOffsetCheck += srcEntryPtr->entry.mSize;
				modSrcOffsetCheck = srcOffsetCheck;
				srcOpenVTablePtrEntry = i;
				i++;
				srcEntryPtr = &srcEntries[i];

				addVTablePtrShiftIteration = 1;  // additional iteration might be needed to process vtable pointer at the end of a class

				PX_ASSERT((i < nbSrcEntries) && ((srcEntryPtr->entry.mFlags & PxMetaDataFlag::ePADDING) == 0));
				// if the second check fails, this whole section might have to be done before the padding bytes get processed. Not sure
				// what other consequences that might have though.
			}
		}
#if PX_CHECKED
		else
		{
			if(!compareEntries(*srcEntryPtr, *dstEntryPtr))
			{
				displayMessage(PxErrorCode::eINVALID_PARAMETER, "\rConvX::convertClass: %s, src meta data and dst meta data don't match!", mc->mClassName);
			    return false;
			}			
		}
#endif

		const ExtraDataEntry2& srcEntry = *srcEntryPtr;
		const ExtraDataEntry2& dstEntry = *dstEntryPtr;

		if(srcEntry.entry.mFlags & PxMetaDataFlag::eUNION)
		{
			// ### hardcoded bit here, will only work when union type is the first int of the struct
			const int* tmp = reinterpret_cast<const int*>(buffer + modSrcOffsetCheck);
			const int unionType = *tmp;

			const char* typeName = getTypeName(srcEntry.entry.mType, unionType);
			assert(typeName);

			MetaClass* unionMC = getMetaClass(typeName, META_DATA_SRC);
			assert(unionMC);

			convertClass(buffer + modSrcOffsetCheck, unionMC, 0);			// ### recurse

			dstOffsetCheck += dstEntry.entry.mSize;

			MetaClass* targetUnionMC = getMetaClass(typeName, META_DATA_DST);
			assert(targetUnionMC);

			const int delta = dstEntry.entry.mSize - targetUnionMC->mSize;
			char* deltaBytes = reinterpret_cast<char*>(PX_ALLOC(sizeof(char)*delta, "deltaBytes"));
			memset(deltaBytes, 0, size_t(delta));
			output(deltaBytes, delta);	// Skip unused bytes at the end of the union
			PX_FREE(deltaBytes);
			srcOffsetCheck += srcEntry.entry.mSize;  // do not use modSrcOffsetCheck here!
		}
		else
		{
			assert(srcEntry.cb);
			assert(srcEntry.offset == modSrcOffsetCheck);

			// ---- big convex surgery ----
			if(convexSurgery)
			{
				if(strcmp(srcEntry.entry.mName, "mNbHullVertices")==0)
				{
					assert(srcEntry.entry.mSize==1);
					const PxU8 nbVerts = static_cast<PxU8>(*(buffer+modSrcOffsetCheck));
					assert(!foundNbVerts);
					foundNbVerts = true;

					const PxU8 gaussMapLimit = static_cast<PxU8>(getBinaryMetaData(META_DATA_DST)->getGaussMapLimit());
					if(nbVerts > gaussMapLimit)
					{
						// We need a gauss map and we have one => keep it
					}
					else
					{
						// We don't need a gauss map and we have one => remove it
						removeBigData = true;
					}
				}
				else
				{
					if(removeBigData)
					{
						const bool isBigConvexData = strcmp(srcEntry.entry.mType, "BigConvexData")==0 || 
							                         strcmp(srcEntry.entry.mType, "BigConvexRawData")==0;
						if(isBigConvexData)
						{
							assert(foundNbVerts);
							setNullPtr(true);
						}
					}
				}
			}
			// ---- big convex surgery ----

			(this->*srcEntry.cb)(buffer+modSrcOffsetCheck, srcEntry.entry, dstEntry.entry);
			assert(dstOffsetCheck==dstEntry.offset);
			dstOffsetCheck += dstEntry.entry.mSize;
			srcOffsetCheck += srcEntry.entry.mSize;  // do not use modSrcOffsetCheck here!

			// ---- big convex surgery ----
			if(convexSurgery && removeBigData)
				setNullPtr(false);
			// ---- big convex surgery ----
		}

		j++;
	}

	displayMessage(PxErrorCode::eDEBUG_INFO, "---------------------------------------------\n");

	while(j<nbDstEntries)
	{
		assert(dstEntries[j].entry.mFlags & PxMetaDataFlag::ePADDING);
		if(dstEntries[j].entry.mFlags & PxMetaDataFlag::ePADDING)
		{
			dstOffsetCheck += dstEntries[j].entry.mSize;
		}
		j++;
	}

	assert(j==nbDstEntries);
	assert(dstOffsetCheck==target_mc->mSize);
	assert(srcOffsetCheck==mc->mSize);

	// ---- big convex surgery ----
	if(convexSurgery)
		mConvexFlags.pushBack(removeBigData);
	// ---- big convex surgery ----

	return true;
}

// Handles data defined with PX_DEF_BIN_METADATA_EXTRA_ARRAY
const char* Sn::ConvX::convertExtraData_Array(const char* Address, const char* lastAddress, const char* objectAddress,
	                                          const ExtraDataEntry& ed)
{
	(void)lastAddress;
	MetaClass* mc = getMetaClass(ed.entry.mType, META_DATA_SRC);
	assert(mc);

	// PT: safe to cast to int here since we're reading a count.
	const int count = int(peek(ed.entry.mSize, objectAddress + ed.offset, ed.entry.mFlags));

	//	if(ed.entry.mCount)	// Reused as align value
	if(ed.entry.mAlignment)
	{
		Address = alignStream(Address, ed.entry.mAlignment);
		//		Address = alignStream(Address, ed.entry.mCount);
		assert(Address<=lastAddress);
	}

	for(int c=0;c<count;c++)
	{
		convertClass(Address, mc, 0);
		Address += mc->mSize;
		assert(Address<=lastAddress);
	}
	return Address;
}

const char* Sn::ConvX::convertExtraData_Ptr(const char* Address, const char* lastAddress, const PxMetaDataEntry& entry, int count,
	                                        int ptrSize_Src, int ptrSize_Dst)
{
	(void)lastAddress;

	PxMetaDataEntry tmpSrc = entry;
	tmpSrc.mCount = count;
	tmpSrc.mSize = count * ptrSize_Src;

	PxMetaDataEntry tmpDst = entry;
	tmpDst.mCount = count;
	tmpDst.mSize = count * ptrSize_Dst;


	displayMessage(PxErrorCode::eDEBUG_INFO, "extra data ptrs\n");
	displayMessage(PxErrorCode::eDEBUG_INFO, "+++++++++++++++++++++++++++++++++++++++++++++\n");
	displayMessage(PxErrorCode::eDEBUG_INFO, "\t0x%p\t%02x\t\t\t%s", Address, static_cast<unsigned char>(Address[0]), entry.mName);
	for (int byteCount = 1; byteCount < ptrSize_Src*count; ++byteCount) 
		displayMessage(PxErrorCode::eDEBUG_INFO, "\t0x%p\t%02x\t\t\t.", Address + byteCount, static_cast<unsigned char>(Address[byteCount]));

	convertPtr(Address, tmpSrc, tmpDst);
	Address += count * ptrSize_Src;
	assert(Address<=lastAddress);
	return Address;
}

static bool decodeControl(PxU64 control, const ExtraDataEntry& ed, PxU64 controlMask = 0)
{
	if(ed.entry.mFlags & PxMetaDataFlag::eCONTROL_FLIP)
	{
		if(controlMask)
		{
			return (control & controlMask) ? false : true;
		}
		else
		{
			return control==0;
		}
	}
	else
	{
		if(controlMask)
		{
			return (control & controlMask) ? true : false;
		}
		else
		{
			return control!=0;
		}
	}

}

// ### currently hardcoded, should change
int Sn::ConvX::getConcreteType(const char* buffer)
{
	MetaClass* mc = getMetaClass("PxBase", META_DATA_SRC);
	assert(mc);
	PxMetaDataEntry entry;
	if(mc->getFieldByType("PxType", entry))
	{
		// PT: safe to cast to int here since we're reading our own PxType
		return int(peek(entry.mSize, buffer + entry.mOffset));
	}
	assert(0);
	return 0xffffffff;
}

struct Item : public shdfnd::UserAllocated
{
	MetaClass*	mc;
	const char*	address;
};

bool Sn::ConvX::convertCollection(const void* buffer, int fileSize, int nbObjects)
{
	const char* lastAddress = reinterpret_cast<const char*>(buffer) + fileSize;
	const char* Address = alignStream(reinterpret_cast<const char*>(buffer));

	const int ptrSize_Src = mSrcPtrSize;
	const int ptrSize_Dst = mDstPtrSize;
	Item* objects = PX_NEW(Item)[PxU32(nbObjects)];

	for(PxU32 i=0;i<PxU32(nbObjects);i++)
	{
		const float percents = float(i)/float(nbObjects);

		displayMessage(PxErrorCode::eDEBUG_INFO, "Object conversion: %d%%", int(percents*100.0f));

		Address = alignStream(Address);
		assert(Address<=lastAddress);

		PxConcreteType::Enum classType = PxConcreteType::Enum(getConcreteType(Address));
		MetaClass* metaClass = getMetaClass(classType, META_DATA_SRC);
		if(!metaClass)
		{
			PX_DELETE_ARRAY(objects);
			return false;
		}

		objects[i].mc = metaClass;
		objects[i].address = Address;

		if(!convertClass(Address, metaClass, 0))
        {
			PX_DELETE_ARRAY(objects);
			return false;
		}

		Address += metaClass->mSize;
		assert(Address<=lastAddress);
	}

	// Fields / extra data
	if(1)
	{
		// ---- big convex surgery ----
		unsigned int nbConvexes = 0;
		// ---- big convex surgery ----
		//const char* StartAddress2 = Address;
		//int startDstSize2 = getCurrentOutputSize();
		for(int i=0;i<nbObjects;i++)
		{
			//const char* StartAddress = Address;
			//int startDstSize = getCurrentOutputSize();

			const float percents = float(i)/float(nbObjects);

			displayMessage(PxErrorCode::eDEBUG_INFO, "Extra data conversion: %d%%", int(percents*100.0f));

			MetaClass* mc0 = objects[i].mc;
			const char* objectAddress = objects[i].address;

			//			printf("%d: %s\n", i, mc->mClassName);
			//			if(strcmp(mc->mClassName, "TriangleMesh")==0)
			//			if(strcmp(mc->mClassName, "NpRigidDynamic")==0)
			if(strcmp(mc0->mClassName, "HybridModel")==0)
			{
				int stop=1;
				(void)(stop);
			}

			// ### we actually need to collect all extra data for this class, including data from embedded members.

			PX_ALLOCA(entries, ExtraDataEntry, 256); 
			int nbEntries = 0;
			_enumerateExtraData(objectAddress, mc0, entries, nbEntries, 0, META_DATA_SRC);
			assert(nbEntries<256);

			Address = alignStream(Address);
			assert(Address<=lastAddress);

			for(int j=0;j<nbEntries;j++)
			{
				const ExtraDataEntry& ed = entries[j];
				assert(ed.entry.mFlags & PxMetaDataFlag::eEXTRA_DATA);

				if(ed.entry.mFlags & PxMetaDataFlag::eEXTRA_ITEM)
				{
					// ---- big convex surgery ----
					if(1)
					{
						const bool isBigConvexData = strcmp(ed.entry.mType, "BigConvexData")==0;
						if(isBigConvexData)
						{
							assert(nbConvexes<mConvexFlags.size());
							if(mConvexFlags[nbConvexes++])
								setNoOutput(true);
						}
					}
					// ---- big convex surgery ----

					MetaClass* extraDataType = getMetaClass(ed.entry.mType, META_DATA_SRC);
					assert(extraDataType);

					//sschirm: we used to have ed.entry.mOffset here, but that made cloth deserialization fail. - sschirm: cloth is gone now...
					const char* controlAddress = objectAddress + ed.offset; 
					const PxU64 controlValue = peek(ed.entry.mOffsetSize, controlAddress);

					if(controlValue)
					{
						if(ed.entry.mAlignment)
						{
							Address = alignStream(Address, ed.entry.mAlignment);
							assert(Address<=lastAddress);
						}

						const char* classAddress = Address;
						convertClass(Address, extraDataType, 0);
						Address += extraDataType->mSize;
						assert(Address<=lastAddress);

						// Enumerate extra data for this optional class, and convert it too.
						// This assumes the extra data for the optional class is always appended to the class itself,
						// which is something we'll need to enforce in the SDK. So far this is only to handle optional
						// inline arrays. 

						// ### this should probably be recursive eventually						
						PX_ALLOCA(entries2, ExtraDataEntry, 256); 
						int nbEntries2 = 0;
						_enumerateExtraData(objectAddress, extraDataType, entries2, nbEntries2, 0, META_DATA_SRC);
						assert(nbEntries2<256);
						for(int k=0;k<nbEntries2;k++)
						{
							const ExtraDataEntry& ed2 = entries2[k];
							assert(ed2.entry.mFlags & PxMetaDataFlag::eEXTRA_DATA);
							if(ed2.entry.mFlags & PxMetaDataFlag::eEXTRA_ITEMS)
							{
								const int controlOffset = ed2.entry.mOffset;
								const int controlSize = ed2.entry.mSize;
								const int countOffset = ed2.entry.mCount;
								const int countSize = ed2.entry.mOffsetSize;

								const PxU64 controlValue2 = peek(controlSize, classAddress + controlOffset);

								PxU64 controlMask = 0;
								if(ed2.entry.mFlags & PxMetaDataFlag::eCONTROL_MASK)
								{
									controlMask = PxU64(ed2.entry.mFlags & (PxMetaDataFlag::eCONTROL_MASK_RANGE << 16));
									controlMask = controlMask >> 16;
								}

								if(decodeControl(controlValue2, ed2, controlMask))
								{
									// PT: safe to cast to int here since we're reading a count
									int count = int(peek(countSize, classAddress + countOffset, ed2.entry.mFlags));

									if(ed2.entry.mAlignment)
									{
										assert(0);	// Never tested
										Address = alignStream(Address, ed2.entry.mAlignment);
										assert(Address<=lastAddress);
									}

									if(ed2.entry.mFlags & PxMetaDataFlag::ePTR)
									{
										assert(0);	// Never tested
									}
									else
									{
										MetaClass* mc = getMetaClass(ed2.entry.mType, META_DATA_SRC);
										assert(mc);

										while(count--)
										{
											convertClass(Address, mc, 0);
											Address += mc->mSize;
											assert(Address<=lastAddress);
										}
									}

								}
							}
							else
							{
								if( (ed2.entry.mFlags & PxMetaDataFlag::eALIGNMENT) && ed2.entry.mAlignment)
								{
									Address = alignStream(Address, ed2.entry.mAlignment);
									assert(Address<=lastAddress);
								}
								else
								{
									// We assume it's an normal array, e.g. the ones from "big convexes"
									assert(!(ed2.entry.mFlags & PxMetaDataFlag::eEXTRA_ITEM));

									Address = convertExtraData_Array(Address, lastAddress, classAddress, ed2);
								}
							}
						}

					}
					else					
					{
						int stop = 0;
						(void)(stop);
					}

					// ---- big convex surgery ----
					setNoOutput(false);
					// ---- big convex surgery ----
				}
				else if(ed.entry.mFlags & PxMetaDataFlag::eEXTRA_ITEMS)
				{
					// PX_DEF_BIN_METADATA_EXTRA_ITEMS
					int reloc = ed.offset - ed.entry.mOffset;	// ### because the enum code only fixed the "controlOffset"!
					const int controlOffset = ed.entry.mOffset;
					const int controlSize = ed.entry.mSize;
					const int countOffset = ed.entry.mCount;
					const int countSize = ed.entry.mOffsetSize;

					//					const int controlValue2 = peek(controlSize, objectAddress + controlOffset);
					const PxU64 controlValue2 = peek(controlSize, objectAddress + controlOffset + reloc);

					PxU64 controlMask = 0;
					if(ed.entry.mFlags & PxMetaDataFlag::eCONTROL_MASK)
					{
						controlMask = PxU64(ed.entry.mFlags & (PxMetaDataFlag::eCONTROL_MASK_RANGE << 16));
						controlMask = controlMask >> 16;
					}

					if(decodeControl(controlValue2, ed, controlMask))
					{
						// PT: safe to cast to int here since we're reading a count
						//						int count = peek(countSize, objectAddress + countOffset);	// ###
						int count = int(peek(countSize, objectAddress + countOffset + reloc, ed.entry.mFlags));	// ###

						if(ed.entry.mAlignment)
						{
							Address = alignStream(Address, ed.entry.mAlignment);
							assert(Address<=lastAddress);
						}

						if(ed.entry.mFlags & PxMetaDataFlag::ePTR)
						{
							Address = convertExtraData_Ptr(Address, lastAddress, ed.entry, count, ptrSize_Src, ptrSize_Dst);
						}
						else
						{
							MetaClass* mc = getMetaClass(ed.entry.mType, META_DATA_SRC);
							assert(mc);

							while(count--)
							{
								convertClass(Address, mc, 0);
								Address += mc->mSize;
								assert(Address<=lastAddress);
							}
						}
					}

				}
				else if(ed.entry.mFlags & PxMetaDataFlag::eALIGNMENT)
				{
					if(ed.entry.mAlignment)
					{
						displayMessage(PxErrorCode::eDEBUG_INFO, " align to %d bytes\n", ed.entry.mAlignment);
						displayMessage(PxErrorCode::eDEBUG_INFO, "---------------------------------------------\n");

						Address = alignStream(Address, ed.entry.mAlignment);
						assert(Address<=lastAddress);
					}
				}
				else if(ed.entry.mFlags & PxMetaDataFlag::eEXTRA_NAME)
				{
					if(ed.entry.mAlignment)
					{
						Address = alignStream(Address, ed.entry.mAlignment);
						assert(Address<=lastAddress);
					}

					//get string count
					MetaClass* mc = getMetaClass("PxU32", META_DATA_SRC);
					assert(mc);
					//safe to cast to int here since we're reading a count.
					const int count = int(peek(mc->mSize, Address, 0));

					displayMessage(PxErrorCode::eDEBUG_INFO, " convert  %d bytes string\n", count);

					convertClass(Address, mc, 0);
					Address += mc->mSize;

					mc = getMetaClass(ed.entry.mType, META_DATA_SRC);
					assert(mc);

					for(int c=0;c<count;c++)
					{
						convertClass(Address, mc, 0);
						Address += mc->mSize;
						assert(Address<=lastAddress);
					}
				}
				else
				{
					Address = convertExtraData_Array(Address, lastAddress, objectAddress, ed);
				}
			}
		}
		PX_DELETE_ARRAY(objects);
		assert(nbConvexes==mConvexFlags.size());
	}

	assert(Address==lastAddress);

	return true;
}

bool Sn::ConvX::convert(const void* buffer, int fileSize)
{
	// Test initial alignment
	if(size_t(buffer) & (ALIGN_DEFAULT-1))
	{
		assert(0);
		return false;
	}

	const int header = read32(buffer);						fileSize -= 4;	(void)header;

	if (header != PX_MAKE_FOURCC('S','E','B','D'))
	{
		displayMessage(physx::PxErrorCode::eINVALID_PARAMETER, 
			"PxBinaryConverter: Buffer contains data with bad header indicating invalid serialized data.");
		return false;
	}

	const int version = read32(buffer);						fileSize -= 4;	(void)version;

	const int binaryVersion = read32(buffer);				fileSize -= 4;

	if (!checkCompatibility(PxU32(version), PxU32(binaryVersion)))
	{
		char buf[512];
		getCompatibilityVersionsStr(buf, 512);

		displayMessage(physx::PxErrorCode::eINVALID_PARAMETER,
			"PxBinaryConverter: Buffer contains data version (%x-%d) is incompatible with this PhysX sdk.\n These versions would be compatible: %s",
			version, binaryVersion, buf);
		return false;
	}
	const int buildNumber = read32(buffer);						fileSize -= 4;	(void)buildNumber;

	//read src platform tag and write dst platform tag according dst meta data
	const int srcPlatformTag = *reinterpret_cast<const int*>(buffer);
	buffer = reinterpret_cast<const void*>(size_t(buffer) + 4);
	fileSize -= 4;
	const int dstPlatformTag = mMetaData_Dst->getPlatformTag();
	output(dstPlatformTag);

	if (srcPlatformTag != mMetaData_Src->getPlatformTag())
	{
		displayMessage(physx::PxErrorCode::eINVALID_PARAMETER,
			"PxBinaryConverter: Mismatch of platform tags of binary data and metadata:\n Binary Data: %s\n MetaData: %s\n",
			getBinaryPlatformName(PxU32(srcPlatformTag)),
			getBinaryPlatformName(PxU32(mMetaData_Src->getPlatformTag())));
		return false;
	}

	//read whether input data has marked padding, and set it for the output data (since 0xcd is written into pads on conversion)
	const int srcMarkedPadding = *reinterpret_cast<const int*>(buffer);
	buffer = reinterpret_cast<const void*>(size_t(buffer) + 4);
	fileSize -= 4; 
	mMarkedPadding = srcMarkedPadding != 0;
	const int dstMarkedPadding = 1;
	output(dstMarkedPadding);

	int nbObjectsInCollection;

	buffer = convertReferenceTables(buffer, fileSize, nbObjectsInCollection);
	if(!buffer)
		return false;

	bool ret = convertCollection(buffer, fileSize, nbObjectsInCollection);
	mMarkedPadding = false;
	return ret;
}

// PT: code below added to support 64bit-to-32bit conversions
void Sn::ConvX::exportIntAsPtr(int value)
{
	const int ptrSize_Src = mSrcPtrSize;
	const int ptrSize_Dst = mDstPtrSize;

	PxMetaDataEntry entry;

	const char* address = NULL;
	const PxU32 value32 = PxU32(value);
	const PxU64 value64 = PxU64(value)&0xffffffff;

	if(ptrSize_Src==4)
	{
		address = reinterpret_cast<const char*>(&value32);
	}
	else if(ptrSize_Src==8)
	{
		address = reinterpret_cast<const char*>(&value64);
	}
	else assert(0);

	convertExtraData_Ptr(address, address + ptrSize_Src, entry, 1, ptrSize_Src, ptrSize_Dst);
}

void Sn::ConvX::exportInt(int value)
{
	output(value);
}

void Sn::ConvX::exportInt64(PxU64 value)
{
	output(value);
}

PointerRemap::PointerRemap()
{
}

PointerRemap::~PointerRemap()
{
}

bool PointerRemap::checkRefIsNotUsed(PxU32 ref) const
{
	const PxU32 size = mData.size();
	for(PxU32 i=0;i<size;i++)
	{
		if(mData[i].id==ref)
			return false;
	}
	return true;
}

void PointerRemap::setObjectRef(PxU64 object64, PxU32 ref)
{
	const PxU32 size = mData.size();
	for(PxU32 i=0;i<size;i++)
	{
		if(mData[i].object==object64)
		{
			mData[i].id = ref;
			return;
		}
	}
	InternalData data;
	data.object	= object64;
	data.id		= ref;
	mData.pushBack(data);
}

bool PointerRemap::getObjectRef(PxU64 object64, PxU32& ref) const
{	
	const PxU32 size = mData.size();
	for(PxU32 i=0;i<size;i++)
	{
		if(mData[i].object==object64)
		{
			ref = mData[i].id;
			return true;
		}
	}
	return false;
}

/**
Converting the PxBase object offsets in the manifest table is fairly complicated now. 
It would be good to have an easy callback mechanism for custom things like this.
*/
const void* Sn::ConvX::convertManifestTable(const void* buffer, int& fileSize)
{
	PxU32 padding = getPadding(size_t(buffer), ALIGN_DEFAULT);
	buffer = alignStream(reinterpret_cast<const char*>(buffer));
	fileSize -= padding;
	int nb = read32(buffer);
	fileSize -= 4;

	MetaClass* mc_src = getMetaClass("Sn::ManifestEntry", META_DATA_SRC);
	assert(mc_src);

	MetaClass* mc_dst = getMetaClass("Sn::ManifestEntry", META_DATA_DST);
	assert(mc_dst);

	bool mdOk;
	PxMetaDataEntry srcTypeField;
	mdOk = mc_src->getFieldByName("type", srcTypeField);
	PX_UNUSED(mdOk);
	PX_ASSERT(mdOk);

	PxMetaDataEntry dstOffsetField;
	mdOk = mc_dst->getFieldByName("offset", dstOffsetField);
	PX_ASSERT(mdOk);

	const char* address = reinterpret_cast<const char*>(buffer);
	PxU32 headerOffset = 0;
	for(int i=0;i<nb;i++)
	{
		PxConcreteType::Enum classType = PxConcreteType::Enum(peek(srcTypeField.mSize, address + srcTypeField.mOffset));

		//convert ManifestEntry but output to tmpStream
		PxDefaultMemoryOutputStream tmpStream;
		{
			//backup output state
			PxOutputStream* outStream = mOutStream;
			PxU32 outputSize = PxU32(mOutputSize);

			mOutStream = &tmpStream;
			mOutputSize = 0;

			convertClass(address, mc_src, 0);
			PX_ASSERT(tmpStream.getSize() == PxU32(mc_dst->mSize));

			//restore output state
			mOutStream = outStream;
			mOutputSize = int(outputSize);
		}

		//output patched offset
		PX_ASSERT(dstOffsetField.mOffset == 0); //assuming offset is the first data
		output(int(headerOffset));

		//output rest of ManifestEntry
		PxU32 restSize = PxU32(mc_dst->mSize - dstOffsetField.mSize);
		mOutStream->write(tmpStream.getData() + dstOffsetField.mSize, restSize);
		mOutputSize += restSize;

		//increment source stream
		address += mc_src->mSize;
		fileSize -= mc_src->mSize;
		assert(fileSize>=0);

		//update headerOffset using the type and dst meta data of the type
		MetaClass* mc_classType_dst = getMetaClass(classType, META_DATA_DST);
		if(!mc_classType_dst)
			return NULL;
		headerOffset += getPadding(size_t(mc_classType_dst->mSize), PX_SERIAL_ALIGN) + mc_classType_dst->mSize;	
	}

	output(int(headerOffset)); //endoffset
	buffer = address + 4;
	fileSize -= 4;
	return buffer;
}

const void* Sn::ConvX::convertImportReferences(const void* buffer, int& fileSize)
{
	PxU32 padding = getPadding(size_t(buffer), ALIGN_DEFAULT);
	buffer = alignStream(reinterpret_cast<const char*>(buffer));
	fileSize -= padding;
	int nb = read32(buffer);
	fileSize -= 4;

	if(!nb)
		return buffer;

	MetaClass* mc = getMetaClass("Sn::ImportReference", META_DATA_SRC);
	assert(mc);

	const char* address = reinterpret_cast<const char*>(buffer);
	for(int i=0;i<nb;i++)
	{
		convertClass(address, mc, 0);
		address += mc->mSize;
		fileSize -= mc->mSize; 
		assert(fileSize>=0);
	}
	return address;
}

const void* Sn::ConvX::convertExportReferences(const void* buffer, int& fileSize)
{
	PxU32 padding = getPadding(size_t(buffer), ALIGN_DEFAULT);
	buffer = alignStream(reinterpret_cast<const char*>(buffer));
	fileSize -= padding;
	int nb = read32(buffer);
	fileSize -= 4;

	if(!nb)
		return buffer;

	MetaClass* mc = getMetaClass("Sn::ExportReference", META_DATA_SRC);
	assert(mc);

	const char* address = reinterpret_cast<const char*>(buffer);
	for(int i=0;i<nb;i++)
	{
		convertClass(address, mc, 0);
		address += mc->mSize;
		fileSize -= mc->mSize; 
		assert(fileSize>=0);
	}
	return address;
}

const void* Sn::ConvX::convertInternalReferences(const void* buffer, int& fileSize)
{
	PxU32 padding = getPadding(size_t(buffer), ALIGN_DEFAULT);
	buffer = alignStream(reinterpret_cast<const char*>(buffer));
	fileSize -= padding;

	//pointer references
	int nbPtrReferences = read32(buffer);
	fileSize -= 4;
	if(nbPtrReferences)
	{
		const char* address = reinterpret_cast<const char*>(buffer);
		MetaClass* mc = getMetaClass("Sn::InternalReferencePtr", META_DATA_SRC);
		assert(mc);
		for(int i=0;i<nbPtrReferences;i++)
		{
			convertClass(address, mc, 0);
			address += mc->mSize;
			fileSize -= mc->mSize; 
			assert(fileSize>=0);
		}
		buffer = address;
	}

	//index references
	int nbIdxReferences = read32(buffer);
	fileSize -= 4;
	if (nbIdxReferences)
	{
		const char* address = reinterpret_cast<const char*>(buffer);
		MetaClass* mc = getMetaClass("Sn::InternalReferenceIdx", META_DATA_SRC);
		assert(mc);
		for(int i=0;i<nbIdxReferences;i++)
		{
			convertClass(address, mc, 0);
			address += mc->mSize;
			fileSize -= mc->mSize; 
			assert(fileSize>=0);
		}
		buffer = address;
	}
	return buffer;
}


const void* Sn::ConvX::convertReferenceTables(const void* buffer, int& fileSize, int& nbObjectsInCollection)
{	
	// PT: the map should not be used while creating it, so use one indirection
	mActiveRemap = NULL;
	mRemap.mData.clear();
	mPointerRemapCounter = 0;

	PxU32 padding = getPadding(size_t(buffer), ALIGN_DEFAULT);
	buffer = alignStream(reinterpret_cast<const char*>(buffer));
	fileSize -= padding;

	nbObjectsInCollection = read32(buffer);
	if (nbObjectsInCollection == 0)
		displayMessage(PxErrorCode::eDEBUG_INFO, "\n\nConverting empty collection!\n\n");
	fileSize -= 4;

	buffer = convertManifestTable(buffer, fileSize);

	if(!buffer)
		return NULL;

	buffer = convertImportReferences(buffer, fileSize);
	buffer = convertExportReferences(buffer, fileSize);
	buffer = convertInternalReferences(buffer, fileSize);

	// PT: the map can now be used
	mActiveRemap = &mRemap;

	return buffer;
}

bool Sn::ConvX::checkPaddingBytes(const char* buffer, int byteCount)
{
	const unsigned char* src = reinterpret_cast<const unsigned char*>(buffer);

	int i = 0;
	while ((i < byteCount) && (src[i] == 0xcd)) 
		i++;
	return (i == byteCount);	
}
