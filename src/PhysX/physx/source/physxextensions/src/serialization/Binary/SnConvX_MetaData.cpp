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

#include "foundation/PxIO.h"
#include "foundation/PxMemory.h"
#include "SnConvX.h"
#include "common/PxSerialFramework.h"
#include "serialization/SnSerialUtils.h"
#include <assert.h>

using namespace physx;
using namespace physx::Sn;

//#define REMOVE_EXPLICIT_PADDING

static const char gVTablePtr[] = "v-table ptr";
static const char gAutoPadding[] = "auto-generated padding";
static const char gByte[] = "paddingByte";

///////////////////////////////////////////////////////////////////////////////

bool PxMetaDataEntry::isVTablePtr() const
{
	return mType==gVTablePtr;
}

///////////////////////////////////////////////////////////////////////////////

bool MetaClass::getFieldByType(const char* type, PxMetaDataEntry& entry) const
{
	assert(type);
	PxU32 nbFields = mFields.size();
	for(PxU32 i=0;i<nbFields;i++)
	{
		if(strcmp(mFields[i].mType, type)==0)
		{
			entry = mFields[i];
			return true;
		}
	}
	return false;
}

bool MetaClass::getFieldByName(const char* name, PxMetaDataEntry& entry) const
{
	assert(name);
	PxU32 nbFields = mFields.size();
	for(PxU32 i=0;i<nbFields;i++)
	{
		if(strcmp(mFields[i].mName, name)==0)
		{
			entry = mFields[i];
			return true;
		}
	}
	return false;
}

void MetaClass::checkAndCompleteClass(const MetaData& owner, int& startOffset, int& nbBytes)
{
	if(startOffset!=-1)
	{
		owner.mConvX.displayMessage(PxErrorCode::eDEBUG_INFO,
			"\n Adding %d padding bytes at offset %d in class %s.\n", nbBytes, startOffset, mClassName);

		// Leap of faith: add padding bytes there
		PxMetaDataEntry padding;
		padding.mType	= gByte;
		padding.mName	= gAutoPadding;
		padding.mOffset	= startOffset;
		padding.mSize	= nbBytes;
		padding.mCount	= nbBytes;
		padding.mFlags	= PxMetaDataFlag::ePADDING;
		mFields.pushBack(padding);

		startOffset = -1;
	}
}

bool MetaClass::check(const MetaData& owner)
{
	owner.mConvX.displayMessage(PxErrorCode::eDEBUG_INFO, "Checking class: %s\n", mClassName);

	if(mCallback)
		return true;	// Skip atomic types
	if(mMaster)
		return true;	// Skip typedefs

	bool* map = reinterpret_cast<bool*>(PX_ALLOC(sizeof(bool)*mSize, "bool"));
	memset(map, 0, size_t(mSize));

	const PxU32 nbFields = mFields.size();
	for(PxU32 i=0;i<nbFields;i++)
	{
		const PxMetaDataEntry& field = mFields[i];
		if(field.mFlags & PxMetaDataFlag::eEXTRA_DATA)
			continue;
//		if((field.mFlags & PxMetaDataFlag::eUNION) && !field.mSize)
//			continue;	// Union type
		assert(field.mSize);
		const int byteStart = field.mOffset;
		const int byteEnd = field.mOffset + field.mSize;
		assert(byteStart>=0 && byteStart<mSize);
		assert(byteEnd>=0 && byteEnd<=mSize);

		int startOffset = -1;
		int nbBytes = 0;
		for(int j=byteStart;j<byteEnd;j++)
		{
			if(map[j])
			{
				if(startOffset==-1)
				{
					startOffset = int(i);
					nbBytes = 0;
				}
				nbBytes++;
//				displayErrorMessage(" %s: found overlapping bytes!\n", mClassName);
			}
			else
			{
				if(startOffset!=-1)
				{
					owner.mConvX.displayMessage(PxErrorCode::eINTERNAL_ERROR, 
						"PxBinaryConverter: %s: %d overlapping bytes at offset %d!\n", mClassName, nbBytes, startOffset);
					startOffset = -1;
					PX_ALWAYS_ASSERT_MESSAGE("Overlapping bytes!");
				}
			}
			map[j] = true;
		}
		if(startOffset!=-1)
		{
			owner.mConvX.displayMessage(PxErrorCode::eINTERNAL_ERROR,
				"PxBinaryConverter: %s: %d overlapping bytes at offset %d!\n", mClassName, nbBytes, startOffset);
			startOffset = -1;
			PX_ALWAYS_ASSERT_MESSAGE("Overlapping bytes!");
		}
	}

	{
		int startOffset = -1;
		int nbBytes = 0;
		for(int i=0;i<mSize;i++)
		{
			if(!map[i])
			{
				if(startOffset==-1)
				{
					startOffset = i;
					nbBytes = 0;
				}
				nbBytes++;
			}
			else
			{
				checkAndCompleteClass(owner, startOffset, nbBytes);
			}
		}
		checkAndCompleteClass(owner, startOffset, nbBytes);
	}
	PX_FREE(map);


	//
	for(PxU32 i=0;i<nbFields;i++)
	{
		const PxMetaDataEntry& current = mFields[i];
		if(current.mFlags & PxMetaDataFlag::ePTR)
			continue;

		MetaClass* fieldMetaClass = owner.mConvX.getMetaClass(current.mType, owner.getType());
		if(!fieldMetaClass)
		{
			owner.mConvX.displayMessage(PxErrorCode::eINTERNAL_ERROR,
				"PxBinaryConverter: Missing meta-data for: %s\n", current.mType);
			return false;
		}
		else
		{
			if(current.mFlags & PxMetaDataFlag::eEXTRA_DATA)
			{       
				owner.mConvX.displayMessage(PxErrorCode::eDEBUG_INFO, "Extra data: %s\n", current.mType);
			}
			else
			{
				assert(fieldMetaClass->mSize*current.mCount==current.mSize);
			}
		}
	}
	return true;
}

///////////////////////////////////////////////////////////////////////////////

MetaData::MetaData(ConvX& convx) :
	mConvX					(convx),
	mType					(META_DATA_NONE),
	mNbEntries				(0),
	mEntries				(NULL),
	mStringTable			(NULL),
	mVersion				(0),
	mBuildNumber			(0),
	mSizeOfPtr				(0),
	mPlatformTag			(0),
	mGaussMapLimit			(0),
	mFlip					(false)
{
}

MetaData::~MetaData()
{
	PxU32 nbMetaClasses = mMetaClasses.size();
	for(PxU32 i=0;i<nbMetaClasses;i++)
	{
		MetaClass* current = mMetaClasses[i];
		PX_DELETE(current);
	}

	PX_FREE(mStringTable);
	PX_DELETE_ARRAY(mEntries);
}

MetaClass* MetaData::getMetaClass(const char* name)	const
{
	PxU32 nbMetaClasses = mMetaClasses.size();
	for(PxU32 i=0;i<nbMetaClasses;i++)
	{
		MetaClass* current = mMetaClasses[i];
		if(strcmp(current->mClassName, name)==0)
		{
			while(current->mMaster)
				current = current->mMaster;
			return current;
		}
	}
	return NULL;
}

MetaClass* MetaData::getMetaClass(PxConcreteType::Enum concreteType) const
{
	for(PxU32 i=0; i< mConcreteTypeTable.size(); i++)
	{
		if(mConcreteTypeTable[i].first == concreteType)
		{
			const char* className = offsetToText(reinterpret_cast<const char*>(size_t(mConcreteTypeTable[i].second)));
			return getMetaClass(className);
		}
	}
	return NULL;
}

MetaClass* MetaData::addNewClass(const char* name, int size, MetaClass* master, ConvertCallback callback)
{
	// PT: if you reach this assert, you used PX_DEF_BIN_METADATA_TYPEDEF twice on the same type
	assert(!getMetaClass(name));
	MetaClass* mc = PX_NEW(MetaClass);
	mc->mCallback	= callback;
	mc->mMaster		= master;
	mc->mClassName	= name;
	mc->mSize		= size;
	mc->mDepth		= 0;
	mc->mProcessed	= false;
//	mc->mNbEntries	= -1;

	mMetaClasses.pushBack(mc);

	return mc;
}

bool MetaData::load(PxInputStream& inputStream, MetaDataType type)
{
	assert(type!=META_DATA_NONE);

	mConvX.displayMessage(PxErrorCode::eDEBUG_INFO, "Loading %s meta-data...\n", type==META_DATA_SRC ? "source" : "target");

	mType = type;

	mFlip = false;
	{
		int header;
		inputStream.read(&header, 4);
		if(header==PX_MAKE_FOURCC('M','E','T','A'))
		{
			mFlip = false;
		}
		else if(header==PX_MAKE_FOURCC('A','T','E','M'))
		{
			mFlip = true;
		}
		else
		{
			mConvX.displayMessage(PxErrorCode::eINVALID_PARAMETER, "PxBinaryConverter: invalid meta-data file!\n");
			return false;
		}

		if (type == META_DATA_SRC && mFlip)
		{
			mConvX.displayMessage(PxErrorCode::eINVALID_PARAMETER,
				"PxBinaryConverter: source meta data needs to match endianness with current system!");
			return false;
		}

		inputStream.read(&mVersion, 4);
		inputStream.read(&mBinaryVersion, 4);
		if(mFlip)
		{
			flip(mVersion);
			flip(mBinaryVersion);
		}

		if (!checkCompatibility(PxU32(mVersion), PxU32(mBinaryVersion)))
		{
			char buffer[512];
		    getCompatibilityVersionsStr(buffer, 512);

			mConvX.displayMessage(PxErrorCode::eINVALID_PARAMETER,
				"PxBinaryConverter: data version (%x-%d) is incompatible with this PhysX sdk.\n These versions would be compatible: %s",
				mVersion, mBinaryVersion, buffer);
			
			return false;
		}
		inputStream.read(&mBuildNumber, 4);
		if(mFlip)
			flip(mBuildNumber);

		
		inputStream.read(&mSizeOfPtr, 4);
		if(mFlip)
			flip(mSizeOfPtr);

		inputStream.read(&mPlatformTag, 4);
		if(mFlip)
			flip(mPlatformTag);

		if (!Sn::isBinaryPlatformTagValid(PxU32(mPlatformTag)))
		{
			mConvX.displayMessage(PxErrorCode::eINVALID_PARAMETER, "PxBinaryConverter: Unknown meta data platform tag");
			return false;
		}

		inputStream.read(&mGaussMapLimit, 4);
		if(mFlip)
			flip(mGaussMapLimit);

		inputStream.read(&mNbEntries, 4);
		if(mFlip)
			flip(mNbEntries);

		mEntries = PX_NEW(PxMetaDataEntry)[PxU32(mNbEntries)];
		if(mSizeOfPtr==8)
		{
			for(int i=0;i<mNbEntries;i++)
			{
				MetaDataEntry64 tmp;
				inputStream.read(&tmp, sizeof(MetaDataEntry64));
				if (mFlip)  // important to flip them first, else the cast below might destroy information
				{
					flip(tmp.mType);
					flip(tmp.mName);
				}
				// We can safely cast to 32bits here since we transformed the pointers to offsets in the string table on export
				mEntries[i].mType		= reinterpret_cast<const char*>(size_t(tmp.mType));
				mEntries[i].mName		= reinterpret_cast<const char*>(size_t(tmp.mName));
				mEntries[i].mOffset		= tmp.mOffset;
				mEntries[i].mSize		= tmp.mSize;
				mEntries[i].mCount		= tmp.mCount;
				mEntries[i].mOffsetSize	= tmp.mOffsetSize;
				mEntries[i].mFlags		= tmp.mFlags;
				mEntries[i].mAlignment	= tmp.mAlignment;
			}
		}
		else
		{
			assert(mSizeOfPtr==4);
//			inputStream.read(mEntries, mNbEntries*sizeof(PxMetaDataEntry));
			for(int i=0;i<mNbEntries;i++)
			{
				MetaDataEntry32 tmp;
				inputStream.read(&tmp, sizeof(MetaDataEntry32));
				if (mFlip)
				{
					flip(tmp.mType);
					flip(tmp.mName);
				}
				mEntries[i].mType		= reinterpret_cast<const char*>(size_t(tmp.mType));
				mEntries[i].mName		= reinterpret_cast<const char*>(size_t(tmp.mName));
				mEntries[i].mOffset		= tmp.mOffset;
				mEntries[i].mSize		= tmp.mSize;
				mEntries[i].mCount		= tmp.mCount;
				mEntries[i].mOffsetSize	= tmp.mOffsetSize;
				mEntries[i].mFlags		= tmp.mFlags;
				mEntries[i].mAlignment	= tmp.mAlignment;
			}
		}

		if(mFlip)
		{
			for(int i=0;i<mNbEntries;i++)
			{
				// mEntries[i].mType and mEntries[i].mName have been flipped already because they need special treatment
				// on 64bit to 32bit platform conversions
				flip(mEntries[i].mOffset);
				flip(mEntries[i].mSize);
				flip(mEntries[i].mCount);
				flip(mEntries[i].mOffsetSize);
				flip(mEntries[i].mFlags);
				flip(mEntries[i].mAlignment);
			}
		}

		int nbConcreteType;
		inputStream.read(&nbConcreteType, 4);
		if(mFlip)
			flip(nbConcreteType);

		for(int i=0; i<nbConcreteType; i++)
		{
			PxU16 concreteType;
			PxU32 nameOffset;
			inputStream.read(&concreteType, 2);
			inputStream.read(&nameOffset, 4);
			if(mFlip)
			{
			    flip(concreteType);
				flip(nameOffset);
			}
			
			mConcreteTypeTable.pushBack( Ps::Pair<PxConcreteType::Enum, PxU32>(PxConcreteType::Enum(concreteType), nameOffset) );
		}

		int tableSize;
		inputStream.read(&tableSize, 4);
		if(mFlip)
			flip(tableSize);

		mStringTable = reinterpret_cast<char*>(PX_ALLOC(sizeof(char)*tableSize, "MetaData StringTable"));
		inputStream.read(mStringTable, PxU32(tableSize));
	}

	// Register atomic types
	{
		addNewClass("bool",			1, NULL, &ConvX::convert8);
		addNewClass("char",			1, NULL, &ConvX::convert8);
		addNewClass("short",		2, NULL, &ConvX::convert16);
		addNewClass("int",			4, NULL, &ConvX::convert32);
		addNewClass("PxU64",		8, NULL, &ConvX::convert64);
		addNewClass("float",		4, NULL, &ConvX::convertFloat);

		addNewClass("paddingByte",	1, NULL, &ConvX::convertPad8);
	}

	{
		MetaClass* currentClass = NULL;
		for(int i=0;i<mNbEntries;i++)
		{
			mEntries[i].mType = offsetToText(mEntries[i].mType);
			mEntries[i].mName = offsetToText(mEntries[i].mName);

			if(mEntries[i].mFlags & PxMetaDataFlag::eTYPEDEF)
			{
				mConvX.displayMessage(PxErrorCode::eDEBUG_INFO, "Found typedef: %s => %s\n", mEntries[i].mName, mEntries[i].mType);
				MetaClass* mc = getMetaClass(mEntries[i].mName);
				if(mc)
					addNewClass(mEntries[i].mType, mc->mSize, mc, mc->mCallback);
				else
					mConvX.displayMessage(PxErrorCode::eINTERNAL_ERROR,
					  "PxBinaryConverter: Invalid typedef - Missing metadata for: %s, please check the source metadata.\n"
					  , mEntries[i].mName);
			}
			else if(mEntries[i].mFlags & PxMetaDataFlag::eCLASS)
			{
				if(!mEntries[i].mName)
				{
					mConvX.displayMessage(PxErrorCode::eDEBUG_INFO, "Found class: %s\n", mEntries[i].mType);
					currentClass = addNewClass(mEntries[i].mType, mEntries[i].mSize);

					if(mEntries[i].mFlags & PxMetaDataFlag::eVIRTUAL)
					{
						PxMetaDataEntry vtable;
						vtable.mType	= gVTablePtr;
						vtable.mName	= gVTablePtr;
						vtable.mOffset	= 0;
						vtable.mSize	= mSizeOfPtr;
						vtable.mCount	= 1;
						vtable.mFlags	= PxMetaDataFlag::ePTR;
						currentClass->mFields.pushBack(vtable);
					}
				}
				else
				{
					assert(currentClass);
					mConvX.displayMessage(PxErrorCode::eDEBUG_INFO, " - inherits from: %s\n", mEntries[i].mName);
					currentClass->mBaseClasses.pushBack(mEntries[i]);
				}
			}
			else
			{
				const int isUnion = mEntries[i].mFlags & PxMetaDataFlag::eUNION;

				if(isUnion && !mEntries[i].mSize)
				{
					mConvX.registerUnionType(mEntries[i].mType, mEntries[i].mName, mEntries[i].mOffset);
				}
				else
				{
					if(isUnion)
					{
						mConvX.registerUnion(mEntries[i].mType);
					}

					const int isPadding = mEntries[i].mFlags & PxMetaDataFlag::ePADDING;

					assert(currentClass);
#ifdef REMOVE_EXPLICIT_PADDING
					if(!isPadding)
#endif
						currentClass->mFields.pushBack(mEntries[i]);
					
					if(isPadding)
						mConvX.displayMessage(PxErrorCode::eDEBUG_INFO, 
						" - contains padding: %s - %s\n", mEntries[i].mType, mEntries[i].mName);
					else if(mEntries[i].mFlags & PxMetaDataFlag::eEXTRA_DATA)
						mConvX.displayMessage(PxErrorCode::eDEBUG_INFO,
						" - contains extra data: %s%s\n", mEntries[i].mType, mEntries[i].mFlags & PxMetaDataFlag::ePTR ? "*" : "");
					else
						mConvX.displayMessage(PxErrorCode::eDEBUG_INFO,
						" - contains field: %s%s\n", mEntries[i].mType, mEntries[i].mFlags & PxMetaDataFlag::ePTR ? "*" : "");
					
				}
			}
		}
	}

	// Sort classes by depth
	struct Local
	{
		static bool _computeDepth(const MetaData& md, MetaClass* current, int currentDepth, int& maxDepth)
		{
			if(currentDepth>maxDepth)
				maxDepth = currentDepth;

			PxU32 nbBases = current->mBaseClasses.size();
			for(PxU32 i=0;i<nbBases;i++)
			{
				const PxMetaDataEntry& baseClassEntry = current->mBaseClasses[i];
				MetaClass* baseClass = md.getMetaClass(baseClassEntry.mName);
				if(!baseClass)
				{
					md.mConvX.displayMessage(PxErrorCode::eINTERNAL_ERROR,
					"PxBinaryConverter: Can't find class %s metadata, please check the source metadata.\n", baseClassEntry.mName);	
					return false;
				}
				if (!_computeDepth(md, baseClass, currentDepth+1, maxDepth))
					return false;
			}
			return true;
		}

		static int compareClasses(const void* c0, const void* c1)
		{
			MetaClass** mc0 = reinterpret_cast<MetaClass**>(const_cast<void*>(c0));
			MetaClass** mc1 = reinterpret_cast<MetaClass**>(const_cast<void*>(c1));
//			return (*mc0)->mSize - (*mc1)->mSize;
			return (*mc0)->mDepth - (*mc1)->mDepth;
		}

		static int compareEntries(const void* c0, const void* c1)
		{
			PxMetaDataEntry* mc0 = reinterpret_cast<PxMetaDataEntry*>(const_cast<void*>(c0));
			PxMetaDataEntry* mc1 = reinterpret_cast<PxMetaDataEntry*>(const_cast<void*>(c1));
			//mOffset is used to access control information for extra data, and not for offsets of the data itself.
			assert(!(mc0->mFlags & PxMetaDataFlag::eEXTRA_DATA));
			assert(!(mc1->mFlags & PxMetaDataFlag::eEXTRA_DATA));
			return mc0->mOffset - mc1->mOffset;
		}
	};
	{
		// Compute depths
		const PxU32 nbMetaClasses = mMetaClasses.size();
		for(PxU32 i=0;i<nbMetaClasses;i++)
		{
			MetaClass* current = mMetaClasses[i];
			int maxDepth = 0;
			if(!Local::_computeDepth(*this, current, 0, maxDepth))
				return false;
			current->mDepth = maxDepth;
		}

		// Sort by depth
		MetaClass** metaClasses = &mMetaClasses[0];
		qsort(metaClasses, size_t(nbMetaClasses), sizeof(MetaClass*), Local::compareClasses);
	}

	// Replicate fields from base classes
	{
		PxU32 nbMetaClasses = mMetaClasses.size();
		for(PxU32 k=0;k<nbMetaClasses;k++)
		{
			MetaClass* current = mMetaClasses[k];
			PxU32 nbBases = current->mBaseClasses.size();

			// merge entries of base classes and current class in the right order
			// this is needed for extra data ordering, which is not covered by the mOffset sort 
			// in the next stage below
			PsArray<PxMetaDataEntry> mergedEntries;

			for(PxU32 i=0;i<nbBases;i++)
			{
				const PxMetaDataEntry& baseClassEntry = current->mBaseClasses[i];
				MetaClass* baseClass = getMetaClass(baseClassEntry.mName);
				assert(baseClass);
				assert(baseClass->mBaseClasses.size()==0 || baseClass->mProcessed);

				PxU32 nbBaseFields = baseClass->mFields.size();
				for(PxU32 j=0;j<nbBaseFields;j++)
				{
					PxMetaDataEntry f = baseClass->mFields[j];
					// Don't merge primary v-tables to avoid redundant v-table entries.
					// It means the base v-table won't be inherited & needs to be explicitly defined in the metadata. Seems reasonable.
					// Could be done better though.

					if(f.mType==gVTablePtr && !f.mOffset && !baseClassEntry.mOffset)
						continue;
					
					f.mOffset += baseClassEntry.mOffset;
					mergedEntries.pushBack(f);
				}
				current->mProcessed = true;
			}

			//append current fields to base class fields 
			for (PxU32 i = 0; i < current->mFields.size(); i++)
			{
				mergedEntries.pushBack(current->mFields[i]);
			}
			current->mFields.clear();
			current->mFields.assign(mergedEntries.begin(), mergedEntries.end());
		}
	}

	// Check classes
	{
		PxU32 nbMetaClasses = mMetaClasses.size();
		for(PxU32 i=0;i<nbMetaClasses;i++)
		{
			MetaClass* current = mMetaClasses[i];
			if(!current->check(*this))
				return false;
		}
	}

	// Sort meta-data by offset
	{
		PxU32 nbMetaClasses = mMetaClasses.size();
		for(PxU32 i=0;i<nbMetaClasses;i++)
		{
			MetaClass* current = mMetaClasses[i];
			PxU32 nbFields = current->mFields.size();
			if(nbFields<2)
				continue;
			PxMetaDataEntry* entries = &current->mFields[0];

			PxMetaDataEntry* newEntries = PX_NEW(PxMetaDataEntry)[nbFields];
			PxU32 nb = 0;
			for(PxU32 j=0;j<nbFields;j++)
				if(!(entries[j].mFlags & PxMetaDataFlag::eEXTRA_DATA))
					newEntries[nb++] = entries[j];
			PxU32 nbToSort = nb;
			for(PxU32 j=0;j<nbFields;j++)
				if(entries[j].mFlags & PxMetaDataFlag::eEXTRA_DATA)
					newEntries[nb++] = entries[j];
			assert(nb==nbFields);
			PxMemCopy(entries, newEntries, nb*sizeof(PxMetaDataEntry));
			PX_DELETE_ARRAY(newEntries);
			qsort(entries, size_t(nbToSort), sizeof(PxMetaDataEntry), Local::compareEntries);
		}
	}
	return true;
}

namespace
{
	//tool functions for MetaData::compare

	bool str_equal(const char* src, const char* dst) 
	{
		if (src == dst)
			return true;

		if (src != nullptr && dst != nullptr)
			return strcmp(src, dst) == 0;

		return false;
	}

	const char* str_print(const char* str)
	{
		return str != nullptr ? str : "(nullptr)";
	}
}

#define COMPARE_METADATA_BOOL_MD(type, src, dst, field) if ((src).field != (dst).field) \
	{ mConvX.displayMessage(PxErrorCode::eDEBUG_INFO, "%s::%s missmatch: src %s dst %s\n", #type, #field, (src).field?"true":"false", (dst).field?"true":"false"); isEquivalent = false; }
#define COMPARE_METADATA_INT_MD(type, src, dst, field) if ((src).field != (dst).field) \
	{ mConvX.displayMessage(PxErrorCode::eDEBUG_INFO, "%s::%s missmatch: src %d dst %d\n", #type, #field, (src).field, (dst).field); isEquivalent = false; }
#define COMPARE_METADATA_STRING_MD(type, src, dst, field) \
	if (!str_equal((src).field, (dst).field)) \
	{ \
		mConvX.displayMessage(PxErrorCode::eDEBUG_INFO, "%s::%s missmatch: src %s dst %s\n", #type, #field, str_print((src).field), str_print((dst).field)); \
		isEquivalent = false; \
	}

bool MetaData::compare(const MetaData& dst) const
{
	bool isEquivalent = true;

	//mType
	COMPARE_METADATA_BOOL_MD(MetaData, *this, dst, mFlip)
	COMPARE_METADATA_INT_MD(MetaData, *this, dst, mVersion)
	COMPARE_METADATA_INT_MD(MetaData, *this, dst, mBinaryVersion)
	//mBuildNumber
	COMPARE_METADATA_INT_MD(MetaData, *this, dst, mSizeOfPtr)
	COMPARE_METADATA_INT_MD(MetaData, *this, dst, mPlatformTag)
	COMPARE_METADATA_INT_MD(MetaData, *this, dst, mGaussMapLimit)
	COMPARE_METADATA_INT_MD(MetaData, *this, dst, mNbEntries)

	//find classes missing in dst
	for (PxU32 i = 0; i<mMetaClasses.size(); i++)
	{
		MetaClass* mcSrc = mMetaClasses[i];
		MetaClass* mcDst = dst.getMetaClass(mcSrc->mClassName);

		if (mcDst == nullptr)
		{
			mConvX.displayMessage(PxErrorCode::eDEBUG_INFO, "dst is missing meta class %s", mcSrc->mClassName);
		}
	}

	//find classes missing in src
	for (PxU32 i = 0; i<dst.mMetaClasses.size(); i++)
	{
		MetaClass* mcDst = dst.mMetaClasses[i];
		MetaClass* mcSrc = getMetaClass(mcDst->mClassName);

		if (mcSrc == nullptr)
		{
			mConvX.displayMessage(PxErrorCode::eDEBUG_INFO, "dst is missing meta class %s", mcSrc->mClassName);
		}
	}

	//compare classes present in src and dst
	for (PxU32 i = 0; i<mMetaClasses.size(); i++)
	{
		const char* className = mMetaClasses[i]->mClassName;
		MetaClass* mcSrc = getMetaClass(className);
		MetaClass* mcDst = dst.getMetaClass(className);
		if (mcSrc != nullptr && mcDst != nullptr)
		{
			COMPARE_METADATA_INT_MD(MetaClass, *mcSrc, *mcDst, mCallback)
			COMPARE_METADATA_INT_MD(MetaClass, *mcSrc, *mcDst, mMaster) //should be 0 for both anyway
			COMPARE_METADATA_STRING_MD(MetaClass, *mcSrc, *mcDst, mClassName)
			COMPARE_METADATA_INT_MD(MetaClass, *mcSrc, *mcDst, mSize)
			COMPARE_METADATA_INT_MD(MetaClass, *mcSrc, *mcDst, mDepth)

			COMPARE_METADATA_INT_MD(MetaClass, *mcSrc, *mcDst, mBaseClasses.size())
			if (mcSrc->mBaseClasses.size() == mcDst->mBaseClasses.size())
			{
				for (PxU32 b = 0; b < mcSrc->mBaseClasses.size(); b++)
				{
					COMPARE_METADATA_STRING_MD(PxMetaDataEntry, mcSrc->mBaseClasses[b], mcDst->mBaseClasses[b], mName);
				}
			}

			COMPARE_METADATA_INT_MD(MetaClass, *mcSrc, *mcDst, mFields.size())
			if (mcSrc->mFields.size() == mcDst->mFields.size())
			{
				for (PxU32 f = 0; f < mcSrc->mFields.size(); f++)
				{
					PxMetaDataEntry srcMde = mcSrc->mFields[f];
					PxMetaDataEntry dstMde = mcDst->mFields[f];

					COMPARE_METADATA_STRING_MD(PxMetaDataEntry, srcMde, dstMde, mType)
					COMPARE_METADATA_STRING_MD(PxMetaDataEntry, srcMde, dstMde, mName)
					COMPARE_METADATA_INT_MD(PxMetaDataEntry, srcMde, dstMde, mOffset)
					COMPARE_METADATA_INT_MD(PxMetaDataEntry, srcMde, dstMde, mSize)
					COMPARE_METADATA_INT_MD(PxMetaDataEntry, srcMde, dstMde, mCount)
					COMPARE_METADATA_INT_MD(PxMetaDataEntry, srcMde, dstMde, mOffsetSize)
					COMPARE_METADATA_INT_MD(PxMetaDataEntry, srcMde, dstMde, mFlags)
					COMPARE_METADATA_INT_MD(PxMetaDataEntry, srcMde, dstMde, mAlignment)
				}
			}
		}
	}
	return isEquivalent;
}

#undef COMPARE_METADATA_BOOL_MD
#undef COMPARE_METADATA_INT_MD
#undef COMPARE_METADATA_STRING_MD

///////////////////////////////////////////////////////////////////////////////

void ConvX::releaseMetaData()
{
	DELETESINGLE(mMetaData_Dst);
	DELETESINGLE(mMetaData_Src);
}

const MetaData* ConvX::loadMetaData(PxInputStream& inputStream, MetaDataType type)
{
	if (type != META_DATA_SRC && type != META_DATA_DST)
	{
		displayMessage(PxErrorCode::eINTERNAL_ERROR,
			           "PxBinaryConverter: Wrong meta data type, please check the source metadata.\n");
		return NULL;
	}

	PX_ASSERT(type == META_DATA_SRC || type == META_DATA_DST);

	MetaData*& metaDataPtr = (type == META_DATA_SRC) ? mMetaData_Src : mMetaData_Dst;
	metaDataPtr = PX_NEW(MetaData)(*this);
	if(!(metaDataPtr)->load(inputStream, type))
		DELETESINGLE(metaDataPtr);
	return metaDataPtr;
}

const MetaData* ConvX::getBinaryMetaData(MetaDataType type)
{
	if(type==META_DATA_SRC)
		return mMetaData_Src;
	if(type==META_DATA_DST)
		return mMetaData_Dst;
	PX_ASSERT(0);
	return NULL;
}

int ConvX::getNbMetaClasses(MetaDataType type)
{
	if(type==META_DATA_SRC)
		return mMetaData_Src->getNbMetaClasses();
	if(type==META_DATA_DST)
		return mMetaData_Dst->getNbMetaClasses();
	PX_ASSERT(0);
	return 0;
}

MetaClass* ConvX::getMetaClass(unsigned int i, MetaDataType type) const
{
	if(type==META_DATA_SRC)
		return mMetaData_Src->getMetaClass(i);
	if(type==META_DATA_DST)
		return mMetaData_Dst->getMetaClass(i);
	PX_ASSERT(0);
	return NULL;
}

MetaClass* ConvX::getMetaClass(const char* name, MetaDataType type) const
{
	if(type==META_DATA_SRC)
		return mMetaData_Src->getMetaClass(name);
	if(type==META_DATA_DST)
		return mMetaData_Dst->getMetaClass(name);
	PX_ASSERT(0);
	return NULL;
}

MetaClass* ConvX::getMetaClass(PxConcreteType::Enum concreteType, MetaDataType type)
{
	MetaClass* metaClass = NULL;
	if(type==META_DATA_SRC)
		metaClass = mMetaData_Src->getMetaClass(concreteType);
	if(type==META_DATA_DST)
		metaClass = mMetaData_Dst->getMetaClass(concreteType);

	if(!metaClass)
	{		
		displayMessage(PxErrorCode::eINTERNAL_ERROR,
			"PxBinaryConverter: Missing concreteType %d metadata! serialized a class without dumping metadata. Please check the metadata.",
			concreteType);
		return NULL;
	}	

	return metaClass;
}

///////////////////////////////////////////////////////////////////////////////

// Peek & poke, yes sir.
PxU64 physx::Sn::peek(int size, const char* buffer, int flags)
{
	const int maskMSB = flags & PxMetaDataFlag::eCOUNT_MASK_MSB;
	const int skipIfOne = flags & PxMetaDataFlag::eCOUNT_SKIP_IF_ONE;
	switch(size)
	{
		case 1:
		{
			unsigned char value = *(reinterpret_cast<const unsigned char*>(buffer));
			if(maskMSB)
				value &= 0x7f;
			if(skipIfOne && value==1)
				return 0;
			return PxU64(value);
		}
		case 2:
		{
			unsigned short value = *(reinterpret_cast<const unsigned short*>(buffer));
			if(maskMSB)
				value &= 0x7fff;
			if(skipIfOne && value==1)
				return 0;
			return PxU64(value);
		}
		case 4:
		{
			unsigned int value = *(reinterpret_cast<const unsigned int*>(buffer));
			if(maskMSB)
				value &= 0x7fffffff;
			if(skipIfOne && value==1)
				return 0;
			return PxU64(value);
		}
		case 8:
		{
			PxU64 value = *(reinterpret_cast<const PxU64*>(buffer));
			if(maskMSB)
				value &= (PxU64(-1))>>1;
			if(skipIfOne && value==1)
				return 0;
			return value;
		}
	};
	PX_ASSERT(0);
	return PxU64(-1);
}

