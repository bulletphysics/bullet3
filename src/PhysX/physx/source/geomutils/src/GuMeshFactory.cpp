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

#include "PsIntrinsics.h"
#include "GuMeshFactory.h"
#include "PxHeightFieldDesc.h"
#include "GuTriangleMesh.h"
#include "GuTriangleMeshBV4.h"
#include "GuTriangleMeshRTree.h"
#include "GuConvexMesh.h"
#include "GuBVHStructure.h"
#include "GuHeightField.h"
#include "GuConvexMeshData.h"
#include "CmUtils.h"
#include "GuMeshData.h"
#include "PsFoundation.h"

using namespace physx;
using namespace Gu;

// PT: TODO: refactor all this with a dedicated container

GuMeshFactory::GuMeshFactory() :
	mTriangleMeshes		(PX_DEBUG_EXP("mesh factory triangle mesh hash")),
	mConvexMeshes		(PX_DEBUG_EXP("mesh factory convex mesh hash")),
	mHeightFields		(PX_DEBUG_EXP("mesh factory height field hash")),
    mBVHStructures 		(PX_DEBUG_EXP("BVH structure factory hash")),
	mFactoryListeners	(PX_DEBUG_EXP("FactoryListeners"))
{
}

GuMeshFactory::~GuMeshFactory()
{
}

///////////////////////////////////////////////////////////////////////////////

template<class T>
static void releaseObjects(Ps::CoalescedHashSet<T*>& objects)
{
	while(objects.size())
	{
		T* object = objects.getEntries()[0];
		PX_ASSERT(object->getRefCount()==1);
		object->release();
	}
}

void GuMeshFactory::release()
{
	// Release all objects in case the user didn't do it
	releaseObjects(mTriangleMeshes);
	releaseObjects(mConvexMeshes);
	releaseObjects(mHeightFields);
	releaseObjects(mBVHStructures);

	PX_DELETE(this);
}

template <typename T>
static void addToHash(Ps::CoalescedHashSet<T*>& hash, T* element, Ps::Mutex* mutex)
{
	if(!element)
		return;

	if(mutex)
		mutex->lock();

	hash.insert(element);

	if(mutex)
		mutex->unlock();
}

///////////////////////////////////////////////////////////////////////////////

void GuMeshFactory::addTriangleMesh(TriangleMesh* np, bool lock)
{
	addToHash(mTriangleMeshes, np, lock ? &mTrackingMutex : NULL);
}

PxTriangleMesh* GuMeshFactory::createTriangleMesh(TriangleMeshData& data)
{	
	TriangleMesh* np;

	if(data.mType==PxMeshMidPhase::eBVH33)
	{
		PX_NEW_SERIALIZED(np, RTreeTriangleMesh)(*this, data);
	}
	else if(data.mType==PxMeshMidPhase::eBVH34)
	{
		PX_NEW_SERIALIZED(np, BV4TriangleMesh)(*this, data);
	}
	else return NULL;

	if(np)
		addTriangleMesh(np);

	return np;
}

// data injected by cooking lib for runtime cooking
PxTriangleMesh* GuMeshFactory::createTriangleMesh(void* data)
{	
	return createTriangleMesh(*reinterpret_cast<TriangleMeshData*>(data));
}

static TriangleMeshData* loadMeshData(PxInputStream& stream)
{
	// Import header
	PxU32 version;
	bool mismatch;
	if(!readHeader('M', 'E', 'S', 'H', version, mismatch, stream))
		return NULL;

	PxU32 midphaseID = PxMeshMidPhase::eBVH33;	// Default before version 14
	if(version>=14)	// this refers to PX_MESH_VERSION
	{
		midphaseID = readDword(mismatch, stream);
	}

	// Check if old (incompatible) mesh format is loaded
	if (version <= 9) // this refers to PX_MESH_VERSION
	{
		Ps::getFoundation().error(PxErrorCode::eINTERNAL_ERROR, __FILE__, __LINE__, "Loading triangle mesh failed: "
			"Deprecated mesh cooking format. Please recook your mesh in a new cooking format.");
		PX_ALWAYS_ASSERT_MESSAGE("Obsolete cooked mesh found. Mesh version has been updated, please recook your meshes.");
		return NULL;
	}

	// Import serialization flags
	const PxU32 serialFlags = readDword(mismatch, stream);

	// Import misc values	
	if (version <= 12) // this refers to PX_MESH_VERSION
	{
		// convexEdgeThreshold was removed in 3.4.0
		readFloat(mismatch, stream);		
	}

	TriangleMeshData* data;
	if(midphaseID==PxMeshMidPhase::eBVH33)
		data = PX_NEW(RTreeTriangleData);
	else if(midphaseID==PxMeshMidPhase::eBVH34)
		data = PX_NEW(BV4TriangleData);
	else return NULL;

	// Import mesh
	PxVec3* verts = data->allocateVertices(readDword(mismatch, stream));
	const PxU32 nbTris = readDword(mismatch, stream);
	bool force32 = (serialFlags & (IMSF_8BIT_INDICES|IMSF_16BIT_INDICES)) == 0;

	//ML: this will allocate CPU triangle indices and GPU triangle indices if we have GRB data built
	void* tris = data->allocateTriangles(nbTris, force32, serialFlags & IMSF_GRB_DATA);

	stream.read(verts, sizeof(PxVec3)*data->mNbVertices);
	if(mismatch)
	{
		for(PxU32 i=0;i<data->mNbVertices;i++)
		{
			flip(verts[i].x);
			flip(verts[i].y);
			flip(verts[i].z);
		}
	}
	//TODO: stop support for format conversion on load!!
	const PxU32 nbIndices = 3*data->mNbTriangles;
	if(serialFlags & IMSF_8BIT_INDICES)
	{
		PxU8 x;
		if(data->has16BitIndices())
		{
			PxU16* tris16 = reinterpret_cast<PxU16*>(tris);
			for(PxU32 i=0;i<nbIndices;i++)
			{
				stream.read(&x, sizeof(PxU8));
				*tris16++ = x;
			}
		}
		else
		{
			PxU32* tris32 = reinterpret_cast<PxU32*>(tris);
			for(PxU32 i=0;i<nbIndices;i++)
			{
				stream.read(&x, sizeof(PxU8));
				*tris32++ = x;
			}
		}
	}
	else if(serialFlags & IMSF_16BIT_INDICES)
	{
		if(data->has16BitIndices())
		{
			PxU16* tris16 = reinterpret_cast<PxU16*>(tris);
			stream.read(tris16, nbIndices*sizeof(PxU16));
			if(mismatch)
			{
				for(PxU32 i=0;i<nbIndices;i++)
					flip(tris16[i]);
			}
		}
		else
		{
			PxU32* tris32 = reinterpret_cast<PxU32*>(tris);
			PxU16 x;
			for(PxU32 i=0;i<nbIndices;i++)
			{
				stream.read(&x, sizeof(PxU16));
				if(mismatch)
					flip(x);

				*tris32++ = x;
			}
		}

	}
	else
	{
		if(data->has16BitIndices())
		{
			PxU32 x;
			PxU16* tris16 = reinterpret_cast<PxU16*>(tris);
			for(PxU32 i=0;i<nbIndices;i++)
			{
				stream.read(&x, sizeof(PxU32));
				if(mismatch)
					flip(x);
				*tris16++ = Ps::to16(x);
			}
		}
		else
		{
			PxU32* tris32 = reinterpret_cast<PxU32*>(tris);
			stream.read(tris32, nbIndices*sizeof(PxU32));

			if(mismatch)
			{
				for(PxU32 i=0;i<nbIndices;i++)
					 flip(tris32[i]);
			}
		}
	}

	if(serialFlags & IMSF_MATERIALS)
	{
		PxU16* materials = data->allocateMaterials();
		stream.read(materials, sizeof(PxU16)*data->mNbTriangles);
		if(mismatch)
		{
			for(PxU32 i=0;i<data->mNbTriangles;i++)
				flip(materials[i]);
		}
	}
	if(serialFlags & IMSF_FACE_REMAP)
	{
		PxU32* remap = data->allocateFaceRemap();
		readIndices(readDword(mismatch, stream), data->mNbTriangles, remap, stream, mismatch);
	}

	if(serialFlags & IMSF_ADJACENCIES)
	{
		PxU32* adj = data->allocateAdjacencies();
		stream.read(adj, sizeof(PxU32)*data->mNbTriangles*3);
		if(mismatch)
		{
			for(PxU32 i=0;i<data->mNbTriangles*3;i++)
				flip(adj[i]);
		}		
	}

	// PT: TODO better
	if(midphaseID==PxMeshMidPhase::eBVH33)
	{
		if(!static_cast<RTreeTriangleData*>(data)->mRTree.load(stream, version, mismatch))
		{
			Ps::getFoundation().error(PxErrorCode::eINTERNAL_ERROR, __FILE__, __LINE__, "RTree binary image load error.");
			PX_DELETE(data);
			return NULL;
		}
	}
	else if(midphaseID==PxMeshMidPhase::eBVH34)
	{
		BV4TriangleData* bv4data = static_cast<BV4TriangleData*>(data);
		if(!bv4data->mBV4Tree.load(stream, mismatch))
		{
			Ps::getFoundation().error(PxErrorCode::eINTERNAL_ERROR, __FILE__, __LINE__, "BV4 binary image load error.");
			PX_DELETE(data);
			return NULL;
		}

		bv4data->mMeshInterface.setNbTriangles(nbTris);
		bv4data->mMeshInterface.setNbVertices(data->mNbVertices);
		if(data->has16BitIndices())
			bv4data->mMeshInterface.setPointers(NULL, reinterpret_cast<IndTri16*>(tris), verts);
		else
			bv4data->mMeshInterface.setPointers(reinterpret_cast<IndTri32*>(tris), NULL, verts);
		bv4data->mBV4Tree.mMeshInterface = &bv4data->mMeshInterface;
	}
	else PX_ASSERT(0);

	// Import local bounds
	data->mGeomEpsilon = readFloat(mismatch, stream);
	readFloatBuffer(&data->mAABB.minimum.x, 6, mismatch, stream);

	PxU32 nb = readDword(mismatch, stream);
	if(nb)
	{
		PX_ASSERT(nb==data->mNbTriangles);
		data->allocateExtraTrigData();
		// No need to convert those bytes
		stream.read(data->mExtraTrigData, nb*sizeof(PxU8));
	}

	if(serialFlags & IMSF_GRB_DATA)
	{
		PxU32 GRB_meshAdjVerticiesTotal = 0;
		if(version < 15)
			GRB_meshAdjVerticiesTotal = readDword(mismatch, stream);

		//read grb triangle indices
		PX_ASSERT(data->mGRB_primIndices);

		if (serialFlags & IMSF_8BIT_INDICES)
		{
			PxU8 x;
			if (data->has16BitIndices())
			{
				PxU16* tris16 = reinterpret_cast<PxU16*>(data->mGRB_primIndices);
				for (PxU32 i = 0; i<nbIndices; i++)
				{
					stream.read(&x, sizeof(PxU8));
					*tris16++ = x;
				}
			}
			else
			{
				PxU32* tris32 = reinterpret_cast<PxU32*>(data->mGRB_primIndices);
				for (PxU32 i = 0; i<nbIndices; i++)
				{
					stream.read(&x, sizeof(PxU8));
					*tris32++ = x;
				}
			}
		}
		else if (serialFlags & IMSF_16BIT_INDICES)
		{
			if (data->has16BitIndices())
			{
				PxU16* tris16 = reinterpret_cast<PxU16*>(data->mGRB_primIndices);
				stream.read(tris16, nbIndices*sizeof(PxU16));
				if (mismatch)
				{
					for (PxU32 i = 0; i<nbIndices; i++)
						flip(tris16[i]);
				}
			}
			else
			{
				PxU32* tris32 = reinterpret_cast<PxU32*>(data->mGRB_primIndices);
				PxU16 x;
				for (PxU32 i = 0; i<nbIndices; i++)
				{
					stream.read(&x, sizeof(PxU16));
					if (mismatch)
						flip(x);

					*tris32++ = x;
				}
			}

		}
		else
		{
			if (data->has16BitIndices())
			{
				PxU32 x;
				PxU16* tris16 = reinterpret_cast<PxU16*>(data->mGRB_primIndices);
				for (PxU32 i = 0; i<nbIndices; i++)
				{
					stream.read(&x, sizeof(PxU32));
					if (mismatch)
						flip(x);
					*tris16++ = Ps::to16(x);
				}
			}
			else
			{
				PxU32* tris32 = reinterpret_cast<PxU32*>(data->mGRB_primIndices);
				stream.read(tris32, nbIndices*sizeof(PxU32));

				if (mismatch)
				{
					for (PxU32 i = 0; i<nbIndices; i++)
						flip(tris32[i]);
				}
			}
		}
		

		data->mGRB_primAdjacencies = static_cast<void *>(PX_NEW(PxU32)[data->mNbTriangles*4]);
		data->mGRB_faceRemap = PX_NEW(PxU32)[data->mNbTriangles];

		stream.read(data->mGRB_primAdjacencies, sizeof(PxU32)*data->mNbTriangles*4);
		if (version < 15)
		{
			//stream.read(data->mGRB_vertValency, sizeof(PxU32)*data->mNbVertices);
			for (PxU32 i = 0; i < data->mNbVertices; ++i)
				readDword(mismatch, stream);
			//stream.read(data->mGRB_adjVertStart, sizeof(PxU32)*data->mNbVertices);
			for (PxU32 i = 0; i < data->mNbVertices; ++i)
				readDword(mismatch, stream);
			//stream.read(data->mGRB_adjVertices, sizeof(PxU32)*GRB_meshAdjVerticiesTotal);
			for (PxU32 i = 0; i < GRB_meshAdjVerticiesTotal; ++i)
				readDword(mismatch, stream);
		}
		stream.read(data->mGRB_faceRemap, sizeof(PxU32)*data->mNbTriangles);

		if(mismatch)
		{
			for(PxU32 i=0;i<data->mNbTriangles*4;i++)
				flip(reinterpret_cast<PxU32 *>(data->mGRB_primIndices)[i]);

			for(PxU32 i=0;i<data->mNbTriangles*4;i++)
				flip(reinterpret_cast<PxU32 *>(data->mGRB_primAdjacencies)[i]);
		}

		//read BV32
		data->mGRB_BV32Tree = PX_NEW(BV32Tree);
		BV32Tree* bv32Tree = static_cast<BV32Tree*>(data->mGRB_BV32Tree);
		if (!bv32Tree->load(stream, mismatch))
		{
			Ps::getFoundation().error(PxErrorCode::eINTERNAL_ERROR, __FILE__, __LINE__, "BV32 binary image load error.");
			PX_DELETE(data);
			return NULL;
		}
	}

	return data;
}

PxTriangleMesh* GuMeshFactory::createTriangleMesh(PxInputStream& desc)
{	
	TriangleMeshData* data = ::loadMeshData(desc);
	if(!data)
		return NULL;
	PxTriangleMesh* m = createTriangleMesh(*data);
	PX_DELETE(data);
	return m;
}

bool GuMeshFactory::removeTriangleMesh(PxTriangleMesh& m)
{
	Ps::Mutex::ScopedLock lock(mTrackingMutex);
	TriangleMesh* gu = static_cast<TriangleMesh*>(&m);
	bool found = mTriangleMeshes.erase(gu);
	return found;
}

PxU32 GuMeshFactory::getNbTriangleMeshes() const
{
	Ps::Mutex::ScopedLock lock(mTrackingMutex);
	return mTriangleMeshes.size();
}

PxU32 GuMeshFactory::getTriangleMeshes(PxTriangleMesh** userBuffer, PxU32 bufferSize, PxU32 startIndex) const
{
	Ps::Mutex::ScopedLock lock(mTrackingMutex);
	return Cm::getArrayOfPointers(userBuffer, bufferSize, startIndex, mTriangleMeshes.getEntries(), mTriangleMeshes.size());
}

///////////////////////////////////////////////////////////////////////////////

void GuMeshFactory::addConvexMesh(ConvexMesh* np, bool lock)
{
	addToHash(mConvexMeshes, np, lock ? &mTrackingMutex : NULL);
}

// data injected by cooking lib for runtime cooking
PxConvexMesh* GuMeshFactory::createConvexMesh(void* data)
{
	return createConvexMesh(*reinterpret_cast<Gu::ConvexHullData*>(data));
}

PxConvexMesh* GuMeshFactory::createConvexMesh(Gu::ConvexHullData& data)
{
	Gu::ConvexMesh *np;
	PX_NEW_SERIALIZED(np, Gu::ConvexMesh)(*this, data);
	if (np)
		addConvexMesh(np);

	return np;
}

PxConvexMesh* GuMeshFactory::createConvexMesh(PxInputStream& desc)
{
	ConvexMesh* np;
	PX_NEW_SERIALIZED(np, ConvexMesh);
	if(!np)
		return NULL;

	np->setMeshFactory(this);

	if(!np->load(desc))
	{
		np->decRefCount();
		return NULL;
	}

	addConvexMesh(np);
	return np;
}

bool GuMeshFactory::removeConvexMesh(PxConvexMesh& m)
{
	Ps::Mutex::ScopedLock lock(mTrackingMutex);
	ConvexMesh* gu = static_cast<ConvexMesh*>(&m);
	bool found = mConvexMeshes.erase(gu);
	return found;
}

PxU32 GuMeshFactory::getNbConvexMeshes() const
{
	Ps::Mutex::ScopedLock lock(mTrackingMutex);
	return mConvexMeshes.size();
}

PxU32 GuMeshFactory::getConvexMeshes(PxConvexMesh** userBuffer, PxU32 bufferSize, PxU32 startIndex) const
{
	Ps::Mutex::ScopedLock lock(mTrackingMutex);
	return Cm::getArrayOfPointers(userBuffer, bufferSize, startIndex, mConvexMeshes.getEntries(), mConvexMeshes.size());
}

///////////////////////////////////////////////////////////////////////////////

void GuMeshFactory::addHeightField(HeightField* np, bool lock)
{
	addToHash(mHeightFields, np, lock ? &mTrackingMutex : NULL);
}

PxHeightField* GuMeshFactory::createHeightField(void* heightFieldMeshData)
{
	HeightField* np;
	PX_NEW_SERIALIZED(np, HeightField)(*this, *reinterpret_cast<Gu::HeightFieldData*>(heightFieldMeshData));
	if(np)
		addHeightField(np);
	
	return np;
}

PxHeightField* GuMeshFactory::createHeightField(PxInputStream& stream)
{
	HeightField* np;
	PX_NEW_SERIALIZED(np, HeightField)(this);
	if(!np)
		return NULL;

	if(!np->load(stream))
	{
		np->decRefCount();
		return NULL;
	}

	addHeightField(np);
	return np;
}

bool GuMeshFactory::removeHeightField(PxHeightField& hf)
{
	Ps::Mutex::ScopedLock lock(mTrackingMutex);
	HeightField* gu = static_cast<HeightField*>(&hf);
	bool found = mHeightFields.erase(gu);
	return found;
}

PxU32 GuMeshFactory::getNbHeightFields() const
{
	Ps::Mutex::ScopedLock lock(mTrackingMutex);
	return mHeightFields.size();
}

PxU32 GuMeshFactory::getHeightFields(PxHeightField** userBuffer, PxU32 bufferSize, PxU32 startIndex) const
{
	Ps::Mutex::ScopedLock lock(mTrackingMutex);
	return Cm::getArrayOfPointers(userBuffer, bufferSize, startIndex, mHeightFields.getEntries(), mHeightFields.size());
}

///////////////////////////////////////////////////////////////////////////////

void GuMeshFactory::addFactoryListener( GuMeshFactoryListener& listener )
{
	Ps::Mutex::ScopedLock lock(mTrackingMutex);
	mFactoryListeners.pushBack( &listener );
}

void GuMeshFactory::removeFactoryListener( GuMeshFactoryListener& listener )
{
	Ps::Mutex::ScopedLock lock(mTrackingMutex);
	for ( PxU32 idx = 0; idx < mFactoryListeners.size(); ++idx )
	{
		if ( mFactoryListeners[idx] == &listener )
		{
			mFactoryListeners.replaceWithLast( idx );
			--idx;
		}
	}
}

void GuMeshFactory::notifyFactoryListener(const PxBase* base, PxType typeID)
{
	const PxU32 nbListeners = mFactoryListeners.size();
	for(PxU32 i=0; i<nbListeners; i++)
		mFactoryListeners[i]->onGuMeshFactoryBufferRelease(base, typeID);
}

///////////////////////////////////////////////////////////////////////////////

void GuMeshFactory::addBVHStructure(BVHStructure* np, bool lock)
{
	addToHash(mBVHStructures, np, lock ? &mTrackingMutex : NULL);
}

// data injected by cooking lib for runtime cooking
PxBVHStructure* GuMeshFactory::createBVHStructure(void* data)
{
	return createBVHStructure(*reinterpret_cast<Gu::BVHStructureData*>(data));
}

PxBVHStructure* GuMeshFactory::createBVHStructure(Gu::BVHStructureData& data)
{
	Gu::BVHStructure *np;
	PX_NEW_SERIALIZED(np, Gu::BVHStructure)(this, data);
	if (np)
		addBVHStructure(np);

	return np;
}

PxBVHStructure* GuMeshFactory::createBVHStructure(PxInputStream& desc)
{
	BVHStructure* np;
	PX_NEW_SERIALIZED(np, BVHStructure)(this);
	if(!np)
		return NULL;

	if(!np->load(desc))
	{
		np->decRefCount();
		return NULL;
	}

	addBVHStructure(np);
	return np;
}

bool GuMeshFactory::removeBVHStructure(PxBVHStructure& m)
{
	Ps::Mutex::ScopedLock lock(mTrackingMutex);
	BVHStructure* gu = static_cast<BVHStructure*>(&m);
	bool found = mBVHStructures.erase(gu);
	return found;
}

PxU32 GuMeshFactory::getNbBVHStructures() const
{
	Ps::Mutex::ScopedLock lock(mTrackingMutex);
	return mBVHStructures.size();
}

PxU32 GuMeshFactory::getBVHStructures(PxBVHStructure** userBuffer, PxU32 bufferSize, PxU32 startIndex) const
{
	Ps::Mutex::ScopedLock lock(mTrackingMutex);
	return Cm::getArrayOfPointers(userBuffer, bufferSize, startIndex, mBVHStructures.getEntries(), mBVHStructures.size());
}

///////////////////////////////////////////////////////////////////////////////

