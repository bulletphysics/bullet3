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


#include "foundation/PxMemory.h"
#include "PsIntrinsics.h"
#include "GuHeightField.h"
#include "PsAllocator.h"
#include "PsUtilities.h"
#include "GuMeshFactory.h"
#include "GuSerialize.h"
#include "CmUtils.h"
#include "CmBitMap.h"
#include "PsFoundation.h"

using namespace physx;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Gu::HeightField::HeightField(GuMeshFactory* meshFactory)
: PxHeightField(PxConcreteType::eHEIGHTFIELD, PxBaseFlag::eOWNS_MEMORY | PxBaseFlag::eIS_RELEASABLE)
, mSampleStride	(0)
, mNbSamples	(0)
, mMinHeight	(0.0f)
, mMaxHeight	(0.0f)
, mModifyCount	(0)
, mMeshFactory	(meshFactory)
{
	mData.format				= PxHeightFieldFormat::eS16_TM;
	mData.rows					= 0;
	mData.columns				= 0;
	mData.convexEdgeThreshold	= 0;
	mData.flags					= PxHeightFieldFlags();
	mData.samples				= NULL;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Gu::HeightField::HeightField(GuMeshFactory& factory, Gu::HeightFieldData& data)
: PxHeightField(PxConcreteType::eHEIGHTFIELD, PxBaseFlag::eOWNS_MEMORY | PxBaseFlag::eIS_RELEASABLE)
, mSampleStride	(0)
, mNbSamples	(0)
, mMinHeight	(0.0f)
, mMaxHeight	(0.0f)
, mModifyCount	(0)
, mMeshFactory	(&factory)
{
	mData = data;
	data.samples = NULL; // set to null so that we don't release the memory
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Gu::HeightField::~HeightField()
{
	releaseMemory();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// PX_SERIALIZATION
void Gu::HeightField::onRefCountZero()
{
	PX_ASSERT(mMeshFactory);
	if(mMeshFactory->removeHeightField(*this))
	{
		GuMeshFactory* mf = mMeshFactory;
		Cm::deletePxBase(this);
		mf->notifyFactoryListener(this, PxConcreteType::eHEIGHTFIELD);
		return;
	}
	
	// PT: if we reach this point, we didn't find the mesh in the Physics object => don't delete!
	// This prevents deleting the object twice.
	Ps::getFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "Gu::HeightField::onRefCountZero: double deletion detected!");
}

void Gu::HeightField::exportExtraData(PxSerializationContext& stream)
{
	// PT: warning, order matters for the converter. Needs to export the base stuff first
	const PxU32 size = mData.rows * mData.columns * sizeof(PxHeightFieldSample);
	stream.alignData(PX_SERIAL_ALIGN);	// PT: generic align within the generic allocator
	stream.writeData(mData.samples, size);
}

void Gu::HeightField::importExtraData(PxDeserializationContext& context)
{
	mData.samples = context.readExtraData<PxHeightFieldSample, PX_SERIAL_ALIGN>(mData.rows * mData.columns);
}

Gu::HeightField* Gu::HeightField::createObject(PxU8*& address, PxDeserializationContext& context)
{
	HeightField* obj = new (address) HeightField(PxBaseFlag::eIS_RELEASABLE);
	address += sizeof(HeightField);	
	obj->importExtraData(context);
	obj->resolveReferences(context);
	return obj;
}

//~PX_SERIALIZATION

void Gu::HeightField::release()
{
	decRefCount();
}

void Gu::HeightField::acquireReference()
{
	incRefCount();
}

PxU32 Gu::HeightField::getReferenceCount() const
{
	return getRefCount();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool Gu::HeightField::modifySamples(PxI32 startCol, PxI32 startRow, const PxHeightFieldDesc& desc, bool shrinkBounds)
{
	const PxU32 nbCols = getNbColumns();
	const PxU32 nbRows = getNbRows();
	PX_CHECK_AND_RETURN_NULL(desc.format == mData.format, "Gu::HeightField::modifySamples: desc.format mismatch");
	//PX_CHECK_AND_RETURN_NULL(startCol + desc.nbColumns <= nbCols,
	//	"Gu::HeightField::modifySamples: startCol + nbColumns out of range");
	//PX_CHECK_AND_RETURN_NULL(startRow + desc.nbRows <= nbRows,
	//	"Gu::HeightField::modifySamples: startRow + nbRows out of range");
	//PX_CHECK_AND_RETURN_NULL(desc.samples.stride == mSampleStride, "Gu::HeightField::modifySamples: desc.samples.stride mismatch");

	// by default bounds don't shrink since the whole point of this function is to avoid modifying the whole HF
	// unless shrinkBounds is specified. then the bounds will be fully recomputed later
	PxReal minHeight = mMinHeight;
	PxReal maxHeight = mMaxHeight;
	PxU32 hiRow = PxMin(PxU32(PxMax(0, startRow + PxI32(desc.nbRows))), nbRows);
	PxU32 hiCol = PxMin(PxU32(PxMax(0, startCol + PxI32(desc.nbColumns))), nbCols);
	for (PxU32 row = PxU32(PxMax(startRow, 0)); row < hiRow; row++)
	{
		for (PxU32 col = PxU32(PxMax(startCol, 0)); col < hiCol; col++)
		{
			const PxU32 vertexIndex = col + row*nbCols;
			PxHeightFieldSample* targetSample = &mData.samples[vertexIndex];

			// update target sample from source sample
			const PxHeightFieldSample& sourceSample =
				(reinterpret_cast<const PxHeightFieldSample*>(desc.samples.data))[col - startCol + (row - startRow) * desc.nbColumns];
			*targetSample = sourceSample;

			if(isCollisionVertexPreca(vertexIndex, row, col, PxHeightFieldMaterial::eHOLE))
				targetSample->materialIndex1.setBit();
			else
				targetSample->materialIndex1.clearBit();

			// grow (but not shrink) the height extents
			const PxReal h = getHeight(vertexIndex);
			minHeight = physx::intrinsics::selectMin(h, minHeight);
			maxHeight = physx::intrinsics::selectMax(h, maxHeight);
		}
	}

	if (shrinkBounds)
	{
		// do a full recompute on vertical bounds to allow shrinking
		minHeight = PX_MAX_REAL;
		maxHeight = -PX_MAX_REAL;
		// have to recompute the min&max from scratch...
		for (PxU32 vertexIndex = 0; vertexIndex < nbRows * nbCols; vertexIndex ++)
		{
				// update height extents
				const PxReal h = getHeight(vertexIndex);
				minHeight = physx::intrinsics::selectMin(h, minHeight);
				maxHeight = physx::intrinsics::selectMax(h, maxHeight);
		}
	}
	mMinHeight = minHeight;
	mMaxHeight = maxHeight;

	// update local space aabb
	CenterExtents& bounds = mData.mAABB;
	bounds.mCenter.y = (maxHeight + minHeight)*0.5f;
	bounds.mExtents.y = (maxHeight - minHeight)*0.5f;

	mModifyCount++;

	return true;
}

bool Gu::HeightField::load(PxInputStream& stream)
{
	// release old memory
	releaseMemory();

	// Import header
	PxU32 version;
	bool endian;
	if(!readHeader('H', 'F', 'H', 'F', version, endian, stream))
		return false;

	// load mData
	mData.rows = readDword(endian, stream);
	mData.columns = readDword(endian, stream);
	mData.rowLimit = readFloat(endian, stream);
	mData.colLimit = readFloat(endian, stream);
	mData.nbColumns = readFloat(endian, stream);
	const float thickness = readFloat(endian, stream);
	PX_UNUSED(thickness);
	mData.convexEdgeThreshold = readFloat(endian, stream);

	PxU16 flags = readWord(endian, stream);
	mData.flags = PxHeightFieldFlags(flags);

	PxU32 format = readDword(endian, stream);
	mData.format = PxHeightFieldFormat::Enum(format);

	PxBounds3 minMaxBounds;
	minMaxBounds.minimum.x = readFloat(endian, stream);
	minMaxBounds.minimum.y = readFloat(endian, stream);
	minMaxBounds.minimum.z = readFloat(endian, stream);
	minMaxBounds.maximum.x = readFloat(endian, stream);
	minMaxBounds.maximum.y = readFloat(endian, stream);
	minMaxBounds.maximum.z = readFloat(endian, stream);
	mData.mAABB = CenterExtents(minMaxBounds);

	mSampleStride = readDword(endian, stream);
	mNbSamples = readDword(endian, stream);
	mMinHeight = readFloat(endian, stream);
	mMaxHeight = readFloat(endian, stream);

	// allocate height samples
	mData.samples = NULL;
	const PxU32 nbVerts = mData.rows * mData.columns;
	if (nbVerts > 0) 
	{
		mData.samples = reinterpret_cast<PxHeightFieldSample*>(PX_ALLOC(nbVerts*sizeof(PxHeightFieldSample), "PxHeightFieldSample"));
		if (mData.samples == NULL)
		{
			Ps::getFoundation().error(PxErrorCode::eOUT_OF_MEMORY, __FILE__, __LINE__, "Gu::HeightField::load: PX_ALLOC failed!");
			return false;
		}
		stream.read(mData.samples, mNbSamples*sizeof(PxHeightFieldSample));
		if (endian)
			for(PxU32 i = 0; i < mNbSamples; i++)
			{
				PxHeightFieldSample& s = mData.samples[i];
				PX_ASSERT(sizeof(PxU16) == sizeof(s.height));
				flip(s.height);
			}
	}

	return true;
}

bool Gu::HeightField::loadFromDesc(const PxHeightFieldDesc& desc)
{
	// verify descriptor	
	PX_CHECK_AND_RETURN_NULL(desc.isValid(), "Gu::HeightField::loadFromDesc: desc.isValid() failed!");
	
	// release old memory
	releaseMemory();

	// copy trivial data
	mData.format				= desc.format;
	mData.rows					= desc.nbRows;
	mData.columns				= desc.nbColumns;
	mData.convexEdgeThreshold	= desc.convexEdgeThreshold;
	mData.flags					= desc.flags;
	mSampleStride				= desc.samples.stride;

	// PT: precompute some data - mainly for Xbox
	mData.rowLimit				= float(mData.rows - 2);
	mData.colLimit				= float(mData.columns - 2);
	mData.nbColumns				= float(desc.nbColumns);

	// allocate and copy height samples
	// compute extents too
	mData.samples = NULL;
	const PxU32 nbVerts = desc.nbRows * desc.nbColumns;
	mMinHeight = PX_MAX_REAL;
	mMaxHeight = -PX_MAX_REAL;

	if(nbVerts > 0) 
	{
		mData.samples = reinterpret_cast<PxHeightFieldSample*>(PX_ALLOC(nbVerts*sizeof(PxHeightFieldSample), "PxHeightFieldSample"));
		if(!mData.samples)
		{
			Ps::getFoundation().error(PxErrorCode::eOUT_OF_MEMORY, __FILE__, __LINE__, "Gu::HeightField::load: PX_ALLOC failed!");
			return false;
		}
		const PxU8* PX_RESTRICT src = reinterpret_cast<const PxU8*>(desc.samples.data);
		PxHeightFieldSample* PX_RESTRICT dst = mData.samples;
		PxI16 minHeight = PX_MAX_I16;
		PxI16 maxHeight = PX_MIN_I16;
		for(PxU32 i=0;i<nbVerts;i++)
		{			
			const PxHeightFieldSample& sample = *reinterpret_cast<const PxHeightFieldSample*>(src);
			*dst++ = sample;
			const PxI16 height = sample.height;
			minHeight = height < minHeight ? height : minHeight;
			maxHeight = height > maxHeight ? height : maxHeight;
			src += desc.samples.stride;	
		}
		mMinHeight = PxReal(minHeight);
		mMaxHeight = PxReal(maxHeight);
	}

	PX_ASSERT(mMaxHeight >= mMinHeight);

	parseTrianglesForCollisionVertices(PxHeightFieldMaterial::eHOLE);

// PT: "mNbSamples" only used by binary converter
	mNbSamples	= mData.rows * mData.columns;

	//Compute local space aabb.
	PxBounds3 bounds;
	bounds.minimum.y = getMinHeight();
	bounds.maximum.y = getMaxHeight();

	bounds.minimum.x = 0;
	bounds.maximum.x = PxReal(getNbRowsFast() - 1);
	bounds.minimum.z = 0;
	bounds.maximum.z = PxReal(getNbColumnsFast() - 1);
	mData.mAABB=bounds;

	return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

PxU32 Gu::HeightField::saveCells(void* destBuffer, PxU32 destBufferSize) const
{
	PxU32 n = mData.columns * mData.rows * sizeof(PxHeightFieldSample);
	if (n > destBufferSize) n = destBufferSize;
	PxMemCopy(destBuffer, mData.samples, n);

	return n;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

PX_PHYSX_COMMON_API void Gu::HeightField::releaseMemory()
{
// PX_SERIALIZATION
	if(getBaseFlags() & PxBaseFlag::eOWNS_MEMORY)
//~PX_SERIALIZATION
	{
		PX_FREE(mData.samples);
		mData.samples = NULL;
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// PT: TODO: use those faster functions everywhere
namespace physx
{

PX_PHYSX_COMMON_API PxU32 getVertexEdgeIndices(const Gu::HeightField& heightfield, PxU32 vertexIndex, PxU32 row, PxU32 column, EdgeData edgeIndices[8])
{
	const PxU32 nbColumns = heightfield.getData().columns;
	const PxU32 nbRows = heightfield.getData().rows;
	PX_ASSERT((vertexIndex / nbColumns)==row);
	PX_ASSERT((vertexIndex % nbColumns)==column);

	PxU32 count = 0;
	
	if (row > 0) 
	{
//		edgeIndices[count++] = 3 * (vertexIndex - nbColumns) + 2;
		const PxU32 cell = vertexIndex - nbColumns;
		edgeIndices[count].edgeIndex	= 3 * cell + 2;
		edgeIndices[count].cell			= cell;
		edgeIndices[count].row			= row-1;
		edgeIndices[count].column		= column;
		count++;
	}
	
	if (column < nbColumns-1)
	{
		if (row > 0)
		{
			if (!heightfield.isZerothVertexShared(vertexIndex - nbColumns))
			{
//				edgeIndices[count++] = 3 * (vertexIndex - nbColumns) + 1;
				const PxU32 cell = vertexIndex - nbColumns;
				edgeIndices[count].edgeIndex	= 3 * cell + 1;
				edgeIndices[count].cell			= cell;
				edgeIndices[count].row			= row-1;
				edgeIndices[count].column		= column;
				count++;
			}
		}
//		edgeIndices[count++] = 3 * vertexIndex;
		edgeIndices[count].edgeIndex	= 3 * vertexIndex;
		edgeIndices[count].cell			= vertexIndex;
		edgeIndices[count].row			= row;
		edgeIndices[count].column		= column;
		count++;

		if (row < nbRows - 1)
		{
			if (heightfield.isZerothVertexShared(vertexIndex))
			{
//				edgeIndices[count++] = 3 * vertexIndex + 1;
				edgeIndices[count].edgeIndex	= 3 * vertexIndex + 1;
				edgeIndices[count].cell			= vertexIndex;
				edgeIndices[count].row			= row;
				edgeIndices[count].column		= column;
				count++;
			}
		}
	}

	if (row < nbRows - 1)
	{
//		edgeIndices[count++] = 3 * vertexIndex + 2;
		edgeIndices[count].edgeIndex	= 3 * vertexIndex + 2;
		edgeIndices[count].cell			= vertexIndex;
		edgeIndices[count].row			= row;
		edgeIndices[count].column		= column;
		count++;
	}

	if (column > 0)
	{
		if (row < nbRows - 1)
		{
			if (!heightfield.isZerothVertexShared(vertexIndex - 1))
			{
//				edgeIndices[count++] = 3 * (vertexIndex - 1) + 1;
				const PxU32 cell = vertexIndex - 1;
				edgeIndices[count].edgeIndex	= 3 * cell + 1;
				edgeIndices[count].cell			= cell;
				edgeIndices[count].row			= row;
				edgeIndices[count].column		= column-1;
				count++;
			}
		}
//		edgeIndices[count++] = 3 * (vertexIndex - 1);
		const PxU32 cell = vertexIndex - 1;
		edgeIndices[count].edgeIndex	= 3 * cell;
		edgeIndices[count].cell			= cell;
		edgeIndices[count].row			= row;
		edgeIndices[count].column		= column-1;
		count++;
		if (row > 0)
		{
			if (heightfield.isZerothVertexShared(vertexIndex - nbColumns - 1))
			{
//				edgeIndices[count++] = 3 * (vertexIndex - nbColumns - 1) + 1;
				const PxU32 cell1 = vertexIndex - nbColumns - 1;
				edgeIndices[count].edgeIndex	= 3 * cell1 + 1;
				edgeIndices[count].cell			= cell1;
				edgeIndices[count].row			= row-1;
				edgeIndices[count].column		= column-1;
				count++;
			}
		}
	}
	return count;
}

PX_PHYSX_COMMON_API PxU32 getEdgeTriangleIndices(const Gu::HeightField& heightfield, const EdgeData& edgeData, PxU32* PX_RESTRICT triangleIndices)
{
	const PxU32 nbColumns = heightfield.getData().columns;
	const PxU32 nbRows = heightfield.getData().rows;

	const PxU32 edgeIndex	= edgeData.edgeIndex;
	const PxU32 cell		= edgeData.cell;
	const PxU32 row			= edgeData.row;
	const PxU32 column		= edgeData.column;
	PX_ASSERT(cell==edgeIndex / 3);
	PX_ASSERT(row==cell / nbColumns);
	PX_ASSERT(column==cell % nbColumns);
	PxU32 count = 0;
	switch (edgeIndex - cell*3)
	{
		case 0:
			if (column < nbColumns - 1)
			{
				if (row > 0)
				{
					if (heightfield.isZerothVertexShared(cell - nbColumns))
						triangleIndices[count++] = ((cell - nbColumns) << 1);
					else 
						triangleIndices[count++] = ((cell - nbColumns) << 1) + 1;
				}
				if (row < nbRows - 1)
				{
					if (heightfield.isZerothVertexShared(cell))
						triangleIndices[count++] = (cell << 1) + 1;
					else 
						triangleIndices[count++] = cell << 1;
				}
			}
			break;
		case 1:
			if ((row < nbRows - 1) && (column < nbColumns - 1))
			{
				triangleIndices[count++] = cell << 1;
				triangleIndices[count++] = (cell << 1) + 1;
			}
			break;
		case 2:
			if (row < nbRows - 1)
			{
				if (column > 0)
				{
					triangleIndices[count++] = ((cell - 1) << 1) + 1;
				}
				if (column < nbColumns - 1)
				{
					triangleIndices[count++] = cell << 1;
				}
			}
			break;
	}

	return count;
}

}

PX_FORCE_INLINE PxU32 anyHole(PxU32 doubleMatIndex, PxU16 holeMaterialIndex)
{
	return PxU32((doubleMatIndex & 0xFFFF) == holeMaterialIndex) | (PxU32(doubleMatIndex >> 16) == holeMaterialIndex);
}

void Gu::HeightField::parseTrianglesForCollisionVertices(PxU16 holeMaterialIndex)
{	
	const PxU32 nbColumns = getNbColumnsFast();
	const PxU32 nbRows = getNbRowsFast();

	Cm::BitMap rowHoles[2];
	rowHoles[0].resizeAndClear(nbColumns + 1);
	rowHoles[1].resizeAndClear(nbColumns + 1);

	for (PxU32 iCol = 0; iCol < nbColumns; iCol++)
	{
		if (anyHole(getMaterialIndex01(iCol), holeMaterialIndex))
		{
			rowHoles[0].set(iCol);
			rowHoles[0].set(iCol + 1);
		}
		PxU32 vertIndex = iCol;
		if(isCollisionVertexPreca(vertIndex, 0, iCol, holeMaterialIndex))
			mData.samples[vertIndex].materialIndex1.setBit();
		else
			mData.samples[vertIndex].materialIndex1.clearBit();
	}

	PxU32 nextRow = 1, currentRow = 0;
	for (PxU32 iRow = 1; iRow < nbRows; iRow++)
	{
		PxU32 rowOffset = iRow*nbColumns;
		for (PxU32 iCol = 0; iCol < nbColumns; iCol++)
		{
			const PxU32 vertIndex = rowOffset + iCol; // column index plus current row offset (vertex/cell index)
			if(anyHole(getMaterialIndex01(vertIndex), holeMaterialIndex))
			{
				rowHoles[currentRow].set(iCol);
				rowHoles[currentRow].set(iCol + 1);
				rowHoles[nextRow].set(iCol);
				rowHoles[nextRow].set(iCol + 1);
			}

			if ((iCol == 0) || (iCol == nbColumns - 1) || (iRow == nbRows - 1) || rowHoles[currentRow].test(iCol))
			{
				if(isCollisionVertexPreca(vertIndex, iRow, iCol, holeMaterialIndex))
					mData.samples[vertIndex].materialIndex1.setBit();
				else
					mData.samples[vertIndex].materialIndex1.clearBit();
			} else
			{
				if (isConvexVertex(vertIndex, iRow, iCol))
					mData.samples[vertIndex].materialIndex1.setBit();
			}
		}

		rowHoles[currentRow].clear();

		// swap prevRow and prevPrevRow
		nextRow ^= 1; currentRow ^= 1;
	}
}

bool Gu::HeightField::isSolidVertex(PxU32 vertexIndex, PxU32 row, PxU32 column, PxU16 holeMaterialIndex, bool& nbSolid) const
{
	// check if solid and boundary
	// retrieve edge indices for current vertexIndex
	EdgeData edgeIndices[8];
	const PxU32 edgeCount = ::getVertexEdgeIndices(*this, vertexIndex, row, column, edgeIndices);

	PxU32 faceCounts[8];
	PxU32 faceIndices[2 * 8];
	PxU32* dst = faceIndices;
	for (PxU32 i = 0; i < edgeCount; i++)
	{
		faceCounts[i] = ::getEdgeTriangleIndices(*this, edgeIndices[i], dst);
		dst += 2;
	}
	
	nbSolid = false;
	const PxU32* currentfaceIndices = faceIndices; // parallel array of pairs of face indices per edge index
	for (PxU32 i = 0; i < edgeCount; i++)
	{
		if (faceCounts[i] > 1)
		{
			const PxU16& material0 = getTriangleMaterial(currentfaceIndices[0]);
			const PxU16& material1 = getTriangleMaterial(currentfaceIndices[1]);
			// ptchernev TODO: this is a bit arbitrary
			if (material0 != holeMaterialIndex)
			{
				nbSolid = true;
				if (material1 == holeMaterialIndex)
					return true; // edge between solid and hole => return true
			}
			if (material1 != holeMaterialIndex)
			{
				nbSolid = true;
				if (material0 == holeMaterialIndex)
					return true; // edge between hole and solid => return true
			}
		}
		else
		{
			if (getTriangleMaterial(currentfaceIndices[0]) != holeMaterialIndex)
				return true;
		}
		currentfaceIndices += 2; // 2 face indices per edge
	}
	return false;
}

bool Gu::HeightField::isCollisionVertexPreca(PxU32 vertexIndex, PxU32 row, PxU32 column, PxU16 holeMaterialIndex) const
{
#ifdef PX_HEIGHTFIELD_DEBUG
	PX_ASSERT(isValidVertex(vertexIndex));
#endif
	PX_ASSERT((vertexIndex / getNbColumnsFast()) == row);
	PX_ASSERT((vertexIndex % getNbColumnsFast()) == column);

	// check boundary conditions - boundary edges shouldn't produce collision with eNO_BOUNDARY_EDGES flag
	if(mData.flags & PxHeightFieldFlag::eNO_BOUNDARY_EDGES) 
		if ((row == 0) || (column == 0) || (row >= mData.rows-1) || (column >= mData.columns-1))
			return false;

	bool nbSolid;
	if(isSolidVertex(vertexIndex, row, column, holeMaterialIndex, nbSolid))
		return true;

	// return true if it is boundary or solid and convex
	return (nbSolid && isConvexVertex(vertexIndex, row, column));
}

// AP: this naming is confusing and inconsistent with return value. the function appears to compute vertex coord rather than cell coords
// it would most likely be better to stay in cell coords instead, since fractional vertex coords just do not make any sense
PxU32 Gu::HeightField::computeCellCoordinates(PxReal x, PxReal z, PxReal& fracX, PxReal& fracZ) const
{
	namespace i = physx::intrinsics;

	x = i::selectMax(x, 0.0f);
	z = i::selectMax(z, 0.0f);
#if 0 // validation code for scaled clamping epsilon computation
	for (PxReal ii = 1.0f; ii < 100000.0f; ii+=1.0f)
	{
		PX_UNUSED(ii);
		PX_ASSERT(PxFloor(ii+(1-1e-7f*ii)) == ii);
	}
#endif
	PxF32 epsx = 1.0f - PxAbs(x+1.0f) * 1e-6f; // epsilon needs to scale with values of x,z...
	PxF32 epsz = 1.0f - PxAbs(z+1.0f) * 1e-6f;
	PxF32 x1 = i::selectMin(x, mData.rowLimit+epsx);
	PxF32 z1 = i::selectMin(z, mData.colLimit+epsz);
	x = PxFloor(x1);
	fracX = x1 - x;
	z = PxFloor(z1);
	fracZ = z1 - z;
	PX_ASSERT(x >= 0.0f && x < PxF32(mData.rows));
	PX_ASSERT(z >= 0.0f && z < PxF32(mData.columns));

	const PxU32 vertexIndex = PxU32(x * (mData.nbColumns) + z);
	PX_ASSERT(vertexIndex < (mData.rows)*(mData.columns));

	return vertexIndex;
}

