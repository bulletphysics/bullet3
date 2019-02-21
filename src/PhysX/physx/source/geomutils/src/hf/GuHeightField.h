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

#ifndef GU_HEIGHTFIELD_H
#define GU_HEIGHTFIELD_H

#include "PsUserAllocated.h"
#include "CmRefCountable.h"
#include "PsMathUtils.h"
#include "GuSphere.h"
#include "PxHeightFieldSample.h"
#include "PxHeightFieldDesc.h"
#include "GuHeightFieldData.h"
#include "PxHeightField.h"

//#define PX_HEIGHTFIELD_VERSION 0
#define PX_HEIGHTFIELD_VERSION 1  // tiled version that was needed for PS3 only has been removed

namespace physx
{
class GuMeshFactory;
class PxHeightFieldDesc;
}

namespace physx
{
namespace Gu
{
class HeightField : public PxHeightField, public Ps::UserAllocated, public Cm::RefCountable
{
//= ATTENTION! =====================================================================================
// Changing the data layout of this class breaks the binary serialization format.  See comments for 
// PX_BINARY_SERIAL_VERSION.  If a modification is required, please adjust the getBinaryMetaData 
// function.  If the modification is made on a custom branch, please change PX_BINARY_SERIAL_VERSION
// accordingly.
//==================================================================================================
public:
// PX_SERIALIZATION
		HeightField(PxBaseFlags baseFlags) : PxHeightField(baseFlags), Cm::RefCountable(PxEmpty), mData(PxEmpty), mModifyCount(0) {}

		PX_PHYSX_COMMON_API virtual		void						exportExtraData(PxSerializationContext&);
		PX_PHYSX_COMMON_API				void						importExtraData(PxDeserializationContext& context);
		PX_FORCE_INLINE					void						setMeshFactory(GuMeshFactory* f)		{ mMeshFactory = f;					}
		PX_PHYSX_COMMON_API	static		HeightField*				createObject(PxU8*& address, PxDeserializationContext& context);
		PX_PHYSX_COMMON_API static		void						getBinaryMetaData(PxOutputStream& stream);
										void						resolveReferences(PxDeserializationContext&) {}

							virtual		void						requiresObjects(PxProcessPxBaseCallback&){}
//~PX_SERIALIZATION

		PX_PHYSX_COMMON_API 										HeightField(GuMeshFactory* meshFactory);
		PX_PHYSX_COMMON_API											HeightField(GuMeshFactory& factory, Gu::HeightFieldData& data);

		// PxHeightField
		PX_PHYSX_COMMON_API virtual		void						release();
		PX_PHYSX_COMMON_API virtual		PxU32						saveCells(void* destBuffer, PxU32 destBufferSize) const;
		PX_PHYSX_COMMON_API virtual		bool						modifySamples(PxI32 startCol, PxI32 startRow, const PxHeightFieldDesc& subfieldDesc, bool shrinkBounds);
		PX_PHYSX_COMMON_API virtual		PxU32						getNbRows()						const	{ return mData.rows;					}
		PX_PHYSX_COMMON_API virtual		PxU32						getNbColumns()					const	{ return mData.columns;					}
		PX_PHYSX_COMMON_API virtual		PxHeightFieldFormat::Enum	getFormat()						const	{ return mData.format;					}
		PX_PHYSX_COMMON_API virtual		PxU32						getSampleStride()				const	{ return sizeof(PxHeightFieldSample);	}
		PX_PHYSX_COMMON_API virtual		PxReal						getConvexEdgeThreshold()		const	{ return mData.convexEdgeThreshold;		}
		PX_PHYSX_COMMON_API virtual		PxHeightFieldFlags			getFlags()						const	{ return mData.flags;					}
		PX_PHYSX_COMMON_API virtual		PxReal						getHeight(PxReal x, PxReal z)	const	{ return getHeightInternal(x, z);		}

		PX_PHYSX_COMMON_API virtual		void						acquireReference();
		PX_PHYSX_COMMON_API virtual		PxU32						getReferenceCount()				const;
		//~PxHeightField

		// RefCountable
		PX_PHYSX_COMMON_API virtual		void						onRefCountZero();
		//~RefCountable
		PX_PHYSX_COMMON_API virtual		PxMaterialTableIndex		getTriangleMaterialIndex(PxTriangleID triangleIndex)	const
																	{
																		return getTriangleMaterial(triangleIndex);
																	}

		PX_PHYSX_COMMON_API virtual		PxVec3						getTriangleNormal(PxTriangleID triangleIndex)	const
																	{
																		return getTriangleNormalInternal(triangleIndex);
																	}

		PX_PHYSX_COMMON_API virtual	const PxHeightFieldSample&		getSample(PxU32 row, PxU32 column) const
																	{
																		const PxU32 cell = row * getNbColumnsFast() + column;
																		return getSample(cell);
																	}
		/**
		\brief Returns the number of times the heightfield data has been modified
	
		Each time the heightfield is changed via 'modifySamples' this increments a counter.  This method will return
		the number of times the heightfield has been modified so that rendering code can know whether or not it needs to
		rebuild the graphics representation of the mesh.
	
		\return the number of times the heightfield sample data has been modified.
		*/
		PX_PHYSX_COMMON_API virtual		PxU32						getTimestamp()			const	{ return mModifyCount;	}

	    PX_PHYSX_COMMON_API bool						loadFromDesc(const PxHeightFieldDesc&);
	    PX_PHYSX_COMMON_API bool						load(PxInputStream&);

	PX_CUDA_CALLABLE PX_FORCE_INLINE	PxU32	getNbRowsFast()					const	{ return mData.rows;				}
	PX_CUDA_CALLABLE PX_FORCE_INLINE	PxU32	getNbColumnsFast()				const	{ return mData.columns;				}
	PX_FORCE_INLINE	PxHeightFieldFormat::Enum	getFormatFast()					const	{ return mData.format;				}
	PX_FORCE_INLINE	PxU32						getFlagsFast()					const	{ return mData.flags;				}

	PX_FORCE_INLINE	bool						isDeltaHeightInsideExtent(PxReal dy, PxReal eps = 0.0f) const	
												{ 
													const float thickness = 0.0f;
													return (thickness <= 0.0f && dy <= eps && dy >= thickness) || 
															(thickness > 0.0f && dy > -eps && dy < thickness);
												}

	PX_FORCE_INLINE	bool						isDeltaHeightOppositeExtent(PxReal dy) const	
												{
													const float thickness = 0.0f;
													return (thickness <= 0.0f && dy > 0.0f) || (thickness > 0.0f && dy < 0.0f);
												}

	PX_CUDA_CALLABLE PX_FORCE_INLINE	bool	isZerothVertexShared(PxU32 vertexIndex) const
												{
//													return (getSample(vertexIndex).tessFlag & PxHeightFieldTessFlag::e0TH_VERTEX_SHARED);
													return getSample(vertexIndex).tessFlag() != 0;
												}

	PX_CUDA_CALLABLE PX_FORCE_INLINE	PxU16	getMaterialIndex0(PxU32 vertexIndex) const	{ return getSample(vertexIndex).materialIndex0;	}
	PX_CUDA_CALLABLE PX_FORCE_INLINE	PxU16	getMaterialIndex1(PxU32 vertexIndex) const	{ return getSample(vertexIndex).materialIndex1;	}
	PX_CUDA_CALLABLE PX_FORCE_INLINE	PxU32	getMaterialIndex01(PxU32 vertexIndex) const
												{
													const PxHeightFieldSample& sample = getSample(vertexIndex);
													return PxU32(sample.materialIndex0 | (sample.materialIndex1 << 16));
												}

	PX_CUDA_CALLABLE PX_FORCE_INLINE	PxReal	getHeight(PxU32 vertexIndex) const
												{
													return PxReal(getSample(vertexIndex).height);
												}

	PX_INLINE		PxReal						getHeightInternal2(PxU32 vertexIndex, PxReal fracX, PxReal fracZ)	const;
	PX_FORCE_INLINE	PxReal						getHeightInternal(PxReal x, PxReal z) const
												{
													PxReal fracX, fracZ;
													const PxU32 vertexIndex = computeCellCoordinates(x, z, fracX, fracZ);

													return getHeightInternal2(vertexIndex, fracX, fracZ);
												}

	PX_FORCE_INLINE bool						isValidVertex(PxU32 vertexIndex) const	{ return vertexIndex < mData.rows*mData.columns;	}

	PX_INLINE		PxVec3						getVertex(PxU32 vertexIndex) const;
	PX_INLINE		bool						isConvexVertex(PxU32 vertexIndex, PxU32 row, PxU32 column) const;

	PX_INLINE		bool						isValidEdge(PxU32 edgeIndex) const;
	PX_INLINE		PxU32						getEdgeTriangleIndices(PxU32 edgeIndex, PxU32 triangleIndices[2]) const;
	PX_INLINE		PxU32						getEdgeTriangleIndices(PxU32 edgeIndex, PxU32 triangleIndices[2], PxU32 cell, PxU32 row, PxU32 column) const;
	PX_INLINE		void						getEdgeVertexIndices(PxU32 edgeIndex, PxU32& vertexIndex0, PxU32& vertexIndex1) const;
//	PX_INLINE		bool						isConvexEdge(PxU32 edgeIndex) const;
	PX_INLINE		bool						isConvexEdge(PxU32 edgeIndex, PxU32 cell, PxU32 row, PxU32 column) const;
	PX_FORCE_INLINE	bool						isConvexEdge(PxU32 edgeIndex) const
												{
													const PxU32 cell = edgeIndex / 3;
													const PxU32 row = cell / mData.columns;
													const PxU32 column = cell % mData.columns;
													return isConvexEdge(edgeIndex, cell, row, column);
												}

	PX_PHYSX_COMMON_API	PxU32					computeCellCoordinates(PxReal x, PxReal z, PxReal& fracX, PxReal& fracZ) const;

	PX_CUDA_CALLABLE PX_FORCE_INLINE	PxU32	getMin(PxReal x, PxU32 nb)	const
												{
													if(x<0.0f)
														return 0;
													if(x>PxReal(nb))
														return nb;

													const PxReal cx = Ps::floor(x);
													const PxU32 icx = PxU32(cx);
													return icx;
												}

	PX_CUDA_CALLABLE PX_FORCE_INLINE	PxU32	getMax(PxReal x, PxU32 nb)	const
												{
													if(x<0.0f)
														return 0;
													if(x>PxReal(nb))
														return nb;

													const PxReal cx = Ps::ceil(x);
													const PxU32 icx = PxU32(cx);
													return icx;
												}

	PX_CUDA_CALLABLE PX_FORCE_INLINE	PxU32	getMinRow(PxReal x)		const	{ return getMin(x, mData.rows-2);		}
	PX_CUDA_CALLABLE PX_FORCE_INLINE	PxU32	getMaxRow(PxReal x)		const	{ return getMax(x, mData.rows-1);		}
	PX_CUDA_CALLABLE PX_FORCE_INLINE	PxU32	getMinColumn(PxReal z)	const	{ return getMin(z, mData.columns-2);	}
	PX_CUDA_CALLABLE PX_FORCE_INLINE	PxU32	getMaxColumn(PxReal z)	const	{ return getMax(z, mData.columns-1);	}

	PX_CUDA_CALLABLE PX_INLINE			bool	isValidTriangle(PxU32 triangleIndex) const;
	PX_CUDA_CALLABLE PX_FORCE_INLINE	bool	isFirstTriangle(PxU32 triangleIndex) const	{ return ((triangleIndex & 0x1) == 0);	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE	PxU16	getTriangleMaterial(PxU32 triangleIndex) const
												{
													return isFirstTriangle(triangleIndex) ? getMaterialIndex0(triangleIndex >> 1) : getMaterialIndex1(triangleIndex >> 1);
												}

	PX_CUDA_CALLABLE PX_INLINE			void	getTriangleVertexIndices(PxU32 triangleIndex, PxU32& vertexIndex0, PxU32& vertexIndex1, PxU32& vertexIndex2) const;
	PX_CUDA_CALLABLE PX_INLINE			PxVec3	getTriangleNormalInternal(PxU32 triangleIndex) const;
	PX_INLINE							void	getTriangleAdjacencyIndices(PxU32 triangleIndex,PxU32 vertexIndex0, PxU32 vertexIndex1, PxU32 vertexIndex2, PxU32& adjacencyIndex0, PxU32& adjacencyIndex1, PxU32& adjacencyIndex2) const;

	PX_INLINE		PxVec3						getNormal_2(PxU32 vertexIndex, PxReal fracX, PxReal fracZ, PxReal xcoeff, PxReal ycoeff, PxReal zcoeff) const;
	PX_FORCE_INLINE PxVec3						getNormal_(PxReal x, PxReal z, PxReal xcoeff, PxReal ycoeff, PxReal zcoeff) const
												{
													PxReal fracX, fracZ;
													const PxU32 vertexIndex = computeCellCoordinates(x, z, fracX, fracZ);

													return getNormal_2(vertexIndex, fracX, fracZ, xcoeff, ycoeff, zcoeff);
												}

	PX_INLINE		PxU32						getTriangleIndex(PxReal x, PxReal z) const;
	PX_INLINE		PxU32						getTriangleIndex2(PxU32 cell, PxReal fracX, PxReal fracZ) const;
	PX_FORCE_INLINE	PxU16						getMaterial(PxReal x, PxReal z) const
												{
													return getTriangleMaterial(getTriangleIndex(x, z));
												}

	PX_FORCE_INLINE	PxReal						getMinHeight()					const	{ return mMinHeight; }
	PX_FORCE_INLINE	PxReal						getMaxHeight()					const	{ return mMaxHeight; }

	PX_FORCE_INLINE	const Gu::HeightFieldData&	getData()						const	{ return mData; }
	
	PX_CUDA_CALLABLE PX_FORCE_INLINE	void	getTriangleVertices(PxU32 triangleIndex, PxU32 row, PxU32 column, PxVec3& v0, PxVec3& v1, PxVec3& v2) const;

												// checks if current vertex is solid or not
					bool						isSolidVertex(PxU32 vertexIndex, PxU32 row, PxU32 coloumn, PxU16 holeMaterialIndex, bool& nbSolid) const;	

												// if precomputed bitmap define is used, the collision vertex information
												// is precomputed during create height field and stored as a bit in materialIndex1
	PX_PHYSX_COMMON_API		bool				isCollisionVertexPreca(PxU32 vertexIndex, PxU32 row, PxU32 column, PxU16 holeMaterialIndex) const;
	PX_FORCE_INLINE	bool						isCollisionVertex(PxU32 vertexIndex, PxU32, PxU32, PxU16) const
												{
													return getSample(vertexIndex).materialIndex1.isBitSet()!=0;
												}
					void						parseTrianglesForCollisionVertices(PxU16 holeMaterialIndex);					

	PX_FORCE_INLINE
	PX_CUDA_CALLABLE const PxHeightFieldSample&	getSample(PxU32 vertexIndex) const
												{
													PX_ASSERT(isValidVertex(vertexIndex));
													return mData.samples[vertexIndex];
												}

#ifdef __CUDACC__
	PX_CUDA_CALLABLE void		setSamplePtr(PxHeightFieldSample* s) { mData.samples = s; }
#endif

					Gu::HeightFieldData			mData;
					PxU32						mSampleStride;
					PxU32						mNbSamples;	// PT: added for platform conversion. Try to remove later.
					PxReal						mMinHeight;
					PxReal						mMaxHeight;
					PxU32						mModifyCount;
					// methods
	PX_PHYSX_COMMON_API void					releaseMemory();

	PX_PHYSX_COMMON_API virtual					~HeightField();

private:
					GuMeshFactory*				mMeshFactory;	// PT: changed to pointer for serialization
};

} // namespace Gu

PX_INLINE PxVec3 Gu::HeightField::getVertex(PxU32 vertexIndex) const
{
	const PxU32 row    = vertexIndex / mData.columns;
	const PxU32 column = vertexIndex % mData.columns;
//	return PxVec3(PxReal(row), getHeight(row * mData.columns + column), PxReal(column));
	return PxVec3(PxReal(row), getHeight(vertexIndex), PxReal(column));
}

// PT: only called from "isCollisionVertex", should move
PX_INLINE bool Gu::HeightField::isConvexVertex(PxU32 vertexIndex, PxU32 row, PxU32 column) const
{
#ifdef PX_HEIGHTFIELD_DEBUG
	PX_ASSERT(isValidVertex(vertexIndex));
#endif
	PX_ASSERT((vertexIndex / mData.columns)==row);
	PX_ASSERT((vertexIndex % mData.columns)==column);

//	PxReal h0 = PxReal(2) * getHeight(vertexIndex);
	PxI32 h0 = getSample(vertexIndex).height;
	h0 += h0;

	bool definedInX, definedInZ;
	PxI32 convexityX, convexityZ;

	if ((row > 0) &&  (row < mData.rows - 1))
	{
//		convexityX = h0 - getHeight(vertexIndex + mData.columns) - getHeight(vertexIndex - mData.columns);
		convexityX = h0 - getSample(vertexIndex + mData.columns).height - getSample(vertexIndex - mData.columns).height;
		definedInX = true;
	}
	else
	{
		convexityX = 0;
		definedInX = false;
	}

	if ((column > 0) &&  (column < mData.columns - 1))
	{
//		convexityZ = h0 - getHeight(vertexIndex + 1) - getHeight(vertexIndex - 1);
		convexityZ = h0 - getSample(vertexIndex + 1).height - getSample(vertexIndex - 1).height;
		definedInZ = true;
	}
	else
	{
		convexityZ = 0;
		definedInZ = false;
	}

	if(definedInX || definedInZ)
	{
		// PT: use XOR here
		// saddle points
/*		if ((convexityX > 0) && (convexityZ < 0)) 
			return false;		
		if ((convexityX < 0) && (convexityZ > 0)) 
			return false;*/
		if(((convexityX ^ convexityZ) & 0x80000000)==0)
			return false;

		const PxReal value = PxReal(convexityX + convexityZ);
		return value > mData.convexEdgeThreshold;
	}

	// this has to be one of the two corner vertices
	return true;
}

PX_INLINE bool Gu::HeightField::isValidEdge(PxU32 edgeIndex) const
{
	const PxU32 cell   = (edgeIndex / 3);
	const PxU32 row    = cell / mData.columns;
	const PxU32 column = cell % mData.columns;
//	switch (edgeIndex % 3)
	switch (edgeIndex - cell*3)
	{
		case 0:
			if (row > mData.rows - 1) return false;
			if (column >= mData.columns - 1) return false;
			break;
		case 1:
			if (row >= mData.rows - 1) return false;
			if (column >= mData.columns - 1) return false;
			break;
		case 2:
			if (row >= mData.rows - 1) return false;
			if (column > mData.columns - 1) return false;
			break;
	}
	return true;
}

PX_INLINE PxU32 Gu::HeightField::getEdgeTriangleIndices(PxU32 edgeIndex, PxU32 triangleIndices[2]) const
{
	const PxU32 cell   = edgeIndex / 3;
	const PxU32 row    = cell / mData.columns;
	const PxU32 column = cell % mData.columns;
	PxU32 count = 0;
//	switch (edgeIndex % 3)
	switch (edgeIndex - cell*3)
	{
		case 0:
			if (column < mData.columns - 1)
			{
				if (row > 0)
				{
/*					if (isZerothVertexShared(cell - mData.columns))
						triangleIndices[count++] = ((cell - mData.columns) << 1);
					else 
						triangleIndices[count++] = ((cell - mData.columns) << 1) + 1;*/
					triangleIndices[count++] = ((cell - mData.columns) << 1) + 1 - isZerothVertexShared(cell - mData.columns);
				}
				if (row < mData.rows - 1)
				{
/*					if (isZerothVertexShared(cell))
						triangleIndices[count++] = (cell << 1) + 1;
					else 
						triangleIndices[count++] = cell << 1;*/
					triangleIndices[count++] = (cell << 1) + isZerothVertexShared(cell);
				}
			}
			break;
		case 1:
			if ((row < mData.rows - 1) && (column < mData.columns - 1))
			{
				triangleIndices[count++] = cell << 1;
				triangleIndices[count++] = (cell << 1) + 1;
			}
			break;
		case 2:
			if (row < mData.rows - 1)
			{
				if (column > 0)
					triangleIndices[count++] = ((cell - 1) << 1) + 1;
				if (column < mData.columns - 1)
					triangleIndices[count++] = cell << 1;
			}
			break;
	}
	return count;
}

PX_INLINE PxU32 Gu::HeightField::getEdgeTriangleIndices(PxU32 edgeIndex, PxU32 triangleIndices[2], PxU32 cell, PxU32 row, PxU32 column) const
{
//	const PxU32 cell   = edgeIndex / 3;
//	const PxU32 row    = cell / mData.columns;
//	const PxU32 column = cell % mData.columns;
	PxU32 count = 0;
//	switch (edgeIndex % 3)
	switch (edgeIndex - cell*3)
	{
		case 0:
			if (column < mData.columns - 1)
			{
				if (row > 0)
				{
/*					if (isZerothVertexShared(cell - mData.columns))
						triangleIndices[count++] = ((cell - mData.columns) << 1);
					else 
						triangleIndices[count++] = ((cell - mData.columns) << 1) + 1;*/
					triangleIndices[count++] = ((cell - mData.columns) << 1) + 1 - isZerothVertexShared(cell - mData.columns);
				}
				if (row < mData.rows - 1)
				{
/*					if (isZerothVertexShared(cell))
						triangleIndices[count++] = (cell << 1) + 1;
					else 
						triangleIndices[count++] = cell << 1;*/
					triangleIndices[count++] = (cell << 1) + isZerothVertexShared(cell);
				}
			}
			break;
		case 1:
			if ((row < mData.rows - 1) && (column < mData.columns - 1))
			{
				triangleIndices[count++] = cell << 1;
				triangleIndices[count++] = (cell << 1) + 1;
			}
			break;
		case 2:
			if (row < mData.rows - 1)
			{
				if (column > 0)
					triangleIndices[count++] = ((cell - 1) << 1) + 1;
				if (column < mData.columns - 1)
					triangleIndices[count++] = cell << 1;
			}
			break;
	}
	return count;
}

PX_INLINE void Gu::HeightField::getEdgeVertexIndices(PxU32 edgeIndex, PxU32& vertexIndex0, PxU32& vertexIndex1) const
{
	const PxU32 cell = edgeIndex / 3;
//	switch (edgeIndex % 3)
	switch (edgeIndex - cell*3)
	{
		case 0:
			vertexIndex0 = cell;
			vertexIndex1 = cell + 1;
			break;
		case 1:
			{
/*			if (isZerothVertexShared(cell))
			{
				vertexIndex0 = cell;
				vertexIndex1 = cell + mData.columns + 1;
			}
			else
			{
				vertexIndex0 = cell + 1;
				vertexIndex1 = cell + mData.columns;
			}*/
			const bool b = isZerothVertexShared(cell);
			vertexIndex0 = cell + 1 - b;
			vertexIndex1 = cell + mData.columns + b;
			}
			break;
		case 2:
			vertexIndex0 = cell;
			vertexIndex1 = cell + mData.columns;
			break;
	}
}

PX_INLINE bool Gu::HeightField::isConvexEdge(PxU32 edgeIndex, PxU32 cell, PxU32 row, PxU32 column) const
{
//	const PxU32 cell = edgeIndex / 3;
	PX_ASSERT(cell == edgeIndex / 3);

//	const PxU32 row = cell / mData.columns;
	PX_ASSERT(row == cell / mData.columns);
	if (row > mData.rows-2) return false;

//	const PxU32 column = cell % mData.columns;
	PX_ASSERT(column == cell % mData.columns);
	if (column > mData.columns-2) return false;

//	PxReal h0 = 0, h1 = 0, h2 = 0, h3 = 0;
//	PxReal convexity = 0;
	PxI32 h0 = 0, h1 = 0, h2 = 0, h3 = 0;
	PxI32 convexity = 0;

//	switch (edgeIndex % 3)
	switch (edgeIndex - cell*3)
	{
		case 0:
			{
			if (row < 1) return false;
/*			if(isZerothVertexShared(cell - mData.columns)) 
			{
				//      <------ COL  
				//       +----+  0  R
				//       |   /  /#  O
				//       |  /  / #  W
				//       | /  /  #  |
				//       |/  /   #  |
				//       +  +====1  |
				//                  |
				//                  |
				//                  |
				//                  |
				//                  |
				//                  |
				//                  V
				//      
//				h0 = getHeight(cell - mData.columns);
//				h1 = getHeight(cell);
				h0 = getSample(cell - mData.columns).height;
				h1 = getSample(cell).height;
			}
			else
			{
				//      <------ COL  
				//       0  +----+  R
				//       #\  \   |  O
				//       # \  \  |  W
				//       #  \  \ |  |
				//       #   \  \|  |
				//       1====+  +  |
				//                  |
				//                  |
				//                  |
				//                  |
				//                  |
				//                  |
				//                  V
				//      
//				h0 = getHeight(cell - mData.columns + 1);
//				h1 = getHeight(cell + 1);
				h0 = getSample(cell - mData.columns + 1).height;
				h1 = getSample(cell + 1).height;
			}*/
			const bool b0 = !isZerothVertexShared(cell - mData.columns);
			h0 = getSample(cell - mData.columns + b0).height;
			h1 = getSample(cell + b0).height;

/*			if(isZerothVertexShared(cell)) 
			{
				//      <------ COL  
				//                  R
				//                  O
				//                  W
				//                  |
				//                  |
				//                  |
				//       2====+  0  |
				//       #   /  /|  |
				//       #  /  / |  |
				//       # /  /  |  |
				//       #/  /   |  |
				//       3  +----+  |
				//                  V
				//      
//				h2 = getHeight(cell + 1);
//				h3 = getHeight(cell + mData.columns + 1);
				h2 = getSample(cell + 1).height;
				h3 = getSample(cell + mData.columns + 1).height;
			}
			else
			{
				//      <------ COL  
				//                  R
				//                  O
				//                  W
				//                  |
				//                  |
				//                  |
				//       +  +====2  |
				//       |\  \   #  |
				//       | \  \  #  |
				//       |  \  \ #  |
				//       |   \  \#  |
				//       +----+  3  |
				//                  V
				//      
//				h2 = getHeight(cell);
//				h3 = getHeight(cell + mData.columns);
				h2 = getSample(cell).height;
				h3 = getSample(cell + mData.columns).height;
			}*/
			const bool b1 = isZerothVertexShared(cell);
			h2 = getSample(cell + b1).height;
			h3 = getSample(cell + mData.columns + b1).height;

			//convex = (h3-h2) < (h1-h0);
			convexity = (h1-h0) - (h3-h2);
			}
			break;
		case 1:
//			h0 = getHeight(cell);
//			h1 = getHeight(cell + 1);
//			h2 = getHeight(cell + mData.columns);
//			h3 = getHeight(cell + mData.columns + 1);
			h0 = getSample(cell).height;
			h1 = getSample(cell + 1).height;
			h2 = getSample(cell + mData.columns).height;
			h3 = getSample(cell + mData.columns + 1).height;
			if (isZerothVertexShared(cell))
				//convex = (h0 + h3) > (h1 + h2);
				convexity = (h0 + h3) - (h1 + h2);
			else 
				//convex = (h2 + h1) > (h0 + h3);
				convexity = (h2 + h1) - (h0 + h3);
			break;
		case 2:
			{
			if (column < 1) return false;
/*			if(isZerothVertexShared(cell-1)) 
			{
				//      <-------------- COL  
				//                1====0  + R
				//                +   /  /| O
				//                +  /  / | W
				//                + /  /  | |
				//                +/  /   | |
				//                +  +----+ V
				//      
//				h0 = getHeight(cell - 1);
//				h1 = getHeight(cell);
				h0 = getSample(cell - 1).height;
				h1 = getSample(cell).height;
			}
			else
			{
				//      <-------------- COL  
				//                +  +----+ R
				//                +\  \   | O
				//                + \  \  | W
				//                +  \  \ | |
				//                +   \  \| |
				//                1====0  + V
				//      
//				h0 = getHeight(cell - 1 + mData.columns);
//				h1 = getHeight(cell + mData.columns);
				h0 = getSample(cell - 1 + mData.columns).height;
				h1 = getSample(cell + mData.columns).height;
			}*/
			const PxU32 offset0 = isZerothVertexShared(cell-1) ? 0 : mData.columns;
			h0 = getSample(cell - 1 + offset0).height;
			h1 = getSample(cell + offset0).height;

/*			if(isZerothVertexShared(cell)) 
			{
				//      <-------------- COL  
				//       +----+  +          R
				//       |   /  /+          O
				//       |  /  / +          W
				//       | /  /  +          |
				//       |/  /   +          |
				//       +  3====2          V
				//      
//				h2 = getHeight(cell + mData.columns);
//				h3 = getHeight(cell + mData.columns + 1);
				h2 = getSample(cell + mData.columns).height;
				h3 = getSample(cell + mData.columns + 1).height;
			}
			else
			{
				//      <-------------- COL  
				//       +  3====2          R
				//       |\  \   +          O
				//       | \  \  +          W
				//       |  \  \ +          |
				//       |   \  \+          |
				//       +----+  +          V
				//      
//				h2 = getHeight(cell);
//				h3 = getHeight(cell + 1);
				h2 = getSample(cell).height;
				h3 = getSample(cell + 1).height;
			}*/
			const PxU32 offset1 = isZerothVertexShared(cell) ? mData.columns : 0;
			h2 = getSample(cell + offset1).height;
			h3 = getSample(cell + offset1 + 1).height;

			//convex = (h3-h2) < (h1-h0);
			convexity = (h1-h0) - (h3-h2);
			}
			break;
	}

	const PxI32 threshold = PxI32(mData.convexEdgeThreshold);
	return convexity > threshold;
}

PX_INLINE bool Gu::HeightField::isValidTriangle(PxU32 triangleIndex) const
{
	const PxU32 cell = triangleIndex >> 1;
	const PxU32 row  = cell / mData.columns;
	if (row >= (mData.rows - 1)) return false;
	const PxU32 column = cell % mData.columns;
	if (column >= (mData.columns - 1)) return false;
	return true;
}

PX_INLINE void Gu::HeightField::getTriangleVertexIndices(PxU32 triangleIndex, PxU32& vertexIndex0, PxU32& vertexIndex1, PxU32& vertexIndex2) const
{
	const PxU32 cell = triangleIndex >> 1;
	if (isZerothVertexShared(cell))
	{
		//      <---- COL  
		//      0----2  1 R
		//      | 1 /  /| O
		//      |  /  / | W
		//      | /  /  | |
		//      |/  / 0 | |
		//      1  2----0 V
		//      
		if (isFirstTriangle(triangleIndex))
		{
			vertexIndex0 = cell + mData.columns;
			vertexIndex1 = cell;
			vertexIndex2 = cell + mData.columns + 1;
		}
		else
		{
			vertexIndex0 = cell + 1;
			vertexIndex1 = cell + mData.columns + 1;
			vertexIndex2 = cell;
		}
	}
	else
	{
		//      <---- COL  
		//      2  1----0 R
		//      |\  \ 0 | O
		//      | \  \  | W
		//      |  \  \ | |
		//      | 1 \  \| |
		//      0----1  2 V
		//                   
		if (isFirstTriangle(triangleIndex))
		{
			vertexIndex0 = cell;
			vertexIndex1 = cell + 1;
			vertexIndex2 = cell + mData.columns;
		}
		else
		{
			vertexIndex0 = cell + mData.columns + 1;
			vertexIndex1 = cell + mData.columns;
			vertexIndex2 = cell + 1;
		}
	}
}

PX_INLINE void Gu::HeightField::getTriangleAdjacencyIndices(PxU32 triangleIndex, PxU32 vertexIndex0, PxU32 vertexIndex1, PxU32 vertexIndex2, PxU32& adjacencyIndex0, PxU32& adjacencyIndex1, PxU32& adjacencyIndex2) const
{
	PX_UNUSED(vertexIndex0);
	PX_UNUSED(vertexIndex1);
	PX_UNUSED(vertexIndex2);

	const PxU32 cell = triangleIndex >> 1;
	if (isZerothVertexShared(cell))
	{
		//      <---- COL  
		//      0----2  1 R
		//      | 1 /  /| O
		//      |  /  / | W
		//      | /  /  | |
		//      |/  / 0 | |
		//      1  2----0 V
		//      
		if (isFirstTriangle(triangleIndex))
		{
			adjacencyIndex0 = 0xFFFFFFFF;
			adjacencyIndex1 = triangleIndex + 1;
			adjacencyIndex2 = 0xFFFFFFFF;

			if((cell % (mData.columns) != 0))
			{
				adjacencyIndex0 = triangleIndex - 1;
			}

			if((cell / mData.columns != mData.rows - 2))
			{
				const PxU32 tMod = isZerothVertexShared(cell + mData.columns) ? 1u : 0u;
				adjacencyIndex2 = ((cell + mData.columns) * 2) + tMod;
			}
		}
		else
		{
			adjacencyIndex0 = 0xFFFFFFFF;
			adjacencyIndex1 = triangleIndex - 1;
			adjacencyIndex2 = 0xFFFFFFFF;

			if(cell % (mData.columns) < (mData.columns - 2))
			{
				adjacencyIndex0 = triangleIndex + 1;
			}

			if(cell >= mData.columns - 1)
			{
				const PxU32 tMod = isZerothVertexShared(cell - mData.columns) ? 0u : 1u;
				adjacencyIndex2 = ((cell - mData.columns) * 2) + tMod;
			}
		}
	}
	else
	{
		//      <---- COL  
		//      2  1----0 R
		//      |\  \ 0 | O
		//      | \  \  | W
		//      |  \  \ | |
		//      | 1 \  \| |
		//      0----1  2 V
		//                   
		if (isFirstTriangle(triangleIndex))
		{
			adjacencyIndex0 = 0xFFFFFFFF;
			adjacencyIndex1 = triangleIndex + 1;
			adjacencyIndex2 = 0xFFFFFFFF;

			if(cell >= mData.columns - 1)
			{
				const PxU32 tMod = isZerothVertexShared(cell - mData.columns) ? 0u : 1u;
				adjacencyIndex0 = ((cell - (mData.columns)) * 2) + tMod;
			}

			if((cell % (mData.columns) != 0))
			{
				adjacencyIndex2 = triangleIndex - 1;
			}
		}
		else
		{
			adjacencyIndex0 = 0xFFFFFFFF;
			adjacencyIndex1 = triangleIndex - 1;
			adjacencyIndex2 = 0xFFFFFFFF;

			if((cell / mData.columns != mData.rows - 2))
			{
				const PxU32 tMod = isZerothVertexShared(cell + mData.columns) ? 1u : 0u;
				adjacencyIndex0 = (cell + (mData.columns)) * 2 + tMod;
			}

			if(cell % (mData.columns) < (mData.columns - 2))
			{
				adjacencyIndex2 = triangleIndex + 1;
			}
		}
	}
}

PX_INLINE PxVec3 Gu::HeightField::getTriangleNormalInternal(PxU32 triangleIndex) const
{
	PxU32 v0, v1, v2;
	getTriangleVertexIndices(triangleIndex, v0, v1, v2); 

//	const PxReal h0 = getHeight(v0);
//	const PxReal h1 = getHeight(v1);
//	const PxReal h2 = getHeight(v2);
	const PxI32 h0 = getSample(v0).height;
	const PxI32 h1 = getSample(v1).height;
	const PxI32 h2 = getSample(v2).height;

	const float thickness = 0.0f;
	const PxReal coeff = physx::intrinsics::fsel(thickness, -1.0f, 1.0f);

//	PxVec3 n(0,1,0);
	const PxU32 cell = triangleIndex >> 1;
	if (isZerothVertexShared(cell))
	{
		//      <---- COL  
		//      0----2  1 R
		//      | 1 /  /| O
		//      |  /  / | W
		//      | /  /  | |
		//      |/  / 0 | |
		//      1  2----0 V
		//      
		if (isFirstTriangle(triangleIndex))
		{
//			n.x = -(h0-h1);
//			n.z = -(h2-h0);
			return PxVec3(coeff*PxReal(h1-h0), coeff, coeff*PxReal(h0-h2));
		}
		else
		{
//			n.x = -(h1-h0);
//			n.z = -(h0-h2);
			return PxVec3(coeff*PxReal(h0-h1), coeff, coeff*PxReal(h2-h0));
		}
	}
	else
	{
		//      <---- COL  
		//      2  1----0 R
		//      |\  \ 0 | O
		//      | \  \  | W
		//      |  \  \ | |
		//      | 1 \  \| |
		//      0----1  2 V
		//                   
		if (isFirstTriangle(triangleIndex))
		{
//			n.x = -(h2-h0);
//			n.z = -(h1-h0);
			return PxVec3(coeff*PxReal(h0-h2), coeff, coeff*PxReal(h0-h1));
		}
		else
		{
//			n.x = -(h0-h2);
//			n.z = -(h0-h1);
			return PxVec3(coeff*PxReal(h2-h0), coeff, coeff*PxReal(h1-h0));
		}
	}
//	return n;
}

PX_INLINE PxReal Gu::HeightField::getHeightInternal2(PxU32 vertexIndex, PxReal fracX, PxReal fracZ) const
{
	if (isZerothVertexShared(vertexIndex))
	{
		//    <----Z---+
		//      +----+ | 
		//      |   /| |
		//      |  / | X
		//      | /  | |
		//      |/   | |
		//      +----+ |
		//             V
		const PxReal h0 = getHeight(vertexIndex);
		const PxReal h2 = getHeight(vertexIndex + mData.columns + 1);
		if (fracZ > fracX)
		{
			//    <----Z---+
			//      1----0 | 
			//      |   /  |
			//      |  /   X
			//      | /    |
			//      |/     |
			//      2      |
			//             V
			const PxReal h1 = getHeight(vertexIndex + 1);
			return h0 + fracZ*(h1-h0) + fracX*(h2-h1);
		}
		else
		{
			//    <----Z---+
			//           0 | 
			//          /| |
			//         / | X
			//        /  | |
			//       /   | |
			//      2----1 |
			//             V
			const PxReal h1 = getHeight(vertexIndex + mData.columns);
			return h0 + fracX*(h1-h0) + fracZ*(h2-h1);
		}
	}
	else
	{
		//    <----Z---+
		//      +----+ | 
		//      |\   | |
		//      | \  | X
		//      |  \ | |
		//      |   \| |
		//      +----+ |
		//             V
		const PxReal h2 = getHeight(vertexIndex + mData.columns);
		const PxReal h1 = getHeight(vertexIndex + 1);
		if (fracX + fracZ < 1.0f)
		{
			//    <----Z---+
			//      1----0 | 
			//       \   | |
			//        \  | X
			//         \ | |
			//          \| |
			//           2 |
			//             V
			const PxReal h0 = getHeight(vertexIndex);
			return h0 + fracZ*(h1-h0) + fracX*(h2-h0);
		}
		else
		{
			//    <----Z---+
			//      1      | 
			//      |\     |
			//      | \    X
			//      |  \   |
			//      |   \  |
			//      0----2 |
			//             V
			//
			// Note that we need to flip fracX and fracZ since we are moving the origin
			const PxReal h0 = getHeight(vertexIndex + mData.columns + 1);
			return h0 + (1.0f - fracZ)*(h2-h0) + (1.0f - fracX)*(h1-h0);
		}
	}
}

PX_INLINE PxVec3 Gu::HeightField::getNormal_2(PxU32 vertexIndex, PxReal fracX, PxReal fracZ, PxReal xcoeff, PxReal ycoeff, PxReal zcoeff) const
{
	PxVec3 normal;
	if (isZerothVertexShared(vertexIndex))
	{
		//    <----Z---+
		//      +----+ | 
		//      |   /| |
		//      |  / | X
		//      | /  | |
		//      |/   | |
		//      +----+ |
		//             V
//		const PxReal h0 = getHeight(vertexIndex);
//		const PxReal h2 = getHeight(vertexIndex + mData.columns + 1);
		const PxI32 ih0 = getSample(vertexIndex).height;
		const PxI32 ih2 = getSample(vertexIndex + mData.columns + 1).height;
		if (fracZ >= fracX)
		{
			//    <----Z---+
			//      1----0 | 
			//      |   /  |
			//      |  /   X
			//      | /    |
			//      |/     |
			//      2      |
			//             V
//			const PxReal h0 = getHeight(vertexIndex);
//			const PxReal h1 = getHeight(vertexIndex + 1);
//			const PxReal h2 = getHeight(vertexIndex + mData.columns + 1);
//			normal.set(-(h2-h1), 1.0f, -(h1-h0));
			const PxI32 ih1 = getSample(vertexIndex + 1).height;
			normal = PxVec3(PxReal(ih1 - ih2)*xcoeff, ycoeff, PxReal(ih0 - ih1)*zcoeff);
		}
		else
		{
			//    <----Z---+
			//           0 | 
			//          /| |
			//         / | X
			//        /  | |
			//       /   | |
			//      2----1 |
			//             V
//			const PxReal h0 = getHeight(vertexIndex);
//			const PxReal h1 = getHeight(vertexIndex + mData.columns);
//			const PxReal h2 = getHeight(vertexIndex + mData.columns + 1);
//			normal.set(-(h1-h0), 1.0f, -(h2-h1));
			const PxI32 ih1 = getSample(vertexIndex + mData.columns).height;
			normal = PxVec3(PxReal(ih0 - ih1)*xcoeff, ycoeff, PxReal(ih1 - ih2)*zcoeff);
		}
	}
	else
	{
		//    <----Z---+
		//      +----+ | 
		//      |\   | |
		//      | \  | X
		//      |  \ | |
		//      |   \| |
		//      +----+ |
		//             V
		const PxI32 ih1 = getSample(vertexIndex + 1).height;
		const PxI32 ih2 = getSample(vertexIndex + mData.columns).height;
		if (fracX + fracZ <= PxReal(1))
		{
			//    <----Z---+
			//      1----0 | 
			//       \   | |
			//        \  | X
			//         \ | |
			//          \| |
			//           2 |
			//             V
//			const PxReal h0 = getHeight(vertexIndex);
//			const PxReal h1 = getHeight(vertexIndex + 1);
//			const PxReal h2 = getHeight(vertexIndex + mData.columns);
//			normal.set(-(h2-h0), 1.0f, -(h1-h0));
			const PxI32 ih0 = getSample(vertexIndex).height;
//			const PxI32 ih1 = getSample(vertexIndex + 1).height;
//			const PxI32 ih2 = getSample(vertexIndex + mData.columns).height;
			normal = PxVec3(PxReal(ih0 - ih2)*xcoeff, ycoeff, PxReal(ih0 - ih1)*zcoeff);
		}
		else
		{
			//    <----Z---+
			//      2      | 
			//      |\     |
			//      | \    X
			//      |  \   |
			//      |   \  |
			//      0----1 |
			//             V
			//
			// Note that we need to flip fracX and fracZ since we are moving the origin
//			const PxReal h2 = getHeight(vertexIndex + 1);
//			const PxReal h1 = getHeight(vertexIndex + mData.columns);
//			const PxReal h0 = getHeight(vertexIndex + mData.columns + 1);
//			normal.set(-(h0-h2), 1.0f, -(h0-h1));
//			const PxI32 ih2 = getSample(vertexIndex + 1).height;
//			const PxI32 ih1 = getSample(vertexIndex + mData.columns).height;
			const PxI32 ih0 = getSample(vertexIndex + mData.columns + 1).height;
//			normal.set(PxReal(ih2 - ih0), 1.0f, PxReal(ih1b - ih0));
			normal = PxVec3(PxReal(ih1 - ih0)*xcoeff, ycoeff, PxReal(ih2 - ih0)*zcoeff);
		}
	}
	return normal;
}

PX_INLINE PxU32 Gu::HeightField::getTriangleIndex2(PxU32 cell, PxReal fracX, PxReal fracZ) const
{
	if (isZerothVertexShared(cell))
		return (fracZ > fracX) ? (cell << 1) + 1 : (cell << 1);
	else
		return (fracX + fracZ > 1) ? (cell << 1) + 1 : (cell << 1);
}

PX_INLINE PxU32 Gu::HeightField::getTriangleIndex(PxReal x, PxReal z) const
{
	PxReal fracX, fracZ;
	const PxU32 cell = computeCellCoordinates(x, z, fracX, fracZ);

	return getTriangleIndex2(cell, fracX, fracZ);
}

PX_FORCE_INLINE void Gu::HeightField::getTriangleVertices(PxU32 triangleIndex, PxU32 row, PxU32 column, PxVec3& v0, PxVec3& v1, PxVec3& v2) const
{
	PxU32 cell = triangleIndex >> 1;
	PX_ASSERT(row * getNbColumnsFast() + column == cell);

	PxReal h0 = getHeight(cell);
	PxReal h1 = getHeight(cell + 1);
	PxReal h2 = getHeight(cell + getNbColumnsFast());
	PxReal h3 = getHeight(cell + getNbColumnsFast() + 1);

	if (isFirstTriangle(triangleIndex))
	{
		if (isZerothVertexShared(cell))
		{
			//      <---- COL  
			//              1 R
			//             /| O
			//            / | W
			//           /  | |
			//          / 0 | |
			//         2----0 V
			//
			v0 = PxVec3(PxReal(row + 1),	h2,	PxReal(column    ));
			v1 = PxVec3(PxReal(row    ),	h0,	PxReal(column    ));
			v2 = PxVec3(PxReal(row + 1),	h3,	PxReal(column + 1));
		}
		else
		{
			//      <---- COL  
			//         1----0 R
			//          \ 0 | O
			//           \  | W
			//            \ | |
			//             \| |
			//              2 V
			//
			v0 = PxVec3(PxReal(row    ),	h0,	PxReal(column    ));
			v1 = PxVec3(PxReal(row    ),	h1,	PxReal(column + 1));
			v2 = PxVec3(PxReal(row + 1),	h2,	PxReal(column    ));
		}
	}
	else
	{
		if (isZerothVertexShared(cell))
		{
			//      <---- COL  
			//      0----2    R
			//      | 1 /     O
			//      |  /      W
			//      | /       |
			//      |/        |
			//      1         V
			//
			v0 = PxVec3(PxReal(row    ),	h1, PxReal(column + 1));
			v1 = PxVec3(PxReal(row + 1),	h3, PxReal(column + 1));
			v2 = PxVec3(PxReal(row    ),	h0,	PxReal(column    ));
		}
		else
		{
			//      <---- COL  
			//      2         R
			//      |\        O
			//      | \       W
			//      |  \      |
			//      | 1 \     |
			//      0----1    V
			//
			v0 = PxVec3(PxReal(row + 1),	h3,	PxReal(column + 1));
			v1 = PxVec3(PxReal(row + 1),	h2,	PxReal(column    ));
			v2 = PxVec3(PxReal(row    ),	h1,	PxReal(column + 1));
		}
	}
}

struct EdgeData
{
	PxU32	edgeIndex;
	PxU32	cell;
	PxU32	row;
	PxU32	column;
};
PX_PHYSX_COMMON_API PxU32 getVertexEdgeIndices(const Gu::HeightField& heightfield, PxU32 vertexIndex, PxU32 row, PxU32 column, EdgeData edgeIndices[8]);
PX_PHYSX_COMMON_API PxU32 getEdgeTriangleIndices(const Gu::HeightField& heightfield, const EdgeData& edgeData, PxU32* PX_RESTRICT triangleIndices);

}

#endif
