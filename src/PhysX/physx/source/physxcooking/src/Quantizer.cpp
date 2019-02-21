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

#include "Quantizer.h"

#include "foundation/PxVec3.h"
#include "foundation/PxBounds3.h"

#include "PsUserAllocated.h"
#include "PsAllocator.h"
#include "PsArray.h"

#include "CmPhysXCommon.h"

using namespace physx;

PxU32	kmeans_cluster3d(const PxVec3 *input,				// an array of input 3d data points.
		PxU32 inputSize,				// the number of input data points.
		PxU32 clumpCount,				// the number of clumps you wish to product.
		PxVec3	*outputClusters,		// The output array of clumps 3d vectors, should be at least 'clumpCount' in size.
		PxU32	*outputIndices,			// A set of indices which remaps the input vertices to clumps; should be at least 'inputSize'
		float errorThreshold=0.01f,	// The error threshold to converge towards before giving up.
		float collapseDistance=0.01f); // distance so small it is not worth bothering to create a new clump.

template <class Vec,class Type >
PxU32	kmeans_cluster(const Vec *input,
						   PxU32 inputCount,
						   PxU32 clumpCount,
						   Vec *clusters,
						   PxU32 *outputIndices,
						   Type threshold, // controls how long it works to converge towards a least errors solution.
						   Type collapseDistance) // distance between clumps to consider them to be essentially equal.
{
	PxU32 convergeCount = 64; // maximum number of iterations attempting to converge to a solution..
	PxU32* counts = reinterpret_cast<PxU32*> (PX_ALLOC_TEMP(sizeof(PxU32)*clumpCount, "PxU32"));
	Type error=0;
	if ( inputCount <= clumpCount ) // if the number of input points is less than our clumping size, just return the input points.
	{
		clumpCount = inputCount;
		for (PxU32 i=0; i<inputCount; i++)
		{
			if ( outputIndices )
			{
				outputIndices[i] = i;
			}
			clusters[i] = input[i];
			counts[i] = 1;
		}
	}
	else
	{		
		PxVec3* centroids = reinterpret_cast<PxVec3*> (PX_ALLOC_TEMP(sizeof(PxVec3)*clumpCount, "PxVec3"));

		// Take a sampling of the input points as initial centroid estimates.
		for (PxU32 i=0; i<clumpCount; i++)
		{
			PxU32 index = (i*inputCount)/clumpCount;
			PX_ASSERT( index < inputCount );
			clusters[i] = input[index];
		}

		// Here is the main convergence loop
		Type old_error = FLT_MAX;	// old and initial error estimates are max Type
		error = FLT_MAX;
		do
		{
			old_error = error;	// preserve the old error
			// reset the counts and centroids to current cluster location
			for (PxU32 i=0; i<clumpCount; i++)
			{
				counts[i] = 0;
				centroids[i] = PxVec3(PxZero);
			}
			error = 0;
			// For each input data point, figure out which cluster it is closest too and add it to that cluster.
			for (PxU32 i=0; i<inputCount; i++)
			{
				Type min_distance = FLT_MAX;
				// find the nearest clump to this point.
				for (PxU32 j=0; j<clumpCount; j++)
				{
					const Type distance = (input[i] - clusters[j]).magnitudeSquared();
					if ( distance < min_distance )
					{
						min_distance = distance;
						outputIndices[i] = j; // save which clump this point indexes
					}
				}
				const PxU32 index = outputIndices[i]; // which clump was nearest to this point.
				centroids[index]+=input[i];
				counts[index]++;	// increment the counter indicating how many points are in this clump.
				error+=min_distance; // save the error accumulation
			}
			// Now, for each clump, compute the mean and store the result.
			for (PxU32 i=0; i<clumpCount; i++)
			{
				if ( counts[i] ) // if this clump got any points added to it...
				{
					const Type recip = 1.0f / Type(counts[i]);	// compute the average (center of those points)
					centroids[i]*=recip;	// compute the average center of the points in this clump.
					clusters[i] = centroids[i]; // store it as the new cluster.
				}
			}
			// decrement the convergence counter and bail if it is taking too long to converge to a solution.
			convergeCount--;
			if (convergeCount == 0 )
			{
				break;
			}
			if ( error < threshold ) // early exit if our first guess is already good enough (if all input points are the same)
				break;
		} while ( PxAbs(error - old_error) > threshold ); // keep going until the error is reduced by this threshold amount.

		PX_FREE(centroids);
	}

	// ok..now we prune the clumps if necessary.
	// The rules are; first, if a clump has no 'counts' then we prune it as it's unused.
	// The second, is if the centroid of this clump is essentially  the same (based on the distance tolerance)
	// as an existing clump, then it is pruned and all indices which used to point to it, now point to the one
	// it is closest too.
	PxU32 outCount = 0; // number of clumps output after pruning performed.
	Type d2 = collapseDistance*collapseDistance; // squared collapse distance.
	for (PxU32 i=0; i<clumpCount; i++)
	{
		if ( counts[i] == 0 ) // if no points ended up in this clump, eliminate it.
			continue;
		// see if this clump is too close to any already accepted clump.
		bool add = true;
		PxU32 remapIndex = outCount; // by default this clump will be remapped to its current index.
		for (PxU32 j=0; j<outCount; j++)
		{
			Type distance = (clusters[i] - clusters[j]).magnitudeSquared();
			if ( distance < d2 )
			{
				remapIndex = j;
				add = false; // we do not add this clump
				break;
			}
		}
		// If we have fewer output clumps than input clumps so far, then we need to remap the old indices to the new ones.
		if ( outputIndices )
		{
			if ( outCount != i || !add ) // we need to remap indices!  everything that was index 'i' now needs to be remapped to 'outCount'
			{
				for (PxU32 j=0; j<inputCount; j++)
				{
					if ( outputIndices[j] == i )
					{
						outputIndices[j] = remapIndex; //
					}
				}
			}
		}
		if ( add )
		{
			clusters[outCount] = clusters[i];
			outCount++;
		}
	}
	PX_FREE(counts);
	clumpCount = outCount;
	return clumpCount;
}

PxU32	kmeans_cluster3d(const PxVec3 *input,				// an array of input 3d data points.
							 PxU32 inputSize,				// the number of input data points.
							 PxU32 clumpCount,				// the number of clumps you wish to produce
							 PxVec3	*outputClusters,		// The output array of clumps 3d vectors, should be at least 'clumpCount' in size.
							 PxU32	*outputIndices,			// A set of indices which remaps the input vertices to clumps; should be at least 'inputSize'
							 float errorThreshold,	// The error threshold to converge towards before giving up.
							 float collapseDistance) // distance so small it is not worth bothering to create a new clump.
{
	return kmeans_cluster< PxVec3, float >(input, inputSize, clumpCount, outputClusters, outputIndices, errorThreshold, collapseDistance);
}

class QuantizerImpl : public Quantizer, public Ps::UserAllocated
{
public:
	QuantizerImpl(void)
	{
		mScale = PxVec3(1.0f, 1.0f, 1.0f);
		mCenter = PxVec3(0.0f, 0.0f, 0.0f);
	}

	// Use the k-means quantizer, similar results, but much slower.
	virtual const PxVec3 * kmeansQuantize3D(PxU32 vcount,
		const PxVec3 *vertices,
		PxU32 stride,
		bool denormalizeResults,
		PxU32 maxVertices,
		PxU32 &outVertsCount)
	{
		const PxVec3 *ret = NULL;
		outVertsCount = 0;
		mNormalizedInput.clear();
		mQuantizedOutput.clear();

		if ( vcount > 0 )
		{
			normalizeInput(vcount,vertices, stride);

			PxVec3* quantizedOutput = reinterpret_cast<PxVec3*> (PX_ALLOC_TEMP(sizeof(PxVec3)*vcount, "PxVec3"));
			PxU32* quantizedIndices = reinterpret_cast<PxU32*> (PX_ALLOC_TEMP(sizeof(PxU32)*vcount, "PxU32"));
			outVertsCount = kmeans_cluster3d(&mNormalizedInput[0], vcount, maxVertices, quantizedOutput, quantizedIndices, 0.01f, 0.0001f );
			if ( outVertsCount > 0 )
			{
				if ( denormalizeResults )
				{
					for (PxU32 i=0; i<outVertsCount; i++)
					{
						PxVec3 v( quantizedOutput[i] );
						v = v.multiply(mScale) + mCenter;
						mQuantizedOutput.pushBack(v);
					}
				}
				else
				{
					for (PxU32 i=0; i<outVertsCount; i++)
					{
						const PxVec3& v( quantizedOutput[i] );
						mQuantizedOutput.pushBack(v);
					}
				}
				ret = &mQuantizedOutput[0];
			}
			PX_FREE(quantizedOutput);
			PX_FREE(quantizedIndices);
		}
		return ret;
	}

	virtual void release(void)
	{
		PX_DELETE(this);
	}

	virtual const PxVec3& getDenormalizeScale(void) const 
	{
		return mScale;
	}

	virtual const PxVec3& getDenormalizeCenter(void) const
	{
		return mCenter;
	}



private:

	void normalizeInput(PxU32 vcount,const PxVec3 *vertices, PxU32 stride)
	{
		const char* vtx = reinterpret_cast<const char *> (vertices);
		mNormalizedInput.clear();
		mQuantizedOutput.clear();
		PxBounds3 bounds;
		bounds.setEmpty();
		for (PxU32 i=0; i<vcount; i++)
		{
			const PxVec3& v = *reinterpret_cast<const PxVec3 *> (vtx);
			vtx += stride;

			bounds.include(v);
		}

		mCenter = bounds.getCenter();

		PxVec3 dim = bounds.getDimensions();
		dim *= 1.001f;
		mScale = dim*0.5f;

		for (PxU32 i = 0; i < 3; i++)
		{
			if(dim[i] == 0)
				mScale[i] = 1.0f;
		}

		PxVec3 recip;
		recip.x = 1.0f / mScale.x;
		recip.y = 1.0f / mScale.y;
		recip.z = 1.0f / mScale.z;

		vtx = reinterpret_cast<const char *> (vertices);
		for (PxU32 i=0; i<vcount; i++)
		{
			PxVec3 v = *reinterpret_cast<const PxVec3 *> (vtx);
			vtx += stride;			

			v = (v - mCenter).multiply(recip);

			mNormalizedInput.pushBack(v);
		}
	}

	virtual ~QuantizerImpl(void)
	{

	}

	private:
		PxVec3					mScale;
		PxVec3					mCenter;
		Ps::Array<PxVec3>		mNormalizedInput;
		Ps::Array<PxVec3>		mQuantizedOutput;
};

Quantizer * physx::createQuantizer(void)
{
	QuantizerImpl *m = PX_NEW(QuantizerImpl);
	return static_cast< Quantizer *>(m);
}
