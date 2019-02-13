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

#include "QuickHullConvexHullLib.h"
#include "ConvexHullUtils.h"

#include "PsAllocator.h"
#include "PsUserAllocated.h"
#include "PsSort.h"
#include "PsMathUtils.h"
#include "PsFoundation.h"
#include "PsUtilities.h"
#include "PsBitUtils.h"

#include "foundation/PxMath.h"
#include "foundation/PxPlane.h"
#include "foundation/PxBounds3.h"
#include "foundation/PxMemory.h"

using namespace physx;

namespace local
{		
	//////////////////////////////////////////////////////////////////////////
	static const float MIN_ADJACENT_ANGLE = 3.0f;  // in degrees  - result wont have two adjacent facets within this angle of each other.
	static const float PLANE_THICKNES = 3.0f * PX_EPS_F32;  // points within this distance are considered on a plane	
	static const float MAXDOT_MINANG = cosf(Ps::degToRad(MIN_ADJACENT_ANGLE)); // adjacent angle for dot product tests

	//////////////////////////////////////////////////////////////////////////

	struct QuickHullFace;
	class ConvexHull;
	class HullPlanes;

	//////////////////////////////////////////////////////////////////////////
	template<typename T, bool useIndexing>
	class MemBlock
	{
	public:
		MemBlock(PxU32 preallocateSize)
			: mPreallocateSize(preallocateSize), mCurrentBlock(0), mCurrentIndex(0)
		{
			PX_ASSERT(preallocateSize);
			T* block = reinterpret_cast<T*>(PX_ALLOC_TEMP(sizeof(T)*preallocateSize, "Quickhull MemBlock"));
			mBlocks.pushBack(block);
		}

		MemBlock()
			: mPreallocateSize(0), mCurrentBlock(0), mCurrentIndex(0)
		{
		}

		void init(PxU32 preallocateSize)
		{
			PX_ASSERT(preallocateSize);
			mPreallocateSize = preallocateSize;
			T* block = reinterpret_cast<T*>(PX_ALLOC_TEMP(sizeof(T)*preallocateSize, "Quickhull MemBlock"));
			if(useIndexing)
			{
				for (PxU32 i = 0; i < mPreallocateSize; i++)
				{
					// placement new to index data
					PX_PLACEMENT_NEW(&block[i], T)(i);
				}
			}
			mBlocks.pushBack(block);
		}

		~MemBlock()
		{
			for (PxU32 i = 0; i < mBlocks.size(); i++)
			{
				PX_FREE(mBlocks[i]);
			}
			mBlocks.clear();
		}

		void reset()
		{
			for (PxU32 i = 0; i < mBlocks.size(); i++)
			{
				PX_FREE(mBlocks[i]);
			}
			mBlocks.clear();

			mCurrentBlock = 0;
			mCurrentIndex = 0;

			init(mPreallocateSize);
		}

		T* getItem(PxU32 index)
		{
			const PxU32 block = index/mPreallocateSize;
			const PxU32 itemIndex = index % mPreallocateSize;
			PX_ASSERT(block <= mCurrentBlock);
			PX_ASSERT(itemIndex < mPreallocateSize);
			return &(mBlocks[block])[itemIndex];
		}

		T* getFreeItem()
		{
			PX_ASSERT(mPreallocateSize);
			// check if we have enough space in block, otherwise allocate new block
			if(mCurrentIndex < mPreallocateSize)
			{
				return &(mBlocks[mCurrentBlock])[mCurrentIndex++];
			}
			else
			{
				T* block = reinterpret_cast<T*>(PX_ALLOC_TEMP(sizeof(T)*mPreallocateSize, "Quickhull MemBlock"));
				mCurrentBlock++;
				if (useIndexing)
				{
					for (PxU32 i = 0; i < mPreallocateSize; i++)
					{
						// placement new to index data
						PX_PLACEMENT_NEW(&block[i], T)(mCurrentBlock*mPreallocateSize + i);
					}
				}
				mBlocks.pushBack(block);				
				mCurrentIndex = 0;
				return &(mBlocks[mCurrentBlock])[mCurrentIndex++];
			}
		}

	private:
		PxU32			mPreallocateSize;
		PxU32			mCurrentBlock;
		PxU32			mCurrentIndex;
		Ps::Array<T*>	mBlocks;
	};

	//////////////////////////////////////////////////////////////////////////
	// representation of quick hull vertex
	struct QuickHullVertex
	{
		PxVec3					point;		// point vector
		PxU32					index;		// point index for compare
		float					dist;		// distance from plane if necessary

		QuickHullVertex*		next;		// link to next vertex, linked list used for conflict list

		PX_FORCE_INLINE bool operator==(const QuickHullVertex& vertex) const
		{
			return index == vertex.index ? true : false;
		}

		PX_FORCE_INLINE bool operator <(const QuickHullVertex& vertex) const
		{
			return dist < vertex.dist ? true : false;
		}
	};

	//////////////////////////////////////////////////////////////////////////
	// representation of quick hull half edge
	struct QuickHullHalfEdge
	{
		QuickHullHalfEdge() : prev(NULL), next(NULL), twin(NULL), face(NULL), edgeIndex(0xFFFFFFFF)
		{
		}

		QuickHullHalfEdge(PxU32 )
			: prev(NULL), next(NULL), twin(NULL), face(NULL), edgeIndex(0xFFFFFFFF)
		{
		}

		QuickHullVertex			tail;  // tail vertex, head vertex is the tail of the twin

		QuickHullHalfEdge*		prev;  // previous edge
		QuickHullHalfEdge*		next;  // next edge
		QuickHullHalfEdge*		twin;  // twin/opposite edge

		QuickHullFace*			face;  // face where the edge belong

		PxU32					edgeIndex; // edge index used for edge creation

		PX_FORCE_INLINE const QuickHullVertex& getTail() const
		{
			return tail;
		}

		PX_FORCE_INLINE const QuickHullVertex& getHead() const
		{
			PX_ASSERT(twin);
			return twin->tail;
		}

		PX_FORCE_INLINE void setTwin(QuickHullHalfEdge* edge)
		{
			twin = edge;
			edge->twin = this;
		}

		PX_FORCE_INLINE QuickHullFace* getOppositeFace() const
		{
			return twin->face;
		}

		float getOppositeFaceDistance() const;
	};

	//////////////////////////////////////////////////////////////////////////

	typedef Ps::Array<QuickHullVertex*>		QuickHullVertexArray;
	typedef Ps::Array<QuickHullHalfEdge*>	QuickHullHalfEdgeArray;
	typedef Ps::Array<QuickHullFace*>		QuickHullFaceArray;

	//////////////////////////////////////////////////////////////////////////
	// representation of quick hull face
	struct QuickHullFace
	{
		enum FaceState
		{
			eVISIBLE,
			eDELETED,
			eNON_CONVEX
		};

		QuickHullHalfEdge*		edge;			// starting edge
		PxU16					numEdges;		// num edges on the face
		QuickHullVertex*		conflictList;	// conflict list, used to determine unclaimed vertices

		PxVec3					normal;			// Newell plane normal
		float					area;			// face area
		PxVec3					centroid;		// face centroid

		float					planeOffset;	// Newell plane offset
		float					expandOffset;	// used for plane expansion if vertex limit reached

		FaceState				state;			// face validity state

		QuickHullFace*			nextFace;		// used to indicate next free face in faceList
		PxU32					index;			// face index for compare identification
		PxU8					outIndex;		// face index used for output descriptor

	public:
		QuickHullFace()
			: edge(NULL), numEdges(0), conflictList(NULL), area(0.0f), planeOffset(0.0f), expandOffset(-FLT_MAX),
			state(eVISIBLE), nextFace(NULL), outIndex(0)
		{
		}

		QuickHullFace(PxU32 ind)
			: edge(NULL), numEdges(0), conflictList(NULL), area(0.0f), planeOffset(0.0f), expandOffset(-FLT_MAX),
			state(eVISIBLE), nextFace(NULL), index(ind), outIndex(0)
		{
		}

		~QuickHullFace()
		{
		}

		// get edge on index
		PX_FORCE_INLINE QuickHullHalfEdge* getEdge(PxU32 i) const
		{
			QuickHullHalfEdge* he = edge;
			while (i > 0)
			{
				he = he->next;
				i--;
			}
			return he;
		}

		// distance from a plane to provided point
		PX_FORCE_INLINE float distanceToPlane(const PxVec3 p) const
		{
			return normal.dot(p) - planeOffset;
		}

		// compute face normal and centroid
		PX_FORCE_INLINE void	computeNormalAndCentroid()
		{
			PX_ASSERT(edge);
			normal = PxVec3(PxZero);
			numEdges = 1;

			QuickHullHalfEdge* testEdge = edge;
			QuickHullHalfEdge* startEdge = NULL;
			float maxDist = 0.0f;
			for (PxU32 i = 0; i < 3; i++)
			{
				const float d = (testEdge->tail.point - testEdge->next->tail.point).magnitudeSquared();
				if (d > maxDist)
				{
					maxDist = d;
					startEdge = testEdge;
				}
				testEdge = testEdge->next;
			}
			PX_ASSERT(startEdge);

			QuickHullHalfEdge* he = startEdge->next;
			const PxVec3& p0 = startEdge->tail.point;
			const PxVec3 d = he->tail.point - p0;
			centroid = startEdge->tail.point;

			do
			{
				numEdges++;
				centroid += he->tail.point;

				normal += d.cross(he->next->tail.point - p0);

				he = he->next;
			} while (he != startEdge);

			area = normal.normalize();
			centroid *= (1.0f / float(numEdges));

			planeOffset = normal.dot(centroid);
		}

		// merge adjacent face
		bool	mergeAdjacentFace(QuickHullHalfEdge* halfEdge, QuickHullFaceArray& discardedFaces);

		// check face consistency
		bool	checkFaceConsistency();

	private:
		// connect halfedges
		QuickHullFace* connectHalfEdges(QuickHullHalfEdge* hedgePrev, QuickHullHalfEdge* hedge);

		// check if the face does have only 3 vertices
		PX_FORCE_INLINE bool	isTriangle() const
		{
			return numEdges == 3 ? true : false;
		}

	};

	//////////////////////////////////////////////////////////////////////////
	struct QuickHullResult
	{
		enum Enum
		{
			eSUCCESS, // ok
			eZERO_AREA_TEST_FAILED, // area test failed for simplex
			eVERTEX_LIMIT_REACHED, // vertex limit reached need to expand hull
			ePOLYGONS_LIMIT_REACHED, // polygons hard limit reached
			eFAILURE // general failure
		};
	};

	//////////////////////////////////////////////////////////////////////////
	// Quickhull base class holding the hull during construction
	class QuickHull : public Ps::UserAllocated
	{
		PX_NOCOPY(QuickHull)
	public:

		QuickHull(const PxCookingParams& params, const PxConvexMeshDesc& desc);

		~QuickHull();

		// preallocate the edges, faces, vertices
		void preallocate(PxU32 numVertices);

		// parse the input verts, store them into internal format
		void parseInputVertices(const PxVec3* verts, PxU32 numVerts);

		// release the hull and data
		void releaseHull();

		// sets the precomputed min/max data
		void setPrecomputedMinMax(const QuickHullVertex* minVertex,const QuickHullVertex* maxVertex, const float tolerance,const float planeTolerance);

		// main entry function to build the hull from provided points
		QuickHullResult::Enum buildHull();

		PxU32 maxNumVertsPerFace() const;

	protected:
		// compute min max verts
		void computeMinMaxVerts();

		// find the initial simplex
		bool findSimplex();

		// add the initial simplex
		void addSimplex(QuickHullVertex* simplex, bool flipTriangle);

		// finds next point to add
		QuickHullVertex* nextPointToAdd(QuickHullFace*& eyeFace);

		// adds point to the hull
		bool addPointToHull(const QuickHullVertex* vertex, QuickHullFace& face, bool& addFailed);

		// creates new face from given triangles 
		QuickHullFace* createTriangle(const QuickHullVertex& v0, const QuickHullVertex& v1, const QuickHullVertex& v2);

		// adds point to the face conflict list
		void addPointToFace(QuickHullFace& face, QuickHullVertex* vertex, float dist);

		// removes eye point from the face conflict list
		void removeEyePointFromFace(QuickHullFace& face, const QuickHullVertex* vertex);

		// calculate the horizon fro the eyePoint against a given face
		void calculateHorizon(const PxVec3& eyePoint, QuickHullHalfEdge* edge, QuickHullFace& face, QuickHullHalfEdgeArray& horizon, QuickHullFaceArray& removedFaces);

		// adds new faces from given horizon and eyePoint
		void addNewFacesFromHorizon(const QuickHullVertex* eyePoint, const QuickHullHalfEdgeArray& horizon, QuickHullFaceArray& newFaces);

		// merge adjacent face
		bool doAdjacentMerge(QuickHullFace& face, bool mergeWrtLargeFace, bool& mergeFailed);

		// merge adjacent face doing normal test
		bool doPostAdjacentMerge(QuickHullFace& face, const float minAngle);

		// delete face points
		void deleteFacePoints(QuickHullFace& faceToDelete, QuickHullFace* absorbingFace);

		// resolve unclaimed points
		void resolveUnclaimedPoints(const QuickHullFaceArray& newFaces);

		// merges polygons with similar normals
		void postMergeHull();

		// check if 2 faces can be merged
		bool canMergeFaces(const QuickHullHalfEdge& he);

		// get next free face
		PX_FORCE_INLINE QuickHullFace* getFreeHullFace()
		{
			return mFreeFaces.getFreeItem();
		}

		// get next free half edge
		PX_FORCE_INLINE QuickHullHalfEdge* getFreeHullHalfEdge()
		{
			return mFreeHalfEdges.getFreeItem();
		}

		PX_FORCE_INLINE PxU32 getNbHullVerts() { return mOutputNumVertices; }

	protected:
		friend class physx::QuickHullConvexHullLib;

		const PxCookingParams&	mCookingParams;		// cooking params
		const PxConvexMeshDesc& mConvexDesc;		// convex desc

		PxVec3					mInteriorPoint;		// interior point for int/ext tests

		PxU32					mMaxVertices;		// maximum number of vertices (can be different as we may add vertices during the cleanup
		PxU32					mNumVertices;		// actual number of input vertices
		PxU32					mOutputNumVertices;	// num vertices of the computed hull
		PxU32					mTerminalVertex;	// in case we failed to generate hull in a regular run we set the terminal vertex and rerun

		QuickHullVertex*		mVerticesList;		// vertices list preallocated
		MemBlock<QuickHullHalfEdge, false>	mFreeHalfEdges;	// free half edges
		MemBlock<QuickHullFace, true>	mFreeFaces;			// free faces

		QuickHullFaceArray		mHullFaces;			// actual hull faces, contains also invalid and not used faces
		PxU32					mNumHullFaces;		// actual number of hull faces

		bool					mPrecomputedMinMax; // if we got the precomputed min/max values
		QuickHullVertex			mMinVertex[3];		// min vertex
		QuickHullVertex			mMaxVertex[3];		// max vertex
		float					mTolerance;			// hull tolerance, used for plane thickness and merge strategy
		float					mPlaneTolerance;	// used for post merge stage

		QuickHullVertexArray	mUnclaimedPoints;	// holds temp unclaimed points

		QuickHullHalfEdgeArray	mHorizon;			// array for horizon computation
		QuickHullFaceArray		mNewFaces;			// new faces created during horizon computation
		QuickHullFaceArray		mRemovedFaces;		// removd faces during horizon computation
		QuickHullFaceArray      mDiscardedFaces;	// discarded faces during face merging
	};

	//////////////////////////////////////////////////////////////////////////
	// return the distance from opposite face
	float QuickHullHalfEdge::getOppositeFaceDistance() const
	{
		PX_ASSERT(face);
		PX_ASSERT(twin);
		return face->distanceToPlane(twin->face->centroid);
	}

	//////////////////////////////////////////////////////////////////////////
	// merge adjacent face from provided half edge. 
	// 1. set new half edges
	// 2. connect the new half edges - check we did not produced redundant triangles, discard them
	// 3. recompute the plane and check consistency
	// Returns false if merge failed
	bool QuickHullFace::mergeAdjacentFace(QuickHullHalfEdge* hedgeAdj, QuickHullFaceArray& discardedFaces)
	{
		QuickHullFace* oppFace = hedgeAdj->getOppositeFace();

		discardedFaces.pushBack(oppFace);
		oppFace->state = QuickHullFace::eDELETED;

		QuickHullHalfEdge* hedgeOpp = hedgeAdj->twin;

		QuickHullHalfEdge* hedgeAdjPrev = hedgeAdj->prev;
		QuickHullHalfEdge* hedgeAdjNext = hedgeAdj->next;
		QuickHullHalfEdge* hedgeOppPrev = hedgeOpp->prev;
		QuickHullHalfEdge* hedgeOppNext = hedgeOpp->next;

		// check if we are lining up with the face in adjPrev dir
		QuickHullHalfEdge* breakEdge = hedgeAdjPrev;
		while (hedgeAdjPrev->getOppositeFace() == oppFace)
		{
			hedgeAdjPrev = hedgeAdjPrev->prev;
			hedgeOppNext = hedgeOppNext->next;

			// Edge case merge face is degenerated and we need to abort merging
			if (hedgeAdjPrev == breakEdge)
			{
				return false;
			}
		}

		// check if we are lining up with the face in adjNext dir
		breakEdge = hedgeAdjNext;
		while (hedgeAdjNext->getOppositeFace() == oppFace)
		{
			hedgeOppPrev = hedgeOppPrev->prev;
			hedgeAdjNext = hedgeAdjNext->next;

			// Edge case merge face is degenerated and we need to abort merging
			if (hedgeAdjNext == breakEdge)
			{
				return false;
			}
		}

		QuickHullHalfEdge* hedge;

		// set new face owner for the line up edges
		for (hedge = hedgeOppNext; hedge != hedgeOppPrev->next; hedge = hedge->next)
		{
			hedge->face = this;
		}

		// if we are about to delete the shared edge, check if its not the starting edge of the face
		if (hedgeAdj == edge)
		{
			edge = hedgeAdjNext;
		}

		// handle the half edges at the head
		QuickHullFace* discardedFace;
		discardedFace = connectHalfEdges(hedgeOppPrev, hedgeAdjNext);
		if (discardedFace != NULL)
		{
			discardedFaces.pushBack(discardedFace);
		}

		// handle the half edges at the tail
		discardedFace = connectHalfEdges(hedgeAdjPrev, hedgeOppNext);
		if (discardedFace != NULL)
		{
			discardedFaces.pushBack(discardedFace);
		}

		computeNormalAndCentroid();
		PX_ASSERT(checkFaceConsistency());

		return true;
	}

	//////////////////////////////////////////////////////////////////////////
	// connect half edges of 2 adjacent faces
	// if we find redundancy - edges are in a line, we drop the addional face if it is just a skinny triangle
	QuickHullFace* QuickHullFace::connectHalfEdges(QuickHullHalfEdge* hedgePrev, QuickHullHalfEdge* hedge)
	{
		QuickHullFace* discardedFace = NULL;

		// redundant edge - can be in a line
		if (hedgePrev->getOppositeFace() == hedge->getOppositeFace())
		{
			// then there is a redundant edge that we can get rid off
			QuickHullFace* oppFace = hedge->getOppositeFace();
			QuickHullHalfEdge* hedgeOpp;

			if (hedgePrev == edge)
			{
				edge = hedge;
			}

			// check if its not a skinny face with just 3 vertices - 3 edges
			if (oppFace->isTriangle())
			{
				// then we can get rid of the opposite face altogether
				hedgeOpp = hedge->twin->prev->twin;

				oppFace->state = QuickHullFace::eDELETED;
				discardedFace = oppFace;
			}
			else
			{
				// if not triangle, merge the 2 opposite halfedges into one
				hedgeOpp = hedge->twin->next;

				if (oppFace->edge == hedgeOpp->prev)
				{
					oppFace->edge = hedgeOpp;
				}
				hedgeOpp->prev = hedgeOpp->prev->prev;
				hedgeOpp->prev->next = hedgeOpp;
			}

			hedge->prev = hedgePrev->prev;
			hedge->prev->next = hedge;

			hedge->twin = hedgeOpp;
			hedgeOpp->twin = hedge;

			// oppFace was modified, so need to recompute
			oppFace->computeNormalAndCentroid();
		}
		else
		{
			// just merge the halfedges
			hedgePrev->next = hedge;
			hedge->prev = hedgePrev;
		}
		return discardedFace;
	}

	//////////////////////////////////////////////////////////////////////////
	// check face consistency
	bool QuickHullFace::checkFaceConsistency()
	{
		// do a sanity check on the face
		QuickHullHalfEdge* hedge = edge;
		PxU32 numv = 0;

		// check degenerate face
		do
		{
			numv++;
			hedge = hedge->next;
		} while (hedge != edge);

		// degenerate face found
		PX_ASSERT(numv > 2);

		numv = 0;
		hedge = edge;
		do
		{
			QuickHullHalfEdge* hedgeOpp = hedge->twin;

			// check if we have twin set
			PX_ASSERT(hedgeOpp != NULL);

			// twin for the twin must be the original edge
			PX_ASSERT(hedgeOpp->twin == hedge);

			QuickHullFace* oppFace = hedgeOpp->face;

			PX_UNUSED(oppFace);

			// opposite edge face must be set and valid
			PX_ASSERT(oppFace != NULL);
			PX_ASSERT(oppFace->state != QuickHullFace::eDELETED);

			// edges face must be this one
			PX_ASSERT(hedge->face == this);			

			hedge = hedge->next;
		} while (hedge != edge);

		return true;
	}

	//////////////////////////////////////////////////////////////////////////

	QuickHull::QuickHull(const PxCookingParams& params, const PxConvexMeshDesc& desc)
		: mCookingParams(params), mConvexDesc(desc), mOutputNumVertices(0), mTerminalVertex(0xFFFFFFFF), mVerticesList(NULL), mNumHullFaces(0), mPrecomputedMinMax(false),
		mTolerance(-1.0f), mPlaneTolerance(-1.0f)
	{
	}

	//////////////////////////////////////////////////////////////////////////

	QuickHull::~QuickHull()
	{
	}

	//////////////////////////////////////////////////////////////////////////
	// sets the precomputed min/max values
	void QuickHull::setPrecomputedMinMax(const QuickHullVertex* minVertex,const QuickHullVertex* maxVertex, const float tolerance,const float planeTolerance)
	{
		for (PxU32 i = 0; i < 3; i++)
		{
			mMinVertex[i] = minVertex[i];
			mMaxVertex[i] = maxVertex[i];
		}

		mTolerance = tolerance;
		mPlaneTolerance = planeTolerance;

		mPrecomputedMinMax = true;
	}

	//////////////////////////////////////////////////////////////////////////
	// preallocate internal buffers
	void QuickHull::preallocate(PxU32 numVertices)
	{
		PX_ASSERT(numVertices > 0);

		// max num vertices = numVertices
		mMaxVertices = PxMax(PxU32(8), numVertices); // 8 is min, since we can expand to AABB during the clean vertices phase
		mVerticesList = reinterpret_cast<QuickHullVertex*> (PX_ALLOC_TEMP(sizeof(QuickHullVertex)*mMaxVertices, "QuickHullVertex"));

		// estimate the max half edges
		PxU32 maxHalfEdges = (3 * mMaxVertices - 6) * 3;
		mFreeHalfEdges.init(maxHalfEdges);

		// estimate the max faces
		PxU32 maxFaces = (2 * mMaxVertices - 4);
		mFreeFaces.init(maxFaces*2);

		mHullFaces.reserve(maxFaces);
		mUnclaimedPoints.reserve(numVertices);

		mNewFaces.reserve(32);
		mRemovedFaces.reserve(32);
		mDiscardedFaces.reserve(32);
		mHorizon.reserve(PxMin(numVertices,PxU32(128)));
	}

	//////////////////////////////////////////////////////////////////////////
	// release internal buffers
	void QuickHull::releaseHull()
	{
		if (mVerticesList)
		{
			PX_FREE_AND_RESET(mVerticesList);
		}
		mHullFaces.clear();
	}

	//////////////////////////////////////////////////////////////////////////
	// returns the maximum number of vertices on a face
	PxU32 QuickHull::maxNumVertsPerFace() const
	{
		PxU32 numFaces = mHullFaces.size();
		PxU32 maxVerts = 0;
		for (PxU32 i = 0; i < numFaces; i++)
		{
			const local::QuickHullFace& face = *mHullFaces[i];
			if (face.state == local::QuickHullFace::eVISIBLE)
			{								
				if (face.numEdges > maxVerts)
					maxVerts = face.numEdges;
			}
		}
		return maxVerts;
	}

	//////////////////////////////////////////////////////////////////////////
	// parse the input vertices and store them in the hull
	void QuickHull::parseInputVertices(const PxVec3* verts, PxU32 numVerts)
	{
		PX_ASSERT(verts);
		PX_ASSERT(numVerts <= mMaxVertices);

		mNumVertices = numVerts;
		for (PxU32 i = 0; i < numVerts; i++)
		{
			mVerticesList[i].point = verts[i];
			mVerticesList[i].index = i;
		}
	}

	//////////////////////////////////////////////////////////////////////////
	// compute min max verts
	void QuickHull::computeMinMaxVerts()
	{
		for (PxU32 i = 0; i < 3; i++)
		{
			mMinVertex[i] = mVerticesList[0];
			mMaxVertex[i] = mVerticesList[0];
		}

		PxVec3 max = mVerticesList[0].point;
		PxVec3 min = mVerticesList[0].point;

		// get the max min vertices along the x,y,z
		for (PxU32 i = 1; i < mNumVertices; i++)
		{
			const QuickHullVertex& testVertex = mVerticesList[i];
			const PxVec3& testPoint = testVertex.point;
			if (testPoint.x > max.x)
			{
				max.x = testPoint.x;
				mMaxVertex[0] = testVertex;
			}
			else if (testPoint.x < min.x)
			{
				min.x = testPoint.x;
				mMinVertex[0] = testVertex;
			}

			if (testPoint.y > max.y)
			{
				max.y = testPoint.y;
				mMaxVertex[1] = testVertex;
			}
			else if (testPoint.y < min.y)
			{
				min.y = testPoint.y;
				mMinVertex[1] = testVertex;
			}

			if (testPoint.z > max.z)
			{
				max.z = testPoint.z;
				mMaxVertex[2] = testVertex;
			}
			else if (testPoint.z < min.z)
			{
				min.z = testPoint.z;
				mMinVertex[2] = testVertex;
			}
		}

		const float sizeTol = (max.x-min.x + max.y - min.y + max.z - min.z)*0.5f;
		mTolerance = PxMax(local::PLANE_THICKNES * sizeTol, local::PLANE_THICKNES);
		mPlaneTolerance = PxMax(mCookingParams.planeTolerance * sizeTol, mCookingParams.planeTolerance);
	}

	//////////////////////////////////////////////////////////////////////////
	// find the initial simplex
	// 1. search in max axis from compute min,max
	// 2. 3rd point is the furthest vertex from the initial line
	// 3. 4th vertex is along the line, 3rd vertex normal
	bool QuickHull::findSimplex()
	{
		float max = 0;
		PxU32 imax = 0;

		for (PxU32 i = 0; i < 3; i++)
		{
			float diff = mMaxVertex[i].point[i] - mMinVertex[i].point[i];
			if (diff > max)
			{
				max = diff;
				imax = i;
			}
		}

		if (max <= mTolerance)
		{
			// should not happen as we clear the vertices before and expand them if they are really close to each other
			Ps::getFoundation().error(PxErrorCode::eINTERNAL_ERROR, __FILE__, __LINE__, "QuickHullConvexHullLib::findSimplex: Simplex input points appers to be almost at the same place");
			return false;
		}

		QuickHullVertex simplex[4];

		// set first two vertices to be those with the greatest
		// one dimensional separation
		simplex[0] = mMaxVertex[imax];
		simplex[1] = mMinVertex[imax];

		// set third vertex to be the vertex farthest from
		// the line between simplex[0] and simplex[1]
		PxVec3 normal;
		float maxDist = 0;
		PxVec3 u01 = (simplex[1].point - simplex[0].point);
		u01.normalize();

		for (PxU32 i = 0; i < mNumVertices; i++)
		{
			const QuickHullVertex& testVert = mVerticesList[i];
			const PxVec3& testPoint = testVert.point;
			const PxVec3 diff = testPoint - simplex[0].point;
			const PxVec3 xprod = u01.cross(diff);
			const float lenSqr = xprod.magnitudeSquared();
			if (lenSqr > maxDist && testVert.index != simplex[0].index && testVert.index != simplex[1].index)
			{
				maxDist = lenSqr;
				simplex[2] = testVert;
				normal = xprod;
			}
		}

		if (PxSqrt(maxDist) <= mTolerance)
		{
			Ps::getFoundation().error(PxErrorCode::eINTERNAL_ERROR, __FILE__, __LINE__, "QuickHullConvexHullLib::findSimplex: Simplex input points appers to be colinear.");
			return false;
		}
		normal.normalize();

		// set the forth vertex in the normal direction	
		const float d0 = simplex[2].point.dot(normal);
		maxDist = 0.0f;
		for (PxU32 i = 0; i < mNumVertices; i++)
		{
			const QuickHullVertex& testVert = mVerticesList[i];
			const PxVec3& testPoint = testVert.point;
			const float dist = PxAbs(testPoint.dot(normal) - d0);
			if (dist > maxDist && testVert.index != simplex[0].index &&
				testVert.index != simplex[1].index && testVert.index != simplex[2].index)
			{
				maxDist = dist;
				simplex[3] = testVert;
			}
		}

		if (PxAbs(maxDist) <= mTolerance)
		{
			Ps::getFoundation().error(PxErrorCode::eINTERNAL_ERROR, __FILE__, __LINE__, "QuickHullConvexHullLib::findSimplex: Simplex input points appers to be coplanar.");
			return false;
		}

		// now create faces from those triangles
		addSimplex(&simplex[0], simplex[3].point.dot(normal) - d0 < 0);

		return true;
	}

	//////////////////////////////////////////////////////////////////////////
	// create triangle from given vertices, produce new face and connect the half edges
	QuickHullFace* QuickHull::createTriangle(const QuickHullVertex& v0, const QuickHullVertex& v1, const QuickHullVertex& v2)
	{
		QuickHullFace* face = getFreeHullFace();

		QuickHullHalfEdge* he0 = getFreeHullHalfEdge();
		he0->face = face;
		he0->tail = v0;

		QuickHullHalfEdge* he1 = getFreeHullHalfEdge();
		he1->face = face;
		he1->tail = v1;

		QuickHullHalfEdge* he2 = getFreeHullHalfEdge();
		he2->face = face;
		he2->tail = v2;

		he0->prev = he2;
		he0->next = he1;
		he1->prev = he0;
		he1->next = he2;
		he2->prev = he1;
		he2->next = he0;

		face->edge = he0;
		face->nextFace = NULL;

		// compute the normal and offset
		face->computeNormalAndCentroid();
		return face;
	}


	//////////////////////////////////////////////////////////////////////////
	// add initial simplex to the quickhull
	// construct triangles from the simplex points and connect them with half edges
	void QuickHull::addSimplex(QuickHullVertex* simplex, bool flipTriangle)
	{
		PX_ASSERT(simplex);

		// get interior point
		PxVec3 vectorSum = simplex[0].point;
		for (PxU32 i = 1; i < 4; i++)
		{
			vectorSum += simplex[i].point;
		}
		mInteriorPoint = vectorSum / 4.0f;

		QuickHullFace* tris[4];
		// create the triangles from the initial simplex
		if (flipTriangle)
		{
			tris[0] = createTriangle(simplex[0], simplex[1], simplex[2]);
			tris[1] = createTriangle(simplex[3], simplex[1], simplex[0]);
			tris[2] = createTriangle(simplex[3], simplex[2], simplex[1]);
			tris[3] = createTriangle(simplex[3], simplex[0], simplex[2]);

			for (PxU32 i = 0; i < 3; i++)
			{
				PxU32 k = (i + 1) % 3;
				tris[i + 1]->getEdge(1)->setTwin(tris[k + 1]->getEdge(0));
				tris[i + 1]->getEdge(2)->setTwin(tris[0]->getEdge(k));
			}
		}
		else
		{
			tris[0] = createTriangle(simplex[0], simplex[2], simplex[1]);
			tris[1] = createTriangle(simplex[3], simplex[0], simplex[1]);
			tris[2] = createTriangle(simplex[3], simplex[1], simplex[2]);
			tris[3] = createTriangle(simplex[3], simplex[2], simplex[0]);

			for (PxU32 i = 0; i < 3; i++)
			{
				PxU32 k = (i + 1) % 3;
				tris[i + 1]->getEdge(0)->setTwin(tris[k + 1]->getEdge(1));
				tris[i + 1]->getEdge(2)->setTwin(tris[0]->getEdge((3 - i) % 3));
			}
		}

		// push back the first 4 faces created from the simplex
		for (PxU32 i = 0; i < 4; i++)
		{
			mHullFaces.pushBack(tris[i]);
		}
		mNumHullFaces = 4;

		// go through points and add point to faces if they are on the plane
		for (PxU32 i = 0; i < mNumVertices; i++)
		{
			const QuickHullVertex& v = mVerticesList[i];

			if (v == simplex[0] || v == simplex[1] || v == simplex[2] || v == simplex[3])
			{
				continue;
			}

			float maxDist = mTolerance;
			QuickHullFace* maxFace = NULL;
			for (PxU32 k = 0; k < 4; k++)
			{
				const float dist = tris[k]->distanceToPlane(v.point);
				if (dist > maxDist)
				{
					maxFace = tris[k];
					maxDist = dist;
				}
			}

			if (maxFace != NULL)
			{
				addPointToFace(*maxFace, &mVerticesList[i], maxDist);
			}
		}
	}

	//////////////////////////////////////////////////////////////////////////
	// adds a point to the conflict list
	// the trick here is to store the most furthest point as the last, thats the only one we care about
	// the rest is not important, we just need to store them and claim to new faces later, if the 
	// faces most furthest point is the current global maximum
	void QuickHull::addPointToFace(QuickHullFace& face, QuickHullVertex* vertex, float dist)
	{
		// if we dont have a conflict list, store the vertex as the first one in the conflict list
		vertex->dist = dist;
		if(!face.conflictList)
		{
			face.conflictList = vertex;
			vertex->dist = dist;
			vertex->next = NULL;
			return;
		}

		PX_ASSERT(face.conflictList);

		// this is not the furthest vertex, store it as next in the linked list
		if (face.conflictList->dist > dist)
			{
			vertex->next = face.conflictList->next;
			face.conflictList->next = vertex;
			}
		else
		{
			// this is the furthest vertex, store it as first in the linked list
			vertex->next = face.conflictList;
			face.conflictList = vertex;
		}
	}

	//////////////////////////////////////////////////////////////////////////
	// removes eye point from a conflict list
	// we know that the vertex must the last, as we store it at the back, so just popback()
	void QuickHull::removeEyePointFromFace(QuickHullFace& face, const QuickHullVertex* vertex)
	{
		PX_UNUSED(vertex);
		// the picked vertex should always be the first in the linked list
		PX_ASSERT(face.conflictList == vertex);

		face.conflictList = face.conflictList->next;		
	}

	//////////////////////////////////////////////////////////////////////////
	// merge polygons with similar normals
	void QuickHull::postMergeHull()
	{		
		// merge faces with similar normals 
		for (PxU32 i = 0; i < mHullFaces.size(); i++)
		{
			QuickHullFace& face = *mHullFaces[i];

			if (face.state == QuickHullFace::eVISIBLE)
			{
				PX_ASSERT(face.checkFaceConsistency());
				while (doPostAdjacentMerge(face, local::MAXDOT_MINANG));
			}
		}
	}

	//////////////////////////////////////////////////////////////////////////
	// builds the hull
	// 1. find the initial simplex
	// 2. check if simplex has a valid area
	// 3. add vertices to the hull. We add vertex most furthest from the hull
	// 4. terminate if hull limit reached or we have added all vertices
	QuickHullResult::Enum QuickHull::buildHull()
	{
		QuickHullVertex* eyeVtx = NULL;
		QuickHullFace*	eyeFace;

		// compute the vertex min max along x,y,z
		if(!mPrecomputedMinMax)
			computeMinMaxVerts();

		// find the initial simplex of the hull
		if (!findSimplex())
		{
			return QuickHullResult::eFAILURE;
		}

		// simplex area test
		const bool useAreaTest = mConvexDesc.flags & PxConvexFlag::eCHECK_ZERO_AREA_TRIANGLES ? true : false;
		const float areaEpsilon = mCookingParams.areaTestEpsilon * 2.0f;
		if (useAreaTest)
		{
			for (PxU32 i = 0; i < mHullFaces.size(); i++)
			{
				if (mHullFaces[i]->area < areaEpsilon)
				{
					return QuickHullResult::eZERO_AREA_TEST_FAILED;
				}
			}
		}

		// add points to the hull
		PxU32 numVerts = 4; // initial vertex count - simplex vertices		
		while ((eyeVtx = nextPointToAdd(eyeFace)) != NULL && eyeVtx->index != mTerminalVertex)
		{
			// if plane shifting vertex limit, we need the reduced hull
			if((mConvexDesc.flags & PxConvexFlag::ePLANE_SHIFTING) && (numVerts >= mConvexDesc.vertexLimit))
				break;

			bool addFailed = false;
			PX_ASSERT(eyeFace);
			if (!addPointToHull(eyeVtx, *eyeFace, addFailed))
			{
				mOutputNumVertices = numVerts;
				// we hit the polygons hard limit
				return QuickHullResult::ePOLYGONS_LIMIT_REACHED;
			}
			// We failed to add the vertex, store the vertex as terminal vertex and re run the hull generator
			if(addFailed)
			{
				// set the terminal vertex
				mTerminalVertex = eyeVtx->index;

				// reset the edges/faces memory
				mFreeHalfEdges.reset();
				mFreeFaces.reset();

				// reset the hull state
				mHullFaces.clear();
				mNumHullFaces = 0;
				mUnclaimedPoints.clear();
				mHorizon.clear();
				mNewFaces.clear();
				mRemovedFaces.clear();
				mDiscardedFaces.clear();

				// rerun the hull generator
				return buildHull();
			}
			numVerts++;
		}
		mOutputNumVertices = numVerts;

		// vertex limit has been reached. We did not stopped the iteration, since we
		// will use the produced hull to compute OBB from it and use the planes
		// to slice the initial OBB
		if (numVerts > mConvexDesc.vertexLimit)
		{
			return QuickHullResult::eVERTEX_LIMIT_REACHED;
		}

		return QuickHullResult::eSUCCESS;
	}

	//////////////////////////////////////////////////////////////////////////
	// finds the best point to add to the hull
	// go through the faces conflict list and pick the global maximum
	QuickHullVertex* QuickHull::nextPointToAdd(QuickHullFace*& eyeFace)
	{	
		QuickHullVertex* eyeVtx = NULL;
		QuickHullFace* eyeF = NULL;
		float maxDist = mPlaneTolerance;
		for (PxU32 i = 0; i < mHullFaces.size(); i++)
		{
			if (mHullFaces[i]->state == QuickHullFace::eVISIBLE && mHullFaces[i]->conflictList)
			{
				const float dist = mHullFaces[i]->conflictList->dist;
				if (maxDist < dist)
				{
					maxDist = dist;
					eyeVtx = mHullFaces[i]->conflictList;
					eyeF = mHullFaces[i];
				}
			}
		}

		eyeFace = eyeF;
		return eyeVtx;
	}

	//////////////////////////////////////////////////////////////////////////
	// adds vertex to the hull
	// sets addFailed to true if we failed to add a point because the merging failed
	// this can happen as the face plane equation changes and some faces might become concave
	// returns false if the new faces count would hit the hull face hard limit (255)
	bool QuickHull::addPointToHull(const QuickHullVertex* eyeVtx, QuickHullFace& eyeFace, bool& addFailed)
	{
		addFailed = false;

		// removes the eyePoint from the conflict list
		removeEyePointFromFace(eyeFace, eyeVtx);

		// calculates the horizon from the eyePoint
		calculateHorizon(eyeVtx->point, NULL, eyeFace, mHorizon, mRemovedFaces);

		// check if we dont hit the polygons hard limit
		if (mNumHullFaces + mHorizon.size() > 255)
		{
			// make the faces visible again and quit 
			for (PxU32 i = 0; i < mRemovedFaces.size(); i++)
			{
				mRemovedFaces[i]->state = QuickHullFace::eVISIBLE;
			}
			mNumHullFaces += mRemovedFaces.size();
			return false;
		}

		// adds new faces from given horizon and eyePoint
		addNewFacesFromHorizon(eyeVtx, mHorizon, mNewFaces);

		bool mergeFailed = false;
		// first merge pass ... merge faces which are non-convex
		// as determined by the larger face
		for (PxU32 i = 0; i < mNewFaces.size(); i++)
		{
			QuickHullFace& face = *mNewFaces[i];

			if (face.state == QuickHullFace::eVISIBLE)
			{
				PX_ASSERT(face.checkFaceConsistency());
				while (doAdjacentMerge(face, true, mergeFailed));
			}
		}
		if (mergeFailed)
		{
			addFailed = true;
			return true;
		}

		// second merge pass ... merge faces which are non-convex
		// wrt either face	     
		for (PxU32 i = 0; i < mNewFaces.size(); i++)
		{
			QuickHullFace& face = *mNewFaces[i];
			if (face.state == QuickHullFace::eNON_CONVEX)
			{
				face.state = QuickHullFace::eVISIBLE;
				while (doAdjacentMerge(face, false, mergeFailed));
			}
		}
		if (mergeFailed)
		{
			addFailed = true;
			return true;
		}

		resolveUnclaimedPoints(mNewFaces);

		mHorizon.clear();
		mNewFaces.clear();
		mRemovedFaces.clear();

		return true;
	}

	//////////////////////////////////////////////////////////////////////////
	// merge adjacent faces
	// We merge 2 adjacent faces if they lie on the same thick plane defined by the mTolerance
	// we do this in 2 steps to ensure we dont leave non-convex faces
	bool QuickHull::doAdjacentMerge(QuickHullFace& face, bool mergeWrtLargeFace, bool& mergeFailed)
	{
		QuickHullHalfEdge* hedge = face.edge;
		mergeFailed = false;

		bool convex = true;
		do
		{
			const QuickHullFace& oppFace = *hedge->getOppositeFace();
			bool merge = false;			

			if (mergeWrtLargeFace)
			{
				// merge faces if they are parallel or non-convex
				// wrt to the larger face; otherwise, just mark
				// the face non-convex for the second pass.
				if (face.area > oppFace.area)
				{
					if (hedge->getOppositeFaceDistance() > -mTolerance)
					{
						merge = true;
					}
					else if (hedge->twin->getOppositeFaceDistance() > -mTolerance)
					{
						convex = false;
					}
				}
				else
				{
					if (hedge->twin->getOppositeFaceDistance() > -mTolerance)
					{
						merge = true;
					}
					else if (hedge->getOppositeFaceDistance() > -mTolerance)
					{
						convex = false;
					}
				}
			}
			else
			{
				// then merge faces if they are definitively non-convex
				if (hedge->getOppositeFaceDistance() > -mTolerance ||
					hedge->twin->getOppositeFaceDistance() > -mTolerance)
				{
					merge = true;
				}
			}

			if (merge)
			{
				mDiscardedFaces.clear();
				if (!face.mergeAdjacentFace(hedge, mDiscardedFaces))
				{
					mergeFailed = true;
					return false;
				}
				mNumHullFaces -= mDiscardedFaces.size();
				for (PxU32 i = 0; i < mDiscardedFaces.size(); i++)
				{
					deleteFacePoints(*mDiscardedFaces[i], &face);
				}
				PX_ASSERT(face.checkFaceConsistency());
				return true;
			}
			hedge = hedge->next;
		} while (hedge != face.edge);

		if (!convex)
		{
			face.state = QuickHullFace::eNON_CONVEX;
		}
		return false;
	}

	//////////////////////////////////////////////////////////////////////////
	// merge adjacent faces doing normal test
	// we try to merge more aggressively 2 faces with the same normal. 	
	bool QuickHull::doPostAdjacentMerge(QuickHullFace& face, const float maxdot_minang)
	{
		QuickHullHalfEdge* hedge = face.edge;

		do
		{
			const QuickHullFace& oppFace = *hedge->getOppositeFace();
			bool merge = false;
			const PxVec3& ni = face.normal;
			const PxVec3& nj = oppFace.normal;
			const float dotP = ni.dot(nj);

			if (dotP > maxdot_minang)
			{
				if (face.area >= oppFace.area)
				{
					// check if we can merge the 2 faces
					merge = canMergeFaces(*hedge);
				}
			}

			if (merge)
			{
				QuickHullFaceArray discardedFaces;
				face.mergeAdjacentFace(hedge, discardedFaces);
				mNumHullFaces -= discardedFaces.size();
				for (PxU32 i = 0; i < discardedFaces.size(); i++)
				{
					deleteFacePoints(*discardedFaces[i], &face);
				}
				PX_ASSERT(face.checkFaceConsistency());
				return true;
			}
			hedge = hedge->next;
		} while (hedge != face.edge);

		return false;
	}

	//////////////////////////////////////////////////////////////////////////
	// checks if 2 adjacent faces can be merged
	// 1. creates a face with merged vertices
	// 2. computes new normal and centroid
	// 3. checks that all verts are not too far away from the plane
	// 4. checks that the new polygon is still convex
	// 5. checks if we are about to merge only 2 neighbor faces, we dont 
	// want to merge additional faces, that might corrupt the convexity
	bool QuickHull::canMergeFaces(const QuickHullHalfEdge& he)
	{
		const QuickHullFace& face1 = *he.face;
		const QuickHullFace& face2 = *he.twin->face;

		// construct the merged face
		PX_ALLOCA(edges, QuickHullHalfEdge, (face1.numEdges + face2.numEdges));
		PxMemSet(edges, 0, (face1.numEdges + face2.numEdges)*sizeof(QuickHullHalfEdge));
		QuickHullFace mergedFace;
		mergedFace.edge =  &edges[0];

		// copy the first face edges
		PxU32 currentEdge = 0;
		const QuickHullHalfEdge* heTwin = NULL;
		const QuickHullHalfEdge* heCopy = NULL;
		const QuickHullHalfEdge* startEdge = (face1.edge != &he) ? face1.edge : face1.edge->next;
 		const QuickHullHalfEdge* copyHe = startEdge;
		do
		{
			edges[currentEdge].face = &mergedFace;
			edges[currentEdge].tail = copyHe->tail;
			if(copyHe == &he)
			{
				heTwin = copyHe->twin;
				heCopy = &edges[currentEdge];
			}
			const PxU32 nextIndex = (copyHe->next == startEdge) ? 0 : currentEdge + 1;
			const PxU32 prevIndex = (currentEdge == 0) ? face1.numEdges - 1 : currentEdge - 1;
			edges[currentEdge].next = &edges.mPointer[nextIndex];
			edges[currentEdge].prev = &edges.mPointer[prevIndex];

			currentEdge++;
			copyHe = copyHe->next;
		} while (copyHe != startEdge);

		// copy the second face edges
		copyHe = face2.edge;
		do
		{
			edges[currentEdge].face = &mergedFace;
			edges[currentEdge].tail = copyHe->tail;
			if(heTwin == copyHe)
				heTwin = &edges[currentEdge];
			const PxU32 nextIndex = (copyHe->next == face2.edge) ? face1.numEdges : currentEdge + 1;
			const PxU32 prevIndex = (currentEdge == face1.numEdges) ? face1.numEdges + face2.numEdges - 1 : currentEdge - 1;
			edges[currentEdge].next = &edges.mPointer[nextIndex];
			edges[currentEdge].prev = &edges.mPointer[prevIndex];

			currentEdge++;
			copyHe = copyHe->next;
		} while (copyHe != face2.edge);

		PX_ASSERT(heTwin);

		QuickHullHalfEdge* hedgeAdjPrev = heCopy->prev;
		QuickHullHalfEdge* hedgeAdjNext = heCopy->next;
		QuickHullHalfEdge* hedgeOppPrev = heTwin->prev;
		QuickHullHalfEdge* hedgeOppNext = heTwin->next;

		hedgeOppPrev->next = hedgeAdjNext;
		hedgeAdjNext->prev = hedgeOppPrev;

		hedgeAdjPrev->next = hedgeOppNext;
		hedgeOppNext->prev = hedgeAdjPrev;

		// compute normal and centroid
		mergedFace.computeNormalAndCentroid();

		// test the vertex distance
		const float maxDist = mPlaneTolerance;
		for(PxU32 iVerts=0; iVerts< mNumVertices; iVerts++)
		{
			const QuickHullVertex& vertex = mVerticesList[iVerts];
			const float dist = mergedFace.distanceToPlane(vertex.point);
			if (dist > maxDist)
			{
				return false;
			}
		}

		// check the convexity
		QuickHullHalfEdge* qhe = mergedFace.edge;
		do
		{
			const QuickHullVertex& vertex = qhe->tail;
			const QuickHullVertex& nextVertex = qhe->next->tail;

			PxVec3 edgeVector = nextVertex.point - vertex.point;
			edgeVector.normalize();
			const PxVec3 outVector = -mergedFace.normal.cross(edgeVector);

			QuickHullHalfEdge* testHe = qhe->next;
			do
			{
				const QuickHullVertex& testVertex = testHe->tail;
				const float dist = (testVertex.point - vertex.point).dot(outVector);

				if (dist > mTolerance)
					return false;

				testHe = testHe->next;
			}  while (testHe != qhe->next);

			qhe = qhe->next;
		} while (qhe != mergedFace.edge);


		const QuickHullFace* oppFace = he.getOppositeFace();

		QuickHullHalfEdge* hedgeOpp = he.twin;

		hedgeAdjPrev = he.prev;
		hedgeAdjNext = he.next;
		hedgeOppPrev = hedgeOpp->prev;
		hedgeOppNext = hedgeOpp->next;

		// check if we are lining up with the face in adjPrev dir
		while (hedgeAdjPrev->getOppositeFace() == oppFace)
		{
			hedgeAdjPrev = hedgeAdjPrev->prev;
			hedgeOppNext = hedgeOppNext->next;
		}

		// check if we are lining up with the face in adjNext dir
		while (hedgeAdjNext->getOppositeFace() == oppFace)
		{
			hedgeOppPrev = hedgeOppPrev->prev;
			hedgeAdjNext = hedgeAdjNext->next;
		}

		// no redundant merges, just clean merge of 2 neighbour faces
		if (hedgeOppPrev->getOppositeFace() == hedgeAdjNext->getOppositeFace())
		{
			return false;
		}

		if (hedgeAdjPrev->getOppositeFace() == hedgeOppNext->getOppositeFace())
		{
			return false;
		}

		return true;
	}

	//////////////////////////////////////////////////////////////////////////
	// delete face points and store them as unclaimed, so we can add them back to new faces later
	void QuickHull::deleteFacePoints(QuickHullFace& face, QuickHullFace* absorbingFace)
	{
		// no conflict list for this face
		if(!face.conflictList)
			return;

		QuickHullVertex* unclaimedVertex = face.conflictList;
		QuickHullVertex* vertexToClaim = NULL;
		while (unclaimedVertex)
		{
			vertexToClaim = unclaimedVertex;
			unclaimedVertex = unclaimedVertex->next;
			vertexToClaim->next = NULL;
				if (!absorbingFace)
				{
				mUnclaimedPoints.pushBack(vertexToClaim);
				}
				else
				{
				const float dist = absorbingFace->distanceToPlane(vertexToClaim->point);
					if (dist > mTolerance)
					{
					addPointToFace(*absorbingFace, vertexToClaim, dist);
					}
					else
					{
					mUnclaimedPoints.pushBack(vertexToClaim);
					}
				}
			}

		face.conflictList = NULL;
		}

	//////////////////////////////////////////////////////////////////////////
	// calculate the horizon from the eyePoint against a given face
	void QuickHull::calculateHorizon(const PxVec3& eyePoint, QuickHullHalfEdge* edge0, QuickHullFace& face, QuickHullHalfEdgeArray& horizon, QuickHullFaceArray& removedFaces)
	{
		deleteFacePoints(face, NULL);
		face.state = QuickHullFace::eDELETED;
		removedFaces.pushBack(&face);
		mNumHullFaces--;
		QuickHullHalfEdge* edge;
		if (edge0 == NULL)
		{
			edge0 = face.getEdge(0);
			edge = edge0;
		}
		else
		{
			edge = edge0->next;
		}

		do
		{
			QuickHullFace* oppFace = edge->getOppositeFace();
			if (oppFace->state == QuickHullFace::eVISIBLE)
			{
				const float dist = oppFace->distanceToPlane(eyePoint);
				if (dist > mTolerance)
				{
					calculateHorizon(eyePoint, edge->twin, *oppFace, horizon, removedFaces);
				}
				else
				{
					horizon.pushBack(edge);
				}
			}
			edge = edge->next;
		} while (edge != edge0);
	}

	//////////////////////////////////////////////////////////////////////////
	// adds new faces from given horizon and eyePoint
	void QuickHull::addNewFacesFromHorizon(const QuickHullVertex* eyePoint, const QuickHullHalfEdgeArray& horizon, QuickHullFaceArray& newFaces)
	{
		QuickHullHalfEdge* hedgeSidePrev = NULL;
		QuickHullHalfEdge* hedgeSideBegin = NULL;

		for (PxU32 i = 0; i < horizon.size(); i++)
		{
			const QuickHullHalfEdge& horizonHe = *horizon[i];

			QuickHullFace* face = createTriangle(*eyePoint, horizonHe.getHead(), horizonHe.getTail());
			mHullFaces.pushBack(face);
			mNumHullFaces++;
			face->getEdge(2)->setTwin(horizonHe.twin);

			QuickHullHalfEdge* hedgeSide = face->edge;
			if (hedgeSidePrev != NULL)
			{
				hedgeSide->next->setTwin(hedgeSidePrev);
			}
			else
			{
				hedgeSideBegin = hedgeSide;
			}
			newFaces.pushBack(face);
			hedgeSidePrev = hedgeSide;
		}
		hedgeSideBegin->next->setTwin(hedgeSidePrev);
	}

	//////////////////////////////////////////////////////////////////////////
	// resolve unclaimed points
	void QuickHull::resolveUnclaimedPoints(const QuickHullFaceArray& newFaces)
	{
		for (PxU32 i = 0; i < mUnclaimedPoints.size(); i++)
		{
			QuickHullVertex* vtx = mUnclaimedPoints[i];

			float maxDist = mTolerance;
			QuickHullFace* maxFace = NULL;
			for (PxU32 j = 0; j < newFaces.size(); j++)
			{
				const QuickHullFace& newFace = *newFaces[j];
				if (newFace.state == QuickHullFace::eVISIBLE)
				{
					const float dist = newFace.distanceToPlane(vtx->point);
					if (dist > maxDist)
					{
						maxDist = dist;
						maxFace = newFaces[j];
					}
				}
			}
			if (maxFace != NULL)
			{
				addPointToFace(*maxFace, vtx, maxDist);
			}
		}

		mUnclaimedPoints.clear();
	}

	//////////////////////////////////////////////////////////////////////////	
	// helper struct for hull expand point
	struct ExpandPoint
	{
		PxPlane		plane[3];		// the 3 planes that will give us the point
		PxU32		planeIndex[3]; // index of the planes for identification		

		bool operator==(const ExpandPoint& expPoint) const
		{
			if (expPoint.planeIndex[0] == planeIndex[0] && expPoint.planeIndex[1] == planeIndex[1] &&
				expPoint.planeIndex[2] == planeIndex[2])
				return true;
			else
				return false;
}	
	};

//////////////////////////////////////////////////////////////////////////
	// gets the half edge neighbors and form the expand point
	void getExpandPoint(const QuickHullHalfEdge& he, ExpandPoint& expandPoint, const Ps::Array<PxU32>* translationTable = NULL)
	{
		// set the first 2 - the edge face and the twin face
		expandPoint.planeIndex[0] = (translationTable) ? ((*translationTable)[he.face->index]) : (he.face->index);

		PxU32 index = translationTable ? ((*translationTable)[he.twin->face->index]) : he.twin->face->index;
		if (index < expandPoint.planeIndex[0])
		{
			expandPoint.planeIndex[1] = expandPoint.planeIndex[0];
			expandPoint.planeIndex[0] = index;
		}
		else
		{
			expandPoint.planeIndex[1] = index;
		}

		// now the 3rd one is the next he twin index
		index = translationTable ? (*translationTable)[he.next->twin->face->index] : he.next->twin->face->index;
		if (index < expandPoint.planeIndex[0])
		{
			expandPoint.planeIndex[2] = expandPoint.planeIndex[1];
			expandPoint.planeIndex[1] = expandPoint.planeIndex[0];
			expandPoint.planeIndex[0] = index;
		}
		else
		{
			if (index < expandPoint.planeIndex[1])
			{
				expandPoint.planeIndex[2] = expandPoint.planeIndex[1];
				expandPoint.planeIndex[1] = index;
			}
			else
			{
				expandPoint.planeIndex[2] = index;
			}
		}
	}

	//////////////////////////////////////////////////////////////////////////
	// adds the expand point, don't add similar point
	void addExpandPoint(const ExpandPoint& expandPoint, Ps::Array<ExpandPoint>& expandPoints)
	{
		for (PxU32 i = expandPoints.size(); i--;)
		{
			if (expandPoint == expandPoints[i])
			{
				return;
			}
		}

		expandPoints.pushBack(expandPoint);
	}

	//////////////////////////////////////////////////////////////////////////
	// helper for 3 planes intersection
	static PxVec3 threePlaneIntersection(const PxPlane &p0, const PxPlane &p1, const PxPlane &p2)
	{
		PxMat33 mp = (PxMat33(p0.n, p1.n, p2.n)).getTranspose();
		PxMat33 mi = (mp).getInverse();
		PxVec3 b(p0.d, p1.d, p2.d);
		return -mi.transform(b);
	}
}	

//////////////////////////////////////////////////////////////////////////

QuickHullConvexHullLib::QuickHullConvexHullLib(const PxConvexMeshDesc& desc, const PxCookingParams& params)
	: ConvexHullLib(desc, params),mQuickHull(NULL), mCropedConvexHull(NULL), mOutMemoryBuffer(NULL), mFaceTranslateTable(NULL)
{
	mQuickHull = PX_NEW_TEMP(local::QuickHull)(params, desc);
	mQuickHull->preallocate(desc.points.count);
}

//////////////////////////////////////////////////////////////////////////

QuickHullConvexHullLib::~QuickHullConvexHullLib()
{
	mQuickHull->releaseHull();
	PX_DELETE(mQuickHull);

	if(mCropedConvexHull)
	{
		PX_DELETE(mCropedConvexHull);
	}

	PX_FREE(mOutMemoryBuffer);
	mFaceTranslateTable = NULL;  // memory is a part of mOutMemoryBuffer
}

//////////////////////////////////////////////////////////////////////////
// create the hull
// 1. clean the input vertices
// 2. check we can construct the simplex, if not expand the input verts
// 3. prepare the quickhull - preallocate, parse input verts
// 4. construct the hull
// 5. post merge faces if limit not reached
// 6. if limit reached, expand the hull
PxConvexMeshCookingResult::Enum QuickHullConvexHullLib::createConvexHull()
{
	PxConvexMeshCookingResult::Enum res = PxConvexMeshCookingResult::eFAILURE;

	PxU32 vcount = mConvexMeshDesc.points.count;
	if ( vcount < 8 ) 
		vcount = 8;

	PxVec3* outvsource  = reinterpret_cast<PxVec3*> (PX_ALLOC_TEMP( sizeof(PxVec3)*vcount, "PxVec3"));
	PxVec3 scale;	
	PxVec3 center;
	PxU32 outvcount;

	// cleanup the vertices first
	if(mConvexMeshDesc.flags & PxConvexFlag::eSHIFT_VERTICES)
	{
		if(!shiftAndcleanupVertices(mConvexMeshDesc.points.count, reinterpret_cast<const PxVec3*> (mConvexMeshDesc.points.data), mConvexMeshDesc.points.stride,
			outvcount, outvsource, scale, center ))
		{
			PX_FREE(outvsource);
			return res;
		}
	}
	else
	{
		if(!cleanupVertices(mConvexMeshDesc.points.count, reinterpret_cast<const PxVec3*> (mConvexMeshDesc.points.data), mConvexMeshDesc.points.stride,
			outvcount, outvsource, scale, center ))
		{
			PX_FREE(outvsource);
			return res;
		}
	}

	// scale vertices back to their original size.
	// move the vertices to the origin
	for (PxU32 i=0; i< outvcount; i++)
	{
		PxVec3& v = outvsource[i];
		v.multiply(scale);
	}

	local::QuickHullVertex minimumVertex[3];
	local::QuickHullVertex maximumVertex[3];
	float tolerance;
	float planeTolerance;
	bool canReuse = cleanupForSimplex(outvsource, outvcount, &minimumVertex[0], &maximumVertex[0], tolerance, planeTolerance);

	mQuickHull->parseInputVertices(outvsource,outvcount);

	if(canReuse)
	{
		mQuickHull->setPrecomputedMinMax(minimumVertex, maximumVertex, tolerance, planeTolerance);
	}

	local::QuickHullResult::Enum qhRes = mQuickHull->buildHull();

	switch(qhRes)
	{
	case local::QuickHullResult::eZERO_AREA_TEST_FAILED:
		res = PxConvexMeshCookingResult::eZERO_AREA_TEST_FAILED;
		break;
	case local::QuickHullResult::eSUCCESS:
		mQuickHull->postMergeHull();
		res = PxConvexMeshCookingResult::eSUCCESS;		
		break;
	case local::QuickHullResult::ePOLYGONS_LIMIT_REACHED:
		if(mQuickHull->getNbHullVerts() > mConvexMeshDesc.vertexLimit)
		{
			// expand the hull
			if(mConvexMeshDesc.flags & PxConvexFlag::ePLANE_SHIFTING)
				res = expandHull();
			else
				res = expandHullOBB();
		}
		res = PxConvexMeshCookingResult::ePOLYGONS_LIMIT_REACHED;		
		break;
	case local::QuickHullResult::eVERTEX_LIMIT_REACHED:
		{
			// expand the hull
			if(mConvexMeshDesc.flags & PxConvexFlag::ePLANE_SHIFTING)
				res = expandHull();
			else
				res = expandHullOBB();
		}
		break;
	case local::QuickHullResult::eFAILURE:
		break;
	};

	// check if we need to build GRB compatible mesh
	// if hull was cropped we already have a compatible mesh, if not check 
	// the max verts per face
	if((mConvexMeshDesc.flags & PxConvexFlag::eGPU_COMPATIBLE) && !mCropedConvexHull &&
		res == PxConvexMeshCookingResult::eSUCCESS)
	{
		PX_ASSERT(mQuickHull);
		// if we hit the vertex per face limit, expand the hull by cropping OBB
		if(mQuickHull->maxNumVertsPerFace() > gpuMaxVertsPerFace)
		{
			res = expandHullOBB();
		}
	}

	PX_FREE(outvsource);
	return res;
}

//////////////////////////////////////////////////////////////////////////
// fixup the input vertices to be not colinear or coplanar for the initial simplex find
bool QuickHullConvexHullLib::cleanupForSimplex(PxVec3* vertices, PxU32 vertexCount, local::QuickHullVertex* minimumVertex, 
	local::QuickHullVertex* maximumVertex, float& tolerance, float& planeTolerance)
{
	bool retVal = true;

	for (PxU32 i = 0; i < 3; i++)
	{
		minimumVertex[i].point = vertices[0];
		minimumVertex[i].index = 0;
		maximumVertex[i].point = vertices[0];
		maximumVertex[i].index = 0;

	}

	PxVec3 max = vertices[0];
	PxVec3 min = vertices[0];

	// get the max min vertices along the x,y,z
	for (PxU32 i = 1; i < vertexCount; i++)
	{
		const PxVec3& testPoint = vertices[i];		
		if (testPoint.x > max.x)
		{
			max.x = testPoint.x;
			maximumVertex[0].point = testPoint;
			maximumVertex[0].index = i;
		}
		else if (testPoint.x < min.x)
		{
			min.x = testPoint.x;
			minimumVertex[0].point = testPoint;
			minimumVertex[0].index = i;
		}

		if (testPoint.y > max.y)
		{
			max.y = testPoint.y;
			maximumVertex[1].point = testPoint;
			maximumVertex[1].index = i;
		}
		else if (testPoint.y < min.y)
		{
			min.y = testPoint.y;
			minimumVertex[1].point = testPoint;
			minimumVertex[1].index = i;
		}

		if (testPoint.z > max.z)
		{
			max.z = testPoint.z;
			maximumVertex[2].point = testPoint;
			maximumVertex[2].index = i;
		}
		else if (testPoint.z < min.z)
		{
			min.z = testPoint.z;
			minimumVertex[2].point = testPoint;
			minimumVertex[2].index = i;
		}
	}

	const float sizeTol = (max.x-min.x + max.y - min.y + max.z - min.z)*0.5f;
	tolerance = PxMax(local::PLANE_THICKNES * sizeTol, local::PLANE_THICKNES);
	planeTolerance = PxMax(mCookingParams.planeTolerance *sizeTol, mCookingParams.planeTolerance);

	float fmax = 0;
	PxU32 imax = 0;

	for (PxU32 i = 0; i < 3; i++)
	{
		float diff = (maximumVertex[i].point)[i] - (minimumVertex[i].point)[i];
		if (diff > fmax)
		{
			fmax = diff;
			imax = i;
		}
	}

	PxVec3 simplex[4];

	// set first two vertices to be those with the greatest
	// one dimensional separation
	simplex[0] = maximumVertex[imax].point;
	simplex[1] = minimumVertex[imax].point;

	// set third vertex to be the vertex farthest from
	// the line between simplex[0] and simplex[1]
	PxVec3 normal;
	float maxDist = 0;
	imax = 0;
	PxVec3 u01 = (simplex[1] - simplex[0]);
	u01.normalize();

	for (PxU32 i = 0; i < vertexCount; i++)
	{		
		const PxVec3& testPoint = vertices[i];
		const PxVec3 diff = testPoint - simplex[0];
		const PxVec3 xprod = u01.cross(diff);
		const float lenSqr = xprod.magnitudeSquared();
		if (lenSqr > maxDist)
		{
			maxDist = lenSqr;
			simplex[2] = testPoint;
			normal = xprod;
			imax = i;
		}
	}

	if (PxSqrt(maxDist) < tolerance)
	{
		// points are collinear, we have to move the point further
		PxVec3 u02 = simplex[2] - simplex[0];		
		float fT = u02.dot(u01);
		const float sqrLen = u01.magnitudeSquared();
		fT /= sqrLen;
		PxVec3 n = u02 - fT*u01;
		n.normalize();
		const PxVec3 mP = simplex[2] + n * tolerance;
		simplex[2] = mP;
		vertices[imax] = mP;		
		retVal = false;
	}
	normal.normalize();

	// set the forth vertex in the normal direction	
	float d0 = simplex[2].dot(normal);
	maxDist = 0.0f;
	imax = 0;
	for (PxU32 i = 0; i < vertexCount; i++)
	{		
		const PxVec3& testPoint = vertices[i];
		float dist = PxAbs(testPoint.dot(normal) - d0);
		if (dist > maxDist)
		{
			maxDist = dist;
			simplex[3] = testPoint;
			imax = i;
		}
	}

	if (PxAbs(maxDist) < tolerance)
	{
		float dist = (vertices[imax].dot(normal) - d0);
		if (dist > 0)
			vertices[imax] = vertices[imax] + normal * tolerance;
		else
			vertices[imax] = vertices[imax] - normal * tolerance;
		retVal = false;
	}

	return retVal;
}

//////////////////////////////////////////////////////////////////////////
// expand the hull with the from the limited triangles set
// expand hull will do following steps:
//	1. get expand points from hull that form the best hull with given vertices
//  2. expand the planes to have all vertices inside the planes volume
//  3. compute new points by 3 adjacency planes intersections
//  4. take those points and create the hull from them
PxConvexMeshCookingResult::Enum QuickHullConvexHullLib::expandHull()
{	
	Ps::Array<local::ExpandPoint> expandPoints;	
	expandPoints.reserve(mQuickHull->mNumVertices);

	// go over faces and gather expand points
	for (PxU32 i = 0; i < mQuickHull->mHullFaces.size(); i++)
	{
		const local::QuickHullFace& face = *mQuickHull->mHullFaces[i];
		if(face.state == local::QuickHullFace::eVISIBLE)
		{
			local::ExpandPoint expandPoint;
			local::QuickHullHalfEdge* he = face.edge;
			local::getExpandPoint(*he, expandPoint);
			local::addExpandPoint(expandPoint, expandPoints);
			he = he->next;
			while (he != face.edge)
			{
				local::getExpandPoint(*he, expandPoint);
				local::addExpandPoint(expandPoint, expandPoints);
				he = he->next;
			}
		}
	}


	// go over the planes now and expand them	
	for(PxU32 iVerts=0;iVerts< mQuickHull->mNumVertices;iVerts++)
	{
		const local::QuickHullVertex& vertex = mQuickHull->mVerticesList[iVerts];

		for (PxU32 i = 0; i < mQuickHull->mHullFaces.size(); i++)
		{
			local::QuickHullFace& face = *mQuickHull->mHullFaces[i];
			if(face.state == local::QuickHullFace::eVISIBLE)
			{				
				const float dist = face.distanceToPlane(vertex.point);
				if(dist > 0 && dist > face.expandOffset)
				{
					face.expandOffset = dist;
				}
			}
		}	
	}

	// fill the expand points planes
	for(PxU32 i=0;i<expandPoints.size();i++)
	{
		local::ExpandPoint& expandPoint = expandPoints[i];
		for (PxU32 k = 0; k < 3; k++)
		{
			const local::QuickHullFace& face = *mQuickHull->mFreeFaces.getItem(expandPoint.planeIndex[k]);
			PX_ASSERT(face.index == expandPoint.planeIndex[k]);
			PxPlane plane;
			plane.n = face.normal;
			plane.d = -face.planeOffset;
			if(face.expandOffset > 0.0f)
				plane.d -= face.expandOffset;
			expandPoint.plane[k] = plane;
		}		
	}	

	// now find the plane intersection
	PX_ALLOCA(vertices,PxVec3,expandPoints.size());
	for(PxU32 i=0;i<expandPoints.size();i++)
	{
		local::ExpandPoint& expandPoint = expandPoints[i];
		vertices[i] = local::threePlaneIntersection(expandPoint.plane[0],expandPoint.plane[1],expandPoint.plane[2]);
	}

	// construct again the hull from the new points
	local::QuickHull* newHull = PX_NEW_TEMP(local::QuickHull)(mQuickHull->mCookingParams, mQuickHull->mConvexDesc);		
	newHull->preallocate(expandPoints.size());
	newHull->parseInputVertices(vertices,expandPoints.size());

	local::QuickHullResult::Enum qhRes = newHull->buildHull();
	switch(qhRes)
	{
	case local::QuickHullResult::eZERO_AREA_TEST_FAILED:
		{
			newHull->releaseHull();
			PX_DELETE(newHull);
			return PxConvexMeshCookingResult::eZERO_AREA_TEST_FAILED;
		}		
	case local::QuickHullResult::eSUCCESS:
	case local::QuickHullResult::eVERTEX_LIMIT_REACHED:
	case local::QuickHullResult::ePOLYGONS_LIMIT_REACHED:
		{
			mQuickHull->releaseHull();
			PX_DELETE(mQuickHull);
			mQuickHull = newHull;
		}
		break;
	case local::QuickHullResult::eFAILURE:
		{
			newHull->releaseHull();
			PX_DELETE(newHull);
			return PxConvexMeshCookingResult::eFAILURE;
		}
	};

	return PxConvexMeshCookingResult::eSUCCESS;
}

//////////////////////////////////////////////////////////////////////////
// expand the hull from the limited triangles set
// 1. collect all planes
// 2. create OBB from the input verts
// 3. slice the OBB with the planes
// 5. iterate till vlimit is reached
PxConvexMeshCookingResult::Enum QuickHullConvexHullLib::expandHullOBB()
{	
	Ps::Array<PxPlane> expandPlanes;
	expandPlanes.reserve(mQuickHull->mHullFaces.size());	

	// collect expand planes
	for (PxU32 i = 0; i < mQuickHull->mHullFaces.size(); i++)
	{
		local::QuickHullFace& face = *mQuickHull->mHullFaces[i];
		if (face.state == local::QuickHullFace::eVISIBLE)
		{
			PxPlane plane;
			plane.n = face.normal;
			plane.d = -face.planeOffset;
			if (face.expandOffset > 0.0f)
				plane.d -= face.expandOffset;

			expandPlanes.pushBack(plane);
		}
	}


	PxTransform obbTransform;
	PxVec3 sides;

	// compute the OBB
	PxConvexMeshDesc convexDesc;
	fillConvexMeshDescFromQuickHull(convexDesc);
	convexDesc.flags = mConvexMeshDesc.flags;
	computeOBBFromConvex(convexDesc, sides, obbTransform);

	// free the memory used for the convex mesh desc
	PX_FREE_AND_RESET(mOutMemoryBuffer);
	mFaceTranslateTable = NULL;

	// crop the OBB
	PxU32 maxplanes = PxMin(PxU32(256), expandPlanes.size());

	ConvexHull* c = PX_NEW_TEMP(ConvexHull)(sides*0.5f,obbTransform, expandPlanes);	

	const float planeTolerance = mQuickHull->mPlaneTolerance;
	const float epsilon = mQuickHull->mTolerance;

	PxI32 k;
	while (maxplanes-- && (k = c->findCandidatePlane(planeTolerance, epsilon)) >= 0)
	{
		ConvexHull* tmp = c;
		c = convexHullCrop(*tmp, expandPlanes[PxU32(k)], planeTolerance);
		if (c == NULL)
		{
			c = tmp;
			break;
		} // might want to debug this case better!!!
		if (!c->assertIntact(planeTolerance))
		{
			PX_DELETE(c);
			c = tmp;
			break;
		} // might want to debug this case better too!!!

		// check for vertex limit
		if (c->getVertices().size() > mConvexMeshDesc.vertexLimit)
		{
			PX_DELETE(c);
			c = tmp;
			maxplanes = 0;
			break;
		}
		// check for vertex limit per face if necessary, GRB supports max 32 verts per face
		if ((mConvexMeshDesc.flags & PxConvexFlag::eGPU_COMPATIBLE) && c->maxNumVertsPerFace() > gpuMaxVertsPerFace)
		{ 
			PX_DELETE(c);
			c = tmp;
			maxplanes = 0;
			break;
		}
		PX_DELETE(tmp);
	}

	PX_ASSERT(c->assertIntact(planeTolerance));

	mCropedConvexHull = c;

	return PxConvexMeshCookingResult::eSUCCESS;
}

//////////////////////////////////////////////////////////////////////////

bool QuickHullConvexHullLib::createEdgeList(const PxU32 nbIndices, const PxU8* indices, PxU8** outHullDataFacesByEdges8, PxU16** outEdgeData16, PxU16** outEdges)
{
	// if we croped hull, we dont have the edge information, early exit
	if (mCropedConvexHull)
		return false;

	PX_ASSERT(mQuickHull);

	// Make sure we did recieved empty buffers
	PX_ASSERT(*outHullDataFacesByEdges8 == NULL);
	PX_ASSERT(*outEdges == NULL);
	PX_ASSERT(*outEdgeData16 == NULL);

	// Allocated the out bufferts
	PxU8* hullDataFacesByEdges8 = PX_NEW(PxU8)[nbIndices];
	PxU16* edges = PX_NEW(PxU16)[nbIndices];
	PxU16* edgeData16 = PX_NEW(PxU16)[nbIndices];

	*outHullDataFacesByEdges8 = hullDataFacesByEdges8;
	*outEdges = edges;
	*outEdgeData16 = edgeData16;

	PxU16 edgeIndex = 0;
	PxU32 edgeOffset = 0;
	for(PxU32 i = 0; i < mQuickHull->mNumHullFaces; i++)
	{
		const local::QuickHullFace& face = *mQuickHull->mHullFaces[mFaceTranslateTable[i]];

		// Face must be visible
		PX_ASSERT(face.state == local::QuickHullFace::eVISIBLE);

		// parse the edges
		const PxU32 startEdgeOffset = edgeOffset;
		local::QuickHullHalfEdge* hedge = face.edge;
		do
		{
			// check if hedge has been stored
			if(hedge->edgeIndex == 0xFFFFFFFF)
			{
				edges[edgeIndex*2] = indices[edgeOffset];
				edges[edgeIndex*2 + 1] = indices[(hedge->next != face.edge) ? edgeOffset + 1 : startEdgeOffset];

				hullDataFacesByEdges8[edgeIndex*2] = hedge->face->outIndex;
				hullDataFacesByEdges8[edgeIndex*2 + 1] = hedge->next->twin->face->outIndex;

				edgeData16[edgeOffset] = edgeIndex;

				hedge->edgeIndex = edgeIndex;
				hedge->next->twin->prev->edgeIndex = edgeIndex;

				edgeIndex++;
			}
			else
			{
				edgeData16[edgeOffset] = Ps::to16(hedge->edgeIndex);
			}

			hedge = hedge->next;
			edgeOffset++;
		} while (hedge != face.edge);
	}

	return true;
}

//////////////////////////////////////////////////////////////////////////
// fill the descriptor with computed verts, indices and polygons
void QuickHullConvexHullLib::fillConvexMeshDesc(PxConvexMeshDesc& desc)
{
	if (mCropedConvexHull)
		fillConvexMeshDescFromCroppedHull(desc);
	else
		fillConvexMeshDescFromQuickHull(desc);

	if(mConvexMeshDesc.flags & PxConvexFlag::eSHIFT_VERTICES)
		shiftConvexMeshDesc(desc);
}

//////////////////////////////////////////////////////////////////////////
// fill the descriptor with computed verts, indices and polygons from quickhull convex
void QuickHullConvexHullLib::fillConvexMeshDescFromQuickHull(PxConvexMeshDesc& desc)
{
	// get the number of indices needed
	PxU32 numIndices = 0;
	PxU32 numFaces = mQuickHull->mHullFaces.size();
	PxU32 numFacesOut = 0;
	PxU32 largestFace = 0;			// remember the largest face, we store it as the first face, required for GRB test (max 32 vers per face supported)
	for (PxU32 i = 0; i < numFaces; i++)
	{
		const local::QuickHullFace& face = *mQuickHull->mHullFaces[i];
		if(face.state == local::QuickHullFace::eVISIBLE)
		{
			numFacesOut++;
			numIndices += face.numEdges;
			if(face.numEdges > mQuickHull->mHullFaces[largestFace]->numEdges)
				largestFace = i;
		}
	}

	// allocate out buffers
	const PxU32 indicesBufferSize = sizeof(PxU32)*numIndices;
	const PxU32 verticesBufferSize = sizeof(PxVec3)*(mQuickHull->mNumVertices + 1);
	const PxU32 facesBufferSize = sizeof(PxHullPolygon)*numFacesOut;
	const PxU32 faceTranslationTableSize = sizeof(PxU16)*numFacesOut;
	const PxU32 translationTableSize = sizeof(PxU32)*mQuickHull->mNumVertices;
	const PxU32 bufferMemorySize = indicesBufferSize + verticesBufferSize + facesBufferSize + faceTranslationTableSize + translationTableSize;
	mOutMemoryBuffer = reinterpret_cast<PxU8*>(PX_ALLOC_TEMP(bufferMemorySize, "ConvexMeshDesc"));

	PxU32* indices = reinterpret_cast<PxU32*> (mOutMemoryBuffer);
	PxVec3* vertices = reinterpret_cast<PxVec3*> (mOutMemoryBuffer + indicesBufferSize);
	PxHullPolygon* polygons = reinterpret_cast<PxHullPolygon*> (mOutMemoryBuffer + indicesBufferSize + verticesBufferSize);
	mFaceTranslateTable = reinterpret_cast<PxU16*> (mOutMemoryBuffer + indicesBufferSize + verticesBufferSize + facesBufferSize);
	PxI32* translateTable = reinterpret_cast<PxI32*> (mOutMemoryBuffer + indicesBufferSize + verticesBufferSize + facesBufferSize + faceTranslationTableSize);
	PxMemSet(translateTable,-1,mQuickHull->mNumVertices*sizeof(PxU32));

	// go over the hullPolygons and mark valid vertices, create translateTable
	PxU32 numVertices = 0;
	for (PxU32 i = 0; i < numFaces; i++)
	{
		const local::QuickHullFace& face = *mQuickHull->mHullFaces[i];		
		if(face.state == local::QuickHullFace::eVISIBLE)
		{
			local::QuickHullHalfEdge* he = face.edge;
			if(translateTable[he->tail.index] == -1)
			{
				vertices[numVertices] = he->tail.point;
				translateTable[he->tail.index] = PxI32(numVertices);
				numVertices++;				
			}
			he = he->next;
			while (he != face.edge)
			{
				if(translateTable[he->tail.index] == -1)
				{
					vertices[numVertices] = he->tail.point;
					translateTable[he->tail.index] = PxI32(numVertices);
					numVertices++;				
				}
				he = he->next;
			}
		}
	}

	
	desc.points.count = numVertices;
	desc.points.data = vertices;
	desc.points.stride = sizeof(PxVec3);

	desc.indices.count = numIndices;
	desc.indices.data = indices;
	desc.indices.stride = sizeof(PxU32);

	desc.polygons.count = numFacesOut;	
	desc.polygons.data = polygons;
	desc.polygons.stride = sizeof(PxHullPolygon);	

	PxU16 indexOffset = 0;
	numFacesOut = 0;
	for (PxU32 i = 0; i < numFaces; i++)
	{	
		// faceIndex - store the largest face first then the rest
		PxU32 faceIndex;
		if(i == 0)
		{
			faceIndex = largestFace;
		}
		else
		{
			faceIndex = (i == largestFace) ? 0 : i;
		}

		local::QuickHullFace& face = *mQuickHull->mHullFaces[faceIndex];
		if(face.state == local::QuickHullFace::eVISIBLE)
		{
			//create index data
			local::QuickHullHalfEdge* he = face.edge;
			PxU32 index = 0;
			he->edgeIndex = 0xFFFFFFFF;
			indices[index + indexOffset] = PxU32(translateTable[he->tail.index]);
			index++;
			he = he->next;
			while (he != face.edge)
			{
				indices[index + indexOffset] = PxU32(translateTable[he->tail.index]);
				index++;
				he->edgeIndex = 0xFFFFFFFF;
				he = he->next;
			}

			// create polygon
			PxHullPolygon polygon;		
			polygon.mPlane[0] = face.normal[0];
			polygon.mPlane[1] = face.normal[1];
			polygon.mPlane[2] = face.normal[2];
			polygon.mPlane[3] = -face.planeOffset;

			polygon.mIndexBase = indexOffset;
			polygon.mNbVerts = face.numEdges;
			indexOffset += face.numEdges;
			polygons[numFacesOut] = polygon;
			mFaceTranslateTable[numFacesOut] = Ps::to16(faceIndex);
			face.outIndex = Ps::to8(numFacesOut);
			numFacesOut++;
		}
	}

	PX_ASSERT(mQuickHull->mNumHullFaces == numFacesOut);	
}

//////////////////////////////////////////////////////////////////////////
// fill the desc from cropped hull data
void QuickHullConvexHullLib::fillConvexMeshDescFromCroppedHull(PxConvexMeshDesc& outDesc)
{
	PX_ASSERT(mCropedConvexHull);

	// allocate the output buffers
	const PxU32 numIndices = mCropedConvexHull->getEdges().size();
	const PxU32 numPolygons = mCropedConvexHull->getFacets().size();
	const PxU32 numVertices = mCropedConvexHull->getVertices().size();
	const PxU32 indicesBufferSize = sizeof(PxU32)*numIndices;
	const PxU32 facesBufferSize = sizeof(PxHullPolygon)*numPolygons;
	const PxU32 verticesBufferSize = sizeof(PxVec3)*(numVertices + 1); // allocate additional vec3 for V4 safe load in VolumeInteration
	const PxU32 bufferMemorySize = indicesBufferSize + verticesBufferSize + facesBufferSize;
	mOutMemoryBuffer = reinterpret_cast<PxU8*>(PX_ALLOC_TEMP(bufferMemorySize, "ConvexMeshDesc"));

	// parse the hullOut and fill the result with vertices and polygons
	PxU32* indicesOut = reinterpret_cast<PxU32*> (mOutMemoryBuffer);	
	PxHullPolygon* polygonsOut = reinterpret_cast<PxHullPolygon*> (mOutMemoryBuffer + indicesBufferSize);	
	PxVec3* vertsOut = reinterpret_cast<PxVec3*> (mOutMemoryBuffer + indicesBufferSize + facesBufferSize);	
	PxMemCopy(vertsOut, mCropedConvexHull->getVertices().begin(), sizeof(PxVec3)*numVertices);

	PxU32 i = 0;
	PxU32 k = 0;
	PxU32 j = 1;
	while (i < mCropedConvexHull->getEdges().size())
	{
		j = 1;
		PxHullPolygon& polygon = polygonsOut[k];
		// get num indices per polygon
		while (j + i < mCropedConvexHull->getEdges().size() && mCropedConvexHull->getEdges()[i].p == mCropedConvexHull->getEdges()[i + j].p)
		{
			j++;
		}
		polygon.mNbVerts = Ps::to16(j);
		polygon.mIndexBase = Ps::to16(i);

		// get the plane
		polygon.mPlane[0] = mCropedConvexHull->getFacets()[k].n[0];
		polygon.mPlane[1] = mCropedConvexHull->getFacets()[k].n[1];
		polygon.mPlane[2] = mCropedConvexHull->getFacets()[k].n[2];

		polygon.mPlane[3] = mCropedConvexHull->getFacets()[k].d;

		while (j--)
		{
			indicesOut[i] = mCropedConvexHull->getEdges()[i].v;
			i++;
		}
		k++;
	}

	PX_ASSERT(k == mCropedConvexHull->getFacets().size());

	outDesc.indices.count = numIndices;
	outDesc.indices.stride = sizeof(PxU32);
	outDesc.indices.data = indicesOut;

	outDesc.points.count = numVertices;
	outDesc.points.stride = sizeof(PxVec3);
	outDesc.points.data = vertsOut;

	outDesc.polygons.count = numPolygons;
	outDesc.polygons.stride = sizeof(PxHullPolygon);
	outDesc.polygons.data = polygonsOut;

	swapLargestFace(outDesc);
}

