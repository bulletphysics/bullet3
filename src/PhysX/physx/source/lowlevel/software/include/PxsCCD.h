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

#include "Ps.h"
#include "PxGeometry.h"
#include "GuCCDSweepConvexMesh.h"
#include "PsHashMap.h"
#include "PxsIslandSim.h"

#ifndef PXS_CCD_H
#define PXS_CCD_H

#define CCD_DEBUG_PRINTS						0
#define CCD_POST_DEPENETRATE_DIST				0.001f
#define CCD_ROTATION_LOCKING					0
#define CCD_MIN_TIME_LEFT						0.01f
#define CCD_ANGULAR_IMPULSE						0

#define DEBUG_RENDER_CCD						0

#if CCD_DEBUG_PRINTS
namespace physx {
	extern void printCCDDebug(const char* msg, const PxsRigidBody* atom0, PxGeometryType::Enum g0, bool printPtr = true);
	extern void printShape(PxsRigidBody* atom0, PxGeometryType::Enum g0, const char* annotation, PxReal dt, PxU32 pass, bool printPtr = true);
}
#define PRINTCCDSHAPE(x) printShape x
#define PRINTCCDDEBUG(x) printCCDDebug x
#else
#define PRINTCCDSHAPE(x)
#define PRINTCCDDEBUG(x)
#endif

namespace physx
{

// ------------------------------------------------------------------------------------------------------------
// a fraction of objects will be CCD active so this is dynamic, not a member of PsxRigidBody
// CCD code builds a temporary array of PxsCCDPair objects (allocated in blocks)
// this is done to gather scattered data from memory and also to reduce PxsRidigBody permanent memory footprint
// we have to do it every pass since new CMs can become fast moving after each pass (and sometimes cease to be)
//
struct PxsCCDBody;
class PxsRigidBody;
struct PxsShapeCore;
struct PxsRigidCore;
class PxsContactManager;
class PxsContext;
class PxCCDContactModifyCallback;
class PxcNpThreadContext;

class PxvNphaseImplementationContext;

namespace Dy
{
	class ThresholdStream;
}


/**
\brief structure to represent interactions between a given body and another body.
*/
struct PxsCCDOverlap
{
	//The body the interaction relates to
	PxsCCDBody* mBody;
	//The next interaction in the list
	PxsCCDOverlap* mNext;
};

/**
\brief Temporary CCD representation for a shape.

Stores data about a shape that may be frequently used in CCD. It also stores update counters per-shape that can be compared with the body's update
counter to determine if the shape needs its transforms re-calculated. This avoids us needing to store a list of shapes in a CCD body.
*/
struct PxsCCDShape : public Gu::CCDShape
{
public:
	const PxsShapeCore*			mShapeCore;					//Shape core (can be shared)
	const PxsRigidCore*			mRigidCore;					//Rigid body core
	IG::NodeIndex				mNodeIndex;

	/**
	\brief Returns the world-space pose for this shape
	\param[in] atom The rigid body that this CCD shape is associated with
	*/
	PxTransform										getAbsPose(const PxsRigidBody* atom)			const;
	/**
	\brief Returns the world-space previous pose for this shape
	\param[in] atom The rigid body that this CCD shape is associated with
	*/
	PxTransform										getLastCCDAbsPose(const PxsRigidBody* atom)		const;						
};

/**
\brief Structure to represent a body in the CCD system. 
*/
struct PxsCCDBody
{
	Cm::SpatialVector			mPreSolverVelocity;
	PxU16						mIndex;						//The CCD body's index
	bool						mPassDone;					//Whether it has been processed in the current CCD pass
	bool						mHasAnyPassDone;			//Whether this body was influenced by any passes
	PxReal						mTimeLeft;					//CCD time left to elapse (normalized in range 0-1)
	PxsRigidBody*				mBody;						//The rigid body 
	PxsCCDOverlap*				mOverlappingObjects;		//A list of overlapping bodies for island update
	PxU32						mUpdateCount;				//How many times this body has eben updated in the CCD. This is correlated with CCD shapes' update counts.
	PxU32						mNbInteractionsThisPass;	//How many interactions this pass

	

	/**
	\brief Returns the CCD body's index.
	\return The CCD body's index.
	*/
	PX_FORCE_INLINE PxU32 getIndex() const { return mIndex; }

	/**
	\brief Tests whether this body has already registered an overlap with a given body.
	\param[in] body The body to test against.
	\return Whether this body has already registered an overlap with a given body.
	*/
	bool overlaps(PxsCCDBody* body) const
	{
		PxsCCDOverlap* overlaps = mOverlappingObjects;

		while(overlaps)
		{
			if(overlaps->mBody == body)
				return true;
			overlaps = overlaps->mNext;
		}
		return false;
	}

	/**
	\brief Registers an overlap with a given body
	\param[in] overlap The CCD overlap to register.
	*/
	void addOverlap(PxsCCDOverlap* overlap)
	{
		overlap->mNext = mOverlappingObjects;
		mOverlappingObjects = overlap;
	}

};

/**
\brief a container class used in the CCD that minimizes frequency of hitting the allocator.

This class stores a set of blocks of memory. It is effectively an array that resizes more efficiently because it doesn't need to 
reallocate an entire buffer and copy data.
*/
template<typename T, int BLOCK_SIZE>
struct PxsCCDBlockArray
{
	/**
	\brief A block of data
	*/
	struct Block : Ps::UserAllocated { T items[BLOCK_SIZE]; };
	/**
	\brief A header for a block of data.
	*/
	struct BlockInfo
	{
		Block* block;
		PxU32 count; // number of elements in this block
		BlockInfo(Block* aBlock, PxU32 aCount) : block(aBlock), count(aCount) {}
	};
	/*
	\brief An array of block headers
	*/
	Ps::Array<BlockInfo> blocks;
	/**
	\brief The current block.
	*/
	PxU32 currentBlock;

	/**
	\brief Constructor
	*/
	PxsCCDBlockArray() : currentBlock(0)
	{
		blocks.pushBack(BlockInfo(PX_NEW(Block), 0));
	}

	/**
	\brief Destructor
	*/
	~PxsCCDBlockArray()
	{
		for (PxU32 i = 0; i < blocks.size(); i++)
		{
			PX_DELETE(blocks[i].block);
		}
		currentBlock = 0;
	}

	/**
	\brief Clears this block array.
	\note This clear function also deletes all additional blocks
	*/
	void clear()
	{
		for (PxU32 i = 0; i < blocks.size(); i++)
		{
			PX_DELETE(blocks[i].block);
		}
		blocks.clear();
		blocks.pushBack(BlockInfo(PX_NEW(Block), 0)); // at least one block is expected to always be present in the array
		currentBlock = 0;
	}

	/**
	\brief Clears this block array but does not release the memory.
	*/
	void clear_NoDelete()
	{
		currentBlock = 0;
		blocks[0].count = 0;
	}

	/**
	\brief Push a new element onto the back of the block array
	\return The new element
	*/
	T& pushBack()
	{
		PxU32 numBlocks = blocks.size();
		if (blocks[currentBlock].count == BLOCK_SIZE)
		{
			if((currentBlock + 1) == numBlocks)
			{
				blocks.pushBack(BlockInfo(PX_NEW(Block), 0));
				numBlocks ++;
			}
			currentBlock++;
			blocks[currentBlock].count = 0;
		}
		const PxU32 count = blocks[currentBlock].count ++;

		return blocks[currentBlock].block->items[count];
	}

	/**
	\brief Pushes a new element onto the back of this array, intitializing it to match the data
	\param data The data to initialize the new element to
	\return The new element
	*/
	T& pushBack(T& data)
	{
		PxU32 numBlocks = blocks.size();
		if (blocks[currentBlock].count == BLOCK_SIZE)
		{
			if((currentBlock + 1) == numBlocks)
			{
				blocks.pushBack(BlockInfo(PX_NEW(Block), 0));
				numBlocks ++;
			}
			currentBlock++;
			blocks[currentBlock].count = 0;
		}
		const PxU32 count = blocks[currentBlock].count ++;
		blocks[currentBlock].block->items[count] = data;
		return blocks[currentBlock].block->items[count];
	}

	/**
	\brief Pops the last element from the list.
	*/
	void popBack()
	{
		PX_ASSERT(blocks[currentBlock].count > 0);
		if (blocks[currentBlock].count > 1)
			blocks[currentBlock].count --;
		else
		{
			PX_DELETE(blocks[currentBlock].block);
			blocks.popBack();
			currentBlock--;
		}
	}

	/**
	\brief Returns the current size of the array.
	\return The current size of the array.
	*/
	PxU32 size() const
	{
		return (currentBlock)*BLOCK_SIZE + blocks[currentBlock].count;
	}

	/**
	\brief Returns the element at a given index in the array
	\param[in] index The index of the element in the array
	\return The element at a given index in the array.
	*/
	T& operator[] (PxU32 index) const
	{
		PX_ASSERT(index/BLOCK_SIZE < blocks.size());
		PX_ASSERT(index%BLOCK_SIZE < blocks[index/BLOCK_SIZE].count);
		return blocks[index/BLOCK_SIZE].block->items[index%BLOCK_SIZE];
	}
};

/**
\brief A structure to represent a potential CCD interaction between a pair of shapes
*/
struct PxsCCDPair
{
	/**
	\brief Defines whether this is an estimated TOI or an accurate TOI.

	We store pairs in a priority queue based on the TOIs. We use cheap estimates to cull away work and lazily evaluate TOIs. This means that an element in the 
	priority queue may either be an estimate or a precise result.
	*/
	enum E_TOIType
	{
		eEstimate,
		ePrecise
	};
	PxsRigidBody*			mBa0;					// Body A. Can be NULL for statics
	PxsRigidBody*			mBa1;					// Body B. Can be NULL for statics
	PxsCCDShape*			mCCDShape0;				// Shape A
	PxsCCDShape*			mCCDShape1;				// Shape B
	PxVec3					mMinToiNormal;			// The contact normal. Only valid for precise results. On the surface of body/shape A
	PxReal					mMinToi;				// Min TOI. Valid for both precise and estimated results but estimates may be too early (i.e. conservative).
	PxReal					mPenetrationPostStep;	// Valid only for precise sweeps. Only used for initial intersections (i.e. at TOI = 0).
	PxVec3					mMinToiPoint;			// The contact point. Only valid for precise sweep results.
	PxReal					mPenetration;			// The penetration. Only valid for precise sweep results.
	PxsContactManager*		mCm;					// The contact manager.
	PxU32					mIslandId;				// The index of the island this pair is in
	PxGeometryType::Enum	mG0, mG1;				// The geometry types for shapes 0 and 1
	bool					mIsEarliestToiHit;		// Indicates this was the earliest hit for one of the bodies in the pair
	bool					mIsModifiable;			// Indicates whether this contact is modifiable
	PxU32					mFaceIndex;				// The face index. Only valid for precise sweeps involving meshes or heightfields.
	PxU16					mMaterialIndex0;		// The material index for shape 0
	PxU16					mMaterialIndex1;		// The material index for shape 1
	PxReal					mDynamicFriction;		// The dynamic friction coefficient
	PxReal					mStaticFriction;		// The static friction coefficient
	PxReal					mRestitution;			// The restitution coefficient
	PxU32					mEstimatePass;			// The current estimation pass. Used after a sweep hit was found to determine if the pair needs re-estimating.
	PxReal					mAppliedForce;			// The applied force for this pair. Only valid if the pair has been responded to.
	PxReal					mMaxImpulse;			// The maximum impulse to be applied 

	E_TOIType				mToiType;				// The TOI type (estimate, precise).
	bool					mHasFriction;			// Whether we want to simulate CCD friction for this pair

	/**
	\brief Perform a precise sweep for this pair
	\param[in] threadContext The per-thread context
	\param[in] dt The time-step
	\param[in] pass The current CCD pass
	\return The normalized TOI. <=1.0 indicates a hit. Otherwise PX_MAX_REAL.
	*/
	PxReal	sweepFindToi(PxcNpThreadContext& threadContext, PxReal dt, PxU32 pass);
	/**
	\brief Performs a sweep estimation for this pair
	\return The normalized TOI. <= 1.0 indicates a potential hit, otherwise PX_MAX_REAL.
	*/
	PxReal	sweepEstimateToi();
	/**
	\brief Advances this pair to the TOI
	\param[in] dt The time-step
	\param[in] clipTrajectoryToToi Indicates whether we clip the body's trajectory to the end pose. Only done in the final pass
	\return Whether the advance was successful. An advance will be unsuccessful if body bodies were already updated.
	*/
	bool	sweepAdvanceToToi(PxReal dt, bool clipTrajectoryToToi);
	/**
	\brief Updates the transforms of the shapes involved in this pair.
	*/
	void	updateShapes();

};

/**
\brief Block array of CCD bodies
*/
typedef PxsCCDBlockArray<PxsCCDBody, 128> PxsCCDBodyArray;
/**
\brief Block array of CCD pairs
*/
typedef PxsCCDBlockArray<PxsCCDPair, 128> PxsCCDPairArray;
/**
\brief Block array of CCD overlaps
*/
typedef PxsCCDBlockArray<PxsCCDOverlap, 128> PxsCCDOverlapArray;
/**
\brief Block array of CCD shapes
*/
typedef PxsCCDBlockArray<PxsCCDShape, 128> PxsCCDShapeArray;

/**
\brief Pair structure to be able to look-up a rigid body-shape pair in a map
*/
typedef Ps::Pair<const PxsRigidCore*, const PxsShapeCore*> PxsRigidShapePair;


/**
\brief CCD context object.
*/
class PxsCCDContext
{
public:

	/**
	\brief Creates this PxsCCDContext
	*/
	static PxsCCDContext* create(PxsContext* context, Dy::ThresholdStream& dynamicsContext, PxvNphaseImplementationContext& nPhaseContext);

	/**
	\brief Destroys this PxsCCDContext
	*/
	void						destroy();

	/**
	\brief Returns the CCD contact modification callback
	\return The CCD contact modification callback
	*/
	PX_FORCE_INLINE		PxCCDContactModifyCallback*	getCCDContactModifyCallback()					const		{ return mCCDContactModifyCallback;	}
	/**
	\brief Sets the CCD contact modification callback
	\param[in] c The CCD contact modification callback
	*/
	PX_FORCE_INLINE		void						setCCDContactModifyCallback(PxCCDContactModifyCallback* c)	{ mCCDContactModifyCallback = c;	}
	/**
	\brief Returns the maximum number of CCD passes
	\return The maximum number of CCD passes
	*/
	PX_FORCE_INLINE		PxU32					getCCDMaxPasses()				const						{ return mCCDMaxPasses;	}
	/**
	\brief Sets the maximum number of CCD passes
	\param[in] ccdMaxPasses The maximum number of CCD passes
	*/
	PX_FORCE_INLINE		void					setCCDMaxPasses(PxU32 ccdMaxPasses)							{ mCCDMaxPasses = ccdMaxPasses;		}
	/**
	\brief Returns the current CCD pass
	\return The current CCD pass
	*/
	PX_FORCE_INLINE		PxU32					getCurrentCCDPass()				const						{ return miCCDPass;					}
	/**
	\brief Returns The number of swept hits reported
	\return The number of swept hits reported
	*/
	PX_FORCE_INLINE		PxI32					getNumSweepHits()				const						{ return mSweepTotalHits;			}
	/**
	\brief Returns The number of updated bodies
	\return The number of updated bodies in this CCD pass
	*/
	PX_FORCE_INLINE		PxU32					getNumUpdatedBodies()			const						{ return mUpdatedCCDBodies.size(); }
	/**
	\brief Returns The update bodies array
	\return The updated bodies array from this CCD pass
	*/
	PX_FORCE_INLINE		PxsRigidBody*const*			getUpdatedBodies()				const						{ return mUpdatedCCDBodies.begin(); }

	/**
	\brief Returns Clears the updated bodies array
	*/
	PX_FORCE_INLINE		void						clearUpdatedBodies()										{ mUpdatedCCDBodies.forceSize_Unsafe(0); }

	/**
	\brief Runs the CCD contact modification.
	\param[in] contacts The list of modifiable contacts
	\param[in] contactCount The number of contacts
	\param[in] shapeCore0 The first shape core
	\param[in] shapeCore1 The second shape core
	\param[in] rigidCore0 The first rigid core
	\param[in] rigidCore1 The second rigid core
	\param[in] rigid0 The first rigid body
	\param[in] rigid1 The second rigid body
	*/
						void					runCCDModifiableContact(PxModifiableContact* PX_RESTRICT contacts, PxU32 contactCount, const PxsShapeCore* PX_RESTRICT shapeCore0, 
													const PxsShapeCore* PX_RESTRICT shapeCore1, const PxsRigidCore* PX_RESTRICT rigidCore0, const PxsRigidCore* PX_RESTRICT rigidCore1,
													const PxsRigidBody* PX_RESTRICT rigid0, const PxsRigidBody* PX_RESTRICT rigid1);
	
	/**
	\brief Performs a single CCD update
	This occurs after broad phase and is responsible for creating islands, finding the TOI of collisions, filtering contacts, issuing modification callbacks and responding to 
	collisions. At the end of this phase all bodies will have stepper to their first TOI if they were involved in a CCD collision this frame.
	\param[in] dt The timestep to simulate
	\param[in] continuation The continuation task
	\param[in] islandSim The island manager
	\param[in] disableResweep If this is true, we perform a reduced-fidelity CCD approach
	*/
						void					updateCCD(PxReal dt, PxBaseTask* continuation, IG::IslandSim& islandSim, bool disableResweep, PxI32 numFastMovingShapes);

	/**
	\brief Signals the beginning of a CCD multi-pass update
	*/
						void					updateCCDBegin();

	/**
	\brief Resets the CCD contact state in any contact managers that previously had a reported CCD touch. This must be called if CCD update is bypassed for a frame
	*/
						void					resetContactManagers();
	

	

protected:

	/**
	\brief Constructor for PxsCCDContext
	\param[in] context The PxsContext that is associated with this PxsCCDContext.
	*/
	PxsCCDContext(PxsContext* context, Dy::ThresholdStream& thresholdStream, PxvNphaseImplementationContext& nPhaseContext);
	/**
	\brief Destructor for PxsCCDContext
	*/
		~PxsCCDContext();

private:

	
	/**
	\brief Verifies the consistency of the CCD context at the beginning
	*/
						void					verifyCCDBegin();

	/**
	\brief Cleans up after the CCD update has completed
	*/
						void					updateCCDEnd();

	/**
	\brief Spawns the update island tasks after the initial sweep estimates have been performed
	\param[in] continuation The continuation task
	*/
						void					postCCDSweep(PxBaseTask* continuation);
	/**
	\brief Creates contact buffers for CCD contacts. These will be sent to the user in the contact notification.
	\param[in] continuation The continuation task
	*/
						void					postCCDAdvance(PxBaseTask* continuation);
	/**
	\brief The final phase of the CCD task chain. Cleans up after the parallel update/postCCDAdvance stages.
	\param[in] continuation The continuation task
	*/
						void					postCCDDepenetrate(PxBaseTask* continuation);

		typedef Cm::DelegateTask<PxsCCDContext, &PxsCCDContext::postCCDSweep> PostCCDSweepTask;
		typedef Cm::DelegateTask<PxsCCDContext, &PxsCCDContext::postCCDAdvance> PostCCDAdvanceTask;
		typedef Cm::DelegateTask<PxsCCDContext, &PxsCCDContext::postCCDDepenetrate> PostCCDDepenetrateTask;

		PostCCDSweepTask mPostCCDSweepTask;
		PostCCDAdvanceTask mPostCCDAdvanceTask;
		PostCCDDepenetrateTask mPostCCDDepenetrateTask;

		PxCCDContactModifyCallback*					mCCDContactModifyCallback;
		
		// CCD global data
		bool					mDisableCCDResweep;
		PxU32					miCCDPass;
		PxI32					mSweepTotalHits;

		// a fraction of objects will be CCD active so PxsCCDBody is dynamic, not a member of PxsRigidBody
		PxsCCDBodyArray mCCDBodies;
		PxsCCDOverlapArray mCCDOverlaps;
		PxsCCDShapeArray mCCDShapes;
		Ps::Array<PxsCCDBody*> mIslandBodies;
		Ps::Array<PxU16> mIslandSizes;
		Ps::Array<PxsRigidBody*> mUpdatedCCDBodies;
		Ps::HashMap<PxsRigidShapePair, PxsCCDShape*> mMap;

		// temporary array updated during CCD update
		//Array<PxsCCDPair> mCCDPairs;
		PxsCCDPairArray mCCDPairs;
		Ps::Array<PxsCCDPair*> mCCDPtrPairs;
		// number of pairs per island
		Ps::Array<PxU32> mCCDIslandHistogram; 
		// thread context valid during CCD update
		PxcNpThreadContext* mCCDThreadContext;
		// number of pairs to process per thread
		PxU32 mCCDPairsPerBatch;
		PxU32 mCCDMaxPasses;

		PxsContext* mContext;
		Dy::ThresholdStream& mThresholdStream;

		PxvNphaseImplementationContext& mNphaseContext;

		Ps::Mutex mMutex;

private:

	PX_NOCOPY(PxsCCDContext)
};


}



#endif

