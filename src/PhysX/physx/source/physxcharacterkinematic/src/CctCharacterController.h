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

#ifndef CCT_CHARACTER_CONTROLLER
#define CCT_CHARACTER_CONTROLLER

//#define USE_CONTACT_NORMAL_FOR_SLOPE_TEST

#include "PxController.h"
#include "PxControllerObstacles.h"
#include "CctCharacterControllerManager.h"
#include "CctUtils.h"
#include "PxTriangle.h"
#include "PsArray.h"
#include "PsHashSet.h"
#include "CmPhysXCommon.h"

namespace physx
{

struct PxFilterData;
class PxQueryFilterCallback;
class PxObstacle;

namespace Cm
{
	class RenderBuffer;
}

namespace Cct
{	    
	struct CCTParams
	{
											CCTParams();

		PxControllerNonWalkableMode::Enum	mNonWalkableMode;
		PxQuat								mQuatFromUp;
		PxVec3								mUpDirection;
		PxF32								mSlopeLimit;
		PxF32								mContactOffset;
		PxF32								mStepOffset;
		PxF32								mInvisibleWallHeight;
		PxF32								mMaxJumpHeight;
		PxF32								mMaxEdgeLength2;
		bool								mTessellation;
		bool								mHandleSlope;		// True to handle walkable parts according to slope
		bool								mOverlapRecovery;
		bool								mPreciseSweeps;
		bool								mPreventVerticalSlidingAgainstCeiling;
	};

//	typedef Ps::Array<PxTriangle>	TriArray;
	typedef Ps::Array<PxU32>		IntArray;

	// PT: using private inheritance to control access, and make sure allocations are SIMD friendly
	class TriArray : private Ps::Array<PxTriangle>
	{
		public:

		PX_FORCE_INLINE PxTriangle* reserve(PxU32 nbTris)
		{
			// PT: customized version of "reserveContainerMemory"

			const PxU32 maxNbEntries = Ps::Array<PxTriangle>::capacity();
			const PxU32 realRequiredSize = Ps::Array<PxTriangle>::size() + nbTris;
			// PT: allocate one more tri to make sure we can safely V4Load the last one...
			const PxU32 requiredSize = realRequiredSize + 1;

			if(requiredSize>maxNbEntries)
			{
				// PT: ok so the commented out growing policy was introduced by PX-837 but it produces
				// large memory usage regressions (see PX-881) while not actually making things run
				// faster. Our benchmarks show no performance difference, but up to +38% more memory
				// used with this "standard" growing policy. So for now we just go back to the initial
				// growing policy. It should be fine since PX-837 was not actually reported by a customer,
				// it was just a concern that appeared while looking at the code. Ideally we'd use a pool
				// with fixed-size slabs to get the best of both worlds but it would make iterating over
				// triangles more complicated and would need more refactoring. So for now we don't bother,
				// but we'll keep this note here for the next time this problem shows up.
				// PT: new August 2018: turns out PX-837 was correct. Not doing this produces very large
				// performance problems (like: the app freezes!) in SampleCCT. We didn't see it because
				// it's an internal sample that it rarely used these days...
				const PxU32 naturalGrowthSize = maxNbEntries ? maxNbEntries*2 : 2;
				const PxU32 newSize = PxMax(requiredSize, naturalGrowthSize);
//				const PxU32 newSize = requiredSize;

				Ps::Array<PxTriangle>::reserve(newSize);
			}

			PxTriangle* buf = Ps::Array<PxTriangle>::end();
			// ...but we still want the size to reflect the correct number
			Ps::Array<PxTriangle>::forceSize_Unsafe(realRequiredSize);
			return buf;
		}

		PX_FORCE_INLINE void pushBack(const PxTriangle& tri)
		{
			PxTriangle* memory = reserve(1);
			memory->verts[0] = tri.verts[0];
			memory->verts[1] = tri.verts[1];
			memory->verts[2] = tri.verts[2];
		}

		PX_FORCE_INLINE PxU32 size()	const
		{
			return Ps::Array<PxTriangle>::size();
		}

		PX_FORCE_INLINE const PxTriangle* begin()	const
		{
			return Ps::Array<PxTriangle>::begin();
		}

		PX_FORCE_INLINE void clear()
		{
			Ps::Array<PxTriangle>::clear();
		}

		PX_FORCE_INLINE void forceSize_Unsafe(PxU32 size)
		{
			Ps::Array<PxTriangle>::forceSize_Unsafe(size);
		}

		PX_FORCE_INLINE	const PxTriangle&	getTriangle(PxU32 index)	const
		{
			return (*this)[index];
		}

	};

	/* Exclude from documentation */
	/** \cond */

	struct TouchedGeomType
	{
		enum Enum
		{
			eUSER_BOX,
			eUSER_CAPSULE,
			eMESH,
			eBOX,
			eSPHERE,
			eCAPSULE,

			eLAST,

			eFORCE_DWORD	= 0x7fffffff
		};
	};

	class SweptVolume;

	// PT: apparently .Net aligns some of them on 8-bytes boundaries for no good reason. This is bad.
	// Whenever a variable points to a field of a specially aligned struct, it has to be declared with __packed (see GHS docu, Structure Packing, page 111).
	// Every reference to such a field needs the __packed declaration: all function parameters and assignment operators etc.
#pragma pack(push,4)

	struct TouchedGeom
	{
		TouchedGeomType::Enum	mType;
		const void*				mTGUserData;	// PxController or PxShape pointer
		const PxRigidActor*		mActor;			// PxActor for PxShape pointers (mandatory with shared shapes)
		PxExtendedVec3			mOffset;		// Local origin, typically the center of the world bounds around the character. We translate both
												// touched shapes & the character so that they are nearby this PxVec3, then add the offset back to
												// computed "world" impacts.
	protected:
		~TouchedGeom(){}
	};

	struct TouchedUserBox : public TouchedGeom
	{
		PxExtendedBox			mBox;
	};
	PX_COMPILE_TIME_ASSERT(sizeof(TouchedUserBox)==sizeof(TouchedGeom)+sizeof(PxExtendedBox));

	struct TouchedUserCapsule : public TouchedGeom
	{
		PxExtendedCapsule		mCapsule;
	};
	PX_COMPILE_TIME_ASSERT(sizeof(TouchedUserCapsule)==sizeof(TouchedGeom)+sizeof(PxExtendedCapsule));

	struct TouchedMesh : public TouchedGeom
	{
		PxU32			mNbTris;
		PxU32			mIndexWorldTriangles;
	};

	struct TouchedBox : public TouchedGeom
	{
		PxVec3			mCenter;
		PxVec3			mExtents;
		PxQuat			mRot;
	};

	struct TouchedSphere : public TouchedGeom
	{
		PxVec3			mCenter;		//!< Sphere's center
		PxF32			mRadius;		//!< Sphere's radius
	};

	struct TouchedCapsule : public TouchedGeom
	{
		PxVec3			mP0;		//!< Start of segment
		PxVec3			mP1;		//!< End of segment
		PxF32			mRadius;	//!< Capsule's radius
	};

#pragma pack(pop)

	struct SweptContact
	{
		PxExtendedVec3		mWorldPos;		// Contact position in world space
		PxVec3				mWorldNormal;	// Contact normal in world space
		PxF32				mDistance;		// Contact distance
		PxU32				mInternalIndex;	// Reserved for internal usage
		PxU32				mTriangleIndex;	// Triangle index for meshes/heightfields
		TouchedGeom*		mGeom;

		PX_FORCE_INLINE		void	setWorldPos(const PxVec3& localImpact, const PxExtendedVec3& offset)
		{
			mWorldPos.x = PxExtended(localImpact.x) + offset.x;
			mWorldPos.y = PxExtended(localImpact.y) + offset.y;
			mWorldPos.z = PxExtended(localImpact.z) + offset.z;
		}
	};

	// PT: user-defined obstacles. Note that "user" is from the SweepTest class' point of view,
	// i.e. the PhysX CCT module is the user in this case. This is to limit coupling between the
	// core CCT module and the PhysX classes.
	struct UserObstacles// : PxObstacleContext
	{
		PxU32						mNbBoxes;
		const PxExtendedBox*		mBoxes;
		const void**				mBoxUserData;

		PxU32						mNbCapsules;
		const PxExtendedCapsule*	mCapsules;
		const void**				mCapsuleUserData;
	};

	struct InternalCBData_OnHit{};
	struct InternalCBData_FindTouchedGeom{};

	enum SweepTestFlag
	{
		STF_HIT_NON_WALKABLE		= (1<<0),
		STF_WALK_EXPERIMENT			= (1<<1),
		STF_VALIDATE_TRIANGLE_DOWN	= (1<<2),	// Validate touched triangle data (down pass)
		STF_VALIDATE_TRIANGLE_SIDE	= (1<<3),	// Validate touched triangle data (side pass)
		STF_TOUCH_OTHER_CCT			= (1<<4),	// Are we standing on another CCT or not? (only updated for down pass)
		STF_TOUCH_OBSTACLE			= (1<<5),	// Are we standing on an obstacle or not? (only updated for down pass)
		STF_NORMALIZE_RESPONSE		= (1<<6),
		STF_FIRST_UPDATE			= (1<<7),
		STF_IS_MOVING_UP			= (1<<8)
	};	

	enum SweepPass
	{
		SWEEP_PASS_UP,
		SWEEP_PASS_SIDE,
		SWEEP_PASS_DOWN,
		SWEEP_PASS_SENSOR
	};
	
	class Controller;

	template<class T>
	struct TouchedObject
	{
		TouchedObject(bool regDl)
			: mTouchedObject(NULL), mRegisterDeletionListener(regDl), mCctManager(NULL)
		{
		}

		PX_FORCE_INLINE const T*	operator->() const { return mTouchedObject; }
		PX_FORCE_INLINE bool		operator==(const TouchedObject& otherObject) { return mTouchedObject == otherObject.mTouchedObject; }
		PX_FORCE_INLINE bool		operator==(const T* otherObject) { return mTouchedObject == otherObject; }
		PX_FORCE_INLINE bool		operator==(const PxBase* otherObject) { return mTouchedObject == otherObject; }
		PX_FORCE_INLINE				operator bool() const { return mTouchedObject != NULL; }
		PX_FORCE_INLINE TouchedObject&	operator=(const T* assignedObject)
		{
			if(mRegisterDeletionListener && (mTouchedObject != assignedObject))
			{
				if(mTouchedObject)
					mCctManager->unregisterObservedObject(mTouchedObject);

				if(assignedObject)
					mCctManager->registerObservedObject(assignedObject);
			}
			mTouchedObject = assignedObject;
			return *this;
		}

		const T*			get() const { return mTouchedObject; }

		void				setCctManager(CharacterControllerManager* cm) { mCctManager = cm; }

	private:
		TouchedObject& operator=(const TouchedObject&);

		const T*					mTouchedObject;
		bool						mRegisterDeletionListener;
		CharacterControllerManager*	mCctManager;
	};

	class SweepTest
	{		
	public:
										SweepTest(bool registerDeletionListener);
										~SweepTest();

		PxControllerCollisionFlags		moveCharacter(	const InternalCBData_FindTouchedGeom* userData,
														InternalCBData_OnHit* user_data2,
														SweptVolume& volume,
														const PxVec3& direction,
														const UserObstacles& userObstacles,
														PxF32 min_dist,
														const PxControllerFilters& filters,
														bool constrainedClimbingMode,
														bool standingOnMoving,
														const PxRigidActor*& touchedActor,
														const PxShape*& touchedShape,
														PxU64 contextID);

					bool				doSweepTest(const InternalCBData_FindTouchedGeom* userDataTouchedGeom,
													InternalCBData_OnHit* userDataOnHit,
													const UserObstacles& userObstacles,
													SweptVolume& swept_volume,
													const PxVec3& direction, const PxVec3& sideVector, PxU32 max_iter,
													PxU32* nb_collisions, PxF32 min_dist, const PxControllerFilters& filters, SweepPass sweepPass,
													const PxRigidActor*& touchedActor, const PxShape*& touchedShape, PxU64 contextID);

					void				findTouchedObstacles(const UserObstacles& userObstacles, const PxExtendedBounds3& world_box);

					void				voidTestCache();
					void				onRelease(const PxBase& observed);
					void				updateCachedShapesRegistration(PxU32 startIndex, bool unregister);

		// observer notifications
					void				onObstacleRemoved(ObstacleHandle index);
					void				onObstacleUpdated(ObstacleHandle index, const PxObstacleContext* context, const PxVec3& origin, const PxVec3& unitDir, const PxReal distance);
					void				onObstacleAdded(ObstacleHandle index, const PxObstacleContext* context, const PxVec3& origin, const PxVec3& unitDir, const PxReal distance);

					void				onOriginShift(const PxVec3& shift);

					Cm::RenderBuffer*	mRenderBuffer;
					PxU32				mRenderFlags;
					TriArray			mWorldTriangles;
					IntArray			mTriangleIndices;
					IntArray			mGeomStream;
					PxExtendedBounds3	mCacheBounds;
					PxU32				mCachedTriIndexIndex;
					mutable	PxU32		mCachedTriIndex[3];
					PxU32				mNbCachedStatic;
					PxU32				mNbCachedT;
	public:
#ifdef USE_CONTACT_NORMAL_FOR_SLOPE_TEST
					PxVec3				mContactNormalDownPass;
#else
					PxVec3				mContactNormalDownPass;
					PxVec3				mContactNormalSidePass;
					float				mTouchedTriMin;
					float				mTouchedTriMax;
					//PxTriangle		mTouchedTriangle;
#endif
					//
					TouchedObject<PxShape>		mTouchedShape;		// Shape on which the CCT is standing
					TouchedObject<PxRigidActor>	mTouchedActor;		// Actor from touched shape
					ObstacleHandle		mTouchedObstacleHandle;	// Obstacle on which the CCT is standing
					PxVec3				mTouchedPos;		// Last known position of mTouchedShape/mTouchedObstacle
					// PT: TODO: union those
					PxVec3				mTouchedPosShape_Local;
					PxVec3				mTouchedPosShape_World;
					PxVec3				mTouchedPosObstacle_Local;
					PxVec3				mTouchedPosObstacle_World;
					//
					CCTParams			mUserParams;
					PxF32				mVolumeGrowth;		// Must be >1.0f and not too big
					PxF32				mContactPointHeight;	// UBI
					PxU32				mSQTimeStamp;
					PxU16				mNbFullUpdates;
					PxU16				mNbPartialUpdates;
					PxU16				mNbTessellation;
					PxU16				mNbIterations;
					PxU32				mFlags;
					bool				mRegisterDeletionListener;

	PX_FORCE_INLINE	void				resetStats()
										{
											mNbFullUpdates		= 0;
											mNbPartialUpdates	= 0;
											mNbTessellation		= 0;
											mNbIterations		= 0;
										}

					void				setCctManager(CharacterControllerManager* cm) 
					{ 
						mCctManager = cm;
						mTouchedActor.setCctManager(cm); 
						mTouchedShape.setCctManager(cm); 
					}

	private:
				void					updateTouchedGeoms(	const InternalCBData_FindTouchedGeom* userData, const UserObstacles& userObstacles,
															const PxExtendedBounds3& worldBox, const PxControllerFilters& filters, const PxVec3& sideVector);				

				CharacterControllerManager*	mCctManager;
				SweepTest(const SweepTest&);
				SweepTest& operator=(const SweepTest& );
	};

	class CCTFilter	// PT: internal filter data, could be replaced with PxControllerFilters eventually
	{
		public:
		PX_FORCE_INLINE	CCTFilter() :
			mFilterData		(NULL),
			mFilterCallback	(NULL),
			mStaticShapes	(false),
			mDynamicShapes	(false),
			mPreFilter		(false),
			mPostFilter		(false),
			mCCTShapes		(NULL)
		{
		}
		const PxFilterData*		mFilterData;
		PxQueryFilterCallback*	mFilterCallback;
		bool					mStaticShapes;
		bool					mDynamicShapes;
		bool					mPreFilter;
		bool					mPostFilter;
		Ps::HashSet<PxShape>*	mCCTShapes;
	};

	PxU32 getSceneTimestamp(const InternalCBData_FindTouchedGeom* userData);

	void findTouchedGeometry(const InternalCBData_FindTouchedGeom* userData,
		const PxExtendedBounds3& world_aabb,

		TriArray& world_triangles,
		IntArray& triIndicesArray,
		IntArray& geomStream,

		const CCTFilter& filter,
		const CCTParams& params,
		PxU16& nbTessellation);

	PxU32 shapeHitCallback(const InternalCBData_OnHit* userData, const SweptContact& contact, const PxVec3& dir, PxF32 length);
	PxU32 userHitCallback(const InternalCBData_OnHit* userData, const SweptContact& contact, const PxVec3& dir, PxF32 length);

} // namespace Cct

}

/** \endcond */
#endif
