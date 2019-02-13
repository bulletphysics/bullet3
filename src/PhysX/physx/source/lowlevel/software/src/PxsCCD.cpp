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

#include "common/PxProfileZone.h"
#include "PxsContactManager.h"
#include "PxsContext.h"
#include "PxsRigidBody.h"
#include "PxcContactMethodImpl.h"
#include "GuContactPoint.h"
#include "PxsCCD.h"
#include "PsSort.h"
#include "PsAtomic.h"
#include "CmFlushPool.h"
#include "PxsMaterialManager.h"
#include "PxcMaterialMethodImpl.h"
#include "PxsMaterialManager.h"
#include "PxsMaterialCombiner.h"
#include "PxcNpContactPrepShared.h"
#include "PxvGeometry.h"
#include "PxvGlobals.h"
#include "DyThresholdTable.h"
#include "GuCCDSweepConvexMesh.h"
#include "PsUtilities.h"
#include "GuBounds.h"

#if DEBUG_RENDER_CCD
#include "CmRenderOutput.h"
#include "CmRenderBuffer.h"
#include "PxPhysics.h"
#include "PxScene.h"
#endif

#define DEBUG_RENDER_CCD_FIRST_PASS_ONLY	1
#define DEBUG_RENDER_CCD_ATOM_PTR			0
#define DEBUG_RENDER_CCD_NORMAL				1


using namespace physx;
using namespace physx::shdfnd;
using namespace physx::Dy;
using namespace Gu;


static PX_FORCE_INLINE void verifyCCDPair(const PxsCCDPair& /*pair*/)
{
#if 0
	if (pair.mBa0)
		pair.mBa0->getPose();
	if (pair.mBa1)
		pair.mBa1->getPose();
#endif
}

#if CCD_DEBUG_PRINTS
namespace physx {

static const char* gGeomTypes[PxGeometryType::eGEOMETRY_COUNT+1] = {
	"sphere", "plane", "capsule", "box", "convex", "trimesh", "heightfield", "*"
};

FILE* gCCDLog = NULL;

static inline void openCCDLog()
{
	if (gCCDLog)
	{
		fclose(gCCDLog);
		gCCDLog = NULL;
	}
	gCCDLog = fopen("c:\\ccd.txt", "wt");
	fprintf(gCCDLog, ">>>>>>>>>>>>>>>>>>>>>>>>>>> CCD START FRAME <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\n");
}

static inline void printSeparator(
	const char* prefix, PxU32 pass, PxsRigidBody* atom0, PxGeometryType::Enum g0, PxsRigidBody* atom1, PxGeometryType::Enum g1)
{
	fprintf(gCCDLog, "------- %s pass %d (%s %x) vs (%s %x)\n", prefix, pass, gGeomTypes[g0], atom0, gGeomTypes[g1], atom1);
	fflush(gCCDLog);
}

static inline void printCCDToi(
	const char* header, PxF32 t, const PxVec3& v,
	PxsRigidBody* atom0, PxGeometryType::Enum g0, PxsRigidBody* atom1, PxGeometryType::Enum g1
)
{
	fprintf(gCCDLog, "%s (%s %x vs %s %x): %.5f, (%.2f, %.2f, %.2f)\n", header, gGeomTypes[g0], atom0, gGeomTypes[g1], atom1, t, v.x, v.y, v.z);
	fflush(gCCDLog);
}

static inline void printCCDPair(const char* header, PxsCCDPair& pair)
{
	printCCDToi(header, pair.mMinToi, pair.mMinToiNormal, pair.mBa0, pair.mG0, pair.mBa1, pair.mG1);
}

// also used in PxcSweepConvexMesh.cpp
void printCCDDebug(const char* msg, const PxsRigidBody* atom0, PxGeometryType::Enum g0, bool printPtr)
{
	fprintf(gCCDLog, "  %s (%s %x)\n", msg, gGeomTypes[g0], printPtr ? atom0 : 0);
	fflush(gCCDLog);
}

// also used in PxcSweepConvexMesh.cpp
void printShape(
	PxsRigidBody* atom0, PxGeometryType::Enum g0, const char* annotation, PxReal dt, PxU32 pass, bool printPtr)
{
	fprintf(gCCDLog, "%s (%s %x) atom=(%.2f, %.2f, %.2f)>(%.2f, %.2f, %.2f) v=(%.1f, %.1f, %.1f), mv=(%.1f, %.1f, %.1f)\n",
		annotation, gGeomTypes[g0], printPtr ? atom0 : 0,
		atom0->getLastCCDTransform().p.x, atom0->getLastCCDTransform().p.y, atom0->getLastCCDTransform().p.z,
		atom0->getPose().p.x, atom0->getPose().p.y, atom0->getPose().p.z,
		atom0->getLinearVelocity().x, atom0->getLinearVelocity().y, atom0->getLinearVelocity().z,
		atom0->getLinearMotionVelocity(dt).x, atom0->getLinearMotionVelocity(dt).y, atom0->getLinearMotionVelocity(dt).z );
	fflush(gCCDLog);
	#if DEBUG_RENDER_CCD && DEBUG_RENDER_CCD_ATOM_PTR
	if (!DEBUG_RENDER_CCD_FIRST_PASS_ONLY || pass == 0)
	{
		PxScene *s; PxGetPhysics()->getScenes(&s, 1, 0);
		Cm::RenderOutput((Cm::RenderBuffer&)s->getRenderBuffer())
			<< Cm::DebugText(atom0->getPose().p, 0.05f, "%x", atom0);
	}
	#endif
}

static inline void flushCCDLog()
{
	fflush(gCCDLog);
}

} // namespace physx

#else

namespace physx
{

void printShape(PxsRigidBody* /*atom0*/, PxGeometryType::Enum /*g0*/, const char* /*annotation*/, PxReal /*dt*/, PxU32 /*pass*/, bool printPtr = true)
{PX_UNUSED(printPtr);}

static inline void openCCDLog() {}

static inline void flushCCDLog() {}

void printCCDDebug(const char* /*msg*/, const PxsRigidBody* /*atom0*/, PxGeometryType::Enum /*g0*/, bool printPtr = true) {PX_UNUSED(printPtr);}

static inline void printSeparator(
	const char* /*prefix*/, PxU32 /*pass*/, PxsRigidBody* /*atom0*/, PxGeometryType::Enum /*g0*/,
	PxsRigidBody* /*atom1*/, PxGeometryType::Enum /*g1*/) {}
} // namespace physx
#endif

namespace
{
	// PT: TODO: refactor with ShapeSim version (SIMD)
	PX_INLINE PxTransform getShapeAbsPose(const PxsShapeCore* shapeCore, const PxsRigidCore* rigidCore, PxU32 isDynamic)
	{
		if(isDynamic)
		{
			const PxsBodyCore* PX_RESTRICT bodyCore = static_cast<const PxsBodyCore*>(rigidCore);
			return bodyCore->body2World * bodyCore->getBody2Actor().getInverse() *shapeCore->transform;
		}
		else 
		{
			return rigidCore->body2World * shapeCore->transform;
		}
	}
}

namespace physx
{

	PxsCCDContext::PxsCCDContext(PxsContext* context, Dy::ThresholdStream& thresholdStream, PxvNphaseImplementationContext& nPhaseContext) :
		mPostCCDSweepTask			(context->getContextId(), this, "PxsContext.postCCDSweep"),
		mPostCCDAdvanceTask			(context->getContextId(), this, "PxsContext.postCCDAdvance"),
		mPostCCDDepenetrateTask		(context->getContextId(), this, "PxsContext.postCCDDepenetrate"),
		mDisableCCDResweep			(false),
		miCCDPass					(0),
		mSweepTotalHits				(0),
		mCCDThreadContext			(NULL),
		mCCDPairsPerBatch			(0),
		mCCDMaxPasses				(1),
		mContext					(context),
		mThresholdStream			(thresholdStream),
		mNphaseContext(nPhaseContext)
{
}

PxsCCDContext::~PxsCCDContext()
{
}

PxsCCDContext* PxsCCDContext::create(PxsContext* context, Dy::ThresholdStream& thresholdStream, PxvNphaseImplementationContext& nPhaseContext)
{
	PxsCCDContext* dc = reinterpret_cast<PxsCCDContext*>(
		PX_ALLOC(sizeof(PxsCCDContext), "PxsCCDContext"));

	if(dc)
	{
		new(dc) PxsCCDContext(context, thresholdStream, nPhaseContext);
	}
	return dc;
}

void PxsCCDContext::destroy()
{
	this->~PxsCCDContext();
	PX_FREE(this);
}

PxTransform PxsCCDShape::getAbsPose(const PxsRigidBody* atom)						const
{
	// PT: TODO: refactor with ShapeSim version (SIMD) - or with redundant getShapeAbsPose() above in this same file!
	if(atom)
		return atom->getPose() * atom->getCore().getBody2Actor().getInverse() * mShapeCore->transform;
	else
		return mRigidCore->body2World * mShapeCore->transform;
}

PxTransform PxsCCDShape::getLastCCDAbsPose(const PxsRigidBody* atom)						const
{
	// PT: TODO: refactor with ShapeSim version (SIMD)
	return atom->getLastCCDTransform() * atom->getCore().getBody2Actor().getInverse() * mShapeCore->transform;
}

PxReal PxsCCDPair::sweepFindToi(PxcNpThreadContext& context, PxReal dt, PxU32 pass)
{
	printSeparator("findToi", pass, mBa0, mG0, NULL, PxGeometryType::eGEOMETRY_COUNT);
	//Update shape transforms if necessary
	updateShapes();

	//Extract the bodies
	PxsRigidBody* atom0 = mBa0;
	PxsRigidBody* atom1 = mBa1;
	PxsCCDShape* ccdShape0 = mCCDShape0;
	PxsCCDShape* ccdShape1 = mCCDShape1;
	PxGeometryType::Enum g0 = mG0, g1 = mG1;

	//If necessary, flip the bodies to make sure that g0 <= g1
	if(mG1 < mG0)
	{
		g0 = mG1;
		g1 = mG0;
		ccdShape0 = mCCDShape1;
		ccdShape1 = mCCDShape0;
		atom0 = mBa1;
		atom1 = mBa0;
	}
	
	PX_ALIGN(16, PxTransform tm0);
	PX_ALIGN(16, PxTransform tm1);
	PX_ALIGN(16, PxTransform lastTm0);
	PX_ALIGN(16, PxTransform lastTm1);

	tm0 = ccdShape0->mCurrentTransform;
	lastTm0 = ccdShape0->mPrevTransform;

	tm1 = ccdShape1->mCurrentTransform;
	lastTm1 = ccdShape1->mPrevTransform;

	const PxVec3 trA = tm0.p - lastTm0.p;
	const PxVec3 trB = tm1.p - lastTm1.p;
	const PxVec3 relTr = trA - trB;

	// Do the sweep
	PxVec3 sweepNormal(0.0f);
	PxVec3 sweepPoint(0.0f);

	PxReal restDistance = PxMax(mCm->getWorkUnit().restDistance, 0.f);

	context.mDt = dt;
	context.mCCDFaceIndex = PXC_CONTACT_NO_FACE_INDEX;

	//Cull the sweep hit based on the relative velocity along the normal
	const PxReal fastMovingThresh0 = ccdShape0->mFastMovingThreshold;
	const PxReal fastMovingThresh1 = ccdShape1->mFastMovingThreshold;

	const PxReal sumFastMovingThresh = (fastMovingThresh0 + fastMovingThresh1);


	PxReal toi = Gu::SweepShapeShape(*ccdShape0, *ccdShape1, tm0, tm1, lastTm0, lastTm1, restDistance,
		sweepNormal, sweepPoint, mMinToi, context.mCCDFaceIndex, sumFastMovingThresh);

	//If toi is after the end of TOI, return no hit
	if (toi >= 1.0f)
	{
		mToiType				= PxsCCDPair::ePrecise;
		mPenetration			= 0.f;
		mPenetrationPostStep	= 0.f;
		mMinToi					= PX_MAX_REAL; // needs to be reset in case of resweep
		return toi;
	}

	PX_ASSERT(PxIsFinite(toi));
	PX_ASSERT(sweepNormal.isFinite());
	mFaceIndex = context.mCCDFaceIndex;

	//Work out linear motion (used to cull the sweep hit)
	const PxReal linearMotion = relTr.dot(-sweepNormal);
	
	//If we swapped the shapes, swap them back
	if(mG1 >= mG0)
		sweepNormal = -sweepNormal;


	mToiType = PxsCCDPair::ePrecise;

	PxReal penetration = 0.f;
	PxReal penetrationPostStep = 0.f;

	//If linear motion along normal < the CCD threshold, set toi to no-hit.
	if((linearMotion) < sumFastMovingThresh)
	{
		mMinToi					= PX_MAX_REAL; // needs to be reset in case of resweep
		return PX_MAX_REAL;
	}
	else if(toi <= 0.f)
	{
		//If the toi <= 0.f, this implies an initial overlap. If the value < 0.f, it implies penetration
		PxReal stepRatio0 = atom0 ? atom0->mCCD->mTimeLeft : 1.f;
		PxReal stepRatio1 = atom1 ? atom1->mCCD->mTimeLeft : 1.f;

		PxReal stepRatio = PxMin(stepRatio0, stepRatio1);
		penetration = -toi;

		toi = 0.f;
		
		if(stepRatio == 1.f)
		{
			//If stepRatio == 1.f (i.e. neither body has stepped forwards any time at all)
			//we extract the advance coefficients from the bodies and permit the bodies to step forwards a small amount
			//to ensure that they won't remain jammed because TOI = 0.0
			const PxReal advance0 = atom0 ? atom0->mCore->ccdAdvanceCoefficient : 1.f;
			const PxReal advance1 = atom1 ? atom1->mCore->ccdAdvanceCoefficient : 1.f;
			const PxReal advance = PxMin(advance0, advance1);

			PxReal fastMoving = PxMin(fastMovingThresh0, atom1 ? fastMovingThresh1 : PX_MAX_REAL);
			PxReal advanceCoeff = advance * fastMoving;
			penetrationPostStep = advanceCoeff/linearMotion;
			
		}
		PX_ASSERT(PxIsFinite(toi));
	}

	//Store TOI, penetration and post-step (how much to step forward in initial overlap conditions)
	mMinToi			= toi;
	mPenetration	= penetration;
	mPenetrationPostStep = penetrationPostStep;

	mMinToiPoint = sweepPoint;

	mMinToiNormal	= sweepNormal;


	//Work out the materials for the contact (restitution, friction etc.)
	context.mContactBuffer.count = 0;
	context.mContactBuffer.contact(mMinToiPoint, mMinToiNormal, 0.f, g1 == PxGeometryType::eTRIANGLEMESH || g1 == PxGeometryType::eHEIGHTFIELD? mFaceIndex : PXC_CONTACT_NO_FACE_INDEX);

	PxsMaterialInfo materialInfo;

	g_GetSingleMaterialMethodTable[g0](ccdShape0->mShapeCore, 0, context, &materialInfo);
	g_GetSingleMaterialMethodTable[g1](ccdShape1->mShapeCore, 1, context, &materialInfo);

	const PxsMaterialData& data0 = *context.mMaterialManager->getMaterial(materialInfo.mMaterialIndex0);
	const PxsMaterialData& data1 = *context.mMaterialManager->getMaterial(materialInfo.mMaterialIndex1);

	const PxReal restitution = PxsMaterialCombiner::combineRestitution(data0, data1);
	PxsMaterialCombiner combiner(1.0f, 1.0f);
	PxsMaterialCombiner::PxsCombinedMaterial combinedMat = combiner.combineIsotropicFriction(data0, data1);
	const PxReal sFriction = combinedMat.staFriction;
	const PxReal dFriction = combinedMat.dynFriction;

	mMaterialIndex0 = materialInfo.mMaterialIndex0;
	mMaterialIndex1 = materialInfo.mMaterialIndex1;
	mDynamicFriction = dFriction;
	mStaticFriction = sFriction;
	mRestitution = restitution;

	return toi;
}

void PxsCCDPair::updateShapes()
{
	if(mBa0)
	{
		//If the CCD shape's update count doesn't match the body's update count, this shape needs its transforms and bounds re-calculated
		if(mBa0->mCCD->mUpdateCount != mCCDShape0->mUpdateCount)
		{
			const PxTransform tm0 = mCCDShape0->getAbsPose(mBa0);
			const PxTransform lastTm0 = mCCDShape0->getLastCCDAbsPose(mBa0);

			const PxVec3 trA = tm0.p - lastTm0.p;

			Gu::Vec3p origin, extents;
			Gu::computeBoundsWithCCDThreshold(origin, extents, mCCDShape0->mShapeCore->geometry.getGeometry(), tm0, NULL);

			mCCDShape0->mCenter = origin - trA;
			mCCDShape0->mExtents = extents;
			mCCDShape0->mPrevTransform = lastTm0;
			mCCDShape0->mCurrentTransform = tm0;
			mCCDShape0->mUpdateCount = mBa0->mCCD->mUpdateCount;
		}
	}

	if(mBa1)
	{
		//If the CCD shape's update count doesn't match the body's update count, this shape needs its transforms and bounds re-calculated
		if(mBa1->mCCD->mUpdateCount != mCCDShape1->mUpdateCount)
		{
			const PxTransform tm1 = mCCDShape1->getAbsPose(mBa1);
			const PxTransform lastTm1 = mCCDShape1->getLastCCDAbsPose(mBa1);

			const PxVec3 trB = tm1.p - lastTm1.p;

			Vec3p origin, extents;
			computeBoundsWithCCDThreshold(origin, extents, mCCDShape1->mShapeCore->geometry.getGeometry(), tm1, NULL);

			mCCDShape1->mCenter = origin - trB;
			mCCDShape1->mExtents = extents;
			mCCDShape1->mPrevTransform = lastTm1;
			mCCDShape1->mCurrentTransform = tm1;
			mCCDShape1->mUpdateCount = mBa1->mCCD->mUpdateCount;
		}
	}
}

PxReal PxsCCDPair::sweepEstimateToi()
{
	//Update shape transforms if necessary
	updateShapes();

	//PxsRigidBody* atom1 = mBa1;
	//PxsRigidBody* atom0 = mBa0;
	PxsCCDShape* ccdShape0 = mCCDShape0;
	PxsCCDShape* ccdShape1 = mCCDShape1;
	PxGeometryType::Enum g0 = mG0, g1 = mG1;
	PX_UNUSED(g0);


	//Flip shapes if necessary
	if(mG1 < mG0)
	{
		g0 = mG1;
		g1 = mG0;
		/*atom0 = mBa1;
		atom1 = mBa0;*/
		ccdShape0 = mCCDShape1;
		ccdShape1 = mCCDShape0;
	}

	//Extract previous/current transforms, translations etc.
	PxTransform tm0, lastTm0, tm1, lastTm1;

	PxVec3 trA(0.f);
	PxVec3 trB(0.f);
	tm0 = ccdShape0->mCurrentTransform;
	lastTm0 = ccdShape0->mPrevTransform;
	trA = tm0.p - lastTm0.p;

	tm1 = ccdShape1->mCurrentTransform;
	lastTm1 = ccdShape1->mPrevTransform;
	trB = tm1.p - lastTm1.p;

	PxReal restDistance = PxMax(mCm->getWorkUnit().restDistance, 0.f);

	const PxVec3 relTr = trA - trB;

	//Work out the sum of the fast moving thresholds scaled by the step ratio
	const PxReal fastMovingThresh0 = ccdShape0->mFastMovingThreshold;
	const PxReal fastMovingThresh1 = ccdShape1->mFastMovingThreshold;
	const PxReal sumFastMovingThresh = (fastMovingThresh0 + fastMovingThresh1);

	mToiType = eEstimate;

	//If the objects are not moving fast-enough relative to each-other to warrant CCD, set estimated time as PX_MAX_REAL
	if((relTr.magnitudeSquared()) <= (sumFastMovingThresh * sumFastMovingThresh))
	{
		mToiType = eEstimate;
		mMinToi = PX_MAX_REAL;
		return PX_MAX_REAL;
	}

	//Otherwise, the objects *are* moving fast-enough so perform estimation pass
	if(g1 == PxGeometryType::eTRIANGLEMESH)
	{
		//Special-case estimation code for meshes
		PxF32 toi = Gu::SweepEstimateAnyShapeMesh(*ccdShape0, *ccdShape1, tm0, tm1, lastTm0, lastTm1, restDistance, sumFastMovingThresh);
									
		mMinToi	= toi;
		return toi;
	}
	else if (g1 == PxGeometryType::eHEIGHTFIELD)
	{
		//Special-case estimation code for heightfields
		PxF32 toi = Gu::SweepEstimateAnyShapeHeightfield(*ccdShape0, *ccdShape1, tm0, tm1, lastTm0, lastTm1, restDistance, sumFastMovingThresh);
									
		mMinToi	= toi;
		return toi;
	}

	//Generic estimation code for prim-prim sweeps
	PxVec3 centreA, extentsA;
	PxVec3 centreB, extentsB;
	centreA = ccdShape0->mCenter;
	extentsA = ccdShape0->mExtents + PxVec3(restDistance);

	centreB = ccdShape1->mCenter;
	extentsB = ccdShape1->mExtents;

	PxF32 toi = Gu::sweepAABBAABB(centreA, extentsA * 1.1f, centreB, extentsB * 1.1f, trA, trB);
	mMinToi			= toi;
	return toi;
}

bool PxsCCDPair::sweepAdvanceToToi(PxReal dt, bool clipTrajectoryToToi)
{
	PxsCCDShape* ccds0 = mCCDShape0;
	PxsRigidBody* atom0 = mBa0;
	PxsCCDShape* ccds1 = mCCDShape1;
	PxsRigidBody* atom1 = mBa1;

	const PxsCCDPair* thisPair = this;

	//Both already had a pass so don't do anything
	if ((atom0 == NULL || atom0->mCCD->mPassDone) && (atom1 == NULL || atom1->mCCD->mPassDone))
		return false;

	//Test to validate that they're both infinite mass objects. If so, there can be no response so terminate on a notification-only
	if((atom0 == NULL || atom0->mCore->inverseMass == 0.f) && (atom1 == NULL || atom1->mCore->inverseMass == 0.f))
		return false;

	//If the TOI < 1.f. If not, this hit happens after this frame or at the very end of the frame. Either way, next frame can handle it
	if (thisPair->mMinToi < 1.0f)
	{
		if(thisPair->mCm->getWorkUnit().flags & PxcNpWorkUnitFlag::eDISABLE_RESPONSE || thisPair->mMaxImpulse == 0.f)
		{
			//Don't mark pass as done on either body
			return true;
		}

		
		PxReal minToi = thisPair->mMinToi;

		PxF32 penetration = -mPenetration * 10.f;
		
		PxVec3 minToiNormal = thisPair->mMinToiNormal;

		if (!minToiNormal.isNormalized())
		{
			// somehow we got zero normal. This can happen for instance if two identical objects spawn exactly on top of one another
			// abort ccd and clip to current toi
			if (atom0 && !atom0->mCCD->mPassDone)
			{
				atom0->advancePrevPoseToToi(minToi);
				atom0->advanceToToi(minToi, dt, true);
				atom0->mCCD->mUpdateCount++;
			}
			return true;
		}

	

		//Get the material indices...
		const PxReal restitution = mRestitution;
		const PxReal sFriction = mStaticFriction;
		const PxReal dFriction = mDynamicFriction;


		PxVec3 v0(0.f), v1(0.f);

		PxReal invMass0(0.f), invMass1(0.f);
#if CCD_ANGULAR_IMPULSE
		PxMat33 invInertia0(PxVec3(0.f), PxVec3(0.f), PxVec3(0.f)), invInertia1(PxVec3(0.f), PxVec3(0.f), PxVec3(0.f));
		PxVec3 localPoint0(0.f), localPoint1(0.f);
#endif

		PxReal dom0 = mCm->getDominance0();
		PxReal dom1 = mCm->getDominance1();

		//Work out velocity and invMass for body 0
		if(atom0)
		{
			//Put contact point in local space, then find how much point is moving using point velocity...
			
#if CCD_ANGULAR_IMPULSE
			localPoint0 = mMinToiPoint - trA.p;
			v0 = atom0->mCore->linearVelocity + atom0->mCore->angularVelocity.cross(localPoint0);
			
			physx::Cm::transformInertiaTensor(atom0->mCore->inverseInertia, PxMat33(trA.q),invInertia0);
			invInertia0 *= dom0;
#else
			v0 = atom0->mCore->linearVelocity + atom0->mCore->angularVelocity.cross(ccds0->mCurrentTransform.p - atom0->mCore->body2World.p);
#endif
			invMass0 = atom0->getInvMass() * dom0;
			
		}

		//Work out velocity and invMass for body 1
		if(atom1)
		{
			
			//Put contact point in local space, then find how much point is moving using point velocity...
#if CCD_ANGULAR_IMPULSE
			
			localPoint1 = mMinToiPoint - trB.p;
			v1 = atom1->mCore->linearVelocity + atom1->mCore->angularVelocity.cross(localPoint1);
			physx::Cm::transformInertiaTensor(atom1->mCore->inverseInertia, PxMat33(trB.q),invInertia1);
			invInertia1 *= dom1;
#else
			v1 = atom1->mCore->linearVelocity + atom1->mCore->angularVelocity.cross(ccds1->mCurrentTransform.p - atom1->mCore->body2World.p);
#endif
			invMass1 = atom1->getInvMass() * dom1;
		}

		PX_ASSERT(v0.isFinite() && v1.isFinite());

		//Work out relative velocity
		PxVec3 vRel = v1 - v0;

		//Project relative velocity onto contact normal and bias with penetration
		PxReal relNorVel = vRel.dot(minToiNormal);
		PxReal relNorVelPlusPen = relNorVel + penetration;

#if CCD_ANGULAR_IMPULSE
		if(relNorVelPlusPen >= -1e-6f)
		{
			//we fall back on linear only parts...
			localPoint0 = PxVec3(0.f);
			localPoint1 = PxVec3(0.f);
			v0 = atom0 ? atom0->getLinearVelocity() : PxVec3(0.f);
			v1 = atom1 ? atom1->getLinearVelocity() : PxVec3(0.f);
			vRel = v1 - v0;
			relNorVel = vRel.dot(minToiNormal);
			relNorVelPlusPen = relNorVel + penetration;
		}
#endif


		//If the relative motion is moving towards each-other, respond
		if(relNorVelPlusPen < -1e-6f)
		{
			PxReal sumRecipMass = invMass0 + invMass1;

			const PxReal jLin = relNorVelPlusPen;
			const PxReal normalResponse = (1.f + restitution) * jLin;

#if CCD_ANGULAR_IMPULSE
			const PxVec3 angularMom0 = invInertia0 * (localPoint0.cross(mMinToiNormal));
			const PxVec3 angularMom1 = invInertia1 * (localPoint1.cross(mMinToiNormal));

			const PxReal jAng = minToiNormal.dot(angularMom0.cross(localPoint0) + angularMom1.cross(localPoint1));
			const PxReal impulseDivisor = sumRecipMass + jAng;

#else
			const PxReal impulseDivisor = sumRecipMass;
#endif
			const PxReal jImp = PxMax(-mMaxImpulse, normalResponse/impulseDivisor);

			PxVec3 j(0.f);

			//If the user requested CCD friction, calculate friction forces.
			//Note, CCD is *linear* so friction is also linear. The net result is that CCD friction can stop bodies' lateral motion so its better to have it disabled
			//unless there's a real need for it.
			if(mHasFriction)
			{
			
				PxVec3 vPerp = vRel - relNorVel * minToiNormal;
				PxVec3 tDir = vPerp;
				PxReal length = tDir.normalize();
				PxReal vPerpImp = length/impulseDivisor;
				PxF32 fricResponse = 0.f;
				PxF32 staticResponse = (jImp*sFriction);
				PxF32 dynamicResponse = (jImp*dFriction);
				
				
				if (PxAbs(staticResponse) >= vPerpImp)
					fricResponse = vPerpImp;
				else
				{
					fricResponse = -dynamicResponse /* times m0 */;
				}

				
				//const PxVec3 fricJ = -vPerp.getNormalized() * (fricResponse/impulseDivisor);
				const PxVec3 fricJ =  tDir * (fricResponse);
				j = jImp * mMinToiNormal + fricJ;
			}
			else
			{
				j = jImp * mMinToiNormal;
			}

			verifyCCDPair(*this);
			//If we have a negative impulse value, then we need to apply it. If not, the bodies are separating (no impulse to apply).
			if(jImp < 0.f)
			{
				mAppliedForce = -jImp;

				//Adjust velocities
				if((atom0 != NULL && atom0->mCCD->mPassDone) || 
					(atom1 != NULL && atom1->mCCD->mPassDone))
				{
					mPenetrationPostStep = 0.f;
				}
				else
				{
					if (atom0)
					{
						//atom0->mAcceleration.linear = atom0->getLinearVelocity();  // to provide pre-"solver" velocity in contact reports

						atom0->setLinearVelocity(atom0->getLinearVelocity() + j * invMass0);
						atom0->constrainLinearVelocity();
	#if CCD_ANGULAR_IMPULSE
						atom0->mAcceleration.angular = atom0->getAngularVelocity();  // to provide pre-"solver" velocity in contact reports
						atom0->setAngularVelocity(atom0->getAngularVelocity() + invInertia0 * localPoint0.cross(j));
						atom0->constrainAngularVelocity();
	#endif
					}
					if (atom1)
					{
						//atom1->mAcceleration.linear = atom1->getLinearVelocity();  // to provide pre-"solver" velocity in contact reports
						atom1->setLinearVelocity(atom1->getLinearVelocity() - j * invMass1);
						atom1->constrainLinearVelocity();
	#if CCD_ANGULAR_IMPULSE
						atom1->mAcceleration.angular = atom1->getAngularVelocity();  // to provide pre-"solver" velocity in contact reports
						atom1->setAngularVelocity(atom1->getAngularVelocity() - invInertia1 * localPoint1.cross(j));
						atom1->constrainAngularVelocity();
	#endif
					}
				}
			}
		}

		//Update poses
		if (atom0 && !atom0->mCCD->mPassDone)
		{
			atom0->advancePrevPoseToToi(minToi);
			atom0->advanceToToi(minToi, dt, clipTrajectoryToToi && mPenetrationPostStep == 0.f);	
			atom0->mCCD->mUpdateCount++;
		}
		if (atom1 && !atom1->mCCD->mPassDone)
		{
			atom1->advancePrevPoseToToi(minToi);
			atom1->advanceToToi(minToi, dt, clipTrajectoryToToi && mPenetrationPostStep == 0.f);
			atom1->mCCD->mUpdateCount++;
		}

		//If we had a penetration post-step (i.e. an initial overlap), step forwards slightly after collision response
		if(mPenetrationPostStep > 0.f)
		{
			if (atom0 && !atom0->mCCD->mPassDone)
			{
				atom0->advancePrevPoseToToi(mPenetrationPostStep);
				if(clipTrajectoryToToi)
					atom0->advanceToToi(mPenetrationPostStep, dt, clipTrajectoryToToi);
			}
			if (atom1 && !atom1->mCCD->mPassDone)
			{
				atom1->advancePrevPoseToToi(mPenetrationPostStep);
				if(clipTrajectoryToToi)
					atom1->advanceToToi(mPenetrationPostStep, dt, clipTrajectoryToToi);
			}
		}
		//Mark passes as done
		if (atom0)
		{
			atom0->mCCD->mPassDone = true;
			atom0->mCCD->mHasAnyPassDone = true;
		}
		if (atom1)
		{
			atom1->mCCD->mPassDone = true;
			atom1->mCCD->mHasAnyPassDone = true;
		}

		return true;
		
		//return false;
	} 
	else
	{
		printCCDDebug("advToi: clean sweep", atom0, mG0);
	}

	return false;
}

struct IslandCompare
{
	bool operator()(PxsCCDPair& a, PxsCCDPair& b) const { return a.mIslandId < b.mIslandId; }
};

struct IslandPtrCompare
{
	bool operator()(PxsCCDPair*& a, PxsCCDPair*& b) const { return a->mIslandId < b->mIslandId; }
};

struct ToiCompare
{
	bool operator()(PxsCCDPair& a, PxsCCDPair& b) const 
	{ 
		return (a.mMinToi < b.mMinToi) || 
		((a.mMinToi == b.mMinToi) && (a.mBa1 != NULL && b.mBa1 == NULL)); 
	}
};

struct ToiPtrCompare
{
	bool operator()(PxsCCDPair*& a, PxsCCDPair*& b) const 
	{ 
		return (a->mMinToi < b->mMinToi) || 
		((a->mMinToi == b->mMinToi) && (a->mBa1 != NULL && b->mBa1 == NULL)); 
	}
};


// --------------------------------------------------------------
/**
\brief Class to perform a set of sweep estimate tasks
*/
class PxsCCDSweepTask : public Cm::Task
{
	PxsCCDPair** 					mPairs;
	PxU32							mNumPairs;
public:
	PxsCCDSweepTask(PxU64 contextID, PxsCCDPair** pairs, PxU32 nPairs)
		:	Cm::Task(contextID), mPairs(pairs), mNumPairs(nPairs)
	{
	}

	virtual void runInternal()
	{
		for (PxU32 j = 0; j < mNumPairs; j++)
		{
			PxsCCDPair& pair = *mPairs[j];
			pair.sweepEstimateToi();
			pair.mEstimatePass = 0;
		}
	}

	virtual const char *getName() const
	{
		return "PxsContext.CCDSweep";
	}

private:
	PxsCCDSweepTask& operator=(const PxsCCDSweepTask&);
};

#define ENABLE_RESWEEP 1

// --------------------------------------------------------------
/**
\brief Class to advance a set of islands
*/
class PxsCCDAdvanceTask : public Cm::Task
{
	PxsCCDPair** 					mCCDPairs;
	PxU32							mNumPairs;
	PxsContext* 					mContext;
	PxsCCDContext* 					mCCDContext;
	PxReal							mDt;
	PxU32							mCCDPass;
	const PxsCCDBodyArray&			mCCDBodies;

	PxU32							mFirstThreadIsland;
	PxU32							mIslandsPerThread;
	PxU32							mTotalIslandCount;
	PxU32							mFirstIslandPair; // pairs are sorted by island
	PxsCCDBody**					mIslandBodies;
	PxU16*							mNumIslandBodies;
	PxI32*							mSweepTotalHits;
	bool							mClipTrajectory;
	bool							mDisableResweep;
	
	PxsCCDAdvanceTask& operator=(const PxsCCDAdvanceTask&);
public:
	PxsCCDAdvanceTask(PxsCCDPair** pairs, PxU32 nPairs, const PxsCCDBodyArray& ccdBodies,
				PxsContext* context, PxsCCDContext* ccdContext, PxReal dt, PxU32 ccdPass,
				PxU32 firstIslandPair, PxU32 firstThreadIsland, PxU32 islandsPerThread, PxU32 totalIslands, 
				PxsCCDBody** islandBodies, PxU16* numIslandBodies, bool clipTrajectory, bool disableResweep,
				PxI32* sweepTotalHits)
		:	Cm::Task(context->getContextId()), mCCDPairs(pairs), mNumPairs(nPairs), mContext(context), mCCDContext(ccdContext), mDt(dt),
			mCCDPass(ccdPass), mCCDBodies(ccdBodies), mFirstThreadIsland(firstThreadIsland), 
			mIslandsPerThread(islandsPerThread), mTotalIslandCount(totalIslands), mFirstIslandPair(firstIslandPair),
			mIslandBodies(islandBodies), mNumIslandBodies(numIslandBodies),	mSweepTotalHits(sweepTotalHits),
			mClipTrajectory(clipTrajectory), mDisableResweep(disableResweep)
			
	{
		PX_ASSERT(mFirstIslandPair < mNumPairs);
	}

	virtual void runInternal()
	{

		PxI32 sweepTotalHits = 0;

		PxcNpThreadContext* threadContext = mContext->getNpThreadContext();

		// --------------------------------------------------------------------------------------
		// loop over island labels assigned to this thread
		PxU32 islandStart = mFirstIslandPair;
		PxU32 lastIsland = PxMin(mFirstThreadIsland + mIslandsPerThread, mTotalIslandCount);
		for (PxU32 iIsland = mFirstThreadIsland; iIsland < lastIsland; iIsland++)
		{
			if (islandStart >= mNumPairs)
				// this is possible when for instance there are two islands with 0 pairs in the second
				// since islands are initially segmented using bodies, not pairs, it can happen
				break;

			// --------------------------------------------------------------------------------------
			// sort all pairs within current island by toi
			PxU32 islandEnd = islandStart+1;
			PX_ASSERT(mCCDPairs[islandStart]->mIslandId == iIsland);
			while (islandEnd < mNumPairs && mCCDPairs[islandEnd]->mIslandId == iIsland) // find first index past the current island id
				islandEnd++;

			if (islandEnd > islandStart+1)
				shdfnd::sort(mCCDPairs+islandStart, islandEnd-islandStart, ToiPtrCompare());

			PX_ASSERT(islandEnd <= mNumPairs);

			// --------------------------------------------------------------------------------------
			// advance all affected pairs within each island to min toi
			// for each pair (A,B) in toi order, find any later-toi pairs that collide against A or B
			// and resweep against changed trajectories of either A or B (excluding statics and kinematics)
			PxReal islandMinToi = PX_MAX_REAL;
			PxU32 estimatePass = 1;

			PxReal dt = mDt;

			for (PxU32 iFront = islandStart; iFront < islandEnd; iFront++)
			{
				PxsCCDPair& pair = *mCCDPairs[iFront];

				verifyCCDPair(pair);

				//If we have reached a pair with a TOI after 1.0, we can terminate this island
				if(pair.mMinToi > 1.f)
					break;

				bool needSweep0 = (pair.mBa0 && pair.mBa0->mCCD->mPassDone == false);
				bool needSweep1 = (pair.mBa1 && pair.mBa1->mCCD->mPassDone == false);

				//If both bodies have been updated (or one has been updated and the other is static), we can skip to the next pair
				if(!(needSweep0 || needSweep1))
					continue;

				{
					//If the pair was an estimate, we must perform an accurate sweep now
					if(pair.mToiType == PxsCCDPair::eEstimate)
					{
						pair.sweepFindToi(*threadContext, dt, mCCDPass);

						//Test to see if the pair is still the earliest pair.
						if((iFront + 1) < islandEnd && mCCDPairs[iFront+1]->mMinToi < pair.mMinToi)
						{
							//If there is an earlier pair, we push this pair into its correct place in the list and return to the start
							//of this update loop
							PxsCCDPair* tmp = &pair;
							PxU32 index = iFront;
							while((index + 1) < islandEnd && mCCDPairs[index+1]->mMinToi < pair.mMinToi)
							{
								mCCDPairs[index] = mCCDPairs[index+1];
								++index;
							}

							mCCDPairs[index] = tmp;

							--iFront;
							continue;
						}
					}

					if (pair.mMinToi > 1.f)
						break;

					//We now have the earliest contact pair for this island and one/both of the bodies have not been updated. We now perform
					//contact modification to find out if the user still wants to respond to the collision
					if(pair.mMinToi <= islandMinToi && 
						pair.mIsModifiable &&
						mCCDContext->getCCDContactModifyCallback())
					{

						PX_ALIGN(16, PxU8 dataBuffer[sizeof(PxModifiableContact) + sizeof(PxContactPatch)]);

						PxContactPatch* patch = reinterpret_cast<PxContactPatch*>(dataBuffer);
						PxModifiableContact* point = reinterpret_cast<PxModifiableContact*>(patch + 1);

						patch->mMassModification.mInvInertiaScale0 = 1.f;
						patch->mMassModification.mInvInertiaScale1 = 1.f;
						patch->mMassModification.mInvMassScale0 = 1.f;
						patch->mMassModification.mInvMassScale1 = 1.f;

						patch->normal = pair.mMinToiNormal;

						patch->dynamicFriction = pair.mDynamicFriction;
						patch->staticFriction = pair.mStaticFriction;
						patch->materialIndex0 = pair.mMaterialIndex0;
						patch->materialIndex1 = pair.mMaterialIndex1;

						patch->startContactIndex = 0;
						patch->nbContacts = 1;

						patch->materialFlags = 0;
						patch->internalFlags = 0;											//44  //Can be a U16


						point->contact = pair.mMinToiPoint;
						point->normal = pair.mMinToiNormal;

						//KS - todo - reintroduce face indices!!!!
						//point.internalFaceIndex0 = PXC_CONTACT_NO_FACE_INDEX;
						//point.internalFaceIndex1 = pair.mFaceIndex;
						point->materialIndex0 = pair.mMaterialIndex0;
						point->materialIndex1 = pair.mMaterialIndex1;
						point->dynamicFriction = pair.mDynamicFriction;
						point->staticFriction = pair.mStaticFriction;
						point->restitution = pair.mRestitution;
						point->separation = 0.f;
						point->maxImpulse = PX_MAX_REAL;
						point->materialFlags = 0;
						point->targetVelocity = PxVec3(0.f);

						mCCDContext->runCCDModifiableContact(point, 1, pair.mCCDShape0->mShapeCore, pair.mCCDShape1->mShapeCore,
							pair.mCCDShape0->mRigidCore, pair.mCCDShape1->mRigidCore, pair.mBa0, pair.mBa1);

						if ((patch->internalFlags & PxContactPatch::eHAS_MAX_IMPULSE))
							pair.mMaxImpulse = point->maxImpulse;

						pair.mDynamicFriction = point->dynamicFriction;
						pair.mStaticFriction = point->staticFriction;
						pair.mRestitution = point->restitution;
						pair.mMinToiPoint = point->contact;
						pair.mMinToiNormal = point->normal;

					}

				}

				// pair.mIsEarliestToiHit is used for contact notification.
				// only mark as such if this is the first impact for both atoms of this pair (the impacts are sorted)
				// and there was an actual impact for this pair
				bool atom0FirstSweep = (pair.mBa0 && pair.mBa0->mCCD->mPassDone == false) || pair.mBa0 == NULL;
				bool atom1FirstSweep = (pair.mBa1 && pair.mBa1->mCCD->mPassDone == false) || pair.mBa1 == NULL;
				if (pair.mMinToi <= 1.0f && atom0FirstSweep && atom1FirstSweep)
					pair.mIsEarliestToiHit = true;

				// sweepAdvanceToToi sets mCCD->mPassDone flags on both atoms, doesn't advance atoms with flag already set
				// can advance one atom if the other already has the flag set
				bool advanced = pair.sweepAdvanceToToi( dt, mClipTrajectory);
				if(pair.mMinToi < 0.f)
					pair.mMinToi = 0.f;
				verifyCCDPair(pair);

				if (advanced && pair.mMinToi <= 1.0f)
				{
					sweepTotalHits++;
					PxU32 islandStartIndex = iIsland == 0 ? 0 : PxU32(mNumIslandBodies[iIsland - 1]);
					PxU32 islandEndIndex = mNumIslandBodies[iIsland];

					if(pair.mMinToi > 0.f)
					{
						for(PxU32 a = islandStartIndex; a < islandEndIndex; ++a)
						{	
							if(!mIslandBodies[a]->mPassDone)
							{
								//If the body has not updated, we advance it to the current time-step that the island has reached.
								PxsRigidBody* atom = mIslandBodies[a]->mBody;
								atom->advancePrevPoseToToi(pair.mMinToi);
								atom->mCCD->mTimeLeft = PxMax(atom->mCCD->mTimeLeft * (1.0f - pair.mMinToi), CCD_MIN_TIME_LEFT);
								atom->mCCD->mUpdateCount++;
							}
						}

						//Adjust remaining dt for the island
						dt -= dt * pair.mMinToi;

						const PxReal recipOneMinusToi = 1.f/(1.f - pair.mMinToi);
						for(PxU32 k = iFront+1; k < islandEnd; ++k)
						{
							PxsCCDPair& pair1 = *mCCDPairs[k];
							pair1.mMinToi = (pair1.mMinToi - pair.mMinToi)*recipOneMinusToi;
						}

					}

					//If we disabled response, we don't need to resweep at all
					if(!mDisableResweep && !(pair.mCm->getWorkUnit().flags & PxcNpWorkUnitFlag::eDISABLE_RESPONSE) && pair.mMaxImpulse != 0.f)
					{
						void* a0 = pair.mBa0 == NULL ? NULL : reinterpret_cast<void*>(pair.mBa0);
						void* a1 = pair.mBa1 == NULL ? NULL : reinterpret_cast<void*>(pair.mBa1);

						for(PxU32 k = iFront+1; k < islandEnd; ++k)
						{
							PxsCCDPair& pair1 = *mCCDPairs[k];

							void* b0 = pair1.mBa0 == NULL ? reinterpret_cast<void*>(pair1.mCCDShape0) : reinterpret_cast<void*>(pair1.mBa0);
							void* b1 = pair1.mBa1 == NULL ? reinterpret_cast<void*>(pair1.mCCDShape1) : reinterpret_cast<void*>(pair1.mBa1);

							bool containsStatic = pair1.mBa0 == NULL || pair1.mBa1 == NULL;

							PX_ASSERT(b0 != NULL && b1 != NULL);

							if ((!containsStatic) &&
								((b0 == a0 && b1 != a1) || (b1 == a0 && b0 != a1) ||
								(b0 == a1 && b1 != a0) || (b1 == a1 && b0 != a0))
							)
							{
								if(estimatePass != pair1.mEstimatePass)
								{
									pair1.mEstimatePass = estimatePass;
									// resweep pair1 since either b0 or b1 trajectory has changed
									PxReal oldToi = pair1.mMinToi;
									verifyCCDPair(pair1);
									PxReal toi1 = pair1.sweepEstimateToi();
									PX_ASSERT(pair1.mBa0); // this is because mMinToiNormal is the impact point here
									if (toi1 < oldToi)
									{
										// if toi decreased, resort the array backwards
										PxU32 kk = k;
										PX_ASSERT(kk > 0);
										
										while (kk-1 > iFront && mCCDPairs[kk-1]->mMinToi > toi1)
										{
											PxsCCDPair* temp = mCCDPairs[kk-1];
											mCCDPairs[kk-1] = mCCDPairs[kk];
											mCCDPairs[kk] = temp;
											kk--;
										}

									}
									else if (toi1 > oldToi)
									{
										// if toi increased, resort the array forwards
										PxU32 kk = k;
										PX_ASSERT(kk > 0);
										PxU32 stepped = 0;
										while (kk+1 < islandEnd && mCCDPairs[kk+1]->mMinToi < toi1)
										{
											stepped = 1;
											PxsCCDPair* temp = mCCDPairs[kk+1];
											mCCDPairs[kk+1] = mCCDPairs[kk];
											mCCDPairs[kk] = temp;
											kk++;
										}
										k -= stepped;
									}
								}
							}
						}
					}
					estimatePass++;
				} // if pair.minToi <= 1.0f
			} // for iFront

			islandStart = islandEnd;
		} // for (PxU32 iIsland = mFirstThreadIsland; iIsland < lastIsland; iIsland++)

		Ps::atomicAdd(mSweepTotalHits, sweepTotalHits);
		mContext->putNpThreadContext(threadContext);
	}

	virtual const char *getName() const
	{
		return "PxsContext.CCDAdvance";
	}
};

// --------------------------------------------------------------
// CCD main function
// Overall structure:
/*
for nPasses (passes are now handled in void Sc::Scene::updateCCDMultiPass)

  update CCD broadphase, generate a list of CMs

  foreach CM
    create CCDPairs, CCDBodies from CM
	add shapes, overlappingShapes to CCDBodies

  foreach CCDBody
    assign island labels per body
    uses overlappingShapes

  foreach CCDPair 
    assign island label to pair  

  sort all pairs by islandId

  foreach CCDPair
    sweep/find toi
    compute normal:
        
  foreach island
    sort within island by toi
    foreach pair within island
      advanceToToi
      from curPairInIsland to lastPairInIsland
        resweep if needed

*/
// --------------------------------------------------------------
void PxsCCDContext::updateCCDBegin()
{
	openCCDLog();

	miCCDPass = 0;
	mSweepTotalHits = 0;
}

// --------------------------------------------------------------
void PxsCCDContext::updateCCDEnd()
{
	if (miCCDPass == mCCDMaxPasses - 1 || mSweepTotalHits == 0)
	{
		// --------------------------------------------------------------------------------------
		// At last CCD pass we need to reset mBody pointers back to NULL
		// so that the next frame we know which ones need to be newly paired with PxsCCDBody objects
		// also free the CCDBody memory blocks

		mMutex.lock();
		for (PxU32 j = 0, n = mCCDBodies.size(); j < n; j++)
		{
			if (mCCDBodies[j].mBody->mCCD && mCCDBodies[j].mBody->mCCD->mHasAnyPassDone)
			{
				//Record this body in the list of bodies that were updated
				mUpdatedCCDBodies.pushBack(mCCDBodies[j].mBody);
			}
			mCCDBodies[j].mBody->mCCD = NULL;
			mCCDBodies[j].mBody->getCore().isFastMoving = false; //Clear the "isFastMoving" bool
		}
		mMutex.unlock();

		mCCDBodies.clear_NoDelete();

	}

	mCCDShapes.clear_NoDelete();

	mMap.clear();

	miCCDPass++;
}

// --------------------------------------------------------------
void PxsCCDContext::verifyCCDBegin()
{
	#if 0
	// validate that all bodies have a NULL mCCD pointer
	if (miCCDPass == 0)
	{
		Cm::BitMap::Iterator it(mActiveContactManager);
		for (PxU32 index = it.getNext(); index != Cm::BitMap::Iterator::DONE; index = it.getNext())
		{
			PxsContactManager* cm = mContactManagerPool.findByIndexFast(index);
			PxsRigidBody* b0 = cm->mBodyShape0->getBodyAtom(), *b1 = cm->mBodyShape1->getBodyAtom();
			PX_ASSERT(b0 == NULL || b0->mCCD == NULL);
			PX_ASSERT(b1 == NULL || b1->mCCD == NULL);
		}
	}
	#endif
}

void PxsCCDContext::resetContactManagers()
{
	Cm::BitMap::Iterator it(mContext->mContactManagersWithCCDTouch);

	for (PxU32 index = it.getNext(); index != Cm::BitMap::Iterator::DONE; index = it.getNext())
	{
		PxsContactManager* cm = mContext->mContactManagerPool.findByIndexFast(index);
		cm->clearCCDContactInfo();
	}

	mContext->mContactManagersWithCCDTouch.clear();
}

// --------------------------------------------------------------
void PxsCCDContext::updateCCD(PxReal dt, PxBaseTask* continuation, IG::IslandSim& islandSim, bool disableResweep, PxI32 numFastMovingShapes)
{
	//Flag to run a slightly less-accurate version of CCD that will ensure that objects don't tunnel through the static world but is not as reliable for dynamic-dynamic collisions
	mDisableCCDResweep = disableResweep;  
	mThresholdStream.clear();  // clear force threshold report stream

	mContext->clearManagerTouchEvents();

	if (miCCDPass == 0)
	{
		resetContactManagers();
	}


	// If we're not in the first pass and the previous pass had no sweeps or the BP didn't generate any fast-moving shapes, we can skip CCD entirely
	if ((miCCDPass > 0 && mSweepTotalHits == 0) || (numFastMovingShapes == 0)) 
	{
		mSweepTotalHits = 0;
		updateCCDEnd();
		return;
	}
	mSweepTotalHits = 0;

	PX_ASSERT(continuation);
	PX_ASSERT(continuation->getReference() > 0);

	//printf("CCD 1\n");

	mCCDThreadContext = mContext->getNpThreadContext();
	mCCDThreadContext->mDt = dt; // doesn't get set anywhere else since it's only used for CCD now

	verifyCCDBegin();

	// --------------------------------------------------------------------------------------
	// From a list of active CMs, build a temporary array of PxsCCDPair objects (allocated in blocks)
	// this is done to gather scattered data from memory and also to reduce PxsRidigBody permanent memory footprint
	// we have to do it every pass since new CMs can become fast moving after each pass (and sometimes cease to be)
	mCCDPairs.clear_NoDelete();
	mCCDPtrPairs.forceSize_Unsafe(0);

	mUpdatedCCDBodies.forceSize_Unsafe(0);

	mCCDOverlaps.clear_NoDelete();

	PxU32 nbKinematicStaticCollisions = 0;


	bool needsSweep = false;

	{
		PX_PROFILE_ZONE("Sim.ccdPair", mContext->mContextID);
	
		Cm::BitMap::Iterator it(mContext->mActiveContactManagersWithCCD);
		for (PxU32 index = it.getNext(); index != Cm::BitMap::Iterator::DONE; index = it.getNext())
		{
			PxsContactManager* cm = mContext->mContactManagerPool.findByIndexFast(index);

			// skip disabled pairs
			if(!cm->getCCD())
				continue;

			bool isJoint0 = (cm->mNpUnit.flags & PxcNpWorkUnitFlag::eARTICULATION_BODY0) == PxcNpWorkUnitFlag::eARTICULATION_BODY0;
			bool isJoint1 = (cm->mNpUnit.flags & PxcNpWorkUnitFlag::eARTICULATION_BODY1) == PxcNpWorkUnitFlag::eARTICULATION_BODY1;
			// skip articulation vs articulation ccd
			//Actually. This is fundamentally wrong also :(. We only want to skip links in the same articulation - not all articulations!!!
			if (isJoint0 && isJoint1)
				continue;

			bool isFastMoving0 = static_cast<const PxsBodyCore*>(cm->mNpUnit.rigidCore0)->isFastMoving != 0;

			bool isFastMoving1 = (cm->mNpUnit.flags & (PxcNpWorkUnitFlag::eARTICULATION_BODY1 | PxcNpWorkUnitFlag::eDYNAMIC_BODY1)) ? static_cast<const PxsBodyCore*>(cm->mNpUnit.rigidCore1)->isFastMoving != 0: false;

			if (!(isFastMoving0 || isFastMoving1))
				continue;

			PxcNpWorkUnit& unit = cm->getWorkUnit();
			const PxsRigidCore* rc0 = unit.rigidCore0;
			const PxsRigidCore* rc1 = unit.rigidCore1;
			
			{
				const PxsShapeCore* sc0 = unit.shapeCore0;
				const PxsShapeCore* sc1 = unit.shapeCore1;

				PxsRigidBody* ba0 = cm->mRigidBody0;
				PxsRigidBody* ba1 = cm->mRigidBody1;

				//Look up the body/shape pair in our CCDShape map
				const Ps::Pair<const PxsRigidShapePair, PxsCCDShape*>* ccdShapePair0 = mMap.find(PxsRigidShapePair(rc0, sc0));
				const Ps::Pair<const PxsRigidShapePair, PxsCCDShape*>* ccdShapePair1 = mMap.find(PxsRigidShapePair(rc1, sc1));

				//If the CCD shapes exist, extract them from the map
				PxsCCDShape* ccdShape0 = ccdShapePair0 ? ccdShapePair0->second : NULL;
				PxsCCDShape* ccdShape1 = ccdShapePair1 ? ccdShapePair1->second : NULL;

				PxReal threshold0 = 0.f;
				PxReal threshold1 = 0.f;

				PxVec3 trA(0.f);
				PxVec3 trB(0.f);

				if(ccdShape0 == NULL)
				{
					//If we hadn't already created ccdShape, create one
					ccdShape0 = &mCCDShapes.pushBack();
					mMap.insert(PxsRigidShapePair(rc0, sc0), ccdShape0);

					ccdShape0->mRigidCore = rc0;
					ccdShape0->mShapeCore = sc0;
					ccdShape0->mGeometry = &sc0->geometry;

					const PxTransform tm0 = ccdShape0->getAbsPose(ba0);
					const PxTransform oldTm0 = ba0 ? ccdShape0->getLastCCDAbsPose(ba0) : tm0;

					trA = tm0.p - oldTm0.p;

					Vec3p origin, extents;

					//Compute the shape's bounds and CCD threshold
					threshold0 = computeBoundsWithCCDThreshold(origin, extents, sc0->geometry.getGeometry(), tm0, NULL);

					//Set up the CCD shape
					ccdShape0->mCenter = origin - trA;
					ccdShape0->mExtents = extents;
					ccdShape0->mFastMovingThreshold = threshold0;
					ccdShape0->mPrevTransform = oldTm0;
					ccdShape0->mCurrentTransform = tm0;
					ccdShape0->mUpdateCount = 0;
					ccdShape0->mNodeIndex = islandSim.getNodeIndex1(cm->getWorkUnit().mEdgeIndex);
				}
				else
				{
					//We had already created the shape, so extract the threshold and translation components
					threshold0 = ccdShape0->mFastMovingThreshold;
					trA = ccdShape0->mCurrentTransform.p - ccdShape0->mPrevTransform.p;
				}

				if(ccdShape1 == NULL)
				{
					//If the CCD shape was not already constructed, create it
					ccdShape1 = &mCCDShapes.pushBack();
					ccdShape1->mRigidCore = rc1;
					ccdShape1->mShapeCore = sc1;
					ccdShape1->mGeometry = &sc1->geometry;

					mMap.insert(PxsRigidShapePair(rc1, sc1), ccdShape1);

					const PxTransform tm1 = ccdShape1->getAbsPose(ba1);
					const PxTransform oldTm1 = ba1 ? ccdShape1->getLastCCDAbsPose(ba1) : tm1;

					trB = tm1.p - oldTm1.p;

					Vec3p origin, extents;
					//Compute the shape's bounds and CCD threshold
					threshold1 = computeBoundsWithCCDThreshold(origin, extents, sc1->geometry.getGeometry(), tm1, NULL);

					//Set up the CCD shape
					ccdShape1->mCenter = origin - trB;
					ccdShape1->mExtents = extents;
					ccdShape1->mFastMovingThreshold = threshold1;
					ccdShape1->mPrevTransform = oldTm1;
					ccdShape1->mCurrentTransform = tm1;
					ccdShape1->mUpdateCount = 0;
					ccdShape1->mNodeIndex = islandSim.getNodeIndex2(cm->getWorkUnit().mEdgeIndex);
				}
				else
				{
					//CCD shape already constructed so just extract thresholds and trB components
					threshold1 = ccdShape1->mFastMovingThreshold;
					trB = ccdShape1->mCurrentTransform.p - ccdShape1->mPrevTransform.p;
				}

				{
					//Initialize the CCD bodies
					PxsRigidBody* atoms[2] = {ba0, ba1};
					for (int k = 0; k < 2; k++)
					{
						PxsRigidBody* b = atoms[k];
						//If there isn't a body (i.e. it's a static), no need to create a CCD body
						if (!b)
							continue;
						if (b->mCCD == NULL)
						{
							// this rigid body has no CCD body created for it yet. Create and initialize one.
							PxsCCDBody& newB = mCCDBodies.pushBack();
							b->mCCD = &newB;
							b->mCCD->mIndex = Ps::to16(mCCDBodies.size()-1);
							b->mCCD->mBody = b;
							b->mCCD->mTimeLeft = 1.0f;
							b->mCCD->mOverlappingObjects = NULL;
							b->mCCD->mUpdateCount = 0;
							b->mCCD->mHasAnyPassDone = false;
							b->mCCD->mNbInteractionsThisPass = 0;
						}
						b->mCCD->mPassDone = 0;
						b->mCCD->mNbInteractionsThisPass++;
					}
					if(ba0 && ba1)
					{
						//If both bodies exist (i.e. this is dynamic-dynamic collision), we create an
						//overlap between the 2 bodies used for island detection.
						if(!(ba0->isKinematic() || ba1->isKinematic()))
						{
							if(!ba0->mCCD->overlaps(ba1->mCCD))
							{
								PxsCCDOverlap* overlapA = &mCCDOverlaps.pushBack();
								PxsCCDOverlap* overlapB = &mCCDOverlaps.pushBack();

								overlapA->mBody = ba1->mCCD;
								overlapB->mBody = ba0->mCCD;

								ba0->mCCD->addOverlap(overlapA);
								ba1->mCCD->addOverlap(overlapB);
							}
						}
					}
				}
				
				//We now create the CCD pair. These are used in the CCD sweep and update phases
				if (ba0->isKinematic() && (ba1 == NULL || ba1->isKinematic()))
					nbKinematicStaticCollisions++;
				{
					PxsCCDPair& p = mCCDPairs.pushBack();
					p.mBa0 = ba0;
					p.mBa1 = ba1;
					p.mCCDShape0 = ccdShape0;
					p.mCCDShape1 = ccdShape1;
					p.mHasFriction = rc0->hasCCDFriction() || rc1->hasCCDFriction();
					p.mMinToi = PX_MAX_REAL;
					p.mG0 = cm->mNpUnit.shapeCore0->geometry.getType();
					p.mG1 = cm->mNpUnit.shapeCore1->geometry.getType();
					p.mCm = cm;
					p.mIslandId = 0xFFFFffff;
					p.mIsEarliestToiHit = false;
					p.mFaceIndex = PXC_CONTACT_NO_FACE_INDEX;
					p.mIsModifiable = cm->isChangeable() != 0;
					p.mAppliedForce = 0.f;
					p.mMaxImpulse = PxMin((ba0->mCore->mFlags & PxRigidBodyFlag::eENABLE_CCD_MAX_CONTACT_IMPULSE) ? ba0->mCore->maxContactImpulse : PX_MAX_F32,
						(ba1 && ba1->mCore->mFlags & PxRigidBodyFlag::eENABLE_CCD_MAX_CONTACT_IMPULSE) ? ba1->mCore->maxContactImpulse : PX_MAX_F32);

#if PX_ENABLE_SIM_STATS
					mContext->mSimStats.mNbCCDPairs[PxMin(p.mG0, p.mG1)][PxMax(p.mG0, p.mG1)] ++;
#endif

					//Calculate the sum of the thresholds and work out if we need to perform a sweep.
					
					const PxReal thresh = threshold0 + threshold1;
					//If no shape pairs in the entire scene are fast-moving, we can bypass the entire of the CCD.
					needsSweep = needsSweep || (trA - trB).magnitudeSquared() >= (thresh * thresh);
				}
				
			}
		}
		//There are no fast-moving pairs in this scene, so we can terminate right now without proceeding any further
		if(!needsSweep)
		{
			updateCCDEnd();
			mContext->putNpThreadContext(mCCDThreadContext);
			return;
		}
	}

	//Create the pair pointer buffer. This is a flattened array of pointers to pairs. It is used to sort the pairs
	//into islands and is also used to prioritize the pairs into their TOIs
	{
		const PxU32 size = mCCDPairs.size();
		mCCDPtrPairs.reserve(size);
		for(PxU32 a = 0; a < size; ++a)
		{
			mCCDPtrPairs.pushBack(&mCCDPairs[a]);
		}

		mThresholdStream.reserve(Ps::nextPowerOfTwo(size));

		for (PxU32 a = 0; a < mCCDBodies.size(); ++a)
		{
			mCCDBodies[a].mPreSolverVelocity.linear = mCCDBodies[a].mBody->getLinearVelocity();
			mCCDBodies[a].mPreSolverVelocity.angular = mCCDBodies[a].mBody->getAngularVelocity();
		}
	}


	PxU32 ccdBodyCount = mCCDBodies.size();

	// --------------------------------------------------------------------------------------
	// assign island labels
	const PxU16 noLabelYet = 0xFFFF;
	
	//Temporary array allocations. Ideally, we should use the scratch pad for there
	Array<PxU32> islandLabels; 
	islandLabels.resize(ccdBodyCount);
	Array<const PxsCCDBody*> stack; 
	stack.reserve(ccdBodyCount);
	stack.forceSize_Unsafe(ccdBodyCount);


	//Initialize all islands labels (for each body) to be unitialized
	mIslandSizes.forceSize_Unsafe(0);
	mIslandSizes.reserve(ccdBodyCount + 1);
	mIslandSizes.forceSize_Unsafe(ccdBodyCount + 1);
	for (PxU32 j = 0; j < ccdBodyCount; j++)
		islandLabels[j] = noLabelYet;

	PxU32 islandCount = 0;
	PxU32 stackSize = 0;
	const PxsCCDBody* top = NULL;
	
	for (PxU32 j = 0; j < ccdBodyCount; j++)
	{
		//If the body has already been labelled or if it is kinematic, continue
		//Also, if the body has no interactions this pass, continue. In single-pass CCD, only bodies with interactions would be part of the CCD. However,
		//with multi-pass CCD, we keep all bodies that interacted in previous passes. If the body now has no interactions, we skip it to ensure that island grouping doesn't fail in
		//later stages by assigning an island ID to a body with no interactions
		if (islandLabels[j] != noLabelYet || mCCDBodies[j].mBody->isKinematic() || mCCDBodies[j].mNbInteractionsThisPass == 0)
			continue;

		top = &mCCDBodies[j];
		//Otherwise push it back into the queue and proceed
		islandLabels[j] = islandCount;
		
		stack[stackSize++] = top;		
		// assign new label to unlabeled atom
		// assign the same label to all connected nodes using stack traversal
		PxU16 islandSize = 0;
		while (stackSize > 0)
		{
			--stackSize;
			const PxsCCDBody* ccdb = top;
			top = stack[PxMax(1u, stackSize)-1];

			PxsCCDOverlap* overlaps = ccdb->mOverlappingObjects;
			while(overlaps)
			{
				if (islandLabels[overlaps->mBody->mIndex] == noLabelYet) // non-static & unlabeled?
				{
					islandLabels[overlaps->mBody->mIndex] = islandCount;
					stack[stackSize++] = overlaps->mBody; // push adjacent node to the top of the stack
					top = overlaps->mBody;
					islandSize++;
				}
				overlaps = overlaps->mNext;
			}
		}
		//Record island size
		mIslandSizes[islandCount] = PxU16(islandSize + 1);
		islandCount++;
	}

	PxU32 kinematicIslandId = islandCount;

	islandCount += nbKinematicStaticCollisions;

	for (PxU32 i = kinematicIslandId; i < islandCount; ++i)
		mIslandSizes[i] = 1;

	

		// --------------------------------------------------------------------------------------
	// label pairs with island ids
	// (need an extra loop since we don't maintain a mapping from atom to all of it's pairs)
	mCCDIslandHistogram.clear(); // number of pairs per island
	mCCDIslandHistogram.resize(islandCount);

	PxU32 totalActivePairs = 0;
	for (PxU32 j = 0, n = mCCDPtrPairs.size(); j < n; j++)
	{
		const PxU32 staticLabel = 0xFFFFffff;
		PxsCCDPair& p = *mCCDPtrPairs[j];
		PxU32 id0 = p.mBa0 && !p.mBa0->isKinematic()? islandLabels[p.mBa0->mCCD->getIndex()] : staticLabel;
		PxU32 id1 = p.mBa1 && !p.mBa1->isKinematic()? islandLabels[p.mBa1->mCCD->getIndex()] : staticLabel;

		PxU32 islandId = PxMin(id0, id1);
		if (islandId == staticLabel)
			islandId = kinematicIslandId++;

		p.mIslandId = islandId;
		mCCDIslandHistogram[p.mIslandId] ++;
		PX_ASSERT(p.mIslandId != staticLabel);
		totalActivePairs++;
	}

	PxU16 count = 0;
	for(PxU16 a = 0; a < islandCount+1; ++a)
	{
		PxU16 islandSize = mIslandSizes[a];
		mIslandSizes[a] = count;
		count += islandSize;
	}

	mIslandBodies.forceSize_Unsafe(0);
	mIslandBodies.reserve(ccdBodyCount);
	mIslandBodies.forceSize_Unsafe(ccdBodyCount);
	for(PxU32 a = 0; a < mCCDBodies.size(); ++a)
	{
		const PxU32 island = islandLabels[mCCDBodies[a].mIndex];
		if (island != 0xFFFF)
		{
			PxU16 writeIndex = mIslandSizes[island];
			mIslandSizes[island] = PxU16(writeIndex + 1);
			mIslandBodies[writeIndex] = &mCCDBodies[a];
		}
	}

	// --------------------------------------------------------------------------------------
	// setup tasks
	mPostCCDDepenetrateTask.setContinuation(continuation);
	mPostCCDAdvanceTask.setContinuation(&mPostCCDDepenetrateTask);
	mPostCCDSweepTask.setContinuation(&mPostCCDAdvanceTask);

	// --------------------------------------------------------------------------------------
	// sort all pairs by islands
	shdfnd::sort(mCCDPtrPairs.begin(), mCCDPtrPairs.size(), IslandPtrCompare());

	// --------------------------------------------------------------------------------------
	// sweep all CCD pairs
	const PxU32 nPairs = mCCDPtrPairs.size();
	const PxU32 numThreads = PxMax(1u, mContext->mTaskManager->getCpuDispatcher()->getWorkerCount()); PX_ASSERT(numThreads > 0);
	mCCDPairsPerBatch = PxMax<PxU32>((nPairs)/numThreads, 1);

	for (PxU32 batchBegin = 0; batchBegin < nPairs; batchBegin += mCCDPairsPerBatch)
	{
		void* ptr = mContext->mTaskPool.allocate(sizeof(PxsCCDSweepTask));
		PX_ASSERT_WITH_MESSAGE(ptr, "Failed to allocate PxsCCDSweepTask");
		const PxU32 batchEnd = PxMin(nPairs, batchBegin + mCCDPairsPerBatch);
		PX_ASSERT(batchEnd >= batchBegin);
		PxsCCDSweepTask* task = PX_PLACEMENT_NEW(ptr, PxsCCDSweepTask)(mContext->getContextId(), mCCDPtrPairs.begin() + batchBegin, batchEnd - batchBegin);
		task->setContinuation(*mContext->mTaskManager, &mPostCCDSweepTask);
		task->removeReference();
	}

	mPostCCDSweepTask.removeReference();
	mPostCCDAdvanceTask.removeReference();
	mPostCCDDepenetrateTask.removeReference();
}

void PxsCCDContext::postCCDSweep(PxBaseTask* continuation)
{
	// --------------------------------------------------------------------------------------
	// batch up the islands and send them over to worker threads
	PxU32 firstIslandPair = 0;
	PxU32 islandCount = mCCDIslandHistogram.size();
	for (PxU32 firstIslandInBatch = 0; firstIslandInBatch < islandCount;)
	{
		PxU32 pairSum = 0;
		PxU32 lastIslandInBatch = firstIslandInBatch+1;
		PxU32 j;
		// add up the numbers in the histogram until we reach target pairsPerBatch
		for (j = firstIslandInBatch; j < islandCount; j++)
		{
			pairSum += mCCDIslandHistogram[j];
			if (pairSum > mCCDPairsPerBatch)
			{
				lastIslandInBatch = j+1;
				break;
			}
		}
		if (j == islandCount) // j is islandCount if not enough pairs were left to fill up to pairsPerBatch
		{
			if (pairSum == 0)
				break; // we are done and there are no islands in this batch
			lastIslandInBatch = islandCount;
		}

		void* ptr = mContext->mTaskPool.allocate(sizeof(PxsCCDAdvanceTask));
		PX_ASSERT_WITH_MESSAGE(ptr , "Failed to allocate PxsCCDSweepTask");
		bool clipTrajectory = (miCCDPass == mCCDMaxPasses-1);
		PxsCCDAdvanceTask* task = PX_PLACEMENT_NEW(ptr, PxsCCDAdvanceTask) (
			mCCDPtrPairs.begin(), mCCDPtrPairs.size(), mCCDBodies, mContext, this, mCCDThreadContext->mDt, miCCDPass, 
			firstIslandPair, firstIslandInBatch, lastIslandInBatch-firstIslandInBatch, islandCount, 
			mIslandBodies.begin(), mIslandSizes.begin(), clipTrajectory, mDisableCCDResweep,
			&mSweepTotalHits);
		firstIslandInBatch = lastIslandInBatch;
		firstIslandPair += pairSum;		
		task->setContinuation(*mContext->mTaskManager, continuation);
		task->removeReference();
	} // for iIsland
}

void PxsCCDContext::postCCDAdvance(PxBaseTask* /*continuation*/)
{	
	// --------------------------------------------------------------------------------------
	// contact notifications: update touch status (multi-threading this section would probably slow it down but might be worth a try)
	PxU32 countLost = 0, countFound = 0, countRetouch = 0;

	PxU32 islandCount = mCCDIslandHistogram.size();
	PxU32 index = 0;

	for (PxU32 island = 0; island < islandCount; ++island)
	{
		PxU32 islandEnd = mCCDIslandHistogram[island] + index;
		for(PxU32 j = index; j < islandEnd; ++j)
		{
			PxsCCDPair& p = *mCCDPtrPairs[j];
			//The CCD pairs are ordered by TOI. If we reach a TOI > 1, we can terminate
			if(p.mMinToi > 1.f)
				break;
		
			//If this was the earliest touch for the pair of bodies, we can notify the user about it. If not, it's a future collision that we haven't stepped to yet
			if(p.mIsEarliestToiHit)
			{
				//Flag that we had a CCD contact
				p.mCm->setHadCCDContact();

				//Test/set the changed touch map
				PxU16 oldTouch = p.mCm->getTouchStatus();
				if (!oldTouch)
				{
					mContext->mContactManagerTouchEvent.growAndSet(p.mCm->getIndex());
					p.mCm->mNpUnit.statusFlags = PxU16((p.mCm->mNpUnit.statusFlags & (~PxcNpWorkUnitStatusFlag::eHAS_NO_TOUCH)) | PxcNpWorkUnitStatusFlag::eHAS_TOUCH);
					//Also need to write it in the CmOutput structure!!!!!
					
					//The achieve this, we need to unregister the CM from the Nphase, then re-register it with the status set. This is the only way to force a push to the GPU
					mNphaseContext.unregisterContactManager(p.mCm);
					mNphaseContext.registerContactManager(p.mCm, 1, 0);
					countFound++;
				}
				else
				{
					mContext->mContactManagerTouchEvent.growAndSet(p.mCm->getIndex());
					p.mCm->raiseCCDRetouch();
					countRetouch++;
				}

				//Do we want to create reports?
				const bool createReports = 
					p.mCm->mNpUnit.flags & PxcNpWorkUnitFlag::eOUTPUT_CONTACTS
					|| (p.mCm->mNpUnit.flags & PxcNpWorkUnitFlag::eFORCE_THRESHOLD
					&& ((p.mCm->mNpUnit.flags & PxcNpWorkUnitFlag::eDYNAMIC_BODY0 && static_cast<const PxsBodyCore*>(p.mCm->mNpUnit.rigidCore0)->shouldCreateContactReports())
					|| (p.mCm->mNpUnit.flags & PxcNpWorkUnitFlag::eDYNAMIC_BODY1 && static_cast<const PxsBodyCore*>(p.mCm->mNpUnit.rigidCore1)->shouldCreateContactReports())));

				if(createReports)
				{
					mContext->mContactManagersWithCCDTouch.growAndSet(p.mCm->getIndex());

					const PxU32 numContacts = 1;
					PxsMaterialInfo matInfo;
					Gu::ContactBuffer& buffer = mCCDThreadContext->mContactBuffer;

					Gu::ContactPoint& cp = buffer.contacts[0];
					cp.point = p.mMinToiPoint;
					cp.normal = -p.mMinToiNormal;						//KS - discrete contact gen produces contacts pointing in the opposite direction to CCD sweeps
					cp.internalFaceIndex1 = p.mFaceIndex;
					cp.separation = 0.0f;
					cp.restitution = p.mRestitution;
					cp.dynamicFriction = p.mDynamicFriction;
					cp.staticFriction = p.mStaticFriction;
					cp.targetVel = PxVec3(0.f);
					cp.maxImpulse = PX_MAX_REAL;
					
					matInfo.mMaterialIndex0 = p.mMaterialIndex0;
					matInfo.mMaterialIndex1 = p.mMaterialIndex1;

					//Write contact stream for the contact. This will allocate memory for the contacts and forces
					PxReal* contactForces;
					//PxU8* contactStream;
					PxU8* contactPatches;
					PxU8* contactPoints;
					PxU16 contactStreamSize;
					PxU8 contactCount;
					PxU8 nbPatches;
					PxsCCDContactHeader* ccdHeader = reinterpret_cast<PxsCCDContactHeader*>(p.mCm->mNpUnit.ccdContacts);
					if (writeCompressedContact(buffer.contacts, numContacts, mCCDThreadContext, contactCount, contactPatches,
						contactPoints, contactStreamSize, contactForces, numContacts*sizeof(PxReal), mCCDThreadContext->mMaterialManager,
												((p.mCm->mNpUnit.flags & PxcNpWorkUnitFlag::eMODIFIABLE_CONTACT) != 0), true, &matInfo, nbPatches, sizeof(PxsCCDContactHeader),NULL, NULL,
												false, NULL, NULL, NULL, p.mFaceIndex != PXC_CONTACT_NO_FACE_INDEX))
					{
						PxsCCDContactHeader* newCCDHeader = reinterpret_cast<PxsCCDContactHeader*>(contactPatches);
						newCCDHeader->contactStreamSize = Ps::to16(contactStreamSize);
						newCCDHeader->isFromPreviousPass = 0;

						p.mCm->mNpUnit.ccdContacts = contactPatches;	// put the latest stream at the head of the linked list since it needs to get accessed every CCD pass
																	// to prepare the reports

						if (!ccdHeader)
							newCCDHeader->nextStream = NULL;
						else
						{
							newCCDHeader->nextStream = ccdHeader;
							ccdHeader->isFromPreviousPass = 1;
						}

						//And write the force and contact count
						PX_ASSERT(contactForces != NULL);
						contactForces[0] = p.mAppliedForce;
					}
					else if (!ccdHeader)
					{
						p.mCm->mNpUnit.ccdContacts = NULL;
						// we do not set the status flag on failure because the pair might have written
						// a contact stream sucessfully during discrete collision this frame.
					}
					else
						ccdHeader->isFromPreviousPass = 1;

					//If the touch event already existed, the solver would have already configured the threshold stream
					if((p.mCm->mNpUnit.flags & (PxcNpWorkUnitFlag::eARTICULATION_BODY0 | PxcNpWorkUnitFlag::eARTICULATION_BODY1)) == 0 && p.mAppliedForce)
					{
#if 1
						ThresholdStreamElement elt;
						elt.normalForce = p.mAppliedForce;
						elt.accumulatedForce = 0.f;
						elt.threshold = PxMin<float>(p.mBa0 == NULL ? PX_MAX_REAL : p.mBa0->mCore->contactReportThreshold, p.mBa1 == NULL ? PX_MAX_REAL : 
							p.mBa1->mCore->contactReportThreshold);
						elt.nodeIndexA = p.mCCDShape0->mNodeIndex;
						elt.nodeIndexB =p.mCCDShape1->mNodeIndex;
						Ps::order(elt.nodeIndexA,elt.nodeIndexB);
						PX_ASSERT(elt.nodeIndexA.index() < elt.nodeIndexB.index());
						mThresholdStream.pushBack(elt);
#endif
					}
				}
			}
		}
		index = islandEnd;
	}

	mContext->mCMTouchEventCount[PXS_LOST_TOUCH_COUNT] += countLost;
	mContext->mCMTouchEventCount[PXS_NEW_TOUCH_COUNT] += countFound;
	mContext->mCMTouchEventCount[PXS_CCD_RETOUCH_COUNT] += countRetouch;
}

void PxsCCDContext::postCCDDepenetrate(PxBaseTask* /*continuation*/)
{
	// --------------------------------------------------------------------------------------
	// reset mOverlappingShapes array for all bodies
	// we do it each pass because this set can change due to movement as well as new objects
	// becoming fast moving due to intra-frame impacts

	for (PxU32 j = 0; j < mCCDBodies.size(); j ++)
	{
		mCCDBodies[j].mOverlappingObjects = NULL;
		mCCDBodies[j].mNbInteractionsThisPass = 0;
	}

	mCCDOverlaps.clear_NoDelete();

	updateCCDEnd();

	mContext->putNpThreadContext(mCCDThreadContext);

	flushCCDLog();
}

Cm::SpatialVector PxsRigidBody::getPreSolverVelocities() const
{
	if (mCCD)
		return mCCD->mPreSolverVelocity;
	return Cm::SpatialVector(PxVec3(0.f), PxVec3(0.f));
}


PxTransform PxsRigidBody::getAdvancedTransform(PxReal toi) const
{
	//If it is kinematic, just return identity. We don't fully support kinematics yet
	if (isKinematic())
		return PxTransform(PxIdentity);

	//Otherwise we interpolate the pose between the current and previous pose and return that pose
	PxVec3 newLastP = mLastTransform.p*(1.0f-toi) + mCore->body2World.p*toi; // advance mLastTransform position to toi
	PxQuat newLastQ = slerp(toi, getLastCCDTransform().q, mCore->body2World.q); // advance mLastTransform rotation to toi
	return PxTransform(newLastP, newLastQ);
}

void PxsRigidBody::advancePrevPoseToToi(PxReal toi)
{
	//If this is kinematic, just return
	if (isKinematic())
		return;

	//update latest pose
	PxVec3 newLastP = mLastTransform.p*(1.0f-toi) + mCore->body2World.p*toi; // advance mLastTransform position to toi
	mLastTransform.p = newLastP;
#if CCD_ROTATION_LOCKING
	mCore->body2World.q = getLastCCDTransform().q;
#else
	// slerp from last transform to current transform with ratio of toi
	PxQuat newLastQ = slerp(toi, getLastCCDTransform().q, mCore->body2World.q); // advance mLastTransform rotation to toi
	mLastTransform.q = newLastQ;
#endif

	

}


void PxsRigidBody::advanceToToi(PxReal toi, PxReal dt, bool clip)
{
	if (isKinematic())
		return;

	
	if (clip)
	{
		//If clip is true, we set the previous and current pose to be the same. This basically makes the object appear stationary in the CCD
		mCore->body2World.p = getLastCCDTransform().p;
#if !CCD_ROTATION_LOCKING
		mCore->body2World.q = getLastCCDTransform().q;
#endif
	}
	else
	{
		// advance new CCD target after impact to remaining toi using post-impact velocities
		mCore->body2World.p = getLastCCDTransform().p + getLinearVelocity() * dt * (1.0f - toi);
#if !CCD_ROTATION_LOCKING
		PxVec3 angularDelta = getAngularVelocity() * dt * (1.0f - toi);
		PxReal deltaMag = angularDelta.magnitude();
		PxVec3 deltaAng = deltaMag > 1e-20f ? angularDelta / deltaMag : PxVec3(1.0f, 0.0f, 0.0f);
		PxQuat angularQuat(deltaMag, deltaAng);
		mCore->body2World.q = getLastCCDTransform().q * angularQuat;
#endif
		PX_ASSERT(mCore->body2World.isSane());
	}

	// rescale total time left to elapse this frame
	mCCD->mTimeLeft = PxMax(mCCD->mTimeLeft * (1.0f - toi), CCD_MIN_TIME_LEFT);
}


void PxsCCDContext::runCCDModifiableContact(PxModifiableContact* PX_RESTRICT contacts, PxU32 contactCount, const PxsShapeCore* PX_RESTRICT shapeCore0, 
											const PxsShapeCore* PX_RESTRICT shapeCore1, const PxsRigidCore* PX_RESTRICT rigidCore0, const PxsRigidCore* PX_RESTRICT rigidCore1,
											const PxsRigidBody* PX_RESTRICT rigid0, const PxsRigidBody* PX_RESTRICT rigid1)
{
	if(!mCCDContactModifyCallback)
		return;

	class PxcContactSet: public PxContactSet
	{
	public:
		PxcContactSet(PxU32 count, PxModifiableContact* contacts_)
		{
			mContacts = contacts_;
			mCount = count;
		}
	};
	{
			PxContactModifyPair p;

			p.shape[0] = gPxvOffsetTable.convertPxsShape2Px(shapeCore0);
			p.shape[1] = gPxvOffsetTable.convertPxsShape2Px(shapeCore1);

			p.actor[0] = rigid0 != NULL ? gPxvOffsetTable.convertPxsRigidCore2PxRigidBody(rigidCore0) 
										: gPxvOffsetTable.convertPxsRigidCore2PxRigidStatic(rigidCore0);

			p.actor[1] = rigid1 != NULL ? gPxvOffsetTable.convertPxsRigidCore2PxRigidBody(rigidCore1) 
										: gPxvOffsetTable.convertPxsRigidCore2PxRigidStatic(rigidCore1);

			p.transform[0] = getShapeAbsPose(shapeCore0, rigidCore0, PxU32(rigid0 != NULL));
			p.transform[1] = getShapeAbsPose(shapeCore1, rigidCore1, PxU32(rigid1 != NULL));

			static_cast<PxcContactSet&>(p.contacts) = 
				PxcContactSet(contactCount, contacts);

			mCCDContactModifyCallback->onCCDContactModify(&p, 1);
	}
}


} //namespace physx


