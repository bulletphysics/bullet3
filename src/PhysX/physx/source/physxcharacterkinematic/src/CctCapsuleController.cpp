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

#include "PxController.h"
#include "CctCapsuleController.h"
#include "CctCharacterControllerManager.h"
#include "PxCapsuleGeometry.h"
#include "PxRigidDynamic.h"
#include "PxShape.h"

using namespace physx;
using namespace Cct;

static PX_FORCE_INLINE float CCTtoProxyRadius(float r, PxF32 coeff)	{ return r * coeff;			}
static PX_FORCE_INLINE float CCTtoProxyHeight(float h, PxF32 coeff)	{ return 0.5f * h * coeff;	}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

CapsuleController::CapsuleController(const PxControllerDesc& desc, PxPhysics& sdk, PxScene* s) : Controller(desc, s)
{
	mType = PxControllerShapeType::eCAPSULE;

	const PxCapsuleControllerDesc& cc = static_cast<const PxCapsuleControllerDesc&>(desc);

	mRadius			= cc.radius;
	mHeight			= cc.height;
	mClimbingMode	= cc.climbingMode;

	// Create kinematic actor under the hood
	PxCapsuleGeometry capsGeom;
	capsGeom.radius		= CCTtoProxyRadius(cc.radius, mProxyScaleCoeff);
	capsGeom.halfHeight	= CCTtoProxyHeight(cc.height, mProxyScaleCoeff);

	createProxyActor(sdk, capsGeom, *desc.material);
}

CapsuleController::~CapsuleController()
{
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void CapsuleController::invalidateCache()
{
	if(mManager->mLockingEnabled)
		mWriteLock.lock();

	mCctModule.voidTestCache();

	if(mManager->mLockingEnabled)
		mWriteLock.unlock();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool CapsuleController::getWorldBox(PxExtendedBounds3& box) const
{
	setCenterExtents(box, mPosition, PxVec3(mRadius, mRadius+mHeight*0.5f, mRadius));
	return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool CapsuleController::setRadius(PxF32 r)
{
	// Set radius for CCT volume
	mRadius = r;

	// Set radius for kinematic proxy
	if(mKineActor)
	{
		PxShape* shape = getKineShape();

		PX_ASSERT(shape->getGeometryType() == PxGeometryType::eCAPSULE);
		PxCapsuleGeometry cg;
		shape->getCapsuleGeometry(cg);

		cg.radius = CCTtoProxyRadius(r, mProxyScaleCoeff);
		shape->setGeometry(cg);
	}
	return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool CapsuleController::setHeight(PxF32 h)
{
	// Set height for CCT volume
	mHeight = h;

	// Set height for kinematic proxy
	if(mKineActor)
	{
		PxShape* shape = getKineShape();

		PX_ASSERT(shape->getGeometryType() == PxGeometryType::eCAPSULE);
		PxCapsuleGeometry cg;
		shape->getCapsuleGeometry(cg);

		cg.halfHeight = CCTtoProxyHeight(h, mProxyScaleCoeff);
		shape->setGeometry(cg);
	}
	return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

PxCapsuleClimbingMode::Enum CapsuleController::getClimbingMode() const
{
	return mClimbingMode;
}

bool CapsuleController::setClimbingMode(PxCapsuleClimbingMode::Enum mode)
{
	if(mode>=PxCapsuleClimbingMode::eLAST)
		return false;
	mClimbingMode = mode;
	return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

PxExtendedVec3 CapsuleController::getFootPosition() const
{
	PxExtendedVec3 groundPosition = mPosition;														// Middle of the CCT
	groundPosition -= mUserParams.mUpDirection * (mUserParams.mContactOffset+mRadius+mHeight*0.5f);	// Ground
	return groundPosition;
}

bool CapsuleController::setFootPosition(const PxExtendedVec3& position)
{
	PxExtendedVec3 centerPosition = position;
	centerPosition += mUserParams.mUpDirection * (mUserParams.mContactOffset+mRadius+mHeight*0.5f);
	return setPosition(centerPosition);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void CapsuleController::getCapsule(PxExtendedCapsule& capsule) const
{
	// PT: TODO: optimize this
	PxExtendedVec3 p0 = mPosition;
	PxExtendedVec3 p1 = mPosition;
	const PxVec3 extents = mUserParams.mUpDirection*mHeight*0.5f;
	p0 -= extents;
	p1 += extents;

	capsule.p0		= p0;
	capsule.p1		= p1;
	capsule.radius	= mRadius;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void CapsuleController::resize(PxReal height)
{
	const float oldHeight = getHeight();
	setHeight(height);

	const float delta = height - oldHeight;
	PxExtendedVec3 pos = getPosition();
	pos += mUserParams.mUpDirection * delta * 0.5f;
	setPosition(pos);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
