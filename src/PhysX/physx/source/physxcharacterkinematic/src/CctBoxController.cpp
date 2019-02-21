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

#include "PxController.h"
#include "CctBoxController.h"
#include "CctCharacterControllerManager.h"
#include "PxBoxGeometry.h"
#include "PxRigidDynamic.h"
#include "PxShape.h"

using namespace physx;
using namespace Cct;

static PX_FORCE_INLINE PxVec3 CCTtoProxyExtents(PxF32 halfHeight, PxF32 halfSideExtent, PxF32 halfForwardExtent, PxF32 coeff)
{
	// PT: because we now orient the box CCT using the same quat as for capsules...
	// i.e. the identity quat corresponds to a up dir = 1,0,0 (which is like the worst choice we could have made, of course)
	return PxVec3(halfHeight * coeff, halfSideExtent * coeff, halfForwardExtent * coeff);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

BoxController::BoxController(const PxControllerDesc& desc, PxPhysics& sdk, PxScene* s) : Controller(desc, s)
{
	mType = PxControllerShapeType::eBOX;

	const PxBoxControllerDesc& bc = static_cast<const PxBoxControllerDesc&>(desc);

	mHalfHeight			= bc.halfHeight;
	mHalfSideExtent		= bc.halfSideExtent;
	mHalfForwardExtent	= bc.halfForwardExtent;

	// Create kinematic actor under the hood
	PxBoxGeometry boxGeom;
	boxGeom.halfExtents = CCTtoProxyExtents(bc.halfHeight, bc.halfSideExtent, bc.halfForwardExtent, mProxyScaleCoeff);

	createProxyActor(sdk, boxGeom, *desc.material);
}

BoxController::~BoxController()
{
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void BoxController::invalidateCache()
{
	if(mManager->mLockingEnabled)
		mWriteLock.lock();

	mCctModule.voidTestCache();

	if(mManager->mLockingEnabled)
		mWriteLock.unlock();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool BoxController::getWorldBox(PxExtendedBounds3& box) const
{
	setCenterExtents(box, mPosition, PxVec3(mHalfHeight, mHalfSideExtent, mHalfForwardExtent));
	return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

PxF32 BoxController::getHalfHeight() const
{
	return mHalfHeight;
}

PxF32 BoxController::getHalfSideExtent() const
{
	return mHalfSideExtent;
}

PxF32 BoxController::getHalfForwardExtent() const
{
	return mHalfForwardExtent;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool BoxController::updateKinematicProxy()
{
	// Set extents for kinematic proxy
	if(mKineActor)
	{
		PxShape* shape = getKineShape();

		PX_ASSERT(shape->getGeometryType() == PxGeometryType::eBOX);
		PxBoxGeometry bg;
		shape->getBoxGeometry(bg);

		bg.halfExtents = CCTtoProxyExtents(mHalfHeight, mHalfSideExtent, mHalfForwardExtent, mProxyScaleCoeff);
		shape->setGeometry(bg);
	}
	return true;
}

bool BoxController::setHalfHeight(PxF32 halfHeight)
{
	if(halfHeight<=0.0f)
		return false;

	mHalfHeight = halfHeight;
	return updateKinematicProxy();
}

bool BoxController::setHalfSideExtent(PxF32 halfSideExtent)
{
	if(halfSideExtent<=0.0f)
		return false;

	mHalfSideExtent = halfSideExtent;
	return updateKinematicProxy();
}

bool BoxController::setHalfForwardExtent(PxF32 halfForwardExtent)
{
	if(halfForwardExtent<=0.0f)
		return false;

	mHalfForwardExtent = halfForwardExtent;
	return updateKinematicProxy();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

PxExtendedVec3 BoxController::getFootPosition() const
{
	PxExtendedVec3 groundPosition = mPosition;													// Middle of the CCT
	groundPosition -= mUserParams.mUpDirection * (mHalfHeight + mUserParams.mContactOffset);	// Ground
	return groundPosition;
}

bool BoxController::setFootPosition(const PxExtendedVec3& position)
{
	PxExtendedVec3 centerPosition = position;
	centerPosition += mUserParams.mUpDirection * (mHalfHeight + mUserParams.mContactOffset);
	return setPosition(centerPosition);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void BoxController::getOBB(PxExtendedBox& obb) const
{
	// PT: TODO: optimize this
	PxExtendedBounds3 worldBox;
	getWorldBox(worldBox);

	getCenter(worldBox, obb.center);
	getExtents(worldBox, obb.extents);
	obb.rot = mUserParams.mQuatFromUp;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void BoxController::resize(PxReal height)
{
	const float oldHeight = getHalfHeight();
	setHalfHeight(height);

	const float delta = height - oldHeight;
	PxExtendedVec3 pos = getPosition();
	pos += mUserParams.mUpDirection * delta;
	setPosition(pos);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

