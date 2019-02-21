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

           
#include "ScIterators.h"
#include "ScBodySim.h"
#include "ScShapeSim.h"
#include "ScShapeInteraction.h"

using namespace physx;

///////////////////////////////////////////////////////////////////////////////

Sc::ContactIterator::Pair::Pair(const void*& contactPatches, const void*& contactPoints, PxU32 /*contactDataSize*/, const PxReal*& forces, PxU32 numContacts, PxU32 numPatches,
	ShapeSim& shape0, ShapeSim& shape1)
: mIndex(0)
, mNumContacts(numContacts)
, mIter(reinterpret_cast<const PxU8*>(contactPatches), reinterpret_cast<const PxU8*>(contactPoints), reinterpret_cast<const PxU32*>(forces + numContacts), numPatches, numContacts)
, mForces(forces)
{	
	mCurrentContact.shape0 = shape0.getPxShape();
	mCurrentContact.shape1 = shape1.getPxShape();
	mCurrentContact.normalForceAvailable = (forces != NULL);
}

Sc::ContactIterator::Pair* Sc::ContactIterator::getNextPair()
{ 
	if(mCurrent < mLast)
	{
		ShapeInteraction* si = static_cast<ShapeInteraction*>(*mCurrent);

		const void* contactPatches = NULL;
		const void* contactPoints = NULL;
		PxU32 contactDataSize = 0;
		const PxReal* forces = NULL;
		PxU32 numContacts = 0;
		PxU32 numPatches = 0;

		PxU32 nextOffset = si->getContactPointData(contactPatches, contactPoints, contactDataSize, numContacts, numPatches, forces, mOffset, *mOutputs);

		if (nextOffset == mOffset)
			++mCurrent;
		else
			mOffset = nextOffset;

		mCurrentPair = Pair(contactPatches, contactPoints, contactDataSize, forces, numContacts, numPatches, si->getShape0(), si->getShape1());
		return &mCurrentPair;
	}
	else
		return NULL;
}

Sc::Contact* Sc::ContactIterator::Pair::getNextContact()
{
	if(mIndex < mNumContacts)
	{
		if(!mIter.hasNextContact())
		{
			if(!mIter.hasNextPatch())
				return NULL;
			mIter.nextPatch();
		}
		PX_ASSERT(mIter.hasNextContact());
		mIter.nextContact();

		mCurrentContact.normal = mIter.getContactNormal();
		mCurrentContact.point = mIter.getContactPoint();
		mCurrentContact.separation = mIter.getSeparation();
		mCurrentContact.normalForce = mForces ? mForces[mIndex] : 0;
		mCurrentContact.faceIndex0 = mIter.getFaceIndex0();
		mCurrentContact.faceIndex1 = mIter.getFaceIndex1();

		mIndex++;
		return &mCurrentContact;
	}
	return NULL;
}

///////////////////////////////////////////////////////////////////////////////
