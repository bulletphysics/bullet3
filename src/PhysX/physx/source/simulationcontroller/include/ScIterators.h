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
      

#ifndef PX_PHYSICS_SCP_ITERATOR
#define PX_PHYSICS_SCP_ITERATOR

#include "foundation/PxVec3.h"
#include "PxContact.h"

namespace physx
{
class PxShape;
class PxsContactManagerOutputIterator;

namespace Sc
{
	class ShapeSim;
	class Interaction;
	
	struct Contact
	{
		Contact() 
			: normal(0.0f)
			, point(0.0f)
			, separation(0.0f)
			, normalForce(0.0f)
		{}

		PxVec3 normal;
		PxVec3 point;
		PxShape* shape0;
		PxShape* shape1;
		PxReal separation;
		PxReal normalForce;
		PxU32 faceIndex0;  // these are the external indices
		PxU32 faceIndex1;
		bool normalForceAvailable;
	};

	class ContactIterator
	{
		public:		

			class Pair
			{
			public:
				Pair() : mIter(NULL, NULL, NULL, 0, 0) {}
				Pair(const void*& contactPatches, const void*& contactPoints, const PxU32 /*contactDataSize*/, const PxReal*& forces, PxU32 numContacts, PxU32 numPatches, ShapeSim& shape0, ShapeSim& shape1);
				Contact* getNextContact();

			private:
				PxU32						mIndex;
				PxU32						mNumContacts;
				PxContactStreamIterator		mIter;
				const PxReal*				mForces;
				Contact						mCurrentContact;
			};

			ContactIterator() {}
			explicit ContactIterator(Interaction** first, Interaction** last, PxsContactManagerOutputIterator& outputs): mCurrent(first), mLast(last), mOffset(0), mOutputs(&outputs) {}
			Pair* getNextPair();

		private:
			Interaction**					mCurrent;
			Interaction**					mLast;
			Pair							mCurrentPair;
			PxU32							mOffset;
			PxsContactManagerOutputIterator* mOutputs;

	private:
	};

}  // namespace Sc

}

#endif
