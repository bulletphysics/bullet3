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

#ifndef DY_CONTACT_REDUCTION_H
#define DY_CONTACT_REDUCTION_H

#include "GuContactPoint.h"
#include "PxsMaterialManager.h"

namespace physx
{


namespace Dy
{

//KS - might be OK with 4 but 5 guarantees the deepest + 4 contacts that contribute to largest surface area
#define CONTACT_REDUCTION_MAX_CONTACTS 6
#define CONTACT_REDUCTION_MAX_PATCHES 32
#define PXS_NORMAL_TOLERANCE 0.995f
#define PXS_SEPARATION_TOLERANCE 0.001f


	//A patch contains a normal, pair of material indices and a list of indices. These indices are 
	//used to index into the PxContact array that's passed by the user
	struct ReducedContactPatch
	{
		PxU32 numContactPoints;
		PxU32 contactPoints[CONTACT_REDUCTION_MAX_CONTACTS];	
	};

	struct ContactPatch
	{	
		PxVec3 rootNormal;
		ContactPatch* mNextPatch;
		PxReal maxPenetration;
		PxU16 startIndex;
		PxU16 stride;
		PxU16 rootIndex;
		PxU16 index;
	};

	struct SortBoundsPredicateManifold
	{
		bool operator()(const ContactPatch* idx1, const ContactPatch* idx2) const
		{
			return idx1->maxPenetration < idx2->maxPenetration;
		}
	};



	template <PxU32 MaxPatches>
	class ContactReduction
	{
	public:
		ReducedContactPatch mPatches[MaxPatches];
		PxU32 mNumPatches;
		ContactPatch mIntermediatePatches[CONTACT_REDUCTION_MAX_PATCHES];
		ContactPatch* mIntermediatePatchesPtrs[CONTACT_REDUCTION_MAX_PATCHES];
		PxU32 mNumIntermediatePatches;
		Gu::ContactPoint* PX_RESTRICT mOriginalContacts;
		PxsMaterialInfo* PX_RESTRICT mMaterialInfo;
		PxU32 mNumOriginalContacts;

		ContactReduction(Gu::ContactPoint* PX_RESTRICT originalContacts, PxsMaterialInfo* PX_RESTRICT materialInfo, PxU32 numContacts) : 
		mNumPatches(0), mNumIntermediatePatches(0),	mOriginalContacts(originalContacts), mMaterialInfo(materialInfo), mNumOriginalContacts(numContacts)
		{
		}

		void reduceContacts()
		{
			//First pass, break up into contact patches, storing the start and stride of the patches
			//We will need to have contact patches and then coallesce them
			mIntermediatePatches[0].rootNormal = mOriginalContacts[0].normal;
			mIntermediatePatches[0].mNextPatch = NULL;
			mIntermediatePatches[0].startIndex = 0;
			mIntermediatePatches[0].rootIndex = 0;
			mIntermediatePatches[0].maxPenetration = mOriginalContacts[0].separation;
			mIntermediatePatches[0].index = 0;
			PxU16 numPatches = 1;
			//PxU32 startIndex = 0;
			PxU32 numUniquePatches = 1;
			PxU16 m = 1;
			for(; m < mNumOriginalContacts; ++m)
			{
				PxI32 index = -1;
				for(PxU32 b = numPatches; b > 0; --b)
				{
					ContactPatch& patch = mIntermediatePatches[b-1];
					if(mMaterialInfo[patch.startIndex].mMaterialIndex0 == mMaterialInfo[m].mMaterialIndex0 && mMaterialInfo[patch.startIndex].mMaterialIndex1 == mMaterialInfo[m].mMaterialIndex1 && 
						patch.rootNormal.dot(mOriginalContacts[m].normal) >= PXS_NORMAL_TOLERANCE)
					{
						index = PxI32(b-1);
						break;
					}
				}

				if(index != numPatches - 1)
				{
					mIntermediatePatches[numPatches-1].stride = PxU16(m - mIntermediatePatches[numPatches - 1].startIndex);
					//Create a new patch...
					if(numPatches == CONTACT_REDUCTION_MAX_PATCHES)
					{
						break;
					}
					mIntermediatePatches[numPatches].startIndex = m;
					mIntermediatePatches[numPatches].mNextPatch = NULL;
					if(index == -1)
					{
						mIntermediatePatches[numPatches].rootIndex = numPatches;
						mIntermediatePatches[numPatches].rootNormal = mOriginalContacts[m].normal;
						mIntermediatePatches[numPatches].maxPenetration = mOriginalContacts[m].separation;
						mIntermediatePatches[numPatches].index = numPatches;
						++numUniquePatches;
					}
					else
					{
						//Find last element in the link
						PxU16 rootIndex = mIntermediatePatches[index].rootIndex;
						mIntermediatePatches[index].mNextPatch = &mIntermediatePatches[numPatches];
						mIntermediatePatches[numPatches].rootNormal = mIntermediatePatches[index].rootNormal;
						mIntermediatePatches[rootIndex].maxPenetration = mIntermediatePatches[numPatches].maxPenetration = PxMin(mIntermediatePatches[rootIndex].maxPenetration, mOriginalContacts[m].separation);
						mIntermediatePatches[numPatches].rootIndex = rootIndex;
						mIntermediatePatches[numPatches].index = numPatches;
					}
					++numPatches;
				}
			}
			mIntermediatePatches[numPatches-1].stride = PxU16(m - mIntermediatePatches[numPatches-1].startIndex);

			//OK, we have a list of contact patches so that we can start contact reduction per-patch

			//OK, now we can go and reduce the contacts on a per-patch basis...

			for(PxU32 a = 0; a < numPatches; ++a)
			{
				mIntermediatePatchesPtrs[a] = &mIntermediatePatches[a];
			}


			SortBoundsPredicateManifold predicate;
			Ps::sort(mIntermediatePatchesPtrs, numPatches, predicate);

			PxU32 numReducedPatches = 0;
			for(PxU32 a = 0; a < numPatches; ++a)
			{
				if(mIntermediatePatchesPtrs[a]->rootIndex == mIntermediatePatchesPtrs[a]->index)
				{
					//Reduce this patch...
					if(numReducedPatches == MaxPatches)
						break;

					ReducedContactPatch& reducedPatch = mPatches[numReducedPatches++];
					//OK, now we need to work out if we have to reduce patches...
					PxU32 contactCount = 0;
					{
						ContactPatch* tmpPatch = mIntermediatePatchesPtrs[a];

						while(tmpPatch)
						{
							contactCount += tmpPatch->stride;
							tmpPatch = tmpPatch->mNextPatch;
						}
					}

					if(contactCount <= CONTACT_REDUCTION_MAX_CONTACTS)
					{
						//Just add the contacts...
						ContactPatch* tmpPatch = mIntermediatePatchesPtrs[a];

						PxU32 ind = 0;
						while(tmpPatch)
						{
							for(PxU32 b = 0; b < tmpPatch->stride; ++b)
							{
								reducedPatch.contactPoints[ind++] = tmpPatch->startIndex + b;
							}
							tmpPatch = tmpPatch->mNextPatch;
						}
						reducedPatch.numContactPoints = contactCount;
					}
					else
					{
						//Iterate through and find the most extreme point
						

						PxU32 ind = 0;

						{
							PxReal dist = 0.f;
							ContactPatch* tmpPatch = mIntermediatePatchesPtrs[a];
							while(tmpPatch)
							{
								for(PxU32 b = 0; b < tmpPatch->stride; ++b)
								{
									PxReal magSq = mOriginalContacts[tmpPatch->startIndex + b].point.magnitudeSquared();
									if(dist < magSq)
									{
										ind = tmpPatch->startIndex + b;
										dist = magSq;
									}
								}
								tmpPatch = tmpPatch->mNextPatch;
							}
						}	
						reducedPatch.contactPoints[0] = ind;
						const PxVec3 p0 = mOriginalContacts[ind].point;

						//Now find the point farthest from this point...						
						{
							PxReal maxDist = 0.f;
							ContactPatch* tmpPatch = mIntermediatePatchesPtrs[a];
							while(tmpPatch)
							{
								for(PxU32 b = 0; b < tmpPatch->stride; ++b)
								{
									PxReal magSq = (p0 - mOriginalContacts[tmpPatch->startIndex + b].point).magnitudeSquared();
									if(magSq > maxDist)
									{
										ind = tmpPatch->startIndex + b;
										maxDist = magSq;
									}
								}
								tmpPatch = tmpPatch->mNextPatch;
							}
						}
						reducedPatch.contactPoints[1] = ind;
						const PxVec3 p1 = mOriginalContacts[ind].point;

						//Now find the point farthest from the segment

						PxVec3 n = (p0 - p1).cross(mIntermediatePatchesPtrs[a]->rootNormal);

						//PxReal tVal = 0.f;
						{
							PxReal maxDist = 0.f;
							//PxReal tmpTVal;
							
							ContactPatch* tmpPatch = mIntermediatePatchesPtrs[a];
							while(tmpPatch)
							{
								for(PxU32 b = 0; b < tmpPatch->stride; ++b)
								{
									
									//PxReal magSq = tmpDistancePointSegmentSquared(p0, p1, mOriginalContacts[tmpPatch->startIndex + b].point, tmpTVal);
									PxReal magSq = (mOriginalContacts[tmpPatch->startIndex + b].point - p0).dot(n);
									if(magSq > maxDist)
									{
										ind = tmpPatch->startIndex + b;
										//tVal = tmpTVal;
										maxDist = magSq;
									}
								}
								tmpPatch = tmpPatch->mNextPatch;
							}
						}
						reducedPatch.contactPoints[2] = ind;

						//const PxVec3 closest = (p0 + (p1 - p0) * tVal);

						const PxVec3 dir = -n;//closest - p3;

						{
							PxReal maxDist = 0.f;
							//PxReal tVal = 0.f;
							ContactPatch* tmpPatch = mIntermediatePatchesPtrs[a];
							while(tmpPatch)
							{
								for(PxU32 b = 0; b < tmpPatch->stride; ++b)
								{
									PxReal magSq =  (mOriginalContacts[tmpPatch->startIndex + b].point - p0).dot(dir);
									if(magSq > maxDist)
									{
										ind = tmpPatch->startIndex + b;
										maxDist = magSq;
									}
								}
								tmpPatch = tmpPatch->mNextPatch;
							}
						}
						reducedPatch.contactPoints[3] = ind;

						//Now, we iterate through all the points, and cluster the points. From this, we establish the deepest point that's within a 
						//tolerance of this point and keep that point

						PxReal separation[CONTACT_REDUCTION_MAX_CONTACTS];
						PxU32 deepestInd[CONTACT_REDUCTION_MAX_CONTACTS];
						for(PxU32 i = 0; i < 4; ++i)
						{
							PxU32 index = reducedPatch.contactPoints[i];
							separation[i] = mOriginalContacts[index].separation - PXS_SEPARATION_TOLERANCE;
							deepestInd[i] = index;
						}

						ContactPatch* tmpPatch = mIntermediatePatchesPtrs[a];
						while(tmpPatch)
						{
							for(PxU32 b = 0; b < tmpPatch->stride; ++b)
							{
								Gu::ContactPoint& point = mOriginalContacts[tmpPatch->startIndex + b];
								
								PxReal distance = PX_MAX_REAL;
								PxU32 index = 0;
								for(PxU32 c = 0; c < 4; ++c)
								{
									PxVec3 dif = mOriginalContacts[reducedPatch.contactPoints[c]].point - point.point;
									PxReal d = dif.magnitudeSquared();
									if(distance > d)
									{
										distance = d;
										index = c;
									}
								}
								if(separation[index] > point.separation)
								{
									deepestInd[index] = tmpPatch->startIndex+b;
									separation[index] = point.separation;
								}

							}
							tmpPatch = tmpPatch->mNextPatch;
						}

						bool chosen[64];
						PxMemZero(chosen, sizeof(chosen));
						for(PxU32 i = 0; i < 4; ++i)
						{
							reducedPatch.contactPoints[i] = deepestInd[i];
							chosen[deepestInd[i]] = true;
						}						
						
						for(PxU32 i = 4; i < CONTACT_REDUCTION_MAX_CONTACTS; ++i)
						{
							separation[i] = PX_MAX_REAL;
							deepestInd[i] = 0;
						}
						tmpPatch = mIntermediatePatchesPtrs[a];
						while(tmpPatch)
						{
							for(PxU32 b = 0; b < tmpPatch->stride; ++b)
							{
								if(!chosen[tmpPatch->startIndex+b])
								{
									Gu::ContactPoint& point = mOriginalContacts[tmpPatch->startIndex + b];	
									for(PxU32 j = 4; j < CONTACT_REDUCTION_MAX_CONTACTS; ++j)
									{
										if(point.separation < separation[j])
										{
											for(PxU32 k = CONTACT_REDUCTION_MAX_CONTACTS-1; k > j; --k)
											{
												separation[k] = separation[k-1];
												deepestInd[k] = deepestInd[k-1];
											}
											separation[j] = point.separation;
											deepestInd[j] = tmpPatch->startIndex+b;
											break;
										}
									}
								}
							}
							tmpPatch = tmpPatch->mNextPatch;
						}

						for(PxU32 i = 4; i < CONTACT_REDUCTION_MAX_CONTACTS; ++i)
						{
							reducedPatch.contactPoints[i] = deepestInd[i];
						}

						reducedPatch.numContactPoints = CONTACT_REDUCTION_MAX_CONTACTS;
					}
				}
			}
			mNumPatches = numReducedPatches;
		}

	};
}

}


#endif //DY_CONTACT_REDUCTION_H
