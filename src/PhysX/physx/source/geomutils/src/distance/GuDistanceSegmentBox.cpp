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

#include "GuDistanceSegmentBox.h"
#include "GuDistancePointBox.h"
#include "GuDistanceSegmentSegment.h"
#include "GuDistancePointSegment.h"
#include "GuIntersectionRayBox.h"

using namespace physx;

static void face(unsigned int i0, unsigned int i1, unsigned int i2, PxVec3& rkPnt, const PxVec3& rkDir, const PxVec3& extents, const PxVec3& rkPmE, PxReal* pfLParam, PxReal& rfSqrDistance)
{
	PxVec3 kPpE;
	PxReal fLSqr, fInv, fTmp, fParam, fT, fDelta;

	kPpE[i1] = rkPnt[i1] + extents[i1];
	kPpE[i2] = rkPnt[i2] + extents[i2];
	if(rkDir[i0]*kPpE[i1] >= rkDir[i1]*rkPmE[i0])
	{
		if(rkDir[i0]*kPpE[i2] >= rkDir[i2]*rkPmE[i0])
		{
			// v[i1] >= -e[i1], v[i2] >= -e[i2] (distance = 0)
			if(pfLParam)
			{
				rkPnt[i0] = extents[i0];
				fInv = 1.0f/rkDir[i0];
				rkPnt[i1] -= rkDir[i1]*rkPmE[i0]*fInv;
				rkPnt[i2] -= rkDir[i2]*rkPmE[i0]*fInv;
				*pfLParam = -rkPmE[i0]*fInv;
			}
		}
		else
		{
			// v[i1] >= -e[i1], v[i2] < -e[i2]
			fLSqr = rkDir[i0]*rkDir[i0] + rkDir[i2]*rkDir[i2];
			fTmp = fLSqr*kPpE[i1] - rkDir[i1]*(rkDir[i0]*rkPmE[i0] + rkDir[i2]*kPpE[i2]);
			if(fTmp <= 2.0f*fLSqr*extents[i1])
			{
				fT = fTmp/fLSqr;
				fLSqr += rkDir[i1]*rkDir[i1];
				fTmp = kPpE[i1] - fT;
				fDelta = rkDir[i0]*rkPmE[i0] + rkDir[i1]*fTmp + rkDir[i2]*kPpE[i2];
				fParam = -fDelta/fLSqr;
				rfSqrDistance += rkPmE[i0]*rkPmE[i0] + fTmp*fTmp + kPpE[i2]*kPpE[i2] + fDelta*fParam;

				if(pfLParam)
				{
					*pfLParam = fParam;
					rkPnt[i0] = extents[i0];
					rkPnt[i1] = fT - extents[i1];
					rkPnt[i2] = -extents[i2];
				}
			}
			else
			{
				fLSqr += rkDir[i1]*rkDir[i1];
				fDelta = rkDir[i0]*rkPmE[i0] + rkDir[i1]*rkPmE[i1] + rkDir[i2]*kPpE[i2];
				fParam = -fDelta/fLSqr;
				rfSqrDistance += rkPmE[i0]*rkPmE[i0] + rkPmE[i1]*rkPmE[i1] + kPpE[i2]*kPpE[i2] + fDelta*fParam;

				if(pfLParam)
				{
					*pfLParam = fParam;
					rkPnt[i0] = extents[i0];
					rkPnt[i1] = extents[i1];
					rkPnt[i2] = -extents[i2];
				}
			}
		}
	}
	else
	{
		if ( rkDir[i0]*kPpE[i2] >= rkDir[i2]*rkPmE[i0] )
		{
			// v[i1] < -e[i1], v[i2] >= -e[i2]
			fLSqr = rkDir[i0]*rkDir[i0] + rkDir[i1]*rkDir[i1];
			fTmp = fLSqr*kPpE[i2] - rkDir[i2]*(rkDir[i0]*rkPmE[i0] + rkDir[i1]*kPpE[i1]);
			if(fTmp <= 2.0f*fLSqr*extents[i2])
			{
				fT = fTmp/fLSqr;
				fLSqr += rkDir[i2]*rkDir[i2];
				fTmp = kPpE[i2] - fT;
				fDelta = rkDir[i0]*rkPmE[i0] + rkDir[i1]*kPpE[i1] + rkDir[i2]*fTmp;
				fParam = -fDelta/fLSqr;
				rfSqrDistance += rkPmE[i0]*rkPmE[i0] + kPpE[i1]*kPpE[i1] + fTmp*fTmp + fDelta*fParam;

				if(pfLParam)
				{
					*pfLParam = fParam;
					rkPnt[i0] = extents[i0];
					rkPnt[i1] = -extents[i1];
					rkPnt[i2] = fT - extents[i2];
				}
			}
			else
			{
				fLSqr += rkDir[i2]*rkDir[i2];
				fDelta = rkDir[i0]*rkPmE[i0] + rkDir[i1]*kPpE[i1] + rkDir[i2]*rkPmE[i2];
				fParam = -fDelta/fLSqr;
				rfSqrDistance += rkPmE[i0]*rkPmE[i0] + kPpE[i1]*kPpE[i1] + rkPmE[i2]*rkPmE[i2] + fDelta*fParam;

				if(pfLParam)
				{
					*pfLParam = fParam;
					rkPnt[i0] = extents[i0];
					rkPnt[i1] = -extents[i1];
					rkPnt[i2] = extents[i2];
				}
			}
		}
		else
		{
			// v[i1] < -e[i1], v[i2] < -e[i2]
			fLSqr = rkDir[i0]*rkDir[i0]+rkDir[i2]*rkDir[i2];
			fTmp = fLSqr*kPpE[i1] - rkDir[i1]*(rkDir[i0]*rkPmE[i0] + rkDir[i2]*kPpE[i2]);
			if(fTmp >= 0.0f)
			{
				// v[i1]-edge is closest
				if ( fTmp <= 2.0f*fLSqr*extents[i1] )
				{
					fT = fTmp/fLSqr;
					fLSqr += rkDir[i1]*rkDir[i1];
					fTmp = kPpE[i1] - fT;
					fDelta = rkDir[i0]*rkPmE[i0] + rkDir[i1]*fTmp + rkDir[i2]*kPpE[i2];
					fParam = -fDelta/fLSqr;
					rfSqrDistance += rkPmE[i0]*rkPmE[i0] + fTmp*fTmp + kPpE[i2]*kPpE[i2] + fDelta*fParam;

					if(pfLParam)
					{
						*pfLParam = fParam;
						rkPnt[i0] = extents[i0];
						rkPnt[i1] = fT - extents[i1];
						rkPnt[i2] = -extents[i2];
					}
				}
				else
				{
					fLSqr += rkDir[i1]*rkDir[i1];
					fDelta = rkDir[i0]*rkPmE[i0] + rkDir[i1]*rkPmE[i1] + rkDir[i2]*kPpE[i2];
					fParam = -fDelta/fLSqr;
					rfSqrDistance += rkPmE[i0]*rkPmE[i0] + rkPmE[i1]*rkPmE[i1] + kPpE[i2]*kPpE[i2] + fDelta*fParam;

					if(pfLParam)
					{
						*pfLParam = fParam;
						rkPnt[i0] = extents[i0];
						rkPnt[i1] = extents[i1];
						rkPnt[i2] = -extents[i2];
					}
				}
				return;
			}

			fLSqr = rkDir[i0]*rkDir[i0] + rkDir[i1]*rkDir[i1];
			fTmp = fLSqr*kPpE[i2] - rkDir[i2]*(rkDir[i0]*rkPmE[i0] + rkDir[i1]*kPpE[i1]);
			if(fTmp >= 0.0f)
			{
				// v[i2]-edge is closest
				if(fTmp <= 2.0f*fLSqr*extents[i2])
				{
					fT = fTmp/fLSqr;
					fLSqr += rkDir[i2]*rkDir[i2];
					fTmp = kPpE[i2] - fT;
					fDelta = rkDir[i0]*rkPmE[i0] + rkDir[i1]*kPpE[i1] + rkDir[i2]*fTmp;
					fParam = -fDelta/fLSqr;
					rfSqrDistance += rkPmE[i0]*rkPmE[i0] + kPpE[i1]*kPpE[i1] + fTmp*fTmp + fDelta*fParam;

					if(pfLParam)
					{
						*pfLParam = fParam;
						rkPnt[i0] = extents[i0];
						rkPnt[i1] = -extents[i1];
						rkPnt[i2] = fT - extents[i2];
					}
				}
				else
				{
					fLSqr += rkDir[i2]*rkDir[i2];
					fDelta = rkDir[i0]*rkPmE[i0] + rkDir[i1]*kPpE[i1] + rkDir[i2]*rkPmE[i2];
					fParam = -fDelta/fLSqr;
					rfSqrDistance += rkPmE[i0]*rkPmE[i0] + kPpE[i1]*kPpE[i1] + rkPmE[i2]*rkPmE[i2] + fDelta*fParam;

					if(pfLParam)
					{
						*pfLParam = fParam;
						rkPnt[i0] = extents[i0];
						rkPnt[i1] = -extents[i1];
						rkPnt[i2] = extents[i2];
					}
				}
				return;
			}

			// (v[i1],v[i2])-corner is closest
			fLSqr += rkDir[i2]*rkDir[i2];
			fDelta = rkDir[i0]*rkPmE[i0] + rkDir[i1]*kPpE[i1] + rkDir[i2]*kPpE[i2];
			fParam = -fDelta/fLSqr;
			rfSqrDistance += rkPmE[i0]*rkPmE[i0] + kPpE[i1]*kPpE[i1] + kPpE[i2]*kPpE[i2] + fDelta*fParam;

			if(pfLParam)
			{
				*pfLParam = fParam;
				rkPnt[i0] = extents[i0];
				rkPnt[i1] = -extents[i1];
				rkPnt[i2] = -extents[i2];
			}
		}
	}
}

static void caseNoZeros(PxVec3& rkPnt, const PxVec3& rkDir, const PxVec3& extents, PxReal* pfLParam, PxReal& rfSqrDistance)
{
	PxVec3 kPmE(rkPnt.x - extents.x, rkPnt.y - extents.y, rkPnt.z - extents.z);

	PxReal fProdDxPy, fProdDyPx, fProdDzPx, fProdDxPz, fProdDzPy, fProdDyPz;

	fProdDxPy = rkDir.x*kPmE.y;
	fProdDyPx = rkDir.y*kPmE.x;
	if(fProdDyPx >= fProdDxPy)
	{
		fProdDzPx = rkDir.z*kPmE.x;
		fProdDxPz = rkDir.x*kPmE.z;
		if(fProdDzPx >= fProdDxPz)
		{
			// line intersects x = e0
			face(0, 1, 2, rkPnt, rkDir, extents, kPmE, pfLParam, rfSqrDistance);
		}
		else
		{
			// line intersects z = e2
			face(2, 0, 1, rkPnt, rkDir, extents, kPmE, pfLParam, rfSqrDistance);
		}
	}
	else
	{
		fProdDzPy = rkDir.z*kPmE.y;
		fProdDyPz = rkDir.y*kPmE.z;
		if(fProdDzPy >= fProdDyPz)
		{
			// line intersects y = e1
			face(1, 2, 0, rkPnt, rkDir, extents, kPmE, pfLParam, rfSqrDistance);
		}
		else
		{
			// line intersects z = e2
			face(2, 0, 1, rkPnt, rkDir, extents, kPmE, pfLParam, rfSqrDistance);
		}
	}
}

static void case0(unsigned int i0, unsigned int i1, unsigned int i2, PxVec3& rkPnt, const PxVec3& rkDir, const PxVec3& extents, PxReal* pfLParam, PxReal& rfSqrDistance)
{
	PxReal fPmE0 = rkPnt[i0] - extents[i0];
	PxReal fPmE1 = rkPnt[i1] - extents[i1];
	PxReal fProd0 = rkDir[i1]*fPmE0;
	PxReal fProd1 = rkDir[i0]*fPmE1;
	PxReal fDelta, fInvLSqr, fInv;

	if(fProd0 >= fProd1)
	{
		// line intersects P[i0] = e[i0]
		rkPnt[i0] = extents[i0];

		PxReal fPpE1 = rkPnt[i1] + extents[i1];
		fDelta = fProd0 - rkDir[i0]*fPpE1;
		if(fDelta >= 0.0f)
		{
			fInvLSqr = 1.0f/(rkDir[i0]*rkDir[i0] + rkDir[i1]*rkDir[i1]);
			rfSqrDistance += fDelta*fDelta*fInvLSqr;
			if(pfLParam)
			{
				rkPnt[i1] = -extents[i1];
				*pfLParam = -(rkDir[i0]*fPmE0+rkDir[i1]*fPpE1)*fInvLSqr;
			}
		}
		else
		{
			if(pfLParam)
			{
				fInv = 1.0f/rkDir[i0];
				rkPnt[i1] -= fProd0*fInv;
				*pfLParam = -fPmE0*fInv;
			}
		}
	}
	else
	{
		// line intersects P[i1] = e[i1]
		rkPnt[i1] = extents[i1];

		PxReal fPpE0 = rkPnt[i0] + extents[i0];
		fDelta = fProd1 - rkDir[i1]*fPpE0;
		if(fDelta >= 0.0f)
		{
			fInvLSqr = 1.0f/(rkDir[i0]*rkDir[i0] + rkDir[i1]*rkDir[i1]);
			rfSqrDistance += fDelta*fDelta*fInvLSqr;
			if(pfLParam)
			{
				rkPnt[i0] = -extents[i0];
				*pfLParam = -(rkDir[i0]*fPpE0+rkDir[i1]*fPmE1)*fInvLSqr;
			}
		}
		else
		{
			if(pfLParam)
			{
				fInv = 1.0f/rkDir[i1];
				rkPnt[i0] -= fProd1*fInv;
				*pfLParam = -fPmE1*fInv;
			}
		}
	}

	if(rkPnt[i2] < -extents[i2])
	{
		fDelta = rkPnt[i2] + extents[i2];
		rfSqrDistance += fDelta*fDelta;
		rkPnt[i2] = -extents[i2];
	}
	else if ( rkPnt[i2] > extents[i2] )
	{
		fDelta = rkPnt[i2] - extents[i2];
		rfSqrDistance += fDelta*fDelta;
		rkPnt[i2] = extents[i2];
	}
}

static void case00(unsigned int i0, unsigned int i1, unsigned int i2, PxVec3& rkPnt, const PxVec3& rkDir, const PxVec3& extents, PxReal* pfLParam, PxReal& rfSqrDistance)
{
	PxReal fDelta;

	if(pfLParam)
		*pfLParam = (extents[i0] - rkPnt[i0])/rkDir[i0];

	rkPnt[i0] = extents[i0];

	if(rkPnt[i1] < -extents[i1])
	{
		fDelta = rkPnt[i1] + extents[i1];
		rfSqrDistance += fDelta*fDelta;
		rkPnt[i1] = -extents[i1];
	}
	else if(rkPnt[i1] > extents[i1])
	{
		fDelta = rkPnt[i1] - extents[i1];
		rfSqrDistance += fDelta*fDelta;
		rkPnt[i1] = extents[i1];
	}

	if(rkPnt[i2] < -extents[i2])
	{
		fDelta = rkPnt[i2] + extents[i2];
		rfSqrDistance += fDelta*fDelta;
		rkPnt[i2] = -extents[i2];
	}
	else if(rkPnt[i2] > extents[i2])
	{
		fDelta = rkPnt[i2] - extents[i2];
		rfSqrDistance += fDelta*fDelta;
		rkPnt[i2] = extents[i2];
	}
}

static void case000(PxVec3& rkPnt, const PxVec3& extents, PxReal& rfSqrDistance)
{
	PxReal fDelta;

	if(rkPnt.x < -extents.x)
	{
		fDelta = rkPnt.x + extents.x;
		rfSqrDistance += fDelta*fDelta;
		rkPnt.x = -extents.x;
	}
	else if(rkPnt.x > extents.x)
	{
		fDelta = rkPnt.x - extents.x;
		rfSqrDistance += fDelta*fDelta;
		rkPnt.x = extents.x;
	}

	if(rkPnt.y < -extents.y)
	{
		fDelta = rkPnt.y + extents.y;
		rfSqrDistance += fDelta*fDelta;
		rkPnt.y = -extents.y;
	}
	else if(rkPnt.y > extents.y)
	{
		fDelta = rkPnt.y - extents.y;
		rfSqrDistance += fDelta*fDelta;
		rkPnt.y = extents.y;
	}

	if(rkPnt.z < -extents.z)
	{
		fDelta = rkPnt.z + extents.z;
		rfSqrDistance += fDelta*fDelta;
		rkPnt.z = -extents.z;
	}
	else if(rkPnt.z > extents.z)
	{
		fDelta = rkPnt.z - extents.z;
		rfSqrDistance += fDelta*fDelta;
		rkPnt.z = extents.z;
	}
}

//! Compute the smallest distance from the (infinite) line to the box.
static PxReal distanceLineBoxSquared(const PxVec3& lineOrigin, const PxVec3& lineDirection,
								  const PxVec3& boxOrigin, const PxVec3& boxExtent, const PxMat33& boxBase,
								  PxReal* lineParam,
								  PxVec3* boxParam)
{
	const PxVec3& axis0 = boxBase.column0;
	const PxVec3& axis1 = boxBase.column1;
	const PxVec3& axis2 = boxBase.column2;
	
	// compute coordinates of line in box coordinate system
	const PxVec3 diff = lineOrigin - boxOrigin;
	PxVec3 pnt(diff.dot(axis0), diff.dot(axis1), diff.dot(axis2));
	PxVec3 dir(lineDirection.dot(axis0), lineDirection.dot(axis1), lineDirection.dot(axis2));

	// Apply reflections so that direction vector has nonnegative components.
	bool reflect[3];
	for(unsigned int i=0;i<3;i++)
	{
		if(dir[i]<0.0f)
		{
			pnt[i] = -pnt[i];
			dir[i] = -dir[i];
			reflect[i] = true;
		}
		else
		{
			reflect[i] = false;
		}
	}

	PxReal sqrDistance = 0.0f;

	if(dir.x>0.0f)
	{
		if(dir.y>0.0f)
		{
			if(dir.z>0.0f)	caseNoZeros(pnt, dir, boxExtent, lineParam, sqrDistance);		// (+,+,+)
			else			case0(0, 1, 2, pnt, dir, boxExtent, lineParam, sqrDistance);	// (+,+,0)
		}
		else
		{
			if(dir.z>0.0f)	case0(0, 2, 1, pnt, dir, boxExtent, lineParam, sqrDistance);	// (+,0,+)
			else			case00(0, 1, 2, pnt, dir, boxExtent, lineParam, sqrDistance);	// (+,0,0)
		}
	}
	else
	{
		if(dir.y>0.0f)
		{
			if(dir.z>0.0f)	case0(1, 2, 0, pnt, dir, boxExtent, lineParam, sqrDistance);	// (0,+,+)
			else			case00(1, 0, 2, pnt, dir, boxExtent, lineParam, sqrDistance);	// (0,+,0)
		}
		else
		{
			if(dir.z>0.0f)	case00(2, 0, 1, pnt, dir, boxExtent, lineParam, sqrDistance);	// (0,0,+)
			else
			{
				case000(pnt, boxExtent, sqrDistance);										// (0,0,0)
				if(lineParam)
					*lineParam = 0.0f;
			}
		}
	}

	if(boxParam)
	{
		// undo reflections
		for(unsigned int i=0;i<3;i++)
		{
			if(reflect[i])
				pnt[i] = -pnt[i];
		}

		*boxParam = pnt;
	}

	return sqrDistance;
}

//! Compute the smallest distance from the (finite) line segment to the box.
PxReal Gu::distanceSegmentBoxSquared(	const PxVec3& segmentPoint0, const PxVec3& segmentPoint1,
										const PxVec3& boxOrigin, const PxVec3& boxExtent, const PxMat33& boxBase,
										PxReal* segmentParam,
										PxVec3* boxParam)
{
	// compute coordinates of line in box coordinate system

	PxReal lp;
	PxVec3 bp;
	PxReal sqrDistance = distanceLineBoxSquared(segmentPoint0, segmentPoint1 - segmentPoint0, boxOrigin, boxExtent, boxBase, &lp, &bp);
	if(lp>=0.0f)
	{
		if(lp<=1.0f)
		{
			if(segmentParam)
				*segmentParam = lp;
			if(boxParam)
				*boxParam = bp;
			return sqrDistance;
		}
		else
		{
			if(segmentParam)
				*segmentParam = 1.0f;
			return Gu::distancePointBoxSquared(segmentPoint1, boxOrigin, boxExtent, boxBase, boxParam);
		}
	}
	else
	{
		if(segmentParam)
			*segmentParam = 0.0f;
		return Gu::distancePointBoxSquared(segmentPoint0, boxOrigin, boxExtent, boxBase, boxParam);
	}
}
