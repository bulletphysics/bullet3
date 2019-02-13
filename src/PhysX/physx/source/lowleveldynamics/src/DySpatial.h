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


#ifndef DY_SPATIAL_H
#define DY_SPATIAL_H

#include "foundation/PxVec3.h"
#include "foundation/PxTransform.h"
#include "PsMathUtils.h"
#include "CmSpatialVector.h"

namespace physx
{
namespace Dy
{
// translate a motion resolved at position p to the origin


// should have a 'from' frame and a 'to' frame
class SpInertia
{
public:
	SpInertia() {}

	SpInertia(const PxMat33& ll, const PxMat33& la, const PxMat33& aa): mLL(ll), mLA(la), mAA(aa)
	{
	}

	static SpInertia getZero()
	{
		return SpInertia(PxMat33(PxZero), PxMat33(PxZero), 
							     PxMat33(PxZero));
	}

	static SpInertia dyad(const Cm::SpatialVector& column, const Cm::SpatialVector& row) 
	{
		return SpInertia(dyad(column.linear, row.linear),  
						 dyad(column.linear, row.angular),  
					     dyad(column.angular, row.angular));
	}


	static SpInertia inertia(PxReal mass, const PxVec3& inertia)
	{
		return SpInertia(PxMat33::createDiagonal(PxVec3(mass,mass,mass)), PxMat33(PxZero),			 
							     PxMat33::createDiagonal(inertia));
	}


	SpInertia operator+(const SpInertia& m) const
	{
		return SpInertia(mLL+m.mLL, mLA+m.mLA, mAA+m.mAA);
	}

	SpInertia operator-(const SpInertia& m) const
	{
		return SpInertia(mLL-m.mLL, mLA-m.mLA, mAA-m.mAA);
	}

	SpInertia operator*(PxReal r) const
	{
		return SpInertia(mLL*r, mLA*r, mAA*r);
	}

	void operator+=(const SpInertia& m)
	{
		mLL+=m.mLL; 
		mLA+=m.mLA;		
		mAA+=m.mAA;
	}

	void operator-=(const SpInertia& m)
	{
		mLL-=m.mLL; 
		mLA-=m.mLA;		
		mAA-=m.mAA;
	}


	PX_FORCE_INLINE Cm::SpatialVector operator *(const Cm::SpatialVector& v) const
	{
		return Cm::SpatialVector(mLL*v.linear            +mLA*v.angular,
		 					    mLA.transformTranspose(v.linear)+mAA*v.angular);
	}

	SpInertia operator *(const SpInertia& v) const
	{
		return SpInertia(mLL*v.mLL             + mLA * v.mLA.getTranspose(), 
						 mLL*v.mLA             + mLA * v.mAA,
						 mLA.getTranspose()*v.mLA + mAA * v.mAA);
	}


	bool isFinite() const
	{
		return true;
//		return mLL.isFinite() && mLA.isFinite() && mAA.isFinite(); 
	}

	PxMat33 mLL, mLA;		// linear force from angular motion, linear force from linear motion
	PxMat33 mAA;		    // angular force from angular motion, mAL = mLA.transpose()

private:
	static PxMat33 dyad(PxVec3 col, PxVec3 row)	
	{ 
		return PxMat33(col*row.x, col*row.y, col*row.z); 
	}


};

}
}

#endif //DY_SPATIAL_H
