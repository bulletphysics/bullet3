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



#ifndef DY_ARTICULATION_REFERENCE_H
#define DY_ARTICULATION_REFERENCE_H

// a per-row struct where we put extra data for debug and setup - ultimately this will move to be just
// debug only



#include "DyArticulationUtils.h"
#include "DyArticulationScalar.h"
#include "DyArticulationFnsScalar.h"
#include "DySpatial.h"

#if DY_ARTICULATION_DEBUG_VERIFY

namespace physx
{

PX_FORCE_INLINE Cm::SpatialVector propagateVelocity(const FsRow& row, 
													const FsJointVectors& jv,
													const PxVec3& SZ, 
													const Cm::SpatialVector& v,
													const FsRowAux& aux)
{
	typedef ArticulationFnsScalar Fns;

	Cm::SpatialVector w = Fns::translateMotion(-getParentOffset(jv), v);
	PxVec3 DSZ = Fns::multiply(row.D, SZ);

	PxVec3 n = Fns::axisDot(getDSI(row), w) + DSZ;
	Cm::SpatialVector result = w - Cm::SpatialVector(getJointOffset(jv).cross(n),n);
#if DY_ARTICULATION_DEBUG_VERIFY
	Cm::SpatialVector check = ArticulationRef::propagateVelocity(row, jv, SZ, v, aux);
	PX_ASSERT((result-check).magnitude()<1e-5*PxMax(check.magnitude(), 1.0f));
#endif
	return result;
}

PX_FORCE_INLINE Cm::SpatialVector propagateImpulse(const FsRow& row, 
												   const FsJointVectors& jv,
												   PxVec3& SZ, 
												   const Cm::SpatialVector& Z,
												   const FsRowAux& aux)
{
	typedef ArticulationFnsScalar Fns;

	SZ = Z.angular + Z.linear.cross(getJointOffset(jv));
	Cm::SpatialVector result = Fns::translateForce(getParentOffset(jv), Z - Fns::axisMultiply(getDSI(row), SZ));
#if DY_ARTICULATION_DEBUG_VERIFY
	PxVec3 SZcheck;
	Cm::SpatialVector check = ArticulationRef::propagateImpulse(row, jv, SZcheck, Z, aux);
	PX_ASSERT((result-check).magnitude()<1e-5*PxMax(check.magnitude(), 1.0f));
	PX_ASSERT((SZ-SZcheck).magnitude()<1e-5*PxMax(SZcheck.magnitude(), 1.0f));
#endif
	return result;
}

}
#endif

#endif //DY_ARTICULATION_REFERENCE_H
