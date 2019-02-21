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



#ifndef DY_ARTICULATION_SCALAR_H
#define DY_ARTICULATION_SCALAR_H

// Scalar helpers for articulations

#include "foundation/PxUnionCast.h"
#include "DyArticulationUtils.h"
#include "DySpatial.h"
#include "PsFPU.h"

namespace physx
{

namespace Dy
{

PX_FORCE_INLINE Cm::SpatialVector&	velocityRef(FsData &m, PxU32 i)
{	
	return reinterpret_cast<Cm::SpatialVector&>(getVelocity(m)[i]); 
}

PX_FORCE_INLINE Cm::SpatialVector&	deferredVelRef(FsData &m, PxU32 i)
{	
	return reinterpret_cast<Cm::SpatialVector&>(getDeferredVel(m)[i]); 
}

PX_FORCE_INLINE PxVec3& deferredSZRef(FsData &m, PxU32 i)
{	
	return reinterpret_cast<PxVec3 &>(getDeferredSZ(m)[i]); 
}

PX_FORCE_INLINE const PxVec3& deferredSZ(const FsData &s, PxU32 i) 
{	
	return reinterpret_cast<const PxVec3 &>(getDeferredSZ(s)[i]); 
}

PX_FORCE_INLINE Cm::SpatialVector& deferredZRef(FsData &s)
{
	return unsimdRef(s.deferredZ);
}


PX_FORCE_INLINE const Cm::SpatialVector& deferredZ(const FsData &s)
{
	return unsimdRef(s.deferredZ);
}

PX_FORCE_INLINE const PxVec3& getJointOffset(const FsJointVectors& j) 
{	
	return reinterpret_cast<const PxVec3& >(j.jointOffset);		
}

PX_FORCE_INLINE const PxVec3& getParentOffset(const FsJointVectors& j) 
{	
	return reinterpret_cast<const PxVec3&>(j.parentOffset);		
}




PX_FORCE_INLINE const Cm::SpatialVector* getDSI(const FsRow& row)
{	
	return PxUnionCast<const Cm::SpatialVector*,const Cm::SpatialVectorV*>(row.DSI); //reinterpret_cast<const Cm::SpatialVector*>(row.DSI); 
}

}

}

#endif //DY_ARTICULATION_SCALAR_H
