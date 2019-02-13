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



#ifndef DY_SOLVERCONTACTPF_H
#define DY_SOLVERCONTACTPF_H

#include "foundation/PxSimpleTypes.h"
#include "foundation/PxVec3.h"
#include "PxvConfig.h"
#include "PsVecMath.h"

namespace physx
{

using namespace Ps::aos;

namespace Dy
{

struct SolverContactCoulombHeader
{
	PxU8	type;					//Note: mType should be first as the solver expects a type in the first byte.
	PxU8	numNormalConstr;
	PxU16	frictionOffset;					//4
	//PxF32	restitution;
	PxF32	angDom0;						//8
	PxF32	dominance0;						//12
	PxF32	dominance1;						//16
	PX_ALIGN(16, PxVec3	normalXYZ);			//28
	PxF32	angDom1;						//32
	
	Sc::ShapeInteraction* shapeInteraction;		//36	40
	PxU8	flags;								//37	41
	PxU8	pad0[3];							//40	44
#if !PX_P64_FAMILY
	PxU32	pad1[2];							//48	
#else
	PxU32 pad1;									//		48
#endif
	
	
	
	PX_FORCE_INLINE void setDominance0(const FloatV f)		{FStore(f, &dominance0);}
	PX_FORCE_INLINE void setDominance1(const FloatV f)		{FStore(f, &dominance1);}
	PX_FORCE_INLINE void setNormal(const Vec3V n)			{V3StoreA(n, normalXYZ);}
	
	PX_FORCE_INLINE FloatV getDominance0() const			{return FLoad(dominance0);}
	PX_FORCE_INLINE FloatV getDominance1() const			{return FLoad(dominance1);}
	//PX_FORCE_INLINE FloatV getRestitution() const			{return FLoad(restitution);}
	PX_FORCE_INLINE	Vec3V getNormal()const					{return V3LoadA(normalXYZ);}

  
	PX_FORCE_INLINE void setDominance0(PxF32 f)				{ dominance0 = f; }
	PX_FORCE_INLINE void setDominance1(PxF32 f)				{ dominance1 = f;}
	//PX_FORCE_INLINE void setRestitution(PxF32 f)			{ restitution = f;}

	PX_FORCE_INLINE PxF32 getDominance0PxF32() const		{return dominance0;}
	PX_FORCE_INLINE PxF32 getDominance1PxF32() const		{return dominance1;}
	//PX_FORCE_INLINE PxF32 getRestitutionPxF32() const		{return restitution;}

}; 
PX_COMPILE_TIME_ASSERT(sizeof(SolverContactCoulombHeader) == 48);

struct SolverFrictionHeader
{
	PxU8	type;					//Note: mType should be first as the solver expects a type in the first byte.
	PxU8	numNormalConstr;
	PxU8	numFrictionConstr;
	PxU8	flags;
	PxF32   staticFriction;
	PxF32   invMass0D0;
	PxF32	invMass1D1;
	PxF32	angDom0;
	PxF32	angDom1;
	PxU32	pad2[2];

	PX_FORCE_INLINE void setStaticFriction(const FloatV f)	{FStore(f, &staticFriction);}
	
	PX_FORCE_INLINE FloatV getStaticFriction() const		{return FLoad(staticFriction);}
	
	PX_FORCE_INLINE void setStaticFriction(PxF32 f)			{staticFriction = f;}

	PX_FORCE_INLINE PxF32 getStaticFrictionPxF32() const	{return staticFriction;}	

	PX_FORCE_INLINE PxU32 getAppliedForcePaddingSize() const {return sizeof(PxU32)*((4 * ((numNormalConstr + 3)/4)));}
	static PX_FORCE_INLINE PxU32 getAppliedForcePaddingSize(const PxU32 numConstr) {return sizeof(PxU32)*((4 * ((numConstr + 3)/4)));}
}; 

PX_COMPILE_TIME_ASSERT(sizeof(SolverFrictionHeader) == 32);

}

}

#endif //DY_SOLVERCONTACTPF_H
