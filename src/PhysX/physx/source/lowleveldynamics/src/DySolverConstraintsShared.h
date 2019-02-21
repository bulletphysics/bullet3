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

#ifndef DY_SOLVER_CORE_SHARED_H
#define DY_SOLVER_CORE_SHARED_H

#include "foundation/PxPreprocessor.h"
#include "PsVecMath.h"

#include "CmPhysXCommon.h"
#include "DySolverBody.h"
#include "DySolverContact.h"
#include "DySolverConstraint1D.h"
#include "DySolverConstraintDesc.h"
#include "PsUtilities.h"
#include "DyConstraint.h"
#include "PsAtomic.h"


namespace physx
{

namespace Dy
{
	PX_FORCE_INLINE static FloatV solveDynamicContacts(SolverContactPoint* contacts, const PxU32 nbContactPoints, const Vec3VArg contactNormal,
	const FloatVArg invMassA, const FloatVArg invMassB, const FloatVArg angDom0, const FloatVArg angDom1, Vec3V& linVel0_, Vec3V& angState0_, 
	Vec3V& linVel1_, Vec3V& angState1_, PxF32* PX_RESTRICT forceBuffer)
{
	Vec3V linVel0 = linVel0_;
	Vec3V angState0 = angState0_;
	Vec3V linVel1 = linVel1_;
	Vec3V angState1 = angState1_;
	FloatV accumulatedNormalImpulse = FZero();

	const Vec3V delLinVel0 = V3Scale(contactNormal, invMassA);
	const Vec3V delLinVel1 = V3Scale(contactNormal, invMassB);

	for(PxU32 i=0;i<nbContactPoints;i++)
	{
		SolverContactPoint& c = contacts[i];
		Ps::prefetchLine(&contacts[i], 128);

		const Vec3V raXn = c.raXn;

		const Vec3V rbXn = c.rbXn;

		const FloatV appliedForce = FLoad(forceBuffer[i]);
		const FloatV velMultiplier = c.getVelMultiplier();
		
		/*const FloatV targetVel = c.getTargetVelocity();
		const FloatV nScaledBias = c.getScaledBias();*/
		const FloatV maxImpulse = c.getMaxImpulse();

		//Compute the normal velocity of the constraint.
		const Vec3V v0 = V3MulAdd(linVel0, contactNormal, V3Mul(angState0, raXn));
		const Vec3V v1 = V3MulAdd(linVel1, contactNormal, V3Mul(angState1, rbXn));
		const FloatV normalVel = V3SumElems(V3Sub(v0, v1));

		const FloatV biasedErr = c.getBiasedErr();//FScaleAdd(targetVel, velMultiplier, nScaledBias);

		//KS - clamp the maximum force
		const FloatV _deltaF = FMax(FNegScaleSub(normalVel, velMultiplier, biasedErr), FNeg(appliedForce));
		const FloatV _newForce = FAdd(appliedForce, _deltaF);
		const FloatV newForce = FMin(_newForce, maxImpulse);
		const FloatV deltaF = FSub(newForce, appliedForce);

		linVel0 = V3ScaleAdd(delLinVel0, deltaF, linVel0);
		linVel1 = V3NegScaleSub(delLinVel1, deltaF, linVel1);
		angState0 = V3ScaleAdd(raXn, FMul(deltaF, angDom0), angState0);
		angState1 = V3NegScaleSub(rbXn, FMul(deltaF, angDom1), angState1);
		
		FStore(newForce, &forceBuffer[i]);

		accumulatedNormalImpulse = FAdd(accumulatedNormalImpulse, newForce);
	}

	linVel0_ = linVel0;
	angState0_ = angState0;
	linVel1_ = linVel1;
	angState1_ = angState1;
	return accumulatedNormalImpulse;
}

PX_FORCE_INLINE static FloatV solveStaticContacts(SolverContactPoint* contacts, const PxU32 nbContactPoints, const Vec3VArg contactNormal,
	const FloatVArg invMassA, const FloatVArg angDom0, Vec3V& linVel0_, Vec3V& angState0_, PxF32* PX_RESTRICT forceBuffer)
{
	Vec3V linVel0 = linVel0_;
	Vec3V angState0 = angState0_;
	FloatV accumulatedNormalImpulse = FZero();

	const Vec3V delLinVel0 = V3Scale(contactNormal, invMassA);

	for(PxU32 i=0;i<nbContactPoints;i++)
	{
		SolverContactPoint& c = contacts[i];
		Ps::prefetchLine(&contacts[i],128);

		const Vec3V raXn = c.raXn;
		
		const FloatV appliedForce = FLoad(forceBuffer[i]);
		const FloatV velMultiplier = c.getVelMultiplier();

		/*const FloatV targetVel = c.getTargetVelocity();
		const FloatV nScaledBias = c.getScaledBias();*/
		const FloatV maxImpulse = c.getMaxImpulse();
		
		const Vec3V v0 = V3MulAdd(linVel0, contactNormal, V3Mul(angState0, raXn));
		const FloatV normalVel = V3SumElems(v0);


		const FloatV biasedErr = c.getBiasedErr();//FScaleAdd(targetVel, velMultiplier, nScaledBias);

		// still lots to do here: using loop pipelining we can interweave this code with the
		// above - the code here has a lot of stalls that we would thereby eliminate
		const FloatV _deltaF = FMax(FNegScaleSub(normalVel, velMultiplier, biasedErr), FNeg(appliedForce));
		const FloatV _newForce = FAdd(appliedForce, _deltaF);
		const FloatV newForce = FMin(_newForce, maxImpulse);
		const FloatV deltaF = FSub(newForce, appliedForce);

		linVel0 = V3ScaleAdd(delLinVel0, deltaF, linVel0);
		angState0 = V3ScaleAdd(raXn, FMul(deltaF, angDom0), angState0);

		FStore(newForce, &forceBuffer[i]);

		accumulatedNormalImpulse = FAdd(accumulatedNormalImpulse, newForce);
	}

	linVel0_ = linVel0;
	angState0_ = angState0;
	return accumulatedNormalImpulse;
}

FloatV solveExtContacts(SolverContactPointExt* contacts, const PxU32 nbContactPoints, const Vec3VArg contactNormal,
	Vec3V& linVel0, Vec3V& angVel0,
	Vec3V& linVel1, Vec3V& angVel1,
	Vec3V& li0, Vec3V& ai0,
	Vec3V& li1, Vec3V& ai1,
	PxF32* PX_RESTRICT appliedForceBuffer);

}

}

#endif //DY_SOLVER_CORE_SHARED_H

