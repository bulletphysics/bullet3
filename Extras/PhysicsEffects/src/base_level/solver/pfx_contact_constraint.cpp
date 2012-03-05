/*
Physics Effects Copyright(C) 2010 Sony Computer Entertainment Inc.
All rights reserved.

Physics Effects is open software; you can redistribute it and/or
modify it under the terms of the BSD License.

Physics Effects is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the BSD License for more details.

A copy of the BSD License is distributed with
Physics Effects under the filename: physics_effects_license.txt
*/

#include "../../../include/physics_effects/base_level/base/pfx_vec_utils.h"
#include "../../../include/physics_effects/base_level/solver/pfx_contact_constraint.h"
#include "pfx_constraint_row_solver.h"

namespace sce {
namespace PhysicsEffects {

#define SCE_PFX_CONTACT_SLOP 0.001f

void pfxSetupContactConstraint(
	PfxConstraintRow &constraintResponse,
	PfxConstraintRow &constraintFriction1,
	PfxConstraintRow &constraintFriction2,
	PfxFloat penetrationDepth,
	PfxFloat restitution,
	PfxFloat friction,
	const PfxVector3 &contactNormal,
	const PfxVector3 &contactPointA,
	const PfxVector3 &contactPointB,
	const PfxRigidState &stateA,
	const PfxRigidState &stateB,
	PfxSolverBody &solverBodyA,
	PfxSolverBody &solverBodyB,
	PfxFloat separateBias,
	PfxFloat timeStep
	)
{
	(void)friction;

	PfxVector3 rA = rotate(solverBodyA.m_orientation,contactPointA);
	PfxVector3 rB = rotate(solverBodyB.m_orientation,contactPointB);

	PfxFloat massInvA = solverBodyA.m_massInv;
	PfxFloat massInvB = solverBodyB.m_massInv;
	PfxMatrix3 inertiaInvA = solverBodyA.m_inertiaInv;
	PfxMatrix3 inertiaInvB = solverBodyB.m_inertiaInv;

	if(solverBodyA.m_motionType == kPfxMotionTypeOneWay) {
		massInvB = 0.0f;
		inertiaInvB = PfxMatrix3(0.0f);
	}
	if(solverBodyB.m_motionType == kPfxMotionTypeOneWay) {
		massInvA = 0.0f;
		inertiaInvA = PfxMatrix3(0.0f);
	}

	PfxMatrix3 K = PfxMatrix3::scale(PfxVector3(massInvA + massInvB)) - 
			crossMatrix(rA) * inertiaInvA * crossMatrix(rA) - 
			crossMatrix(rB) * inertiaInvB * crossMatrix(rB);

	PfxVector3 vA = stateA.getLinearVelocity() + cross(stateA.getAngularVelocity(),rA);
	PfxVector3 vB = stateB.getLinearVelocity() + cross(stateB.getAngularVelocity(),rB);
	PfxVector3 vAB = vA-vB;

	PfxVector3 tangent1,tangent2;
	pfxGetPlaneSpace(contactNormal,tangent1,tangent2);

	// Contact Constraint
	{
		PfxVector3 normal = contactNormal;

		PfxFloat denom = dot(K*normal,normal);

		constraintResponse.m_rhs = -(1.0f+restitution)*dot(vAB,normal); // velocity error
		constraintResponse.m_rhs -= (separateBias * SCE_PFX_MIN(0.0f,penetrationDepth+SCE_PFX_CONTACT_SLOP)) / timeStep; // position error
		constraintResponse.m_rhs /= denom;
		constraintResponse.m_jacDiagInv = 1.0f/denom;
		constraintResponse.m_lowerLimit = 0.0f;
		constraintResponse.m_upperLimit = SCE_PFX_FLT_MAX;
		pfxStoreVector3(normal,constraintResponse.m_normal);
	}

	// Friction Constraint 1
	{
		PfxVector3 normal = tangent1;

		PfxFloat denom = dot(K*normal,normal);

		constraintFriction1.m_jacDiagInv = 1.0f/denom;
		constraintFriction1.m_rhs = -dot(vAB,normal);
		constraintFriction1.m_rhs *= constraintFriction1.m_jacDiagInv;
		constraintFriction1.m_lowerLimit = 0.0f;
		constraintFriction1.m_upperLimit = SCE_PFX_FLT_MAX;
		pfxStoreVector3(normal,constraintFriction1.m_normal);
	}
	
	// Friction Constraint 2
	{
		PfxVector3 normal = tangent2;

		PfxFloat denom = dot(K*normal,normal);

		constraintFriction2.m_jacDiagInv = 1.0f/denom;
		constraintFriction2.m_rhs = -dot(vAB,normal);
		constraintFriction2.m_rhs *= constraintFriction2.m_jacDiagInv;
		constraintFriction2.m_lowerLimit = 0.0f;
		constraintFriction2.m_upperLimit = SCE_PFX_FLT_MAX;
		pfxStoreVector3(normal,constraintFriction2.m_normal);
	}
}

void pfxSolveContactConstraint(
	PfxConstraintRow &constraintResponse,
	PfxConstraintRow &constraintFriction1,
	PfxConstraintRow &constraintFriction2,
	const PfxVector3 &contactPointA,
	const PfxVector3 &contactPointB,
	PfxSolverBody &solverBodyA,
	PfxSolverBody &solverBodyB,
	PfxFloat friction
	)
{
	PfxVector3 rA = rotate(solverBodyA.m_orientation,contactPointA);
	PfxVector3 rB = rotate(solverBodyB.m_orientation,contactPointB);

	PfxFloat massInvA = solverBodyA.m_massInv;
	PfxFloat massInvB = solverBodyB.m_massInv;
	PfxMatrix3 inertiaInvA = solverBodyA.m_inertiaInv;
	PfxMatrix3 inertiaInvB = solverBodyB.m_inertiaInv;

	if(solverBodyA.m_motionType == kPfxMotionTypeOneWay) {
		massInvB = 0.0f;
		inertiaInvB = PfxMatrix3(0.0f);
	}
	if(solverBodyB.m_motionType == kPfxMotionTypeOneWay) {
		massInvA = 0.0f;
		inertiaInvA = PfxMatrix3(0.0f);
	}

// ARA begin insert new code
#ifdef __ARM_NEON__
	pfxSolveLinearConstraintRowNEON(constraintResponse,
		solverBodyA.m_deltaLinearVelocity,solverBodyA.m_deltaAngularVelocity,massInvA,inertiaInvA,rA,
		solverBodyB.m_deltaLinearVelocity,solverBodyB.m_deltaAngularVelocity,massInvB,inertiaInvB,rB);
#else // __ARM_NEON__
// ARA end

	pfxSolveLinearConstraintRow(constraintResponse,
		solverBodyA.m_deltaLinearVelocity,solverBodyA.m_deltaAngularVelocity,massInvA,inertiaInvA,rA,
		solverBodyB.m_deltaLinearVelocity,solverBodyB.m_deltaAngularVelocity,massInvB,inertiaInvB,rB);

// ARA begin insert new code
#endif // __ARM_NEON__
// ARA end

	PfxFloat mf = friction*fabsf(constraintResponse.m_accumImpulse);
	constraintFriction1.m_lowerLimit = -mf;
	constraintFriction1.m_upperLimit =  mf;
	constraintFriction2.m_lowerLimit = -mf;
	constraintFriction2.m_upperLimit =  mf;

// ARA begin insert new code
#ifdef __ARM_NEON__
	pfxSolveLinearConstraintRowNEON(constraintFriction1,
		solverBodyA.m_deltaLinearVelocity,solverBodyA.m_deltaAngularVelocity,massInvA,inertiaInvA,rA,
		solverBodyB.m_deltaLinearVelocity,solverBodyB.m_deltaAngularVelocity,massInvB,inertiaInvB,rB);

	pfxSolveLinearConstraintRowNEON(constraintFriction2,
		solverBodyA.m_deltaLinearVelocity,solverBodyA.m_deltaAngularVelocity,massInvA,inertiaInvA,rA,
		solverBodyB.m_deltaLinearVelocity,solverBodyB.m_deltaAngularVelocity,massInvB,inertiaInvB,rB);
#else // __ARM_NEON__
// ARA end

	pfxSolveLinearConstraintRow(constraintFriction1,
		solverBodyA.m_deltaLinearVelocity,solverBodyA.m_deltaAngularVelocity,massInvA,inertiaInvA,rA,
		solverBodyB.m_deltaLinearVelocity,solverBodyB.m_deltaAngularVelocity,massInvB,inertiaInvB,rB);

	pfxSolveLinearConstraintRow(constraintFriction2,
		solverBodyA.m_deltaLinearVelocity,solverBodyA.m_deltaAngularVelocity,massInvA,inertiaInvA,rA,
		solverBodyB.m_deltaLinearVelocity,solverBodyB.m_deltaAngularVelocity,massInvB,inertiaInvB,rB);

// ARA begin insert new code
#endif // __ARM_NEON__
// ARA end
}

} //namespace PhysicsEffects
} //namespace sce
