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
#include "../../../include/physics_effects/base_level/solver/pfx_joint_ball.h"
#include "pfx_constraint_row_solver.h"

namespace sce {
namespace PhysicsEffects {

PfxInt32 pfxInitializeBallJoint(PfxJoint &joint,
	const PfxRigidState &stateA,const PfxRigidState &stateB,
	const PfxBallJointInitParam &param)
{
	joint.m_active = 1;
	joint.m_numConstraints = 3;
	joint.m_userData = NULL;
	joint.m_type = kPfxJointBall;
	joint.m_rigidBodyIdA = stateA.getRigidBodyId();
	joint.m_rigidBodyIdB = stateB.getRigidBodyId();
	
	for(int i=0;i<3;i++) {
		joint.m_constraints[i].reset();
	}
	
	joint.m_constraints[0].m_lock = SCE_PFX_JOINT_LOCK_FIX;
	joint.m_constraints[1].m_lock = SCE_PFX_JOINT_LOCK_FIX;
	joint.m_constraints[2].m_lock = SCE_PFX_JOINT_LOCK_FIX;
	
	PfxMatrix3 rotA = transpose(PfxMatrix3(stateA.getOrientation()));
	PfxMatrix3 rotB = transpose(PfxMatrix3(stateB.getOrientation()));
	
	joint.m_anchorA = rotA * (param.anchorPoint - stateA.getPosition());
	joint.m_anchorB = rotB * (param.anchorPoint - stateB.getPosition());
	
	joint.m_frameA = PfxMatrix3::identity();
	joint.m_frameB = rotB * PfxMatrix3(stateA.getOrientation()) * joint.m_frameA;
	
	return SCE_PFX_OK;
}

void pfxSetupBallJoint(
	PfxJoint &joint,
	const PfxRigidState &stateA,
	const PfxRigidState &stateB,
	PfxSolverBody &solverBodyA,
	PfxSolverBody &solverBodyB,
	PfxFloat timeStep
	)
{
	PfxVector3 rA = rotate(solverBodyA.m_orientation,joint.m_anchorA);
	PfxVector3 rB = rotate(solverBodyB.m_orientation,joint.m_anchorB);

	PfxVector3 vA = stateA.getLinearVelocity() + cross(stateA.getAngularVelocity(),rA);
	PfxVector3 vB = stateB.getLinearVelocity() + cross(stateB.getAngularVelocity(),rB);
	PfxVector3 vAB = vA-vB;

	PfxVector3 distance = (stateA.getPosition() + rA) - (stateB.getPosition() + rB);

	PfxMatrix3 worldFrameA,worldFrameB;
	worldFrameA = PfxMatrix3(solverBodyA.m_orientation) * joint.m_frameA;
	worldFrameB = PfxMatrix3(solverBodyB.m_orientation) * joint.m_frameB;
	
	// Linear Constraint
	PfxMatrix3 K = PfxMatrix3::scale(PfxVector3(solverBodyA.m_massInv + solverBodyB.m_massInv)) - 
			crossMatrix(rA) * solverBodyA.m_inertiaInv * crossMatrix(rA) - 
			crossMatrix(rB) * solverBodyB.m_inertiaInv * crossMatrix(rB);
	
	for(int c=0;c<3;c++) {
		PfxJointConstraint &jointConstraint = joint.m_constraints[c];
		PfxConstraintRow &constraint = jointConstraint.m_constraintRow;

		PfxVector3 normal = worldFrameA[c];
		
		PfxFloat posErr = dot(distance,-normal);
		PfxFloat lowerLimit = -jointConstraint.m_maxImpulse;
		PfxFloat upperLimit =  jointConstraint.m_maxImpulse;
		PfxFloat velocityAmp = 1.0f;
		
		pfxCalcLinearLimit(jointConstraint,posErr,velocityAmp,lowerLimit,upperLimit);

		PfxFloat denom = dot(K*normal,normal);
		
		constraint.m_rhs = -velocityAmp*dot(vAB,normal);
		constraint.m_rhs -= (jointConstraint.m_bias * (-posErr)) / timeStep;
		constraint.m_rhs *= jointConstraint.m_weight/denom;
		constraint.m_jacDiagInv = jointConstraint.m_weight*velocityAmp/denom;
		constraint.m_lowerLimit = lowerLimit;
		constraint.m_upperLimit = upperLimit;
		pfxStoreVector3(normal,constraint.m_normal);
	}
}

void pfxWarmStartBallJoint(
	PfxJoint &joint,
	PfxSolverBody &solverBodyA,
	PfxSolverBody &solverBodyB
	)
{
	PfxVector3 rA = rotate(solverBodyA.m_orientation,joint.m_anchorA);
	PfxVector3 rB = rotate(solverBodyB.m_orientation,joint.m_anchorB);

	// Linear Constraint
	for(int c=0;c<3;c++) {
		PfxJointConstraint &jointConstraint = joint.m_constraints[c];
		PfxConstraintRow &constraint = jointConstraint.m_constraintRow;

		if(jointConstraint.m_warmStarting == 0) {
			constraint.m_accumImpulse = 0.0f;
		}
		else {
			PfxVector3 normal = pfxReadVector3(constraint.m_normal);
			PfxFloat deltaImpulse = constraint.m_accumImpulse;
			solverBodyA.m_deltaLinearVelocity += deltaImpulse * solverBodyA.m_massInv * normal;
			solverBodyA.m_deltaAngularVelocity += deltaImpulse * solverBodyA.m_inertiaInv * cross(rA,normal);
			solverBodyB.m_deltaLinearVelocity -= deltaImpulse * solverBodyB.m_massInv * normal;
			solverBodyB.m_deltaAngularVelocity -= deltaImpulse * solverBodyB.m_inertiaInv * cross(rB,normal);
		}
	}
}

void pfxSolveBallJoint(
	PfxJoint &joint,
	PfxSolverBody &solverBodyA,
	PfxSolverBody &solverBodyB
	)
{
	PfxVector3 rA = rotate(solverBodyA.m_orientation,joint.m_anchorA);
	PfxVector3 rB = rotate(solverBodyB.m_orientation,joint.m_anchorB);

	// Linear Constraint
// ARA begin insert new code
#ifdef __ARM_NEON__
	pfxSolveLinearConstraintRowNEON(joint.m_constraints[0].m_constraintRow,
		solverBodyA.m_deltaLinearVelocity,solverBodyA.m_deltaAngularVelocity,solverBodyA.m_massInv,solverBodyA.m_inertiaInv,rA,
		solverBodyB.m_deltaLinearVelocity,solverBodyB.m_deltaAngularVelocity,solverBodyB.m_massInv,solverBodyB.m_inertiaInv,rB);

	pfxSolveLinearConstraintRowNEON(joint.m_constraints[1].m_constraintRow,
		solverBodyA.m_deltaLinearVelocity,solverBodyA.m_deltaAngularVelocity,solverBodyA.m_massInv,solverBodyA.m_inertiaInv,rA,
		solverBodyB.m_deltaLinearVelocity,solverBodyB.m_deltaAngularVelocity,solverBodyB.m_massInv,solverBodyB.m_inertiaInv,rB);

	pfxSolveLinearConstraintRowNEON(joint.m_constraints[2].m_constraintRow,
		solverBodyA.m_deltaLinearVelocity,solverBodyA.m_deltaAngularVelocity,solverBodyA.m_massInv,solverBodyA.m_inertiaInv,rA,
		solverBodyB.m_deltaLinearVelocity,solverBodyB.m_deltaAngularVelocity,solverBodyB.m_massInv,solverBodyB.m_inertiaInv,rB);
#else // __ARM_NEON__
// ARA end

	pfxSolveLinearConstraintRow(joint.m_constraints[0].m_constraintRow,
		solverBodyA.m_deltaLinearVelocity,solverBodyA.m_deltaAngularVelocity,solverBodyA.m_massInv,solverBodyA.m_inertiaInv,rA,
		solverBodyB.m_deltaLinearVelocity,solverBodyB.m_deltaAngularVelocity,solverBodyB.m_massInv,solverBodyB.m_inertiaInv,rB);

	pfxSolveLinearConstraintRow(joint.m_constraints[1].m_constraintRow,
		solverBodyA.m_deltaLinearVelocity,solverBodyA.m_deltaAngularVelocity,solverBodyA.m_massInv,solverBodyA.m_inertiaInv,rA,
		solverBodyB.m_deltaLinearVelocity,solverBodyB.m_deltaAngularVelocity,solverBodyB.m_massInv,solverBodyB.m_inertiaInv,rB);

	pfxSolveLinearConstraintRow(joint.m_constraints[2].m_constraintRow,
		solverBodyA.m_deltaLinearVelocity,solverBodyA.m_deltaAngularVelocity,solverBodyA.m_massInv,solverBodyA.m_inertiaInv,rA,
		solverBodyB.m_deltaLinearVelocity,solverBodyB.m_deltaAngularVelocity,solverBodyB.m_massInv,solverBodyB.m_inertiaInv,rB);

// ARA begin insert new code
#endif // __ARM_NEON__
// ARA end
}

} //namespace PhysicsEffects
} //namespace sce
