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
#include "../../../include/physics_effects/base_level/solver/pfx_joint_swing_twist.h"
#include "pfx_constraint_row_solver.h"


namespace sce {
namespace PhysicsEffects {

PfxInt32 pfxInitializeSwingTwistJoint(PfxJoint &joint,
	const PfxRigidState &stateA,const PfxRigidState &stateB,
	const PfxSwingTwistJointInitParam &param)
{
	joint.m_active = 1;
	joint.m_numConstraints = 6;
	joint.m_userData = NULL;
	joint.m_type = kPfxJointSwingtwist;
	joint.m_rigidBodyIdA = stateA.getRigidBodyId();
	joint.m_rigidBodyIdB = stateB.getRigidBodyId();
	
	for(int i=0;i<6;i++) {
		joint.m_constraints[i].reset();
	}
	
	joint.m_constraints[0].m_lock = SCE_PFX_JOINT_LOCK_FIX;
	joint.m_constraints[1].m_lock = SCE_PFX_JOINT_LOCK_FIX;
	joint.m_constraints[2].m_lock = SCE_PFX_JOINT_LOCK_FIX;
	joint.m_constraints[3].m_lock = SCE_PFX_JOINT_LOCK_LIMIT;
	joint.m_constraints[4].m_lock = SCE_PFX_JOINT_LOCK_LIMIT;
	joint.m_constraints[5].m_lock = SCE_PFX_JOINT_LOCK_FREE;
	
	// Set twist angle limit
	if(param.twistLowerAngle > param.twistUpperAngle || 
		!SCE_PFX_RANGE_CHECK(param.twistLowerAngle,-SCE_PFX_PI,SCE_PFX_PI) || 
		!SCE_PFX_RANGE_CHECK(param.twistUpperAngle,-SCE_PFX_PI,SCE_PFX_PI)) {
		return SCE_PFX_ERR_OUT_OF_RANGE;
	}
	joint.m_constraints[3].m_movableLowerLimit = param.twistLowerAngle;
	joint.m_constraints[3].m_movableUpperLimit = param.twistUpperAngle;
	
	// Set swing angle limit
	if(param.swingLowerAngle > param.swingUpperAngle || 
		!SCE_PFX_RANGE_CHECK(param.swingLowerAngle,0.0f,SCE_PFX_PI) || 
		!SCE_PFX_RANGE_CHECK(param.swingUpperAngle,0.0f,SCE_PFX_PI)) {
		return SCE_PFX_ERR_OUT_OF_RANGE;
	}
	joint.m_constraints[4].m_movableLowerLimit = param.swingLowerAngle;
	joint.m_constraints[4].m_movableUpperLimit = param.swingUpperAngle;
	
	// Calc joint frame
	PfxMatrix3 rotA = transpose(PfxMatrix3(stateA.getOrientation()));
	PfxMatrix3 rotB = transpose(PfxMatrix3(stateB.getOrientation()));
	
	PfxVector3 axisInA = rotA * normalize(param.twistAxis);
	
	joint.m_anchorA = rotA * (param.anchorPoint - stateA.getPosition());
	joint.m_anchorB = rotB * (param.anchorPoint - stateB.getPosition());
	
	PfxVector3 axis1, axis2;
	
	pfxGetPlaneSpace(axisInA, axis1, axis2 );
	
	joint.m_frameA = PfxMatrix3(axisInA, axis1, axis2);
	joint.m_frameB = rotB * PfxMatrix3(stateA.getOrientation()) * joint.m_frameA;
	
	return SCE_PFX_OK;
}

void pfxSetupSwingTwistJoint(
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

	PfxVector3 angAB = stateA.getAngularVelocity() - stateB.getAngularVelocity();

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

	PfxVector3 axis[3];
	PfxFloat angle[3];
	pfxCalcJointAngleSwingTwist(worldFrameA,worldFrameB,angle,axis);

	// Angular Constraint
	for(int c=3;c<6;c++) {
		PfxJointConstraint &jointConstraint = joint.m_constraints[c];
		PfxConstraintRow &constraint = jointConstraint.m_constraintRow;

		PfxVector3 normal = axis[c-3];

		PfxFloat posErr = angle[c-3];
		PfxFloat lowerLimit = -jointConstraint.m_maxImpulse;
		PfxFloat upperLimit =  jointConstraint.m_maxImpulse;
		PfxFloat velocityAmp = 1.0f;
		
		pfxCalcAngularLimit(jointConstraint,posErr,velocityAmp,lowerLimit,upperLimit);

		PfxFloat denom = dot((solverBodyA.m_inertiaInv+solverBodyB.m_inertiaInv)*normal,normal);
		
		constraint.m_rhs = -velocityAmp*dot(angAB,normal); // velocity error
		constraint.m_rhs -= (jointConstraint.m_bias * (-posErr)) / timeStep; // position error
		constraint.m_rhs *= jointConstraint.m_weight/denom;
		constraint.m_jacDiagInv = jointConstraint.m_weight*velocityAmp/denom;
		constraint.m_lowerLimit = lowerLimit;
		constraint.m_upperLimit = upperLimit;
		pfxStoreVector3(normal,constraint.m_normal);
	}
}

void pfxWarmStartSwingTwistJoint(
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

	// Angular Constraint
	for(int c=3;c<6;c++) {
		PfxJointConstraint &jointConstraint = joint.m_constraints[c];
		PfxConstraintRow &constraint = jointConstraint.m_constraintRow;

		if(jointConstraint.m_warmStarting == 0) {
			constraint.m_accumImpulse = 0.0f;
		}
		else {
			PfxVector3 normal = pfxReadVector3(constraint.m_normal);
			PfxFloat deltaImpulse = constraint.m_accumImpulse;
			solverBodyA.m_deltaAngularVelocity += deltaImpulse * solverBodyA.m_inertiaInv * normal;
			solverBodyB.m_deltaAngularVelocity -= deltaImpulse * solverBodyB.m_inertiaInv * normal;
		}
	}
}

void pfxSolveSwingTwistJoint(
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

	// Angular Constraint
	pfxSolveAngularConstraintRow(joint.m_constraints[3].m_constraintRow,
		solverBodyA.m_deltaAngularVelocity,solverBodyA.m_inertiaInv,rA,
		solverBodyB.m_deltaAngularVelocity,solverBodyB.m_inertiaInv,rB);

	pfxSolveAngularConstraintRow(joint.m_constraints[4].m_constraintRow,
		solverBodyA.m_deltaAngularVelocity,solverBodyA.m_inertiaInv,rA,
		solverBodyB.m_deltaAngularVelocity,solverBodyB.m_inertiaInv,rB);

	pfxSolveAngularConstraintRow(joint.m_constraints[5].m_constraintRow,
		solverBodyA.m_deltaAngularVelocity,solverBodyA.m_inertiaInv,rA,
		solverBodyB.m_deltaAngularVelocity,solverBodyB.m_inertiaInv,rB);
}

} //namespace PhysicsEffects
} //namespace sce
