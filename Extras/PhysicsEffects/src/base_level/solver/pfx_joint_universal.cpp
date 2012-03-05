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
#include "../../../include/physics_effects/base_level/solver/pfx_joint_universal.h"
#include "pfx_constraint_row_solver.h"

namespace sce {
namespace PhysicsEffects {

PfxInt32 pfxInitializeUniversalJoint(PfxJoint &joint,
	const PfxRigidState &stateA,const PfxRigidState &stateB,
	const PfxUniversalJointInitParam &param)
{
	joint.m_active = 1;
	joint.m_numConstraints = 6;
	joint.m_userData = NULL;
	joint.m_type = kPfxJointUniversal;
	joint.m_rigidBodyIdA = stateA.getRigidBodyId();
	joint.m_rigidBodyIdB = stateB.getRigidBodyId();
	
	for(int i=0;i<6;i++) {
		joint.m_constraints[i].reset();
	}
	
	joint.m_constraints[0].m_lock = SCE_PFX_JOINT_LOCK_FIX;
	joint.m_constraints[1].m_lock = SCE_PFX_JOINT_LOCK_FIX;
	joint.m_constraints[2].m_lock = SCE_PFX_JOINT_LOCK_FIX;
	joint.m_constraints[3].m_lock = SCE_PFX_JOINT_LOCK_FIX;
	joint.m_constraints[4].m_lock = SCE_PFX_JOINT_LOCK_LIMIT;
	joint.m_constraints[5].m_lock = SCE_PFX_JOINT_LOCK_LIMIT;
	
	// Set swing angle limit
	if(param.swing1LowerAngle > param.swing1UpperAngle || 
		!SCE_PFX_RANGE_CHECK(param.swing1LowerAngle,-SCE_PFX_PI,SCE_PFX_PI) || 
		!SCE_PFX_RANGE_CHECK(param.swing1UpperAngle,-SCE_PFX_PI,SCE_PFX_PI) || 
		param.swing2LowerAngle > param.swing2UpperAngle || 
		!SCE_PFX_RANGE_CHECK(param.swing2LowerAngle,-SCE_PFX_PI,SCE_PFX_PI) || 
		!SCE_PFX_RANGE_CHECK(param.swing2UpperAngle,-SCE_PFX_PI,SCE_PFX_PI)) {
		return SCE_PFX_ERR_OUT_OF_RANGE;
	}
	
	joint.m_constraints[4].m_movableLowerLimit = param.swing1LowerAngle;
	joint.m_constraints[4].m_movableUpperLimit = param.swing1UpperAngle;
	joint.m_constraints[5].m_movableLowerLimit = param.swing2LowerAngle;
	joint.m_constraints[5].m_movableUpperLimit = param.swing2UpperAngle;
	
	// Calc joint frame
	PfxMatrix3 rotA = transpose(PfxMatrix3(stateA.getOrientation()));
	PfxMatrix3 rotB = transpose(PfxMatrix3(stateB.getOrientation()));
	
	PfxVector3 axisInA = rotA * normalize(param.axis);
	
	joint.m_anchorA = rotA * (param.anchorPoint - stateA.getPosition());
	joint.m_anchorB = rotB * (param.anchorPoint - stateB.getPosition());
	
	PfxVector3 axis1, axis2;
	
	pfxGetPlaneSpace(axisInA, axis1, axis2 );
	
	joint.m_frameA = PfxMatrix3(axisInA, axis1, axis2);
	joint.m_frameB = rotB * PfxMatrix3(stateA.getOrientation()) * joint.m_frameA;
	
	return SCE_PFX_OK;
}

void pfxSetupUniversalJoint(
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
	pfxCalcJointAngleUniversal(worldFrameA,worldFrameB,angle,axis);

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
} //namespace PhysicsEffects
} //namespace sce
