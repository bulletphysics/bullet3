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

#include "../../../include/physics_effects/base_level/base/pfx_perf_counter.h"
#include "../../../include/physics_effects/base_level/solver/pfx_contact_constraint.h"
#include "../../../include/physics_effects/low_level/solver/pfx_joint_constraint_func.h"
#include "../../../include/physics_effects/low_level/solver/pfx_constraint_solver.h"
#include "../../base_level/solver/pfx_check_solver.h"
#include "pfx_parallel_group.h"

namespace sce {
namespace PhysicsEffects {

PfxUInt32 pfxGetWorkBytesOfSolveConstraints(PfxUInt32 numRigidBodies,PfxUInt32 numContactPairs,PfxUInt32 numJointPairs,PfxUInt32 maxTasks)
{
	(void)maxTasks;
	PfxUInt32 workBytes = SCE_PFX_ALLOC_BYTES_ALIGN16(sizeof(PfxUInt8) * numRigidBodies) +
		SCE_PFX_ALLOC_BYTES_ALIGN16(sizeof(PfxUInt32)*((SCE_PFX_MAX(numContactPairs,numJointPairs)+31)/32));

	workBytes += 128 + (SCE_PFX_ALLOC_BYTES_ALIGN16(sizeof(PfxParallelGroup)) + 
		 SCE_PFX_ALLOC_BYTES_ALIGN128(sizeof(PfxParallelBatch)*(SCE_PFX_MAX_SOLVER_PHASES*SCE_PFX_MAX_SOLVER_BATCHES))) * 2;

	return workBytes;
}

PfxInt32 pfxCheckParamOfSetupSolverBodies(const PfxSetupSolverBodiesParam &param)
{
	if(!param.states || !param.bodies || !param.solverBodies ) return SCE_PFX_ERR_INVALID_VALUE;
	if(!SCE_PFX_PTR_IS_ALIGNED16(param.states) || !SCE_PFX_PTR_IS_ALIGNED16(param.bodies) || !SCE_PFX_PTR_IS_ALIGNED16(param.solverBodies)) return SCE_PFX_ERR_INVALID_ALIGN;
	return SCE_PFX_OK;
}

PfxInt32 pfxCheckParamOfSetupContactConstraints(const PfxSetupContactConstraintsParam &param)
{
	if((param.numContactPairs>0&&(!param.contactPairs||!param.offsetContactManifolds)) || !param.offsetRigidStates || 
		!param.offsetRigidBodies  || !param.offsetSolverBodies || param.timeStep <= 0.0f) return SCE_PFX_ERR_INVALID_VALUE;
	if(!SCE_PFX_PTR_IS_ALIGNED16(param.contactPairs) || !SCE_PFX_PTR_IS_ALIGNED16(param.offsetContactManifolds) || !SCE_PFX_PTR_IS_ALIGNED16(param.offsetRigidStates) ||
		!SCE_PFX_PTR_IS_ALIGNED16(param.offsetRigidBodies) || !SCE_PFX_PTR_IS_ALIGNED16(param.offsetSolverBodies)) return SCE_PFX_ERR_INVALID_ALIGN;
	return SCE_PFX_OK;
}

PfxInt32 pfxCheckParamOfSetupJointConstraints(const PfxSetupJointConstraintsParam &param)
{
	if((param.numJointPairs>0&&(!param.jointPairs||!param.offsetJoints)) || !param.offsetRigidStates || 
		!param.offsetRigidBodies  || !param.offsetSolverBodies || param.timeStep <= 0.0f) return SCE_PFX_ERR_INVALID_VALUE;
	if(!SCE_PFX_PTR_IS_ALIGNED16(param.jointPairs) || !SCE_PFX_PTR_IS_ALIGNED16(param.offsetJoints) || !SCE_PFX_PTR_IS_ALIGNED16(param.offsetRigidStates) ||
		!SCE_PFX_PTR_IS_ALIGNED16(param.offsetRigidBodies) || !SCE_PFX_PTR_IS_ALIGNED16(param.offsetSolverBodies)) return SCE_PFX_ERR_INVALID_ALIGN;
	return SCE_PFX_OK;
}

PfxInt32 pfxCheckParamOfSolveConstraints(const PfxSolveConstraintsParam &param)
{
	if((param.numContactPairs>0&&(!param.contactPairs||!param.offsetContactManifolds)) || 
		(param.numJointPairs>0&&(!param.jointPairs||!param.offsetJoints)) || !param.offsetRigidStates || !param.offsetSolverBodies) return SCE_PFX_ERR_INVALID_VALUE;
	if(!SCE_PFX_PTR_IS_ALIGNED16(param.contactPairs) || !SCE_PFX_PTR_IS_ALIGNED16(param.offsetContactManifolds) || 
		!SCE_PFX_PTR_IS_ALIGNED16(param.jointPairs) || !SCE_PFX_PTR_IS_ALIGNED16(param.offsetJoints) || !SCE_PFX_PTR_IS_ALIGNED16(param.offsetRigidStates) ||
		!SCE_PFX_PTR_IS_ALIGNED16(param.offsetSolverBodies)) return SCE_PFX_ERR_INVALID_ALIGN;
	if(SCE_PFX_AVAILABLE_BYTES_ALIGN16(param.workBuff,param.workBytes) < pfxGetWorkBytesOfSolveConstraints(param.numRigidBodies,param.numContactPairs,param.numJointPairs) ) return SCE_PFX_ERR_OUT_OF_BUFFER;
	return SCE_PFX_OK;
}

///////////////////////////////////////////////////////////////////////////////
// SINGLE THREAD

PfxInt32 pfxSetupSolverBodies(PfxSetupSolverBodiesParam &param)
{
	PfxInt32 ret = pfxCheckParamOfSetupSolverBodies(param);
	if(ret != SCE_PFX_OK) return ret;
	
	SCE_PFX_PUSH_MARKER("pfxSetupSolverBodies");

	PfxRigidState *states = param.states;
	PfxRigidBody *bodies = param.bodies;
	PfxSolverBody *solverBodies = param.solverBodies;
	PfxUInt32 numRigidBodies = param.numRigidBodies;
	
	for(PfxUInt32 i=0;i<numRigidBodies;i++) {
		PfxRigidState &state = states[i];
		PfxRigidBody &body = bodies[i];
		PfxSolverBody &solverBody = solverBodies[i];
		
		solverBody.m_orientation = state.getOrientation();
		solverBody.m_deltaLinearVelocity = PfxVector3(0.0f);
		solverBody.m_deltaAngularVelocity = PfxVector3(0.0f);
		solverBody.m_motionType = state.getMotionMask();

		if(SCE_PFX_MOTION_MASK_DYNAMIC(state.getMotionType())) {
			PfxMatrix3 ori(solverBody.m_orientation);
			solverBody.m_massInv = body.getMassInv();
			solverBody.m_inertiaInv = ori * body.getInertiaInv() * transpose(ori);
		}
		else {
			solverBody.m_massInv = 0.0f;
			solverBody.m_inertiaInv = PfxMatrix3(0.0f);
		}
	}

	SCE_PFX_POP_MARKER();

	return SCE_PFX_OK;
}

PfxInt32 pfxSetupContactConstraints(PfxSetupContactConstraintsParam &param)
{
	PfxInt32 ret = pfxCheckParamOfSetupContactConstraints(param);
	if(ret != SCE_PFX_OK) return ret;
	
	SCE_PFX_PUSH_MARKER("pfxSetupContactConstraints");

	PfxConstraintPair *contactPairs = param.contactPairs;
	PfxUInt32 numContactPairs = param.numContactPairs;
	PfxContactManifold *offsetContactManifolds = param.offsetContactManifolds;
	PfxRigidState *offsetRigidStates = param.offsetRigidStates;
	PfxRigidBody *offsetRigidBodies = param.offsetRigidBodies;
	PfxSolverBody *offsetSolverBodies = param.offsetSolverBodies;
	
	for(PfxUInt32 i=0;i<numContactPairs;i++) {
		PfxConstraintPair &pair = contactPairs[i];
		if(!pfxCheckSolver(pair)) {
			continue;
		}

		PfxUInt16 iA = pfxGetObjectIdA(pair);
		PfxUInt16 iB = pfxGetObjectIdB(pair);
		PfxUInt32 iConstraint = pfxGetConstraintId(pair);

		PfxContactManifold &contact = offsetContactManifolds[iConstraint];

		SCE_PFX_ALWAYS_ASSERT(iA==contact.getRigidBodyIdA());
		SCE_PFX_ALWAYS_ASSERT(iB==contact.getRigidBodyIdB());

		PfxRigidState &stateA = offsetRigidStates[iA];
		PfxRigidBody &bodyA = offsetRigidBodies[iA];
		PfxSolverBody &solverBodyA = offsetSolverBodies[iA];

		PfxRigidState &stateB = offsetRigidStates[iB];
		PfxRigidBody &bodyB = offsetRigidBodies[iB];
		PfxSolverBody &solverBodyB = offsetSolverBodies[iB];
	
		contact.setInternalFlag(0);
		
		PfxFloat restitution = 0.5f * (bodyA.getRestitution() + bodyB.getRestitution());
		if(contact.getDuration() > 1) restitution = 0.0f;
		
		PfxFloat friction = sqrtf(bodyA.getFriction() * bodyB.getFriction());
		
		for(int j=0;j<contact.getNumContacts();j++) {
			PfxContactPoint &cp = contact.getContactPoint(j);
			
			pfxSetupContactConstraint(
				cp.m_constraintRow[0],
				cp.m_constraintRow[1],
				cp.m_constraintRow[2],
				cp.m_distance,
				restitution,
				friction,
				pfxReadVector3(cp.m_constraintRow[0].m_normal),
				pfxReadVector3(cp.m_localPointA),
				pfxReadVector3(cp.m_localPointB),
				stateA,
				stateB,
				solverBodyA,
				solverBodyB,
				param.separateBias,
				param.timeStep
				);
		}

		contact.setCompositeFriction(friction);
	}

	SCE_PFX_POP_MARKER();

	return SCE_PFX_OK;
}

PfxInt32 pfxSetupJointConstraints(PfxSetupJointConstraintsParam &param)
{
	PfxInt32 ret = pfxCheckParamOfSetupJointConstraints(param);
	if(ret != SCE_PFX_OK) return ret;
	
	SCE_PFX_PUSH_MARKER("pfxSetupJointConstraints");

	PfxConstraintPair *jointPairs = param.jointPairs;
	PfxUInt32 numJointPairs = param.numJointPairs;
	PfxJoint *offsetJoints = param.offsetJoints;
	PfxRigidState *offsetRigidStates = param.offsetRigidStates;
	PfxSolverBody *offsetSolverBodies = param.offsetSolverBodies;
	
	for(PfxUInt32 i=0;i<numJointPairs;i++) {
		PfxConstraintPair &pair = jointPairs[i];
		if(!pfxCheckSolver(pair)) {
			continue;
		}

		PfxUInt16 iA = pfxGetObjectIdA(pair);
		PfxUInt16 iB = pfxGetObjectIdB(pair);
		PfxUInt32 iConstraint = pfxGetConstraintId(pair);
		
		PfxJoint &joint = offsetJoints[iConstraint];

		SCE_PFX_ALWAYS_ASSERT(iA==joint.m_rigidBodyIdA);
		SCE_PFX_ALWAYS_ASSERT(iB==joint.m_rigidBodyIdB);

		PfxRigidState &stateA = offsetRigidStates[iA];
		PfxSolverBody &solverBodyA = offsetSolverBodies[iA];

		PfxRigidState &stateB = offsetRigidStates[iB];
		PfxSolverBody &solverBodyB = offsetSolverBodies[iB];
		
		pfxGetSetupJointConstraintFunc(joint.m_type)(
			joint,
			stateA,
			stateB,
			solverBodyA,
			solverBodyB,
			param.timeStep);
	}

	SCE_PFX_POP_MARKER();

	return SCE_PFX_OK;
}

PfxInt32 pfxSolveConstraints(PfxSolveConstraintsParam &param)
{
	PfxInt32 ret = pfxCheckParamOfSolveConstraints(param);
	if(ret != SCE_PFX_OK) return ret;

	SCE_PFX_PUSH_MARKER("pfxSolveConstraints");

	PfxConstraintPair *contactPairs = param.contactPairs;
	PfxUInt32 numContactPairs = param.numContactPairs;
	PfxContactManifold *offsetContactManifolds = param.offsetContactManifolds;
	PfxConstraintPair *jointPairs = param.jointPairs;
	PfxUInt32 numJointPairs = param.numJointPairs;
	PfxJoint *offsetJoints = param.offsetJoints;
	PfxRigidState *offsetRigidStates = param.offsetRigidStates;
	PfxSolverBody *offsetSolverBodies = param.offsetSolverBodies;
	PfxUInt32 numRigidBodies = param.numRigidBodies;
	
	// Warm Starting
	{
		for(PfxUInt32 i=0;i<numJointPairs;i++) {
			PfxConstraintPair &pair = jointPairs[i];
			if(!pfxCheckSolver(pair)) {
				continue;
			}

			PfxUInt16 iA = pfxGetObjectIdA(pair);
			PfxUInt16 iB = pfxGetObjectIdB(pair);

			PfxJoint &joint = offsetJoints[pfxGetConstraintId(pair)];

			SCE_PFX_ASSERT(iA==joint.m_rigidBodyIdA);
			SCE_PFX_ASSERT(iB==joint.m_rigidBodyIdB);

			PfxSolverBody &solverBodyA = offsetSolverBodies[iA];
			PfxSolverBody &solverBodyB = offsetSolverBodies[iB];
			
			pfxGetWarmStartJointConstraintFunc(joint.m_type)(
				joint,
				solverBodyA,
				solverBodyB);
		}
		for(PfxUInt32 i=0;i<numContactPairs;i++) {
			PfxConstraintPair &pair = contactPairs[i];
			if(!pfxCheckSolver(pair)) {
				continue;
			}

			PfxUInt16 iA = pfxGetObjectIdA(pair);
			PfxUInt16 iB = pfxGetObjectIdB(pair);

			PfxContactManifold &contact = offsetContactManifolds[pfxGetConstraintId(pair)];

			SCE_PFX_ASSERT(iA==contact.getRigidBodyIdA());
			SCE_PFX_ASSERT(iB==contact.getRigidBodyIdB());

			PfxSolverBody &solverBodyA = offsetSolverBodies[iA];
			PfxSolverBody &solverBodyB = offsetSolverBodies[iB];
			
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

			for(int j=0;j<contact.getNumContacts();j++) {
				PfxContactPoint &cp = contact.getContactPoint(j);
				
				PfxVector3 rA = rotate(solverBodyA.m_orientation,pfxReadVector3(cp.m_localPointA));
				PfxVector3 rB = rotate(solverBodyB.m_orientation,pfxReadVector3(cp.m_localPointB));
				
				for(int k=0;k<3;k++) {
					PfxVector3 normal = pfxReadVector3(cp.m_constraintRow[k].m_normal);
					PfxFloat deltaImpulse = cp.m_constraintRow[k].m_accumImpulse;
					solverBodyA.m_deltaLinearVelocity += deltaImpulse * massInvA * normal;
					solverBodyA.m_deltaAngularVelocity += deltaImpulse * inertiaInvA * cross(rA,normal);
					solverBodyB.m_deltaLinearVelocity -= deltaImpulse * massInvB * normal;
					solverBodyB.m_deltaAngularVelocity -= deltaImpulse * inertiaInvB * cross(rB,normal);
				}
			}
		}
	}
	
	// Solver
	for(PfxUInt32 iteration=0;iteration<param.iteration;iteration++) {
		for(PfxUInt32 i=0;i<numJointPairs;i++) {
			PfxConstraintPair &pair = jointPairs[i];
			if(!pfxCheckSolver(pair)) {
				continue;
			}

			PfxUInt16 iA = pfxGetObjectIdA(pair);
			PfxUInt16 iB = pfxGetObjectIdB(pair);

			PfxJoint &joint = offsetJoints[pfxGetConstraintId(pair)];

			SCE_PFX_ASSERT(iA==joint.m_rigidBodyIdA);
			SCE_PFX_ASSERT(iB==joint.m_rigidBodyIdB);

			PfxSolverBody &solverBodyA = offsetSolverBodies[iA];
			PfxSolverBody &solverBodyB = offsetSolverBodies[iB];
			
			pfxGetSolveJointConstraintFunc(joint.m_type)(
				joint,
				solverBodyA,
				solverBodyB);
		}
		for(PfxUInt32 i=0;i<numContactPairs;i++) {
			PfxConstraintPair &pair = contactPairs[i];
			if(!pfxCheckSolver(pair)) {
				continue;
			}

			PfxUInt16 iA = pfxGetObjectIdA(pair);
			PfxUInt16 iB = pfxGetObjectIdB(pair);

			PfxContactManifold &contact = offsetContactManifolds[pfxGetConstraintId(pair)];

			SCE_PFX_ASSERT(iA==contact.getRigidBodyIdA());
			SCE_PFX_ASSERT(iB==contact.getRigidBodyIdB());

			PfxSolverBody &solverBodyA = offsetSolverBodies[iA];
			PfxSolverBody &solverBodyB = offsetSolverBodies[iB];
			
			for(int j=0;j<contact.getNumContacts();j++) {
				PfxContactPoint &cp = contact.getContactPoint(j);
				
				pfxSolveContactConstraint(
					cp.m_constraintRow[0],
					cp.m_constraintRow[1],
					cp.m_constraintRow[2],
					pfxReadVector3(cp.m_localPointA),
					pfxReadVector3(cp.m_localPointB),
					solverBodyA,
					solverBodyB,
					contact.getCompositeFriction()
					);
			}
		}
	}

	for(PfxUInt32 i=0;i<numRigidBodies;i++) {
		offsetRigidStates[i].setLinearVelocity(
			offsetRigidStates[i].getLinearVelocity()+offsetSolverBodies[i].m_deltaLinearVelocity);
		offsetRigidStates[i].setAngularVelocity(
			offsetRigidStates[i].getAngularVelocity()+offsetSolverBodies[i].m_deltaAngularVelocity);
	}

	SCE_PFX_POP_MARKER();

	return SCE_PFX_OK;
}

} //namespace PhysicsEffects
} //namespace sce
