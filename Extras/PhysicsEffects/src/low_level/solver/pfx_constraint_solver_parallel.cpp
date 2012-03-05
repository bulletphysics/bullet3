/*
Applied Research Associates Inc. (c)2011

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

///////////////////////////////////////////////////////////////////////////////
// These functions are implemented in pfx_constraint_solver_single.cpp
extern PfxUInt32 pfxGetWorkBytesOfSolveConstraints(PfxUInt32 numRigidBodies,PfxUInt32 numContactPairs,PfxUInt32 numJointPairs,PfxUInt32 maxTasks);
extern PfxInt32 pfxCheckParamOfSetupSolverBodies(const PfxSetupSolverBodiesParam &param);
extern PfxInt32 pfxCheckParamOfSetupContactConstraints(const PfxSetupContactConstraintsParam &param);
extern PfxInt32 pfxCheckParamOfSetupJointConstraints(const PfxSetupJointConstraintsParam &param);
extern PfxInt32 pfxCheckParamOfSolveConstraints(const PfxSolveConstraintsParam &param);

//----------------------------------------------------------------------------
//  pfxCheckParamOfSolveConstraints
//
/// This function verifies that the input parameter for solving constraints
/// in parallel is good.
//----------------------------------------------------------------------------
PfxInt32 pfxCheckParamOfSolveConstraints(const PfxSolveConstraintsParam &param,
	PfxTaskManager *taskManager)
{
	if((param.numContactPairs>0&&(!param.contactPairs||!param.offsetContactManifolds)) || 
		(param.numJointPairs>0&&(!param.jointPairs||!param.offsetJoints)) || !param.offsetRigidStates || !param.offsetSolverBodies) return SCE_PFX_ERR_INVALID_VALUE;
	if(!SCE_PFX_PTR_IS_ALIGNED16(param.contactPairs) || !SCE_PFX_PTR_IS_ALIGNED16(param.offsetContactManifolds) || 
		!SCE_PFX_PTR_IS_ALIGNED16(param.jointPairs) || !SCE_PFX_PTR_IS_ALIGNED16(param.offsetJoints) || !SCE_PFX_PTR_IS_ALIGNED16(param.offsetRigidStates) ||
		!SCE_PFX_PTR_IS_ALIGNED16(param.offsetSolverBodies)) return SCE_PFX_ERR_INVALID_ALIGN;
	if(SCE_PFX_AVAILABLE_BYTES_ALIGN16(param.workBuff,param.workBytes) < pfxGetWorkBytesOfSolveConstraints(param.numRigidBodies,param.numContactPairs,param.numJointPairs, taskManager->getNumTasks()) ) return SCE_PFX_ERR_OUT_OF_BUFFER;
	return SCE_PFX_OK;
}

///////////////////////////////////////////////////////////////////////////////
// MULTIPLE THREADS

//----------------------------------------------------------------------------
//  pfxSetupSolverBodiesTaskEntry
//
/// The thread PfxTaskEntry function used to setup solver bodies in parallel.
//----------------------------------------------------------------------------
void pfxSetupSolverBodiesTaskEntry(PfxTaskArg *arg)
{
	PfxSetupSolverBodiesParam &param = *((PfxSetupSolverBodiesParam*)arg->io);

	PfxUInt32 iFirstBody = arg->data[0];
	PfxUInt32 iEndBody = arg->data[1];

	PfxRigidState *states = param.states;
	PfxRigidBody *bodies = param.bodies;
	PfxSolverBody *solverBodies = param.solverBodies;
	
	for(PfxUInt32 i = iFirstBody; i < iEndBody; i++)
	{
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
}

//----------------------------------------------------------------------------
//  pfxSetupSolverBodies
//
/// Perform setup solver bodies in parallel using a task manager.
///
/// @param param        Information about rigid bodies
/// @param taskManager  Pointer to the thread task manager to use
///
/// @return SCE_PFX_OK if successful, otherwise, returns an error code.
//----------------------------------------------------------------------------
PfxInt32 pfxSetupSolverBodies(PfxSetupSolverBodiesParam &param,
	PfxTaskManager *taskManager)
{
	PfxInt32 ret = pfxCheckParamOfSetupSolverBodies(param);
	if(ret != SCE_PFX_OK) return ret;
	
	SCE_PFX_PUSH_MARKER("pfxSetupSolverBodies");

	PfxUInt32 maxBatchSize = param.numRigidBodies / (PfxUInt32)(taskManager->getNumTasks());
	PfxUInt32 iEnd = maxBatchSize, iStart = 0;
	int task = 0;
	taskManager->setTaskEntry((void*)pfxSetupSolverBodiesTaskEntry);

	for (task = 0; task < taskManager->getNumTasks() - 1; task++, iStart += maxBatchSize, iEnd += maxBatchSize)
	{
		taskManager->startTask(task, static_cast<void*>(&param), iStart, iEnd, 0, 0);
	}

	// send final task
	iEnd = param.numRigidBodies;
	taskManager->startTask(taskManager->getNumTasks() - 1, static_cast<void*>(&param), iStart, iEnd, 0, 0);

	// wait for tasks to complete
	PfxUInt32 data1, data2, data3, data4;
	for (PfxUInt32 i = 0; i < taskManager->getNumTasks(); i++)
		taskManager->waitTask(task, data1, data2, data3, data4);

	SCE_PFX_POP_MARKER();
	
	return SCE_PFX_OK;
}

//----------------------------------------------------------------------------
//  pfxSetupContactConstraintsTaskEntry
//
/// The thread PfxTaskEntry function used to setup contact constraints in
/// parallel.
//----------------------------------------------------------------------------
void pfxSetupContactConstraintsTaskEntry(PfxTaskArg *arg)
{
	PfxSetupContactConstraintsParam &param = *((PfxSetupContactConstraintsParam*)arg->io);

	PfxUInt32 iFirstContactPair = arg->data[0];
	PfxUInt32 iEndContactPair = arg->data[1];

	PfxConstraintPair *contactPairs = param.contactPairs;
	PfxUInt32 numContactPairs = param.numContactPairs;
	PfxContactManifold *offsetContactManifolds = param.offsetContactManifolds;
	PfxRigidState *offsetRigidStates = param.offsetRigidStates;
	PfxRigidBody *offsetRigidBodies = param.offsetRigidBodies;
	PfxSolverBody *offsetSolverBodies = param.offsetSolverBodies;
	
	for(PfxUInt32 i = iFirstContactPair; i < iEndContactPair; i++)
	{
		PfxConstraintPair &pair = contactPairs[i];
		if(!pfxCheckSolver(pair))
		{
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
		
		for(int j=0;j<contact.getNumContacts();j++)
		{
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
}

//----------------------------------------------------------------------------
//  pfxSetupContactConstraints
//
/// Perform setup contact constraints in parallel using a task manager.
///
/// @param param        Information about contact constraints
/// @param taskManager  Pointer to the thread task manager to use
///
/// @return SCE_PFX_OK if successful, otherwise, returns an error code.
//----------------------------------------------------------------------------
PfxInt32 pfxSetupContactConstraints(PfxSetupContactConstraintsParam &param,
	PfxTaskManager *taskManager)
{
	PfxInt32 ret = pfxCheckParamOfSetupContactConstraints(param);
	if(ret != SCE_PFX_OK) return ret;
	
	SCE_PFX_PUSH_MARKER("pfxSetupContactConstraints");

	PfxUInt32 maxBatchSize = param.numContactPairs / (PfxUInt32)(taskManager->getNumTasks());
	PfxUInt32 iEnd = maxBatchSize, iStart = 0;
	int task = 0;
	taskManager->setTaskEntry((void*)pfxSetupContactConstraintsTaskEntry);

	for (task = 0; task < taskManager->getNumTasks() - 1; task++, iStart += maxBatchSize, iEnd += maxBatchSize)
	{
		taskManager->startTask(task, static_cast<void*>(&param), iStart, iEnd, 0, 0);
	}

	// send final task
	iEnd = param.numContactPairs;
	taskManager->startTask(taskManager->getNumTasks() - 1, static_cast<void*>(&param), iStart, iEnd, 0, 0);

	// wait for tasks to complete
	PfxUInt32 data1, data2, data3, data4;
	for (PfxUInt32 i = 0; i < taskManager->getNumTasks(); i++)
		taskManager->waitTask(task, data1, data2, data3, data4);

	SCE_PFX_POP_MARKER();
	
	return SCE_PFX_OK;
}

//----------------------------------------------------------------------------
//  pfxSetupJointConstraintsTaskEntry
//
/// The thread PfxTaskEntry function used to setup joint constraints in
/// parallel.
//----------------------------------------------------------------------------
void pfxSetupJointConstraintsTaskEntry(PfxTaskArg *arg)
{
	PfxSetupJointConstraintsParam &param = *((PfxSetupJointConstraintsParam*)arg->io);

	PfxUInt32 iFirstJointPair = arg->data[0];
	PfxUInt32 iEndJointPair = arg->data[1];

	PfxConstraintPair *jointPairs = param.jointPairs;
	PfxUInt32 numJointPairs = param.numJointPairs;
	PfxJoint *offsetJoints = param.offsetJoints;
	PfxRigidState *offsetRigidStates = param.offsetRigidStates;
	PfxSolverBody *offsetSolverBodies = param.offsetSolverBodies;
	
	for(PfxUInt32 i = iFirstJointPair; i < iEndJointPair; i++)
	{
		PfxConstraintPair &pair = jointPairs[i];
		if(!pfxCheckSolver(pair))
		{
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
}

//----------------------------------------------------------------------------
//  pfxSetupJointConstraints
//
/// Perform setup joint constraints in parallel using a task manager.
///
/// @param param        Information about joint constraints
/// @param taskManager  Pointer to the thread task manager to use
///
/// @return SCE_PFX_OK if successful, otherwise, returns an error code.
//----------------------------------------------------------------------------
PfxInt32 pfxSetupJointConstraints(PfxSetupJointConstraintsParam &param,
	PfxTaskManager *taskManager)
{
	PfxInt32 ret = pfxCheckParamOfSetupJointConstraints(param);
	if(ret != SCE_PFX_OK) return ret;
	
	SCE_PFX_PUSH_MARKER("pfxSetupJointConstraints");

	PfxUInt32 maxBatchSize = param.numJointPairs / (PfxUInt32)(taskManager->getNumTasks());
	PfxUInt32 iEnd = maxBatchSize, iStart = 0;
	int task = 0;
	taskManager->setTaskEntry((void*)pfxSetupJointConstraintsTaskEntry);

	for (task = 0; task < taskManager->getNumTasks() - 1; task++, iStart += maxBatchSize, iEnd += maxBatchSize)
	{
		taskManager->startTask(task, static_cast<void*>(&param), iStart, iEnd, 0, 0);
	}

	// send final task
	iEnd = param.numJointPairs;
	taskManager->startTask(taskManager->getNumTasks() - 1, static_cast<void*>(&param), iStart, iEnd, 0, 0);

	// wait for tasks to complete
	PfxUInt32 data1, data2, data3, data4;
	for (PfxUInt32 i = 0; i < taskManager->getNumTasks(); i++)
		taskManager->waitTask(task, data1, data2, data3, data4);

	SCE_PFX_POP_MARKER();
	
	return SCE_PFX_OK;
}

//----------------------------------------------------------------------------
//  pfxSplitConstraints
//
/// Given a set of constraints to be solved, split the constraints into
/// a collection of phases, with each phase having one or more independent
/// batches that can be solved in parallel. The phases must be solved
/// sequentially.
///
/// @param numRigidBodies    [in] Total number of rigid bodies referenced
///                          in the given set of constraints
/// @param constraintpairs   [in] Pointer to array of constraints to split.
///                          For clarity, note that the pairs always stay
///                          together. Some pairs are split to be solved
///                          in parallel with other pairs.
/// @param numConstraints    [in] Number of constraints to split
/// @param taskManager       [in] Pointer to the thread task manager that will
///                          eventually be used to solve the constraints.
/// @param group             [out] On return, contains information about
///                          the phases and batches that define the splitting
/// @param batches           [out] Caller should pass a pointer to a pre-
///                          allocated array of SCE_PFX_MAX_SOLVER_BATCHES
///                          PfxParallelBatch objects. On output, these will
///                          be populated with the correct number of pairs
///                          for each batch.

///
/// @return SCE_PFX_OK if successful, otherwise, returns an error code.
//----------------------------------------------------------------------------
void pfxSplitConstraints(PfxUInt32 numRigidBodies, PfxConstraintPair *constraintPairs,
	PfxUInt32 numConstraints, PfxTaskManager *taskManager, PfxParallelGroup *group,
	PfxParallelBatch *batches)
{
	SCE_PFX_PUSH_MARKER("pfxSplitConstraints");

	// allocate a table that will be used to indicate, for a given phase being
	// populated, which batch a given body belongs to.
	PfxInt32 bufSize = sizeof(PfxUInt8) * numRigidBodies;
	bufSize = ((bufSize + 127) >> 7) << 7; // 128 bytes alignment
	PfxUInt8 *bodyTable = (PfxUInt8*)taskManager->allocate(bufSize);

	// allocate a table that will be used to indicate, for a given phase being
	// populated, which batch a given pair of bodies belongs to.
	PfxUInt32 *pairTable;
	size_t allocSize = sizeof(PfxUInt32)*((numConstraints + 31) / 32);
	pairTable = (PfxUInt32*)taskManager->allocate(allocSize);
	memset(pairTable, 0, allocSize);

	// 
	PfxUInt32 numTasks = taskManager->getNumTasks();
	PfxUInt32 targetCount = SCE_PFX_MAX(PfxUInt32(SCE_PFX_MIN_SOLVER_PAIRS),
										SCE_PFX_MIN(numConstraints / (numTasks * 2), PfxUInt32(SCE_PFX_MAX_SOLVER_PAIRS)));
	PfxUInt32 startIndex = 0;
	
	PfxUInt32 phaseId;
	PfxUInt32 batchId;
	PfxUInt32 totalCount = 0;
	
	PfxUInt32 maxBatches = SCE_PFX_MIN(numTasks, PfxUInt32(SCE_PFX_MAX_SOLVER_BATCHES));

	// accumulate phases and batches until group resources are exhausted or all incoming
	// pairs are accounted for.
	for (phaseId = 0; phaseId < SCE_PFX_MAX_SOLVER_PHASES && totalCount < numConstraints; phaseId++)
	{
		bool startIndexCheck = true;

		group->numBatches[phaseId] = 0;

		PfxUInt32 i = startIndex;
		// Initialize body table such that no body is assigned to any batch. (0xff is explicitly assumed to
		// mean no batch assigned)
		memset(bodyTable, 0xff, bufSize);

		// accumulate batches within the current phase. This code creates batches that are
		// independent, e.g., no batch on this phase will touch the same bodies as any other
		// batch on the phase. Batches within a phase can be solved in parallel on shared memory
		// multiprocessor hardware.
		for (batchId = 0; i < numConstraints && totalCount < numConstraints && batchId < maxBatches; batchId++)
		{
			PfxUInt32 pairCount=0;
			PfxParallelBatch &batch = batches[(phaseId * SCE_PFX_MAX_SOLVER_BATCHES) + batchId];
			PfxUInt32 pairId = 0;

			// iterate through pairs, and assigns the pairs to batches
			for (; i < numConstraints && pairCount < targetCount; i++)
			{
				PfxUInt32 idxP = i >> 5;
				PfxUInt32 maskP = 1L << (i & 31);

				if(pairTable[idxP] & maskP)	// pair is already assigned to a phase/batch
					continue;

				PfxUInt16 idxA = pfxGetObjectIdA(constraintPairs[i]);
				PfxUInt16 idxB = pfxGetObjectIdB(constraintPairs[i]);

				// It is possible an incoming constraint pair can be skipped. For example, if the pair is inactive,
				// or if both its objects are static, unmoving objects and therefore would be unaffected by constraints.
				// This conditional statement addresses constraints to be skipped.
				if (!pfxGetActive(constraintPairs[i]) ||
					(SCE_PFX_MOTION_MASK_STATIC(pfxGetMotionMaskA(constraintPairs[i])&SCE_PFX_MOTION_MASK_TYPE) &&
					 SCE_PFX_MOTION_MASK_STATIC(pfxGetMotionMaskB(constraintPairs[i])&SCE_PFX_MOTION_MASK_TYPE)) )
				{
					if (startIndexCheck) 
						startIndex++;

					//assign pair -> skip it because it has no constraints
					pairTable[idxP] |= maskP;
					totalCount++;
					continue;
				}

				// If either body of the current pair belongs to another batch already, we cannot add the current
				// pair to the current batch. Must defer to another phase
				if( (bodyTable[idxA] != batchId && bodyTable[idxA] != 0xff) || 
					(bodyTable[idxB] != batchId && bodyTable[idxB] != 0xff) )
				{
					startIndexCheck = false; // so we will revisit this during allocation of next phase
					continue;
				}

				// Dynamic bodies for current pair are assigned to the current batch in this phase.
				// Static bodies are not assigned. Since they never move, and their corresponding solver
				// bodies are therefore never touched, they can actually be used by any batch.
				if (SCE_PFX_MOTION_MASK_DYNAMIC(pfxGetMotionMaskA(constraintPairs[i])&SCE_PFX_MOTION_MASK_TYPE))
					bodyTable[idxA] = batchId;

				if (SCE_PFX_MOTION_MASK_DYNAMIC(pfxGetMotionMaskB(constraintPairs[i])&SCE_PFX_MOTION_MASK_TYPE))
					bodyTable[idxB] = batchId;

				if(startIndexCheck)
					startIndex++;

				pairTable[idxP] |= maskP;	// pair has been handled

				//add the pair 'i' to the current batch
				batch.pairIndices[pairId++] = i;
				pairCount++;
			}

			group->numPairs[(phaseId * SCE_PFX_MAX_SOLVER_BATCHES) + batchId] = (PfxUInt16)pairId;
			totalCount += pairCount;
		}
		group->numBatches[phaseId] = batchId;
	}

	group->numPhases = phaseId;

	taskManager->deallocate(bodyTable);
	taskManager->deallocate(pairTable);

	SCE_PFX_POP_MARKER();
}

//----------------------------------------------------------------------------
//  pfxSolveConstraintsTaskEntry
//
/// The thread PfxTaskEntry function used to solve constraints in parallel.
//----------------------------------------------------------------------------
void pfxSolveConstraintsTaskEntry(PfxTaskArg *arg)
{
	PfxSolveConstraintsParam &param = *((PfxSolveConstraintsParam*)arg->io);

	PfxConstraintPair *contactPairs = param.contactPairs;
	PfxContactManifold *offsetContactManifolds = param.offsetContactManifolds;
	PfxConstraintPair *jointPairs = param.jointPairs;
	PfxJoint *offsetJoints = param.offsetJoints;
	PfxRigidState *offsetRigidStates = param.offsetRigidStates;
	PfxSolverBody *offsetSolverBodies = param.offsetSolverBodies;

	PfxParallelGroup *jointgroup = (PfxParallelGroup*)arg->data[0];
	PfxParallelBatch *jointbatches = (PfxParallelBatch*)arg->data[1];
	PfxParallelGroup *contactgroup = (PfxParallelGroup*)arg->data[2];
	PfxParallelBatch *contactbatches = (PfxParallelBatch*)arg->data[3];

	// Warm Starting
	{
		// Joints
		for (PfxUInt16 phase = 0; phase < jointgroup->numPhases; phase++)
		{
			for (PfxUInt16 batchId = 0; batchId < jointgroup->numBatches[phase]; batchId++)
			{
				PfxUInt16 numJointPairs = jointgroup->numPairs[(phase * SCE_PFX_MAX_SOLVER_BATCHES) + batchId];
				if ((arg->taskId == (batchId % arg->maxTasks)) && numJointPairs > 0) // only spend time on batches meant for this task
				{
					const PfxParallelBatch &batch = jointbatches[(phase * SCE_PFX_MAX_SOLVER_BATCHES) + batchId];
					for (PfxUInt16 i = 0; i < numJointPairs; i++)
					{
						PfxConstraintPair &pair = jointPairs[batch.pairIndices[i]];
						if(!pfxCheckSolver(pair))
							continue;

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
				}

				arg->barrier->sync();	// block until all threads are ready to go to next phase
			}
		}

		// Contacts
		for (PfxUInt16 phase = 0; phase < contactgroup->numPhases; phase++)
		{
			for (PfxUInt16 batchId = 0; batchId < contactgroup->numBatches[phase]; batchId++)
			{
				PfxUInt16 numContactPairs = contactgroup->numPairs[(phase * SCE_PFX_MAX_SOLVER_BATCHES) + batchId];
				if ((arg->taskId == (batchId % arg->maxTasks)) && numContactPairs > 0) // only spend time on batches meant for this task
				{
					const PfxParallelBatch &batch = contactbatches[(phase * SCE_PFX_MAX_SOLVER_BATCHES) + batchId];
					for (PfxUInt16 i = 0; i < numContactPairs; i++)
					{
						PfxConstraintPair &pair = contactPairs[batch.pairIndices[i]];
						if(!pfxCheckSolver(pair))
						{
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

						if(solverBodyA.m_motionType == kPfxMotionTypeOneWay)
						{
							massInvB = 0.0f;
							inertiaInvB = PfxMatrix3(0.0f);
						}

						if(solverBodyB.m_motionType == kPfxMotionTypeOneWay)
						{
							massInvA = 0.0f;
							inertiaInvA = PfxMatrix3(0.0f);
						}

						for(int j = 0; j < contact.getNumContacts(); j++)
						{
							PfxContactPoint &cp = contact.getContactPoint(j);
				
							PfxVector3 rA = rotate(solverBodyA.m_orientation,pfxReadVector3(cp.m_localPointA));
							PfxVector3 rB = rotate(solverBodyB.m_orientation,pfxReadVector3(cp.m_localPointB));
				
							for(int k = 0; k < 3; k++)
							{
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
			}

			arg->barrier->sync();	// block until all threads are ready to go to next phase
		}
	}
	
	// Solver
	for(PfxUInt32 iteration = 0; iteration < param.iteration; iteration++)
	{
		// Joints
		for (PfxUInt16 phase = 0; phase < jointgroup->numPhases; phase++)
		{
			for (PfxUInt16 batchId = 0; batchId < jointgroup->numBatches[phase]; batchId++)
			{
				PfxUInt16 numJointPairs = jointgroup->numPairs[(phase * SCE_PFX_MAX_SOLVER_BATCHES) + batchId];
				if ((arg->taskId == (batchId % arg->maxTasks)) && numJointPairs > 0) // only spend time on batches meant for this task
				{
					const PfxParallelBatch &batch = jointbatches[(phase * SCE_PFX_MAX_SOLVER_BATCHES) + batchId];
					for(PfxUInt16 i = 0; i < numJointPairs; i++)
					{
						PfxConstraintPair &pair = jointPairs[batch.pairIndices[i]];
						if(!pfxCheckSolver(pair))
							continue;

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
				}
			}

			arg->barrier->sync();	// block until all threads are ready to go to next phase
		}

		// Contacts
		for (PfxUInt32 phase = 0; phase < contactgroup->numPhases; phase++)
		{
			for (PfxUInt32 batchId = 0; batchId < contactgroup->numBatches[phase]; batchId++)
			{
				PfxUInt32 numContactPairs = contactgroup->numPairs[(phase * SCE_PFX_MAX_SOLVER_BATCHES) + batchId];
				if ((arg->taskId == (batchId % arg->maxTasks)) && numContactPairs > 0) // only spend time on batches meant for this task
				{
					const PfxParallelBatch &batch = contactbatches[(phase * SCE_PFX_MAX_SOLVER_BATCHES) + batchId];
					for(PfxUInt32 i = 0; i < numContactPairs; i++)
					{
						PfxConstraintPair &pair = contactPairs[batch.pairIndices[i]];
						if(!pfxCheckSolver(pair))
							continue;

						PfxUInt16 iA = pfxGetObjectIdA(pair);
						PfxUInt16 iB = pfxGetObjectIdB(pair);

						PfxContactManifold &contact = offsetContactManifolds[pfxGetConstraintId(pair)];

						SCE_PFX_ASSERT(iA==contact.getRigidBodyIdA());
						SCE_PFX_ASSERT(iB==contact.getRigidBodyIdB());

						PfxSolverBody &solverBodyA = offsetSolverBodies[iA];
						PfxSolverBody &solverBodyB = offsetSolverBodies[iB];
			
						for(int j = 0; j < contact.getNumContacts(); j++)
						{
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
			}

			arg->barrier->sync();	// block until all threads are ready to go to next phase
		}
	}
}

//----------------------------------------------------------------------------
//  pfxSolveConstraints
//
/// Perform setup joint constraints in parallel using a task manager.
///
/// @param param        Information about constraints
/// @param taskManager  Pointer to the thread task manager to use
///
/// @return SCE_PFX_OK if successful, otherwise, returns an error code.
//----------------------------------------------------------------------------
PfxInt32 pfxSolveConstraints(PfxSolveConstraintsParam &param,
	PfxTaskManager *taskManager)
{
	PfxInt32 ret = pfxCheckParamOfSolveConstraints(param);
	if(ret != SCE_PFX_OK) return ret;

	SCE_PFX_PUSH_MARKER("pfxSolveConstraints");

	PfxParallelGroup *contactgroup = (PfxParallelGroup*)taskManager->allocate(sizeof(PfxParallelGroup));
	PfxParallelBatch *contactbatches = (PfxParallelBatch*)taskManager->allocate(sizeof(PfxParallelBatch) * (SCE_PFX_MAX_SOLVER_PHASES * SCE_PFX_MAX_SOLVER_BATCHES));
	PfxParallelGroup *jointgroup = (PfxParallelGroup*)taskManager->allocate(sizeof(PfxParallelGroup));
	PfxParallelBatch *jointbatches = (PfxParallelBatch*)taskManager->allocate(sizeof(PfxParallelBatch) * (SCE_PFX_MAX_SOLVER_PHASES * SCE_PFX_MAX_SOLVER_BATCHES));

	// split constraints into independent phases and batches. Phases allow
	// a set of non-independent constraints to be solved in parallel. One
	// phase may have dependencies within another phase, but the phases are
	// solved sequentially. Within a phases, there are multiple batches, and
	// the batches are independent. Since they are independent, they can be
	// distributed to different processors and solved in parallel.
	pfxSplitConstraints(param.numRigidBodies, param.contactPairs, param.numContactPairs,
						taskManager, contactgroup, contactbatches);
	pfxSplitConstraints(param.numRigidBodies, param.jointPairs, param.numJointPairs,
						taskManager, jointgroup, jointbatches);

	// parallel solve
	taskManager->setTaskEntry((void*)pfxSolveConstraintsTaskEntry);
	int task = 0;
	for (; task < taskManager->getNumTasks(); task++)
	{
		taskManager->startTask(task, static_cast<void*>(&param), (PfxUInt32)jointgroup, (PfxUInt32)jointbatches,
								(PfxUInt32)contactgroup, (PfxUInt32)contactbatches);
	}

	// wait for tasks to complete
	PfxUInt32 data1, data2, data3, data4;
	for (PfxUInt32 i = 0; i < taskManager->getNumTasks(); i++)
		taskManager->waitTask(task, data1, data2, data3, data4);

	// post solve
	PfxRigidState *offsetRigidStates = param.offsetRigidStates;
	PfxSolverBody *offsetSolverBodies = param.offsetSolverBodies;
	for (PfxUInt32 i = 0; i < param.numRigidBodies; i++)
	{
		param.offsetRigidStates[i].setLinearVelocity(param.offsetRigidStates[i].getLinearVelocity() +
														param.offsetSolverBodies[i].m_deltaLinearVelocity);
		param.offsetRigidStates[i].setAngularVelocity(param.offsetRigidStates[i].getAngularVelocity() +
														param.offsetSolverBodies[i].m_deltaAngularVelocity);
	}

	taskManager->clearPool();

	SCE_PFX_POP_MARKER();

	return SCE_PFX_OK;
}

} //namespace PhysicsEffects
} //namespace sce
