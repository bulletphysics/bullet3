/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2018 Google Inc. http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "btMultiBodyBlockConstraintSolver.h"

#include <string.h>

#include "LinearMath/btQuickprof.h"
#include "btMultiBodyMLCPConstraintSolver.h"
#include "BulletDynamics/MLCPSolvers/btDantzigSolver.h"

static void rigidBodyNotSupported()
{
	printf("Attempts to use rigid body for block solver, which is not supported yet.\n");
	btAssert(false);
}

btMultiBodyConstraintBlock::btMultiBodyConstraintBlock()
	: m_constraintConfigId(-1)
{
	// Do nothing
}

btMultiBodyConstraintBlock::btMultiBodyConstraintBlock(
	btTypedConstraint** m_constraints,
	int m_numConstraints,
	btAlignedObjectArray<btSolverBody>* m_solverBodyPool,
	btConstraintArray& m_nonContactConstraints,
	btConstraintArray& m_normalContactConstraints,
	btConstraintArray& m_frictionContactConstraints,
	btConstraintArray& m_rollingFrictionContactConstraints,
	btMultiBodyConstraint** m_multiBodyConstraints,
	int m_numMultiBodyConstraints,
	btAlignedObjectArray<btMultiBodySolverConstraint>& m_multiBodyNonContactConstraints,
	btAlignedObjectArray<btMultiBodySolverConstraint>& m_multiBodyNormalContactConstraints,
	btAlignedObjectArray<btMultiBodySolverConstraint>& m_multiBodyFrictionContactConstraints,
	btAlignedObjectArray<btMultiBodySolverConstraint>& m_multiBodyTorsionalFrictionContactConstraints,
	btMultiBodyJacobianData* m_data)
	: m_constraintConfigId(-1)
{
	// Do nothing
}

static void copyConstraintDynamicDataToBlock(btAlignedObjectArray<btMultiBodySolverConstraint*>& originalConstraints, const btAlignedObjectArray<btMultiBodySolverConstraint>& blockConstraints)
{
	btAssert(originalConstraints.size() == blockConstraints.size());
	for (int i = 0; i < blockConstraints.size(); ++i)
	{
		btMultiBodySolverConstraint& originalConstraint = *originalConstraints[i];
		const btMultiBodySolverConstraint& blockConstraint = blockConstraints[i];

		blockConstraint.m_appliedImpulse = originalConstraint.m_appliedImpulse;
		blockConstraint.m_appliedPushImpulse = originalConstraint.m_appliedPushImpulse;
	}
}

void debugPrint(const btScalar* data, int size)
{
	for (int i = 0; i < size; ++i)
	{
		printf("\t%.5f", data[i]);
		if (i != size - 1)
			printf(",");
	}
	printf("\n");
}

void debugPrintDiff(const btScalar* dataA, const btScalar* dataB, int size, bool ignoreZero = false)
{
	for (int i = 0; i < size; ++i)
	{
		if (ignoreZero)
		{
			if (btFabs(dataA[i] - dataB[i]) < 1e-9)
				continue;
		}
		printf("\t%f", dataA[i] - dataB[i]);
		if (i != size - 1)
			printf(",");
	}
	printf("\n");
}

void btMultiBodyConstraintBlock::copyDynamicDataFromOriginalToBlock()
{
	copyConstraintDynamicDataToBlock(m_originalMultiBodyNormalContactConstraintPtrs, m_internalData.m_multiBodyNormalContactConstraints);
	copyConstraintDynamicDataToBlock(m_originalMultiBodyFrictionContactConstraintPtrs, m_internalData.m_multiBodyFrictionContactConstraints);
	copyConstraintDynamicDataToBlock(m_originalMultiBodyTorsionalFrictionContactConstraintPtrs, m_internalData.m_multiBodyTorsionalFrictionContactConstraints);

	btAssert(m_multiBodies.size() == m_originalDeltaVelIndices.size());
	btAssert(m_multiBodies.size() == m_deltaVelIndices.size());
	for (int i = 0; i < m_multiBodies.size(); ++i)
	{
		btMultiBody* multiBody = m_multiBodies[i];
		const int ndof = multiBody->getNumDofs() + 6;

		btMultiBodyJacobianData& originalData = m_internalData.m_data;  // TODO(JS): WRONG !!
		btAlignedObjectArray<btScalar>& originaDeltaVelocities = originalData.m_deltaVelocities;

		btAlignedObjectArray<btScalar>& blockDeltaVelocities = m_internalData.m_data.m_deltaVelocities;

		const int originalIndex = m_originalDeltaVelIndices[i];
		const int blockIndex = m_deltaVelIndices[i];

		const btScalar* originalDeltaVelocitiesPtr = &originaDeltaVelocities[originalIndex];
		btScalar* blockDeltaVelocitiesPtr = &blockDeltaVelocities[blockIndex];

		//		printf("[ original --> block ]\n");
		//		printf("original: ");
		//		debugPrint(originalDeltaVelocitiesPtr, ndof);
		//		printf("block: ");
		//		debugPrint(blockDeltaVelocitiesPtr, ndof);
		//		printf("diff: ");
		//		debugPrintDiff(originalDeltaVelocitiesPtr, blockDeltaVelocitiesPtr, ndof, true);
		//		printf("\n");

		memcpy(blockDeltaVelocitiesPtr, originalDeltaVelocitiesPtr, ndof * sizeof(btScalar));
	}
}

static void copyConstraintDynamicDataFromToOriginal(btAlignedObjectArray<btMultiBodySolverConstraint*>& originalConstraints, const btAlignedObjectArray<btMultiBodySolverConstraint>& blockConstraints)
{
	btAssert(originalConstraints.size() == blockConstraints.size());
	for (int i = 0; i < blockConstraints.size(); ++i)
	{
		btMultiBodySolverConstraint& originalConstraint = *originalConstraints[i];
		const btMultiBodySolverConstraint& blockConstraint = blockConstraints[i];

		originalConstraint.m_appliedImpulse = blockConstraint.m_appliedImpulse;
		originalConstraint.m_appliedPushImpulse = blockConstraint.m_appliedPushImpulse;
	}
}

void btMultiBodyConstraintBlock::copyDynamicDataFromBlockToOriginal()
{
	copyConstraintDynamicDataFromToOriginal(m_originalMultiBodyNormalContactConstraintPtrs, m_internalData.m_multiBodyNormalContactConstraints);
	copyConstraintDynamicDataFromToOriginal(m_originalMultiBodyFrictionContactConstraintPtrs, m_internalData.m_multiBodyFrictionContactConstraints);
	copyConstraintDynamicDataFromToOriginal(m_originalMultiBodyTorsionalFrictionContactConstraintPtrs, m_internalData.m_multiBodyTorsionalFrictionContactConstraints);

	btAssert(m_multiBodies.size() == m_originalDeltaVelIndices.size());
	btAssert(m_multiBodies.size() == m_deltaVelIndices.size());
	for (int i = 0; i < m_multiBodies.size(); ++i)
	{
		btMultiBody* multiBody = m_multiBodies[i];
		const int ndof = multiBody->getNumDofs() + 6;

		btMultiBodyJacobianData& originalData = m_internalData.m_data;
		btAlignedObjectArray<btScalar>& originaDeltaVelocities = originalData.m_deltaVelocities;

		btAlignedObjectArray<btScalar>& blockDeltaVelocities = m_internalData.m_data.m_deltaVelocities;

		const int originalIndex = m_originalDeltaVelIndices[i];
		const int blockIndex = m_deltaVelIndices[i];

		btScalar* originalDeltaVelocitiesPtr = &originaDeltaVelocities[originalIndex];
		const btScalar* blockDeltaVelocitiesPtr = &blockDeltaVelocities[blockIndex];

		//		printf("[ block --> original ]\n");
		//		printf("original: ");
		//		debugPrint(originalDeltaVelocitiesPtr, ndof);
		//		printf("block: ");
		//		debugPrint(blockDeltaVelocitiesPtr, ndof);
		//				printf("diff: ");
		//				debugPrintDiff(originalDeltaVelocitiesPtr, blockDeltaVelocitiesPtr, ndof, true);
		//		printf("\n");

		memcpy(originalDeltaVelocitiesPtr, blockDeltaVelocitiesPtr, ndof * sizeof(btScalar));
	}
}

btSingleBlockSplittingPolicy::btSingleBlockSplittingPolicy(btMultiBodyConstraintSolver* solver)
	: m_solver(solver)
{
	// Do nothing
}

btSingleBlockSplittingPolicy::~btSingleBlockSplittingPolicy()
{
	// Do nothing
}

void btSingleBlockSplittingPolicy::split(btMultiBodyConstraintSolver::btMultiBodyInternalConstraintData& blockInput, const btAlignedObjectArray<btBlockConstraintSolverConfig>& availableConfigs, btAlignedObjectArray<btMultiBodyConstraintBlock>& blocksOutput)
{
	btMultiBodyConstraintBlock newBlock;
	//	newBlock.m_originalInternalDataBlock = blockInput;
	//	m_solver->setMultiBodyInternalConstraintData(newBlock.m_originalInternalDataBlock);
	newBlock.m_solver = m_solver;
	//	newBlock.m_constraints = blockInput.m_constraints;
	//	newBlock.m_numConstraints = blockInput.m_numConstraints;
	//	newBlock.m_multiBodyConstraints = blockInput.m_multiBodyConstraints;
	//	newBlock.m_numMultiBodyConstraints = blockInput.m_numMultiBodyConstraints;

	blocksOutput.push_back(newBlock);
}

btDoubleBlockSplittingPolicy::btDoubleBlockSplittingPolicy(btMultiBodyConstraintSolver* solver)
	: m_solver(solver)
{
	// Do nothing
}

btDoubleBlockSplittingPolicy::~btDoubleBlockSplittingPolicy()
{
	// Do nothing
}

template <typename ArrayT>
void splitContactConstraints(const ArrayT& input, ArrayT& output1, ArrayT& output2)
{
	const int totalSize = input.size();
	const int halfSize = totalSize / 2;

	output1.resize(halfSize);
	output2.resize(totalSize - halfSize);

	for (int i = 0; i < halfSize; ++i)
	{
		output1[i] = input[i];
	}

	for (int i = halfSize; i < totalSize; ++i)
	{
		output2[i - halfSize] = input[i];
	}
}

btMultiBodyConstraintBlock initializeConstraintBlock(btMultiBodyConstraintSolver::btMultiBodyInternalConstraintData& input)
{
	btMultiBodyConstraintBlock output;

	// MultiBody

	output.m_internalData.m_multiBodyConstraints = input.m_multiBodyConstraints;
	output.m_internalData.m_numMultiBodyConstraints = input.m_numMultiBodyConstraints;
	//output.m_multiBodyConstraintSet.m_data = input.m_multiBodyConstraintSet.m_data;

	btAssert(output.m_internalData.m_multiBodyNormalContactConstraints.size() == 0);
	btAssert(output.m_internalData.m_multiBodyFrictionContactConstraints.size() == 0);

	return output;
}

static void setupBlockMultiBodyJacobianData(
	btMultiBody* multiBody,
	btAlignedObjectArray<btMultiBody*>& multiBodySet,
	btAlignedObjectArray<int>& blockDeltaVelIndices,
	int& blockDeltaVelIndex,
	int& blockJacobianIndex,
	btMultiBodyJacobianData& blockJacobianData,
	btAlignedObjectArray<int>& originalDeltaVelIndices,
	const int originalDeltaVelIndex,
	const int originalJacobianIndex,
	const btMultiBodyJacobianData& originalJacobianData)
{
	const int ndof = multiBody->getNumDofs() + 6;

	btAlignedObjectArray<btScalar>& blockJacobians = blockJacobianData.m_jacobians;
	btAlignedObjectArray<btScalar>& blockDeltaVelocities = blockJacobianData.m_deltaVelocities;
	btAlignedObjectArray<btScalar>& blockDeltaVelocitiesUnitImpulse = blockJacobianData.m_deltaVelocitiesUnitImpulse;

	const btAlignedObjectArray<btScalar>& originalJacobians = originalJacobianData.m_jacobians;
	const btAlignedObjectArray<btScalar>& originalDeltaVelocitiesUnitImpulse = originalJacobianData.m_deltaVelocitiesUnitImpulse;

	int indexInBlock = -1;
	for (int i = 0; i < multiBodySet.size(); ++i)
	{
		if (multiBody == multiBodySet[i])
		{
			indexInBlock = i;
			break;
		}
	}

	if (indexInBlock == -1)
	{
		blockDeltaVelIndex = blockDeltaVelocities.size();
		blockDeltaVelocities.resize(blockDeltaVelocities.size() + ndof);
		multiBodySet.push_back(multiBody);
		originalDeltaVelIndices.push_back(originalDeltaVelIndex);
		blockDeltaVelIndices.push_back(blockDeltaVelIndex);
	}
	else
	{
		blockDeltaVelIndex = blockDeltaVelIndices[indexInBlock];
	}

	blockJacobianIndex = blockJacobians.size();
	blockJacobians.resize(blockJacobians.size() + ndof);
	blockDeltaVelocitiesUnitImpulse.resize(blockDeltaVelocitiesUnitImpulse.size() + ndof);
	btAssert(blockJacobians.size() == blockDeltaVelocitiesUnitImpulse.size());

	btScalar* blockJacobiansRawPtr = &blockJacobians[blockJacobianIndex];
	const btScalar* originalJacobiansRawPtr = &originalJacobians[originalJacobianIndex];
	memcpy(blockJacobiansRawPtr, originalJacobiansRawPtr, ndof * sizeof(btScalar));

	btScalar* blockDeltaVelUnitImp = &blockDeltaVelocitiesUnitImpulse[blockJacobianIndex];
	const btScalar* originalDeltaVelUnitImp = &originalDeltaVelocitiesUnitImpulse[originalJacobianIndex];
	memcpy(blockDeltaVelUnitImp, originalDeltaVelUnitImp, ndof * sizeof(btScalar));

	btAssert(blockJacobians.size() >= blockDeltaVelocities.size());
}

static void setupMultiBodyBlockConstraintData(
	btMultiBodyConstraintBlock& block,
	btAlignedObjectArray<btMultiBody*>& blockMultiBodySet,
	btAlignedObjectArray<int>& blockDeltaVelIndices,
	btMultiBodySolverConstraint& blockContactConstraint,
	btMultiBodyJacobianData& blockJacobianData,
	int blockFrictionIndex,
	btMultiBodyConstraintSolver::btMultiBodyInternalConstraintData& originalInternalData,
	btAlignedObjectArray<int>& originalDeltaVelIndices,
	const btMultiBodySolverConstraint& originalContactConstraint,
	const btMultiBodyJacobianData& originalJacobianData)
{
	// Copy all the values. Some values will be updated below if necessary.
	blockContactConstraint = originalContactConstraint;
	blockContactConstraint.m_frictionIndex = blockFrictionIndex;

	btMultiBody* multiBodyA = blockContactConstraint.m_multiBodyA;
	if (multiBodyA)
	{
		setupBlockMultiBodyJacobianData(
			multiBodyA,
			blockMultiBodySet,
			blockDeltaVelIndices,
			blockContactConstraint.m_deltaVelAindex,
			blockContactConstraint.m_jacAindex,
			blockJacobianData,
			originalDeltaVelIndices,
			originalContactConstraint.m_deltaVelAindex,
			originalContactConstraint.m_jacAindex,
			originalJacobianData);
	}
	else
	{
		rigidBodyNotSupported();
	}

	btMultiBody* multiBodyB = blockContactConstraint.m_multiBodyB;
	if (multiBodyB)
	{
		setupBlockMultiBodyJacobianData(
			multiBodyB,
			blockMultiBodySet,
			blockDeltaVelIndices,
			blockContactConstraint.m_deltaVelBindex,
			blockContactConstraint.m_jacBindex,
			blockJacobianData,
			originalDeltaVelIndices,
			originalContactConstraint.m_deltaVelBindex,
			originalContactConstraint.m_jacBindex,
			originalJacobianData);
	}
	else
	{
		rigidBodyNotSupported();
	}
}

void btDoubleBlockSplittingPolicy::split(
	btMultiBodyConstraintSolver::btMultiBodyInternalConstraintData& originalInternalData,
	const btAlignedObjectArray<btBlockConstraintSolverConfig>& availableConfigs,
	btAlignedObjectArray<btMultiBodyConstraintBlock>& subBlocks)
{
	btMultiBodyConstraintBlock constraintBlock1 = initializeConstraintBlock(originalInternalData);
	btMultiBodyConstraintBlock constraintBlock2 = initializeConstraintBlock(originalInternalData);
	btMultiBodyConstraintBlock constraintBlock3 = initializeConstraintBlock(originalInternalData);

	constraintBlock1.m_solver = m_solver;
	constraintBlock2.m_solver = m_solver;
	constraintBlock3.m_solver = m_solver;

	btDantzigSolver* mlcp = new btDantzigSolver();
	btMultiBodyMLCPConstraintSolver* sol = new btMultiBodyMLCPConstraintSolver(mlcp);
//	constraintBlock2.m_solver = sol;

	const int totalMultiBodyContactConstraintSize = originalInternalData.m_multiBodyNormalContactConstraints.size();
	const int halfMultiBodyContactConstraintSize = totalMultiBodyContactConstraintSize / 2;

	for (int i = 0; i < halfMultiBodyContactConstraintSize; ++i)
	{
		copyMultiBodyContactConstraint(constraintBlock1, originalInternalData, i);
	}

	for (int i = halfMultiBodyContactConstraintSize; i < totalMultiBodyContactConstraintSize; ++i)
	{
		copyMultiBodyContactConstraint(constraintBlock2, originalInternalData, i);
	}

	const int totalMultiBodyNonContactConstraintSize = originalInternalData.m_multiBodyNonContactConstraints.size();

	for (int i = 0; i < totalMultiBodyNonContactConstraintSize; ++i)
	{
//		copyMultiBodyNonContactConstraint(constraintBlock3, originalInternalData, i);
	}


//	for (int i = 0; i < totalMultiBodyContactConstraintSize; ++i)
//	{
//		if (strcmp(originalInternalData.m_multiBodyNormalContactConstraints[i].m_multiBodyA->getBaseName(), "group1") == 0)
//			copyMultiBodyContactConstraint(constraintBlock1, originalInternalData, i);
//		else
//			copyMultiBodyContactConstraint(constraintBlock2, originalInternalData, i);
//	}

	subBlocks.push_back(constraintBlock1);
	subBlocks.push_back(constraintBlock2);
	subBlocks.push_back(constraintBlock3);
}

btMultiBodyBlockSplittingPolicy::~btMultiBodyBlockSplittingPolicy()
{
	// Do nothing
}

void btMultiBodyBlockSplittingPolicy::copyMultiBodyNonContactConstraint(btMultiBodyConstraintBlock& block, btMultiBodyConstraintSolver::btMultiBodyInternalConstraintData& originalInternalData, int originalNonContactConstraintIndex)
{
	btAlignedObjectArray<btMultiBodySolverConstraint>& originalNonContactConstraints = originalInternalData.m_multiBodyNonContactConstraints;
	const btMultiBodyJacobianData& originalJacobianData = originalInternalData.m_data;

	btMultiBodyConstraintSolver::btMultiBodyInternalConstraintData& blockInternalData = block.m_internalData;
	btAlignedObjectArray<btMultiBodySolverConstraint>& blockNonContactConstraints = blockInternalData.m_multiBodyNonContactConstraints;

	btAlignedObjectArray<btMultiBodySolverConstraint*>& blockOriginalNonContactConstraintPtrs = block.m_originalMultiBodyNonContactConstraintPtrs;

	btMultiBodyJacobianData& blockJacobianData = block.m_internalData.m_data;

	btAlignedObjectArray<btMultiBody*>& blockMultiBodySet = block.m_multiBodies;

	const int blockFrictionIndex = blockNonContactConstraints.size();

	btMultiBodySolverConstraint& originalNonContactConstraint = originalNonContactConstraints[originalNonContactConstraintIndex];

	btMultiBodySolverConstraint& blockNonContactConstraint = blockNonContactConstraints.expandNonInitializing();
	blockOriginalNonContactConstraintPtrs.push_back(&originalNonContactConstraint);

	setupMultiBodyBlockConstraintData(
		block,
		blockMultiBodySet,
		block.m_deltaVelIndices,
		blockNonContactConstraint,
		blockJacobianData,
		blockFrictionIndex,
		originalInternalData,
		block.m_originalDeltaVelIndices,
		originalNonContactConstraint,
		originalJacobianData);
}

void btMultiBodyBlockSplittingPolicy::copyMultiBodyContactConstraint(btMultiBodyConstraintBlock& block, btMultiBodyConstraintSolver::btMultiBodyInternalConstraintData& originalInternalData, int originalNormalContactConstraintIndex)
{
	btAlignedObjectArray<btMultiBodySolverConstraint>& originalNormalContactConstraints = originalInternalData.m_multiBodyNormalContactConstraints;
	btAlignedObjectArray<btMultiBodySolverConstraint>& originalFrictionContactConstraints = originalInternalData.m_multiBodyFrictionContactConstraints;
	btAlignedObjectArray<btMultiBodySolverConstraint>& originalTortionalFrictionContactConstraints = originalInternalData.m_multiBodyTorsionalFrictionContactConstraints;
	const btMultiBodyJacobianData& originalJacobianData = originalInternalData.m_data;

	btMultiBodyConstraintSolver::btMultiBodyInternalConstraintData& blockInternalData = block.m_internalData;
	btAlignedObjectArray<btMultiBodySolverConstraint>& blockNormalContactConstraints = blockInternalData.m_multiBodyNormalContactConstraints;
	btAlignedObjectArray<btMultiBodySolverConstraint>& blockFrictionContactConstraints = blockInternalData.m_multiBodyFrictionContactConstraints;
	btAlignedObjectArray<btMultiBodySolverConstraint>& blockTortionalFrictionContactConstraints = blockInternalData.m_multiBodyTorsionalFrictionContactConstraints;

	btAlignedObjectArray<btMultiBodySolverConstraint*>& blockOriginalNormalContactConstraintPtrs = block.m_originalMultiBodyNormalContactConstraintPtrs;
	btAlignedObjectArray<btMultiBodySolverConstraint*>& blockOriginalFrictionContactConstraintPtrs = block.m_originalMultiBodyFrictionContactConstraintPtrs;
	btAlignedObjectArray<btMultiBodySolverConstraint*>& blockOriginalTorsionalFrictionContactConstraintPtrs = block.m_originalMultiBodyTorsionalFrictionContactConstraintPtrs;

	btMultiBodyJacobianData& blockJacobianData = block.m_internalData.m_data;

	const int numFrictionPerContact = originalNormalContactConstraints.size() == originalFrictionContactConstraints.size() ? 1 : 2;

	btAlignedObjectArray<btMultiBody*>& blockMultiBodySet = block.m_multiBodies;

	const int blockFrictionIndex = blockNormalContactConstraints.size();

	//-- 1. Normal contact

	btMultiBodySolverConstraint& originalNormalContactConstraint = originalNormalContactConstraints[originalNormalContactConstraintIndex];

	btMultiBodySolverConstraint& blockNormalContactConstraint = blockNormalContactConstraints.expandNonInitializing();
	blockOriginalNormalContactConstraintPtrs.push_back(&originalNormalContactConstraint);

	setupMultiBodyBlockConstraintData(
		block,
		blockMultiBodySet,
		block.m_deltaVelIndices,
		blockNormalContactConstraint,
		blockJacobianData,
		blockFrictionIndex,
		originalInternalData,
		block.m_originalDeltaVelIndices,
		originalNormalContactConstraint,
		originalJacobianData);

	//-- 2. Friction contacts

	btAssert(originalFrictionContactConstraints.size() != 0);
	const int originalFrictionContactConstraintIndex1 = originalNormalContactConstraintIndex * numFrictionPerContact;
	btMultiBodySolverConstraint& originalFrictionContactConstraint = originalFrictionContactConstraints[originalFrictionContactConstraintIndex1];

	blockOriginalFrictionContactConstraintPtrs.push_back(&originalFrictionContactConstraint);
	btMultiBodySolverConstraint& blockFrictionContactConstraint1 = blockFrictionContactConstraints.expandNonInitializing();
	setupMultiBodyBlockConstraintData(
		block,
		blockMultiBodySet,
		block.m_deltaVelIndices,
		blockFrictionContactConstraint1,
		blockJacobianData,
		blockFrictionIndex,
		originalInternalData,
		block.m_originalDeltaVelIndices,
		originalFrictionContactConstraint,
		originalJacobianData);

	if (numFrictionPerContact == 2)
	{
		const int originalFrictionContactConstraintIndex2 = originalFrictionContactConstraintIndex1 + 1;
		btMultiBodySolverConstraint& originalFrictionContactConstraint = originalFrictionContactConstraints[originalFrictionContactConstraintIndex2];

		blockOriginalFrictionContactConstraintPtrs.push_back(&originalFrictionContactConstraint);
		btMultiBodySolverConstraint& blockFrictionContactConstraint2 = blockFrictionContactConstraints.expandNonInitializing();
		setupMultiBodyBlockConstraintData(
			block,
			blockMultiBodySet,
			block.m_deltaVelIndices,
			blockFrictionContactConstraint2,
			blockJacobianData,
			blockFrictionIndex,
			originalInternalData,
			block.m_originalDeltaVelIndices,
			originalFrictionContactConstraint,
			originalJacobianData);
	}

	// TODO(JS): Torsional friction contact constraints
}

btMultiBodyBlockConstraintSolver::btMultiBodyBlockConstraintSolver()
{
	// Do nothing
}

btMultiBodyBlockConstraintSolver::~btMultiBodyBlockConstraintSolver()
{
	// Do nothing
}

btScalar btMultiBodyBlockConstraintSolver::solveGroupConvertConstraintPoststep(btCollisionObject** bodies, int numBodies, btPersistentManifold** manifoldPtr, int numManifolds, btTypedConstraint** constraints, int numConstraints, const btContactSolverInfo& infoGlobal, btIDebugDraw* debugDrawer)
{
	return btMultiBodyConstraintSolver::solveGroupConvertConstraintPoststep(bodies, numBodies, manifoldPtr, numManifolds, constraints, numConstraints, infoGlobal, debugDrawer);
}

void btMultiBodyBlockConstraintSolver::solveMultiBodyGroup(
	btCollisionObject** bodies,
	int numBodies,
	btPersistentManifold** manifold,
	int numManifolds,
	btTypedConstraint** constraints,
	int numConstraints,
	btMultiBodyConstraint** multiBodyConstraints,
	int numMultiBodyConstraints,
	const btContactSolverInfo& info,
	btIDebugDraw* debugDrawer,
	btDispatcher* /*dispatcher*/)
{
	m_tmpMultiBodyConstraints = multiBodyConstraints;
	m_tmpNumMultiBodyConstraints = numMultiBodyConstraints;

	// 1. Convert rigid bodies/multibodies, joints, contacts into constraints.
	solveGroupCacheFriendlySetup(bodies, numBodies, manifold, numManifolds, constraints, numConstraints, info, debugDrawer);

	// 2. Split constraints into constraint blocks
	btMultiBodyInternalConstraintData originalInternalDataCopy;
	getMultiBodyInternalConstraintData(originalInternalDataCopy);

	btAlignedObjectArray<btBlockConstraintSolverConfig> configs;
	// TODO(JS): This is just for test
	//m_splittingPolicy = new btSingleBlockSplittingPolicy(new btMultiBodyConstraintSolver());

//	btDantzigSolver* mlcp = new btDantzigSolver();
//	btMultiBodyMLCPConstraintSolver* sol = new btMultiBodyMLCPConstraintSolver(mlcp);
//	m_splittingPolicy = new btDoubleBlockSplittingPolicy(sol);

	m_splittingPolicy = new btDoubleBlockSplittingPolicy(new btMultiBodyConstraintSolver());

    btAssert(m_splittingPolicy);
	m_blocks.resize(0);
	m_splittingPolicy->split(originalInternalDataCopy, configs, m_blocks);

//	for (int i = 0; i < m_blocks.size(); ++i)
//	{
//		btMultiBodyConstraintBlock& block = m_blocks[i];
//		btMultiBodyConstraintSolver* solver = block.m_solver;
//		btAssert(solver);

//		solver->solveGroupConvertConstraintPrestep(bodies, numBodies, manifold, numManifolds, constraints, numConstraints, info, debugDrawer);
//		copyDynamicDataFromOriginalToBlock(block);
////			block.copyDynamicDataFromOriginalToBlock();
//		solver->setMultiBodyInternalConstraintData(block.m_internalData, false);
//		solver->solveGroupConvertConstraintPoststep(bodies, numBodies, manifold, numManifolds, constraints, numConstraints, info, debugDrawer);
//	}

	// 3. Gauss-Seidel iterations

	const int maxIterations = m_maxOverrideNumSolverIterations > info.m_numIterations ? m_maxOverrideNumSolverIterations : info.m_numIterations;

	m_leastSquaresResidual = 0;

	for (int iteration = 0; iteration < maxIterations; ++iteration)
	{
		for (int i = 0; i < m_blocks.size(); ++i)
		{
			// Change the sweep direction every iteration
			const int index = iteration & 1 ? m_blocks.size() - 1 - i : i;

			btMultiBodyConstraintBlock& block = m_blocks[index];
			btMultiBodyConstraintSolver* solver = block.m_solver;
			btAssert(solver);

			solver->solveGroupConvertConstraintPrestep(bodies, numBodies, manifold, numManifolds, constraints, numConstraints, info, debugDrawer);
			copyDynamicDataFromOriginalToBlock(block);
			solver->setMultiBodyInternalConstraintData(block.m_internalData, false);
			solver->solveGroupConvertConstraintPoststep(bodies, numBodies, manifold, numManifolds, constraints, numConstraints, info, debugDrawer);

			// TODO(JS): Add split impulse
			btScalar newSquaredResidual = solver->solveGroupCacheFriendlyIterations(bodies, numBodies, manifold, numManifolds, constraints, numConstraints, info, debugDrawer);
			m_leastSquaresResidual = btMax(m_leastSquaresResidual, newSquaredResidual);

			solver->solveGroupConvertBackPrestep(bodies, numBodies, info);
			solver->solveGroupConvertBack(bodies, numBodies, info);
			solver->getMultiBodyInternalConstraintData(block.m_internalData, false);
			copyDynamicDataFromBlockToOriginal(block);
			solver->solveGroupConvertBackPoststep(bodies, numBodies, info);
		}

		if (m_leastSquaresResidual <= info.m_leastSquaresResidualThreshold || (iteration >= (maxIterations - 1)))
		{
#ifdef VERBOSE_RESIDUAL_PRINTF
			printf("residual = %f at iteration #%d\n", m_leastSquaresResidual, iteration);
#endif
			break;
		}
	}

//	solveGroupConvertBackPrestep(bodies, numBodies, info);
//	solveGroupConvertBack(bodies, numBodies, info);
//	getMultiBodyInternalConstraintData(block.m_internalData, false);
//	copyDynamicDataFromBlockToOriginal(block);
//	solveGroupConvertBackPoststep(bodies, numBodies, info);

	solveGroupCacheFriendlyFinish(bodies, numBodies, info);

	m_tmpMultiBodyConstraints = 0;
	m_tmpNumMultiBodyConstraints = 0;
}

void btMultiBodyBlockConstraintSolver::setSplittingPolicy(btMultiBodyBlockSplittingPolicy* policy)
{
	m_splittingPolicy = policy;
}

void btMultiBodyBlockConstraintSolver::copyDynamicDataFromOriginalToBlock(btMultiBodyConstraintBlock& block)
{
	copyConstraintDynamicDataToBlock(block.m_originalMultiBodyNormalContactConstraintPtrs, block.m_internalData.m_multiBodyNormalContactConstraints);
	copyConstraintDynamicDataToBlock(block.m_originalMultiBodyFrictionContactConstraintPtrs, block.m_internalData.m_multiBodyFrictionContactConstraints);
	copyConstraintDynamicDataToBlock(block.m_originalMultiBodyTorsionalFrictionContactConstraintPtrs, block.m_internalData.m_multiBodyTorsionalFrictionContactConstraints);

	btAssert(block.m_multiBodies.size() == block.m_originalDeltaVelIndices.size());
	btAssert(block.m_multiBodies.size() == block.m_deltaVelIndices.size());
	for (int i = 0; i < block.m_multiBodies.size(); ++i)
	{
		btMultiBody* multiBody = block.m_multiBodies[i];
		const int ndof = multiBody->getNumDofs() + 6;

		btMultiBodyJacobianData& originalData = m_data;  // TODO(JS): WRONG !!
		btAlignedObjectArray<btScalar>& originaDeltaVelocities = originalData.m_deltaVelocities;

		btAlignedObjectArray<btScalar>& blockDeltaVelocities = block.m_internalData.m_data.m_deltaVelocities;

		const int originalIndex = block.m_originalDeltaVelIndices[i];
		const int blockIndex = block.m_deltaVelIndices[i];

		const btScalar* originalDeltaVelocitiesPtr = &originaDeltaVelocities[originalIndex];
		btScalar* blockDeltaVelocitiesPtr = &blockDeltaVelocities[blockIndex];

		//		printf("[ original --> block ]\n");
		//		printf("original: ");
		//		debugPrint(originalDeltaVelocitiesPtr, ndof);
		//		printf("block: ");
		//		debugPrint(blockDeltaVelocitiesPtr, ndof);
		//		printf("diff: ");
		//		debugPrintDiff(originalDeltaVelocitiesPtr, blockDeltaVelocitiesPtr, ndof, true);
		//		printf("\n");

		memcpy(blockDeltaVelocitiesPtr, originalDeltaVelocitiesPtr, ndof * sizeof(btScalar));
	}
}

void btMultiBodyBlockConstraintSolver::copyDynamicDataFromBlockToOriginal(btMultiBodyConstraintBlock& block)
{
	copyConstraintDynamicDataFromToOriginal(block.m_originalMultiBodyNormalContactConstraintPtrs, block.m_internalData.m_multiBodyNormalContactConstraints);
	copyConstraintDynamicDataFromToOriginal(block.m_originalMultiBodyFrictionContactConstraintPtrs, block.m_internalData.m_multiBodyFrictionContactConstraints);
	copyConstraintDynamicDataFromToOriginal(block.m_originalMultiBodyTorsionalFrictionContactConstraintPtrs, block.m_internalData.m_multiBodyTorsionalFrictionContactConstraints);

	btAssert(block.m_multiBodies.size() == block.m_originalDeltaVelIndices.size());
	btAssert(block.m_multiBodies.size() == block.m_deltaVelIndices.size());
	for (int i = 0; i < block.m_multiBodies.size(); ++i)
	{
		btMultiBody* multiBody = block.m_multiBodies[i];
		const int ndof = multiBody->getNumDofs() + 6;

		btMultiBodyJacobianData& originalData = m_data;
		btAlignedObjectArray<btScalar>& originaDeltaVelocities = originalData.m_deltaVelocities;

		btAlignedObjectArray<btScalar>& blockDeltaVelocities = block.m_internalData.m_data.m_deltaVelocities;

		const int originalIndex = block.m_originalDeltaVelIndices[i];
		const int blockIndex = block.m_deltaVelIndices[i];

		btScalar* originalDeltaVelocitiesPtr = &originaDeltaVelocities[originalIndex];
		const btScalar* blockDeltaVelocitiesPtr = &blockDeltaVelocities[blockIndex];

		//		printf("[ block --> original ]\n");
		//		printf("original: ");
		//		debugPrint(originalDeltaVelocitiesPtr, ndof);
		//		printf("block: ");
		//		debugPrint(blockDeltaVelocitiesPtr, ndof);
		//				printf("diff: ");
		//				debugPrintDiff(originalDeltaVelocitiesPtr, blockDeltaVelocitiesPtr, ndof, true);
		//		printf("\n");

		memcpy(originalDeltaVelocitiesPtr, blockDeltaVelocitiesPtr, ndof * sizeof(btScalar));
	}
}

int btMultiBodyBlockConstraintSolver::addConfig(btBlockConstraintSolverConfig& config)
{
	m_configs.push_back(config);
	return m_configs.size();
}

int btMultiBodyBlockConstraintSolver::getNumConfigs() const
{
	return m_configs.size();
}

void btMultiBodyBlockConstraintSolver::removeConfig(int configIndex)
{
	m_configs.removeAtIndex(configIndex);
}
