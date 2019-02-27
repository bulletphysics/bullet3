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

#ifndef BT_MULTIBODY_BLOCK_CONSTRAINT_SOLVER_H
#define BT_MULTIBODY_BLOCK_CONSTRAINT_SOLVER_H

#include "BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h"

struct btBlockConstraintSolverConfig
{
	int m_solverType;  //SI or MLCP Dantzig
	//to be decided: full or subset of

	btContactSolverInfo m_info;
};

struct btMultiBodyConstraintBlock
{
	/// \{ \name Multi-body Data

	btAlignedObjectArray<btMultiBody*> m_multiBodies;
	btAlignedObjectArray<int> m_originalDeltaVelIndices;
	btAlignedObjectArray<int> m_deltaVelIndices;

//	btMultiBodyJacobianData* m_originalDataPtr;

	btAlignedObjectArray<btMultiBodySolverConstraint*> m_originalMultiBodyNonContactConstraintPtrs;
	btAlignedObjectArray<btMultiBodySolverConstraint*> m_originalMultiBodyNormalContactConstraintPtrs;
	btAlignedObjectArray<btMultiBodySolverConstraint*> m_originalMultiBodyFrictionContactConstraintPtrs;
	btAlignedObjectArray<btMultiBodySolverConstraint*> m_originalMultiBodyTorsionalFrictionContactConstraintPtrs;

	/// \}

	btMultiBodyConstraintSolver::btMultiBodyInternalConstraintData m_internalData;

	/// Constraint solver
	btMultiBodyConstraintSolver* m_solver;

	bool m_ownSolver = false;
	// TODO(JS): If this is true, then don't copy all the constraint data, but
	// only dynamic data
	// TODO(JS): not utilized yet

	/// Index to constraint solver configuration
	int m_constraintConfigId;

	/// Default constructor
	btMultiBodyConstraintBlock();

	/// Constructor
	btMultiBodyConstraintBlock(
		btTypedConstraint** m_constraints,
		int m_numConstraints,
		btAlignedObjectArray<btSolverBody>* m_solverBodyPool,
		btConstraintArray& m_originalNonContactConstraints,
		btConstraintArray& m_originalNormalContactConstraints,
		btConstraintArray& m_originalFrictionContactConstraints,
		btConstraintArray& m_orginalRollingFrictionContactConstraints,
		btMultiBodyConstraint** m_multiBodyConstraints,
		int m_numMultiBodyConstraints,
		btAlignedObjectArray<btMultiBodySolverConstraint>& m_multiBodyNonContactConstraints,
		btAlignedObjectArray<btMultiBodySolverConstraint>& m_multiBodyNormalContactConstraints,
		btAlignedObjectArray<btMultiBodySolverConstraint>& m_multiBodyFrictionContactConstraints,
		btAlignedObjectArray<btMultiBodySolverConstraint>& m_multiBodyTorsionalFrictionContactConstraints,
		btMultiBodyJacobianData* m_data);

	void copyDynamicDataFromOriginalToBlock();
	void copyDynamicDataFromBlockToOriginal();
};

class btMultiBodyBlockSplittingPolicy
{
public:
	/// Destructor
	virtual ~btMultiBodyBlockSplittingPolicy();

	/// Splits a set of constraints into multiple subsets.
	///
	/// \param[in] blockInput
	/// \param[in] availableConfigs
	/// \param[in,out] blocksOutput The splitted blocks. This function adds blocks without clearning the array
	/// beforehand. Clearning the array is the caller's responsibility.
	virtual void split(btMultiBodyConstraintSolver::btMultiBodyInternalConstraintData& blockInput, const btAlignedObjectArray<btBlockConstraintSolverConfig>& availableConfigs, btAlignedObjectArray<btMultiBodyConstraintBlock>& blocksOutput) = 0;

protected:
	void copyMultiBodyNonContactConstraint(
		btMultiBodyConstraintBlock& block,
		btMultiBodyConstraintSolver::btMultiBodyInternalConstraintData& originalInternalData,
		int originalNonContactConstraintIndex);

	void copyMultiBodyContactConstraint(
		btMultiBodyConstraintBlock& block,
		btMultiBodyConstraintSolver::btMultiBodyInternalConstraintData& originalInternalData,
		int originalNormalContactConstraintIndex);
};

class btSingleBlockSplittingPolicy : public btMultiBodyBlockSplittingPolicy
{
protected:
	btMultiBodyConstraintSolver* m_solver;

public:
	/// Constructor
	btSingleBlockSplittingPolicy(btMultiBodyConstraintSolver* solver);

	/// Destructor
	virtual ~btSingleBlockSplittingPolicy();

	// Documentation inherited
	virtual void split(btMultiBodyConstraintSolver::btMultiBodyInternalConstraintData& blockInput, const btAlignedObjectArray<btBlockConstraintSolverConfig>& availableConfigs, btAlignedObjectArray<btMultiBodyConstraintBlock>& blocksOutput);
};

class btDoubleBlockSplittingPolicy : public btMultiBodyBlockSplittingPolicy
{
protected:
	btMultiBodyConstraintSolver* m_solver;

public:
	/// Constructor
	btDoubleBlockSplittingPolicy(btMultiBodyConstraintSolver* solver);

	/// Destructor
	virtual ~btDoubleBlockSplittingPolicy();

	// Documentation inherited
	virtual void split(btMultiBodyConstraintSolver::btMultiBodyInternalConstraintData& blockInput, const btAlignedObjectArray<btBlockConstraintSolverConfig>& availableConfigs, btAlignedObjectArray<btMultiBodyConstraintBlock>& blocksOutput);
};

class btMultiBodyBlockConstraintSolver : public btMultiBodyConstraintSolver
{
protected:
	/// Splitting policy. Assumed not a null.
	btMultiBodyBlockSplittingPolicy* m_splittingPolicy;

	/// Array of constraint configurations for constraint blocks.
	btAlignedObjectArray<btBlockConstraintSolverConfig> m_configs;

	/// Array of constraint blocks.
	btAlignedObjectArray<btMultiBodyConstraintBlock> m_blocks;

public:
	/// Constructor
	btMultiBodyBlockConstraintSolver();

	/// Destructor
	virtual ~btMultiBodyBlockConstraintSolver();

	virtual btScalar solveGroupConvertConstraintPoststep(btCollisionObject** bodies, int numBodies, btPersistentManifold** manifoldPtr, int numManifolds, btTypedConstraint** constraints, int numConstraints, const btContactSolverInfo& infoGlobal, btIDebugDraw* debugDrawer);


protected:
	// Documentation inherited.
	virtual void solveMultiBodyGroup(btCollisionObject** bodies, int numBodies, btPersistentManifold** manifold, int numManifolds, btTypedConstraint** constraints, int numConstraints, btMultiBodyConstraint** multiBodyConstraints, int numMultiBodyConstraints, const btContactSolverInfo& info, btIDebugDraw* debugDrawer, btDispatcher* dispatcher);

	/// Sets the splitting policy.
	virtual void setSplittingPolicy(btMultiBodyBlockSplittingPolicy* policy);

	void copyDynamicDataFromOriginalToBlock(btMultiBodyConstraintBlock& block);
	void copyDynamicDataFromBlockToOriginal(btMultiBodyConstraintBlock& block);

	/// Adds a constraint block configuration and returns the total number of configurations added to this solver.
	virtual int addConfig(btBlockConstraintSolverConfig& config);

	/// Returns the number of configurations added to this solver.
	virtual int getNumConfigs() const;

	/// Removes an configuration at \c configIndex
	///
	/// \param[in] configIndex The configuration indext in the range of [0, numConfigs). Passing out of the range is an
	/// undefined behavior.
	virtual void removeConfig(int configIndex);
};

#endif  // BT_MULTIBODY_BLOCK_CONSTRAINT_SOLVER_H
