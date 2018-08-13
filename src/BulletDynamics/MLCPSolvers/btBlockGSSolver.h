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

#ifndef BT_BGS_SOLVER_H
#define BT_BGS_SOLVER_H

#include "BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h"
#include "BulletDynamics/MLCPSolvers/btSolveProjectedGaussSeidel.h"
#include "LinearMath/btMatrixX.h"
#include "LinearMath/btThreads.h"
#include "BulletDynamics/MLCPSolvers/btMLCPSolverInterface.h"

/// Implementation of the blocked Gauss-Seidel (BGS) constraint solver.
///
/// Loosely speaking this class is a mix of btSequentialImpulseConstraintSolver and btMLCPSolver. Specifically, BSG
/// forms many small MLCPs of subsets of the constraints and solves each MLCP using solvers suitable for small-sized
/// problems (e.g., Dantzig), and then applies the Gauss-Seidel splitting to the blocked MLCPs unlike the regular
/// Gauss-Seidel applies it to every single constraint. A block is usually defined as all constraints associated with a
/// single contact point or any set of constraints better to be solved together at once.
///
/// The expected performance is somewhere between btSequentialImpulseConstraintSolver and btMLCPSolver in terms of
/// speed and accuracy.
class btBlockGSSolver : public btSequentialImpulseConstraintSolver
{
protected:
	/// Data struct for MLCP block
	struct btMLCP
	{
		btMatrixXu m_A;
		btVectorXu m_b;
		btVectorXu m_bSplit;
		btVectorXu m_x;
		btVectorXu m_xSplit;
		btVectorXu m_lo;
		btVectorXu m_hi;
		btAlignedObjectArray<int> m_limitDependencies;
		btAlignedObjectArray<btSolverConstraint*> m_allConstraintPtrArray;
	};

	/// Array of MLCP blocks.
	btAlignedObjectArray<btMLCP> m_mlcpArray;
	// Note: If we know the size of MLCP in compile time, we could use a fixed size MLCP struct, which may don't
	// require memory allocations in the simulation loops.

	/// Default MLCP solver
	btMLCPSolverInterface* m_defaultSolver;

	/// PGS MLCP solver to be used when \c m_defaultSolver is failed.
	btSolveProjectedGaussSeidel m_pgsSolver;

	/// Count of fallbacks of using PGS LCP solver (\c m_pgsSolver), which happens when the default MLCP solver (\c m_defaultSolver) fails.
	int m_fallback;

	// Documentation inherited.
	btScalar solveGroupCacheFriendlySetup(
		btCollisionObject** bodies,
		int numBodies,
		btPersistentManifold** manifoldPtr,
		int numManifolds,
		btTypedConstraint** constraints,
		int numConstraints,
		const btContactSolverInfo& infoGlobal,
		btIDebugDraw* debugDrawer) BT_OVERRIDE;

	/// Constructs a MLCP for the constraints associated with a contact point such as normal contact constraints,
	/// frictional constraints, and torsional constraints. The constructed MLCP block is stored in \c m_mlcpArray.
	///
	/// \param[in] index Index to the MLCP block.
	/// \param[in] numFrictionPerContact Number of friction constraints per contact.
	/// \param[in] infoGlobal Global configurations for contact solver.
	void setupContactConstraintMLCPBlock(
		int index,
		int numFrictionPerContact,
		const btContactSolverInfo& infoGlobal);

	// Documentation inherited.
	btScalar solveSingleIteration(
		int iteration,
		btCollisionObject** bodies,
		int numBodies,
		btPersistentManifold** manifoldPtr,
		int numManifolds,
		btTypedConstraint** constraints,
		int numConstraints,
		const btContactSolverInfo& infoGlobal,
		btIDebugDraw* debugDrawer) BT_OVERRIDE;

	/// Solves a MLCP block, which are stored in \c m_mlcpArray.
	///
	/// \param[in] index Index to the MLCP block.
	/// \param[in] infoGlobal Global configurations for contact solver.
	btScalar solveMLCPBlock(int index, const btContactSolverInfo& infoGlobal);

public:
	/// Constructor
	///
	/// \param[in] solver MLCP solver. Assumed it's not null.
	btBlockGSSolver(btMLCPSolverInterface* solver);

	/// Destructor
	virtual ~btBlockGSSolver();

	/// Sets MLCP solver. Assumed it's not null.
	void setMLCPSolver(btMLCPSolverInterface* solver);

	/// Returns the number of fallbacks of using btSequentialImpulseConstraintSolver, which happens when the MLCP
	/// solver fails.
	int getNumFallbacks() const;

	/// Sets the number of fallbacks. This function may be used to reset the number to zero.
	void setNumFallbacks(int num);

	/// Returns the constraint solver type.
	virtual btConstraintSolverType getSolverType() const;
};

#endif  // BT_BGS_SOLVER_H
