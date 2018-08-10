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

#include "btBGSSolver.h"

#include <string>
#include "LinearMath/btMatrixX.h"
#include "LinearMath/btQuickprof.h"
#include "btSolveProjectedGaussSeidel.h"

// Helper function to compute a delta velocity in the constraint space.
static btScalar computeDeltaVelocityInConstraintSpace(
	const btVector3& angularDeltaVelocity,
	const btVector3& contactNormal,
	btScalar invMass,
	const btVector3& angularJacobian,
	const btVector3& linearJacobian)
{
	return angularDeltaVelocity.dot(angularJacobian) + contactNormal.dot(linearJacobian) * invMass;
}

// Faster version of computeDeltaVelocityInConstraintSpace that can be used when contactNormal and linearJacobian are
// identical.
static btScalar computeDeltaVelocityInConstraintSpace(
	const btVector3& angularDeltaVelocity,
	btScalar invMass,
	const btVector3& angularJacobian)
{
	return angularDeltaVelocity.dot(angularJacobian) + invMass;
}

// Computes the delta velocity at 'constraint' when the unit impulse is applied to 'constraint'.
static btScalar computeConstraintMatrixDiagElementRigidBody(
	const btAlignedObjectArray<btSolverBody>& solverBodyPool,
	const btSolverConstraint& constraint)
{
	BT_PROFILE("Compute diagonal");

	btScalar ret = btScalar(0);

	const int solverBodyIdA = constraint.m_solverBodyIdA;
	const btSolverBody* solverBodyA = &solverBodyPool[solverBodyIdA];
	const btScalar invMassA = solverBodyA->m_originalBody ? solverBodyA->m_originalBody->getInvMass() : 0.0;
	ret += computeDeltaVelocityInConstraintSpace(
		constraint.m_relpos1CrossNormal,
		invMassA,
		constraint.m_angularComponentA);

	const int solverBodyIdB = constraint.m_solverBodyIdB;
	const btSolverBody* solverBodyB = &solverBodyPool[solverBodyIdB];
	const btScalar invMassB = solverBodyB->m_originalBody ? solverBodyB->m_originalBody->getInvMass() : 0.0;
	ret += computeDeltaVelocityInConstraintSpace(
		constraint.m_relpos2CrossNormal,
		invMassB,
		constraint.m_angularComponentB);

	return ret;
}

// Computes the delta velocity at 'offDiagConstraint' when the unit impulse is applied to 'constraint'.
static btScalar computeConstraintMatrixOffDiagElementRigidBody(
	const btAlignedObjectArray<btSolverBody>& solverBodyPool,
	const btSolverConstraint& constraint,
	const btSolverConstraint& offDiagConstraint)
{
	BT_PROFILE("Compute off diagonal");

	btScalar ret = btScalar(0);

	const int solverBodyIdA = constraint.m_solverBodyIdA;
	const int solverBodyIdB = constraint.m_solverBodyIdB;

	const int offDiagSolverBodyIdA = offDiagConstraint.m_solverBodyIdA;
	const btSolverBody* offDiagSolverBodyA = &solverBodyPool[offDiagSolverBodyIdA];
	const btScalar invMassA = offDiagSolverBodyA->m_originalBody ? offDiagSolverBodyA->m_originalBody->getInvMass() : 0.0;

	if (offDiagSolverBodyIdA == solverBodyIdA)
	{
		ret += computeDeltaVelocityInConstraintSpace(
			constraint.m_angularComponentA,
			constraint.m_contactNormal1,
			invMassA,
			offDiagConstraint.m_relpos1CrossNormal,
			offDiagConstraint.m_contactNormal1);
	}
	else if (offDiagSolverBodyIdA == solverBodyIdB)
	{
		ret += computeDeltaVelocityInConstraintSpace(
			constraint.m_angularComponentB,
			constraint.m_contactNormal2,
			invMassA,
			offDiagConstraint.m_relpos1CrossNormal,
			offDiagConstraint.m_contactNormal1);
	}

	const int offDiagSolverBodyIdB = offDiagConstraint.m_solverBodyIdB;
	const btSolverBody* offDiagSolverBodyB = &solverBodyPool[offDiagSolverBodyIdB];
	const btScalar invMassB = offDiagSolverBodyB->m_originalBody ? offDiagSolverBodyB->m_originalBody->getInvMass() : 0.0;

	if (offDiagSolverBodyIdB == solverBodyIdA)
	{
		ret += computeDeltaVelocityInConstraintSpace(
			constraint.m_angularComponentA,
			constraint.m_contactNormal1,
			invMassB,
			offDiagConstraint.m_relpos2CrossNormal,
			offDiagConstraint.m_contactNormal2);
	}
	else if (offDiagSolverBodyIdB == solverBodyIdB)
	{
		ret += computeDeltaVelocityInConstraintSpace(
			constraint.m_angularComponentB,
			constraint.m_contactNormal2,
			invMassB,
			offDiagConstraint.m_relpos2CrossNormal,
			offDiagConstraint.m_contactNormal2);
	}

	return ret;
}

btScalar btBGSSolver::solveGroupCacheFriendlySetup(btCollisionObject** bodies, int numBodies, btPersistentManifold** manifoldPtr, int numManifolds, btTypedConstraint** constraints, int numConstraints, const btContactSolverInfo& infoGlobal, btIDebugDraw* debugDrawer)
{
	const btScalar val = btSequentialImpulseConstraintSolver::solveGroupCacheFriendlySetup(bodies, numBodies, manifoldPtr, numManifolds, constraints, numConstraints, infoGlobal, debugDrawer);

	{
		BT_PROFILE("gather constraint data");

		const int numFrictionPerContact = m_tmpSolverContactConstraintPool.size() == m_tmpSolverContactFrictionConstraintPool.size() ? 1 : 2;
		const int numNormalContactConstraints = m_tmpSolverContactConstraintPool.size();
		m_mlcpArray.resize(numNormalContactConstraints);

		for (int i = 0; i < numNormalContactConstraints; ++i)
		{
			setupContactConstraintMLCPBlock(i, numFrictionPerContact, infoGlobal);
		}
	}

	return val;
}

void btBGSSolver::setupContactConstraintMLCPBlock(int normalContactIndex, int numFrictionPerContact, const btContactSolverInfo& infoGlobal)
{
	btAssert(0 <= normalContactIndex);
	btAssert(normalContactIndex < m_mlcpArray.size());
	btAssert(0 <= numFrictionPerContact);

	btMLCP& mlcp = m_mlcpArray[normalContactIndex];
	btAlignedObjectArray<btSolverConstraint*>& allConstraintPtrArray = mlcp.m_allConstraintPtrArray;
	btMatrixXu& A = mlcp.m_A;
	btVectorXu& b = mlcp.m_b;
	btVectorXu& bSplit = mlcp.m_bSplit;
	btVectorXu& x = mlcp.m_x;
	btVectorXu& xSplit = mlcp.m_xSplit;
	btVectorXu& lo = mlcp.m_lo;
	btVectorXu& hi = mlcp.m_hi;
	btAlignedObjectArray<int>& limitDependencies = mlcp.m_limitDependencies;

	const int numConstraints = 1 + numFrictionPerContact;

	// 1. Setup constraint list. We assume that the first constraint is a normal contact constraint followed by one or two frictional contraints (todo: and torsional frcition constraints)
	allConstraintPtrArray.resize(numConstraints);
	allConstraintPtrArray[0] = &m_tmpSolverContactConstraintPool[normalContactIndex];
	for (int i = 0; i < numFrictionPerContact; ++i)
	{
		const int frictionIndex = normalContactIndex * numFrictionPerContact + i;
		allConstraintPtrArray[1 + i] = &m_tmpSolverContactFrictionConstraintPool[frictionIndex];
	}

	// 2. Compute b, lo, hi, and m_limitDependencies
	{
		BT_PROFILE("init b (rhs), lo/hi, and limitDependencies");

		b.resize(numConstraints);
		b.setZero();

		bSplit.resize(numConstraints);
		bSplit.setZero();

		// Just resize is required. The values are set by btBGSSolver::solveDiagonalBlock()
		lo.resize(numConstraints);
		hi.resize(numConstraints);

		// This will not be changed.
		limitDependencies.resize(numConstraints);
		limitDependencies[0] = -1;
		for (int i = 1; i < numConstraints; ++i)  // starts from index 1 intentionally
		{
			limitDependencies[i] = 0;
		}
	}

	// 3. Construct A matrix by using the impulse testing
	{
		BT_PROFILE("Compute A");

		{
			BT_PROFILE("m_A.resize");
			A.resize(numConstraints, numConstraints);
		}

		for (int i = 0; i < numConstraints; ++i)
		{
			// Compute the diagonal of A, which is A(i, i)
			const btSolverConstraint& constraint = *(allConstraintPtrArray[i]);
			const btScalar diagA = computeConstraintMatrixDiagElementRigidBody(m_tmpSolverBodyPool, constraint);
			A.setElem(i, i, diagA);

			// Computes the off-diagonals of A:
			//   a. The rest of i-th row of A, from A(i, i+1) to A(i, n)
			//   b. The rest of i-th column of A, from A(i+1, i) to A(n, i)
			for (int j = i + 1; j < numConstraints; ++j)
			{
				const btSolverConstraint& offDiagConstraint = *(allConstraintPtrArray[j]);
				const btScalar offDiagA = computeConstraintMatrixOffDiagElementRigidBody(m_tmpSolverBodyPool, constraint, offDiagConstraint);
#ifndef NDEBUG
				// Symmetry check
				if (!btFuzzyZero(offDiagA))
				{
					const btScalar offDiagA2 = computeConstraintMatrixOffDiagElementRigidBody(m_tmpSolverBodyPool, offDiagConstraint, constraint);
					btScalar a = btFabs((offDiagA - offDiagA2) / offDiagA);
					// Expect the error should be less than five percent.
					btAssert(a < btScalar(5));
				}
#endif

				// Set the off-diagonal values of A. Note that A is symmetric.
				A.setElem(i, j, offDiagA);
				A.setElem(j, i, offDiagA);
			}
		}
	}
	// Add CFM to the diagonal of m_A
	for (int i = 0; i < A.rows(); ++i)
	{
		A.setElem(i, i, A(i, i) + infoGlobal.m_globalCfm / infoGlobal.m_timeStep);
	}

	// 4. Initialize x
	{
		BT_PROFILE("resize/init x");

		x.resize(numConstraints);
		xSplit.resize(numConstraints);

		if (infoGlobal.m_solverMode & SOLVER_USE_WARMSTARTING)
		{
			for (int i = 0; i < numConstraints; ++i)
			{
				const btSolverConstraint& constraint = *(allConstraintPtrArray[i]);
				x[i] = constraint.m_appliedImpulse;
				xSplit[i] = constraint.m_appliedPushImpulse;
			}
		}
		else
		{
			x.setZero();
			xSplit.setZero();
		}
	}
}

btScalar btBGSSolver::solveSingleIteration(int iteration, btCollisionObject** /*bodies*/, int /*numBodies*/, btPersistentManifold** /*manifoldPtr*/, int /*numManifolds*/, btTypedConstraint** constraints, int numConstraints, const btContactSolverInfo& infoGlobal, btIDebugDraw* /*debugDrawer*/)
{
	BT_PROFILE("solveSingleIteration");
	btScalar squaredResidual = btScalar(0);

	// 1. Solve for non-contact constraints, which is identical to btSequentialImpulseConstraintSolver::solveSingleIteration() without contact/friction/torsional constraints handlings.

	int numNonContactPool = m_tmpSolverNonContactConstraintPool.size();
	int numConstraintPool = m_tmpSolverContactConstraintPool.size();
	int numFrictionPool = m_tmpSolverContactFrictionConstraintPool.size();

	if (infoGlobal.m_solverMode & SOLVER_RANDMIZE_ORDER)
	{
		if (1)  // uncomment this for a bit less random ((iteration & 7) == 0)
		{
			for (int j = 0; j < numNonContactPool; ++j)
			{
				int tmp = m_orderNonContactConstraintPool[j];
				int swapi = btRandInt2(j + 1);
				m_orderNonContactConstraintPool[j] = m_orderNonContactConstraintPool[swapi];
				m_orderNonContactConstraintPool[swapi] = tmp;
			}

			// contact/friction constraints are not solved more than
			if (iteration < infoGlobal.m_numIterations)
			{
				for (int j = 0; j < numConstraintPool; ++j)
				{
					int tmp = m_orderTmpConstraintPool[j];
					int swapi = btRandInt2(j + 1);
					m_orderTmpConstraintPool[j] = m_orderTmpConstraintPool[swapi];
					m_orderTmpConstraintPool[swapi] = tmp;
				}

				for (int j = 0; j < numFrictionPool; ++j)
				{
					int tmp = m_orderFrictionConstraintPool[j];
					int swapi = btRandInt2(j + 1);
					m_orderFrictionConstraintPool[j] = m_orderFrictionConstraintPool[swapi];
					m_orderFrictionConstraintPool[swapi] = tmp;
				}
			}
		}
	}

	// solve all joint constraints
	for (int j = 0; j < m_tmpSolverNonContactConstraintPool.size(); j++)
	{
		const btSolverConstraint& constraint = m_tmpSolverNonContactConstraintPool[m_orderNonContactConstraintPool[j]];
		if (iteration < constraint.m_overrideNumSolverIterations)
		{
			const btScalar residual = resolveSingleConstraintRowGeneric(m_tmpSolverBodyPool[constraint.m_solverBodyIdA], m_tmpSolverBodyPool[constraint.m_solverBodyIdB], constraint);
			squaredResidual = btMax(squaredResidual, residual * residual);
		}
	}

	if (iteration >= infoGlobal.m_numIterations)
		return squaredResidual;

	for (int j = 0; j < numConstraints; j++)
	{
		if (constraints[j]->isEnabled())
		{
			const int bodyAid = getOrInitSolverBody(constraints[j]->getRigidBodyA(), infoGlobal.m_timeStep);
			const int bodyBid = getOrInitSolverBody(constraints[j]->getRigidBodyB(), infoGlobal.m_timeStep);
			btSolverBody& bodyA = m_tmpSolverBodyPool[bodyAid];
			btSolverBody& bodyB = m_tmpSolverBodyPool[bodyBid];
			constraints[j]->solveConstraintObsolete(bodyA, bodyB, infoGlobal.m_timeStep);
		}
	}

	// 2. Solve all contact/friction/torsional constraints, which is different from btSequentialImpulseConstraintSolver::solveSingleIteration()
	{
		const int numContactConstraints = m_tmpSolverContactConstraintPool.size();
		for (int c = 0; c < numContactConstraints; ++c)
		{
			const btScalar newSquaredResidual = solveMLCPBlock(c, infoGlobal);
			squaredResidual = btMax(newSquaredResidual, squaredResidual);
		}
	}

	return squaredResidual;
}

static btScalar clampDeltaImpulse(btScalar deltaImpulse, btSolverConstraint& c, int limitDependencies, const btAlignedObjectArray<btSolverConstraint*>& allConstraintPtrArray)
{
	const btScalar sum = btScalar(c.m_appliedImpulse) + deltaImpulse;

	if (limitDependencies == -1)
	{
		if (sum < c.m_lowerLimit)
		{
			deltaImpulse = c.m_lowerLimit - btScalar(c.m_appliedImpulse);
			c.m_appliedImpulse = c.m_lowerLimit;
		}
		else if (sum > c.m_upperLimit)
		{
			deltaImpulse = c.m_upperLimit - btScalar(c.m_appliedImpulse);
			c.m_appliedImpulse = c.m_upperLimit;
		}
		else
		{
			c.m_appliedImpulse = sum;
		}
	}
	else
	{
		const int fIndex = limitDependencies;
		const btSolverConstraint& normalContactConst = *(allConstraintPtrArray[fIndex]);
		const btScalar normalAppliedImpulse = normalContactConst.m_appliedImpulse;
		if (!btFuzzyZero(normalAppliedImpulse))
		{
			const btScalar lowerLimit = c.m_lowerLimit * normalAppliedImpulse;
			const btScalar upperLimit = c.m_upperLimit * normalAppliedImpulse;

			// This clamping is necessary for the two cases: (1) round off error or (2) failure of the LCP solver.
			if (sum < lowerLimit)
			{
				deltaImpulse = lowerLimit - btScalar(c.m_appliedImpulse);
				c.m_appliedImpulse = lowerLimit;
			}
			else if (sum > upperLimit)
			{
				deltaImpulse = upperLimit - btScalar(c.m_appliedImpulse);
				c.m_appliedImpulse = upperLimit;
			}
			else
			{
				c.m_appliedImpulse = sum;
			}
		}
	}

	return deltaImpulse;
}

static btScalar clampDeltaPushImpulse(btScalar deltaPushImpulse, const btSolverConstraint& c, int limitDependencies, const btAlignedObjectArray<btSolverConstraint*>& allConstraintPtrArray)
{
	const btScalar sum = btScalar(c.m_appliedPushImpulse) + deltaPushImpulse;

	if (limitDependencies == -1)
	{
		if (sum < c.m_lowerLimit)
		{
			deltaPushImpulse = c.m_lowerLimit - btScalar(c.m_appliedPushImpulse);
			c.m_appliedPushImpulse = c.m_lowerLimit;
		}
		else if (sum > c.m_upperLimit)
		{
			deltaPushImpulse = c.m_upperLimit - btScalar(c.m_appliedPushImpulse);
			c.m_appliedPushImpulse = c.m_upperLimit;
		}
		else
		{
			c.m_appliedPushImpulse = sum;
		}
	}
	else
	{
		const int fIndex = limitDependencies;
		const btSolverConstraint& normalContactConst = *(allConstraintPtrArray[fIndex]);
		const btScalar normalAppliedImpulse = normalContactConst.m_appliedPushImpulse;
		if (!btFuzzyZero(normalAppliedImpulse))
		{
			const btScalar lowerLimit = c.m_lowerLimit * normalAppliedImpulse;
			const btScalar upperLimit = c.m_upperLimit * normalAppliedImpulse;

			// This clamping is necessary for the two cases: (1) round off error or (2) failure of the LCP solver.
			if (sum < lowerLimit)
			{
				deltaPushImpulse = lowerLimit - btScalar(c.m_appliedPushImpulse);
				c.m_appliedPushImpulse = lowerLimit;
			}
			else if (sum > upperLimit)
			{
				deltaPushImpulse = upperLimit - btScalar(c.m_appliedPushImpulse);
				c.m_appliedPushImpulse = upperLimit;
			}
			else
			{
				c.m_appliedPushImpulse = sum;
			}
		}
	}

	return deltaPushImpulse;
}

btScalar btBGSSolver::solveMLCPBlock(int index, const btContactSolverInfo& infoGlobal)
{
	btAssert(index >= 0);
	btAssert(index < m_mlcpArray.size());

	btMLCP& mlcp = m_mlcpArray[index];
	btAlignedObjectArray<btSolverConstraint*>& allConstraintPtrArray = mlcp.m_allConstraintPtrArray;
	btMatrixXu& A = mlcp.m_A;
	btVectorXu& b = mlcp.m_b;
	btVectorXu& bSplit = mlcp.m_bSplit;
	btVectorXu& x = mlcp.m_x;
	btVectorXu& xSplit = mlcp.m_xSplit;
	btVectorXu& lo = mlcp.m_lo;
	btVectorXu& hi = mlcp.m_hi;
	btAlignedObjectArray<int>& limitDependencies = mlcp.m_limitDependencies;

	if (A.rows() == 0)
		return true;

	const int numConstraints = allConstraintPtrArray.size();

	// Update b, ho, and hi in Gauss-Seidel style
	for (int i = 0; i < numConstraints; ++i)
	{
		const btSolverConstraint& c = *(allConstraintPtrArray[i]);
		const btScalar jacDiag = c.m_jacDiagABInv;
		if (!btFuzzyZero(jacDiag))
		{
			btSolverBody& bodyA = m_tmpSolverBodyPool[c.m_solverBodyIdA];
			btSolverBody& bodyB = m_tmpSolverBodyPool[c.m_solverBodyIdB];

			b[i] = (c.m_rhs - btScalar(c.m_appliedImpulse) * c.m_cfm) / jacDiag;
			const btScalar deltaVel1Dotn = c.m_contactNormal1.dot(bodyA.internalGetDeltaLinearVelocity()) + c.m_relpos1CrossNormal.dot(bodyA.internalGetDeltaAngularVelocity());
			const btScalar deltaVel2Dotn = c.m_contactNormal2.dot(bodyB.internalGetDeltaLinearVelocity()) + c.m_relpos2CrossNormal.dot(bodyB.internalGetDeltaAngularVelocity());
			b[i] -= deltaVel1Dotn;
			b[i] -= deltaVel2Dotn;

			bSplit[i] = (c.m_rhsPenetration - btScalar(c.m_appliedPushImpulse) * c.m_cfm) / jacDiag;
			const btScalar deltaPushVel1Dotn = c.m_contactNormal1.dot(bodyA.getPushVelocity()) + c.m_relpos1CrossNormal.dot(bodyA.getTurnVelocity());
			const btScalar deltaPushVel2Dotn = c.m_contactNormal2.dot(bodyB.getPushVelocity()) + c.m_relpos2CrossNormal.dot(bodyB.getTurnVelocity());
			bSplit[i] -= deltaPushVel1Dotn;
			bSplit[i] -= deltaPushVel2Dotn;
		}
#ifndef NDEBUG
		else
		{
			// Assumed b[i] is set to zero in advance and never changed.
			btAssert(btFuzzyZero(b[i]));
			btAssert(btFuzzyZero(bSplit[i]));
		}
#endif

		const int fIndex = limitDependencies[i];
		// Normal contact constraint
		if (fIndex == -1)
		{
			lo[i] = btMin(c.m_lowerLimit - btScalar(c.m_appliedImpulse), c.m_lowerLimit);
			hi[i] = btMax(c.m_upperLimit - btScalar(c.m_appliedImpulse), c.m_upperLimit);
		}
		// Friction or torsional friction constraints
		else
		{
			btSolverConstraint& normalContactConst = *(allConstraintPtrArray[fIndex]);
			const btScalar appliedNormalImpulse = normalContactConst.m_appliedImpulse;
			if (!btFuzzyZero(appliedNormalImpulse))
			{
				const btScalar appliedFriction = btScalar(c.m_appliedImpulse) / appliedNormalImpulse;
				lo[i] = btMin(c.m_lowerLimit - appliedFriction, c.m_lowerLimit);
				hi[i] = btMax(c.m_upperLimit - appliedFriction, c.m_upperLimit);
			}
			else
			{
				lo[i] = c.m_lowerLimit;
				hi[i] = c.m_upperLimit;
			}
		}
		// TODO(JS): Needs to update for torsional friction once introduced
	}

	bool result = m_defaultSolver->solveMLCP(A, b, x, lo, hi, limitDependencies, infoGlobal.m_numIterations);

	if (!result)
	{
		result = m_pgsSolver.solveMLCP(A, b, x, lo, hi, limitDependencies, infoGlobal.m_numIterations);
		// PGS solver never return false but iterate up to the maximum iteration number
		btAssert(result);
		m_fallback++;
	}

	if (infoGlobal.m_splitImpulse)
	{
		const btMatrixXu Acopy = A;
		const btAlignedObjectArray<int> limitDependenciesCopy = limitDependencies;
		result = m_defaultSolver->solveMLCP(Acopy, bSplit, xSplit, lo, hi, limitDependenciesCopy, infoGlobal.m_numIterations);

		if (!result)
		{
			result = m_pgsSolver.solveMLCP(Acopy, bSplit, xSplit, lo, hi, limitDependenciesCopy, infoGlobal.m_numIterations);
			// PGS solver never return false but iterate up to the maximum iteration number
			btAssert(result);
			m_fallback++;
		}
	}

	btScalar squaredResidual = btScalar(0);

	{
		BT_PROFILE("process block MCLP result");

		for (int i = 0; i < numConstraints; i++)
		{
			btSolverConstraint& c = *(allConstraintPtrArray[i]);

			const int sbA = c.m_solverBodyIdA;
			const int sbB = c.m_solverBodyIdB;
			btSolverBody& solverBodyA = m_tmpSolverBodyPool[sbA];
			btSolverBody& solverBodyB = m_tmpSolverBodyPool[sbB];

			const btScalar deltaImpulse = clampDeltaImpulse(x[i], c, limitDependencies[i], allConstraintPtrArray);
			solverBodyA.internalApplyImpulse(c.m_contactNormal1 * solverBodyA.internalGetInvMass(), c.m_angularComponentA, deltaImpulse);
			solverBodyB.internalApplyImpulse(c.m_contactNormal2 * solverBodyB.internalGetInvMass(), c.m_angularComponentB, deltaImpulse);

			if (infoGlobal.m_splitImpulse)
			{
				const btScalar deltaPushImpulse = clampDeltaPushImpulse(xSplit[i], c, limitDependencies[i], allConstraintPtrArray);
				solverBodyA.internalApplyPushImpulse(c.m_contactNormal1 * solverBodyA.internalGetInvMass(), c.m_angularComponentA, deltaPushImpulse);
				solverBodyB.internalApplyPushImpulse(c.m_contactNormal2 * solverBodyB.internalGetInvMass(), c.m_angularComponentB, deltaPushImpulse);
			}

			const btScalar residual = deltaImpulse * (btScalar(1) / c.m_jacDiagABInv);

			squaredResidual = btMax(squaredResidual, residual * residual);
		}
	}

	return squaredResidual;
}

btBGSSolver::btBGSSolver(btMLCPSolverInterface* solver)
	: m_defaultSolver(solver),
	  m_fallback(0)
{
	btAssert(solver);
}

btBGSSolver::~btBGSSolver()
{
	// Do nothing
}

void btBGSSolver::setMLCPSolver(btMLCPSolverInterface* solver)
{
	btAssert(solver);
	m_defaultSolver = solver;
}

int btBGSSolver::getNumFallbacks() const
{
	return m_fallback;
}

void btBGSSolver::setNumFallbacks(int num)
{
	m_fallback = num;
}

btConstraintSolverType btBGSSolver::getSolverType() const
{
	return BT_BGS_SOLVER;
}
