//
//  btDeformableMultiBodyConstraintSolver.cpp
//  BulletSoftBody
//
//  Created by Xuchen Han on 8/23/19.
//

#include "btDeformableMultiBodyConstraintSolver.h"
#include <iostream>
// override the iterations method to include deformable/multibody contact
btScalar btDeformableMultiBodyConstraintSolver::solveGroupCacheFriendlyIterations(btCollisionObject** bodies,int numBodies,btPersistentManifold** manifoldPtr, int numManifolds,btTypedConstraint** constraints,int numConstraints,const btContactSolverInfo& infoGlobal,btIDebugDraw* debugDrawer)
{
    {
        ///this is a special step to resolve penetrations (just for contacts)
        solveGroupCacheFriendlySplitImpulseIterations(bodies, numBodies, manifoldPtr, numManifolds, constraints, numConstraints, infoGlobal, debugDrawer);
        
        int maxIterations = m_maxOverrideNumSolverIterations > infoGlobal.m_numIterations ? m_maxOverrideNumSolverIterations : infoGlobal.m_numIterations;
        for (int iteration = 0; iteration < maxIterations; iteration++)
        {
            m_leastSquaresResidual = solveSingleIteration(iteration, bodies, numBodies, manifoldPtr, numManifolds, constraints, numConstraints, infoGlobal, debugDrawer);
            
            solverBodyWriteBack(infoGlobal);
            m_leastSquaresResidual = btMax(m_leastSquaresResidual, m_deformableSolver->solveContactConstraints());
            writeToSolverBody(bodies, numBodies, infoGlobal);
            
            if (m_leastSquaresResidual <= infoGlobal.m_leastSquaresResidualThreshold || (iteration >= (maxIterations - 1)))
            {
#ifdef VERBOSE_RESIDUAL_PRINTF
                printf("residual = %f at iteration #%d\n", m_leastSquaresResidual, iteration);
#endif
                m_analyticsData.m_numSolverCalls++;
                m_analyticsData.m_numIterationsUsed = iteration+1;
                m_analyticsData.m_islandId = -2;
                if (numBodies>0)
                    m_analyticsData.m_islandId = bodies[0]->getCompanionId();
                m_analyticsData.m_numBodies = numBodies;
                m_analyticsData.m_numContactManifolds = numManifolds;
                m_analyticsData.m_remainingLeastSquaresResidual = m_leastSquaresResidual;
                break;
            }
        }
    }
    return 0.f;
}
