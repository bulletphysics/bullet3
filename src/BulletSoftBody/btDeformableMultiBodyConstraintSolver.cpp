/*
 Written by Xuchen Han <xuchenhan2015@u.northwestern.edu>
 
 Bullet Continuous Collision Detection and Physics Library
 Copyright (c) 2019 Google Inc. http://bulletphysics.org
 This software is provided 'as-is', without any express or implied warranty.
 In no event will the authors be held liable for any damages arising from the use of this software.
 Permission is granted to anyone to use this software for any purpose,
 including commercial applications, and to alter it and redistribute it freely,
 subject to the following restrictions:
 1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
 2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
 3. This notice may not be removed or altered from any source distribution.
 */


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
