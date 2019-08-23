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

#include <stdio.h>
#include <limits>
#include "btDeformableBodySolver.h"
#include "LinearMath/btQuickprof.h"

btDeformableBodySolver::btDeformableBodySolver()
: m_numNodes(0)
, m_cg(50)
{
    m_objective = new btDeformableBackwardEulerObjective(m_softBodySet, m_backupVelocity);
}

btDeformableBodySolver::~btDeformableBodySolver()
{
    delete m_objective;
}

void btDeformableBodySolver::solveDeformableConstraints(btScalar solverdt)
{
    BT_PROFILE("solveConstraints");
    m_objective->computeResidual(solverdt, m_residual);
    m_objective->applyDynamicFriction(m_residual);
    computeStep(m_dv, m_residual);
    
    updateVelocity();
}

void btDeformableBodySolver::computeStep(TVStack& dv, const TVStack& residual)
{
    btScalar tolerance = std::numeric_limits<float>::epsilon() * 16 * m_objective->computeNorm(residual);
    m_cg.solve(*m_objective, dv, residual, tolerance);
}

void btDeformableBodySolver::reinitialize(const btAlignedObjectArray<btSoftBody *>& softBodies, btScalar dt)
{
    m_softBodySet.copyFromArray(softBodies);
    bool nodeUpdated = updateNodes();
    
    if (nodeUpdated)
    {
        m_dv.resize(m_numNodes, btVector3(0,0,0));
        m_residual.resize(m_numNodes, btVector3(0,0,0));
        m_backupVelocity.resize(m_numNodes, btVector3(0,0,0));
    }
    
    // need to setZero here as resize only set value for newly allocated items
    for (int i = 0; i < m_numNodes; ++i)
    {
        m_dv[i].setZero();
        m_residual[i].setZero();
    }
    
    m_objective->reinitialize(nodeUpdated, dt);
}

void btDeformableBodySolver::setConstraints()
{
    BT_PROFILE("setConstraint");
    m_objective->setConstraints();
}

btScalar btDeformableBodySolver::solveContactConstraints()
{
    BT_PROFILE("setConstraint");
    btScalar maxSquaredResidual = m_objective->projection.update();
    m_objective->enforceConstraint(m_dv);
    m_objective->updateVelocity(m_dv);
    return maxSquaredResidual;
}


void btDeformableBodySolver::updateVelocity()
{
    int counter = 0;
    for (int i = 0; i < m_softBodySet.size(); ++i)
    {
        btSoftBody* psb = m_softBodySet[i];
        for (int j = 0; j < psb->m_nodes.size(); ++j)
        {
            // set NaN to zero;
            if (m_dv[counter] != m_dv[counter])
            {
                m_dv[counter].setZero();
            }
            psb->m_nodes[j].m_v = m_backupVelocity[counter]+m_dv[counter];
            ++counter;
        }
    }
}

void btDeformableBodySolver::backupVelocity()
{
    int counter = 0;
    for (int i = 0; i < m_softBodySet.size(); ++i)
    {
        btSoftBody* psb = m_softBodySet[i];
        for (int j = 0; j < psb->m_nodes.size(); ++j)
        {
            m_backupVelocity[counter++] = psb->m_nodes[j].m_v;
        }
    }
}

void btDeformableBodySolver::revertVelocity()
{
    int counter = 0;
    for (int i = 0; i < m_softBodySet.size(); ++i)
    {
        btSoftBody* psb = m_softBodySet[i];
        for (int j = 0; j < psb->m_nodes.size(); ++j)
        {
            psb->m_nodes[j].m_v = m_backupVelocity[counter++];
        }
    }
}

bool btDeformableBodySolver::updateNodes()
{
    int numNodes = 0;
    for (int i = 0; i < m_softBodySet.size(); ++i)
        numNodes += m_softBodySet[i]->m_nodes.size();
    if (numNodes != m_numNodes)
    {
        m_numNodes = numNodes;
        return true;
    }
    return false;
}


void btDeformableBodySolver::predictMotion(float solverdt)
{
    for (int i = 0; i < m_softBodySet.size(); ++i)
    {
        btSoftBody *psb = m_softBodySet[i];
        
        if (psb->isActive())
        {
            // apply explicit forces to velocity
            m_objective->applyExplicitForce(m_residual);
            // predict motion for collision detection
            predictDeformableMotion(psb, solverdt);
        }
    }
}

void btDeformableBodySolver::predictDeformableMotion(btSoftBody* psb, btScalar dt)
{
    int i, ni;
    
    /* Prepare                */
    psb->m_sst.sdt = dt * psb->m_cfg.timescale;
    psb->m_sst.isdt = 1 / psb->m_sst.sdt;
    psb->m_sst.velmrg = psb->m_sst.sdt * 3;
    psb->m_sst.radmrg = psb->getCollisionShape()->getMargin();
    psb->m_sst.updmrg = psb->m_sst.radmrg * (btScalar)0.25;
    /* Integrate            */
    for (i = 0, ni = psb->m_nodes.size(); i < ni; ++i)
    {
        btSoftBody::Node& n = psb->m_nodes[i];
        n.m_q = n.m_x;
        n.m_x += n.m_v * dt;
    }
    /* Bounds                */
    psb->updateBounds();
    /* Nodes                */
    ATTRIBUTE_ALIGNED16(btDbvtVolume)
    vol;
    for (i = 0, ni = psb->m_nodes.size(); i < ni; ++i)
    {
        btSoftBody::Node& n = psb->m_nodes[i];
        vol = btDbvtVolume::FromCR(n.m_x, psb->m_sst.radmrg);
        psb->m_ndbvt.update(n.m_leaf,
                       vol,
                       n.m_v * psb->m_sst.velmrg,
                       psb->m_sst.updmrg);
    }

    /* Clear contacts        */
    psb->m_rcontacts.resize(0);
    psb->m_scontacts.resize(0);
    /* Optimize dbvt's        */
    psb->m_ndbvt.optimizeIncremental(1);
}

void btDeformableBodySolver::updateSoftBodies()
{
    for (int i = 0; i < m_softBodySet.size(); i++)
    {
        btSoftBody *psb = (btSoftBody *)m_softBodySet[i];
        if (psb->isActive())
        {
            psb->updateNormals(); // normal is updated here
        }
    }
}
