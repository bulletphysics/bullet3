//
//  btDeformableBodySolver.cpp
//  BulletSoftBody
//
//  Created by Xuchen Han on 7/9/19.
//

#include <stdio.h>
#include "btDeformableBodySolver.h"

btDeformableBodySolver::btDeformableBodySolver()
: m_numNodes(0)
, cg(10)
{
    m_objective = new btDeformableBackwardEulerObjective(m_softBodySet, m_backupVelocity);
}

btDeformableBodySolver::~btDeformableBodySolver()
{
    delete m_objective;
}

void btDeformableBodySolver::solveConstraints(float solverdt)
{
    m_objective->setDt(solverdt);
    
    // add constraints to the solver
    setConstraints();
    
    // save v_{n+1}^* velocity after explicit forces
    backupVelocity();
    m_objective->computeResidual(solverdt, m_residual);
//   m_objective->initialGuess(m_dv, m_residual);
    computeStep(m_dv, m_residual);
    updateVelocity();
}

void btDeformableBodySolver::computeStep(TVStack& dv, const TVStack& residual)
{
    btScalar tolerance = std::numeric_limits<float>::epsilon()* 1024 * m_objective->computeNorm(residual);
    cg.solve(*m_objective, dv, residual, tolerance);
}

void btDeformableBodySolver::reinitialize(const btAlignedObjectArray<btSoftBody *>& softBodies)
{
    m_softBodySet.copyFromArray(softBodies);
    bool nodeUpdated = updateNodes();
    if (nodeUpdated)
    {
        m_dv.resize(m_numNodes);
        m_residual.resize(m_numNodes);
    }
    
    for (int i = 0; i < m_dv.size(); ++i)
    {
        m_dv[i].setZero();
        m_residual[i].setZero();
    }
    m_objective->reinitialize(nodeUpdated);
}

void btDeformableBodySolver::setConstraints()
{
    m_objective->setConstraints();
}

void btDeformableBodySolver::setWorld(btDeformableRigidDynamicsWorld* world)
{
    m_objective->setWorld(world);
}

void btDeformableBodySolver::updateVelocity()
{
    // serial implementation
    int counter = 0;
    for (int i = 0; i < m_softBodySet.size(); ++i)
    {
        btSoftBody* psb = m_softBodySet[i];
        for (int j = 0; j < psb->m_nodes.size(); ++j)
        {
            psb->m_nodes[j].m_v = m_backupVelocity[counter]+m_dv[counter];
            ++counter;
        }
    }
}

void btDeformableBodySolver::backupVelocity()
{
    // serial implementation
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

bool btDeformableBodySolver::updateNodes()
{
    int numNodes = 0;
    for (int i = 0; i < m_softBodySet.size(); ++i)
        numNodes += m_softBodySet[i]->m_nodes.size();
    if (numNodes != m_numNodes)
    {
        m_numNodes = numNodes;
        m_backupVelocity.resize(numNodes);
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
