//
//  btDeformableBackwardEulerObjective.cpp
//  BulletSoftBody
//
//  Created by Xuchen Han on 7/9/19.
//

#include "btDeformableBackwardEulerObjective.h"

btDeformableBackwardEulerObjective::btDeformableBackwardEulerObjective(btAlignedObjectArray<btSoftBody *>& softBodies, const TVStack& backup_v)
: m_softBodies(softBodies)
, projection(m_softBodies, m_dt, &m_nodes)
, m_backupVelocity(backup_v)
{
    m_preconditioner = new DefaultPreconditioner();
}

void btDeformableBackwardEulerObjective::reinitialize(bool nodeUpdated)
{
    if(nodeUpdated)
    {
        updateId();
        projection.setSoftBodies(m_softBodies);
    }
    for (int i = 0; i < m_lf.size(); ++i)
    {
        m_lf[i]->reinitialize(nodeUpdated);
    }
    projection.reinitialize(nodeUpdated);
    m_preconditioner->reinitialize(nodeUpdated);
}

void btDeformableBackwardEulerObjective::setDt(btScalar dt)
{
    m_dt = dt;
}

void btDeformableBackwardEulerObjective::multiply(const TVStack& x, TVStack& b) const
{
    for (int i = 0; i < b.size(); ++i)
        b[i].setZero();
    
    // add in the mass term
    size_t counter = 0;
    for (int i = 0; i < m_softBodies.size(); ++i)
    {
        btSoftBody* psb = m_softBodies[i];
        for (int j = 0; j < psb->m_nodes.size(); ++j)
        {
            const btSoftBody::Node& node = psb->m_nodes[j];
            b[counter] += (node.m_im == 0) ? btVector3(0,0,0) : x[counter] / node.m_im;
            ++counter;
        }
    }
    
    for (int i = 0; i < m_lf.size(); ++i)
    {
        // add damping matrix
        m_lf[i]->addScaledForceDifferential(-m_dt, x, b);
    }
}

void btDeformableBackwardEulerObjective::updateVelocity(const TVStack& dv)
{
    // only the velocity of the constrained nodes needs to be updated during CG solve
//    for (auto it : projection.m_constraints)
    for (int i = 0; i < projection.m_constraints.size(); ++i)
    {
        int index = projection.m_constraints.getKeyAtIndex(i).getUid1();
        m_nodes[index]->m_v = m_backupVelocity[index] + dv[index];
    }
}

void btDeformableBackwardEulerObjective::applyForce(TVStack& force, bool setZero)
{
    size_t counter = 0;
    for (int i = 0; i < m_softBodies.size(); ++i)
    {
        btSoftBody* psb = m_softBodies[i];
        for (int j = 0; j < psb->m_nodes.size(); ++j)
        {
            btScalar one_over_mass = (psb->m_nodes[j].m_im == 0) ? 0 : psb->m_nodes[j].m_im;
            psb->m_nodes[j].m_v += one_over_mass * force[counter++];
        }
    }
    if (setZero)
    {
        for (int i = 0; i < force.size(); ++i)
            force[i].setZero();
    }
}

void btDeformableBackwardEulerObjective::computeResidual(btScalar dt, TVStack &residual) const
{
    // add implicit force
    for (int i = 0; i < m_lf.size(); ++i)
    {
        m_lf[i]->addScaledImplicitForce(dt, residual);
    }
}

btScalar btDeformableBackwardEulerObjective::computeNorm(const TVStack& residual) const
{
    btScalar norm_squared = 0;
    for (int i = 0; i < residual.size(); ++i)
    {
        norm_squared += residual[i].length2();
    }
    return std::sqrt(norm_squared+SIMD_EPSILON);
}

void btDeformableBackwardEulerObjective::applyExplicitForce(TVStack& force)
{
    for (int i = 0; i < m_lf.size(); ++i)
        m_lf[i]->addScaledExplicitForce(m_dt, force);
    applyForce(force, true);
}

void btDeformableBackwardEulerObjective::initialGuess(TVStack& dv, const TVStack& residual)
{
    size_t counter = 0;
    for (int i = 0; i < m_softBodies.size(); ++i)
    {
        btSoftBody* psb = m_softBodies[i];
        for (int j = 0; j < psb->m_nodes.size(); ++j)
        {
            dv[counter] = psb->m_nodes[j].m_im * residual[counter];
            ++counter;
        }
    }
}

//set constraints as projections
void btDeformableBackwardEulerObjective::setConstraints()
{
    // build islands for multibody solve
    m_world->btMultiBodyDynamicsWorld::buildIslands();
    projection.setConstraints();
}
