//
//  btDeformableBackwardEulerObjective.cpp
//  BulletSoftBody
//
//  Created by Xuchen Han on 7/9/19.
//

#include "btDeformableBackwardEulerObjective.h"

btDeformableBackwardEulerObjective::btDeformableBackwardEulerObjective(btAlignedObjectArray<btSoftBody *>& softBodies, const TVStack& backup_v)
: m_softBodies(softBodies)
, projection(m_softBodies, m_dt)
, m_backupVelocity(backup_v)
{
    // TODO: this should really be specified in initialization instead of here
    btDeformableMassSpringForce* mass_spring = new btDeformableMassSpringForce(m_softBodies);
    btDeformableGravityForce* gravity = new btDeformableGravityForce(m_softBodies, btVector3(0,-10,0));
    m_preconditioner = new DefaultPreconditioner();
    m_lf.push_back(mass_spring);
    m_lf.push_back(gravity);
}

void btDeformableBackwardEulerObjective::reinitialize(bool nodeUpdated)
{
    if(nodeUpdated)
    {
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
            const auto& node = psb->m_nodes[j];
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
    for (auto it : projection.m_constraints)
    {
        int i = projection.m_indices[it.first];
        it.first->m_v = m_backupVelocity[i] + dv[i];
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
