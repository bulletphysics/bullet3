//
//  btBackwardEulerObjective.cpp
//  BulletSoftBody
//
//  Created by Xuchen Han on 7/9/19.
//

#include "btBackwardEulerObjective.h"

btBackwardEulerObjective::btBackwardEulerObjective(btAlignedObjectArray<btSoftBody *>& softBodies, const TVStack& backup_v)
: cg(20)
, m_softBodies(softBodies)
, precondition(DefaultPreconditioner())
, projection(m_softBodies, m_dt)
, m_backupVelocity(backup_v)
{
    // TODO: this should really be specified in initialization instead of here
    btMassSpring* mass_spring = new btMassSpring(m_softBodies);
    m_lf.push_back(mass_spring);
}

void btBackwardEulerObjective::reinitialize(bool nodeUpdated)
{
    if(nodeUpdated)
    {
        projection.setSoftBodies(m_softBodies);
    }
    for (int i = 0; i < m_lf.size(); ++i)
    {
        m_lf[i]->reinitialize(nodeUpdated);
        projection.reinitialize(nodeUpdated);
    }
}


void btBackwardEulerObjective::multiply(const TVStack& x, TVStack& b) const
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
        m_lf[i]->addScaledDampingForceDifferential(-m_dt, x, b);
        
        // add stiffness matrix when fully implicity
        m_lf[i]->addScaledElasticForceDifferential(-m_dt*m_dt, x, b);
    }
}

void btBackwardEulerObjective::computeStep(TVStack& dv, const TVStack& residual, const btScalar& dt)
{
    m_dt = dt;
    btScalar tolerance = std::numeric_limits<float>::epsilon()* 16 * computeNorm(residual);
    cg.solve(*this, dv, residual, tolerance);
    }

void btBackwardEulerObjective::updateVelocity(const TVStack& dv)
{
    for (int i = 0; i < m_softBodies.size(); ++i)
    {
        int counter = 0;
        for (int i = 0; i < m_softBodies.size(); ++i)
        {
            btSoftBody* psb = m_softBodies[i];
            for (int j = 0; j < psb->m_nodes.size(); ++j)
            {
                // only the velocity of the constrained nodes needs to be updated during CG solve
                if (projection.m_constrainedDirections.size() > 0)
                    psb->m_nodes[j].m_v = m_backupVelocity[counter] + dv[counter];
                ++counter;
            }
        }
    }
}