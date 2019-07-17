//
//  btBackwardEulerObjective.h
//  BulletSoftBody
//
//  Created by Xuchen Han on 7/1/19.
//

#ifndef BT_BACKWARD_EULER_OBJECTIVE_H
#define BT_BACKWARD_EULER_OBJECTIVE_H
#include <functional>
#include "btConjugateGradient.h"
#include "btLagrangianForce.h"
#include "btMassSpring.h"
#include "btContactProjection.h"
#include "btDeformableRigidDynamicsWorld.h"

class btDeformableRigidDynamicsWorld;

class Preconditioner
{
public:
    using TVStack = btAlignedObjectArray<btVector3>;
    virtual void operator()(const TVStack& x, TVStack& b) = 0;
    virtual void reinitialize(bool nodeUpdated) = 0;
};

class DefaultPreconditioner : public Preconditioner
{
public:
    virtual void operator()(const TVStack& x, TVStack& b)
    {
        btAssert(b.size() == x.size());
        for (int i = 0; i < b.size(); ++i)
            b[i] = x[i];
    }
    virtual void reinitialize(bool nodeUpdated)
    {
        
    }
};

class MassPreconditioner : public Preconditioner
{
    btAlignedObjectArray<btScalar> m_inv_mass;
    const btAlignedObjectArray<btSoftBody *>& m_softBodies;
public:
    MassPreconditioner(const btAlignedObjectArray<btSoftBody *>& softBodies)
    : m_softBodies(softBodies)
    {
    }
    
    virtual void reinitialize(bool nodeUpdated)
    {
        if (nodeUpdated)
        {
            m_inv_mass.clear();
            for (int i = 0; i < m_softBodies.size(); ++i)
            {
                btSoftBody* psb = m_softBodies[i];
                for (int j = 0; j < psb->m_nodes.size(); ++j)
                    m_inv_mass.push_back(psb->m_nodes[j].m_im);
            }
        }
    }
    
    virtual void operator()(const TVStack& x, TVStack& b)
    {
        btAssert(b.size() == x.size());
        btAssert(m_inv_mass.size() == x.size());
        for (int i = 0; i < b.size(); ++i)
            b[i] = x[i] * m_inv_mass[i];
    }
};

class btBackwardEulerObjective
{
public:
    using TVStack = btAlignedObjectArray<btVector3>;
    btScalar m_dt;
    btConjugateGradient<btBackwardEulerObjective> cg;
    btDeformableRigidDynamicsWorld* m_world;
    btAlignedObjectArray<btLagrangianForce*> m_lf;
    btAlignedObjectArray<btSoftBody *>& m_softBodies;
    Preconditioner* m_preconditioner;
    btContactProjection projection;
    const TVStack& m_backupVelocity;
    
    btBackwardEulerObjective(btAlignedObjectArray<btSoftBody *>& softBodies, const TVStack& backup_v);
    
    virtual ~btBackwardEulerObjective() {}
    
    void initialize(){}
    
    void computeResidual(btScalar dt, TVStack& residual) const
    {
        // add implicit force
        for (int i = 0; i < m_lf.size(); ++i)
        {
            m_lf[i]->addScaledImplicitForce(dt, residual);
        }
    }
    
    void applyExplicitForce(TVStack& force)
    {
        for (int i = 0; i < m_lf.size(); ++i)
            m_lf[i]->addScaledExplicitForce(m_dt, force);
        
        size_t counter = 0;
        for (int i = 0; i < m_softBodies.size(); ++i)
        {
            btSoftBody* psb = m_softBodies[i];
            for (int j = 0; j < psb->m_nodes.size(); ++j)
            {
                btScalar one_over_mass = (psb->m_nodes[j].m_im == 0) ? 0 : psb->m_nodes[j].m_im;
                psb->m_nodes[j].m_v += one_over_mass * force[counter];
                force[counter].setZero();
                counter++;
            }
        }
    }
    
    btScalar computeNorm(const TVStack& residual) const
    {
        btScalar norm_squared = 0;
        for (int i = 0; i < residual.size(); ++i)
        {
            norm_squared += residual[i].length2();
        }
        return std::sqrt(norm_squared+SIMD_EPSILON);
    }
    
    void computeStep(TVStack& dv, const TVStack& residual, const btScalar& dt);
    
    void multiply(const TVStack& x, TVStack& b) const;
    
    void updateProjection(const TVStack& dv)
    {
        projection.update(dv, m_backupVelocity);
    }
    void initialGuess(TVStack& dv, const TVStack& residual);
    
    void reinitialize(bool nodeUpdated);
    
    void enforceConstraint(TVStack& x)
    {
        projection.enforceConstraint(x);
        updateVelocity(x);
    }
    
    void updateVelocity(const TVStack& dv);
    
    void setConstraintDirections()
    {
        projection.setConstraintDirections();
    }
    void project(TVStack& r, const TVStack& dv)
    {
        updateProjection(dv);
        projection(r);
    }
    
    void precondition(const TVStack& x, TVStack& b)
    {
        m_preconditioner->operator()(x,b);
    }
    
    virtual void setWorld(btDeformableRigidDynamicsWorld* world)
    {
        m_world = world;
        projection.setWorld(world);
    }
};

#endif /* btBackwardEulerObjective_h */
