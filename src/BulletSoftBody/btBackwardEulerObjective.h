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
class btBackwardEulerObjective
{
public:
    using TVStack = btAlignedObjectArray<btVector3>;
    struct DefaultPreconditioner
    {
        void operator()(const TVStack& x, TVStack& b)
        {
            btAssert(b.size() == x.size());
            for (int i = 0; i < b.size(); ++i)
                b[i] = x[i];
        }
    };
    btScalar m_dt;
    btConjugateGradient<btBackwardEulerObjective> cg;
    btDeformableRigidDynamicsWorld* m_world;
    btAlignedObjectArray<btLagrangianForce*> m_lf;
    btAlignedObjectArray<btSoftBody *>& m_softBodies;
    std::function<void(const TVStack&, TVStack&)> precondition;
    btContactProjection projection;
    
    btBackwardEulerObjective(btAlignedObjectArray<btSoftBody *>& softBodies, const TVStack& backup_v)
    : cg(20)
    , m_softBodies(softBodies)
    , precondition(DefaultPreconditioner())
    , projection(m_softBodies)
    {
        // TODO: this should really be specified in initialization instead of here
        btMassSpring* mass_spring = new btMassSpring(m_softBodies);
        m_lf.push_back(mass_spring);
    }
    
    virtual ~btBackwardEulerObjective() {}
    
    void initialize(){}
    
    void computeResidual(btScalar dt, TVStack& residual) const
    {
        // gravity is treated explicitly in predictUnconstraintMotion
        
        // add force
        for (int i = 0; i < m_lf.size(); ++i)
        {
            m_lf[i]->addScaledForce(dt, residual);
        }
    }
    
    btScalar computeNorm(const TVStack& residual) const
    {
        btScalar norm_squared = 0;
        for (int i = 0; i < residual.size(); ++i)
        {
            norm_squared += residual[i].length2();
        }
        return std::sqrt(norm_squared);
    }
    
    void computeStep(TVStack& dv, const TVStack& residual, const btScalar& dt)
    {
        m_dt = dt;
        btScalar tolerance = std::numeric_limits<float>::epsilon()*16 * computeNorm(residual);
        cg.solve(*this, dv, residual, tolerance);
    }
    
    void multiply(const TVStack& x, TVStack& b) const
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
            // damping force is implicit and elastic force is explicit
            m_lf[i]->addScaledDampingForceDifferential(-m_dt, x, b);
//            m_lf[i]->addScaledElasticForceDifferential(-m_dt*m_dt, x, b);
        }
    }
    
    void updateProjection(const TVStack& dv)
    {
        projection.update(m_dt, dv);
    }
    
    void reinitialize(bool nodeUpdated)
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
    
    void enforceConstraint(TVStack& x)
    {
        projection.enforceConstraint(x);
    }
    
    void project(TVStack& r, const TVStack& dv)
    {
        updateProjection(dv);
        projection(r);
    }
    
    template <class Func>
    void setPreconditioner(Func preconditioner_func)
    {
        precondition = preconditioner_func;
    }
    
    virtual void setWorld(btDeformableRigidDynamicsWorld* world)
    {
        m_world = world;
        projection.setWorld(world);
    }
};

#endif /* btBackwardEulerObjective_h */
