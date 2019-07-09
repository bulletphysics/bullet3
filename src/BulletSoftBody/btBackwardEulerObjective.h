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
    const TVStack& m_backupVelocity;
    
    btBackwardEulerObjective(btAlignedObjectArray<btSoftBody *>& softBodies, const TVStack& backup_v);
    
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
    
    void computeStep(TVStack& dv, const TVStack& residual, const btScalar& dt);
    
    void multiply(const TVStack& x, TVStack& b) const;
    
    void updateProjection(const TVStack& dv)
    {
        projection.update(dv, m_backupVelocity);
    }
    
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
