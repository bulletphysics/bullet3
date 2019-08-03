//
//  btDeformableBackwardEulerObjective.h
//  BulletSoftBody
//
//  Created by Xuchen Han on 7/1/19.
//

#ifndef BT_BACKWARD_EULER_OBJECTIVE_H
#define BT_BACKWARD_EULER_OBJECTIVE_H
#include <functional>
#include "btConjugateGradient.h"
#include "btDeformableLagrangianForce.h"
#include "btDeformableMassSpringForce.h"
#include "btDeformableGravityForce.h"
#include "btDeformableContactProjection.h"
#include "btPreconditioner.h"
#include "btDeformableRigidDynamicsWorld.h"

class btDeformableRigidDynamicsWorld;
class btDeformableBackwardEulerObjective
{
public:
//    using TVStack = btAlignedObjectArray<btVector3>;
    typedef btAlignedObjectArray<btVector3> TVStack;
    btScalar m_dt;
    btDeformableRigidDynamicsWorld* m_world;
    btAlignedObjectArray<btDeformableLagrangianForce*> m_lf;
    btAlignedObjectArray<btSoftBody *>& m_softBodies;
    Preconditioner* m_preconditioner;
    btDeformableContactProjection projection;
    const TVStack& m_backupVelocity;
    btAlignedObjectArray<btSoftBody::Node* > m_nodes;
    
    btDeformableBackwardEulerObjective(btAlignedObjectArray<btSoftBody *>& softBodies, const TVStack& backup_v);
    
    virtual ~btDeformableBackwardEulerObjective() {}
    
    void initialize(){}
    
    // compute the rhs for CG solve, i.e, add the dt scaled implicit force to residual
    void computeResidual(btScalar dt, TVStack& residual) const;
    
    // add explicit force to the velocity
    void applyExplicitForce(TVStack& force);
    
    // apply force to velocity and optionally reset the force to zero
    void applyForce(TVStack& force, bool setZero);
    
    // compute the norm of the residual
    btScalar computeNorm(const TVStack& residual) const;
    
    // compute one step of the solve (there is only one solve if the system is linear)
    void computeStep(TVStack& dv, const TVStack& residual, const btScalar& dt);
    
    // perform A*x = b
    void multiply(const TVStack& x, TVStack& b) const;
    
    // set initial guess for CG solve
    void initialGuess(TVStack& dv, const TVStack& residual);
    
    // reset data structure
    void reinitialize(bool nodeUpdated);
    
    void setDt(btScalar dt);
    
    // enforce constraints in CG solve
    void enforceConstraint(TVStack& x)
    {
        projection.enforceConstraint(x);
        updateVelocity(x);
    }
    
    // add dv to velocity
    void updateVelocity(const TVStack& dv);
    
    //set constraints as projections
    void setConstraints();
    
    // update the projections and project the residual
    void project(TVStack& r)
    {
        projection.update();
        projection.project(r);
    }
    
    // perform precondition M^(-1) x = b
    void precondition(const TVStack& x, TVStack& b)
    {
        m_preconditioner->operator()(x,b);
    }
    
    virtual void setWorld(btDeformableRigidDynamicsWorld* world)
    {
        m_world = world;
        projection.setWorld(world);
    }
    
    virtual void updateId()
    {
        size_t id = 0;
        m_nodes.clear();
        for (int i = 0; i < m_softBodies.size(); ++i)
        {
            btSoftBody* psb = m_softBodies[i];
            for (int j = 0; j < psb->m_nodes.size(); ++j)
            {
                psb->m_nodes[j].index = id;
                m_nodes.push_back(&psb->m_nodes[j]);
                ++id;
            }
        }
    }
    
    const btAlignedObjectArray<btSoftBody::Node*>* getIndices() const
    {
        return &m_nodes;
    }
};

#endif /* btBackwardEulerObjective_h */
