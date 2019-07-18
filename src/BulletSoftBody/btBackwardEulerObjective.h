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
#include "btPreconditioner.h"
#include "btDeformableRigidDynamicsWorld.h"

class btDeformableRigidDynamicsWorld;
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
    
    // update the constraints treated as projections
    void updateProjection(const TVStack& dv)
    {
        projection.update(dv, m_backupVelocity);
    }
    
    // set initial guess for CG solve
    void initialGuess(TVStack& dv, const TVStack& residual);
    
    // reset data structure
    void reinitialize(bool nodeUpdated);
    
    // enforce constraints in CG solve
    void enforceConstraint(TVStack& x)
    {
        projection.enforceConstraint(x);
        updateVelocity(x);
    }
    
    // add dv to velocity
    void updateVelocity(const TVStack& dv);
    
    //set constraints as projections
    void setConstraints()
    {
        projection.setConstraints();
    }
    
    // update the projections and project the residual
    void project(TVStack& r, const TVStack& dv)
    {
        updateProjection(dv);
        projection(r);
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
};

#endif /* btBackwardEulerObjective_h */
