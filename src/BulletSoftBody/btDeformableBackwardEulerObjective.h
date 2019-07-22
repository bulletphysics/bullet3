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
    using TVStack = btAlignedObjectArray<btVector3>;
    btScalar m_dt;
    btDeformableRigidDynamicsWorld* m_world;
    btAlignedObjectArray<btDeformableLagrangianForce*> m_lf;
    btAlignedObjectArray<btSoftBody *>& m_softBodies;
    Preconditioner* m_preconditioner;
    btDeformableContactProjection projection;
    const TVStack& m_backupVelocity;
    
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
    void setConstraints()
    {
        projection.setConstraints();
    }
    
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
};

#endif /* btBackwardEulerObjective_h */
