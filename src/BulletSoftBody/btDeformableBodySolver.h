//
//  btDeformableBodySolver.h
//  BulletSoftBody
//
//  Created by Xuchen Han on 7/1/19.
//

#ifndef BT_DEFORMABLE_BODY_SOLVERS_H
#define BT_DEFORMABLE_BODY_SOLVERS_H

#include <iostream>
#include "btSoftBodySolvers.h"
#include "btDeformableBackwardEulerObjective.h"
#include "btDeformableRigidDynamicsWorld.h"
#include "BulletDynamics/Featherstone/btMultiBodyLinkCollider.h"
#include "BulletDynamics/Featherstone/btMultiBodyConstraint.h"

struct btCollisionObjectWrapper;
class btDeformableBackwardEulerObjective;
class btDeformableRigidDynamicsWorld;

class btDeformableBodySolver : public btSoftBodySolver
{
    using TVStack = btAlignedObjectArray<btVector3>;
protected:
    int m_numNodes;
    TVStack m_dv;
    TVStack m_residual;
    btAlignedObjectArray<btSoftBody *> m_softBodySet;
   
    btAlignedObjectArray<btVector3> m_backupVelocity;
    btScalar m_dt;
    btConjugateGradient<btDeformableBackwardEulerObjective> cg;
    
public:
    btDeformableBackwardEulerObjective* m_objective;
    
    btDeformableBodySolver();
    
    virtual ~btDeformableBodySolver();
    
    virtual SolverTypes getSolverType() const
    {
        return DEFORMABLE_SOLVER;
    }

    virtual void updateSoftBodies();

    virtual void copyBackToSoftBodies(bool bMove = true) {}

    void extracted(float solverdt);
    
    virtual void solveConstraints(float solverdt);
    
    void reinitialize(const btAlignedObjectArray<btSoftBody *>& softBodies);
    
    void setConstraints();
    
    void predictDeformableMotion(btSoftBody* psb, btScalar dt);
    
    void backupVelocity();
    void revertVelocity();
    void updateVelocity();
    
    bool updateNodes();
    
    void computeStep(TVStack& dv, const TVStack& residual);
                     
    virtual void predictMotion(float solverdt);

    virtual void copySoftBodyToVertexBuffer(const btSoftBody *const softBody, btVertexBufferDescriptor *vertexBuffer) {}

    virtual void processCollision(btSoftBody * softBody, const btCollisionObjectWrapper * collisionObjectWrap)
    {
        softBody->defaultCollisionHandler(collisionObjectWrap);
    }

    virtual void processCollision(btSoftBody * softBody, btSoftBody * otherSoftBody) {
        softBody->defaultCollisionHandler(otherSoftBody);
    }
    virtual void optimize(btAlignedObjectArray<btSoftBody *> &softBodies, bool forceUpdate = false){}
    virtual bool checkInitialized(){return true;}
    virtual void setWorld(btDeformableRigidDynamicsWorld* world);
};

#endif /* btDeformableBodySolver_h */
