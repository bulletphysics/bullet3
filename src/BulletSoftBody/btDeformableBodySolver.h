/*
 Bullet Continuous Collision Detection and Physics Library
 Copyright (c) 2019 Google Inc. http://bulletphysics.org
 This software is provided 'as-is', without any express or implied warranty.
 In no event will the authors be held liable for any damages arising from the use of this software.
 Permission is granted to anyone to use this software for any purpose,
 including commercial applications, and to alter it and redistribute it freely,
 subject to the following restrictions:
 1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
 2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
 3. This notice may not be removed or altered from any source distribution.
 */

#ifndef BT_DEFORMABLE_BODY_SOLVERS_H
#define BT_DEFORMABLE_BODY_SOLVERS_H


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
//    using TVStack = btAlignedObjectArray<btVector3>;
    typedef btAlignedObjectArray<btVector3> TVStack;
protected:
    int m_numNodes;
    TVStack m_dv;
    TVStack m_residual;
    btAlignedObjectArray<btSoftBody *> m_softBodySet;
   
    btAlignedObjectArray<btVector3> m_backupVelocity;
    btScalar m_dt;
    btConjugateGradient<btDeformableBackwardEulerObjective> m_cg;
    
    
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
    
    void reinitialize(const btAlignedObjectArray<btSoftBody *>& softBodies, btScalar dt);
    
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
