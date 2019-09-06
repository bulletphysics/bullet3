/*
 Written by Xuchen Han <xuchenhan2015@u.northwestern.edu>
 
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
#include "btDeformableMultiBodyDynamicsWorld.h"
#include "BulletDynamics/Featherstone/btMultiBodyLinkCollider.h"
#include "BulletDynamics/Featherstone/btMultiBodyConstraint.h"

struct btCollisionObjectWrapper;
class btDeformableBackwardEulerObjective;
class btDeformableMultiBodyDynamicsWorld;

class btDeformableBodySolver : public btSoftBodySolver
{
//    using TVStack = btAlignedObjectArray<btVector3>;
    typedef btAlignedObjectArray<btVector3> TVStack;
protected:
    int m_numNodes;
    TVStack m_dv;
    TVStack m_ddv;
    TVStack m_residual;
    btAlignedObjectArray<btSoftBody *> m_softBodySet;
   
    btAlignedObjectArray<btVector3> m_backupVelocity;
    btScalar m_dt;
    btScalar m_contact_iterations;
    btConjugateGradient<btDeformableBackwardEulerObjective> m_cg;
    bool m_implicit;
    int m_maxNewtonIterations;
    btScalar m_newtonTolerance;
    
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
    
    virtual void solveDeformableConstraints(btScalar solverdt);
    
    btScalar solveContactConstraints();
    
    virtual void solveConstraints(btScalar dt){}
    
    void reinitialize(const btAlignedObjectArray<btSoftBody *>& softBodies, btScalar dt);
    
    void setConstraints();
    
    void predictDeformableMotion(btSoftBody* psb, btScalar dt);
    
    void backupVelocity();
    void backupVn();
    void revertVelocity();
    void updateVelocity();
    
    bool updateNodes();
    
    void computeStep(TVStack& dv, const TVStack& residual);
                     
    virtual void predictMotion(btScalar solverdt);

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
    
    void setImplicit(bool implicit);
    
    void updateState();
    
    void updateDv();
    
    void updateTempPosition();
};

#endif /* btDeformableBodySolver_h */
