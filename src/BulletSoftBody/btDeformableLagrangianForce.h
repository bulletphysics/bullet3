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

#ifndef BT_DEFORMABLE_LAGRANGIAN_FORCE_H
#define BT_DEFORMABLE_LAGRANGIAN_FORCE_H

#include "btSoftBody.h"
#include <LinearMath/btHashMap.h>

enum btDeformableLagrangianForceType
{
    BT_GRAVITY_FORCE = 1,
    BT_MASSSPRING_FORCE = 2
};

class btDeformableLagrangianForce
{
public:
//    using TVStack = btAlignedObjectArray<btVector3>;
    typedef btAlignedObjectArray<btVector3> TVStack;
    btAlignedObjectArray<btSoftBody *> m_softBodies;
    const btAlignedObjectArray<btSoftBody::Node*>* m_nodes;
    
    btDeformableLagrangianForce()
    {
    }
    
    virtual ~btDeformableLagrangianForce(){}
    
    virtual void addScaledImplicitForce(btScalar scale, TVStack& force) = 0;
    
    virtual void addScaledForceDifferential(btScalar scale, const TVStack& dv, TVStack& df) = 0;
    
    virtual void addScaledExplicitForce(btScalar scale, TVStack& force) = 0;
    
    virtual btDeformableLagrangianForceType getForceType() = 0;
    
    virtual void reinitialize(bool nodeUpdated)
    {
    }
    
    virtual int getNumNodes()
    {
        int numNodes = 0;
        for (int i = 0; i < m_softBodies.size(); ++i)
        {
            numNodes += m_softBodies[i]->m_nodes.size();
        }
        return numNodes;
    }
    
    virtual void addSoftBody(btSoftBody* psb)
    {
        m_softBodies.push_back(psb);
    }
    
    virtual void setIndices(const btAlignedObjectArray<btSoftBody::Node*>* nodes)
    {
        m_nodes = nodes;
    }
};
#endif /* BT_DEFORMABLE_LAGRANGIAN_FORCE */
