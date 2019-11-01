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

#ifndef BT_CONTACT_PROJECTION_H
#define BT_CONTACT_PROJECTION_H
#include "btCGProjection.h"
#include "btSoftBody.h"
#include "BulletDynamics/Featherstone/btMultiBodyLinkCollider.h"
#include "BulletDynamics/Featherstone/btMultiBodyConstraint.h"
#include "btDeformableContactConstraint.h"
#include "LinearMath/btHashMap.h"
#include <vector>
class btDeformableContactProjection
{
public:
    typedef btAlignedObjectArray<btVector3> TVStack;
    btAlignedObjectArray<btSoftBody *>& m_softBodies;
    
    // map from node index to static constraint
    btHashMap<btHashInt, btDeformableStaticConstraint> m_staticConstraints;
    // map from node index to node rigid constraint
    btHashMap<btHashInt, btAlignedObjectArray<btDeformableNodeRigidContactConstraint> > m_nodeRigidConstraints;
    // map from node index to face rigid constraint
    btHashMap<btHashInt, btAlignedObjectArray<btDeformableFaceRigidContactConstraint*> > m_faceRigidConstraints;
    // map from node index to deformable constraint
    btHashMap<btHashInt, btAlignedObjectArray<btDeformableFaceNodeContactConstraint*> > m_deformableConstraints;
    // map from node index to node anchor constraint
    btHashMap<btHashInt, btDeformableNodeAnchorConstraint> m_nodeAnchorConstraints;
    
    // all constraints involving face
    btAlignedObjectArray<btDeformableContactConstraint*> m_allFaceConstraints;
    
    // map from node index to projection directions
    btHashMap<btHashInt, btAlignedObjectArray<btVector3> > m_projectionsDict;
    
    btDeformableContactProjection(btAlignedObjectArray<btSoftBody *>& softBodies)
    : m_softBodies(softBodies)
    {
    }
    
    virtual ~btDeformableContactProjection()
    {
    }
    
    // apply the constraints to the rhs of the linear solve
    virtual void project(TVStack& x);
    
    // add friction force to the rhs of the linear solve
    virtual void applyDynamicFriction(TVStack& f);
    
    // update the constraints
    virtual btScalar update();
    
    // Add constraints to m_constraints. In addition, the constraints that each vertex own are recorded in m_constraintsDict.
    virtual void setConstraints();
    
    // Set up projections for each vertex by adding the projection direction to
    virtual void setProjection();
    
    virtual void reinitialize(bool nodeUpdated);
};
#endif /* btDeformableContactProjection_h */
