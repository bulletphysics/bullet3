//
//  btDeformableContactProjection.h
//  BulletSoftBody
//
//  Created by Xuchen Han on 7/4/19.
//

#ifndef BT_CONTACT_PROJECTION_H
#define BT_CONTACT_PROJECTION_H
#include "btCGProjection.h"
#include "btSoftBody.h"
#include "BulletDynamics/Featherstone/btMultiBodyLinkCollider.h"
#include "BulletDynamics/Featherstone/btMultiBodyConstraint.h"
#include <iostream>
class btDeformableContactProjection : public btCGProjection
{
public:
    std::unordered_map<btSoftBody::Node *, btAlignedObjectArray<DeformableContactConstraint> > m_constraints;
    std::unordered_map<btSoftBody::Node *, btAlignedObjectArray<DeformableFrictionConstraint> > m_frictions;
    
    btDeformableContactProjection(btAlignedObjectArray<btSoftBody *>& softBodies, const btScalar& dt)
    : btCGProjection(softBodies, dt)
    {
    }
    
    virtual ~btDeformableContactProjection()
    {
    }
    
    // apply the constraints to the rhs
    virtual void project(TVStack& x);
    
    // apply constraints to x in Ax=b
    virtual void enforceConstraint(TVStack& x);
    
    // update the constraints
    virtual void update();
    
    virtual void setConstraints();
    
    virtual void reinitialize(bool nodeUpdated);
};
#endif /* btDeformableContactProjection_h */
