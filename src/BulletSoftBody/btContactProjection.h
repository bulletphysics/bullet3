//
//  btContactProjection.h
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
class btContactProjection : public btCGProjection
{
public:
    btContactProjection(btAlignedObjectArray<btSoftBody *>& softBodies, const btScalar& dt)
    : btCGProjection(softBodies, dt)
    {
    }
    
    virtual ~btContactProjection()
    {
    }
    
    // apply the constraints to the rhs
    virtual void operator()(TVStack& x);
    
    // apply constraints to x in Ax=b
    virtual void enforceConstraint(TVStack& x);
    
    // update the constraints
    virtual void update(const TVStack& dv, const TVStack& backupVelocity);
    
    virtual void setConstraints();
};
#endif /* btContactProjection_h */
