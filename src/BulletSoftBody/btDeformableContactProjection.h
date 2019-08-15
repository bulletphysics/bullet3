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

#ifndef BT_CONTACT_PROJECTION_H
#define BT_CONTACT_PROJECTION_H
#include "btCGProjection.h"
#include "btSoftBody.h"
#include "BulletDynamics/Featherstone/btMultiBodyLinkCollider.h"
#include "BulletDynamics/Featherstone/btMultiBodyConstraint.h"
#include "LinearMath/btHashMap.h"
class btDeformableContactProjection : public btCGProjection
{
public:
    // map from node index to constraints
    btHashMap<btHashInt, btAlignedObjectArray<DeformableContactConstraint> > m_constraints;
    btHashMap<btHashInt, btAlignedObjectArray<DeformableFrictionConstraint> >m_frictions;
    
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
