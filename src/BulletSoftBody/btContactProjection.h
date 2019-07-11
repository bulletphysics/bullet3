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
    
    // apply the constraints
    virtual void operator()(TVStack& x)
    {
        const int dim = 3;
        for (auto it : m_constraints)
        {
            const btAlignedObjectArray<Constraint>& constraints = it.second;
            size_t i = m_indices[it.first];
            btAssert(constraints.size() <= dim);
            btAssert(constraints.size() > 0);
            if (constraints.size() == 1)
            {
                x[i] -= x[i].dot(constraints[0].m_direction) * constraints[0].m_direction;
            }
            else if (constraints.size() == 2)
            {
                btVector3 free_dir = btCross(constraints[0].m_direction, constraints[1].m_direction);
                free_dir.normalize();
                x[i] = x[i].dot(free_dir) * free_dir;
            }
            else
                x[i].setZero();
        }
    }

    
    virtual void enforceConstraint(TVStack& x)
    {
        const int dim = 3;
        for (auto it : m_constraints)
        {
            const btAlignedObjectArray<Constraint>& constraints = it.second;
            size_t i = m_indices[it.first];
            btAssert(constraints.size() <= dim);
            btAssert(constraints.size() > 0);
            if (constraints.size() == 1)
            {
                x[i] -= x[i].dot(constraints[0].m_direction) * constraints[0].m_direction;
                x[i] += constraints[0].m_value * constraints[0].m_direction;
            }
            else if (constraints.size() == 2)
            {
                btVector3 free_dir = btCross(constraints[0].m_direction, constraints[1].m_direction);
                free_dir.normalize();
                x[i] = x[i].dot(free_dir) * free_dir + constraints[0].m_direction * constraints[0].m_value + constraints[1].m_direction * constraints[1].m_value;
            }
            else
                x[i] = constraints[0].m_value * constraints[0].m_direction + constraints[1].m_value * constraints[1].m_direction + constraints[2].m_value * constraints[2].m_direction;
        }
    }
    
    // update the constraints
    virtual void update(const TVStack& dv, const TVStack& backupVelocity);
    
    virtual void setConstraintDirections();
};
#endif /* btContactProjection_h */
