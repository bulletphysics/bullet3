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
        for (auto& it : m_constraints)
        {
            const btAlignedObjectArray<Constraint>& constraints = it.second;
            size_t i = m_indices[it.first];
            const Friction& friction = m_frictions[it.first];
            btAssert(constraints.size() <= dim);
            btAssert(constraints.size() > 0);
            if (constraints.size() == 1)
            {
                x[i] -= x[i].dot(constraints[0].m_direction[0]) * constraints[0].m_direction[0];
                if (friction.m_direction.norm() > SIMD_EPSILON)
                {
                    btVector3 dir = friction.m_direction.normalized();
                    x[i] -= x[i].dot(dir) * dir;
                }
            }
            else if (constraints.size() == 2)
            {
                // TODO : friction
                btVector3 free_dir = btCross(constraints[0].m_direction[0], constraints[1].m_direction[0]);
                btAssert(free_dir.norm() > SIMD_EPSILON)
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
        for (auto& it : m_constraints)
        {
            const btAlignedObjectArray<Constraint>& constraints = it.second;
            size_t i = m_indices[it.first];
            const Friction& friction = m_frictions[it.first];
            btAssert(constraints.size() <= dim);
            btAssert(constraints.size() > 0);
            if (constraints.size() == 1)
            {
                x[i] -= x[i].dot(constraints[0].m_direction[0]) * constraints[0].m_direction[0];
                for (int j = 0; j < constraints[0].m_direction.size(); ++j)
                    x[i] += constraints[0].m_value[j] * constraints[0].m_direction[j];
                if (friction.m_direction.norm() > SIMD_EPSILON)
                {
                    btVector3 dir = friction.m_direction.normalized();
                    x[i] -= x[i].dot(dir) * dir;
                    x[i] += friction.m_dv;
                }
            }
            else if (constraints.size() == 2)
            {
                btVector3 free_dir = btCross(constraints[0].m_direction[0], constraints[1].m_direction[0]);
                btAssert(free_dir.norm() > SIMD_EPSILON)
                free_dir.normalize();
                x[i] = x[i].dot(free_dir) * free_dir;
                for (int j = 0; j < constraints.size(); ++j)
                {
                    for (int k = 0; k < constraints[j].m_direction.size(); ++k)
                    {
                        x[i] += constraints[j].m_value[k] * constraints[j].m_direction[k];
                    }
                }
            }
            else
            {
                x[i].setZero();
                for (int j = 0; j < constraints.size(); ++j)
                {
                    for (int k = 0; k < constraints[j].m_direction.size(); ++k)
                    {
                        x[i] += constraints[j].m_value[k] * constraints[j].m_direction[k];
                    }
                }
            }
        }
    }
    
    // update the constraints
    virtual void update(const TVStack& dv, const TVStack& backupVelocity);
    
    virtual void setConstraintDirections();
};
#endif /* btContactProjection_h */
