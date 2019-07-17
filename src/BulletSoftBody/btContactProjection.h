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
            btAlignedObjectArray<Friction>& frictions = m_frictions[it.first];
            btAssert(constraints.size() <= dim);
            btAssert(constraints.size() > 0);
            if (constraints.size() == 1)
            {
                x[i] -= x[i].dot(constraints[0].m_direction[0]) * constraints[0].m_direction[0];
                Friction& friction= frictions[0];
                
                bool has_static_constraint = false;
                for (int j = 0; j < friction.m_static.size(); ++j)
                    has_static_constraint = has_static_constraint || friction.m_static[j];
                
                for (int j = 0; j < friction.m_direction.size(); ++j)
                {
                    // clear the old friction force
                    if (friction.m_static_prev[j] == false)
                    {
                        x[i] -= friction.m_direction_prev[j] * friction.m_impulse_prev[j];
                    }
                    
                    // only add to the rhs if there is no static friction constraint on the node
                    if (friction.m_static[j] == false && !has_static_constraint)
                    {
                        x[i] += friction.m_direction[j] * friction.m_impulse[j];
                    }
                }
            }
            else if (constraints.size() == 2)
            {
                // TODO : friction
                btVector3 free_dir = btCross(constraints[0].m_direction[0], constraints[1].m_direction[0]);
                btAssert(free_dir.norm() > SIMD_EPSILON)
                free_dir.normalize();
                x[i] = x[i].dot(free_dir) * free_dir;
                
                bool has_static_constraint = false;
                for (int f = 0; f < 2; ++f)
                {
                    Friction& friction= frictions[f];
                    for (int j = 0; j < friction.m_static.size(); ++j)
                        has_static_constraint = has_static_constraint || friction.m_static[j];
                }
                
                for (int f = 0; f < 2; ++f)
                {
                    Friction& friction= frictions[f];
                    for (int j = 0; j < friction.m_direction.size(); ++j)
                    {
                        // clear the old friction force
                        if (friction.m_static_prev[j] == false)
                        {
                            x[i] -= friction.m_direction_prev[j] * friction.m_impulse_prev[j];
                        }
                        
                        // only add to the rhs if there is no static friction constraint on the node
                        if (friction.m_static[j] == false && !has_static_constraint)
                        {
                            x[i] += friction.m_direction[j] * friction.m_impulse[j];
                        }
                    }
                }
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
            const btAlignedObjectArray<Friction>& frictions = m_frictions[it.first];
            btAssert(constraints.size() <= dim);
            btAssert(constraints.size() > 0);
            if (constraints.size() == 1)
            {
                x[i] -= x[i].dot(constraints[0].m_direction[0]) * constraints[0].m_direction[0];
                for (int j = 0; j < constraints[0].m_direction.size(); ++j)
                    x[i] += constraints[0].m_value[j] * constraints[0].m_direction[j];
                
                const Friction& friction= frictions[0];
                for (int j = 0; j < friction.m_direction.size(); ++j)
                {
                    // clear the old constraint
                    if (friction.m_static_prev[j] == true)
                    {
                        x[i] -= friction.m_direction_prev[j] * friction.m_dv_prev[j];
                    }
                    // add the new constraint
                    if (friction.m_static[j] == true)
                    {
                        x[i] += friction.m_direction[j] * friction.m_dv[j];
                    }
                }
            }
            else if (constraints.size() == 2)
            {
                // TODO: friction
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
                
                for (int f = 0; f < 2; ++f)
                {
                    const Friction& friction= frictions[f];
                    for (int j = 0; j < friction.m_direction.size(); ++j)
                    {
                        // clear the old constraint
                        if (friction.m_static_prev[j] == true)
                        {
                            x[i] -= friction.m_direction_prev[j] * friction.m_dv_prev[j];
                        }
                        // add the new constraint
                        if (friction.m_static[j] == true)
                        {
                            x[i] += friction.m_direction[j] * friction.m_dv[j];
                        }
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
