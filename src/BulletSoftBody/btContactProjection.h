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
        for (int i = 0; i < x.size(); ++i)
        {
            for (int j = 0; j < m_constrainedDirections[i].size(); ++j)
            {
                x[i] -= x[i].dot(m_constrainedDirections[i][j]) * m_constrainedDirections[i][j];
            }
        }
    }
    
    virtual void enforceConstraint(TVStack& x)
    {
        for (int i = 0; i < x.size(); ++i)
        {
            for (int j = 0; j < m_constrainedDirections[i].size(); ++j)
            {
                x[i] -= x[i].dot(m_constrainedDirections[i][j]) * m_constrainedDirections[i][j];
                x[i] += m_constrainedValues[i][j] * m_constrainedDirections[i][j];
            }
        }
    }
    
    // update the constraints
    virtual void update(const TVStack& dv, const TVStack& backupVelocity);
    
    virtual void setConstraintDirections();
};
#endif /* btContactProjection_h */
