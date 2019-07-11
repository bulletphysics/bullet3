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
        for (int j = 0; j < m_constrainedId.size(); ++j)
        {
            int i = m_constrainedId[j];
            btAssert(m_constrainedDirections[i].size() <= dim);
            if (m_constrainedDirections[i].size() <= 1)
            {
                for (int j = 0; j < m_constrainedDirections[i].size(); ++j)
                {
                    x[i] -= x[i].dot(m_constrainedDirections[i][j]) * m_constrainedDirections[i][j];
                }
            }
            else if (m_constrainedDirections[i].size() == 2)
            {
                btVector3 free_dir = btCross(m_constrainedDirections[i][0], m_constrainedDirections[i][1]);
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
        for (int j = 0; j < m_constrainedId.size(); ++j)
        {
            int i = m_constrainedId[j];
            btAssert(m_constrainedDirections[i].size() <= dim);
            if (m_constrainedDirections[i].size() <= 1)
            {
                for (int j = 0; j < m_constrainedDirections[i].size(); ++j)
                {
                    x[i] -= x[i].dot(m_constrainedDirections[i][j]) * m_constrainedDirections[i][j];
                    x[i] += m_constrainedValues[i][j] * m_constrainedDirections[i][j];
                }
            }
            else if (m_constrainedDirections[i].size() == 2)
            {
                btVector3 free_dir = btCross(m_constrainedDirections[i][0], m_constrainedDirections[i][1]);
                free_dir.normalize();
                x[i] = x[i].dot(free_dir) * free_dir + m_constrainedDirections[i][0] * m_constrainedValues[i][0] + m_constrainedDirections[i][1] * m_constrainedValues[i][1];
            }
            else
                x[i] = m_constrainedDirections[i][0] * m_constrainedValues[i][0] + m_constrainedDirections[i][1] * m_constrainedValues[i][1] + m_constrainedDirections[i][2] * m_constrainedValues[i][2];
        }
    }
    
    // update the constraints
    virtual void update(const TVStack& dv, const TVStack& backupVelocity);
    
    virtual void setConstraintDirections();
};
#endif /* btContactProjection_h */
