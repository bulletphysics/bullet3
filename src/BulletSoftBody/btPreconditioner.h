//
//  btPreconditioner.h
//  BulletSoftBody
//
//  Created by Xuchen Han on 7/18/19.
//

#ifndef BT_PRECONDITIONER_H
#define BT_PRECONDITIONER_H

class Preconditioner
{
public:
    using TVStack = btAlignedObjectArray<btVector3>;
    virtual void operator()(const TVStack& x, TVStack& b) = 0;
    virtual void reinitialize(bool nodeUpdated) = 0;
};

class DefaultPreconditioner : public Preconditioner
{
public:
    virtual void operator()(const TVStack& x, TVStack& b)
    {
        btAssert(b.size() == x.size());
        for (int i = 0; i < b.size(); ++i)
            b[i] = x[i];
    }
    virtual void reinitialize(bool nodeUpdated)
    {
        
    }
};

class MassPreconditioner : public Preconditioner
{
    btAlignedObjectArray<btScalar> m_inv_mass;
    const btAlignedObjectArray<btSoftBody *>& m_softBodies;
public:
    MassPreconditioner(const btAlignedObjectArray<btSoftBody *>& softBodies)
    : m_softBodies(softBodies)
    {
    }
    
    virtual void reinitialize(bool nodeUpdated)
    {
        if (nodeUpdated)
        {
            m_inv_mass.clear();
            for (int i = 0; i < m_softBodies.size(); ++i)
            {
                btSoftBody* psb = m_softBodies[i];
                for (int j = 0; j < psb->m_nodes.size(); ++j)
                    m_inv_mass.push_back(psb->m_nodes[j].m_im);
            }
        }
    }
    
    virtual void operator()(const TVStack& x, TVStack& b)
    {
        btAssert(b.size() == x.size());
        btAssert(m_inv_mass.size() == x.size());
        for (int i = 0; i < b.size(); ++i)
            b[i] = x[i] * m_inv_mass[i];
    }
};

#endif /* BT_PRECONDITIONER_H */
