/*
 Bullet Continuous Collision Detection and Physics Library
 Copyright (c) 2016 Google Inc. http://bulletphysics.org
 This software is provided 'as-is', without any express or implied warranty.
 In no event will the authors be held liable for any damages arising from the use of this software.
 Permission is granted to anyone to use this software for any purpose,
 including commercial applications, and to alter it and redistribute it freely,
 subject to the following restrictions:
 1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
 2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
 3. This notice may not be removed or altered from any source distribution.
 */

#ifndef BT_COROTATED_H
#define BT_COROTATED_H

#include "btDeformableLagrangianForce.h"
#include "LinearMath/btPolarDecomposition.h"

static inline int PolarDecompose(const btMatrix3x3& m, btMatrix3x3& q, btMatrix3x3& s)
{
    static const btPolarDecomposition polar;
    return polar.decompose(m, q, s);
}

class btDeformableCorotatedForce : public btDeformableLagrangianForce
{
public:
    typedef btAlignedObjectArray<btVector3> TVStack;
    btScalar m_mu, m_lambda;
    btDeformableCorotatedForce(): m_mu(1), m_lambda(1)
    {
        
    }
    
    btDeformableCorotatedForce(btScalar mu, btScalar lambda): m_mu(mu), m_lambda(lambda)
    {
    }
    
    virtual void addScaledImplicitForce(btScalar scale, TVStack& force)
    {
    }
    
    virtual void addScaledExplicitForce(btScalar scale, TVStack& force)
    {
        addScaledElasticForce(scale, force);
    }
    
    virtual void addScaledDampingForce(btScalar scale, TVStack& force)
    {
    }
    
    virtual void addScaledElasticForce(btScalar scale, TVStack& force)
    {
        int numNodes = getNumNodes();
        btAssert(numNodes <= force.size())
        btVector3 grad_N_hat_1st_col = btVector3(-1,-1,-1);
        for (int i = 0; i < m_softBodies.size(); ++i)
        {
            btSoftBody* psb = m_softBodies[i];
            for (int j = 0; j < psb->m_tetras.size(); ++j)
            {
                btSoftBody::Tetra& tetra = psb->m_tetras[j];
                updateDs(tetra);
                btMatrix3x3 F = tetra.m_ds * tetra.m_Dm_inverse;
                btMatrix3x3 P;
                firstPiola(F,P);
                btVector3 force_on_node0 = P * (tetra.m_Dm_inverse.transpose()*grad_N_hat_1st_col);
                btMatrix3x3 force_on_node123 = P * tetra.m_Dm_inverse.transpose();
                
                btSoftBody::Node* node0 = tetra.m_n[0];
                btSoftBody::Node* node1 = tetra.m_n[1];
                btSoftBody::Node* node2 = tetra.m_n[2];
                btSoftBody::Node* node3 = tetra.m_n[3];
                size_t id0 = node0->index;
                size_t id1 = node1->index;
                size_t id2 = node2->index;
                size_t id3 = node3->index;
                
                // elastic force
                // explicit elastic force
                btScalar scale1 = scale * tetra.m_element_measure;
                force[id0] -= scale1 * force_on_node0;
                force[id1] -= scale1 * force_on_node123.getColumn(0);
                force[id2] -= scale1 * force_on_node123.getColumn(1);
                force[id3] -= scale1 * force_on_node123.getColumn(2);
            }
        }
    }
    
    void firstPiola(const btMatrix3x3& F, btMatrix3x3& P)
    {
        // btMatrix3x3 JFinvT = F.adjoint();
        btScalar J = F.determinant();
        P =  F.adjoint() * (m_lambda * (J-1));
        if (m_mu > SIMD_EPSILON)
        {
            btMatrix3x3 R,S;
            if (J < 1024 * SIMD_EPSILON)
                R.setIdentity();
            else
                PolarDecompose(F, R, S); // this QR is not robust, consider using implicit shift svd
            /*https://fuchuyuan.github.io/research/svd/paper.pdf*/
            P += (F-R) * 2 * m_mu;
        }
    }
    void updateDs(btSoftBody::Tetra& t)
    {
        btVector3 c1 = t.m_n[1]->m_q - t.m_n[0]->m_q;
        btVector3 c2 = t.m_n[2]->m_q - t.m_n[0]->m_q;
        btVector3 c3 = t.m_n[3]->m_q - t.m_n[0]->m_q;
        btMatrix3x3 Ds(c1.getX(), c2.getX(), c3.getX(),
                       c1.getY(), c2.getY(), c3.getY(),
                       c1.getZ(), c2.getZ(), c3.getZ());
        t.m_ds = Ds;
    }
    
    virtual void addScaledForceDifferential(btScalar scale, const TVStack& dv, TVStack& df)
    {
        // implicit damping force differential
        for (int i = 0; i < m_softBodies.size(); ++i)
        {
            const btSoftBody* psb = m_softBodies[i];
            btScalar scaled_k_damp = psb->m_dampingCoefficient * scale;
            for (int j = 0; j < psb->m_links.size(); ++j)
            {
                const btSoftBody::Link& link = psb->m_links[j];
                btSoftBody::Node* node1 = link.m_n[0];
                btSoftBody::Node* node2 = link.m_n[1];
                size_t id1 = node1->index;
                size_t id2 = node2->index;
                btVector3 local_scaled_df = scaled_k_damp * (dv[id2] - dv[id1]);
                df[id1] += local_scaled_df;
                df[id2] -= local_scaled_df;
            }
        }
    }
    
    virtual btDeformableLagrangianForceType getForceType()
    {
        return BT_COROTATED_FORCE;
    }
    
};


#endif /* btCorotated_h */
