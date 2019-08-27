/*
Written by Xuchen Han <xuchenhan2015@u.northwestern.edu>

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

#ifndef BT_NEOHOOKEAN_H
#define BT_NEOHOOKEAN_H

#include "btDeformableLagrangianForce.h"

class btDeformableNeoHookeanForce : public btDeformableLagrangianForce
{
public:
    typedef btAlignedObjectArray<btVector3> TVStack;
    btScalar m_mu, m_lambda;
    btDeformableNeoHookeanForce(): m_mu(1), m_lambda(1)
    {
        
    }
    
    btDeformableNeoHookeanForce(btScalar mu, btScalar lambda): m_mu(mu), m_lambda(lambda)
    {
    }
    
    virtual void addScaledImplicitForce(btScalar scale, TVStack& force)
    {
        addScaledDampingForce(scale, force);
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
                btMatrix3x3 P;
                firstPiola(tetra.m_F,P);
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
        btMatrix3x3 C = F.transpose()*F;
        btScalar J = F.determinant();
        btScalar trace = C[0].getX() + C[1].getY() + C[2].getZ();
        P = F * m_mu * ( 1. - 1. / (trace + 1.)) + F.adjoint().transpose() * (m_lambda * (J - 1) - 0.75 * m_mu);
    }
    
    virtual void addScaledForceDifferential(btScalar scale, const TVStack& dv, TVStack& df)
    {
    }
    
    void firstPiolaDifferential(const btMatrix3x3& F, const btMatrix3x3& dF,  btMatrix3x3& dP)
    {
        btScalar J = F.determinant();
        addScaledCofactorMatrixDifferential(F, dF, m_lambda*(J-1) - 0.75*m_mu, dP);
        dP += F.adjoint().transpose() * m_lambda * DotProduct(F.adjoint().transpose(), dF);
        
        //todo @xuchenhan: add the dP of the m_mu term.
    }
    
    btScalar DotProduct(const btMatrix3x3& A, const btMatrix3x3& B)
    {
        btScalar ans = 0;
        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                ans += A[i][j] * B[i][j];
            }
        }
        return ans;
    }
    
    void addScaledCofactorMatrixDifferential(const btMatrix3x3& F, const btMatrix3x3& dF, btScalar scale, btMatrix3x3& M)
    {
        M[0][0] = scale * (dF[1][1] * F[2][2] + F[1][1] * dF[2][2] - dF[2][1] * F[1][2] - F[2][1] * dF[1][2]);
        M[1][0] = scale * (dF[2][1] * F[0][2] + F[2][1] * dF[0][2] - dF[0][1] * F[2][2] - F[0][1] * dF[2][2]);
        M[2][0] = scale * (dF[0][1] * F[1][2] + F[0][1] * dF[1][2] - dF[1][1] * F[0][2] - F[1][1] * dF[0][2]);
        M[0][1] = scale * (dF[2][0] * F[1][2] + F[2][0] * dF[1][2] - dF[1][0] * F[2][2] - F[1][0] * dF[2][2]);
        M[1][1] = scale * (dF[0][0] * F[2][2] + F[0][0] * dF[2][2] - dF[2][0] * F[0][2] - F[2][0] * dF[0][2]);
        M[2][1] = scale * (dF[1][0] * F[0][2] + F[1][0] * dF[0][2] - dF[0][0] * F[1][2] - F[0][0] * dF[1][2]);
        M[0][2] = scale * (dF[1][0] * F[2][1] + F[1][0] * dF[2][1] - dF[2][0] * F[1][1] - F[2][0] * dF[1][1]);
        M[1][2] = scale * (dF[2][0] * F[0][1] + F[2][0] * dF[0][1] - dF[0][0] * F[2][1] - F[0][0] * dF[2][1]);
        M[2][2] = scale * (dF[0][0] * F[1][1] + F[0][0] * dF[1][1] - dF[1][0] * F[0][1] - F[1][0] * dF[0][1]);
    }
    
    virtual btDeformableLagrangianForceType getForceType()
    {
        return BT_NEOHOOKEAN_FORCE;
    }
    
};
#endif /* BT_NEOHOOKEAN_H */
