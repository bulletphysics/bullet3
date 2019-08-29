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
    btScalar m_mu_damp, m_lambda_damp;
    btDeformableNeoHookeanForce(): m_mu(1), m_lambda(1)
    {
        btScalar damping = 0.005;
        m_mu_damp = damping * m_mu;
        m_lambda_damp = damping * m_lambda;
    }
    
    btDeformableNeoHookeanForce(btScalar mu, btScalar lambda, btScalar damping = 0.005): m_mu(mu), m_lambda(lambda)
    {
        m_mu_damp = damping * m_mu;
        m_lambda_damp = damping * m_lambda;
    }
    
    virtual void addScaledForces(btScalar scale, TVStack& force)
    {
        addScaledDampingForce(scale, force);
        addScaledElasticForce(scale, force);
    }
    
    virtual void addScaledExplicitForce(btScalar scale, TVStack& force)
    {
        addScaledElasticForce(scale, force);
    }
    
    virtual void addScaledDampingForce(btScalar scale, TVStack& force)
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
                btSoftBody::Node* node0 = tetra.m_n[0];
                btSoftBody::Node* node1 = tetra.m_n[1];
                btSoftBody::Node* node2 = tetra.m_n[2];
                btSoftBody::Node* node3 = tetra.m_n[3];
                size_t id0 = node0->index;
                size_t id1 = node1->index;
                size_t id2 = node2->index;
                size_t id3 = node3->index;
                btMatrix3x3 dF = DsFromVelocity(node0, node1, node2, node3) * tetra.m_Dm_inverse;
                btMatrix3x3 dP;
                firstPiolaDampingDifferential(tetra.m_F, dF, dP);
                btVector3 df_on_node0 = dP * (tetra.m_Dm_inverse.transpose()*grad_N_hat_1st_col);
                btMatrix3x3 df_on_node123 = dP * tetra.m_Dm_inverse.transpose();
                
                // damping force differential
                btScalar scale1 = scale * tetra.m_element_measure;
                force[id0] -= scale1 * df_on_node0;
                force[id1] -= scale1 * df_on_node123.getColumn(0);
                force[id2] -= scale1 * df_on_node123.getColumn(1);
                force[id3] -= scale1 * df_on_node123.getColumn(2);
            }
        }
    }
    
    virtual double totalElasticEnergy()
    {
        double energy = 0;
        for (int i = 0; i < m_softBodies.size(); ++i)
        {
            btSoftBody* psb = m_softBodies[i];
            for (int j = 0; j < psb->m_tetras.size(); ++j)
            {
                btSoftBody::Tetra& tetra = psb->m_tetras[j];
                energy += tetra.m_element_measure * elasticEnergyDensity(tetra);
            }
        }
        return energy;
    }
    
    double elasticEnergyDensity(const btSoftBody::Tetra& t)
    {
        double density = 0;
        btMatrix3x3 F = t.m_F;
        btMatrix3x3 C = F.transpose()*F;
        double J = F.determinant();
        double trace = C[0].getX() + C[1].getY() + C[2].getZ();
        density += m_mu * 0.5 * (trace - 3.);
        density += m_lambda * 0.5 * (J - 1. - 0.75 * m_mu / m_lambda)* (J - 1. - 0.75 * m_mu / m_lambda);
        density -= m_mu * 0.5 * log(trace+1);
        return density;
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
                btScalar scale1 = scale * tetra.m_element_measure;
                force[id0] -= scale1 * force_on_node0;
                force[id1] -= scale1 * force_on_node123.getColumn(0);
                force[id2] -= scale1 * force_on_node123.getColumn(1);
                force[id3] -= scale1 * force_on_node123.getColumn(2);
            }
        }
    }
    
    virtual void addScaledDampingForceDifferential(btScalar scale, const TVStack& dv, TVStack& df)
    {
        int numNodes = getNumNodes();
        btAssert(numNodes <= df.size())
        btVector3 grad_N_hat_1st_col = btVector3(-1,-1,-1);
        for (int i = 0; i < m_softBodies.size(); ++i)
        {
            btSoftBody* psb = m_softBodies[i];
            for (int j = 0; j < psb->m_tetras.size(); ++j)
            {
                btSoftBody::Tetra& tetra = psb->m_tetras[j];
                btSoftBody::Node* node0 = tetra.m_n[0];
                btSoftBody::Node* node1 = tetra.m_n[1];
                btSoftBody::Node* node2 = tetra.m_n[2];
                btSoftBody::Node* node3 = tetra.m_n[3];
                size_t id0 = node0->index;
                size_t id1 = node1->index;
                size_t id2 = node2->index;
                size_t id3 = node3->index;
                btMatrix3x3 dF = Ds(id0, id1, id2, id3, dv) * tetra.m_Dm_inverse;
                btMatrix3x3 dP;
                firstPiolaDampingDifferential(tetra.m_F, dF, dP);
                btVector3 df_on_node0 = dP * (tetra.m_Dm_inverse.transpose()*grad_N_hat_1st_col);
                btMatrix3x3 df_on_node123 = dP * tetra.m_Dm_inverse.transpose();
                
                // damping force differential
                btScalar scale1 = scale * tetra.m_element_measure;
                df[id0] -= scale1 * df_on_node0;
                df[id1] -= scale1 * df_on_node123.getColumn(0);
                df[id2] -= scale1 * df_on_node123.getColumn(1);
                df[id3] -= scale1 * df_on_node123.getColumn(2);
            }
        }
    }
    
    virtual void addScaledElasticForceDifferential(btScalar scale, const TVStack& dx, TVStack& df)
    {
        int numNodes = getNumNodes();
        btAssert(numNodes <= df.size())
        btVector3 grad_N_hat_1st_col = btVector3(-1,-1,-1);
        for (int i = 0; i < m_softBodies.size(); ++i)
        {
            btSoftBody* psb = m_softBodies[i];
            for (int j = 0; j < psb->m_tetras.size(); ++j)
            {
                btSoftBody::Tetra& tetra = psb->m_tetras[j];
                btSoftBody::Node* node0 = tetra.m_n[0];
                btSoftBody::Node* node1 = tetra.m_n[1];
                btSoftBody::Node* node2 = tetra.m_n[2];
                btSoftBody::Node* node3 = tetra.m_n[3];
                size_t id0 = node0->index;
                size_t id1 = node1->index;
                size_t id2 = node2->index;
                size_t id3 = node3->index;
                btMatrix3x3 dF = Ds(id0, id1, id2, id3, dx) * tetra.m_Dm_inverse;
                btMatrix3x3 dP;
                firstPiolaDifferential(tetra.m_F, dF, dP);
                btVector3 df_on_node0 = dP * (tetra.m_Dm_inverse.transpose()*grad_N_hat_1st_col);
                btMatrix3x3 df_on_node123 = dP * tetra.m_Dm_inverse.transpose();
                
                // elastic force differential
                btScalar scale1 = scale * tetra.m_element_measure;
                df[id0] -= scale1 * df_on_node0;
                df[id1] -= scale1 * df_on_node123.getColumn(0);
                df[id2] -= scale1 * df_on_node123.getColumn(1);
                df[id3] -= scale1 * df_on_node123.getColumn(2);
            }
        }
    }
    
    void firstPiola(const btMatrix3x3& F, btMatrix3x3& P)
    {
        btMatrix3x3 C = F.transpose()*F;
        btScalar J = F.determinant();
        btScalar trace = C[0].getX() + C[1].getY() + C[2].getZ();
        P = F * m_mu * ( 1. - 1. / (trace + 1.)) + F.adjoint().transpose() * (m_lambda * (J - 1.) - 0.75 * m_mu);
    }
    
    void firstPiolaDifferential(const btMatrix3x3& F, const btMatrix3x3& dF,  btMatrix3x3& dP)
    {
        btScalar J = F.determinant();
        btMatrix3x3 C = F.transpose()*F;
        btScalar trace = C[0].getX() + C[1].getY() + C[2].getZ();
        dP = dF * m_mu * ( 1. - 1. / (trace + 1.)) + F * (2*m_mu) * DotProduct(F, dF) * (1./((1.+trace)*(1.+trace)));
        
        addScaledCofactorMatrixDifferential(F, dF, m_lambda*(J-1.) - 0.75*m_mu, dP);
        dP += F.adjoint().transpose() * m_lambda * DotProduct(F.adjoint().transpose(), dF);
    }
    
    void firstPiolaDampingDifferential(const btMatrix3x3& F, const btMatrix3x3& dF,  btMatrix3x3& dP)
    {
        btScalar J = F.determinant();
        btMatrix3x3 C = F.transpose()*F;
        btScalar trace = C[0].getX() + C[1].getY() + C[2].getZ();
        dP = dF * m_mu_damp * ( 1. - 1. / (trace + 1.)) + F * (2*m_mu_damp) * DotProduct(F, dF) * (1./((1.+trace)*(1.+trace)));
        
        addScaledCofactorMatrixDifferential(F, dF, m_lambda_damp*(J-1.) - 0.75*m_mu_damp, dP);
        dP += F.adjoint().transpose() * m_lambda_damp * DotProduct(F.adjoint().transpose(), dF);
    }
    
    btScalar DotProduct(const btMatrix3x3& A, const btMatrix3x3& B)
    {
        btScalar ans = 0;
        for (int i = 0; i < 3; ++i)
        {
            ans += A[i].dot(B[i]);
        }
        return ans;
    }
    
    void addScaledCofactorMatrixDifferential(const btMatrix3x3& F, const btMatrix3x3& dF, btScalar scale, btMatrix3x3& M)
    {
        M[0][0] += scale * (dF[1][1] * F[2][2] + F[1][1] * dF[2][2] - dF[2][1] * F[1][2] - F[2][1] * dF[1][2]);
        M[1][0] += scale * (dF[2][1] * F[0][2] + F[2][1] * dF[0][2] - dF[0][1] * F[2][2] - F[0][1] * dF[2][2]);
        M[2][0] += scale * (dF[0][1] * F[1][2] + F[0][1] * dF[1][2] - dF[1][1] * F[0][2] - F[1][1] * dF[0][2]);
        M[0][1] += scale * (dF[2][0] * F[1][2] + F[2][0] * dF[1][2] - dF[1][0] * F[2][2] - F[1][0] * dF[2][2]);
        M[1][1] += scale * (dF[0][0] * F[2][2] + F[0][0] * dF[2][2] - dF[2][0] * F[0][2] - F[2][0] * dF[0][2]);
        M[2][1] += scale * (dF[1][0] * F[0][2] + F[1][0] * dF[0][2] - dF[0][0] * F[1][2] - F[0][0] * dF[1][2]);
        M[0][2] += scale * (dF[1][0] * F[2][1] + F[1][0] * dF[2][1] - dF[2][0] * F[1][1] - F[2][0] * dF[1][1]);
        M[1][2] += scale * (dF[2][0] * F[0][1] + F[2][0] * dF[0][1] - dF[0][0] * F[2][1] - F[0][0] * dF[2][1]);
        M[2][2] += scale * (dF[0][0] * F[1][1] + F[0][0] * dF[1][1] - dF[1][0] * F[0][1] - F[1][0] * dF[0][1]);
    }
    
    virtual btDeformableLagrangianForceType getForceType()
    {
        return BT_NEOHOOKEAN_FORCE;
    }
    
};
#endif /* BT_NEOHOOKEAN_H */
