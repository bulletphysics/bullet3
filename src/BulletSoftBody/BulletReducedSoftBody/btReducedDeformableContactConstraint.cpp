#include "btReducedDeformableContactConstraint.h"
#include <iostream>

// ================= static constraints ===================
btReducedDeformableStaticConstraint::btReducedDeformableStaticConstraint(
  btReducedSoftBody* rsb, 
  btSoftBody::Node* node,
	const btVector3& ri,
  const btContactSolverInfo& infoGlobal,
	btScalar dt)
  : m_rsb(rsb), m_ri(ri), m_dt(dt), btDeformableStaticConstraint(node, infoGlobal)
{
	// get impulse
  m_impulseFactor = rsb->getImpulseFactor(m_node->index);
}

btScalar btReducedDeformableStaticConstraint::solveConstraint(const btContactSolverInfo& infoGlobal)
{
	// target velocity of fixed constraint is 0
  btVector3 impulse = -(m_impulseFactor.inverse() * m_node->m_v);
  
  // apply full space impulse
	applyImpulse(impulse);

	return 0;
}
  
// this calls reduced deformable body's applyFullSpaceImpulse
void btReducedDeformableStaticConstraint::applyImpulse(const btVector3& impulse)
{
	m_rsb->applyFullSpaceImpulse(impulse, m_ri, m_node->index, m_dt);
}


// ================= base contact constraints ===================
btReducedDeformableRigidContactConstraint::btReducedDeformableRigidContactConstraint(
  btReducedSoftBody* rsb, 
  const btSoftBody::DeformableRigidContact& c, 
  const btContactSolverInfo& infoGlobal,
	btScalar dt)
  : m_rsb(rsb), m_dt(dt), btDeformableRigidContactConstraint(c, infoGlobal)
{}

btScalar btReducedDeformableRigidContactConstraint::solveConstraint(const btContactSolverInfo& infoGlobal)
{
  const btSoftBody::sCti& cti = m_contact->m_cti;
	btVector3 va = getVa();
	btVector3 vb = getVb();
	btVector3 vr = vb - va;
	btScalar dn = btDot(vr, cti.m_normal) + m_total_normal_dv.dot(cti.m_normal) * infoGlobal.m_deformable_cfm;
	if (m_penetration > 0)
	{
		dn += m_penetration / infoGlobal.m_timeStep;
	}
	if (!infoGlobal.m_splitImpulse)
	{
		dn += m_penetration * infoGlobal.m_deformable_erp / infoGlobal.m_timeStep;
	}
	// dn is the normal component of velocity diffrerence. Approximates the residual.
	btVector3 impulse = m_contact->m_c0 * (vr + m_total_normal_dv * infoGlobal.m_deformable_cfm + ((m_penetration > 0) ? m_penetration / infoGlobal.m_timeStep * cti.m_normal : btVector3(0, 0, 0)));
	if (!infoGlobal.m_splitImpulse)
	{
		impulse += m_contact->m_c0 * (m_penetration * infoGlobal.m_deformable_erp / infoGlobal.m_timeStep * cti.m_normal);
	}
	btVector3 impulse_normal = m_contact->m_c0 * (cti.m_normal * dn);
	btVector3 impulse_tangent = impulse - impulse_normal;
	if (dn > 0)
	{
		return 0;
	}
	m_binding = true;
	btScalar residualSquare = dn * dn;
	btVector3 old_total_tangent_dv = m_total_tangent_dv;
	// m_c5 is the inverse mass of the deformable node/face
	m_total_normal_dv -= m_contact->m_c5 * impulse_normal;
	m_total_tangent_dv -= m_contact->m_c5 * impulse_tangent;

	if (m_total_normal_dv.dot(cti.m_normal) < 0)
	{
		// separating in the normal direction
		m_binding = false;
		m_static = false;
		impulse_tangent.setZero();
	}
	else
	{
		if (m_total_normal_dv.norm() * m_contact->m_c3 < m_total_tangent_dv.norm())
		{
			// dynamic friction
			// with dynamic friction, the impulse are still applied to the two objects colliding, however, it does not pose a constraint in the cg solve, hence the change to dv merely serves to update velocity in the contact iterations.
			m_static = false;
			if (m_total_tangent_dv.safeNorm() < SIMD_EPSILON)
			{
				m_total_tangent_dv = btVector3(0, 0, 0);
			}
			else
			{
				m_total_tangent_dv = m_total_tangent_dv.normalized() * m_total_normal_dv.safeNorm() * m_contact->m_c3;
			}
			//            impulse_tangent = -btScalar(1)/m_contact->m_c2 * (m_total_tangent_dv - old_total_tangent_dv);
			impulse_tangent = m_contact->m_c5.inverse() * (old_total_tangent_dv - m_total_tangent_dv);
		}
		else
		{
			// static friction
			m_static = true;
		}
	}
	impulse = impulse_normal + impulse_tangent;
	// apply impulse to deformable nodes involved and change their velocities
	applyImpulse(impulse); // TODO: apply impulse?
	// apply impulse to the rigid/multibodies involved and change their velocities
	if (cti.m_colObj->getInternalType() == btCollisionObject::CO_RIGID_BODY)
	{
		btRigidBody* rigidCol = 0;
		rigidCol = (btRigidBody*)btRigidBody::upcast(cti.m_colObj);
		if (rigidCol)
		{
			rigidCol->applyImpulse(impulse, m_contact->m_c1);
		}
	}
	else if (cti.m_colObj->getInternalType() == btCollisionObject::CO_FEATHERSTONE_LINK)
	{
		btMultiBodyLinkCollider* multibodyLinkCol = 0;
		multibodyLinkCol = (btMultiBodyLinkCollider*)btMultiBodyLinkCollider::upcast(cti.m_colObj);
		if (multibodyLinkCol)
		{
			const btScalar* deltaV_normal = &m_contact->jacobianData_normal.m_deltaVelocitiesUnitImpulse[0];
			// apply normal component of the impulse
			multibodyLinkCol->m_multiBody->applyDeltaVeeMultiDof2(deltaV_normal, impulse.dot(cti.m_normal));
			if (impulse_tangent.norm() > SIMD_EPSILON)
			{
				// apply tangential component of the impulse
				const btScalar* deltaV_t1 = &m_contact->jacobianData_t1.m_deltaVelocitiesUnitImpulse[0];
				multibodyLinkCol->m_multiBody->applyDeltaVeeMultiDof2(deltaV_t1, impulse.dot(m_contact->t1));
				const btScalar* deltaV_t2 = &m_contact->jacobianData_t2.m_deltaVelocitiesUnitImpulse[0];
				multibodyLinkCol->m_multiBody->applyDeltaVeeMultiDof2(deltaV_t2, impulse.dot(m_contact->t2));
			}
		}
	}
	return residualSquare;
}

// ================= node vs rigid constraints ===================
btReducedDeformableNodeRigidContactConstraint::btReducedDeformableNodeRigidContactConstraint(
  btReducedSoftBody* rsb, 
  const btSoftBody::DeformableNodeRigidContact& contact, 
  const btContactSolverInfo& infoGlobal,
	btScalar dt)
  : m_node(contact.m_node), btReducedDeformableRigidContactConstraint(rsb, contact, infoGlobal, dt)
{}

btVector3 btReducedDeformableNodeRigidContactConstraint::getVb() const
{
	return m_node->m_v;
}

btVector3 btReducedDeformableNodeRigidContactConstraint::getSplitVb() const
{
	return m_node->m_splitv;
}

btVector3 btReducedDeformableNodeRigidContactConstraint::getDv(const btSoftBody::Node* node) const
{
	return m_total_normal_dv + m_total_tangent_dv;
}

void btReducedDeformableNodeRigidContactConstraint::applyImpulse(const btVector3& impulse)
{
  m_rsb->applyFullSpaceImpulse(impulse, m_contact->m_c1, m_node->index, m_dt);
}

// ================= face vs rigid constraints ===================
btReducedDeformableFaceRigidContactConstraint::btReducedDeformableFaceRigidContactConstraint(
  btReducedSoftBody* rsb, 
  const btSoftBody::DeformableFaceRigidContact& contact, 
  const btContactSolverInfo& infoGlobal,
	btScalar dt, 
  bool useStrainLimiting)
  : m_face(contact.m_face), m_useStrainLimiting(useStrainLimiting), btReducedDeformableRigidContactConstraint(rsb, contact, infoGlobal, dt)
{}

btVector3 btReducedDeformableFaceRigidContactConstraint::getVb() const
{
	const btSoftBody::DeformableFaceRigidContact* contact = getContact();
	btVector3 vb = m_face->m_n[0]->m_v * contact->m_bary[0] + m_face->m_n[1]->m_v * contact->m_bary[1] + m_face->m_n[2]->m_v * contact->m_bary[2];
	return vb;
}

btVector3 btReducedDeformableFaceRigidContactConstraint::getSplitVb() const
{
	const btSoftBody::DeformableFaceRigidContact* contact = getContact();
	btVector3 vb = (m_face->m_n[0]->m_splitv) * contact->m_bary[0] + (m_face->m_n[1]->m_splitv) * contact->m_bary[1] + (m_face->m_n[2]->m_splitv) * contact->m_bary[2];
	return vb;
}

btVector3 btReducedDeformableFaceRigidContactConstraint::getDv(const btSoftBody::Node* node) const
{
	btVector3 face_dv = m_total_normal_dv + m_total_tangent_dv;
	const btSoftBody::DeformableFaceRigidContact* contact = getContact();
	if (m_face->m_n[0] == node)
	{
		return face_dv * contact->m_weights[0];
	}
	if (m_face->m_n[1] == node)
	{
		return face_dv * contact->m_weights[1];
	}
	btAssert(node == m_face->m_n[2]);
	return face_dv * contact->m_weights[2];
}

void btReducedDeformableFaceRigidContactConstraint::applyImpulse(const btVector3& impulse)
{
  //
}