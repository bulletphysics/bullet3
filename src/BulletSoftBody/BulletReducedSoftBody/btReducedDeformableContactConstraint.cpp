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
	std::cout << "node: " << m_node->index << " impulse: " << impulse[0] << '\t' << impulse[1] << '\t' << impulse[2] << '\n';
	// std::cout << "impulse norm: " << impulse.norm() << "\n";

	m_rsb->applyFullSpaceImpulse(impulse, m_ri, m_node->index, m_dt);

	// get residual //TODO: only calculate the velocity of the given node
	m_rsb->mapToFullVelocity(m_rsb->getInterpolationWorldTransform());

	// calculate residual
	btScalar residualSquare = btDot(m_node->m_v, m_node->m_v);

	return residualSquare;
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

	// btVector3 normal(0, 1, 0);
	
	// for (int p = 0; p < m_rsb->m_nFull; ++p)
	// {
	// 	for (int k = 0; k < 3; ++k)
	// 	{
	// 		std::cout << m_rsb->m_nodes[p].m_x[k] << '\t';
	// 	}
	// 	std::cout << '\n';
	// }

	// get relative velocity and magnitude
	btVector3 v_rel = vb - va;
	btScalar v_rel_normal = btDot(v_rel, cti.m_normal);
	if (m_penetration > 0)
	{
		std::cout << "penetrate!!!!\n";
		v_rel_normal += m_penetration / infoGlobal.m_timeStep;		// add penetration correction vel
	}
	// if (!infoGlobal.m_splitImpulse)
	// {
	// 	dn += m_penetration * infoGlobal.m_deformable_erp / infoGlobal.m_timeStep;
	// }
	
	// if it's separating, no need to do anything
	if (v_rel_normal > 0)
	{
		return 0;
	}
	btScalar residualSquare = v_rel_normal * v_rel_normal;	// get residual

	// compute the tangential relative vel
	btVector3 v_rel_tangent = v_rel - v_rel_normal * cti.m_normal;

	// friction correction
	btScalar delta_v_rel_normal = v_rel_normal;
	btScalar delta_v_rel_tangent = m_contact->m_c3 * v_rel_normal;
	// btScalar delta_v_rel_tangent = 0.3 * v_rel_normal;

	btVector3 impulse_tangent(0, 0, 0);
	if (v_rel_tangent.norm() < delta_v_rel_tangent)
	{
		// the object should be static
		impulse_tangent = m_contact->m_c0 * (-v_rel_tangent);
	}
	else
	{
		// apply friction
		impulse_tangent = m_contact->m_c0 * (-v_rel_tangent.safeNormalize() * delta_v_rel_tangent);
		std::cout << "friction called\n";
	}

	// get total impulse
	btVector3 impulse_normal = m_contact->m_c0 * (cti.m_normal * (-v_rel_normal));
	// btVector3 impulse = impulse_normal + impulse_tangent;
	btVector3 impulse = impulse_normal.dot(cti.m_normal) * cti.m_normal;

	std::cout << "impulse direct: " << impulse[0] / cti.m_normal[0] << '\t' << impulse[1] / cti.m_normal[1]<< '\t' << impulse[2] / cti.m_normal[2] << '\n';

	applyImpulse(impulse);
	// applyImpulse(impulse); // TODO: apply impulse?
	// apply impulse to the rigid/multibodies involved and change their velocities
	// if (cti.m_colObj->getInternalType() == btCollisionObject::CO_RIGID_BODY)
	// {
	// 	btRigidBody* rigidCol = 0;
	// 	rigidCol = (btRigidBody*)btRigidBody::upcast(cti.m_colObj);
	// 	if (rigidCol)
	// 	{
	// 		rigidCol->applyImpulse(impulse, m_contact->m_c1);
	// 	}
	// }
	// else if (cti.m_colObj->getInternalType() == btCollisionObject::CO_FEATHERSTONE_LINK)
	// {
	// 	btMultiBodyLinkCollider* multibodyLinkCol = 0;
	// 	multibodyLinkCol = (btMultiBodyLinkCollider*)btMultiBodyLinkCollider::upcast(cti.m_colObj);
	// 	if (multibodyLinkCol)
	// 	{
	// 		const btScalar* deltaV_normal = &m_contact->jacobianData_normal.m_deltaVelocitiesUnitImpulse[0];
	// 		// apply normal component of the impulse
	// 		multibodyLinkCol->m_multiBody->applyDeltaVeeMultiDof2(deltaV_normal, impulse.dot(cti.m_normal));
	// 		if (impulse_tangent.norm() > SIMD_EPSILON)
	// 		{
	// 			// apply tangential component of the impulse
	// 			const btScalar* deltaV_t1 = &m_contact->jacobianData_t1.m_deltaVelocitiesUnitImpulse[0];
	// 			multibodyLinkCol->m_multiBody->applyDeltaVeeMultiDof2(deltaV_t1, impulse.dot(m_contact->t1));
	// 			const btScalar* deltaV_t2 = &m_contact->jacobianData_t2.m_deltaVelocitiesUnitImpulse[0];
	// 			multibodyLinkCol->m_multiBody->applyDeltaVeeMultiDof2(deltaV_t2, impulse.dot(m_contact->t2));
	// 		}
	// 	}
	// }
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
	std::cout << "impulse applied: " << impulse[0] << '\t' << impulse[1] << '\t' << impulse[2] << '\n';
  m_rsb->applyFullSpaceImpulse(impulse, m_contact->m_c1, m_node->index, m_dt);
	m_rsb->mapToFullVelocity(m_rsb->getInterpolationWorldTransform());
	std::cout << "node: " << m_node->index << " vel: " << m_node->m_v[0] << '\t' << m_node->m_v[1] << '\t' << m_node->m_v[2] << '\n';
	// std::cout << "node: " << m_node->index << " m_x: " << m_node->m_x[0] << '\t' << m_node->m_x[1] << '\t' << m_node->m_x[2] << '\n';
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