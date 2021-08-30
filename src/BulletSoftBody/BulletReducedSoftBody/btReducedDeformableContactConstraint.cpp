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
{
	m_appliedNormalImpulse = 0;
  m_appliedTangentImpulse = 0;
	m_rhs = 0;
	m_cfm = 0;
	m_erp = btScalar(0.2);			// TODO: add parameter handle
	m_friction = btScalar(0);

  m_contactNormalA = c.m_cti.m_normal;
  m_contactNormalB = -c.m_cti.m_normal;
  m_impulseFactor = c.m_c0;
	m_normalImpulseFactor = (m_impulseFactor * m_contactNormalA).dot(m_contactNormalA);
	m_tangentImpulseFactor = 0;

	m_relPosA = c.m_c1;

	btRigidBody* rb = m_contact->m_cti.m_colObj ? (btRigidBody*)btRigidBody::upcast(m_contact->m_cti.m_colObj) : nullptr;
	if (!rb)
	{
		btAssert(false);
	}
	else
	{
		m_collideStatic = rb->isStaticObject();
	}
}

void btReducedDeformableRigidContactConstraint::setSolverBody(btSolverBody& solver_body)
{
	m_solverBody = &solver_body;
	m_linearComponent = m_contactNormalA * m_solverBody->internalGetInvMass();
	btVector3	torqueAxis = m_relPosA.cross(m_contactNormalA);
	m_angularComponent = m_solverBody->m_originalBody->getInvInertiaTensorWorld() * torqueAxis;
}

btVector3 btReducedDeformableRigidContactConstraint::getVa() const
{
	btVector3 Va(0, 0, 0);
	if (!m_collideStatic)
	{
		Va = btDeformableRigidContactConstraint::getVa();
	}
	return Va;
}

btScalar btReducedDeformableRigidContactConstraint::solveConstraint(const btContactSolverInfo& infoGlobal)
{
	btVector3 Va = getVa();
	// btVector3 deltaVa = Va - m_bufferVelocityA;
	btVector3 deltaVa = getDeltaVa();
	btVector3 deltaVb = getDeltaVb();
	std::cout << "deltaVa: " << deltaVa[0] << '\t' << deltaVa[1] << '\t' << deltaVa[2] << '\n';
	std::cout << "deltaVb: " << deltaVb[0] << '\t' << deltaVb[1] << '\t' << deltaVb[2] << '\n';

	// get delta relative velocity and magnitude (i.e., how much impulse has been applied?)
	btVector3 deltaV_rel = deltaVa - deltaVb;
	btScalar deltaV_rel_normal = -btDot(deltaV_rel, m_contactNormalA);
	std::cout << "deltaV_rel_normal: " << deltaV_rel_normal << "\n";
	
	// get the normal impulse to be applied
	btScalar deltaImpulse = m_rhs - deltaV_rel_normal / m_normalImpulseFactor;
	{
		// cumulative impulse that has been applied
		btScalar sum = m_appliedNormalImpulse + deltaImpulse;
		// if the cumulative impulse is pushing the object into the rigid body, set it zero
		if (sum < 0)
		{
			std::cout <<"set zeroed!!!\n";
			deltaImpulse = -m_appliedNormalImpulse;
			m_appliedNormalImpulse = 0;
		}
		else
		{
			m_appliedNormalImpulse = sum;
		}	
	}
	std::cout << "m_appliedNormalImpulse: " << m_appliedNormalImpulse << '\n';
	std::cout << "deltaImpulse: " << deltaImpulse << '\n';

	// residual is the nodal normal velocity change in current iteration
	btScalar residualSquare = deltaImpulse * m_normalImpulseFactor;	// get residual
	residualSquare *= residualSquare;

	
	// apply Coulomb friction (based on delta velocity, |dv_t| = |dv_n * friction|)
	btScalar deltaImpulse_tangent = 0;
	{
		// trial velocity change
		// deltaImpulse_tangent = m_friction * deltaImpulse;
		// btScalar deltaV_tangent_trial = deltaImpulse_tangent * m_impulseFactorTangent;

		btScalar total_deltaV_rel_normal = m_appliedNormalImpulse * m_normalImpulseFactor;
		btScalar deltaV_tangent_trial = m_friction * total_deltaV_rel_normal;

		// initial relative tangential veloity magnitude
		btScalar v_rel_tangent = abs((Va - getVb()).dot(m_contactTangent));

		// if the trial velocity change is greater than the current tangential relative velocity
		if (deltaV_tangent_trial > v_rel_tangent)
		{	
			deltaImpulse_tangent = v_rel_tangent / m_tangentImpulseFactor;
		}
		else
		{
			deltaImpulse_tangent = deltaV_tangent_trial / m_tangentImpulseFactor;
		}		

		btScalar sum = m_appliedTangentImpulse + deltaImpulse_tangent;
		btScalar lower_limit = - m_appliedNormalImpulse * m_friction;
		btScalar upper_limit = m_appliedNormalImpulse * m_friction;
		if (sum > upper_limit)
		{
			deltaImpulse_tangent = upper_limit - m_appliedTangentImpulse;
			m_appliedTangentImpulse = upper_limit;
		}
		else if (sum < lower_limit)
		{
			deltaImpulse_tangent = lower_limit - m_appliedTangentImpulse;
			m_appliedTangentImpulse = lower_limit;
		}
		else
		{
			m_appliedTangentImpulse = sum;
		}
	}

	// get the total impulse vector
	btVector3 impulse_normal = deltaImpulse * m_contactNormalA;
	btVector3 impulse_tangent = -deltaImpulse_tangent * m_contactTangent;
	btVector3 impulse = impulse_normal + impulse_tangent;

	applyImpulse(impulse);
	
	// apply impulse to the rigid/multibodies involved and change their velocities
	if (!m_collideStatic)
	{
		const btSoftBody::sCti& cti = m_contact->m_cti;
		if (cti.m_colObj->getInternalType() == btCollisionObject::CO_RIGID_BODY)
		{
			m_solverBody->internalApplyImpulse(m_linearComponent, m_angularComponent, -deltaImpulse);
			// m_solverBody->internalApplyImpulse(m_linearComponent, m_angularComponent, -deltaImpulse_tangent);
		}
		else if (cti.m_colObj->getInternalType() == btCollisionObject::CO_FEATHERSTONE_LINK)
		{
			btAssert(false);	//TODO: unsupported yet
			// btMultiBodyLinkCollider* multibodyLinkCol = 0;
			// multibodyLinkCol = (btMultiBodyLinkCollider*)btMultiBodyLinkCollider::upcast(cti.m_colObj);
			// if (multibodyLinkCol)
			// {
			// 	const btScalar* deltaV_normal = &m_contact->jacobianData_normal.m_deltaVelocitiesUnitImpulse[0];
			// 	// apply normal component of the impulse
			// 	multibodyLinkCol->m_multiBody->applyDeltaVeeMultiDof2(deltaV_normal, impulse.dot(cti.m_normal));
			// 	if (impulse_tangent.norm() > SIMD_EPSILON)
			// 	{
			// 		// apply tangential component of the impulse
			// 		const btScalar* deltaV_t1 = &m_contact->jacobianData_t1.m_deltaVelocitiesUnitImpulse[0];
			// 		multibodyLinkCol->m_multiBody->applyDeltaVeeMultiDof2(deltaV_t1, impulse.dot(m_contact->t1));
			// 		const btScalar* deltaV_t2 = &m_contact->jacobianData_t2.m_deltaVelocitiesUnitImpulse[0];
			// 		multibodyLinkCol->m_multiBody->applyDeltaVeeMultiDof2(deltaV_t2, impulse.dot(m_contact->t2));
			// 	}
			// }
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
{
	m_relPosB = m_node->m_x - m_rsb->getRigidTransform().getOrigin();
	warmStarting();
}

void btReducedDeformableNodeRigidContactConstraint::warmStarting()
{
	btVector3 va = getVa();
	btVector3 vb = getVb();
	m_bufferVelocityA = va;
	m_bufferVelocityB = vb;

	// we define the (+) direction of errors to be the outward surface normal of the rigid object
	btVector3 v_rel = vb - va;
	// get tangent direction of the relative velocity
	m_contactTangent = (v_rel - v_rel.dot(m_contactNormalA) * m_contactNormalA).safeNormalize();

	// tangent impulse factor
	m_tangentImpulseFactor = (m_impulseFactor * m_contactTangent).dot(m_contactTangent);

	btScalar velocity_error = btDot(v_rel, m_contactNormalA);	// magnitude of relative velocity
	btScalar position_error = 0;
	if (m_penetration > 0)
	{
		// velocity_error += m_penetration / m_dt; // TODO: why?
	}
	else
	{
		// add penetration correction vel
		position_error = m_penetration * m_erp / m_dt;
	}
	// get the initial estimate of impulse magnitude to be applied
	// m_rhs = -velocity_error * m_normalImpulseFactorInv;
	m_rhs = -(velocity_error + position_error) / m_normalImpulseFactor;
}

btVector3 btReducedDeformableNodeRigidContactConstraint::getVb() const
{
	return m_node->m_v;
}

btVector3 btReducedDeformableNodeRigidContactConstraint::getDeltaVa() const
{
	btVector3 deltaVa(0, 0, 0);
	if (!m_collideStatic)
	{
		deltaVa = m_solverBody->internalGetDeltaLinearVelocity() + m_relPosA.cross(m_solverBody->internalGetDeltaAngularVelocity());
	}
	return deltaVa;
}

btVector3 btReducedDeformableNodeRigidContactConstraint::getDeltaVb() const
{
	return m_rsb->internalComputeNodeDeltaVelocity(m_rsb->getInterpolationWorldTransform(), m_node->index);
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
  m_rsb->internalApplyFullSpaceImpulse(impulse, m_relPosB, m_node->index, m_dt);
	// m_rsb->applyFullSpaceImpulse(impulse, m_relPosB, m_node->index, m_dt);
	// m_rsb->mapToFullVelocity(m_rsb->getInterpolationWorldTransform());
	std::cout << "node: " << m_node->index << " vel: " << m_node->m_v[0] << '\t' << m_node->m_v[1] << '\t' << m_node->m_v[2] << '\n';
	btVector3 v_after = getDeltaVb() + m_node->m_v;
	std::cout << "vel after: " << v_after[0] << '\t' << v_after[1] << '\t' << v_after[2] << '\n';
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