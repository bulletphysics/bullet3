#include "btReducedSoftBody.h"
#include "btReducedSoftBodyHelpers.h"
#include "LinearMath/btTransformUtil.h"
#include <iostream>

btReducedSoftBody::btReducedSoftBody(btSoftBodyWorldInfo* worldInfo, int node_count, const btVector3* x, const btScalar* m)
 : btSoftBody(worldInfo, node_count, x, m)
{
  // reduced deformable
  m_reducedModel = true;
  m_startMode = 0;
  m_nReduced = 0;
  m_nFull = 0;

  m_ksScale = 1.0;
  m_rhoScale = 1.0;

  // rigid motion
  m_linearVelocity.setValue(btScalar(0.0), btScalar(0.0), btScalar(0.0));
	m_angularVelocity.setValue(btScalar(0.0), btScalar(0.0), btScalar(0.0));
	m_angularFactor.setValue(1, 1, 1);
	m_linearFactor.setValue(1, 1, 1);
  m_invInertiaLocal.setValue(1, 1, 1);
  m_mass = 0.0;
  m_inverseMass = 0.0;
}

void btReducedSoftBody::setReducedModes(int start_mode, int num_modes, int full_size)
{
  m_startMode = start_mode;
  m_nReduced = num_modes;
  m_nFull = full_size;
  m_reducedDofs.resize(m_nReduced, 0);
  m_reducedVelocity.resize(m_nReduced, 0);
  m_nodalMass.resize(full_size, 0);
}

void btReducedSoftBody::setMassProps(const tDenseArray& mass_array)
{
  // nodal mass
  btScalar total_mass = 0;
	for (int i = 0; i < m_nFull; ++i)
	{
		m_nodalMass[i] = mass_array[3 * i];
		m_nodes[i].m_im = mass_array[3 * i] > 0 ? mass_array[3 * i] : 0;
		total_mass += mass_array[3 * i];
	}
  // total rigid body mass
  m_mass = total_mass;
  m_inverseMass = total_mass > 0 ? 1.0 / total_mass : 0;
}

void btReducedSoftBody::setInertiaProps(const btVector3& inertia)
{
  // TODO: only support box shape now
  // set local intertia
  m_invInertiaLocal.setValue(
                inertia.x() != btScalar(0.0) ? btScalar(1.0) / inertia.x() : btScalar(0.0),
							  inertia.y() != btScalar(0.0) ? btScalar(1.0) / inertia.y() : btScalar(0.0),
							  inertia.z() != btScalar(0.0) ? btScalar(1.0) / inertia.z() : btScalar(0.0));

  // update world inertia tensor
  updateInertiaTensor();
}

void btReducedSoftBody::setRigidVelocity(const btVector3& v)
{
  m_linearVelocity = v;
}

void btReducedSoftBody::setRigidAngularVelocity(const btVector3& omega)
{
  m_angularVelocity = omega;
}

void btReducedSoftBody::setStiffnessScale(const btScalar ks)
{
  m_ksScale = ks;
}

void btReducedSoftBody::predictIntegratedTransform(btScalar timeStep, btTransform& predictedTransform)
{
	btTransformUtil::integrateTransform(m_worldTransform, m_linearVelocity, m_angularVelocity, timeStep, predictedTransform);
}

void btReducedSoftBody::updateReducedDofs()
{
  btAssert(m_reducedDofs.size() == m_nReduced);
  for (int j = 0; j < m_nReduced; ++j) 
  {
    m_reducedDofs[j] = 0;
    for (int i = 0; i < m_nFull; ++i)
    {
      for (int k = 0; k < 3; ++k)
      {
        // std::cout << m_nodes[i].m_x[k] - m_x0[i][k] << "\n";
        m_reducedDofs[j] += m_modes[j][3 * i + k] * (m_nodes[i].m_x[k] - m_x0[i][k]);
      }
    }
  }
}

void btReducedSoftBody::updateFullDofs()
{
  btAssert(m_nodes.size() == m_nFull);
  btAlignedObjectArray<btVector3> delta_x;
  delta_x.resize(m_nFull);
  btVector3 origin = getWorldTransform().getOrigin();
  btMatrix3x3 rotation = getWorldTransform().getBasis();

  for (int i = 0; i < m_nFull; ++i)
  {
    for (int k = 0; k < 3; ++k)
    {
      // compute displacement
      delta_x[i][k] = 0;
      for (int j = 0; j < m_nReduced; ++j) 
      {
        delta_x[i][k] += m_modes[j][3 * i + k] * m_reducedDofs[j];
      }
    }
    // get new coordinates
    m_nodes[i].m_x = rotation * (m_x0[i] + delta_x[i]) + origin; //TODO: assume the initial origin is at (0,0,0)
  }
}

void btReducedSoftBody::proceedToTransform(const btTransform& newTrans)
{
	setCenterOfMassTransform(newTrans);
}

void btReducedSoftBody::setCenterOfMassTransform(const btTransform& xform)
{
	if (isKinematicObject())
	{
		m_interpolationWorldTransform = m_worldTransform;
	}
	else
	{
		m_interpolationWorldTransform = xform;
	}
	m_interpolationLinearVelocity = getLinearVelocity();
	m_interpolationAngularVelocity = getAngularVelocity();
	m_worldTransform = xform;
	updateInertiaTensor();
}

void btReducedSoftBody::translate(const btVector3& trs)
{
  // translate mesh
	btSoftBody::translate(trs);
  updateRestNodalPositions();
  // for (int i = 0; i < m_nFull; ++i)
  //   for (int k = 0; k < 3; ++k)
  //     std::cout << m_nodes[i].m_x[k] << "\t" << m_x0[i][k] << "\n";
  
  // update rigid frame
  // m_worldTransform.setOrigin(trs);
}

void btReducedSoftBody::updateRestNodalPositions()
{
  m_x0.resize(m_nFull);
	for (int i = 0; i < m_nFull; ++i)
		m_x0[i] = m_nodes[i].m_x;
}

void btReducedSoftBody::updateInertiaTensor()
{
	m_invInertiaTensorWorld = m_worldTransform.getBasis().scaled(m_invInertiaLocal) * m_worldTransform.getBasis().transpose();
}

void btReducedSoftBody::applyCentralImpulse(const btVector3& impulse)
{
  m_linearVelocity += impulse * m_linearFactor * m_inverseMass;
  #if defined(BT_CLAMP_VELOCITY_TO) && BT_CLAMP_VELOCITY_TO > 0
  clampVelocity(m_linearVelocity);
  #endif
}

void btReducedSoftBody::applyTorqueImpulse(const btVector3& torque)
{
  m_angularVelocity += m_invInertiaTensorWorld * torque * m_angularFactor;
  #if defined(BT_CLAMP_VELOCITY_TO) && BT_CLAMP_VELOCITY_TO > 0
  clampVelocity(m_angularVelocity);
  #endif
}

void btReducedSoftBody::applyImpulse(const btVector3& impulse, const btVector3& rel_pos)
{
  if (m_inverseMass != btScalar(0.))
  {
    applyCentralImpulse(impulse);
    if (m_angularFactor)
    {
      applyTorqueImpulse(rel_pos.cross(impulse * m_linearFactor));
    }
  }
}

void btReducedSoftBody::applyFullSpaceImpulse(const btVector3& target_vel, int n_node, btScalar dt, tDenseArray& reduced_force)
{
  // impulse leads to the deformation in the reduced space
  btVector3 impulse = m_nodalMass[n_node] / dt * (target_vel - m_nodes[n_node].m_v);
  for (int i = 0; i < m_nReduced; ++i)
  {
    for (int k = 0; k < 3; ++k)
    {
      reduced_force[i] += m_modes[i][3 * n_node + k] * impulse[k];
    }
  }
  // impulse causes rigid motion
  applyImpulse(impulse, m_nodes[n_node].m_x);
}

void btReducedSoftBody::applyRigidGravity(const btVector3& gravity, btScalar dt)
{
  m_linearVelocity += dt * gravity;
}

void btReducedSoftBody::applyReducedInternalForce(tDenseArray& reduced_force, const btScalar damping_alpha, const btScalar damping_beta)
{
  for (int r = 0; r < m_nReduced; ++r) 
  {
    reduced_force[r] += m_ksScale * m_Kr[r] * (m_reducedDofs[r] + damping_beta * m_reducedVelocity[r]);
  }
}