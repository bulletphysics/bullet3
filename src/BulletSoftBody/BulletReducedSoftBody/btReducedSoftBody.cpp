#include "btReducedSoftBody.h"
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

  // rigid motion
  m_linearVelocity.setValue(btScalar(0.0), btScalar(0.0), btScalar(0.0));
	m_angularVelocity.setValue(btScalar(0.0), btScalar(0.0), btScalar(0.0));
	m_angularFactor.setValue(1, 1, 1);
	m_linearFactor.setValue(1, 1, 1);
  m_invInertiaLocal.setValue(1, 1, 1); //TODO: get correct inertia through shape
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

void btReducedSoftBody::setMass(btScalar m)
{
  m_mass = m;
  m_inverseMass = m > 0 ? 1.0 / m : 0;
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
      for (int k = 0; k < 3; ++k)
        m_reducedDofs[j] += m_modes[j][3 * i + k] * (m_nodes[i].m_x[k] - m_x0[3 * i + k]);
  }
}

void btReducedSoftBody::updateFullDofs()
{
  btAssert(m_nodes.size() == m_nFull);
  btAlignedObjectArray<btVector3> delta_x;
  delta_x.resize(m_nFull);
  btVector3 origin = getWorldTransform().getOrigin();

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
      // get new coordinates
      m_nodes[i].m_x[k] = m_x0[3 * i + k] + delta_x[i][k] + origin[k]; //TODO: assume the initial origin is at (0,0,0)
    }
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