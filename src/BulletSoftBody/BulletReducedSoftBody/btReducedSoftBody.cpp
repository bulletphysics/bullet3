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

  m_rigidTransformWorld.setIdentity();
}

void btReducedSoftBody::setReducedModes(int start_mode, int num_modes, int full_size)
{
  m_startMode = start_mode;
  m_nReduced = num_modes;
  m_nFull = full_size;
  m_reducedDofs.resize(m_nReduced, 0);
  m_reducedVelocity.resize(m_nReduced, 0);
  m_reducedForce.resize(m_nReduced, 0);
  m_nodalMass.resize(full_size, 0);
  m_localMomentArm.resize(m_nFull);
}

void btReducedSoftBody::setMassProps(const tDenseArray& mass_array)
{
  // nodal mass
  btScalar total_mass = 0;
	for (int i = 0; i < m_nFull; ++i)
	{
		m_nodalMass[i] = m_rhoScale * mass_array[3 * i];
		m_nodes[i].m_im = mass_array[3 * i] > 0 ? 1.0 / (m_rhoScale * mass_array[3 * i]) : 0;
		total_mass += m_rhoScale * mass_array[3 * i];
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

void btReducedSoftBody::setMassScale(const btScalar rho)
{
  m_rhoScale = rho;
}

void btReducedSoftBody::setFixedNodes()
{
  // for (int i = 0; i < m_nFull; ++i)
  // {
  //   if (abs(m_nodes[i].m_x[2] - (-2)) < 1e-3)
  //     m_fixedNodes.push_back(i);
  // }
  m_fixedNodes.push_back(0);
}

void btReducedSoftBody::internalInitialization()
{
  // zeroing
  endOfTimeStepZeroing();
  // initialize rest position
	updateRestNodalPositions();
  // initialize projection matrix
  updateExternalForceProjectMatrix(false);
  // initialize local nodal moment arm form the CoM
  updateLocalMomentArm();
}

void btReducedSoftBody::updateLocalMomentArm()
{
  TVStack delta_x;
  delta_x.resize(m_nFull);

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
    // get new moment arm Sq + x0
    m_localMomentArm[i] = m_x0[i] - m_initialOrigin + delta_x[i];
  }
}

void btReducedSoftBody::updateExternalForceProjectMatrix(bool initialized)
{
  // if not initialized, need to compute both P_A and Cq
  // otherwise, only need to udpate Cq
  if (!initialized)
  {
    // resize
    m_projPA.resize(m_nReduced);
    m_projCq.resize(m_nReduced);

    // P_A
    for (int r = 0; r < m_nReduced; ++r)
    {
      m_projPA[r].setValue(0, 0, 0);
      for (int i = 0; i < m_nFull; ++i)
      {
        btScalar ratio = m_nodalMass[i] / m_mass;
        m_projPA[r] += btVector3(- m_modes[r][3 * i] * ratio,
                                 - m_modes[r][3 * i + 1] * ratio,
                                 - m_modes[r][3 * i + 2] * ratio);
      }
    }
  }

  // C(q) is updated once per position update
  for (int r = 0; r < m_nReduced; ++r)
  {
    m_projCq[r].setValue(0, 0, 0);
    for (int i = 0; i < m_nFull; ++i)
    {
      btVector3 si(m_modes[r][3 * i], m_modes[r][3 * i + 1], m_modes[r][3 * i + 2]);
      m_projCq[r] += m_nodalMass[i] * si.cross(m_localMomentArm[i]);
    }
  }
}

void btReducedSoftBody::endOfTimeStepZeroing()
{
  for (int i = 0; i < m_nReduced; ++i)
  {
    m_reducedForce[i] = 0;
  }
}

void btReducedSoftBody::predictIntegratedTransform(btScalar timeStep, btTransform& predictedTransform)
{
	btTransformUtil::integrateTransform(m_rigidTransformWorld, m_linearVelocity, m_angularVelocity, timeStep, predictedTransform);
}

void btReducedSoftBody::updateReducedDofs(btScalar solverdt)
{
  for (int r = 0; r < m_nReduced; ++r)
  { 
    m_reducedDofs[r] += solverdt * m_reducedVelocity[r];
  }
}

void btReducedSoftBody::mapToFullDofs()
{
  btVector3 origin = m_rigidTransformWorld.getOrigin();
  btMatrix3x3 rotation = m_rigidTransformWorld.getBasis();

  for (int i = 0; i < m_nFull; ++i)
  {
    m_nodes[i].m_x = rotation * m_localMomentArm[i] + origin;
  }
}

void btReducedSoftBody::updateReducedVelocity(btScalar solverdt)
{
  // update reduced velocity
  for (int r = 0; r < m_nReduced; ++r)
  {
    btScalar mass_inv = (m_Mr[r] == 0) ? 0 : 1.0 / m_Mr[r]; // TODO: this might be redundant, because Mr is identity
    btScalar delta_v = solverdt * mass_inv * m_reducedForce[r];
    m_reducedVelocity[r] += delta_v;
  }
}

void btReducedSoftBody::mapToFullVelocity(btScalar solverdt)
{
  TVStack v_from_reduced;
  v_from_reduced.resize(m_nFull);
  // btVector3 current_com = m_rigidTransformWorld.getOrigin() - m_initialOrigin;  
  for (int i = 0; i < m_nFull; ++i)
  {
    btVector3 r_com = m_nodes[i].m_x - m_rigidTransformWorld.getOrigin();

    // compute velocity contributed by the reduced velocity
    for (int k = 0; k < 3; ++k)
    {
      v_from_reduced[i][k] = 0;
      for (int r = 0; r < m_nReduced; ++r)
      {
        v_from_reduced[i][k] += m_modes[r][3 * i + k] * m_reducedVelocity[r];
      }
    }

    // get new velocity
    m_nodes[i].m_v = m_angularVelocity.cross(r_com) + 
                     m_rigidTransformWorld.getBasis() * v_from_reduced[i] +
                     m_linearVelocity;
  }
  std::cout << m_nodes[0].m_v[0] << '\t' << m_nodes[0].m_v[1] << '\t' << m_nodes[0].m_v[2] << '\n';
}

void btReducedSoftBody::proceedToTransform(const btTransform& newTrans)
{
	setCenterOfMassTransform(newTrans);
}

void btReducedSoftBody::setCenterOfMassTransform(const btTransform& xform)
{
	if (isKinematicObject())
	{
		m_interpolationWorldTransform = m_rigidTransformWorld;
	}
	else
	{
		m_interpolationWorldTransform = xform;
	}
	m_interpolationLinearVelocity = getLinearVelocity();
	m_interpolationAngularVelocity = getAngularVelocity();
	m_rigidTransformWorld = xform;
	updateInertiaTensor();
}

void btReducedSoftBody::translate(const btVector3& trs)
{
  // translate mesh
	btSoftBody::translate(trs);
  updateRestNodalPositions();
  
  // update rigid frame
  m_rigidTransformWorld.setOrigin(trs);
  m_interpolationWorldTransform = m_rigidTransformWorld;
  m_initialOrigin = m_rigidTransformWorld.getOrigin();
  updateInertiaTensor();
}

void btReducedSoftBody::updateRestNodalPositions()
{
  m_x0.resize(m_nFull);
	for (int i = 0; i < m_nFull; ++i)
		m_x0[i] = m_nodes[i].m_x;
}

void btReducedSoftBody::updateInertiaTensor()
{
	m_invInertiaTensorWorld = m_rigidTransformWorld.getBasis().scaled(m_invInertiaLocal) * m_rigidTransformWorld.getBasis().transpose();
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

void btReducedSoftBody::applyRigidImpulse(const btVector3& impulse, const btVector3& rel_pos)
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

void btReducedSoftBody::applyFullSpaceImpulse(const btVector3& target_vel, int n_node, btScalar dt)
{
  // TODO: get correct impulse using the impulse factor
  btVector3 ri = m_localMomentArm[n_node];
  btMatrix3x3 ri_skew(0,    -ri[2], ri[1],
                      ri[2], 0,    -ri[0],
                     -ri[1], ri[0], 0);

  // calcualte impulse factor
  btScalar inv_mass = m_nodalMass[n_node] > btScalar(0) ? btScalar(1) / m_mass : btScalar(0);
  btMatrix3x3 impulse_factor(inv_mass, 0, 0,
                             0, inv_mass, 0,
                             0, 0, inv_mass);
  impulse_factor -= ri_skew.scaled(m_invInertiaLocal) * ri_skew;

  // get impulse
  btVector3 impulse = impulse_factor * (target_vel - m_nodes[n_node].m_v);

  // apply impulse force
  applyFullSpaceNodalForce(impulse / dt, n_node);

  // impulse causes rigid motion
  applyRigidImpulse(impulse, m_nodes[n_node].m_x);
}

void btReducedSoftBody::applyFullSpaceNodalForce(const btVector3& f_ext, int n_node)
{
  // f_local = R^-1 * f_ext
  btVector3 f_local = m_worldTransform.getBasis().transpose() * f_ext;

  // f_scaled = localInvInertia * (r_k x f_local)
  btVector3 rk_cross_f_local = m_localMomentArm[n_node].cross(f_local);
  btVector3 f_scaled(0, 0, 0);
  for (int k = 0; k < 3; ++k)
  {
    f_scaled[k] = m_invInertiaLocal[k] * rk_cross_f_local[k];
  }

  // f_ext_r = [S^T * P]_{n_node} * f_local
  for (int r = 0; r < m_nReduced; ++r)
  {
    for (int k = 0; k < 3; ++k)
    {
      m_reducedForce[r] += m_modes[r][3 * n_node + k] * f_local[k] + 
                           m_projPA[r][k] * f_local[k] + 
                           m_projCq[r][k] * f_scaled[k];
    }
  }
}

void btReducedSoftBody::applyRigidGravity(const btVector3& gravity, btScalar dt)
{
  // update rigid frame velocity
  m_linearVelocity += dt * gravity;
}

void btReducedSoftBody::applyReducedInternalForce(const btScalar damping_alpha, const btScalar damping_beta)
{
  for (int r = 0; r < m_nReduced; ++r) 
  {
    m_reducedForce[r] = - m_ksScale * m_Kr[r] * (m_reducedDofs[r] + damping_beta * m_reducedVelocity[r]);
  }
}

void btReducedSoftBody::applyFixedContraints(btScalar dt)
{
  for (int n = 0; n < m_fixedNodes.size(); ++n)
  {
    // std::cout << reduced_force[0] << "\t" << reduced_force[1] << "\n";
    applyFullSpaceImpulse(btVector3(0, 0, 0), m_fixedNodes[n], dt);
    // std::cout << reduced_force[0] << "\t" << reduced_force[1] << "\n";
  }
}