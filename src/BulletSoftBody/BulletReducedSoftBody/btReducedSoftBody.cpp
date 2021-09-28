#include "btReducedSoftBody.h"
#include "../btSoftBodyInternals.h"
#include "btReducedSoftBodyHelpers.h"
#include "LinearMath/btTransformUtil.h"
#include <iostream>
#include <fstream>

btReducedSoftBody::btReducedSoftBody(btSoftBodyWorldInfo* worldInfo, int node_count, const btVector3* x, const btScalar* m)
 : btSoftBody(worldInfo, node_count, x, m)
{
  m_rigidOnly = true;     //! only use rigid frame to debug

  // reduced deformable
  m_reducedModel = true;
  m_startMode = 0;
  m_nReduced = 0;
  m_nFull = 0;

  m_ksScale = 1.0;
  m_rhoScale = 1.0;

  // rigid motion
  m_linearVelocity.setZero();
	m_angularVelocity.setZero();
  m_internalDeltaLinearVelocity.setZero();
  m_internalDeltaAngularVelocity.setZero();
	m_angularFactor.setValue(1, 1, 1);
	m_linearFactor.setValue(1, 1, 1);
  m_invInertiaLocal.setValue(1, 1, 1);
  m_mass = 0.0;
  m_inverseMass = 0.0;

  m_linearDamping = 0;
  m_angularDamping = 0;

  // Rayleigh damping
  m_dampingAlpha = 0;
  m_dampingBeta = 0;

  m_rigidTransformWorld.setIdentity();
}

void btReducedSoftBody::setReducedModes(int start_mode, int num_modes, int full_size)
{
  m_startMode = start_mode;
  m_nReduced = num_modes;
  m_nFull = full_size;
  m_reducedDofs.resize(m_nReduced, 0);
  m_reducedDofsBuffer.resize(m_nReduced, 0);
  m_reducedVelocity.resize(m_nReduced, 0);
  m_reducedVelocityBuffer.resize(m_nReduced, 0);
  m_reducedForceElastic.resize(m_nReduced, 0);
  m_reducedForceDamping.resize(m_nReduced, 0);
  m_reducedForceExternal.resize(m_nReduced, 0);
  m_internalDeltaReducedVelocity.resize(m_nReduced, 0);
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
  // set local inertia
  m_invInertiaLocal.setValue(
                inertia.x() != btScalar(0.0) ? btScalar(1.0) / inertia.x() : btScalar(0.0),
							  inertia.y() != btScalar(0.0) ? btScalar(1.0) / inertia.y() : btScalar(0.0),
							  inertia.z() != btScalar(0.0) ? btScalar(1.0) / inertia.z() : btScalar(0.0));
  


  // update world inertia tensor
  btMatrix3x3 rotation;
  rotation.setIdentity();
  updateInitialInertiaTensor(rotation);
  // updateInitialInertiaTensorFromNodes();
  updateInertiaTensor();
  m_interpolateInvInertiaTensorWorld = m_invInertiaTensorWorld;
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

void btReducedSoftBody::setFixedNodes(const int n_node)
{
  m_fixedNodes.push_back(n_node);
  m_nodes[n_node].m_im = 0;   // set inverse mass to be zero for the constraint solver.
}

void btReducedSoftBody::setDamping(const btScalar alpha, const btScalar beta)
{
  m_dampingAlpha = alpha;
  m_dampingBeta = beta;
}

void btReducedSoftBody::internalInitialization()
{
  // zeroing
  endOfTimeStepZeroing();
  // initialize rest position
  updateRestNodalPositions();
  // initialize local nodal moment arm form the CoM
  updateLocalMomentArm();
  // initialize projection matrix
  updateExternalForceProjectMatrix(false);
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

    m_STP.resize(m_nReduced);
    m_MrInvSTP.resize(m_nReduced);

    // P_A
    for (int r = 0; r < m_nReduced; ++r)
    {
      m_projPA[r].resize(3 * m_nFull, 0);
      for (int i = 0; i < m_nFull; ++i)
      {
        btMatrix3x3 mass_scaled_i = Diagonal(1) - Diagonal(m_nodalMass[i] / m_mass);
        btVector3 s_ri(m_modes[r][3 * i], m_modes[r][3 * i + 1], m_modes[r][3 * i + 2]);
        btVector3 prod_i = mass_scaled_i * s_ri;

        for (int k = 0; k < 3; ++k)
          m_projPA[r][3 * i + k] = prod_i[k];

        // btScalar ratio = m_nodalMass[i] / m_mass;
        // m_projPA[r] += btVector3(- m_modes[r][3 * i] * ratio,
        //                          - m_modes[r][3 * i + 1] * ratio,
        //                          - m_modes[r][3 * i + 2] * ratio);
      }
    }
  }

  // C(q) is updated once per position update
  for (int r = 0; r < m_nReduced; ++r)
  {
  	m_projCq[r].resize(3 * m_nFull, 0);
    for (int i = 0; i < m_nFull; ++i)
    {
      btMatrix3x3 r_star = Cross(m_localMomentArm[i]);
      btVector3 s_ri(m_modes[r][3 * i], m_modes[r][3 * i + 1], m_modes[r][3 * i + 2]);
      btVector3 prod_i = r_star * m_invInertiaTensorWorld * r_star * s_ri;

      for (int k = 0; k < 3; ++k)
        m_projCq[r][3 * i + k] = m_nodalMass[i] * prod_i[k];

      // btVector3 si(m_modes[r][3 * i], m_modes[r][3 * i + 1], m_modes[r][3 * i + 2]);
      // m_projCq[r] += m_nodalMass[i] * si.cross(m_localMomentArm[i]);
    }
  }
}

void btReducedSoftBody::endOfTimeStepZeroing()
{
  for (int i = 0; i < m_nReduced; ++i)
  {
    m_reducedForceElastic[i] = 0;
    m_reducedForceDamping[i] = 0;
    m_reducedForceExternal[i] = 0;
    m_internalDeltaReducedVelocity[i] = 0;
    m_reducedDofsBuffer[i] = m_reducedDofs[i];
    m_reducedVelocityBuffer[i] = m_reducedVelocity[i];
  }
  // std::cout << "zeroed!\n";
}

void btReducedSoftBody::applyInternalVelocityChanges()
{
  m_linearVelocity += m_internalDeltaLinearVelocity;
  m_angularVelocity += m_internalDeltaAngularVelocity;
  m_internalDeltaLinearVelocity.setZero();
  m_internalDeltaAngularVelocity.setZero();
  for (int r = 0; r < m_nReduced; ++r)
  {
    m_reducedVelocity[r] += m_internalDeltaReducedVelocity[r];
    m_internalDeltaReducedVelocity[r] = 0;
  }
}

void btReducedSoftBody::predictIntegratedTransform(btScalar dt, btTransform& predictedTransform)
{
	btTransformUtil::integrateTransform(m_rigidTransformWorld, m_linearVelocity, m_angularVelocity, dt, predictedTransform);
}

void btReducedSoftBody::updateReducedDofs(btScalar solverdt)
{
  for (int r = 0; r < m_nReduced; ++r)
  { 
    m_reducedDofs[r] = m_reducedDofsBuffer[r] + solverdt * m_reducedVelocity[r];
  }
}

void btReducedSoftBody::mapToFullPosition(const btTransform& ref_trans)
{
  btVector3 origin = ref_trans.getOrigin();
  btMatrix3x3 rotation = ref_trans.getBasis();
  

  for (int i = 0; i < m_nFull; ++i)
  {
    m_nodes[i].m_x = rotation * m_localMomentArm[i] + origin;
    m_nodes[i].m_q = m_nodes[i].m_x;
  }
}

void btReducedSoftBody::updateReducedVelocity(btScalar solverdt, bool explicit_force)
{
  // update reduced velocity
  for (int r = 0; r < m_nReduced; ++r)
  {
    btScalar mass_inv = (m_Mr[r] == 0) ? 0 : 1.0 / m_Mr[r]; // TODO: this might be redundant, because Mr is identity
    btScalar delta_v = 0;
    if (explicit_force)
    {
      delta_v = solverdt * mass_inv * (m_reducedForceElastic[r] + m_reducedForceDamping[r]);
    }
    else
    {
      delta_v = solverdt * mass_inv * (m_reducedForceDamping[r] + m_reducedForceExternal[r]);
    }
    // delta_v = solverdt * mass_inv * (m_reducedForceElastic[r] + m_reducedForceDamping[r] + m_reducedForceExternal[r]);
    // std::cout << "delta_v: " << delta_v << '\n';
    // m_reducedVelocity[r] = m_reducedVelocityBuffer[r] + delta_v;
    m_reducedVelocity[r] += delta_v;
  }
  // std::cout << "reduce_vel: " << m_reducedVelocity[0] << '\n';
  // std::cout << "force: " << m_reducedForceElastic[0] << '\t' << m_reducedForceDamping[0] << '\t' << m_reducedForceExternal[0] << '\n';
}

void btReducedSoftBody::mapToFullVelocity(const btTransform& ref_trans)
{
  for (int i = 0; i < m_nFull; ++i)
  {
    m_nodes[i].m_v = computeNodeFullVelocity(ref_trans, i);
  }
}

const btVector3 btReducedSoftBody::computeNodeFullVelocity(const btTransform& ref_trans, int n_node) const
{
  btVector3 v_from_reduced(0, 0, 0);
  btVector3 r_com = ref_trans.getBasis() * m_localMomentArm[n_node];
  // compute velocity contributed by the reduced velocity
  for (int k = 0; k < 3; ++k)
  {
    for (int r = 0; r < m_nReduced; ++r)
    {
      v_from_reduced[k] += m_modes[r][3 * n_node + k] * m_reducedVelocity[r];
    }
  }
  // get new velocity
  btVector3 vel = m_angularVelocity.cross(r_com) + 
                  ref_trans.getBasis() * v_from_reduced +
                  m_linearVelocity;
  return vel;
}

const btVector3 btReducedSoftBody::internalComputeNodeDeltaVelocity(const btTransform& ref_trans, int n_node) const
{
  btVector3 deltaV_from_reduced(0, 0, 0);
  btVector3 r_com = ref_trans.getBasis() * m_localMomentArm[n_node];

  // compute velocity contributed by the reduced velocity
  for (int k = 0; k < 3; ++k)
  {
    for (int r = 0; r < m_nReduced; ++r)
    {
      deltaV_from_reduced[k] += m_modes[r][3 * n_node + k] * m_internalDeltaReducedVelocity[r];
    }
  }

  // get delta velocity
  btVector3 deltaV = m_internalDeltaAngularVelocity.cross(r_com) + 
                     ref_trans.getBasis() * deltaV_from_reduced +
                     m_internalDeltaLinearVelocity;
  return deltaV;
}

void btReducedSoftBody::proceedToTransform(btScalar dt, bool end_of_time_step)
{
  btTransformUtil::integrateTransform(m_rigidTransformWorld, m_linearVelocity, m_angularVelocity, dt, m_interpolationWorldTransform);
  updateInertiaTensor();
  // m_interpolateInvInertiaTensorWorld = m_interpolationWorldTransform.getBasis().scaled(m_invInertiaLocal) * m_interpolationWorldTransform.getBasis().transpose();
  m_rigidTransformWorld = m_interpolationWorldTransform;
  m_invInertiaTensorWorld = m_interpolateInvInertiaTensorWorld;
}

void btReducedSoftBody::transform(const btTransform& trs)
{
  // translate mesh
	btSoftBody::transform(trs);
  // update modes
  updateModesByRotation(trs.getBasis());

  // update inertia tensor
  updateInitialInertiaTensor(trs.getBasis());
  updateInertiaTensor();
  m_interpolateInvInertiaTensorWorld = m_invInertiaTensorWorld;
  
  // update rigid frame (No need to update the rotation. Nodes have already been updated.)
  m_rigidTransformWorld.setOrigin(trs.getOrigin());
  m_interpolationWorldTransform = m_rigidTransformWorld;
  m_initialOrigin = m_rigidTransformWorld.getOrigin();

  internalInitialization();
}

void btReducedSoftBody::scale(const btVector3& scl)
{
  // scale the mesh
  btSoftBody::scale(scl);
}

void btReducedSoftBody::updateRestNodalPositions()
{
  // update reset nodal position
  m_x0.resize(m_nFull);
  for (int i = 0; i < m_nFull; ++i)
  {
    m_x0[i] = m_nodes[i].m_x;
  }
}

void btReducedSoftBody::updateInitialInertiaTensorFromNodes()
{
  btMatrix3x3 inertia_tensor;
  inertia_tensor.setZero();

  for (int p = 0; p < m_nFull; ++p)
  {
    btVector3 r = m_nodes[p].m_x - m_initialOrigin;
    for (int i = 0; i < 3; ++i)
    {
      for (int j = 0; j < 3; ++j)
      {
        inertia_tensor[i][j] += m_nodalMass[p] * r[i] * r[j];
      }
    }
  }
  m_invInertiaTensorWorldInitial = inertia_tensor.inverse();
}

void btReducedSoftBody::updateInitialInertiaTensor(const btMatrix3x3& rotation)
{
  m_invInertiaTensorWorldInitial = rotation.scaled(m_invInertiaLocal) * rotation.transpose();
}

void btReducedSoftBody::updateModesByRotation(const btMatrix3x3& rotation)
{
  for (int r = 0; r < m_nReduced; ++r)
  {
    for (int i = 0; i < m_nFull; ++i)
    {
      btVector3 nodal_disp(m_modes[r][3 * i], m_modes[r][3 * i + 1], m_modes[r][3 * i + 2]);
      nodal_disp = rotation * nodal_disp;

      for (int k = 0; k < 3; ++k)
      {
        m_modes[r][3 * i + k] = nodal_disp[k];
      }
    }
  }
}

void btReducedSoftBody::updateInertiaTensor()
{
	m_invInertiaTensorWorld = m_rigidTransformWorld.getBasis() * m_invInertiaTensorWorldInitial * m_rigidTransformWorld.getBasis().transpose();
}

void btReducedSoftBody::applyDamping(btScalar timeStep)
{
  m_linearVelocity *= btScalar(1) - m_linearDamping;
  m_angularDamping *= btScalar(1) - m_angularDamping;
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
  m_angularVelocity += m_interpolateInvInertiaTensorWorld * torque * m_angularFactor;
  #if defined(BT_CLAMP_VELOCITY_TO) && BT_CLAMP_VELOCITY_TO > 0
  clampVelocity(m_angularVelocity);
  #endif
}

void btReducedSoftBody::internalApplyRigidImpulse(const btVector3& impulse, const btVector3& rel_pos)
{
  if (m_inverseMass == btScalar(0.))
  {
    std::cout << "something went wrong...probably didn't initialize?\n";
    btAssert(false);
  }
  // delta linear velocity
  m_internalDeltaLinearVelocity += impulse * m_linearFactor * m_inverseMass;
  // delta angular velocity
  btVector3 torque = rel_pos.cross(impulse * m_linearFactor);
  m_internalDeltaAngularVelocity += m_interpolateInvInertiaTensorWorld * torque * m_angularFactor;
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

btVector3 btReducedSoftBody::getRelativePos(int n_node)
{
  btMatrix3x3 rotation = m_interpolationWorldTransform.getBasis();
  btVector3 ri = rotation * m_localMomentArm[n_node];
  return ri;
}

btMatrix3x3 btReducedSoftBody::getImpulseFactor(int n_node)
{
  // relative position
  btMatrix3x3 rotation = m_interpolationWorldTransform.getBasis();
  btVector3 ri = rotation * m_localMomentArm[n_node];
  btMatrix3x3 ri_skew = Cross(ri);

  // calculate impulse factor
  // rigid part
  btScalar inv_mass = m_nodalMass[n_node] > btScalar(0) ? btScalar(1) / m_mass : btScalar(0);
  btMatrix3x3 K1 = Diagonal(inv_mass);
  K1 -= ri_skew * m_interpolateInvInertiaTensorWorld * ri_skew;

  // reduced deformable part
  btMatrix3x3 SA;
  SA.setZero();
  for (int i = 0; i < 3; ++i)
  {
    for (int j = 0; j < 3; ++j)
    {
      for (int r = 0; r < m_nReduced; ++r)
      {
        SA[i][j] += m_modes[r][3 * n_node + i] * (m_projPA[r][3 * n_node + j] + m_projCq[r][3 * n_node + j]);
      }
    }
  }
  btMatrix3x3 RSARinv = rotation * SA * rotation.transpose();


  TVStack omega_helper; // Sum_i m_i r*_i R S_i
  omega_helper.resize(m_nReduced);
  for (int r = 0; r < m_nReduced; ++r)
  {
    omega_helper[r].setZero();
    for (int i = 0; i < m_nFull; ++i)
    {
      btMatrix3x3 mi_rstar_i = rotation * Cross(m_localMomentArm[i]) * m_nodalMass[i];
      btVector3 s_ri(m_modes[r][3 * i], m_modes[r][3 * i + 1], m_modes[r][3 * i + 2]);
      omega_helper[r] += mi_rstar_i * rotation * s_ri;
    }
  }

  btMatrix3x3 sum_multiply_A;
  sum_multiply_A.setZero();
  for (int i = 0; i < 3; ++i)
  {
    for (int j = 0; j < 3; ++j)
    {
      for (int r = 0; r < m_nReduced; ++r)
      {
        sum_multiply_A[i][j] += omega_helper[r][i] * (m_projPA[r][3 * n_node + j] + m_projCq[r][3 * n_node + j]);
      }
    }
  }

  btMatrix3x3 K2 = RSARinv + ri_skew * m_interpolateInvInertiaTensorWorld * sum_multiply_A * rotation.transpose();

  return m_rigidOnly ? K1 : K1 + K2;
}

void btReducedSoftBody::applyVelocityConstraint(const btVector3& target_vel, int n_node, btScalar dt)
{
  // get impulse
  btMatrix3x3 impulse_factor = getImpulseFactor(n_node);
  btVector3 impulse = impulse_factor.inverse() * (target_vel - m_nodes[n_node].m_v);
  // btScalar impulse_magnitude = impulse.norm();
  
  // if (impulse_magnitude < 5e-7)
  // {
  //   impulse.setZero();
  //   impulse_magnitude = 0;
  // }

  // relative position
  btMatrix3x3 rotation = m_interpolationWorldTransform.getBasis();
  btVector3 ri = rotation * m_localMomentArm[n_node];
  
  // apply full space impulse
  applyFullSpaceImpulse(impulse, ri, n_node, dt);
}

void btReducedSoftBody::applyPositionConstraint(const btVector3& target_pos, int n_node, btScalar dt)
{
  btVector3 impulse(0, 0, 0);

  // apply full space impulse
  // applyFullSpaceImpulse(impulse, n_node, dt);
}

void btReducedSoftBody::internalApplyFullSpaceImpulse(const btVector3& impulse, const btVector3& rel_pos, int n_node, btScalar dt)
{
  if (!m_rigidOnly)
  {
    // apply impulse force
    applyFullSpaceNodalForce(impulse / dt, n_node);

    // update delta damping force
    applyReducedDampingForce(m_internalDeltaReducedVelocity);

    // delta reduced velocity
    for (int r = 0; r < m_nReduced; ++r)
    {
      btScalar mass_inv = (m_Mr[r] == 0) ? 0 : 1.0 / m_Mr[r]; // TODO: this might be redundant, because Mr is identity
      m_internalDeltaReducedVelocity[r] += dt * mass_inv * (m_reducedForceDamping[r] + m_reducedForceExternal[r]);
    }
  }

  internalApplyRigidImpulse(impulse, rel_pos);
}

void btReducedSoftBody::applyFullSpaceImpulse(const btVector3& impulse, const btVector3& rel_pos, int n_node, btScalar dt)
{
  if (!m_rigidOnly)
  {
    // apply impulse force
    applyFullSpaceNodalForce(impulse / dt, n_node);

    // update reduced internal force
    applyReducedDampingForce(m_reducedVelocity);

    // update reduced velocity
    updateReducedVelocity(dt); // TODO: add back
  }

  // // update reduced dofs
  // updateReducedDofs(dt);

  // // internal force
  // // applyReducedInternalForce(0, 0.01); // TODO: this should be necessary, but no obvious effects. Check again.

  // // update local moment arm
  // updateLocalMomentArm();
  // updateExternalForceProjectMatrix(true);

  // impulse causes rigid motion
  applyRigidImpulse(impulse, rel_pos);
}

void btReducedSoftBody::applyFullSpaceNodalForce(const btVector3& f_ext, int n_node)
{
  // f_local = R^-1 * f_ext //TODO: interpoalted transfrom
  // btVector3 f_local = m_rigidTransformWorld.getBasis().transpose() * f_ext;
  btVector3 f_local = m_interpolationWorldTransform.getBasis().transpose() * f_ext;

  // f_ext_r = [S^T * P]_{n_node} * f_local
  tDenseArray f_ext_r;
  f_ext_r.resize(m_nReduced, 0);
  for (int r = 0; r < m_nReduced; ++r)
  {
    m_reducedForceExternal[r] = 0;
    for (int k = 0; k < 3; ++k)
    {
      f_ext_r[r] += (m_projPA[r][3 * n_node + k] + m_projCq[r][3 * n_node + k]) * f_local[k];
    }

    m_reducedForceExternal[r] += f_ext_r[r];
  }
}

void btReducedSoftBody::applyRigidGravity(const btVector3& gravity, btScalar dt)
{
  // update rigid frame velocity
  m_linearVelocity += dt * gravity;
}

void btReducedSoftBody::applyReducedElasticForce(const tDenseArray& reduce_dofs)
{
  for (int r = 0; r < m_nReduced; ++r) 
  {
    m_reducedForceElastic[r] = - m_ksScale * m_Kr[r] * reduce_dofs[r];
  }
}

void btReducedSoftBody::applyReducedDampingForce(const tDenseArray& reduce_vel)
{
  for (int r = 0; r < m_nReduced; ++r) 
  {
    m_reducedForceDamping[r] = - m_dampingBeta * m_Kr[r] * reduce_vel[r];
  }
}

void btReducedSoftBody::applyFixedContraints(btScalar dt)
{
  for (int n = 0; n < m_fixedNodes.size(); ++n)
  {
    // apply impulse, velocity constraint
    applyVelocityConstraint(btVector3(0, 0, 0), m_fixedNodes[n], dt);

    // update predicted basis
    proceedToTransform(dt, false);  // TODO: maybe don't need?

    // update full space velocity
    mapToFullVelocity(getInterpolationWorldTransform());

    // apply impulse again, position constraint
    // applyPositionConstraint(m_x0[m_fixedNodes[n]], m_fixedNodes[n], dt);
  }
}

btScalar btReducedSoftBody::getTotalMass() const
{
  return m_mass;
}

btTransform& btReducedSoftBody::getRigidTransform()
{
  return m_rigidTransformWorld;
}

const btVector3& btReducedSoftBody::getLinearVelocity() const
{
  return m_linearVelocity;
}

const btVector3& btReducedSoftBody::getAngularVelocity() const
{
  return m_angularVelocity;
}