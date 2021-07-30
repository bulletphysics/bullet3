#ifndef BT_REDUCED_SOFT_BODY_H
#define BT_REDUCED_SOFT_BODY_H

#include "../btSoftBody.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btMatrix3x3.h"
#include "LinearMath/btTransform.h"

// Reduced deformable body is a simplified deformable object embedded in a rigid frame.
class btReducedSoftBody : public btSoftBody
{
 private:
  // scaling factors
  btScalar m_rhoScale;         // mass density scale
  btScalar m_ksScale;          // stiffness scale

 protected:
  // rigid frame
  btScalar m_mass;          // total mass of the rigid frame
  btScalar m_inverseMass;   // inverse of the total mass of the rigid frame
	btVector3 m_angularVelocity;
	btVector3 m_linearFactor;
	btVector3 m_angularFactor;
	btVector3 m_invInertiaLocal;
  btTransform m_rigidTransformWorld;
  btMatrix3x3 m_invInertiaTensorWorld;
  btVector3 m_initialOrigin;  // initial center of mass (original of the m_rigidTransformWorld)

 public:
  //
  //  Typedefs
  //
  typedef btAlignedObjectArray<btVector3> TVStack;
  typedef btAlignedObjectArray<btScalar> tDenseArray;
  typedef btAlignedObjectArray<btAlignedObjectArray<btScalar> > tDenseMatrix;

  btVector3 m_linearVelocity;
  //
  //  Fields
  //
  
  bool m_reducedModel;																	 // Reduced deformable model flag

  // reduced space
  int m_startMode;
  int m_nReduced;
  int m_nFull;
	tDenseMatrix m_modes;														// modes of the reduced deformable model. Each inner array is a mode, outer array size = n_modes
	tDenseArray m_reducedDofs;				   // Reduced degree of freedom
	tDenseArray m_reducedVelocity;		   // Reduced velocity array
  tDenseArray m_reducedForce;          // reduced force
	tDenseArray m_eigenvalues;		// eigenvalues of the reduce deformable model
	tDenseArray m_Kr;	// reduced stiffness matrix
	tDenseArray m_Mr;	// reduced mass matrix //TODO: do we need this?
  
  // full space
  TVStack m_x0;					     				 // Rest position
  tDenseArray m_nodalMass;           // Mass on each node
  btAlignedObjectArray<int> m_fixedNodes; // index of the fixed nodes

  //
  // Api
  //
  btReducedSoftBody(btSoftBodyWorldInfo* worldInfo, int node_count, const btVector3* x, const btScalar* m);

  ~btReducedSoftBody() {}

  void setReducedModes(int start_mode, int num_modes, int full_size);

  void setMassProps(const tDenseArray& mass_array);

  void setInertiaProps(const btVector3& inertia);

  void setRigidVelocity(const btVector3& v);

  void setRigidAngularVelocity(const btVector3& omega);

  void setStiffnessScale(const btScalar ks);

  void setMassScale(const btScalar rho);

  void setFixedNodes();

  virtual void translate(const btVector3& trs);

  void updateRestNodalPositions();

  void updateInertiaTensor();

  void predictIntegratedTransform(btScalar step, btTransform& predictedTransform);

  void endOfTimeStepZeroing();

  //
  // reduced dof related
  //

  // compute reduced degree of freedoms
  void updateReducedDofs(btScalar solverdt);

  // map to reduced degree of freedoms
  void mapToReducedDofs();

  // compute full degree of freedoms
  void updateFullDofs(btScalar solverdt);

  // map to full degree of freedoms
  void mapToFullDofs();

  // compute reduced velocity update
  void updateReducedVelocity(btScalar solverdt);

  // compute full space velocity from the reduced velocity
  void updateFullVelocity(btScalar solverdt);

  // update the full space mesh positions
  void updateMeshNodePositions(btScalar solverdt);

  //
  // rigid motion related
  //
  void applyCentralImpulse(const btVector3& impulse);

	void applyTorqueImpulse(const btVector3& torque);

  // apply impulse to the rigid frame
	void applyImpulse(const btVector3& impulse, const btVector3& rel_pos);

  // apply impulse to nodes in the full space
  void applyFullSpaceImpulse(const btVector3& target_vel, int n_node, btScalar dt);

  // apply fixed contraints to the nodes
  void applyFixedContraints(btScalar dt);

  // apply gravity to the rigid frame
  void applyRigidGravity(const btVector3& gravity, btScalar dt);

  // apply reduced force
  void applyReducedInternalForce(const btScalar damping_alpha, const btScalar damping_beta);

  void proceedToTransform(const btTransform& newTrans);

  void setCenterOfMassTransform(const btTransform& xform);

  btScalar getTotalMass() const
  {
    return m_mass;
  }

  btTransform& getWorldTransform()
	{
		return m_rigidTransformWorld;
	}

  const btVector3& getLinearVelocity() const
	{
		return m_linearVelocity;
	}
	const btVector3& getAngularVelocity() const
	{
		return m_angularVelocity;
	}

  const btVector3& getOrigin() const
  {
    return m_rigidTransformWorld.getOrigin();
  }

  #if defined(BT_CLAMP_VELOCITY_TO) && BT_CLAMP_VELOCITY_TO > 0
  void clampVelocity(btVector3& v) const {
      v.setX(
          fmax(-BT_CLAMP_VELOCITY_TO,
                fmin(BT_CLAMP_VELOCITY_TO, v.getX()))
      );
      v.setY(
          fmax(-BT_CLAMP_VELOCITY_TO,
                fmin(BT_CLAMP_VELOCITY_TO, v.getY()))
      );
      v.setZ(
          fmax(-BT_CLAMP_VELOCITY_TO,
                fmin(BT_CLAMP_VELOCITY_TO, v.getZ()))
      );
  }
  #endif
};

#endif // BT_REDUCED_SOFT_BODY_H