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
  //

 protected:
  //


 public:
  //
  //  Typedefs
  //
  typedef btAlignedObjectArray<btVector3> TVStack;
  typedef btAlignedObjectArray<btScalar> tDenseArray;
  typedef btAlignedObjectArray<btAlignedObjectArray<btScalar> > tDenseMatrix;

  //
  //  Fields
  //
  
  bool m_reducedModel;																	 // Reduced deformable model flag

  // reduced space
	tDenseMatrix m_modes;														// modes of the reduced deformable model. Each inner array is a mode, outer array size = n_modes
	tDenseArray m_reducedDofs;				   // Reduced degree of freedom
	tDenseArray m_reducedVelocity;		   // Reduced velocity array
	tDenseArray m_eigenvalues;		// eigenvalues of the reduce deformable model
	tDenseArray m_Kr;	// reduced stiffness matrix
	tDenseArray m_Mr;	// reduced mass matrix //TODO: do we need this?

  // rigid frame
  btScalar m_mass;
  btScalar m_inverseMass;
  btVector3 m_linearVelocity;
	btVector3 m_angularVelocity;
	btVector3 m_linearFactor;
	btVector3 m_angularFactor;
	btVector3 m_invInertiaLocal;
  btMatrix3x3 m_invInertiaTensorWorld;
  
  // full space
  tDenseArray m_x0;									 // Rest position

  //
  // Api
  //
  btReducedSoftBody(btSoftBodyWorldInfo* worldInfo, int node_count, const btVector3* x, const btScalar* m);
  ~btReducedSoftBody() {}

  void setMass(btScalar m);

  void predictIntegratedTransform(btScalar step, btTransform& predictedTransform);

  // rigid motion related

  void applyCentralImpulse(const btVector3& impulse);

	void applyTorqueImpulse(const btVector3& torque);

	void applyImpulse(const btVector3& impulse, const btVector3& rel_pos);

  void proceedToTransform(const btTransform& newTrans);

  void setCenterOfMassTransform(const btTransform& xform);

  void updateInertiaTensor();

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
    return m_worldTransform.getOrigin();
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