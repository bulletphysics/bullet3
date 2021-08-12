#include "../btDeformableContactConstraint.h"
#include "btReducedSoftBody.h"

// ================= static constraints ===================
class btReducedDeformableStaticConstraint : public btDeformableStaticConstraint
{
 public:
  btReducedSoftBody* m_rsb;
  btScalar m_dt;

  btReducedDeformableStaticConstraint(btReducedSoftBody* rsb, 
                                      btSoftBody::Node* node,
                                      const btContactSolverInfo& infoGlobal,
                                      btScalar dt);
	// btReducedDeformableStaticConstraint(const btReducedDeformableStaticConstraint& other);
  btReducedDeformableStaticConstraint() {}
  virtual ~btReducedDeformableStaticConstraint() {}

  // virtual btScalar solveConstraint(const btContactSolverInfo& infoGlobal);
  
  // // this calls reduced deformable body's applyFullSpaceImpulse
  // virtual void applyImpulse(const btVector3& impulse);

  // virtual void applySplitImpulse(const btVector3& impulse) {}
};

// ================= base contact constraints ===================
class btReducedDeformableRigidContactConstraint : public btDeformableRigidContactConstraint
{
 public:
  btReducedSoftBody* m_rsb;
  btScalar m_dt;

  btReducedDeformableRigidContactConstraint(btReducedSoftBody* rsb, 
                                            const btSoftBody::DeformableRigidContact& c, 
                                            const btContactSolverInfo& infoGlobal,
                                            btScalar dt);
	// btReducedDeformableRigidContactConstraint(const btReducedDeformableRigidContactConstraint& other);
  btReducedDeformableRigidContactConstraint() {}
  virtual ~btReducedDeformableRigidContactConstraint() {}
  
  virtual btScalar solveConstraint(const btContactSolverInfo& infoGlobal);

  virtual void applySplitImpulse(const btVector3& impulse) {} // TODO: may need later
};

// ================= node vs rigid constraints ===================
class btReducedDeformableNodeRigidContactConstraint : public btReducedDeformableRigidContactConstraint
{
 public:
  btSoftBody::Node* m_node;

  btReducedDeformableNodeRigidContactConstraint(btReducedSoftBody* rsb, 
                                                const btSoftBody::DeformableNodeRigidContact& contact, 
                                                const btContactSolverInfo& infoGlobal,
                                                btScalar dt);
	// btReducedDeformableNodeRigidContactConstraint(const btReducedDeformableNodeRigidContactConstraint& other);
  btReducedDeformableNodeRigidContactConstraint() {}
  virtual ~btReducedDeformableNodeRigidContactConstraint() {}

  // get the velocity of the deformable node in contact
	virtual btVector3 getVb() const;

	// get the split impulse velocity of the deformable face at the contact point
	virtual btVector3 getSplitVb() const;

	// get the velocity change of the input soft body node in the constraint
	virtual btVector3 getDv(const btSoftBody::Node*) const;

	// cast the contact to the desired type
	const btSoftBody::DeformableNodeRigidContact* getContact() const
	{
		return static_cast<const btSoftBody::DeformableNodeRigidContact*>(m_contact);
	}
  
  // this calls reduced deformable body's applyFullSpaceImpulse
  virtual void applyImpulse(const btVector3& impulse);
};

// ================= face vs rigid constraints ===================
class btReducedDeformableFaceRigidContactConstraint : public btReducedDeformableRigidContactConstraint
{
 public:
  btSoftBody::Face* m_face;
	bool m_useStrainLimiting;

  btReducedDeformableFaceRigidContactConstraint(btReducedSoftBody* rsb, 
                                                const btSoftBody::DeformableFaceRigidContact& contact, 
                                                const btContactSolverInfo& infoGlobal,
                                                btScalar dt, 
                                                bool useStrainLimiting);
	// btReducedDeformableFaceRigidContactConstraint(const btReducedDeformableFaceRigidContactConstraint& other);
  btReducedDeformableFaceRigidContactConstraint() {}
  virtual ~btReducedDeformableFaceRigidContactConstraint() {}

  // get the velocity of the deformable face at the contact point
	virtual btVector3 getVb() const;

	// get the split impulse velocity of the deformable face at the contact point
	virtual btVector3 getSplitVb() const;

	// get the velocity change of the input soft body node in the constraint
	virtual btVector3 getDv(const btSoftBody::Node*) const;

	// cast the contact to the desired type
	const btSoftBody::DeformableFaceRigidContact* getContact() const
	{
		return static_cast<const btSoftBody::DeformableFaceRigidContact*>(m_contact);
	}

  // this calls reduced deformable body's applyFullSpaceImpulse
  virtual void applyImpulse(const btVector3& impulse);
};