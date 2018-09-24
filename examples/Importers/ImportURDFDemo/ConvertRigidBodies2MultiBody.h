#ifndef CONVERT_RIGIDBODIES_2_MULTIBODY_H
#define CONVERT_RIGIDBODIES_2_MULTIBODY_H

struct ConvertRigidBodies2MultiBody
{
	btAlignedObjectArray<btRigidBody*> m_bodies;
	btAlignedObjectArray<btTypedConstraint*> m_constraints;

	virtual void addRigidBody(btRigidBody* body);
	virtual void addConstraint(btTypedConstraint* constraint, bool disableCollisionsBetweenLinkedBodies = false);
	virtual btMultiBody* convertToMultiBody();
};
#endif  //CONVERT_RIGIDBODIES_2_MULTIBODY_H
