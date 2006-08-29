INCLUDE_DIRECTORIES(
${BULLET_PHYSICS_SOURCE_DIR}/LinearMath ${BULLET_PHYSICS_SOURCE_DIR}/Bullet ${BULLET_PHYSICS_SOURCE_DIR}/BulletDynamics }
)
	
ADD_LIBRARY(LibBulletDynamics

	ConstraintSolver/ContactConstraint.cpp
	ConstraintSolver/Generic6DofConstraint.cpp
	ConstraintSolver/HingeConstraint.cpp
	ConstraintSolver/Point2PointConstraint.cpp
	ConstraintSolver/SequentialImpulseConstraintSolver.cpp
	ConstraintSolver/Solve2LinearConstraint.cpp
	ConstraintSolver/TypedConstraint.cpp
	Dynamics/BU_Joint.cpp
	Dynamics/ContactJoint.cpp
	Dynamics/RigidBody.cpp
	Vehicle/RaycastVehicle.cpp
	Vehicle/WheelInfo.cpp
)