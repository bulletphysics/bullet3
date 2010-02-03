
IF (USE_GLUT)

if (CMAKE_SIZEOF_VOID_P MATCHES "8")
	SUBDIRS( OpenGL AllBulletDemos  ConvexDecompositionDemo Benchmarks HelloWorld
		CcdPhysicsDemo ConstraintDemo SliderConstraintDemo GenericJointDemo Raytracer
		RagdollDemo ForkLiftDemo BasicDemo Box2dDemo BspDemo MovingConcaveDemo VehicleDemo
		UserCollisionAlgorithm CharacterDemo SoftDemo HeightFieldFluidDemo
		CollisionInterfaceDemo ConcaveConvexcastDemo SimplexDemo DynamicControlDemo
		DoublePrecisionDemo ConcaveDemo CollisionDemo
		ContinuousConvexCollision ConcaveRaycastDemo GjkConvexCastDemo
		MultiMaterialDemo
	)
else (CMAKE_SIZEOF_VOID_P MATCHES "8")
	SUBDIRS( OpenGL AllBulletDemos ConvexDecompositionDemo Benchmarks HelloWorld
		MultiThreadedDemo CcdPhysicsDemo ConstraintDemo SliderConstraintDemo Raytracer
		GenericJointDemo RagdollDemo ForkLiftDemo BasicDemo Box2dDemo BspDemo MovingConcaveDemo
		VehicleDemo UserCollisionAlgorithm CharacterDemo SoftDemo
	 	HeightFieldFluidDemo
		CollisionInterfaceDemo ConcaveConvexcastDemo SimplexDemo DynamicControlDemo
		DoublePrecisionDemo ConcaveDemo CollisionDemo
		ContinuousConvexCollision ConcaveRaycastDemo GjkConvexCastDemo
		MultiMaterialDemo
	)
endif (CMAKE_SIZEOF_VOID_P MATCHES "8")

ELSE (USE_GLUT)

SUBDIRS( 
	OpenGL 
	BasicDemo 
	Benchmarks 
	Box2dDemo 
	CollisionInterfaceDemo 
	ConcaveDemo 
	ConstraintDemo 
	ConvexDecompositionDemo 
	InternalEdgeDemo
	GimpactTestDemo 
	GenericJointDemo 
	SerializeDemo 
	SoftDemo
)

ENDIF (USE_GLUT)