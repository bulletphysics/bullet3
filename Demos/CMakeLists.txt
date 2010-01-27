SUBDIRS( OpenGL BasicDemo Benchmarks Box2dDemo ConcaveDemo ConstraintDemo ConvexDecompositionDemo SerializeDemo )


#todo: re-enable the rest of the demos again

#if (CMAKE_SIZEOF_VOID_P MATCHES "8")
#	SUBDIRS( OpenGL AllBulletDemos  ConvexDecompositionDemo Benchmarks HelloWorld
#	CcdPhysicsDemo ConstraintDemo SliderConstraintDemo GenericJointDemo Raytracer
#	RagdollDemo ForkLiftDemo BasicDemo Box2dDemo BspDemo MovingConcaveDemo VehicleDemo
#	ColladaDemo UserCollisionAlgorithm CharacterDemo SoftDemo HeightFieldFluidDemo
#	CollisionInterfaceDemo ConcaveConvexcastDemo SimplexDemo DynamicControlDemo
#	DoublePrecisionDemo ConcaveDemo CollisionDemo
#	ContinuousConvexCollision ConcaveRaycastDemo GjkConvexCastDemo
#	MultiMaterialDemo)
#else (CMAKE_SIZEOF_VOID_P MATCHES "8")
#	SUBDIRS( OpenGL AllBulletDemos ConvexDecompositionDemo Benchmarks HelloWorld
#	MultiThreadedDemo CcdPhysicsDemo ConstraintDemo SliderConstraintDemo Raytracer
#	GenericJointDemo RagdollDemo ForkLiftDemo BasicDemo Box2dDemo BspDemo MovingConcaveDemo
#	VehicleDemo ColladaDemo UserCollisionAlgorithm CharacterDemo SoftDemo
# 	HeightFieldFluidDemo
#	CollisionInterfaceDemo ConcaveConvexcastDemo SimplexDemo DynamicControlDemo
#	DoublePrecisionDemo ConcaveDemo CollisionDemo
#	ContinuousConvexCollision ConcaveRaycastDemo GjkConvexCastDemo
#	MultiMaterialDemo)
#endif (CMAKE_SIZEOF_VOID_P MATCHES "8")
