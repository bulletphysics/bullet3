
IF (USE_GLUT)

IF(BUILD_CPU_DEMOS)
	SET(SharedDemoSubdirs
		OpenGL AllBulletDemos  ConvexDecompositionDemo Benchmarks HelloWorld
		CcdPhysicsDemo ConstraintDemo SliderConstraintDemo GenericJointDemo Raytracer
		RagdollDemo ForkLiftDemo BasicDemo Box2dDemo BspDemo MovingConcaveDemo VehicleDemo
		UserCollisionAlgorithm CharacterDemo SoftDemo HeightFieldFluidDemo
		CollisionInterfaceDemo ConcaveConvexcastDemo SimplexDemo DynamicControlDemo
		DoublePrecisionDemo ConcaveDemo CollisionDemo
		ContinuousConvexCollision ConcaveRaycastDemo GjkConvexCastDemo
		MultiMaterialDemo SerializeDemo InternalEdgeDemo
	)
ELSE()
	SET(SharedDemoSubdirs
		OpenGL
	)
ENDIF()

if (CMAKE_SIZEOF_VOID_P MATCHES "8")
	SUBDIRS( 	${SharedDemoSubdirs}
	)
else (CMAKE_SIZEOF_VOID_P MATCHES "8")
	SUBDIRS( 		${SharedDemoSubdirs}
		ThreadingDemo
		MultiThreadedDemo
		VectorAdd_OpenCL
		ParticlesOpenCL
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
	VectorAdd_OpenCL
)

ENDIF (USE_GLUT)