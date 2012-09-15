
IF (USE_DX11)
	SUBDIRS(DX11ClothDemo)
ENDIF()

SUBDIRS( HelloWorld )


IF (USE_GLUT)
	IF (GLUT_FOUND)
		IF(BUILD_CPU_DEMOS)
			SET(SharedDemoSubdirs
			OpenGL AllBulletDemos  ConvexDecompositionDemo
			CcdPhysicsDemo ConstraintDemo SliderConstraintDemo GenericJointDemo Raytracer
			RagdollDemo ForkLiftDemo BasicDemo RollingFrictionDemo RaytestDemo VoronoiFractureDemo GyroscopicDemo FractureDemo Box2dDemo BspDemo MovingConcaveDemo VehicleDemo
			UserCollisionAlgorithm CharacterDemo SoftDemo 
			CollisionInterfaceDemo ConcaveConvexcastDemo SimplexDemo DynamicControlDemo
			ConvexHullDistance
			DoublePrecisionDemo ConcaveDemo CollisionDemo
			ContinuousConvexCollision ConcaveRaycastDemo GjkConvexCastDemo
			MultiMaterialDemo SerializeDemo InternalEdgeDemo 
			MultiThreadedDemo
			ParticlesOpenCL
			OpenCLClothDemo
			)
		ELSE()
			SET(SharedDemoSubdirs
			OpenGL
		)
		ENDIF()

		SUBDIRS( 		
			${SharedDemoSubdirs}
			Benchmarks
			ThreadingDemo
			VectorAdd_OpenCL
		)
	ENDIF(GLUT_FOUND)
ELSE (USE_GLUT)
	IF (WIN32)
		SUBDIRS( 
		OpenGL 
		BasicDemo
		RaytestDemo
		FractureDemo
		Benchmarks 
		Box2dDemo 
		CollisionInterfaceDemo 
		ConcaveDemo 
		ConstraintDemo
		RollingFrictionDemo
		ConvexDecompositionDemo 
		InternalEdgeDemo
		GimpactTestDemo
		GyroscopicDemo
		GenericJointDemo 
		SerializeDemo 
		SoftDemo
		VectorAdd_OpenCL
		VoronoiFractureDemo
	)
	ENDIF(WIN32)
ENDIF (USE_GLUT)

