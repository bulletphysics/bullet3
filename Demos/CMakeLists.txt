
IF (USE_DX11 AND BUILD_MULTITHREADING)
	SUBDIRS(DX11ClothDemo)
ENDIF()

SUBDIRS( HelloWorld )


IF (USE_GLUT)
	IF (GLUT_FOUND)
		IF(BUILD_CPU_DEMOS)
			SET(SharedDemoSubdirs
				OpenGL AllBulletDemos  ConvexDecompositionDemo
				CcdPhysicsDemo BulletXmlImportDemo ConstraintDemo SliderConstraintDemo GenericJointDemo Raytracer
				RagdollDemo ForkLiftDemo BasicDemo FeatherstoneMultiBodyDemo RollingFrictionDemo RaytestDemo VoronoiFractureDemo 
				GyroscopicDemo FractureDemo Box2dDemo BspDemo MovingConcaveDemo VehicleDemo
				UserCollisionAlgorithm CharacterDemo SoftDemo 
				CollisionInterfaceDemo ConcaveConvexcastDemo SimplexDemo DynamicControlDemo
				ConvexHullDistance
				DoublePrecisionDemo ConcaveDemo CollisionDemo
				ContinuousConvexCollision ConcaveRaycastDemo GjkConvexCastDemo
				MultiMaterialDemo SerializeDemo InternalEdgeDemo 
			)
		ELSE()
			SET(SharedDemoSubdirs			OpenGL		)
		ENDIF(BUILD_CPU_DEMOS)
		
		IF(BUILD_MULTITHREADING)
			SUBDIRS(	MultiThreadedDemo			OpenCLClothDemo			)
		ENDIF(BUILD_MULTITHREADING)

		SUBDIRS( 		
			${SharedDemoSubdirs}
			Benchmarks
		)
		
		IF(BUILD_MULTITHREADING)
			SUBDIRS(	ThreadingDemo		VectorAdd_OpenCL )
		ENDIF()

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
		FeatherstoneMultiBodyDemo
		GenericJointDemo 
		SerializeDemo 
		SoftDemo
		VectorAdd_OpenCL
		VoronoiFractureDemo
	)
	ENDIF(WIN32)
ENDIF (USE_GLUT)

