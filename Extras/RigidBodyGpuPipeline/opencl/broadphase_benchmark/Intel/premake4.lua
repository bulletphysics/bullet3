	
	hasCL = findOpenCL_Intel()
	
	if (hasCL) then

		project "OpenCL_broadphase_benchmark_Intel"

		initOpenCL_Intel()
	
		language "C++"
				
		kind "ConsoleApp"
		targetdir "../../../bin"

		initOpenGL()
		initGlut()
		initGlew()

			includedirs {
		"../../../rendering/BulletMath",
		"../../primitives",
		"../../../../../src"
		}
		
		files {
			"../main.cpp",
			"../findPairsOpenCL.cpp",
			"../findPairsOpenCL.h",
			"../btGridBroadphaseCL.cpp",
			"../btGridBroadphaseCL.h",
			"../../3dGridBroadphase/Shared/bt3dGridBroadphaseOCL.cpp",
			"../../3dGridBroadphase/Shared/bt3dGridBroadphaseOCL.h",
			"../../3dGridBroadphase/Shared/btGpu3DGridBroadphase.cpp",
			"../../3dGridBroadphase/Shared/btGpu3DGridBroadphase.h",
			"../../../../../src/LinearMath/btAlignedAllocator.cpp",
			"../../../../../src/LinearMath/btQuickprof.cpp",
			"../../../../../src/LinearMath/btQuickprof.h",
			"../../../../../src/BulletCollision/BroadphaseCollision/btBroadphaseProxy.cpp",
			"../../../../../src/BulletCollision/BroadphaseCollision/btOverlappingPairCache.cpp",
			"../../../../../src/BulletCollision/BroadphaseCollision/btSimpleBroadphase.cpp",
			"../../basic_initialize/btOpenCLUtils.cpp",
			"../../basic_initialize/btOpenCLUtils.h",
			"../../opengl_interop/btOpenCLGLInteropBuffer.cpp",
			"../../opengl_interop/btOpenCLGLInteropBuffer.h",
			"../../opengl_interop/btStopwatch.cpp",
			"../../opengl_interop/btStopwatch.h"
		}
		
	end