	
	hasCL = findOpenCL_NVIDIA()
	
	if (hasCL) then

		project "OpenCL_integration_NVIDIA"

		initOpenCL_NVIDIA()
	
		language "C++"
				
		kind "ConsoleApp"
		targetdir "../../../bin"

		initOpenGL()
		initGlut()
		initGlew()


			includedirs {
		"../../../rendering/BulletMath",
		"../../primitives"
		}
		
		files {
			"../main.cpp",
			"../../basic_initialize/btOpenCLUtils.cpp",
			"../../basic_initialize/btOpenCLUtils.h",
			"../../opengl_interop/btOpenCLGLInteropBuffer.cpp",
			"../../opengl_interop/btOpenCLGLInteropBuffer.h",
			"../../opengl_interop/btStopwatch.cpp",
			"../../opengl_interop/btStopwatch.h"
		}
		
	end