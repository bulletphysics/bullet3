	
	hasCL = findOpenCL_NVIDIA()
	
	if (hasCL) then

		project "AppOpenCLClothDemo_NVIDIA"

		defines { "USE_NVIDIA_OPENCL","CL_PLATFORM_NVIDIA"}

		initOpenCL_NVIDIA()
	
		language "C++"
				
		kind "ConsoleApp"
		targetdir "../../.."

		libdirs {"../../../Glut"}

		links {
			"LinearMath",
			"BulletCollision",
			"BulletDynamics", 
			"BulletSoftBody", 
			"BulletSoftBodySolvers_OpenCL_NVIDIA",
			"opengl32"
		}
		
			configuration { "Windows" }
			defines { "GLEW_STATIC"}
	
	
			configuration "x64"
			links {
				"glut64",
				"glew64s"
			}
			configuration "x32"
			links {
				"glut32",
				"glew32s"
			}
		
			configuration{}
			
		
			includedirs {
			"../../../src",
			"../../../Glut",
			"../../SharedOpenCL",
			"../../OpenGL"
		}
		
		files {
			"../cl_cloth_demo.cpp",
			"../../SharedOpenCL/btOpenCLUtils.cpp",
			"../../SharedOpenCL/btOpenCLUtils.h",
			"../../SharedOpenCL/btOpenCLInclude.h",
			"../../OpenGL/GLDebugDrawer.cpp",
			"../../OpenGL/stb_image.cpp",
			"../../OpenGL/stb_image.h",			
			"../gl_win.cpp",
			"../clstuff.cpp",
			"../clstuff.h",
			"../gl_win.h",
			"../cloth.h"
		}
		
	end