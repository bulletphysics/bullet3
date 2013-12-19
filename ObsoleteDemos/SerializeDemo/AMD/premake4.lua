	
	hasCL = findOpenCL_AMD()
	
	if (hasCL) then

		project "AppOpenCLClothDemo_AMD"

		defines { "USE_AMD_OPENCL","CL_PLATFORM_AMD"}

		initOpenCL_AMD()
	
		language "C++"
				
		kind "ConsoleApp"
		targetdir "../../.."

		libdirs {"../../../Glut"}

		links {
			"LinearMath",
			"BulletCollision",
			"BulletDynamics", 
			"BulletSoftBody", 
			"BulletSoftBodySolvers_OpenCL_AMD",
			"opengl32"
		}
		
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
			"../../SharedOpenCL/btOclUtils.h",
			"../../SharedOpenCL/btOclCommon.h",
			"../../SharedOpenCL/btOclUtils.cpp",
			"../../SharedOpenCL/btOclCommon.cpp",
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