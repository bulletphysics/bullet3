	
	hasCL = findOpenCL_AMD()
	
	if (hasCL) then

		project "AppSoftBodyDemo_AMD"

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
			"OpenGLSupport",
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
			"../../SharedOpenCL",
			"../../../src",
			"../../../Glut",
			"../../OpenGL"
		}
		
		files {
			"../SoftDemo.cpp",
			"../SoftDemo.h",
			"../main.cpp",
			"../../SharedOpenCL/btOpenCLUtils.cpp",
			"../../SharedOpenCL/btOpenCLUtils.h",
			"../../SharedOpenCL/btOpenCLInclude.h"
		}
		
	end