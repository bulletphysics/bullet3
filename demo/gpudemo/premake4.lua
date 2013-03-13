function createProject(vendor)
	
	hasCL = findOpenCL(vendor)
	
	if (hasCL) then

		project ("Bullet3_OpenCL_gpu_demo_" .. vendor)

		initOpenCL(vendor)
		
		language "C++"
				
		kind "ConsoleApp"
		targetdir "../../bin"


		initOpenGL()
		initGlew()

		includedirs {
		 	"..",
		 	"../../src",
		 	"../../btgui",
		 	"../../opencl"
		}
		
		links {
			"gwen"
		}
		
		files {
			"main_opengl3core.cpp",
			"gwenUserInterface.cpp",
			"gwenUserInterface.h",
			"ParticleDemo.cpp",
			"ParticleDemo.h",

		}

		if os.is("Windows") then 
			files{  
				"../../btgui/OpenGLWindow/Win32OpenGLWindow.cpp",
      	"../../btgui/OpenGLWindow/Win32OpenGLWindow.h",
      	"../../btgui/OpenGLWindow/Win32Window.cpp",
      	"../../btgui/OpenGLWindow/Win32Window.h",
			}
		end
		if os.is("Linux") then
			files {
				"../../btgui/OpenGLWindow/X11OpenGLWindow.cpp",
				"../../btgui/OpenGLWindow/X11OpenGLWindows.h"
			}
		end
	end
end

createProject("Apple")
createProject("AMD")
createProject("Intel")
createProject("NVIDIA")