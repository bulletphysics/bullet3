function createProject(vendor)

		hasCL = findOpenCL(vendor)
	
	if (hasCL) then
		
		project ("OpenCL_GUI_Intialize_" .. vendor)

		initOpenCL(vendor)
	
		language "C++"
				
		kind "ConsoleApp"
		targetdir "../../bin"

 		includedirs {
                "..",
                "../../opencl",
                "../../btgui",
                "../../src",
                }

		links {
			"gwen"
		}
		
		initOpenGL()
		initGlew()
		
		files {
		"main.cpp",
			"../../src/Bullet3OpenCL/Initialize/b3OpenCLUtils.cpp",
			"../../src/Bullet3OpenCL/Initialize/b3OpenCLUtils.h",
			"../../btgui/OpenGLWindow/GLInstancingRenderer.cpp",
			"../../btgui/OpenGLWindow/GLInstancingRenderer.h",
			"../../btgui/OpenGLWindow/GLRenderToTexture.cpp",
			"../../btgui/OpenGLWindow/GLRenderToTexture.h",
			"../../btgui/OpenGLWindow/GLPrimitiveRenderer.h",
			"../../btgui/OpenGLWindow/GLPrimitiveRenderer.cpp",
			"../../btgui/OpenGLWindow/LoadShader.cpp",
			"../../btgui/OpenGLWindow/LoadShader.h",
			"../../btgui/OpenGLWindow/TwFonts.cpp",
			"../../btgui/OpenGLWindow/TwFonts.h",
      "../../btgui/OpenGLWindow/GwenOpenGL3CoreRenderer.h",
      "../../btgui/OpenGLWindow/OpenSans.cpp",
			"../../btgui/OpenGLWindow/fontstash.cpp",
			"../../btgui/OpenGLWindow/fontstash.h",
			"../../btgui/OpenGLWindow/opengl_fontstashcallbacks.cpp",
 			"../../btgui/OpenGLWindow/opengl_fontstashcallbacks.h",
			"../../src/Bullet3Geometry/b3ConvexHullComputer.cpp",
			"../../src/Bullet3Geometry/b3ConvexHullComputer.h",
			"../../src/Bullet3Common/b3AlignedAllocator.cpp",
			"../../src/Bullet3Common/b3Logging.cpp",
			"../../src/Bullet3Common/b3Logging.h",
			"../../btgui/Timing/b3Quickprof.cpp",
			"../../btgui/Timing/b3Quickprof.h",
			"../../btgui/Timing/b3Clock.cpp",
			"../../btgui/Timing/b3Clock.h",

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
			links {"X11","pthread"}
			files {
				"../../btgui/OpenGLWindow/X11OpenGLWindow.cpp",
				"../../btgui/OpenGLWindow/X11OpenGLWindows.h"
			}
		end
		if os.is("MacOSX") then
			links {"Cocoa.framework"}
			files {
				"../../btgui/OpenGLWindow/MacOpenGLWindow.h",
                        	"../../btgui/OpenGLWindow/MacOpenGLWindow.mm",	
			}
		end
		
	end
end

createProject("clew")
createProject("Apple")
createProject("AMD")
createProject("Intel")
createProject("NVIDIA")
