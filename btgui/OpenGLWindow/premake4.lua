	

		project "OpenGL_rendertest"

	
		language "C++"
				
		kind "ConsoleApp"
		targetdir "../../bin"


		initOpenGL()
		initGlew()

		includedirs {
		 	"..",
		 	"../../src",
		}
		
		links {
			"gwen"
		}
		
		files {
			"main.cpp",
			"renderscene.cpp",
			"renderscene.h",
			"GLInstancingRenderer.cpp",
			"GLInstancingRenderer.h",
			"GLPrimitiveRenderer.h",
			"GLPrimitiveRenderer.cpp",
			"LoadShader.cpp",
			"LoadShader.h",
			"gwenWindow.cpp",
			"gwenWindow.h",
			"TwFonts.cpp",
			"TwFonts.h",
      "GwenOpenGL3CoreRenderer.h",
      "../FontFiles/OpenSans.cpp",
			"../OpenGLTrueTypeFont/fontstash.cpp",
			"../OpenGLTrueTypeFont/fontstash.h",
			"../OpenGLTrueTypeFont/opengl_fontstashcallbacks.cpp",
 			"../OpenGLTrueTypeFont/opengl_fontstashcallbacks.h",
			"../../src/Bullet3Geometry/b3ConvexHullComputer.cpp",
			"../../src/Bullet3Geometry/b3ConvexHullComputer.h",
			"../../src/Bullet3Common/b3AlignedAllocator.cpp",
			"../Timing/b3Quickprof.cpp",
			"../Timing/b3Quickprof.h",
			"../Timing/b3Clock.cpp",
			"../Timing/b3Clock.h",
		}

		if os.is("Windows") then 
			files{  
				"Win32OpenGLWindow.cpp",
      	"Win32OpenGLWindow.h",
      	"Win32Window.cpp",
      	"Win32Window.h",
			}
		end
		if os.is("Linux") then
			files {
				"X11OpenGLWindow.cpp",
				"X11OpenGLWindows.h"
			}
		end
		if os.is("MacOSX") then
			links{"Cocoa.framework"}
			files
			{
                                "../OpenGLWindow/MacOpenGLWindow.h",
                                "../OpenGLWindow/MacOpenGLWindow.mm",
                	} 
		end

