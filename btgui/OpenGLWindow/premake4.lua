	

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
			"../../src/BulletGeometry/btConvexHullComputer.cpp",
			"../../src/BulletGeometry/btConvexHullComputer.h",
			"../../src/BulletCommon/btAlignedAllocator.cpp",
			"../../src/BulletCommon/btQuickprof.cpp",
			"../../src/BulletCommon/btQuickprof.h"
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
