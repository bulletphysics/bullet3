function createProject(vendor)
	
	hasCL = findOpenCL(vendor)
	
	if (hasCL) then

		project ("App_Bullet3_OpenCL_Demos_" .. vendor)

		initOpenCL(vendor)
		
		language "C++"
				
		kind "ConsoleApp"
		targetdir "../../bin"


		initOpenGL()
		initGlew()

		includedirs {
		 	"..",
		 	"../../src",
		 	"../../btgui"
		}
		
		links {
			"gwen",
			"Bullet3Common",
			"Bullet3Geometry",
			"Bullet3Collision",
			"Bullet3Dynamics",
			"Bullet2FileLoader",
			"Bullet3OpenCL_" .. vendor
			
		}
		
		files {
			"**.cpp",
			"**.h",
			
			"../Wavefront/string_extra.cpp",
			"../Wavefront/string_extra.h",
			"../Wavefront/objLoader.cpp",
			"../Wavefront/objLoader.h",
			"../Wavefront/obj_parser.cpp",
			"../Wavefront/obj_parser.h",
			"../Wavefront/list.cpp",
			"../Wavefront/list.h",
			
			
			"../../btgui/OpenGLWindow/GLInstancingRenderer.cpp",
			"../../btgui/OpenGLWindow/GLInstancingRenderer.h",
			"../../btgui/OpenGLWindow/GLPrimitiveRenderer.cpp",
			"../../btgui/OpenGLWindow/GLPrimitiveRenderer.h",
			"../../btgui/OpenGLWindow/LoadShader.cpp",
			"../../btgui/OpenGLWindow/LoadShader.h",
			"../../btgui/OpenGLWindow/TwFonts.cpp",
			"../../btgui/OpenGLWindow/TwFonts.h",
			"../../btgui/OpenGLTrueTypeFont/fontstash.cpp",
			"../../btgui/OpenGLTrueTypeFont/fontstash.h",
			"../../btgui/OpenGLTrueTypeFont/opengl_fontstashcallbacks.cpp",
			"../../btgui/OpenGLTrueTypeFont/opengl_fontstashcallbacks.h",
			"../../btgui/FontFiles/OpenSans.cpp",
			"../../btgui/stb_image/stb_image.cpp",
			"../../btgui/stb_image/stb_image.h",
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
		if os.is("MacOSX") then
			links {"Cocoa.framework"}
			files {
				"../../btgui/OpenGLWindow/MacOpenGLWindow.h",
                        	"../../btgui/OpenGLWindow/MacOpenGLWindow.mm",	
			}
		end
	end
end

createProject("Apple")
createProject("AMD")
createProject("Intel")
createProject("NVIDIA")
