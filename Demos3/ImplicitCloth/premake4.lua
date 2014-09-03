	
		project "App_ImplicitCloth"

		language "C++"
				
		kind "ConsoleApp"
		targetdir "../../bin"

  	includedirs {
                ".",
                "../../src",
                "../../btgui"
                }

		initOpenGL()
		initGlew()
			
		--links{"gwen"}

		files {
		"**.cpp",
		"**.h",
		"../../btgui/OpenGLWindow/SimpleOpenGL3App.cpp",
		"../../btgui/OpenGLWindow/SimpleOpenGL3App.h",
		"../../btgui/OpenGLWindow/TwFonts.cpp",
		"../../btgui/OpenGLWindow/TwFonts.h",
		"../../btgui/OpenGLWindow/LoadShader.cpp",
		"../../btgui/OpenGLWindow/LoadShader.h",
		"../../btgui/OpenGLWindow/GLPrimitiveRenderer.cpp",
		"../../btgui/OpenGLWindow/GLPrimitiveRenderer.h",				
		"../../btgui/OpenGLWindow/GwenOpenGL3CoreRenderer.h",
		"../../btgui/OpenGLWindow/GLInstancingRenderer.cpp",
		"../../btgui/OpenGLWindow/GLInstancingRenderer.h",
		"../../btgui/OpenGLWindow/GLRenderToTexture.cpp",
		"../../btgui/OpenGLWindow/GLRenderToTexture.h",
		"../../btgui/OpenGLWindow/TwFonts.cpp",
		"../../btgui/OpenGLWindow/TwFonts.h",
		"../../btgui/OpenGLWindow/OpenSans.cpp",
		"../../btgui/OpenGLWindow/fontstash.cpp",
		"../../btgui/OpenGLWindow/fontstash.h",
		"../../btgui/OpenGLWindow/opengl_fontstashcallbacks.cpp",
 		"../../btgui/OpenGLWindow/opengl_fontstashcallbacks.h",
 		"../../src/Bullet3Common/**.cpp",
 		"../../src/Bullet3Common/**.h",
		"../../btgui/Timing/b3Clock.cpp",
		"../../btgui/Timing/b3Clock.h"

		}

if os.is("Windows") then
	files {
		"../../btgui/OpenGLWindow/Win32OpenGLWindow.cpp",
		"../../btgui/OpenGLWindow/Win32OpenGLWindow.h",
    "../../btgui/OpenGLWindow/Win32Window.cpp",
    "../../btgui/OpenGLWindow/Win32Window.h",
	}
	end
	if os.is("Linux") then 
		links ("X11")
		files{
		"../../btgui/OpenGLWindow/X11OpenGLWindow.h",
		"../../btgui/OpenGLWindow/X11OpenGLWindow.cpp"
		}
		links{"pthread"}
	end
	if os.is("MacOSX") then
		links{"Cocoa.framework"}
		files{
		"../../btgui/OpenGLWindow/MacOpenGLWindow.mm",
		"../../btgui/OpenGLWindow/MacOpenGLWindow.h",
		}
	end
