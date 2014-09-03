
	project "Test_Gwen_OpenGL"
		
	kind "ConsoleApp"
	flags {"Unicode"}
	
	defines { "GWEN_COMPILE_STATIC" , "_HAS_EXCEPTIONS=0", "_STATIC_CPPLIB" }
	defines { "DONT_USE_GLUT"}
	
	targetdir "../../bin"
	
	includedirs 
	{
	
		"..",
		".",
	}

	initOpenGL()
	initGlew()
			
	links {
		"gwen",
	}
	
	
	files {
		"../OpenGLWindow/OpenSans.cpp",
		"../OpenGLWindow/TwFonts.cpp",
		"../OpenGLWindow/TwFonts.h",
		"../OpenGLWindow/LoadShader.cpp",
		"../OpenGLWindow/LoadShader.h",
		"../OpenGLWindow/GLPrimitiveRenderer.cpp",
		"../OpenGLWindow/GLPrimitiveRenderer.h",				
		"../OpenGLWindow/GwenOpenGL3CoreRenderer.h",
		"../OpenGLWindow/fontstash.cpp",
		"../OpenGLWindow/fontstash.h",
		"../OpenGLWindow/opengl_fontstashcallbacks.cpp",
 		"../OpenGLWindow/opengl_fontstashcallbacks.h",
		"../../btgui/Timing/b3Clock.cpp",
		"../../btgui/Timing/b3Clock.h",
		"**.cpp",
		"**.h",
	}
	if os.is("Windows") then
	files {
		"../OpenGLWindow/Win32OpenGLWindow.cpp",
                "../OpenGLWindow/Win32OpenGLWindow.h",
                "../OpenGLWindow/Win32Window.cpp",
                "../OpenGLWindow/Win32Window.h",
	}
	end
	if os.is("Linux") then 
		initX11()
		files{
		"../OpenGLWindow/X11OpenGLWindow.h",
		"../OpenGLWindow/X11OpenGLWindow.cpp"
		}
		links{"pthread"}
	end
	if os.is("MacOSX") then
		links{"Cocoa.framework"}
print("hello!")
		files{
		"../OpenGLWindow/MacOpenGLWindow.mm",
		"../OpenGLWindow/MacOpenGLWindow.h",
		}
	end
