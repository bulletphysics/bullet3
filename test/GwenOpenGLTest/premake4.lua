
	project "Test_Gwen_OpenGL"
		
	kind "ConsoleApp"
	flags {"Unicode"}
	
	defines { "GWEN_COMPILE_STATIC" , "_HAS_EXCEPTIONS=0", "_STATIC_CPPLIB" }
	defines { "DONT_USE_GLUT"}
	
	targetdir "../../bin"
	
	includedirs 
	{
	
		"../../examples/ThirdPartyLibs",
		"../../examples",	
		".",
	}

	initOpenGL()
	initGlew()
			
	links {
		"gwen",
	}
	
	
	files {
		"../../examples/OpenGLWindow/OpenSans.cpp",
		"../../examples/OpenGLWindow/TwFonts.cpp",
		"../../examples/OpenGLWindow/TwFonts.h",
		"../../examples/OpenGLWindow/LoadShader.cpp",
		"../../examples/OpenGLWindow/LoadShader.h",
		"../../examples/OpenGLWindow/GLPrimitiveRenderer.cpp",
		"../../examples/OpenGLWindow/GLPrimitiveRenderer.h",				
		"../../examples/OpenGLWindow/GwenOpenGL3CoreRenderer.h",
		"../../examples/OpenGLWindow/fontstash.cpp",
		"../../examples/OpenGLWindow/fontstash.h",
		"../../examples/OpenGLWindow/opengl_fontstashcallbacks.cpp",
 		"../../examples/OpenGLWindow/opengl_fontstashcallbacks.h",
		"../../examples/Utils/b3Clock.cpp",
		"../../examples/Utils/b3Clock.h",
		"**.cpp",
		"**.h",
	}
	if os.is("Windows") then
	files {
		"../../examples/OpenGLWindow/Win32OpenGLWindow.cpp",
                "../../examples/OpenGLWindow/Win32OpenGLWindow.h",
                "../../examples/OpenGLWindow/Win32Window.cpp",
                "../../examples/OpenGLWindow/Win32Window.h",
	}
	end
	if os.is("Linux") then 
		initX11()
		files{
		"../../examples/OpenGLWindow/X11OpenGLWindow.h",
		"../../examples/OpenGLWindow/X11OpenGLWindow.cpp"
		}
		links{"pthread"}
	end
	if os.is("MacOSX") then
		links{"Cocoa.framework"}
print("hello!")
		files{
		"../../examples/OpenGLWindow/MacOpenGLWindow.mm",
		"../../examples/OpenGLWindow/MacOpenGLWindow.h",
		}
	end
