
	project "Test_Gwen_OpenGL"
		
	kind "ConsoleApp"
	flags {"Unicode"}
	
	defines { "GWEN_COMPILE_STATIC" , "_HAS_EXCEPTIONS=0", "_STATIC_CPPLIB" }
	defines { "DONT_USE_GLUT"}
	
	
	
	includedirs 
	{
	
		"../../examples/ThirdPartyLibs",
		"../../examples",	
		".",
		"../../src"
	}

	initOpenGL()
	initGlew()
			
	links {
		"gwen","Bullet3Common"
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
		"../../examples/ThirdPartyLibs/stb_image/stb_image_write.cpp",
		"**.cpp",
		"**.h",
	}
	files {
		"../../examples/OpenGLWindow/GLFWOpenGLWindow.cpp",
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
		files{
		"../../examples/OpenGLWindow/MacOpenGLWindow.cpp",
		"../../examples/OpenGLWindow/MacOpenGLWindow.h",
		"../../examples/OpenGLWindow/MacOpenGLWindowObjC.m",
		"../../examples/OpenGLWindow/MacOpenGLWindowObjC.h",
		}
	end
