
	project "Gwen_OpenGLTest"
		
	kind "ConsoleApp"
	flags {"Unicode"}
	
	defines { "GWEN_COMPILE_STATIC" , "_HAS_EXCEPTIONS=0", "_STATIC_CPPLIB" }
	
	targetdir "../../bin"
	
	includedirs 
	{
	
		"..",
		".",
		"../../src"
	}

	initOpenGL()
	initGlew()
			
	links {
		"gwen",
	}
	
	
	files {
		"../FontFiles/OpenSans.cpp",
		"../OpenGLWindow/TwFonts.cpp",
		"../OpenGLWindow/TwFonts.h",
		"../OpenGLWindow/LoadShader.cpp",
		"../OpenGLWindow/LoadShader.h",
		"../OpenGLWindow/GLPrimitiveRenderer.cpp",
		"../OpenGLWindow/GLPrimitiveRenderer.h",				
		"../OpenGLWindow/GwenOpenGL3CoreRenderer.h",
		"../OpenGLTrueTypeFont/fontstash.cpp",
		"../OpenGLTrueTypeFont/fontstash.h",
		"../OpenGLTrueTypeFont/opengl_fontstashcallbacks.cpp",
 		"../OpenGLTrueTypeFont/opengl_fontstashcallbacks.h",
		"../../src/BulletGeometry/btConvexHullComputer.cpp",
		"../../src/BulletGeometry/btConvexHullComputer.h",
		"../../src/BulletCommon/btAlignedAllocator.cpp",
		"../../src/BulletCommon/btQuickprof.cpp",
		"../../src/BulletCommon/btQuickprof.h",
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
		files{
		"../OpenGLWindow/X11OpenGLWindow.h",
		"../OpenGLWindow/X11OpenGLWindow.cpp"
		}
	end
