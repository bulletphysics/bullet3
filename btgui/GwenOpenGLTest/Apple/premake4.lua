	if os.is("macosx") then

	project "Gwen_OpenGLTest_Apple"
		
	kind "ConsoleApp"
	flags {"Unicode"}
	
	defines { "GWEN_COMPILE_STATIC" , "_HAS_EXCEPTIONS=0", "_STATIC_CPPLIB" }
	
	targetdir "../../../../bin"
	
	includedirs 
	{
	
		"../..","..","../../../rendertest","../../../../bullet2"
	}

	initOpenGL()
	initGlew()
			
	links {
		"gwen","Cocoa.framework"
	}
	
	
	files {
		"../../../rendertest/MacOpenGLWindow.mm",
		"../../../rendertest/MacOpenGLWindow.h",
		"../../../rendertest/TwFonts.cpp",
		"../../../rendertest/TwFonts.h",
		"../../../rendertest/LoadShader.cpp",
		"../../../rendertest/LoadShader.h",
		"../../../rendertest/GLPrimitiveRenderer.cpp",
		"../../../rendertest/GLPrimitiveRenderer.h",				
		"../../../rendertest/GwenOpenGL3CoreRenderer.h",
		"../../../OpenGLTrueTypeFont/fontstash.cpp",
		"../../../OpenGLTrueTypeFont/fontstash.h",
		"../../../OpenGLTrueTypeFont/opengl_fontstashcallbacks.cpp",
 		"../../../OpenGLTrueTypeFont/opengl_fontstashcallbacks.h",
		"../../../../bullet2/LinearMath/b3ConvexHullComputer.cpp",
		"../../../../bullet2/LinearMath/b3ConvexHullComputer.h",
		"../../../../bullet2/LinearMath/btSerializer.cpp",
		"../../../../bullet2/LinearMath/btSerializer.h",
		"../../../../bullet2/LinearMath/b3AlignedAllocator.cpp",
		"../../../../bullet2/LinearMath/b3Quickprof.cpp",
		"../../../../bullet2/LinearMath/b3Quickprof.h",
		"../**.cpp",
		"../**.h",
	}
	end

