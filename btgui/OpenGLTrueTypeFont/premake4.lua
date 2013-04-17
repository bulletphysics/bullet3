	

		project "OpenGL_TrueTypeFont"

		language "C++"
				
		kind "ConsoleApp"
		targetdir "../../bin"


		initOpenGL()
		initGlew()

		includedirs {
		"../../src",
		".."
		}
		
			
		
		files {
			"main.cpp",
			"../FontFiles/OpenSans.cpp",
			"../OpenGLWindow/LoadShader.cpp",
			"../OpenGLWindow/LoadShader.h",
			"../../src/BulletCommon/b3AlignedAllocator.cpp",
			"../../src/BulletCommon/b3Quickprof.cpp",
			"../../src/BulletCommon/b3Quickprof.h" ,
			"fontstash.cpp",
      "fontstash.h",
      "opengl_fontstashcallbacks.cpp",
      "opengl_fontstashcallbacks.h",
      "stb_image_write.h",
      "stb_truetype.h",
			}

		if os.is("Windows") then
			files{
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
			
		if os.is("MacOSX") then
				links { "Cocoa.framework" }
				files{
					"../OpenGLWindow/MacOpenGLWindow.h",
					"../OpenGLWindow/MacOpenGLWindow.mm",
				}
		
		end
		
