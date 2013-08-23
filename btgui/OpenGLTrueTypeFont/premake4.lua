	

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
			"../../src/Bullet3Common/b3AlignedAllocator.cpp",
			"../../src/Bullet3Common/b3Logging.cpp",
			"../Timing/b3Quickprof.cpp",
			"../Timing/b3Quickprof.h" ,
			"../Timing/b3Clock.cpp",
			"../Timing/b3Clock.h" ,
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
			links{"X11"}
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
		
