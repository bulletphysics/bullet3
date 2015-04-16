	
		project "OpenGL_Window"
	
		language "C++"
				
		kind "StaticLib"

		initOpenGL()
		initGlew()

		includedirs {
		 	
		 	"../../src",
		}
		
		--links {
		--}
		
		files {
			"*.cpp",
			"*.h",
			"OpenGLWindow/*.c",
			"OpenGLWindow/*.h",
			"OpenGLWindow/GL/*.h"
		}

		if not os.is("Windows") then 
			excludes {  
				"Win32OpenGLWindow.cpp",
      	"Win32OpenGLWindow.h",
      	"Win32Window.cpp",
      	"Win32Window.h",
			}
		end
		if os.is("Linux") then
			initX11()
		end
		if not os.is("Linux") then
			excludes {
				"X11OpenGLWindow.cpp",
				"X11OpenGLWindows.h"
			}
		end
		if os.is("MacOSX") then
			files
			{
					"../OpenGLWindow/MacOpenGLWindow.h",
					"../OpenGLWindow/MacOpenGLWindow.mm",
			} 
		end
