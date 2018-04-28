	
		project "OpenGL_Window"
	
		language "C++"
				
		kind "StaticLib"

		initOpenGL()
		initGlew()

		includedirs {
		 	"../ThirdPartyLibs",
		 	"../../src",
		}

        if os.is("Linux") then
            buildoptions{"-fPIC"}
        end
        		
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
		if not os.is("MacoSX") then
			excludes {
				"MacOpenGLWindow.cpp"
			}
		end
		if os.is("MacOSX") then
			files
			{
					"MacOpenGLWindow.h",
					"MacOpenGLWindow.cpp",
					"MacOpenGLWindowObjC.m",
					"MacOpenGLWindowObjC.h",
			} 
		end
