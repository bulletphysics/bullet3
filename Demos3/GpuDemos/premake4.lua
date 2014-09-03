function createProject(vendor)
	
	hasCL = findOpenCL(vendor)
	
	if (hasCL) then

		project ("App_Bullet3_OpenCL_Demos_" .. vendor)

		initOpenCL(vendor)
		
		language "C++"
				
		kind "ConsoleApp"
		targetdir "../../bin"


		initOpenGL()
		initGlew()

		includedirs {
		 	"..",
		 	"../../src",
		 	"../../btgui"
		}
		
		links {
			"gwen",
			"Bullet2FileLoader",
			 "Bullet3OpenCL_" .. vendor,
			"Bullet3Dynamics",
			"Bullet3Collision",
			"Bullet3Geometry",
			"Bullet3Common",
	
		}
		
		files {
			"**.cpp",
			"**.h",
			
			"../Wavefront/tiny_obj_loader.cpp",
			"../Wavefront/tiny_obj_loader.h",
			
			
			"../../btgui/OpenGLWindow/GLInstancingRenderer.cpp",
			"../../btgui/OpenGLWindow/GLInstancingRenderer.h",
			"../../btgui/OpenGLWindow/GLPrimitiveRenderer.cpp",
			"../../btgui/OpenGLWindow/GLPrimitiveRenderer.h",
			"../../btgui/OpenGLWindow/LoadShader.cpp",
			"../../btgui/OpenGLWindow/LoadShader.h",
			"../../btgui/OpenGLWindow/TwFonts.cpp",
			"../../btgui/OpenGLWindow/TwFonts.h",
			"../../btgui/OpenGLWindow/GLRenderToTexture.cpp",
			"../../btgui/OpenGLWindow/GLRenderToTexture.h",
			"../../btgui/OpenGLWindow/fontstash.cpp",
			"../../btgui/OpenGLWindow/fontstash.h",
			"../../btgui/OpenGLWindow/opengl_fontstashcallbacks.cpp",
			"../../btgui/OpenGLWindow/opengl_fontstashcallbacks.h",
			"../../btgui/OpenGLWindow/OpenSans.cpp",
			"../../btgui/stb_image/stb_image.cpp",
			"../../btgui/stb_image/stb_image.h",
			"../../btgui/Timing/b3Quickprof.cpp",
			"../../btgui/Timing/b3Quickprof.h",
			"../../btgui/Timing/b3Clock.cpp",
			"../../btgui/Timing/b3Clock.h",
		}

	if _OPTIONS["midi"] then
		if os.is("Windows") then
			files {"../../btgui/MidiTest/RtMidi.cpp"}
			links {"winmm"}
			defines {"__WINDOWS_MM__", "WIN32","B3_USE_MIDI"}
		end
	
		if os.is("Linux") then 
		end
	
		if os.is("MacOSX") then
			files {"../../btgui/MidiTest/RtMidi.cpp"}
			links{"CoreAudio.framework", "coreMIDI.framework", "Cocoa.framework"}
			defines {"__MACOSX_CORE__","B3_USE_MIDI"}
		end
	end
	
		if os.is("Windows") then 
			files{  
				"../../btgui/OpenGLWindow/Win32OpenGLWindow.cpp",
      	"../../btgui/OpenGLWindow/Win32OpenGLWindow.h",
      	"../../btgui/OpenGLWindow/Win32Window.cpp",
      	"../../btgui/OpenGLWindow/Win32Window.h",
			}
		end
		if os.is("Linux") then
			initX11()	
			files {
				"../../btgui/OpenGLWindow/X11OpenGLWindow.cpp",
				"../../btgui/OpenGLWindow/X11OpenGLWindows.h"
			}
		end
		if os.is("MacOSX") then
			links {"Cocoa.framework"}
			files {
				"../../btgui/OpenGLWindow/MacOpenGLWindow.h",
                        	"../../btgui/OpenGLWindow/MacOpenGLWindow.mm",	
			}
		end
	end
end

createProject("clew")
createProject("Apple")
createProject("AMD")
createProject("Intel")
createProject("NVIDIA")
