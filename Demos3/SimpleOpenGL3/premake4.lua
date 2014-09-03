	
		project "App_SimpleOpenGL3"

		language "C++"
				
		kind "ConsoleApp"
		targetdir "../../bin"

  	includedirs {
                ".",
                "../../src",
                "../../btgui"
                }

			
		links{ "OpenGL_Window"}
		initOpenGL()	
		initGlew()
	
		files {
		"**.cpp",
		"**.h",
		"../../src/Bullet3Common/**.cpp",
 		"../../src/Bullet3Common/**.h",

		}
		
if os.is("Linux") then initX11() end

if os.is("MacOSX") then
	links{"Cocoa.framework"}
end
