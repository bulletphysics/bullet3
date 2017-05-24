	
		project "App_SimpleOpenGL3"

		language "C++"
				
		kind "ConsoleApp"

  	includedirs {
                ".",
                "../../src",
                ".."
                }

			
		links{ "OpenGL_Window","Bullet3Common"}
		initOpenGL()	
		initGlew()
	
		files {
		"main.cpp",
		"*.h",
		}
		
if os.is("Linux") then initX11() end

if os.is("MacOSX") then
	links{"Cocoa.framework"}
end
