	
		project "App_TinyRenderer"

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
		"*.cpp",
		"*.h",
		"../Utils/b3ResourcePath.cpp"
		}
		
if os.is("Linux") then initX11() end

if os.is("MacOSX") then
	links{"Cocoa.framework"}
end
