
	function initOpenGL()
		configuration {}
		configuration {"Windows"}
			links {"opengl32","glu32"}
		configuration {"MacOSX"}
 			links { "OpenGL.framework"} 
		configuration {"not Windows", "not MacOSX"}
			links {"GL"}
		configuration{}
	end

	function initGlut()
		configuration {}
		configuration {"Windows"}

			includedirs {
				projectRootDir .. "rendering/GlutGlewWindows"
			}
			libdirs { projectRootDir .. "rendering/GlutGlewWindows"}
		configuration {"Windows", "x32"}
			links {"glut32"}
		configuration {"Windows", "x64"}
			links {"glut64"}
	
		configuration {"MacOSX"}
 			links { "Glut.framework" } 
		configuration {"Linux"}
			links {"glut","GLU"}
		configuration{}
	end

	function initGlew()
		configuration {}
		if os.is("Windows") then
			configuration {"Windows"}
			defines { "GLEW_STATIC"}
			includedirs {
					projectRootDir .. "rendering/GlutGlewWindows"
			}
			libdirs {	projectRootDir .. "rendering/GlutGlewWindows"}
			files { projectRootDir .. "rendering/GlutGlewWindows/glew.c"}
		end
		if os.is("Linux") then
			links{"GLEW"}
		end
		configuration{}
	end



