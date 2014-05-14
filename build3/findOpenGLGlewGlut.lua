	function findOpenGL()
		configuration{}
		if os.is("Linux") then
			if os.isdir("/usr/include") and os.isfile("/usr/include/GL/gl.h") then return true
			end
			return false
		end
		--assume OpenGL is available on Mac OSX, Windows etc
		return true
	end

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
		if os.is("Windows") then
			configuration {"Windows"}
			includedirs {
				projectRootDir .. "btgui/OpenGLWindow/Glut"
			}
			libdirs { projectRootDir .. "btgui/OpenGLWindow/Glut"}
			configuration {"Windows", "x32"}
				links {"glut32"}
			configuration {"Windows", "x64"}
				links {"glut64"}
		end
		
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
					projectRootDir .. "btgui/OpenGLWindow/GlewWindows"
			}
			files { projectRootDir .. "btgui/OpenGLWindow/GlewWindows/glew.c"}
		end
		if os.is("Linux") then
			links{"GLEW"}
		end
		configuration{}
	end



