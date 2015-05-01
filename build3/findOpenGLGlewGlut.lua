
	function findOpenGL()
		configuration{}
		if os.is("Linux") then
			return true
		end
		--assume OpenGL is available on Mac OSX, Windows etc
		return true
	end

      function findOpenGL3()
                configuration{}
                if os.is("MacOSX") then
                        local osversion = os.getversion()
		--Mac OSX 10.9 and above supports OpenGL 3, below doesn't, so ...
                        if osversion.majorversion > 10 or (osversion.majorversion == 10 and osversion.minorversion >=9) then
                                return findOpenGL()
                        else
                                return false
                        end
                else
                        return findOpenGL()
                end
        end


	function initOpenGL()
		configuration {}
		configuration {"Windows"}
			links {"opengl32","glu32"}
		configuration {"MacOSX"}
 			links { "OpenGL.framework"} 
		configuration {"not Windows", "not MacOSX"}
		if os.is("Linux") then	
			if not  _OPTIONS["force_dlopen_opengl"] and (os.isdir("/usr/include") and os.isfile("/usr/include/GL/gl.h")) then
				links {"GL"}
			else
				print("No GL/gl.h found, using dynamic loading of GL using glew")
				defines {"GLEW_INIT_OPENGL11_FUNCTIONS=1"}
				links {"dl"}
			end
		end
		configuration{}
	end


	function initGlew()
		configuration {}
		if os.is("Windows") then
			configuration {"Windows"}
			defines { "GLEW_STATIC"}
			includedirs {
					projectRootDir .. "examples/ThirdPartyLibs/Glew"
			}
			files { projectRootDir .. "examples/ThirdPartyLibs/Glew/glew.c"}
		end
		if os.is("Linux") then
			configuration{"Linux"}
			if not _OPTIONS["force_dlopen_opengl"] and (os.isdir("/usr/include") and os.isfile("/usr/include/GL/gl.h") and os.isfile("/usr/include/GL/glew.h"))  then
				links {"GLEW"}
				print ("linking against system GLEW")
			else
				print("Using static glew and dynamic loading of glx functions")
			 	defines { "GLEW_STATIC","GLEW_DYNAMIC_LOAD_ALL_GLX_FUNCTIONS=1"}
                        	includedirs {
                                        projectRootDir .. "examples/ThirdPartyLibs/Glew"
                        	}
                        	files { projectRootDir .. "examples/ThirdPartyLibs/Glew/glew.c"}
				links {"dl"}
			end

		end
		configuration{}
	end

	function initX11()
		if os.is("Linux") then
			if not _OPTIONS["force_dlopen_x11"] and (os.isdir("/usr/include") and os.isfile("/usr/include/X11/X.h")) then
				links{"X11","pthread"}
			else
				print("No X11/X.h found, using dynamic loading of X11")
				includedirs {
                                        projectRootDir .. "examples/ThirdPartyLibs/optionalX11"
                                }
				defines {"DYNAMIC_LOAD_X11_FUNCTIONS"}	
				links {"dl","pthread"}
			end
		end
	end

