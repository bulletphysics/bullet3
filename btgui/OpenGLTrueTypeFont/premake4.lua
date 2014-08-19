	

		project "OpenGL_TrueTypeFont"

		language "C++"
				
		kind "StaticLib"
		

		initOpenGL()
		initGlew()

		includedirs {
		"../../src",
		".."
		}
		if os.is("Linux") then
			initX11()
		end
		
		files 
		{
			"../FontFiles/OpenSans.cpp",
			"fontstash.cpp",
      "fontstash.h",
      "opengl_fontstashcallbacks.cpp",
      "opengl_fontstashcallbacks.h",
      "stb_image_write.h",
      "stb_truetype.h",
		}

