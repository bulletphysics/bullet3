	
		project "Bullet3AppSupport"
	
		language "C++"
				
		kind "StaticLib"

		initOpenGL()
		initGlew()

		includedirs {
		 	"..",
		 	"../../src",
		}
		
		--links {
		--}
		
		files {
			"*.cpp",
			"*.h",
		}

		if os.is("Linux") then
			initX11()
		end
