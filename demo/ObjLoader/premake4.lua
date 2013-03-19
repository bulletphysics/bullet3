
		project "wavefrontObjLoader"
	
		language "C++"
				
		kind "ConsoleApp"
		targetdir "../../bin"

	 	includedirs {
                ".",
                }

		
		files {
			"objTester.cpp",
			"string_extra.cpp",
			"string_extra.h",
			"objLoader.cpp",
			"objLoader.h",
			"obj_parser.cpp",
			"obj_parser.h",
			"list.cpp",
			"list.h"
		}
