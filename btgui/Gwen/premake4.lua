	project "gwen"
		
	kind "StaticLib"
	
	flags {"Unicode"}
	
	 
	
	defines { "GWEN_COMPILE_STATIC" , "_HAS_EXCEPTIONS=0", "_STATIC_CPPLIB" }
		
	targetdir "../../lib"	
	includedirs {
		".",".."
	}
	files {
		"**.cpp",
		"**.h"
	}
