	project "gwen"
		
	kind "StaticLib"
		
	flags {"Unicode"}
initOpenGL()
initGlew()
if os.is("Linux") then	
	initX11()
end 
	
	defines { "GWEN_COMPILE_STATIC" , "_HAS_EXCEPTIONS=0", "_STATIC_CPPLIB" }
	 defines { "DONT_USE_GLUT"}	
	targetdir "../../lib"	
	includedirs {
		".",".."
	}
	files {
		"**.cpp",
		"**.h"
	}
