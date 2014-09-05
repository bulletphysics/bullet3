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
	includedirs {
		".",".."
	}
	files {
		"*.cpp",
		"*.h",
		"Controls/*.cpp",
		"Controls/*.h",
		"Controls/Dialog/*.cpp",
		"Controls/Dialogs/*.h",
		"Controls/Layout/*.h",
		"Controls/Property/*.h",
		"Input/*.h",
		"Platforms/*.cpp",
		"Renderers/*.cpp",
		"Skins/*.cpp",
		"Skins/*.h"
	}
