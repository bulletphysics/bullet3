	project "clsocket"
		
	kind "StaticLib"
	
	if os.is("Windows") then 
		defines { "WIN32","_WINSOCK_DEPRECATED_NO_WARNINGS" }
		end
		if os.is("Linux") then
		 defines {"HAS_SOCKLEN_T","_LINUX"}
		end
		if os.is("MacOSX") then
		 defines {"HAS_SOCKLEN_T","_DARWIN"}
		end		
		
	
	includedirs {
		".","include","src"
	}
	files {
		"src/SimpleSocket.cpp",
		"src/ActiveSocket.cpp",
		"src/PassiveSocket.cpp",
		"**.h"
	}
