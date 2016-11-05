	project "enet"
		
	kind "StaticLib"
	
	if os.is("Windows") then 
		defines { "WIN32" }
			files{"win32.c"}
		end
		if os.is("Linux") then
		 defines {"HAS_SOCKLEN_T"}
			files {"unix.c",}
		end
		if os.is("MacOSX") then
			files{"unix.c"}
		end		
		
	
	includedirs {
		".","include"
	}
	files {
		"callbacks.c",
		"compress.c",
		"host.c",
		"list.c",
		"packet.c",
		"peer.c",
		"protocol.c",
		"**.h"
	}
