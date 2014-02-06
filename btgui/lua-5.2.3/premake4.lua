	project "lua-5.2.3"
		
	kind "StaticLib"
	
	--flags {}
	
	 
	
	defines { "LUA_COMPAT_ALL"}
		
	targetdir "../../lib"	
	includedirs {
		"src"
	}
	files {
		"src/*.c",
		"src/*.h"
	}

include "lua_compiler"
include "lua_standalone"