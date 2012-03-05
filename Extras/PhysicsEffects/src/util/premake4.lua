	project "physicseffects2_util"
		
	kind "StaticLib"
	targetdir "../../build/lib"	
	includedirs {
		".",
	}
	files {
		"**.cpp",
		"../../include/physics_effects/util/**.h"

	}