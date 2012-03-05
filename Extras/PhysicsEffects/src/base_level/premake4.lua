	project "physicseffects2_baselevel"
		
	kind "StaticLib"
	targetdir "../../build/lib"	
	includedirs {
		".",
	}
	files {
		"**.cpp",
		"../../include/physics_effects/base_level/**.h"

	}