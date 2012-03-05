	project "physicseffects2_lowlevel"
		
	kind "StaticLib"
	targetdir "../../build/lib"	
	includedirs {
		".",
	}
	files {
		"**.cpp",
		"../../include/physics_effects/low_level/**.h"

	}